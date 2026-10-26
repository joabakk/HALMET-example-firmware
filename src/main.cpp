// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <NMEA2000_esp32.h>

#include "n2k_senders.h"
#include "sensesp/net/discovery.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/system_status_led.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/time_counter.h"
#include "sensesp/ui/config_item.h"
#include "sensesp_app_builder.h"
#define BUILDER_CLASS SensESPAppBuilder

#include "halmet_analog.h"
#include "halmet_const.h"
#include "halmet_digital.h"
#include "halmet_display.h"
#include "halmet_serial.h"
#include "sensesp/net/http_server.h"
#include "sensesp/net/networking.h"

using namespace sensesp;
using namespace halmet;

/////////////////////////////////////////////////////////////////////
// Declare some global variables required for the firmware operation.

tNMEA2000* nmea2000;
elapsedMillis n2k_time_since_rx = 0;
elapsedMillis n2k_time_since_tx = 0;

TwoWire* i2c;
Adafruit_SSD1306* display;

// Store alarm states in an array for local display output
bool alarm_states[4] = {false, false, false, false};

// Set the ADS1115 GAIN to adjust the analog input voltage range.
// On HALMET, this refers to the voltage range of the ADS1115 input
// AFTER the 33.3/3.3 voltage divider.

// GAIN_TWOTHIRDS: 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
// GAIN_ONE:       1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
// GAIN_TWO:       2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
// GAIN_FOUR:      4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
// GAIN_EIGHT:     8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
// GAIN_SIXTEEN:   16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

const adsGain_t kADS1115Gain = GAIN_ONE;

/////////////////////////////////////////////////////////////////////
// Test output pin configuration. If ENABLE_TEST_OUTPUT_PIN is defined,
// GPIO 33 will output a pulse wave at 380 Hz with a 50% duty cycle.
// If this output and GND are connected to one of the digital inputs, it can
// be used to test that the frequency counter functionality is working.
#define ENABLE_TEST_OUTPUT_PIN
#ifdef ENABLE_TEST_OUTPUT_PIN
const int kTestOutputPin = GPIO_NUM_33;
// With the default pulse rate of 100 pulses per revolution (configured in
// halmet_digital.cpp), this frequency corresponds to 3.8 r/s or about 228 rpm.
const int kTestOutputFrequency = 380;
#endif

// Sender voltages below this mean the gauges have no supply (ignition off) or
// the sender is disconnected. The sender voltage and the values calculated from
// it are then sent as null, so Signal K and NMEA 2000 show no value rather than
// a wrong one.
const float kMinSenderVoltage = 0.2;

// Pass a calibrated value through, or null if its sender voltage is too low.
LambdaTransform<float, NullableFloat>* NullWhenUnpowered(
    FloatProducer* sender_voltage) {
  return new LambdaTransform<float, NullableFloat>(
      [sender_voltage](float value) {
        if (sender_voltage->get() < kMinSenderVoltage) {
          return NullableFloat(NullableFloat::invalid());
        }
        return NullableFloat(value);
      });
}

// Limit a curve's output to the range of its samples, so that readings outside
// the calibrated range (e.g. a disconnected sender) don't extrapolate to
// nonsense values.
LambdaTransform<float, float>* ClampToCurve(CurveInterpolator* curve) {
  return new LambdaTransform<float, float>([curve](float value) {
    const auto& samples = curve->get_samples();
    if (samples.empty()) {
      return value;
    }
    float lo = samples.begin()->output_;
    float hi = lo;
    for (const auto& sample : samples) {
      lo = std::min(lo, sample.output_);
      hi = std::max(hi, sample.output_);
    }
    return std::max(lo, std::min(hi, value));
  });
}

/////////////////////////////////////////////////////////////////////
// The setup function performs one-time application initialization.
void setup() {
  SetupLogging(ESP_LOG_DEBUG);

  // These calls can be used for fine-grained control over the logging level.
  // esp_log_level_set("*", esp_log_level_t::ESP_LOG_DEBUG);

  Serial.begin(115200);

  /////////////////////////////////////////////////////////////////////
  // Initialize the application framework

  // Construct the global SensESPApp() object
  BUILDER_CLASS builder;

  // OTA updates are enabled when ota_password is set in secrets.ini.
  if (strlen(OTA_PASSWORD) > 0) {
    builder.enable_ota(OTA_PASSWORD);
  }

  sensesp_app = (&builder)
                    // EDIT: Set a custom hostname for the app.
                    ->set_hostname("halmet")
                    // EDIT: Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi("My WiFi SSID", "my_wifi_password")
                    //->set_sk_server("192.168.10.3", 80)
                    ->get_app();

  // initialize the I2C bus
  i2c = new TwoWire(0);
  i2c->begin(kSDAPin, kSCLPin);

  // Initialize ADS1115
  auto ads1115 = new Adafruit_ADS1115();

  ads1115->setGain(kADS1115Gain);
  bool ads_initialized = ads1115->begin(kADS1115Address, i2c);
  debugD("ADS1115 initialized: %d", ads_initialized);

#ifdef ENABLE_TEST_OUTPUT_PIN
  pinMode(kTestOutputPin, OUTPUT);
  // Set the LEDC peripheral to a 13-bit resolution
  ledcAttach(kTestOutputPin, kTestOutputFrequency, 13);
  // Set the duty cycle to 50%
  // Duty cycle value is calculated based on the resolution
  // For 13-bit resolution, max value is 8191, so 50% is 4096
  ledcWrite(kTestOutputPin, 4096);
#endif

  /////////////////////////////////////////////////////////////////////
  // Initialize NMEA 2000 functionality

  nmea2000 = new tNMEA2000_esp32(kCANTxPin, kCANRxPin);

  // Reserve enough buffer for sending all messages.
  nmea2000->SetN2kCANSendFrameBufSize(250);
  nmea2000->SetN2kCANReceiveFrameBufSize(250);

  // Set Product information
  // EDIT: Change the values below to match your device.
  nmea2000->SetProductInformation(
      "20231229",  // Manufacturer's Model serial code (max 32 chars)
      104,         // Manufacturer's product code
      "HALMET",    // Manufacturer's Model ID (max 33 chars)
      "1.0.0",     // Manufacturer's Software version code (max 40 chars)
      "1.0.0"      // Manufacturer's Model version (max 24 chars)
  );

  // For device class/function information, see:
  // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf

  // For mfg registration list, see:
  // https://actisense.com/nmea-certified-product-providers/
  // The format is inconvenient, but the manufacturer code below should be
  // one not already on the list.

  // EDIT: Change the class and function values below to match your device.
  nmea2000->SetDeviceInformation(
      GetBoardSerialNumber(),  // Unique number. Use e.g. Serial number.
      140,                     // Device function: Engine
      50,                      // Device class: Propulsion
      2046);                   // Manufacturer code

  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly,
                    71  // Default N2k node address
  );
  nmea2000->EnableForward(false);
  nmea2000->Open();

  // No need to parse the messages at every single loop iteration; 1 ms will do
  event_loop()->onRepeat(1, []() { nmea2000->ParseMessages(); });

  // Initialize the OLED display
  bool display_present = InitializeSSD1306(sensesp_app->get(), &display, i2c);

  ///////////////////////////////////////////////////////////////////
  // Analog inputs

  bool enable_signalk_output = true;

  // Connect the tank senders.
  // EDIT: To enable more tanks, uncomment the lines below.
  auto tank_a1_volume = ConnectTankSender(ads1115, 0, "Fuel", "fuel.main", 3000,
                                          enable_signalk_output);
  // auto tank_a2_volume = ConnectTankSender(ads1115, 1, "A2");
  // auto tank_a3_volume = ConnectTankSender(ads1115, 2, "A3");
  // auto tank_a4_volume = ConnectTankSender(ads1115, 3, "A4");

  // Tank 1, instance 0. Capacity 200 liters. You can change the capacity
  // in the web UI as well.
  // EDIT: Make sure this matches your tank configuration above.
  N2kFluidLevelSender* tank_a1_sender = new N2kFluidLevelSender(
      "/Tanks/Fuel/NMEA 2000", 0, N2kft_Fuel, 200, nmea2000);

  ConfigItem(tank_a1_sender)
      ->set_title("Tank A1 NMEA 2000")
      ->set_description("NMEA 2000 tank sender for tank A1")
      ->set_sort_order(3005);

  // When the level is null, nothing is set and the NMEA 2000 tank level
  // expires to "not available".
  tank_a1_volume->connect_to(
      new LambdaConsumer<NullableFloat>([tank_a1_sender](NullableFloat value) {
        if (value.is_valid()) {
          tank_a1_sender->tank_level_.set(value.value());
        }
      }));

  if (display_present) {
    // EDIT: Duplicate the lines below to make the display show all your tanks.
    tank_a1_volume->connect_to(
        new LambdaConsumer<NullableFloat>([](NullableFloat value) {
          if (value.is_valid()) {
            PrintValue(display, 2, "Tank A1", 100 * value.value());
          } else {
            PrintValue(display, 2, "Tank A1", String("--"));
          }
        }));
  }

  ///////////////////////////////////////////////////////////////////
  // Engine wiring (Deutz/Sisu, William Knudsen AS diagram 434)
  //
  // HALMET  Signal                       Relay box term.  Wire colour
  // A2      Lube oil pressure sender     7                yellow 1.5
  // A3      Coolant temperature sender   9                green 1.5
  // A4      Battery voltage (key sw. 15) -                -
  // D1      Inductive RPM pickup         4 / 5            transparent
  // D2      Common alarm (alarm relay)   -                brown
  // D3      Alternator D+                6                orange 1
  // GND     Engine ground                10               black
  //
  // The original gauges stay connected, so A2 and A3 measure the voltage on
  // the sender wire rather than the sender resistance. The sender voltage
  // depends on the gauge and the supply voltage, so the curves below must be
  // calibrated against the gauges (web UI: "Oil Pressure Curve" and
  // "Coolant Temperature Curve"). Keep the HALMET current source disabled on
  // A2 and A3.

  N2kEngineParameterDynamicSender* engine_dynamic_sender =
      new N2kEngineParameterDynamicSender("/NMEA 2000/Engine 1 Dynamic", 0,
                                          nmea2000);

  ConfigItem(engine_dynamic_sender)
      ->set_title("Engine 1 Dynamic")
      ->set_description("NMEA 2000 dynamic engine parameters for engine 1")
      ->set_sort_order(3010);

  // A2: Lube oil pressure

  auto a2_voltage = new ADS1115VoltageInput(ads1115, 1, "/Oil Pressure/Voltage");

  ConfigItem(a2_voltage)
      ->set_title("Oil Pressure Sender Voltage")
      ->set_description("Voltage on the oil pressure sender wire (A2)")
      ->set_sort_order(3100);

  a2_voltage->connect_to(NullWhenUnpowered(a2_voltage))
      ->connect_to(new SKOutput<NullableFloat>(
      "propulsion.main.oilPressureSenderVoltage", "/Oil Pressure/Voltage SK Path",
      new SKMetadata("V", "Oil pressure sender voltage")));

  // EDIT: Placeholder curve (sender voltage -> Pa). Replace with values read
  // off the oil pressure gauge at a few engine speeds. The sender resistance
  // rises with pressure, so the sender voltage rises with pressure too.
  auto oil_pressure_curve =
      (new CurveInterpolator(nullptr, "/Oil Pressure/Curve"))
          ->set_input_title("Sender voltage (V)")
          ->set_output_title("Oil pressure (Pa)");

  ConfigItem(oil_pressure_curve)
      ->set_title("Oil Pressure Curve")
      ->set_description("Sender voltage to oil pressure (Pa). 1 bar = 100000 Pa")
      ->set_sort_order(3101);

  if (oil_pressure_curve->get_samples().empty()) {
    oil_pressure_curve->clear_samples();
    oil_pressure_curve->add_sample(CurveInterpolator::Sample(0., 0.));
    oil_pressure_curve->add_sample(CurveInterpolator::Sample(1., 0.));
    oil_pressure_curve->add_sample(CurveInterpolator::Sample(8., 500000.));
  }

  a2_voltage->connect_to(oil_pressure_curve);
  auto oil_pressure = oil_pressure_curve
                          ->connect_to(ClampToCurve(oil_pressure_curve))
                          ->connect_to(NullWhenUnpowered(a2_voltage));

  oil_pressure->connect_to(new SKOutput<NullableFloat>(
      "propulsion.main.oilPressure", "/Oil Pressure/SK Path",
      new SKMetadata("Pa", "Oil pressure")));
  // When the value is null, nothing is set and the NMEA 2000 field expires to
  // "not available" after a few seconds.
  oil_pressure->connect_to(new LambdaConsumer<NullableFloat>(
      [engine_dynamic_sender](NullableFloat value) {
        if (value.is_valid()) {
          engine_dynamic_sender->oil_pressure_->set(value.value());
        }
      }));

  // A3: Coolant temperature

  auto a3_voltage =
      new ADS1115VoltageInput(ads1115, 2, "/Coolant Temperature/Voltage");

  ConfigItem(a3_voltage)
      ->set_title("Coolant Temperature Sender Voltage")
      ->set_description("Voltage on the coolant temperature sender wire (A3)")
      ->set_sort_order(3110);

  a3_voltage->connect_to(NullWhenUnpowered(a3_voltage))
      ->connect_to(new SKOutput<NullableFloat>(
      "propulsion.main.coolantTemperatureSenderVoltage",
      "/Coolant Temperature/Voltage SK Path",
      new SKMetadata("V", "Coolant temperature sender voltage")));

  // EDIT: Placeholder curve (sender voltage -> K). Replace with values read
  // off the temperature gauge from cold to fully warm. The sender resistance
  // falls with temperature, so the sender voltage falls as the engine warms up.
  auto coolant_temperature_curve =
      (new CurveInterpolator(nullptr, "/Coolant Temperature/Curve"))
          ->set_input_title("Sender voltage (V)")
          ->set_output_title("Coolant temperature (K)");

  ConfigItem(coolant_temperature_curve)
      ->set_title("Coolant Temperature Curve")
      ->set_description(
          "Sender voltage to coolant temperature (K). K = degC + 273.15")
      ->set_sort_order(3111);

  if (coolant_temperature_curve->get_samples().empty()) {
    coolant_temperature_curve->clear_samples();
    coolant_temperature_curve->add_sample(CurveInterpolator::Sample(1., 393.15));
    coolant_temperature_curve->add_sample(CurveInterpolator::Sample(4., 353.15));
    coolant_temperature_curve->add_sample(CurveInterpolator::Sample(8., 313.15));
  }

  a3_voltage->connect_to(coolant_temperature_curve);
  auto coolant_temperature =
      coolant_temperature_curve
          ->connect_to(ClampToCurve(coolant_temperature_curve))
          ->connect_to(NullWhenUnpowered(a3_voltage));

  coolant_temperature->connect_to(new SKOutput<NullableFloat>(
      "propulsion.main.coolantTemperature", "/Coolant Temperature/SK Path",
      new SKMetadata("K", "Coolant temperature")));
  coolant_temperature->connect_to(new LambdaConsumer<NullableFloat>(
      [engine_dynamic_sender](NullableFloat value) {
        if (value.is_valid()) {
          engine_dynamic_sender->temperature_->set(value.value());
        }
      }));

  // A4: Battery voltage, taken from the key switch terminal 15 (switched +).

  auto a4_voltage = new ADS1115VoltageInput(ads1115, 3, "/Battery/Voltage");

  ConfigItem(a4_voltage)
      ->set_title("Start Battery Voltage")
      ->set_description("Start battery voltage (A4)")
      ->set_sort_order(3120);

  a4_voltage->connect_to(new SKOutputFloat(
      "electrical.batteries.start.voltage", "/Battery/Voltage SK Path",
      new SKMetadata("V", "Start battery voltage")));
  // NMEA 2000 has no battery field in the engine PGN; the alternator
  // potential field is the closest match and is shown on most engine pages.
  a4_voltage->connect_to(new LambdaConsumer<float>(
      [engine_dynamic_sender](float value) {
        engine_dynamic_sender->alternator_potential_->set(value);
      }));

  if (display_present) {
    coolant_temperature->connect_to(
        new LambdaConsumer<NullableFloat>([](NullableFloat value) {
          if (value.is_valid()) {
            PrintValue(display, 5, "Temp C", value.value() - 273.15);
          } else {
            PrintValue(display, 5, "Temp C", String("--"));
          }
        }));
  }

  ///////////////////////////////////////////////////////////////////
  // D1: Inductive RPM pickup

  // EDIT: Set the revolution multiplier in the web UI to 1 / (number of
  // flywheel teeth), or calibrate it against the tachometer.
  auto tacho_d1_frequency = ConnectTachoSender(kDigitalInputPin1, "main");

  tacho_d1_frequency->connect_to(new SKOutputFloat(
      "propulsion.main.revolutions", "/Tacho main/SK Path",
      new SKMetadata("Hz", "Engine revolutions")));

  N2kEngineParameterRapidSender* engine_rapid_sender =
      new N2kEngineParameterRapidSender("/NMEA 2000/Engine 1 Rapid Update", 0,
                                        nmea2000);  // Engine 1, instance 0

  ConfigItem(engine_rapid_sender)
      ->set_title("Engine 1 Rapid Update")
      ->set_description("NMEA 2000 rapid update engine parameters for engine 1")
      ->set_sort_order(3015);

  tacho_d1_frequency->connect_to(&(engine_rapid_sender->engine_speed_));

  if (display_present) {
    tacho_d1_frequency->connect_to(new LambdaConsumer<float>(
        [](float value) { PrintValue(display, 3, "RPM", 60 * value); }));
  }

  ///////////////////////////////////////////////////////////////////
  // D2: Common alarm from the engine alarm unit (brown wire to the alarm
  // relay coil). Lights for alternator, low oil pressure, high coolant
  // temperature and exhaust temperature all end up on this one signal.

  // HALMET digital inputs read high when nothing is connected, so an active
  // (powered) signal reads low. EDIT: Set to false if the alarm shows as
  // active while the engine alarm unit is quiet.
  const bool kCommonAlarmActiveLow = true;

  auto alarm_d2_input = new DigitalInputState(kDigitalInputPin2, INPUT, 100);
  auto common_alarm = alarm_d2_input->connect_to(new LambdaTransform<bool, bool>(
      [](bool value) { return kCommonAlarmActiveLow ? !value : value; }));

  common_alarm->connect_to(new SKOutputBool(
      "propulsion.main.alarm", "/Alarm/Common SK Path",
      new SKMetadata("", "Engine common alarm")));
  common_alarm->connect_to(
      new LambdaConsumer<bool>([](bool value) { alarm_states[1] = value; }));
  common_alarm->connect_to(new LambdaConsumer<bool>(
      [engine_dynamic_sender](bool value) {
        engine_dynamic_sender->check_engine_->set(value);
      }));

  ///////////////////////////////////////////////////////////////////
  // D3: Alternator D+. High while the alternator is charging, which also
  // means the engine is running.

  // HALMET digital inputs read high when nothing is connected, so D+ at
  // charging voltage reads low. EDIT: Set to false if alternatorCharging is
  // true while the engine is stopped.
  const bool kDPlusActiveLow = true;

  auto d3_input = new DigitalInputState(kDigitalInputPin3, INPUT, 100);
  auto charging = d3_input->connect_to(new LambdaTransform<bool, bool>(
      [](bool value) { return kDPlusActiveLow ? !value : value; }));

  charging->connect_to(new SKOutputBool(
      "propulsion.main.alternatorCharging", "/Alternator/Charging SK Path",
      new SKMetadata("", "Alternator charging (D+)")));

  // Engine hours, counted while D+ is high. The total is stored in flash and
  // can be set to the hour meter reading in the web UI.
  auto engine_hours = charging->connect_to(new TimeCounter<bool>("/Engine Hours"));

  ConfigItem(engine_hours)
      ->set_title("Engine Hours")
      ->set_description("Engine running time, counted while D+ is high")
      ->set_sort_order(3130);

  engine_hours->connect_to(new SKOutputFloat(
      "propulsion.main.runTime", "/Engine Hours/SK Path",
      new SKMetadata("s", "Engine running time")));
  engine_hours->connect_to(new LambdaConsumer<double>(
      [engine_dynamic_sender](double value) {
        engine_dynamic_sender->total_engine_hours_->set(value);
      }));

  // Charge warning: the engine is turning but the alternator is not charging.
  // EDIT: Adjust the RPM threshold to just below idle.
  const float kChargeWarningMinRevs = 300 / 60.;  // Hz
  static float engine_revs = 0;
  static bool alternator_charging = false;
  tacho_d1_frequency->connect_to(
      new LambdaConsumer<float>([](float value) { engine_revs = value; }));
  charging->connect_to(new LambdaConsumer<bool>(
      [](bool value) { alternator_charging = value; }));

  event_loop()->onRepeat(500, [engine_dynamic_sender, kChargeWarningMinRevs]() {
    bool no_charge = engine_revs > kChargeWarningMinRevs && !alternator_charging;
    alarm_states[2] = no_charge;
    engine_dynamic_sender->charge_indicator_->set(no_charge);
  });

  ///////////////////////////////////////////////////////////////////
  // Display setup

  // Connect the outputs to the display
  if (display_present) {
    event_loop()->onRepeat(1000, []() {
      PrintValue(display, 1, "IP:", WiFi.localIP().toString());
    });

    // Create a poor man's "christmas tree" display for the alarms
    event_loop()->onRepeat(1000, []() {
      char state_string[5] = {};
      for (int i = 0; i < 4; i++) {
        state_string[i] = alarm_states[i] ? '*' : '_';
      }
      PrintValue(display, 4, "Alarm", state_string);
    });
  }

  // To avoid garbage collecting all shared pointers created in setup(),
  // loop from here.
  while (true) {
    loop();
  }
}

void loop() { event_loop()->tick(); }
