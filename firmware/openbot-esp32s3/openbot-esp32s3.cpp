#include <Arduino.h>

// OpenBot firmware for an ESP32-S3 (N16R8) Off-Road 4WD Ackermann robot car
// ---------------------------------------------------------------------------
// Speaks the same serial / BLE protocol as firmware/openbot/openbot.ino, so the
// OpenBot Android app drives it unchanged. Where openbot.ino spreads its drive
// code across a differential-drive H-bridge, this car has:
//
//   - one ESC driving all four motors (a single throttle line)
//   - one steering servo on the front axle (Ackermann geometry)
//
// which is the same arrangement as openbot.ino's RC_CAR body, except that body
// is compiled for Arduino Nano only and uses the AVR Servo library. This sketch
// targets the ESP32-S3 and generates the 50 Hz servo/ESC pulses with the LEDC
// peripheral directly, so it needs no external library at all.
//
// Peripherals fitted: an LDROBOT LD14/LD14P LiDAR, turn indicators and
// front/rear/status LEDs. There are no wheel encoders or voltage divider, so
// those readings are not reported and the app does not ask for them (see the
// feature string in process_feature_msg).
//
// The LiDAR does three jobs:
//
//   - it replaces the HC-SR04. The nearest return in a forward cone is
//     reported on the OpenBot 's' sonar channel, so the FreeRoam obstacle bar,
//     the RobotInfo readout and the sonarReading() BotFunction all work, and
//     stop_at_obstacle() holds the car back from whatever is in front.
//   - the same revolution is folded into eight proximity octants on the 'p'
//     channel, which is what the app's object avoidance reads to stop before
//     hitting things (ArduPilot Rover-style, see that feature's own docs).
//   - it also sends full 360 degree scans over UDP in the "LDS1" format, so
//     robot_ws's udp_lidar_receiver_node can publish /scan for SLAM. That part
//     needs WiFi and is entirely optional - see LIDAR_UDP in SETTINGS. The car
//     is driven over BLE either way and never waits on the network.
//
// Wiring and calibration live in the SETTINGS section below. Nothing else needs
// editing.
// ---------------------------------------------------------------------------

//------------------------------------------------------//
// SETTINGS - set these for your robot
//------------------------------------------------------//
// To keep your settings across firmware updates, copy this section (between
// #else and #endif) into a new file openbot_settings.h next to this sketch. If
// that file exists it is used instead, and git ignores it.
// RC_PROTOCOL values for section 12 (defined here so a settings file can use them).
#define RC_IBUS 1  // FlySky FS-iA6B and other iBUS receivers: one signal wire
#define RC_PWM 2   // FlySky FS-iA6 and other plain receivers: one wire per channel

#if __has_include("openbot_settings.h")
#include "openbot_settings.h"
#ifndef OPENBOT_BOARD
#error "openbot_settings.h must set OPENBOT_BOARD."
#endif
#else

// 1. Board. Only ESP32-S3 is supported here; see the pin notes below.
#define OPENBOT_BOARD ESP32S3

// 2. Driving
#define DEBUG 0  // debug output on the Serial Monitor (1,0)

// 3. ESC and servo calibration, in microseconds. 1500 us is neutral for both.
//    Widen the throttle range only after confirming the ESC arms and the wheels
//    leave the ground - an off-road car on full throttle will run away.
#define ESC_MIN_US 1000     // full reverse
#define ESC_NEUTRAL_US 1500 // stopped
#define ESC_MAX_US 2000     // full forward
#define SERVO_MIN_US 1000   // full lock one way
#define SERVO_CENTER_US 1500
#define SERVO_MAX_US 2000   // full lock the other way
#define SERVO_TRIM_US 0     // nudge the straight-ahead position, + or - us
#define SERVO_REVERSED 0    // 1 if the car steers the wrong way

// 4. Failsafe. If no command arrives within this many milliseconds the motors
//    stop and the wheels straighten. Set 0 to disable (not recommended).
#define CONTROL_TIMEOUT_MS 1000

// 5. Pins. N16R8 is 16 MB flash + 8 MB *octal* PSRAM, so both the flash bus and
//    the octal PSRAM bus are wired on the die and cannot be freed.
//
//      GPIO 26-32  flash (SPICS1/SPIHD/SPIWP/SPICS0/SPICLK/SPIQ/SPID)
//      GPIO 33-37  octal PSRAM (DQ4-DQ7, DQS/DM) - ESP32-S3 Series Datasheet
//                  v2.2 Table 2-14, "Octal SPI/OPI" column
//
//    The datasheet is explicit: "Do not use the pins connected to in-package
//    flash/PSRAM for any other purposes." Wiring anything there makes the
//    board unbootable, so nothing in this sketch touches 26-37.
//
//    Also avoided: GPIO 0, 3, 45, 46 (strapping - boot mode, JTAG source,
//    VDD_SPI voltage, ROM print) and GPIO 19/20 (USB D-/D+) and 43/44 (UART0).
//    Everything below is a free general-purpose GPIO.
//
//    Replace these with the labels printed on your wiring loom.
#define PIN_ESC 1          // throttle, 50 Hz
#define PIN_SERVO 2        // steering, 50 Hz
#define PIN_LED_LI 4       // left turn indicator
#define PIN_LED_RI 5       // right turn indicator
#define PIN_LED_LF 6       // left front light
#define PIN_LED_RF 7       // right front light
#define PIN_LED_LB 8       // left rear light
#define PIN_LED_RB 9       // right rear light
#define PIN_LED_Y 10       // status, yellow
#define PIN_LED_G 11       // status, green
#define PIN_LED_B 12       // status, blue

// 6. LiDAR. LDROBOT LD14/LD14P on UART1. Only the LiDAR's TX is wired - it
//    free-runs and never needs a command, so LIDAR_TX_PIN stays -1.
#define LIDAR_RX_PIN 13    // LiDAR TX -> here
#define LIDAR_TX_PIN -1    // not routed
#define HAS_LIDAR 1        // set 0 to build without any LiDAR support

// 7. LiDAR model profile. The two lines must match each other and the unit.
//
//      model    LIDAR_BAUD  LIDAR_MEASURE_FREQ
//      LD14       115200           2300
//      LD14P      230400           4000
//
//    If the LiDAR stays silent the sketch walks CANDIDATE_BAUD looking for a
//    rate that frames up, so a wrong guess here is self-correcting - but
//    LIDAR_MEASURE_FREQ must still match the model, since it is used to reject
//    corrupt packets whose CRC happened to pass.
#define LIDAR_BAUD 115200
#define LIDAR_MEASURE_FREQ 2300
static const uint32_t CANDIDATE_BAUD[] = {115200, 230400, 460800, 921600};

// 8. Scan geometry. One bin per SCAN_BINS-th of a turn. Too many bins
//    manufactures phantom gaps, too few throws away detail.
#define SCAN_BINS 360
// Valid LD14/LD14P range is 0.02 .. 8.0 m.
#define RANGE_MIN_MM 20
#define RANGE_MAX_MM 8000
// A revolution whose wrap-around point never arrived is flushed once this old.
#define SCAN_MAX_AGE_MS 500

// 9. Obstacle detection. LIDAR_FORWARD_DEG is where the car's nose sits in the
//    LiDAR's own clockwise frame: 0 means "mounted with the zero mark facing
//    forward". Set it if the unit is bolted on rotated. The forward cone is
//    +/- OBSTACLE_HALF_ANGLE_DEG around that, and the nearest return inside it
//    is what gets reported and acted on.
#define LIDAR_FORWARD_DEG 0.0f
#define OBSTACLE_HALF_ANGLE_DEG 20.0f
// Hold the car back when something in the cone is closer than this, in cm.
// openbot.ino uses 10 cm for a slow indoor robot; an off-road car needs to
// stop much further out, since it cannot stop in 10 cm.
#define STOP_DISTANCE 40

// Proximity sectors reported on the 'p' channel, which the app's object
// avoidance consumes. Fixed at 8 to match the MAVLink DISTANCE_SENSOR octant
// layout that ArduPilot's proximity library uses - sector 0 straight ahead,
// then clockwise every 45 degrees. Change both sides together if at all.
#define PROX_SECTORS 8

// 10. Scan streaming to ROS 2 over UDP, in the "LDS1" wire format that
//     robot_ws's udp_lidar_receiver_node understands. Set LIDAR_UDP 0 to build
//     without WiFi at all - the car is still fully drivable over BLE, it just
//     sends no scans.
#define LIDAR_UDP 1
#define WIFI_SSID "your-ssid"
#define WIFI_PASS "your-password"
#define WIFI_RETRY_MS 10000
#define LIDAR_UDP_DEST_IP "192.168.1.100"  // the machine running ROS 2
#define LIDAR_UDP_PORT 9900                // udp_lidar_receiver_node's port

// 10b. Scans down the USB cable to a phone (see lidar_serial.h). Sent only
//      after the app asks with "x<ms>", so a serial monitor never sees them.
//      SERIAL_SCAN_STEP folds that many bins into one: 2 suits a 115200 baud
//      COM/UART port, 1 (full resolution) suits the native USB port.
#define LIDAR_SERIAL_SCAN 1
#define SERIAL_SCAN_STEP 2

// 11. Parts: set 1 (on) or 0 (off) if your car differs from the above.
#define HAS_INDICATORS 1
#define HAS_LEDS_FRONT 1
#define HAS_LEDS_BACK 1
#define HAS_LEDS_STATUS 1
#define HAS_BLUETOOTH 1  // BLE link to the phone

// 12. RC receiver and the MANUAL / AUTO / HOLD switch (FlySky FS-i6 transmitter).
//     A transmitter switch picks who drives: MANUAL = the RC sticks,
//     AUTO = the phone or computer, HOLD = stopped. With no receiver wired the
//     car behaves as before (the app drives). See rc_input.h and
//     docs/FIRMWARE_GUIDE.md for the transmitter setup, including its failsafe.
#define HAS_RC 1
#define RC_PROTOCOL RC_IBUS   // RC_IBUS (FS-iA6B, "SERVO" port) or RC_PWM (FS-iA6)
#define RC_IBUS_PIN 14        // iBUS signal -> here (UART2 RX)
#define RC_PWM_PIN_STEER 14   // RC_PWM only: CH1 -> here
#define RC_PWM_PIN_THROTTLE 15  // RC_PWM only: CH2 -> here
#define RC_PWM_PIN_MODE 16    // RC_PWM only: CH5 -> here
// Channels (iBUS only; with RC_PWM the wiring decides). On a mode-2 FS-i6 the
// right stick springs back to centre on both axes, so it drives the car:
// CH1 = right stick left/right, CH2 = right stick up/down. The left stick
// (CH3) has no spring and would leave the car running.
#define RC_CH_STEER 1
#define RC_CH_THROTTLE 2
#define RC_CH_MODE 5          // assign SwC (3-position) to CH5 on the transmitter
// What each switch position means: RC_MANUAL, RC_AUTO or RC_HOLD.
// For a 2-position switch use LOW and HIGH only (e.g. HIGH = RC_AUTO).
#define RC_MODE_LOW RC_MANUAL   // about 1000 us (SwC up)
#define RC_MODE_MID RC_AUTO     // about 1500 us (SwC middle)
#define RC_MODE_HIGH RC_HOLD    // about 2000 us (SwC down)
#define RC_DEADBAND_US 30       // centred stick within +/-30 us = stop / straight
#define RC_TIMEOUT_MS 250       // no valid RC frame for this long = link lost
#define RC_LOST_ACTION 0        // on link loss: 0 = hold, 1 = keep driving if in AUTO
#define RC_MANUAL_OBSTACLE_STOP 1  // the LiDAR stop also guards MANUAL driving

#endif
//------------------------------------------------------//

// Defaults for settings added after openbot_settings.h files were first copied
// out, so an older settings file still builds.
#ifndef LIDAR_SERIAL_SCAN
#define LIDAR_SERIAL_SCAN 1
#endif
#ifndef SERIAL_SCAN_STEP
#define SERIAL_SCAN_STEP 2
#endif
// An openbot_settings.h written before section 12 existed has no receiver.
#ifndef HAS_RC
#define HAS_RC 0
#endif
#if (HAS_RC)
#ifndef RC_PROTOCOL
#define RC_PROTOCOL RC_IBUS
#endif
#ifndef RC_IBUS_PIN
#define RC_IBUS_PIN 14
#endif
#ifndef RC_PWM_PIN_STEER
#define RC_PWM_PIN_STEER 14
#endif
#ifndef RC_PWM_PIN_THROTTLE
#define RC_PWM_PIN_THROTTLE 15
#endif
#ifndef RC_PWM_PIN_MODE
#define RC_PWM_PIN_MODE 16
#endif
#ifndef RC_CH_STEER
#define RC_CH_STEER 1
#endif
#ifndef RC_CH_THROTTLE
#define RC_CH_THROTTLE 2
#endif
#ifndef RC_CH_MODE
#define RC_CH_MODE 5
#endif
#ifndef RC_MODE_LOW
#define RC_MODE_LOW RC_MANUAL
#endif
#ifndef RC_MODE_MID
#define RC_MODE_MID RC_AUTO
#endif
#ifndef RC_MODE_HIGH
#define RC_MODE_HIGH RC_HOLD
#endif
#ifndef RC_DEADBAND_US
#define RC_DEADBAND_US 30
#endif
#ifndef RC_TIMEOUT_MS
#define RC_TIMEOUT_MS 250
#endif
#ifndef RC_LOST_ACTION
#define RC_LOST_ACTION 0
#endif
#ifndef RC_MANUAL_OBSTACLE_STOP
#define RC_MANUAL_OBSTACLE_STOP 1
#endif
#endif

#if (OPENBOT_BOARD != ESP32S3)
#error "This firmware is written for the ESP32-S3. For other boards use firmware/openbot/openbot.ino."
#endif
#if !defined(ARDUINO_ARCH_ESP32)
#error "Select an ESP32-S3 board in the Arduino IDE, e.g. \"ESP32S3 Dev Module\"."
#endif

// Reported to the app in the feature string. Keep it in sync with the parts
// above - it is how the app decides which readings to expect.
const String robot_type = "RC_S3";

// The OpenBot 's' channel is specced in centimetres and the app's FreeRoam bar
// is drawn for this ceiling, so the LiDAR's 8 m range is capped to it rather
// than overflowing the gauge. Matches openbot.ino.
const unsigned int MAX_SONAR_DISTANCE = 300;  // cm

// Forward declarations. Arduino normally generates these with ctags, but that
// pass is unreliable (and its bundled ctags is an Intel-only binary), so they
// are written out here. Keep in sync with the definitions below.
void stop_vehicle();
void stop_at_obstacle();
void drive_from_rc();
void update_vehicle();
void write_throttle(int throttle);
void write_steering(int steering);
void update_indicator();
void update_light();
void on_serial_rx();
void on_ble_rx(char inChar);
void process_header(char inChar);
void process_body(char inChar);
void parse_msg();
void process_ctrl_msg();
void process_heartbeat_msg();
void process_sonar_msg();
void process_indicator_msg();
void process_light_msg();
void process_notification_msg();
void process_feature_msg();
void sendData(String data);
void setup_ble();
void send_sonar_reading();
void send_proximity_reading();

#if (HAS_LIDAR)
#include "lidar_ld14.h"
// lidar_udp.h gates itself on LIDAR_UDP and stubs the three entry points out
// when scans are not being streamed, so no further #ifs are needed in loop().
#include "lidar_udp.h"
// Same scans down the USB serial link, for a phone plugged in by cable.
#include "lidar_serial.h"
#else
// No LiDAR fitted: no obstacle distance and no scans. Stubs keep loop() simple.
static void wifiLoop() {}
static void lidarSendPendingScan() {}
static void lidarUdpBegin() {}
static void lidarBegin() {}
static void lidarSerialPrepare() {}
static void process_scan_request(const char *body) { (void)body; }
static void lidarSendSerialScan() {}
static unsigned int lidarObstacleCm() { return MAX_SONAR_DISTANCE; }
static bool lidarProximityCm(unsigned int out_cm[PROX_SECTORS]) {
  (void)out_cm;
  return false;
}
#endif

// RC receiver and the MANUAL / AUTO / HOLD switch. Stubs itself out when
// HAS_RC is 0, so loop() needs no #ifs.
#include "rc_input.h"

//------------------------------------------------------//
// LEDC helpers
//------------------------------------------------------//
// The LEDC API changed in the ESP32 Arduino core 3.0: channel-based calls were
// replaced by pin-based ones. These two wrappers keep the sketch building on
// either core without pulling in a servo library.
#include <esp_arduino_version.h>

static void pwm_setup(int pin, int channel, double freq, int resolution) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, freq, resolution);
#else
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(pin, channel);
#endif
}

static void pwm_write(int pin, int channel, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)channel;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(channel, duty);
#endif
}

// A hobby ESC and a steering servo both take a 50 Hz pulse whose width sets the
// command: 1000 us one extreme, 2000 us the other, 1500 us neutral.
const double SERVO_FREQ = 50.0;
const int SERVO_RES = 16;  // 65535 counts per 20 ms period, ~0.3 us per count
const uint32_t SERVO_PERIOD_US = 20000;

const int CH_ESC = 0;
const int CH_SERVO = 1;

static uint32_t us_to_duty(unsigned int us) {
  return (uint32_t)us * 65535UL / SERVO_PERIOD_US;
}

// Maps a -510..510 drive value onto a pulse width. 510 is 255 + 255, the widest
// sum or difference the app can send with its default speed multipliers.
static unsigned int drive_to_us(int drive, unsigned int min_us, unsigned int neutral_us,
                               unsigned int max_us) {
  const int limit = 510;
  if (drive <= -limit) return min_us;
  if (drive >= limit) return max_us;
  if (drive > 0) {
    return neutral_us + (int)((int64_t)drive * (int)(max_us - neutral_us) / limit);
  }
  return neutral_us - (int)((int64_t)(-drive) * (int)(neutral_us - min_us) / limit);
}

//------------------------------------------------------//
// Protocol
//------------------------------------------------------//
// Messages are one line each, terminated by '\n'. The first character is the
// header, the rest is the body. This is the same framing as openbot.ino.
//
//   app -> car:  c<left>,<right>   drive, each -255..255
//                i<left>,<right>   turn indicators, 0 or 1
//                l<front>,<back>   lights, 0..255
//                n<led>,<state>    status LED, led is 'y','g' or 'b'
//                h<ms>             heartbeat interval
//                s<ms>             obstacle report interval
//                v<ms> w<ms>       sensor intervals (unused, accepted anyway)
//                f                 feature query
//                x<ms>             LiDAR scans on this serial link, 0 = off
//
//   car -> app:  f<robot_type>:<features>   reply to f
//                s<cm>                       forward-cone obstacle distance
//                p<d0>,<d1>,...<d7>          proximity octants, cm
//                                          0 = nothing in range (clear),
//                                          not "obstacle at zero"
//                vmin:<v> vlow:<v> vmax:<v>  reply to v
//                x<base64 LDS1 datagram>     one LiDAR revolution, serial
//                                          only, after the app sent x<ms>
//
enum msgParts { HEADER, BODY };

msgParts msgPart = HEADER;
char header;
const char endChar = '\n';
const char MAX_MSG_SZ = 60;
char msg_buf[MAX_MSG_SZ] = "";
int msg_idx = 0;

//Vehicle Control
int ctrl_left = 0;
int ctrl_right = 0;

// Obstacle report pacing. The app sets this with 's'; 100 ms is what it asks
// for once it sees the 's' feature flag. Only used when a LiDAR is fitted.
unsigned long sonar_interval = 100;  // ms
unsigned long sonar_time = 0;

// Indicator Signal
unsigned long indicator_interval = 500;  // blink rate of the indicators (ms)
unsigned long indicator_time = 0;
bool indicator_left = 0;
bool indicator_right = 0;

#if (HAS_LEDS_FRONT || HAS_LEDS_BACK)
unsigned int light_front = 0;
unsigned int light_back = 0;
#endif

// Heartbeat. The app repeats 'h750' every 250 ms while it is driving; when the
// interval elapses with no command the car stops and straightens. Armed from
// boot with CONTROL_TIMEOUT_MS so the car is never left without a failsafe -
// unlike openbot.ino, which starts at -1 and only arms it once the app sends
// its first 'h'.
unsigned long heartbeat_interval = CONTROL_TIMEOUT_MS;
unsigned long heartbeat_time = 0;

#if (HAS_BLUETOOTH)
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer *bleServer = NULL;
BLECharacteristic *pTxCharacteristic;
BLECharacteristic *pRxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
// The app scans for this exact service; the UUIDs must match
// robot/android/.../vehicle/BluetoothManager.java.
const char *SERVICE_UUID = "61653dc3-4021-4d1e-ba83-8b4eec61d613";
const char *CHARACTERISTIC_UUID_RX = "06386c14-86ea-4d71-811c-48f97c58f8c9";
const char *CHARACTERISTIC_UUID_TX = "9bf1103b-834c-47cf-b149-c9e4bcf778a7";
#endif

//------------------------------------------------------//
// SETUP
//------------------------------------------------------//
void setup() {
  // Room for a whole LiDAR scan line in the transmit buffer; must come before
  // begin(). Also stops the short reports from ever blocking the loop.
  lidarSerialPrepare();
  Serial.begin(115200);

  // Motors first, so they are held neutral as early as possible after boot.
  pwm_setup(PIN_ESC, CH_ESC, SERVO_FREQ, SERVO_RES);
  pwm_setup(PIN_SERVO, CH_SERVO, SERVO_FREQ, SERVO_RES);
  stop_vehicle();

#if (HAS_INDICATORS)
  pinMode(PIN_LED_LI, OUTPUT);
  pinMode(PIN_LED_RI, OUTPUT);
#endif
#if (HAS_LEDS_BACK)
  pinMode(PIN_LED_LB, OUTPUT);
  pinMode(PIN_LED_RB, OUTPUT);
#endif
#if (HAS_LEDS_FRONT)
  pinMode(PIN_LED_LF, OUTPUT);
  pinMode(PIN_LED_RF, OUTPUT);
#endif
#if (HAS_LEDS_STATUS)
  pinMode(PIN_LED_Y, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
#endif

  // Test sequence for the indicator LEDs
#if (HAS_INDICATORS)
  digitalWrite(PIN_LED_LI, LOW);
  digitalWrite(PIN_LED_RI, LOW);
  delay(500);
  digitalWrite(PIN_LED_LI, HIGH);
  delay(500);
  digitalWrite(PIN_LED_LI, LOW);
  digitalWrite(PIN_LED_RI, HIGH);
  delay(500);
  digitalWrite(PIN_LED_RI, LOW);
#endif

  Serial.print("OpenBot ");
  Serial.print(robot_type);
  Serial.println(" (ESP32-S3): ESC throttle, Ackermann steering servo, LD14 LiDAR, LEDs");

#if (HAS_LIDAR)
  lidarBegin();
  lidarUdpBegin();  // no-op unless LIDAR_UDP; never blocks on the router
  // Drain the LiDAR UART on its own task so BLE and WiFi work in loop() can
  // never starve it. A late scan costs one revolution; a late command frame
  // costs a heartbeat and stops the car.
  xTaskCreatePinnedToCore(lidarTask, "lidar", 4096, NULL, 2, NULL, 0);
#endif

  rcBegin();  // no-op unless HAS_RC

  Serial.println("Waiting for a client connection...");

#if (HAS_BLUETOOTH)
  setup_ble();
#endif
}

#if (HAS_BLUETOOTH)
// The classes must be defined before setup_ble() news them; the Arduino build
// only auto-prototypes free functions, not classes.

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *bleServer) {
    deviceConnected = true;
    Serial.println("BT Connected");
  };
  void onDisconnect(BLEServer *bleServer) {
    deviceConnected = false;
    Serial.println("BT Disconnected");
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue();
    for (unsigned int i = 0; i < value.length(); i++) {
      on_ble_rx(value[i]);
    }
  }
};

void on_ble_rx(char inChar) {
  if (inChar != endChar) {
    switch (msgPart) {
      case HEADER:
        process_header(inChar);
        return;
      case BODY:
        process_body(inChar);
        return;
    }
  } else {
    msg_buf[msg_idx] = '\0';
    parse_msg();
  }
}

void setup_ble() {
  // The device name is what the app lists when scanning.
  BLEDevice::init("OpenBot");
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new MyServerCallbacks());
  BLEService *bleService = bleServer->createService(SERVICE_UUID);
  pTxCharacteristic = bleService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());
  pRxCharacteristic = bleService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new MyCallbacks());
  bleService->start();
  bleServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}
#endif

//------------------------------------------------------//
// LOOP
//------------------------------------------------------//
void loop() {
#if (HAS_BLUETOOTH)
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);                     // give the bluetooth stack time to settle
    bleServer->startAdvertising();  // restart advertising
    Serial.println("Waiting a client connection to notify...");
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
#endif

  if (Serial.available() > 0) {
    on_serial_rx();
  }

  // Who drives: the RC switch decides. Without a receiver this is always
  // RC_NOT_FITTED and the app drives, exactly as before.
  rcPoll();
  RcMode rc_mode = rcMode();
  static RcMode rc_mode_prev = RC_NOT_FITTED;
  if (rc_mode != rc_mode_prev) {
    // Forget the last app command on every switch, so flipping back to AUTO
    // waits for a fresh command instead of replaying an old one.
    ctrl_left = 0;
    ctrl_right = 0;
    Serial.print("RC mode: ");
    Serial.println(rcModeName(rc_mode));
    rc_mode_prev = rc_mode;
  }

  if (rc_mode == RC_MANUAL) {
    drive_from_rc();
  } else if (rc_mode == RC_HOLD || rc_mode == RC_LOST) {
    stop_vehicle();
  } else {
    // RC_AUTO or no receiver: the app drives.

    // Failsafe: without a heartbeat the car stops and straightens.
    if (heartbeat_interval > 0 && (millis() - heartbeat_time) >= heartbeat_interval) {
      ctrl_left = 0;
      ctrl_right = 0;
      stop_vehicle();
    }

    // Hold the car back from whatever is in front. This has to run after the
    // failsafe and before update_vehicle(), so the override is what gets driven.
    stop_at_obstacle();

    update_vehicle();
  }

#if (HAS_LIDAR)
  // Report the forward-cone distance on the OpenBot 's' channel at the rate
  // the app asked for, and the proximity octants on 'p' alongside it - the app
  // sets the interval with 's' and expects both at that rate.
  if ((millis() - sonar_time) >= sonar_interval) {
    sonar_time = millis();
    send_sonar_reading();
    send_proximity_reading();
  }
#endif

  // Scan streaming. Both are cheap when there is nothing to do, and neither
  // blocks: WiFi associates in the background and a missing scan is simply
  // dropped rather than queued.
  wifiLoop();
  lidarSendPendingScan();
  lidarSendSerialScan();

#if (HAS_INDICATORS)
  if ((millis() - indicator_time) >= indicator_interval) {
    update_indicator();
    indicator_time = millis();
  }
#endif
}

//------------------------------------------------------//
// VEHICLE
//------------------------------------------------------//
// The app's controls are a pair of tank-style values. An Ackermann car has one
// throttle and one steering angle, so they are mixed the way an RC transmitter
// mixes two sticks:
//
//   throttle = ctrl_left + ctrl_right   forward / backward
//   steering = ctrl_left - ctrl_right   left / right
//
// The same mixing as openbot.ino's RC_CAR body.
void update_vehicle() {
  int throttle = ctrl_left + ctrl_right;
  int steering = ctrl_left - ctrl_right;

  // Reverse the steering when driving backwards so the car turns the way the
  // tail is pointing, as a real car does.
  if (throttle < 0) {
    steering = -steering;
  }

  write_throttle(throttle);
  write_steering(steering);
}

void write_throttle(int throttle) {
  unsigned int us = drive_to_us(throttle, ESC_MIN_US, ESC_NEUTRAL_US, ESC_MAX_US);
  pwm_write(PIN_ESC, CH_ESC, us_to_duty(us));
}

void write_steering(int steering) {
  if (SERVO_REVERSED) {
    steering = -steering;
  }
  unsigned int us = drive_to_us(steering, SERVO_MIN_US, SERVO_CENTER_US, SERVO_MAX_US);
  us = (int)us + SERVO_TRIM_US;
  if (us < SERVO_MIN_US) us = SERVO_MIN_US;
  if (us > SERVO_MAX_US) us = SERVO_MAX_US;
  pwm_write(PIN_SERVO, CH_SERVO, us_to_duty(us));
}

void stop_vehicle() {
  ctrl_left = 0;
  ctrl_right = 0;
  // Neutral throttle and straight wheels. The ESC needs to see neutral to arm.
  pwm_write(PIN_ESC, CH_ESC, us_to_duty(ESC_NEUTRAL_US));
  unsigned int us = (int)SERVO_CENTER_US + SERVO_TRIM_US;
  pwm_write(PIN_SERVO, CH_SERVO, us_to_duty(us));
}

// MANUAL mode: the RC sticks drive the ESC and servo directly. Unlike the app
// path there is no left/right mixing and no steering flip in reverse - the
// steering stick is the steering wheel, as on any RC car. Reverse a channel on
// the transmitter if it runs the wrong way. The servo trim, SERVO_REVERSED and
// the ESC/servo limits in SETTINGS still apply.
void drive_from_rc() {
  int throttle = rcUsToDrive(rcThrottleUs(), RC_DEADBAND_US);
  int steering = rcUsToDrive(rcSteerUs(), RC_DEADBAND_US);
#if (HAS_RC && RC_MANUAL_OBSTACLE_STOP)
  // Same rule as stop_at_obstacle(): block forward, allow backing away.
  if (STOP_DISTANCE > 0 && throttle > 0 && lidarObstacleCm() <= (unsigned int)STOP_DISTANCE) {
    throttle = 0;
  }
#endif
  write_throttle(throttle);
  write_steering(steering);
}

// Stop the car driving forward when something in the LiDAR's forward cone is
// closer than STOP_DISTANCE. Same idea as openbot.ino's, but keyed off throttle
// (left + right) rather than both channels being positive: on an Ackermann car
// the two app channels are tank-style sticks that mix into one throttle, so
// "both positive" is not the same as "driving forward".
//
// Reversing is deliberately not blocked - backing away from a wall is exactly
// what the driver wants.
void stop_at_obstacle() {
  if (STOP_DISTANCE <= 0)
    return;  // disabled
  if (lidarObstacleCm() > (unsigned int)STOP_DISTANCE)
    return;
  if (ctrl_left + ctrl_right > 0) {
    ctrl_left = 0;
    ctrl_right = 0;
  }
}

// The LiDAR's forward-cone distance, reported on the OpenBot 's' sonar channel.
// This is what the FreeRoam obstacle bar, the RobotInfo readout and the
// sonarReading() BotFunction all consume.
void send_sonar_reading() {
  sendData("s" + String(lidarObstacleCm()));
}

// The eight proximity octants, reported on the 'p' channel for the app's object
// avoidance. Sector 0 is straight ahead and the numbering goes clockwise, the
// MAVLink DISTANCE_SENSOR orientation layout.
//
// Body size: "p" + 8 x up to 3 digits + 7 commas = 32 chars, comfortably under
// MAX_MSG_SZ.
//
// When there is no fresh scan nothing is sent at all. Sending zeros would read
// as "clear road in every sector" and drive the car straight at whatever just
// killed the LiDAR; silence lets the app's staleness check take over instead.
void send_proximity_reading() {
  unsigned int cm[PROX_SECTORS];
  if (!lidarProximityCm(cm))
    return;

  String msg = "p";
  for (int i = 0; i < PROX_SECTORS; i++) {
    if (i > 0)
      msg += ",";
    msg += String(cm[i]);
  }
  sendData(msg);
}

#if (HAS_INDICATORS)
void update_indicator() {
  if (indicator_left > 0) {
    digitalWrite(PIN_LED_LI, !digitalRead(PIN_LED_LI));
  } else {
#if (HAS_LEDS_BACK)
    digitalWrite(PIN_LED_LI, light_back);
#else
    digitalWrite(PIN_LED_LI, LOW);
#endif
  }
  if (indicator_right > 0) {
    digitalWrite(PIN_LED_RI, !digitalRead(PIN_LED_RI));
  } else {
#if (HAS_LEDS_BACK)
    digitalWrite(PIN_LED_RI, light_back);
#else
    digitalWrite(PIN_LED_RI, LOW);
#endif
  }
}
#endif

#if (HAS_LEDS_FRONT || HAS_LEDS_BACK)
void update_light() {
#if (HAS_LEDS_FRONT)
  analogWrite(PIN_LED_LF, light_front);
  analogWrite(PIN_LED_RF, light_front);
#endif
#if (HAS_LEDS_BACK)
  analogWrite(PIN_LED_LB, light_back);
  analogWrite(PIN_LED_RB, light_back);
#endif
}
#endif

//------------------------------------------------------//
// MESSAGE HANDLING
//------------------------------------------------------//
void on_serial_rx() {
  char inChar = Serial.read();
  if (inChar != endChar) {
    switch (msgPart) {
      case HEADER:
        process_header(inChar);
        return;
      case BODY:
        process_body(inChar);
        return;
    }
  } else {
    msg_buf[msg_idx] = '\0';
    parse_msg();
  }
}

void process_header(char inChar) {
  header = inChar;
  msgPart = BODY;
}

void process_body(char inChar) {
  if (msg_idx < MAX_MSG_SZ - 1) {
    msg_buf[msg_idx] = inChar;
    msg_idx++;
  }
}

void parse_msg() {
  switch (header) {
    case 'c':
      process_ctrl_msg();
      break;
    case 'f':
      process_feature_msg();
      break;
    case 'h':
      process_heartbeat_msg();
      break;
#if (HAS_INDICATORS)
    case 'i':
      process_indicator_msg();
      break;
#endif
#if (HAS_LEDS_FRONT || HAS_LEDS_BACK)
    case 'l':
      process_light_msg();
      break;
#endif
#if (HAS_LEDS_STATUS)
    case 'n':
      process_notification_msg();
      break;
#endif
#if (HAS_LIDAR)
    case 's':
      process_sonar_msg();
      break;
    case 'x':
      process_scan_request(msg_buf);
      break;
#endif
    // The app paces its sensors even when a car has none. Accepting these
    // keeps it from waiting on a reply that would never come.
    case 'v':
    case 'w':
      break;
  }
  msg_idx = 0;
  msgPart = HEADER;
  header = '\0';
}

void process_ctrl_msg() {
  char *tmp = strtok(msg_buf, ",:");  // replace delimiter with \0
  ctrl_left = atoi(tmp);
  tmp = strtok(NULL, ",:");
  ctrl_right = atoi(tmp);
  heartbeat_time = millis();
#if DEBUG
  Serial.print("Control: ");
  Serial.print(ctrl_left);
  Serial.print(",");
  Serial.println(ctrl_right);
#endif
}

void process_heartbeat_msg() {
  heartbeat_interval = atol(msg_buf);
  heartbeat_time = millis();
#if DEBUG
  Serial.print("Heartbeat Interval: ");
  Serial.println(heartbeat_interval);
#endif
}

#if (HAS_LIDAR)
// 's<ms>' is the app asking for obstacle readings at that rate. It sends 100
// once it sees the 's' feature flag.
void process_sonar_msg() {
  long interval = atol(msg_buf);
  if (interval > 0)
    sonar_interval = (unsigned long)interval;
#if DEBUG
  Serial.print("Sonar Interval: ");
  Serial.println(sonar_interval);
#endif
}
#endif

#if (HAS_INDICATORS)
void process_indicator_msg() {
  char *tmp = strtok(msg_buf, ",:");
  indicator_left = atoi(tmp);
  tmp = strtok(NULL, ",:");
  indicator_right = atoi(tmp);
#if DEBUG
  Serial.print("Indicator: ");
  Serial.print(indicator_left);
  Serial.print(",");
  Serial.println(indicator_right);
#endif
}
#endif

#if (HAS_LEDS_FRONT || HAS_LEDS_BACK)
void process_light_msg() {
  char *tmp = strtok(msg_buf, ",:");
  light_front = atoi(tmp);
  tmp = strtok(NULL, ",:");
  light_back = atoi(tmp);
  update_light();
#if DEBUG
  Serial.print("Light: ");
  Serial.print(light_front);
  Serial.print(",");
  Serial.println(light_back);
#endif
}
#endif

#if (HAS_LEDS_STATUS)
void process_notification_msg() {
  char *tmp = strtok(msg_buf, ",:");
  char led = tmp[0];
  tmp = strtok(NULL, ",:");
  int state = atoi(tmp);
  switch (led) {
    case 'y':
      digitalWrite(PIN_LED_Y, state);
      break;
    case 'g':
      digitalWrite(PIN_LED_G, state);
      break;
    case 'b':
      digitalWrite(PIN_LED_B, state);
      break;
  }
}
#endif

// Replies with the parts this car has, so the app only shows readings that mean
// something. No 'v', 'wf' or 'wb': there is no voltage divider or wheel
// encoders fitted.
void process_feature_msg() {
  String msg = "f" + robot_type + ":";
#if HAS_INDICATORS
  msg += "i:";
#endif
#if HAS_LEDS_FRONT
  msg += "lf:";
#endif
#if HAS_LEDS_BACK
  msg += "lb:";
#endif
#if HAS_LEDS_STATUS
  msg += "ls:";
#endif
#if HAS_LIDAR
  // 's' is the sonar/obstacle feature. The LiDAR stands in for the HC-SR04, so
  // it is advertised as one - that is what lights up the FreeRoam obstacle bar
  // and the sonarReading() BotFunction.
  msg += "s:";
  // 'p' is the proximity-octant feature, which the app's object avoidance
  // needs. Only advertised when the LiDAR is fitted - without one there are no
  // sectors to report.
  msg += "p:";
#if LIDAR_SERIAL_SCAN
  // 'x': full scans can come down this serial link on request. The app asks
  // only on a USB connection - a BLE notification is far too small for one.
  msg += "x:";
#endif
#endif
  sendData(msg);
}

//------------------------------------------------------//
// OUTPUT
//------------------------------------------------------//
void sendData(String data) {
  Serial.print(data);
  Serial.println();
#if (HAS_BLUETOOTH)
  if (deviceConnected) {
    char outData[MAX_MSG_SZ] = "";
    for (unsigned int i = 0; i < data.length() && i < MAX_MSG_SZ - 1; i++) {
      outData[i] = data[i];
    }
    pTxCharacteristic->setValue(outData);
    pTxCharacteristic->notify();
  }
#endif
}
