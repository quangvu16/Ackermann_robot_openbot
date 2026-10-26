# OpenBot Ackermann Robot: Firmware Guide

This guide explains the ESP32-S3 firmware in `firmware/openbot-esp32s3/` in simple words: how to wire the car, flash the board, check that it works, and calibrate it. Each point lists its **Source** (the file that proves it) and an **Example**. Points marked *general practice* come from normal RC and electronics practice, not from the firmware.

![ESP32-S3 pinout and wiring](ESP32S3_PINOUT.svg)

**Contents**

1. [What the firmware does](#1-what-the-firmware-does)
2. [Parts you need](#2-parts-you-need)
3. [Wiring](#3-wiring)
4. [Settings to edit before flashing](#4-settings-to-edit-before-flashing)
5. [Flash the board](#5-flash-the-board)
6. [First power-on checks](#6-first-power-on-checks)
7. [Calibrate throttle and steering](#7-calibrate-throttle-and-steering)
8. [Connect a phone or computer](#8-connect-a-phone-or-computer)
9. [RC controller: manual and auto](#9-rc-controller-manual-and-auto)
10. [Commands the firmware understands](#10-commands-the-firmware-understands)
11. [Safety features](#11-safety-features)
12. [Troubleshooting](#12-troubleshooting)
13. [Questions to understand deeply](#13-questions-to-understand-deeply)

See also: [APPS_GUIDE.md](APPS_GUIDE.md) for the Android, iOS and Linux apps.

---

## 1. What the firmware does

The firmware turns the phone's drive commands into signals for one **ESC** (motor speed) and one **steering servo**. It also reads the **LiDAR**, stops the car before it hits something, and controls the lights.

- **Two app values become throttle and steering.** The app sends a left and a right value, like a tank. The firmware mixes them: `throttle = left + right`, `steering = left − right`.
  - *Source:* `openbot-esp32s3.cpp`, `update_vehicle()`.
  - *Example:* the app sends `c200,200`, so throttle is 400 (forward) and steering is 0 (straight). `c200,0` gives throttle 200 and steering 200, so the car goes forward and turns.
- **When reversing, steering flips,** so the car turns the way the tail points, like a real car.
  - *Source:* `update_vehicle()`, "Reverse the steering when driving backwards".
- **The LiDAR does three jobs:** it acts as the obstacle sensor (`s` channel), it gives 8 direction sectors for object avoidance (`p` channel), and it can send full 360° scans over Wi-Fi (UDP) or the USB cable.
  - *Source:* the header comment at the top of `openbot-esp32s3.cpp`.
- **No extra libraries for the motors.** The 50 Hz pulses come from the ESP32's built-in LEDC hardware.
  - *Source:* `pwm_setup()` and `pwm_write()`; they build on both ESP32 Arduino core 2.x and 3.x.

**Why mix two values into one throttle?** The OpenBot app was made for tank-style robots with a left and right motor. Mixing lets the same app drive a car with steering, unchanged.

---

## 2. Parts you need

| Part | Used for | Source |
|---|---|---|
| ESP32-S3 N16R8 board | The controller | `openbot-esp32s3.cpp`, settings section 1 and 5 |
| ESC (electronic speed controller) | One throttle line for all four motors | header comment |
| Steering servo | Front-axle steering | header comment |
| LDROBOT LD14 or LD14P LiDAR | Obstacles and scans | settings section 6–7 |
| LEDs + resistors | 2 indicators, 2 front, 2 rear, 3 status (yellow, green, blue) | settings section 5 |
| FlySky FS-i6 + FS-iA6B receiver (optional) | MANUAL / AUTO / HOLD switch | settings section 12 |
| 5 V supply | Board and LiDAR (about 200 mA for the LiDAR) | `lidar_ld14.h` wiring comment |
| USB-C OTG cable (Android) | Phone link and power | `robot/README.md` |

There are no wheel encoders and no battery-voltage sensor on this car, so the app does not show those readings.
- *Source:* header comment and `process_feature_msg()`.

---

## 3. Wiring

The full map is in [ESP32S3_PINOUT.svg](ESP32S3_PINOUT.svg). The map uses GPIO numbers. Match them to the labels printed on your own board.

| GPIO | Connects to | Signal | Source |
|---|---|---|---|
| 1 | ESC signal wire | 50 Hz pulse, 1000–2000 µs, 1500 µs = stop | `PIN_ESC`, section 3 and 5 |
| 2 | Steering servo signal wire | 50 Hz pulse, 1500 µs = straight | `PIN_SERVO` |
| 4 / 5 | Left / right turn indicator LED | on/off, blinks every 500 ms | `PIN_LED_LI`, `PIN_LED_RI`, `indicator_interval` |
| 6 / 7 | Left / right front light LED | brightness 0–255 (PWM) | `PIN_LED_LF`, `PIN_LED_RF`, `update_light()` |
| 8 / 9 | Left / right rear light LED | brightness 0–255 (PWM) | `PIN_LED_LB`, `PIN_LED_RB` |
| 10 / 11 / 12 | Status LED yellow / green / blue | on/off, set by the app | `PIN_LED_Y/G/B`, `process_notification_msg()` |
| 13 | LiDAR **TX** wire | UART1 input | `LIDAR_RX_PIN`, `lidar_ld14.h` |
| 14 | RC receiver iBUS signal (or CH1 in PWM mode) | UART2 input | `RC_IBUS_PIN`, section 9 |
| 15 / 16 | RC CH2 / CH5 (PWM mode only) | pulse input | `RC_PWM_PIN_*`, section 9 |

**LiDAR (4 wires):**
- LiDAR TX → GPIO 13. It is 3.3 V logic, so no level shifter is needed.
- LiDAR RX → not connected. The LiDAR runs by itself and needs no commands (`LIDAR_TX_PIN -1`).
- LiDAR GND → GND, shared with the ESP32.
- LiDAR VCC → the 5 V rail. **Not** the 3V3 pin: it needs about 200 mA.
- *Source:* `firmware/openbot-esp32s3/lidar_ld14.h`, wiring comment.

**Ground and power:**
- Join all grounds: ESP32, ESC, servo, LiDAR and LEDs. A signal only works when both sides share ground. *(general practice; the firmware states it for the LiDAR)*
- Power the servo from the ESC's BEC or a separate 5–6 V supply, not from the ESP32. *(general practice)*
- If the phone powers the board through USB **and** the car's BEC also feeds 5 V, put a diode on USB VBUS so the car cannot push power into the phone.
  - *Source:* `robot/README.md`.

**LEDs are active-high:** GPIO → resistor → LED → GND. A 220–330 Ω resistor suits most LEDs. *(resistor value: general practice)*
- *Source:* `setup()` turns LEDs on with `digitalWrite(..., HIGH)`.

**Pins you must not use:**
- GPIO 26–37: wired to the flash and octal PSRAM inside the N16R8 chip. Using them makes the board unbootable.
- GPIO 0, 3, 45, 46: strapping pins that decide how the chip boots.
- GPIO 19/20 (native USB) and 43/44 (UART0): the phone link.
- *Source:* `openbot-esp32s3.cpp`, settings section 5 (quotes the ESP32-S3 datasheet v2.2, Table 2-14).
- *Example:* if you add a buzzer, use a free pin such as GPIO 14, and not GPIO 35.

**Why so many reserved pins?** The N16R8 has 8 MB of fast *octal* PSRAM inside the chip. Octal means 8 data lines, which use up GPIO 33–37 on top of the flash pins 26–32.

---

## 4. Settings to edit before flashing

All settings are in the **SETTINGS** block at the top of `openbot-esp32s3.cpp`.

**Tip:** copy that block into a new file `openbot_settings.h` next to the sketch. The firmware uses that file instead, so your settings survive firmware updates.
- *Source:* settings block, opening comment (`__has_include("openbot_settings.h")`).

The settings you most likely need:

| Setting | Default | Change it when | Source |
|---|---|---|---|
| `WIFI_SSID`, `WIFI_PASS` | `"your-ssid"`, `"your-password"` | You use the Wi-Fi scan stream | section 10 |
| `LIDAR_UDP` | 1 | Set 0 to build without Wi-Fi | section 10 |
| `LIDAR_UDP_DEST_IP` | `192.168.1.100` | Set to the phone or PC that shows the scans | section 10 |
| `LIDAR_BAUD`, `LIDAR_MEASURE_FREQ` | 115200, 2300 | LD14P: 230400, 4000 | section 7 |
| `LIDAR_FORWARD_DEG` | 0 | The LiDAR's zero mark does not face forward | section 9 |
| `STOP_DISTANCE` | 40 cm | A faster car needs more room to stop | section 9 |
| `SERIAL_SCAN_STEP` | 2 | Set 1 on the native USB port | section 10b |
| `SERVO_REVERSED` | 0 | The car steers the wrong way | section 3 |
| `HAS_...` | 1 | A part is not fitted on your car | section 11 |
| `HAS_RC`, `RC_PROTOCOL` | 1, `RC_IBUS` | `RC_PWM` for an FS-iA6 receiver; see section 9 | section 12 |
| `RC_MODE_LOW/MID/HIGH` | MANUAL / AUTO / HOLD | What each switch position does | section 12 |
| `RC_LOST_ACTION` | 0 (stop) | 1 = keep driving in AUTO when the radio drops | section 12 |

- *Example:* your LiDAR is mounted turned 90° clockwise. Set `LIDAR_FORWARD_DEG` to the angle in the LiDAR's own clockwise frame where the car's nose sits, then check on the plan view that "front" is really in front.

**Why must both LiDAR numbers match the model?** The firmware uses `LIDAR_MEASURE_FREQ` to throw away corrupt packets. A wrong value can throw away good data. A wrong baud rate fixes itself, because the firmware tries `CANDIDATE_BAUD` rates when the LiDAR is silent.
- *Source:* settings section 7.

---

## 5. Flash the board

1. Install the **Arduino IDE** and the **esp32** board package by Espressif (Boards Manager). The code builds on core 2.x and 3.x. *(Source: LEDC helper comment.)*
2. Open `firmware/openbot-esp32s3/openbot-esp32s3.ino`. The real code is in the `.cpp` next to it, which the IDE compiles too.
   - *Source:* `openbot-esp32s3.ino` comment.
3. In **Tools**, choose:
   - **Board:** *ESP32S3 Dev Module*. *(Source: the `#error` message in `openbot-esp32s3.cpp`.)*
   - **Flash Size:** 16 MB, and **PSRAM:** *OPI PSRAM*, to match the N16R8's 16 MB flash and octal PSRAM. *(Menu names: Arduino esp32 core.)*
   - **USB CDC On Boot:** *Disabled* if the phone plugs into the **COM/UART** port, *Enabled* for the **native USB** port.
     - *Source:* `robot/README.md`.
   - **Partition Scheme:** if the build says the sketch is too big (BLE plus Wi-Fi is large), choose a scheme with a bigger app partition. *(general practice)*
4. Plug the board into the computer, choose its port, and click **Upload**.

**Why does USB CDC On Boot matter?** It decides which physical USB socket the firmware's `Serial` uses. The phone must be plugged into that socket, or it can't talk to the car.

---

## 6. First power-on checks

Do this with **the wheels off the ground**. Open the **Serial Monitor** at **115200** baud.

| You should see | What it proves | Source |
|---|---|---|
| Left indicator blinks, then the right one | GPIO 4 and 5 are wired right | `setup()`, indicator test sequence |
| `OpenBot RC_S3 (ESP32-S3): ESC throttle, Ackermann steering servo, LD14 LiDAR, LEDs` | The firmware started | `setup()` |
| `LiDAR  UART1 @ 115200 baud, RX=GPIO13 TX=-1 (-1 = unwired)` | The LiDAR port opened | `lidar_ld14.h` |
| `LiDAR SN ...  hw ...  fw ...` | The LiDAR is talking | `lidar_device_info_callback()` |
| `Waiting a client connection to notify...` | Bluetooth is advertising as "OpenBot" | `setup_ble()` |
| The ESC beeps and arms | It saw neutral (1500 µs) at boot | `stop_vehicle()` comment |

- *Example:* you see `nothing at all on GPIO13 - not a single byte received`. That means the LiDAR sends nothing: check its power, then swap its two signal wires (see Troubleshooting).

---

## 7. Calibrate throttle and steering

All values are pulse widths in microseconds (µs). 1500 µs is neutral for both.

1. **Steering centre:** send no command. If the wheels are not straight, change `SERVO_TRIM_US` a few µs at a time until they are.
2. **Steering direction:** steer left in the app. If the wheels turn right, set `SERVO_REVERSED 1`.
3. **Steering limits:** if the servo buzzes or strains at full lock, move `SERVO_MIN_US` and `SERVO_MAX_US` closer to 1500.
4. **Throttle:** keep `ESC_MIN_US 1000`, `ESC_NEUTRAL_US 1500`, `ESC_MAX_US 2000` until the ESC arms and the wheels spin the right way. To limit top speed, lower `ESC_MAX_US` (for example to 1700).

- *Source:* settings section 3. It warns: "an off-road car on full throttle will run away".
- *Example:* the car drifts slightly right when driving straight. Move `SERVO_TRIM_US` by 10 µs, test, and repeat.

**Why start with the full 1000–2000 µs range?** Many ESCs learn their range from the first pulses they see. A full, standard range lets the ESC arm. Speed limits are safer to add afterwards.

---

## 8. Connect a phone or computer

| Link | Who uses it | How | Source |
|---|---|---|---|
| USB cable, 115200 baud | Android app, Linux app | Turn on USB in the app's Robot Info | `Serial.begin(115200)`, `robot/README.md` |
| Bluetooth LE, name **OpenBot** | iOS app (and Android) | Scan and connect in the app | `setup_ble()` |
| Wi-Fi UDP, port 9900 | LiDAR plan view on iPhone, ROS 2 | Set `WIFI_*` and `LIDAR_UDP_DEST_IP` | settings section 10 |

The app finds the car by its Bluetooth service UUID `61653dc3-4021-4d1e-ba83-8b4eec61d613`. The app and firmware must use the same UUIDs.
- *Source:* `SERVICE_UUID` in `openbot-esp32s3.cpp`.

---

## 9. RC controller: manual and auto

A FlySky FS-i6 transmitter can take over the car. A 3-position switch picks who drives:

| Switch SwC (CH5) | Mode | Who drives |
|---|---|---|
| Up (about 1000 µs) | **MANUAL** | Your RC sticks. App drive commands are ignored; lights still work. |
| Middle (about 1500 µs) | **AUTO** | The phone or computer, exactly as without a receiver. |
| Down (about 2000 µs) | **HOLD** | Nobody. Throttle neutral, wheels straight. |

- *Source:* `firmware/openbot-esp32s3/rc_input.h` and settings section 12 in `openbot-esp32s3.cpp`.
- *Example:* the phone is driving a GPS mission and heads for a ditch. Flip SwC up and steer away with the sticks. Flip it to the middle and the phone drives again.

**Why a 3-position switch?** Two positions would give MANUAL and AUTO only. The third, HOLD, is a quick stop you can reach with one finger, whoever is driving.

### 9.1 Wiring

**FS-iA6B receiver (has iBUS, recommended):** one cable from its **SERVO** iBUS port (not the **SENS** port).

| Receiver SERVO port | ESP32-S3 |
|---|---|
| Signal | GPIO 14 |
| + | 5 V rail |
| − | GND |

**FS-iA6 receiver (no iBUS):** one signal wire per channel, and set `RC_PROTOCOL RC_PWM`.

| Receiver | ESP32-S3 |
|---|---|
| CH1 signal (steering) | GPIO 14 |
| CH2 signal (throttle) | GPIO 15 |
| CH5 signal (mode switch) | GPIO 16 |
| + and − of any channel | 5 V and GND |

- *Source:* settings section 12 (`RC_IBUS_PIN`, `RC_PWM_PIN_*`); the map is in [ESP32S3_PINOUT.svg](ESP32S3_PINOUT.svg).
- iBUS uses the ESP32's third UART (UART2). UART0 is the USB link to the phone and UART1 is the LiDAR.
- Many projects wire FlySky receiver signals straight to 3.3 V ESP32 pins. A 1 kΩ resistor in series on each signal wire adds protection. *(general practice)*
- The firmware refuses to build if you set an RC pin to a reserved one (26–37, 0, 3, 45, 46, 19, 20, 43, 44).

**Why iBUS?** One wire carries all channels, and every frame has a checksum, so a noisy wire gives "no data" instead of a wrong stick position.

### 9.2 Set up the FS-i6 transmitter

Menu names are from the FS-i6 manual and can differ slightly between firmware versions.

1. **Pick the sticks.** The firmware steers with **CH1** and drives with **CH2**. On a mode-2 FS-i6 both are the **right stick**, which springs back to centre on both axes. Do **not** use the left stick (CH3) for throttle: it has no spring, so the car would keep driving when you let go.
2. **Put SwC on CH5.** Hold **OK** → *Functions setup* → *Aux. channels* → *Channel 5* → **SwC**. Hold **CANCEL** to save.
3. **Turn on iBUS (FS-iA6B only).** *System setup* → *Output mode* → *Serial*: **i-BUS**. It is usually on already.
4. **Set the failsafe. This matters most.** *Functions setup* → *Failsafe*. Set **CH5 = SwC down (HOLD)** and **CH2 = stick centre**, then save. Without this, a FlySky receiver may keep sending the last stick positions after the radio link drops.
5. **Fix directions.** If steering or throttle runs the wrong way in MANUAL, use *Functions setup* → *Reverse* for CH1 or CH2.

- *Source:* channel choice and failsafe reason are in settings section 12 and the `rc_input.h` header comment.
- *Example:* if you prefer a 2-position switch (SwA, SwB or SwD), put it on CH5 and set `RC_MODE_HIGH RC_AUTO`. Then up = MANUAL and down = AUTO.

### 9.3 What happens if the radio link drops

| Situation | What the firmware does | Source |
|---|---|---|
| No receiver heard since power-on | Acts as if none is fitted: the app drives | `rcDecideMode()` |
| Receiver heard, then no valid frame for 250 ms | **Stops** (mode "LOST") | `RC_TIMEOUT_MS`, `RC_LOST_ACTION 0` |
| Transmitter failsafe set as in 9.2 | Receiver sends CH5 = down, so the car **holds** | your transmitter |
| `RC_LOST_ACTION 1` | Keeps driving **only if** it was in AUTO. MANUAL always stops | `rcDecideMode()` |

**Why stop even in AUTO?** If you can't reach the car by radio, you can't take over. Stopping is the safe default. Use `RC_LOST_ACTION 1` only if the phone must finish missions beyond radio range.

### 9.4 Good to know

- **In MANUAL the LiDAR still blocks forward driving** into something closer than `STOP_DISTANCE`. Reverse is allowed. Set `RC_MANUAL_OBSTACLE_STOP 0` to turn this off.
  - *Source:* `drive_from_rc()`.
- **In MANUAL the steering stick works like a steering wheel,** also in reverse. (The app path flips steering in reverse; the RC path does not.)
  - *Source:* `drive_from_rc()` comment.
- **A centred stick really stops the car:** anything within ±30 µs of centre counts as zero (`RC_DEADBAND_US`).
- **Every mode change clears the last app command.** Back in AUTO, the car waits for a fresh command from the app.
  - *Source:* the "Who drives" block in `loop()`.
- **The app does not yet know about MANUAL.** A GPS mission keeps running on the phone while you drive by hand, and it takes over again when you flip back to AUTO. Pause the mission first if you don't want that.

### 9.5 Test it on blocks

1. Power on. The Serial Monitor (115200) shows `RC     iBUS on GPIO14: CH1 steering, CH2 throttle, CH5 mode switch`.
2. Flip SwC through all three positions. You should see `RC mode: MANUAL`, `RC mode: AUTO` and `RC mode: HOLD`.
3. In **MANUAL**, the sticks move the wheels and the app's joystick does nothing.
4. In **AUTO**, the app drives and the sticks do nothing.
5. **Failsafe test:** in MANUAL, hold a little throttle, then switch the transmitter **off**. The wheels must stop within a quarter of a second, and the monitor shows `RC mode: HOLD` or `RC mode: LOST (failsafe)`. If the wheels keep turning, set the failsafe (9.2 step 4) before driving on the ground.

---

## 10. Commands the firmware understands

Each message is one text line ending in a newline. The first letter says what it is.
- *Source:* the "Protocol" comment in `openbot-esp32s3.cpp`.

**App → car**

| Message | Meaning | Example |
|---|---|---|
| `c<left>,<right>` | Drive, each −255 to 255 | `c150,150` = forward |
| `i<left>,<right>` | Turn indicators, 0 or 1 | `i1,0` = left indicator on |
| `l<front>,<back>` | Light brightness, 0–255 | `l255,80` |
| `n<led>,<state>` | Status LED `y`, `g` or `b` | `ng,1` = green on |
| `h<ms>` | Heartbeat (failsafe) time | `h750` |
| `s<ms>` | How often to report obstacles | `s100` |
| `f` | Ask which parts the car has | `f` |
| `x<ms>` | Send LiDAR scans on this USB link, 0 = off | `x200` |

**Car → app**

| Message | Meaning |
|---|---|
| `fRC_S3:i:lf:lb:ls:s:p:x:` | The car's parts: indicators, front/back lights, status LEDs, obstacle sensor, proximity sectors, USB scans |
| `s<cm>` | Nearest obstacle in the front cone, capped at 300 cm |
| `p<d0>,…,<d7>` | Distance in 8 directions, sector 0 = front, then clockwise every 45°; 0 = nothing in range |
| `x<base64>` | One full LiDAR turn, only on USB after `x<ms>` |

- *Example:* you can test the board from the Arduino Serial Monitor (115200, newline on). Type `f` and the car replies with its feature line. Type `i1,0` and the left indicator blinks. Keep the wheels off the ground before you try `c`.

---

## 11. Safety features

- **Failsafe from power-on.** If no command arrives for `CONTROL_TIMEOUT_MS` (1000 ms), throttle goes to neutral and the wheels straighten. It is active from boot, unlike the original OpenBot firmware.
  - *Source:* heartbeat comment and `stop_vehicle()`.
  - *Example:* the phone's battery dies mid-drive; the car stops within about a second.
- **Stop at obstacle.** If something in the front cone (±20°) is closer than `STOP_DISTANCE` (40 cm), forward driving is blocked. **Reversing is allowed**, so you can back away.
  - *Source:* `stop_at_obstacle()`.
- **No fake "all clear".** If the LiDAR has no fresh scan, the firmware sends **no** `p` line instead of zeros. Zeros would mean "clear road" and could drive the car into what broke the LiDAR. Silence lets the app's staleness check stop the car.
  - *Source:* `send_proximity_reading()` comment.
- **The LiDAR has its own task.** It reads the LiDAR on a separate FreeRTOS task, so Bluetooth or Wi-Fi work can never block it. A late scan costs one turn; a late command could stop the car.
  - *Source:* `setup()`, `xTaskCreatePinnedToCore(lidarTask, ...)`.
- **The network never blocks driving.** Wi-Fi connects in the background, and a scan that can't be sent is dropped, not queued.
  - *Source:* `loop()` comment above `wifiLoop()`.

**Why block only forward driving?** When a car is close to a wall, backing away is the safe move. Blocking reverse too would trap it.

---

## 12. Troubleshooting

| Problem | What to check | Source |
|---|---|---|
| `nothing at all on GPIO13` | 1. Is the LiDAR motor spinning? If not, it is a power problem: it needs 5 V and about 200 mA. 2. LiDAR **TX** must go to GPIO 13; try swapping the two signal wires. 3. LiDAR GND and ESP32 GND must be connected. 4. GPIO 13 must not be a reserved pin. | `diagnoseSilentLidar()` in `lidar_ld14.h` |
| Board does not boot after wiring | Something is on GPIO 26–37 or a strapping pin | settings section 5 |
| ESC does not arm | It must see 1500 µs at power-on; check the signal and ground wires | `stop_vehicle()` |
| Car steers the wrong way | `SERVO_REVERSED 1` | settings section 3 |
| Car stops on its own every second | No heartbeat or commands arrive; check the app link | heartbeat comment |
| Phone connects but no data | Wrong USB socket for your **USB CDC On Boot** setting | `robot/README.md` |
| RC sticks do nothing | Mode is not MANUAL, or the receiver is not heard; check the Serial Monitor for `RC mode:` lines and use the SERVO port, not SENS | section 9 |
| Car keeps driving after the transmitter is off | Receiver holds the last values; set the transmitter failsafe | section 9.2 |
| Build error: "Select an ESP32-S3 board" | Choose *ESP32S3 Dev Module* in Tools → Board | `#error` in `openbot-esp32s3.cpp` |
| Build error: "openbot_settings.h must set OPENBOT_BOARD" | Your settings file is missing `#define OPENBOT_BOARD ESP32S3` | settings block |

---

## 13. Questions to understand deeply

Try to answer each one before reading the source.

1. **Why does the ESC need 1500 µs at power-on?** What would happen if it saw full throttle first? *(`stop_vehicle()`)*
2. **Why does steering flip when reversing?** Picture a real car backing into a parking space. *(`update_vehicle()`)*
3. **Why send nothing on `p` when the LiDAR fails, instead of zeros?** What does a zero mean to the app? *(`send_proximity_reading()`)*
4. **Why does the LiDAR get its own task on core 0?** What is worse: a late scan or a late drive command? *(`setup()`)*
5. **Why are GPIO 26–37 forbidden on the N16R8 but not on every ESP32-S3?** What changes with octal PSRAM? *(settings section 5)*
6. **Why is the obstacle distance capped at 300 cm when the LiDAR sees 8 m?** What draws that value in the app? *(`MAX_SONAR_DISTANCE`)*
7. **Why does the RC stick steer the same way in reverse, while the app flips it?** Which feels right with a steering wheel, and which with a tank joystick? *(`drive_from_rc()`, `update_vehicle()`)*
8. **Why should losing the radio stop the car even when the phone is driving?** *(section 9.3)*
9. **Why keep your settings in `openbot_settings.h`?** What happens to edits in the `.cpp` when you update the firmware? *(settings block comment)*

**Why ask these?** Each answer explains a safety or design choice. Knowing the reason lets you change the car without breaking what protects it.

---

*Verification:* the firmware (including the RC code) compiles with the ESP32 Arduino core 3.3.12 for an ESP32-S3 with 16 MB flash and OPI PSRAM, in iBUS, PWM and no-receiver builds. The RC decoding and mode logic passed 25 host tests. It has not been tested on real hardware: test on blocks first.
