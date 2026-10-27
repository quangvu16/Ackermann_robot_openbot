# OpenBot Ackermann Robot: App Guide

This guide explains the three OpenBot apps for the ESP32-S3 Ackermann car in simple words. Each point lists its **Source** (the file in the OpenBot code that proves it) and an **Example**. Read the **Before you drive** section first.

**Contents**

1. [Before you drive](#1-before-you-drive)
2. [Which app should I use?](#2-which-app-should-i-use)
   - [Home screen and STOP button](#2a-home-screen-and-stop-button-android-and-ios)
   - [RC mode faces](#2b-rc-mode-faces-android-and-ios)
3. [Android app](#3-android-app)
4. [iOS app (beta)](#4-ios-app-beta)
5. [Linux app (Python)](#5-linux-app-python)
6. [Firmware settings you may change](#6-firmware-settings-you-may-change)
7. [Troubleshooting](#7-troubleshooting)
8. [Words used in this guide](#8-words-used-in-this-guide)
9. [Questions to understand deeply](#9-questions-to-understand-deeply)

Pinout: see [ESP32S3_PINOUT.svg](ESP32S3_PINOUT.svg). Wiring, flashing and calibration: see [FIRMWARE_GUIDE.md](FIRMWARE_GUIDE.md).

---

## 1. Before you drive

**The car can run away, so check these first.**

- **Wheels off the ground** for the first test. Only put the car on the floor after the wheels turn the right way.
  - *Source:* `firmware/openbot-esp32s3/openbot-esp32s3.cpp`, section 3 says to widen the throttle range only after the ESC arms and the wheels leave the ground.
  - *Example:* lift the rear wheels on a box, then send a small forward command. The wheels should spin forward.
- **Keep a controller ready.** You must be able to stop the car at any time.
  - *Source:* `robot/android/app/README.md`, Disclaimers.
- **Failsafe.** If no command arrives for 1000 ms, the motors stop and the wheels straighten.
  - *Source:* `firmware/openbot-esp32s3/openbot-esp32s3.cpp`, `CONTROL_TIMEOUT_MS`.
  - *Example:* if the phone app is closed mid-drive, the car stops within about one second.
- **Set your Wi-Fi.** The firmware ships with `"your-ssid"` and `"your-password"` placeholders. Change them, or set `LIDAR_UDP 0` if you do not use Wi-Fi.
  - *Source:* `openbot-esp32s3.cpp`, section 10.

**Why does the first test use blocks?** A wrong steering or throttle direction on the floor can hit people or walls. Blocks let you see the direction safely.

---

## 2. Which app should I use?

| | Android | iOS (beta) | Linux (Python) |
|---|---|---|---|
| **Runs on** | Android phone, Android 7.0 or later | iPhone, build with Xcode | Linux computer with a camera |
| **Connects to the ESP32-S3 by** | **USB cable** | **Bluetooth (BLE)** only | USB serial port |
| **Phone GPS for missions** | Yes | Yes | No. Uses an NMEA GPS receiver |
| **LiDAR plan view** | Over USB cable, no Wi-Fi needed | Needs Wi-Fi (UDP) | Yes, `lidar_view.py` |
| **Object avoidance** | Yes | Yes | Yes |

- *Source:* `robot/README.md` (phone GPS and iPhone sections), `robot/ios/OpenBot/README.md` (Bluetooth section), `robot/linux/README.md`.
- *Example:* you want to test GPS missions with a phone plugged into the car by cable. Use Android. An iPhone cannot use the cable for data, so it would use Bluetooth.

**Why does iPhone use Bluetooth?** iOS does not let apps talk to USB serial chips like the one on the ESP32-S3. This is a limit of the platform, not a bug in OpenBot.

---

## 2a. Home screen and STOP button (Android and iOS)

Both phone apps open on a **Car dashboard** that answers two questions: *is the car ready?* and *what do you want to do?*

| Part | What it shows or does |
|---|---|
| **Connection card** | Coloured dot and plain status ("Car connected (USB)", "Car not connected"), plus one big **Connect** button. Android uses the USB cable; iPhone opens the Bluetooth list. |
| **Status chips** | **LiDAR** (OK / no data / –), **RC** mode (Manual / Auto / Hold / signal lost), **Avoidance** (on / off). Tap the Avoidance chip to switch it. |
| **Three task tiles** | **Drive** (phone or game controller), **GPS Mission** (waypoints, return home), **Car setup** (sensors, lights, avoidance). |
| **More features** | The full OpenBot list (AI, data collection, projects and more). Hidden until you open it; the app remembers your choice. |

**STOP button.** While the car is connected, a big round red **STOP** is always on screen: bottom-right on the home screens, top-right on driving screens (Drive, GPS Mission, Object Tracking and the others), where the bottom belongs to their control panels. Pressing it stops the car and keeps it stopped: every drive command from the app is replaced by "stop", whether it comes from the joystick, a GPS mission, an AI mode or a game controller. The button turns green **RESUME**; press it to drive again.
- *Source:* `vehicle/EmergencyStop.java` and `Vehicle.sendControl()` (Android); `CarDashboard.swift` and `writeWire()` in `BluetoothDataController.swift` (iOS).
- *Example:* during a GPS mission the car heads for a puddle. Press STOP; the mission stays loaded but the car does not move until you press RESUME.
- In **MANUAL** the RC transmitter drives, so the smiling face covers the phone's STOP. Use the transmitter's **HOLD** position to stop there.

**GPS Mission screen.** It now uses the same design as Object Tracking: the map fills the screen and everything else is in one bottom panel with a drag handle.
- The panel's top part always shows: the mission status (and any warning, such as "No GPS signal"), the car connection, and **Start/Pause**, **Stop** and **Return home**.
- The data panel below it can be hidden: drag the panel down (or swipe down on iPhone), tap the handle, or tap **Hide data**. Tap **Show data** to bring it back. The app remembers your choice.
- The data panel holds the readouts (distance and bearing to the next waypoint, speed, GPS accuracy), mission info, the avoidance switch, the planning buttons (add waypoint, clear, save, import/export) and, on Android, the waypoint list.
- *Source:* `res/layout/fragment_gps_navigation.xml` and `setupPanel()` in `GpsNavigationFragment.java` (Android); `setupPanel()` in `GpsMissionController.swift` (iOS).
- *Example:* during a mission, hide the data to see more of the map. The status line still tells you if the car stops for an obstacle or loses GPS.

**Why this layout?** You open the app to *do* something. Showing the car's state and the three main tasks first means no searching through a list of research features before you can drive (Hick's law), and a STOP button in the same place on every screen means you never have to look for it in a hurry.

---

## 2b. RC mode faces (Android and iOS)

When a FlySky RC transmitter is connected to the car (see [FIRMWARE_GUIDE.md](FIRMWARE_GUIDE.md#9-rc-controller-manual-and-auto)), the phone shows which mode the switch is in:

| Switch | Screen |
|---|---|
| **MANUAL** (RC drives) | Full-screen **smiling face** on green |
| **AUTO** (phone drives) | The normal app screen |
| **HOLD** (stopped), or RC link lost | Full-screen **crying face** on grey-blue |

- **Tap** the face to hide it until the mode changes, for example to pause a mission.
- The face goes away by itself if the car stops reporting for 3 seconds (cable pulled, Bluetooth lost, car off).
- *Source:* the firmware sends `m<mode>` once a second (`send_rc_mode()` in `openbot-esp32s3.cpp`); Android draws it in `main/RcModeFaceView.java`, iOS in `Controllers/Lidar/RcModeFace.swift`.
- *Example:* you flip the switch to MANUAL to steer around a puddle. The phone on the car shows a big smile, so people nearby can see a person is driving.

**Why a face?** It is readable from a distance. Anyone near the car can tell at a glance whether a person (smile) is in control or the car is stopped (cry).

---

## 2c. Sensor link: SensorServer over WebSocket (Android and iOS)

The **GPS Mission**, **Object Tracking** and **Autopilot** screens have a **Sensor link**. On Android it is a row in the bottom panel with a **Sensor link** button. On iOS it is an antenna button in the top bar. The link speaks the WebSocket API of the [SensorServer](https://github.com/UmerCodez/SensorServer) app, so tools made for SensorServer work with OpenBot.

| Mode | What it does | Example |
|---|---|---|
| **Off** | Nothing (default). | — |
| **Server** | This phone shares its sensors and GPS. Other devices connect to `ws://<phone address>:8080`. | A laptop logs the car's IMU and GPS while Autopilot drives. |
| **Client** | This phone reads GPS and compass from a SensorServer app on another phone. With **Use its GPS and compass for GPS Mission** turned on, missions drive on them. | A second phone sits high on the car, away from the motors, so its compass is cleaner. |

**Endpoints** (the same in both apps; Server mode serves them, Client mode reads `/gps` and the rotation vector):

```
ws://<address>:8080/sensor/connect?type=android.sensor.accelerometer
ws://<address>:8080/sensors/connect?types=["android.sensor.accelerometer","android.sensor.gyroscope"]
ws://<address>:8080/gps
```

- Sensor messages look like `{"values":[x,y,z],"timestamp":<ns>,"accuracy":3}`. On `/sensors/connect` they also carry `"type"`.
- GPS messages look like `{"latitude":..,"longitude":..,"altitude":..,"bearing":..,"accuracy":..,"speed":..,"time":<ms>}`. Send the text `getLastKnownLocation` to get the last fix at once.
- **iPhone as server:** iOS sensors are published under the Android names, units and signs SensorServer clients expect: accelerometer, gyroscope, magnetic_field, gravity, linear_acceleration, rotation_vector and pressure. Open `http://<iphone address>:8080/` in a browser to see the list.
- **Android as server:** every sensor the phone has, by its Android type name.

**How to use it**
1. Put both devices on the **same Wi-Fi**, or join one phone's **hotspot**.
2. Open GPS Mission, Object Tracking or Autopilot, then tap **Sensor link**.
3. Choose **Server** and note the address on the status line, for example `ws://192.168.43.1:8080`. Or choose **Client** and type the other phone's address.
4. Test from a laptop (Python, `pip install websocket-client`):
   ```python
   import websocket
   ws = websocket.create_connection("ws://192.168.43.1:8080/sensor/connect?type=android.sensor.gyroscope")
   print(ws.recv())
   ```

**Things to know**
- The link runs only while one of the three screens is open, and each sensor runs only while someone is connected to it. That saves battery.
- In Client mode, GPS Mission goes back to the phone's own GPS and compass if the other phone stops sending for **3 seconds**.
- The other phone must be mounted like the car's phone: standing up with its back camera facing forward, or lying flat with its top edge forward. Otherwise its compass heading is wrong.
- The link has no password. Use it only on a Wi-Fi or hotspot you trust.
- *Source:* Android `sensorlink/` (`SensorLinkServer.java` uses Java-WebSocket, MIT; `SensorLinkClient.java` uses OkHttp). iOS `Controllers/GpsNav/SensorLink*.swift`. The protocol is written from SensorServer's documentation; no SensorServer code is copied (SensorServer is GPL-3.0, this project is MIT).

**Why WebSocket?** One small, always-open connection per sensor. It works from Python, a web page, Unity or ROS with no extra software on the phone.

---

## 3. Android app

Source folder: `robot/android/` (the *robot* build). Build details: `robot/android/README.md`.

### 3.1 Build and install

1. Open **Android Studio** and choose *Open*. Select the folder `robot/android`.
2. Choose the **robot** run configuration, then press **Run** with your phone connected and USB debugging on.
   - *Source:* `robot/android/README.md`, Build process.
3. The app's minimum Android version is **Android 7.0 (API 24)** and it compiles against **API 36**.
   - *Source:* `robot/android/app/build.gradle`: `minSdkVersion 24`, `compileSdkVersion 36`.
   - *Note:* `robot/android/README.md` still says API 21 and 33. The code is the newer source of truth.

### 3.2 Connect the phone to the car by USB

1. Plug the phone into the ESP32-S3 with a **USB-C OTG** cable.
2. Open **Robot Info** and turn USB on. The app tries to connect by itself. You can use the USB switch if it does not.
3. The port depends on the Arduino setting **USB CDC On Boot**:
   - **Disabled** → the **COM/UART** port (chip CH343, CP2102 or CH340). Speed 115200.
   - **Enabled** → the **native USB** port. Use this for full 1° LiDAR scans.
   - *Source:* `robot/README.md` (Android + USB cable); `robot/android/app/src/main/res/xml/device_filter.xml` lists the USB devices the app accepts (for example Espressif and CH343/CH9102 chips).
 
**Why does the choice matter?** The COM port is slower. It needs `SERIAL_SCAN_STEP 2` so the LiDAR scan fits. The native port is faster and allows `SERIAL_SCAN_STEP 1`.

**Power note:** the phone can power the ESP32 and the LiDAR through the cable. If the board also gets 5 V from the car's battery converter (BEC), add a diode on USB VBUS so the car cannot push power back into the phone.
- *Source:* `robot/README.md`.

### 3.3 GPS missions with the phone's own GPS

No external GPS is needed. The phone's GPS is used.

- **Mount the phone upright**, back camera facing forward. The app takes "forward" to be the direction the camera looks when the car is still.
  - *Source:* `robot/README.md`; `robot/android/app/src/main/java/org/openbot/app/robot/gpsNav/GpsNavLocationProvider.java`, `headingFromRotationMatrix`.
  - *Example:* mount the phone upright in a holder on the car's body, as the README describes. The app reads a flat phone differently, so do not lay it flat.
- **Keep the phone 15–20 cm away from motors, the ESC and steel.** Magnets and metal disturb the compass.
  - *Source:* `robot/README.md`.
- **Compass warning.** The GPS screen shows **COMPASS UNRELIABLE** when the phone says the compass needs calibration, or when the magnetic field is not Earth-like (Earth is about 25–70 µT).
  - *Source:* `GpsNavLocationProvider.java` (`FIELD_MIN_UT=25`, `FIELD_MAX_UT=70`, `isCompassReliable`); `strings.xml` (`gps_nav_compass_unreliable`).
  - *Example:* a magnet near the phone pushes the field to 150 µT, so the warning appears. Move the phone away and it goes away.
- **Waypoint radius is 4 m.** A waypoint counts as reached within 4 m, because phone GPS is only accurate to about 3–5 m.
  - *Source:* `robot/android/app/src/main/java/org/openbot/app/robot/gpsNav/Waypoint.java`, `DEFAULT_ACCEPTANCE_RADIUS_M = 4.0f`.
  - *Example:* a waypoint 3.5 m away counts as reached; one 4.5 m away does not yet.
- **The mission holds** when GPS accuracy is worse than 15 m or no fix arrives for 3 s. The car stops and waits.
  - *Source:* `robot/README.md`; `GpsNavigationFragment.java` (`FIX_TIMEOUT_MS = 3000`).
- **Mission files.** Import and export `.waypoints` files (QGC WPL 110, the Mission Planner format) with the **Import** and **Export** buttons.
  - *Source:* `robot/android/app/src/main/res/layout/fragment_gps_navigation.xml` (`import_waypoints_button`, `export_waypoints_button`).

### 3.4 LiDAR plan view without Wi-Fi

The firmware sends each LiDAR turn down the USB cable as a text line that starts with `x`. The app asks for these lines when it sees a USB connection.

- *Source:* `robot/android/app/src/main/java/org/openbot/app/robot/vehicle/Vehicle.java` (`requestSerialScans`, every 200 ms); `ControlsFragment.java` (`case 'x'`); `firmware/openbot-esp32s3/lidar_serial.h`.
- *Example:* unplug Wi-Fi, keep the USB cable in, and open the plan view. You should see the car at the centre with the LiDAR points around it.
- *Speed:* with `SERIAL_SCAN_STEP 2` it uses about a fifth of the 115200 line. This is an estimate, not a measured value.

### 3.5 Object avoidance

Avoidance stops the car when something is close in front. The backup-speed slider (0 to 0.5, step 0.05) sets how actively the car backs away from an obstacle.

- *Source:* `RobotInfoFragment.java`; `fragment_robot_info.xml` (`avoid_backup_slider`).
- *Example:* a box is 30 cm ahead. The car stops and, if the backup speed is 0.2, reverses slowly.

---

## 4. iOS app (beta)

Source folder: `robot/ios/OpenBot/`. Build details: `robot/ios/README.md`.

### 4.1 Build

1. Open the project in **Xcode** and choose the **OpenBot** configuration.
2. Run `pod install` in `robot/ios/OpenBot`, then open `OpenBot.xcworkspace` (not the `.xcodeproj`). The iOS app no longer uses **Firebase**, so no Firebase project is needed. Google sign-in (for Google Drive projects) only needs an OAuth client ID, set as `GIDClientID` in `Info.plist`; without one, sign-in is simply off.
3. Use your own Apple ID team for signing (*Automatically manage signing* is on).
   - *Source:* `robot/ios/README.md`, Prerequisites.
   - *Note:* the README lists deployment target iOS 15.5 and also says iOS 13 or later. Check the target in Xcode.

### 4.2 Connect and drive

1. Open **Bluetooth** from the top right of the main screen.
2. Pick your car from the list and press **Connect**. The list only shows cars with matching OpenBot UUIDs.
   - *Source:* `robot/ios/OpenBot/README.md` (Bluetooth Connection).
3. The cable can still power the board. Data goes over Bluetooth.

### 4.3 Missions and GPS

- Missions use the phone GPS, the same as Android. Waypoint radius is 4 m (`defaultAcceptanceRadiusMeters` in `GpsNavCore.swift`).
- The compass warning uses `CLHeading` accuracy and the raw magnetic field (heading error limit 25°, field 25–70 µT).
  - *Source:* `robot/ios/OpenBot/OpenBot/Controllers/GpsNav/GpsNavLocationProvider.swift` (`isCompassReliable`, max 25° heading error, field 25–70 µT).
- The **file menu** on the mission screen shares, opens and imports `.waypoints` files through the iOS share and document pickers.
  - *Source:* `GpsMissionController.swift` (`showFileActions`, `UIDocumentPickerDelegate`); `Info.plist` (`UIFileSharingEnabled`).

### 4.4 Avoidance settings

- **Quick-disable** switch on the free-roam and mission bars. It releases a held command at once.
- **Avoidance** settings sheet on Robot Info: margin 0.3–3.0 m and backup speed 0–0.5.
  - *Source:* `LidarCore.swift` (`AvoidanceSettings`, clamped and saved in `UserDefaults`); `LidarView.swift` (`AvoidanceSettingsController`).

### 4.5 LiDAR plan view

On iPhone the plan view uses Wi-Fi. Put the phone and the ESP32 on the same network and set `LIDAR_UDP_DEST_IP` in the firmware to the phone's IP.

- *Source:* `robot/README.md`; `firmware/openbot-esp32s3/openbot-esp32s3.cpp`, section 10.
- *Why:* one LiDAR turn is too large to send over Bluetooth.

---

## 5. Linux app (Python)

Source folder: `robot/linux/`. Setup and full options: `robot/linux/README.md`.

### 5.1 Setup

```bash
cd robot/linux
python3 -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt
```

- *Source:* `robot/linux/requirements.txt` (pyserial, opencv-python, pygame and others).

### 5.2 Driving modes

`run.py` runs in one of three modes, set with `--run_mode`:

- `debug`: runs the policy on recorded data from `tests/test_data/logs1` instead of the live camera and joystick.
- `inference`: runs the neural network on the live camera. Press **A** on the joystick to switch to joystick mode.
- `joystick`: you drive with a joystick. Press **A** to switch back to inference.

```bash
python run.py --run_mode joystick --control_mode dual
```

- *Source:* `robot/linux/README.md`, Running Robot.
- *Example:* start in `joystick` mode first, so you have full control before turning on a policy.

### 5.3 Object avoidance flags

`--avoid_margin`, `--avoid_backup` and `--no_avoid` tune avoidance. `--lidar_port`, `--no_lidar` and `--lidar_forward_deg` set where the LiDAR scans come from.

- *Source:* `robot/linux/run.py`; `robot/linux/README.md`.

### 5.4 GPS missions

`run_mission.py` drives `MISSION` (a `.waypoints` file in order), `GUIDED` (one point) or `RTL` (return to launch).

```bash
python run_mission.py --goto 47.3977,8.5456 --gps_device /dev/ttyUSB1
python run_mission.py --mission_path missions/field.waypoints \
    --gps_device /dev/ttyUSB1 --export_kml field.kml
```

- *Source:* `robot/linux/README.md`, GPS Mission Navigation.
- *Waypoint radius:* 2.5 m, not 4 m. The Linux app may read an RTK receiver, which is far more accurate. Use `--dry_run` to print the screen without moving the motors.
- *Source:* `robot/linux/gpsnav/guidance.py`; `robot/README.md`.

### 5.5 LiDAR plan view window

```bash
python lidar_view.py          # OpenCV window; press q or Esc to quit
python lidar_view.py --snapshot scan.png   # headless: write one frame
```

- *Source:* `robot/linux/lidar_view.py`; `robot/linux/lidar/plan_view.py`.
- *Example:* gaps in the plot mean "no return" for that angle. They are not zero distance.

### 5.6 Tests

```bash
python3 -m pytest tests
```

- *Source:* `robot/linux/tests/`. The last run passed 135 tests and skipped 1 (the one that needs the full inference stack).

---

## 6. Firmware settings you may change

Source: `firmware/openbot-esp32s3/openbot-esp32s3.cpp`, the settings block at the top. Copy the block into `openbot_settings.h` to keep your changes when the firmware updates.

| Setting | Default | What it does | Example |
|---|---|---|---|
| `ESC_MIN_US` / `ESC_MAX_US` | 1000 / 2000 | Throttle range in microseconds | Lower `ESC_MAX_US` to 1700 to limit top speed |
| `SERVO_TRIM_US` | 0 | Moves the straight-ahead position | Change it by a few µs until the car drives straight |
| `SERVO_REVERSED` | 0 | Flips steering direction | Set 1 if the car steers the wrong way |
| `CONTROL_TIMEOUT_MS` | 1000 | Failsafe time | Keep it. Setting 0 disables the failsafe |
| `STOP_DISTANCE` | 40 cm | Stop distance for something in front | Raise it for faster cars |
| `OBSTACLE_HALF_ANGLE_DEG` | 20 | Width of the front cone | Make it wider in tight spaces |
| `LIDAR_FORWARD_DEG` | 0 | Where the nose sits in the LiDAR frame | Set it if you bolt the LiDAR on rotated |
| `LIDAR_BAUD` / `LIDAR_MEASURE_FREQ` | 115200 / 2300 | LD14 profile | LD14P: 230400 / 4000 |
| `SERIAL_SCAN_STEP` | 2 | Bins merged in the USB scan | 1 on the native USB port |
| `LIDAR_UDP_DEST_IP` / `LIDAR_UDP_PORT` | 192.168.1.100 / 9900 | Where Wi-Fi scans go | Set to your phone or PC's IP |

**Why does the LiDAR profile need both numbers?** The firmware uses `LIDAR_MEASURE_FREQ` to reject corrupt packets. A wrong value rejects good data.

---

## 7. Troubleshooting

| Problem | Likely cause | What to check | Source |
|---|---|---|---|
| Car does not move | No command, or the failsafe stopped it | App connected? Heartbeat running? | `CONTROL_TIMEOUT_MS` in the firmware |
| Car steers the wrong way | Servo direction | Set `SERVO_REVERSED 1` | Firmware section 3 |
| Phone says "connected" but no data | Wrong USB port | Check **USB CDC On Boot** and the port choice | `robot/README.md` |
| Plan view empty over USB | Scan not requested or step too large | Keep USB on; try `SERIAL_SCAN_STEP 2` | `Vehicle.java` (`requestSerialScans`) |
| Mission holds | GPS accuracy above 15 m, or no fix for 3 s | Move outside, away from trees and walls | `robot/README.md` |
| COMPASS UNRELIABLE | Magnets or metal near the phone | Move the phone away; calibrate with a figure-8 | `GpsNavLocationProvider.java` |
| Waypoint never reached | Radius too small for phone GPS | Use waypoints 10 m or more apart | `Waypoint.java` |

---

## 8. Words used in this guide

- **Ackermann steering:** the front wheels turn to different angles, like a normal car. The firmware drives one ESC (throttle) and one servo (steering).
- **ESC:** the electronic speed controller. It sets motor speed.
- **BLE:** Bluetooth Low Energy, the link iPhone uses.
- **NMEA:** the text format GPS receivers send. Used by the Linux app.
- **QGC WPL 110 / `.waypoints`:** a plain text mission file that Mission Planner can read.
- **LiDAR:** a spinning laser that measures distance in all directions. LD14 and LD14P are the models used.
- **Failsafe:** an automatic safety stop when the link is lost.

---

## 9. Questions to understand deeply

Think about each question before you read the answer in the source file. The "why" is the point.

1. **Why does the car stop after one second with no command?** A lost phone should not leave the car driving.
   - *Source:* `CONTROL_TIMEOUT_MS`, firmware section 4.
2. **Why is the waypoint radius 4 m on the phone but 2.5 m on Linux?** What would you need to know about the GPS before you change it?
   - *Source:* `Waypoint.java`, `robot/linux/gpsnav/guidance.py`.
3. **Why does a magnet near the phone make the compass wrong?** How does a field strength check catch this?
   - *Source:* `GpsNavLocationProvider.java`.
4. **Why does the native USB port allow finer LiDAR scans than the COM port?** What limits the speed of the link?
   - *Source:* `robot/README.md`; `firmware/openbot-esp32s3/lidar_serial.h`.
5. **Why does iPhone use Bluetooth, and why can't the plan view use it?** How big is one LiDAR turn in bytes?
   - *Source:* `robot/ios/OpenBot/README.md`; `robot/README.md`.
6. **Why does the firmware keep the `p` channel for avoidance, even when Wi-Fi is off?** What does a car need to stop safely?
   - *Source:* `robot/linux/README.md`, LiDAR and Object Avoidance.
7. **Why should the first test use blocks?** What could happen on the floor that cannot happen on blocks?
   - *Source:* `robot/android/app/README.md`, Disclaimers.

**Why ask these?** Each answer shows what the code protects against. Knowing the reason helps you change a setting safely instead of guessing.
