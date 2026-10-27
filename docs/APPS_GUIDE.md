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
| **Three task tiles** | **Drive** (phone or game controller), **GPS Mission** (waypoints, return home), **Car setup** (what the car has, live readings, tests, LiDAR, avoidance). |
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
| **Server** | This phone shares its sensors and GPS. Open `http://<phone address>:8080` in any browser to watch them live, or connect a script to `ws://<phone address>:8080`. | A laptop browser shows the car's GPS, compass heading and IMU graphs while Autopilot drives. |
| **Client** | This phone reads GPS and compass from a SensorServer app on another phone. With **Use its GPS and compass for GPS Mission** turned on, missions drive on them. | A second phone sits high on the car, away from the motors, so its compass is cleaner. |

**Endpoints** (the same in both apps; Server mode serves them, Client mode reads `/gps` and the rotation vector):

```
ws://<address>:8080/sensor/connect?type=android.sensor.accelerometer
ws://<address>:8080/sensors/connect?types=["android.sensor.accelerometer","android.sensor.gyroscope"]
ws://<address>:8080/gps
```

- Sensor messages look like `{"values":[x,y,z],"timestamp":<ns>,"accuracy":3}`. On `/sensors/connect` they also carry `"type"`.
- GPS messages look like `{"latitude":..,"longitude":..,"altitude":..,"bearing":..,"accuracy":..,"speed":..,"time":<ms>}`. Send the text `getLastKnownLocation` to get the last fix at once.
- **iPhone as server:** iOS sensors are published under the Android names, units and signs SensorServer clients expect: accelerometer, gyroscope, magnetic_field, gravity, linear_acceleration, rotation_vector and pressure.
- `GET /sensors` returns the list as JSON, for example `[{"type":"android.sensor.gyroscope","name":"Gyroscope"}]`.
- **Android as server:** every sensor the phone has, by its Android type name.

**How to use it**
1. Put both devices on the **same Wi-Fi**, or join one phone's **hotspot**.
2. Open GPS Mission, Object Tracking or Autopilot, then tap **Sensor link**.
3. Choose **Server** and note the address on the status line, for example `http://192.168.43.1:8080`. Or choose **Client** and type the other phone's address.
4. **Browser:** open that address on a laptop, tablet or another phone on the same Wi-Fi. The page shows:
   - a **GPS** card (latitude, longitude, accuracy, speed, course, time since the last fix, and an *Open map* link);
   - one card per sensor with a switch. Turned on, it shows live values, a graph of the last 3 seconds and the readings per second. The rotation vector card also shows the compass heading;
   - a **Pause** button, to freeze the numbers while you read them.
   - a **Camera** card: the phone camera live, with a **Snapshot** link (saves a JPEG) and a switch to turn the view off;
   - a **LiDAR** card: a top-down view with the car in the middle, nose up, and 1 m rings. Blue dots are the scan; coloured wedges are the car's 8 proximity sectors (red under 0.5 m, amber under 1 m, green further). Beside it: nearest obstacle, distance straight ahead, points per scan, scan age, and a 2 / 4 / 8 m range switch;
   - a **Settings** card that changes the phone from the browser. **Driving** comes first: a big **STOP / RESUME** button, speed mode, object avoidance on/off and its distance. Then **Camera** (front/back lens, flashlight, zoom, stream quality, frames per second), the screen's own settings (Object Tracking: detection confidence, slow down when close) and **Sensors** (sensor speed).
   The page needs no internet: the phone serves it. Your switch choices are remembered in that browser.

   *Example:* the car drives an Autopilot run in the garden. On the laptop you watch the camera, lower the speed to *slow* and switch the flashlight on as it gets dark. If it heads for the flower bed, you press **STOP** in the browser.

   **Camera per screen:** Object Tracking and Autopilot share the camera they already use. GPS Mission has no camera on screen, so it opens one only while a browser watches and closes it when you turn the view off or close the page. That saves battery.
5. **Script:** test from a laptop (Python, `pip install websocket-client`):
   ```python
   import websocket
   ws = websocket.create_connection("ws://192.168.43.1:8080/sensor/connect?type=android.sensor.gyroscope")
   print(ws.recv())
   ```

**Things to know**
- The link runs only while one of the three screens is open, and each sensor runs only while someone is connected to it. That saves battery.
- **LiDAR** on `WS /lidar`: one message per revolution (about 6–11 per second), and a refresh every second even with no LiDAR: `{"scanId":42,"bins":360,"forwardOffset":0,"age":35,"ranges":[mm,...],"sectors":[mm x8],"sectorsAge":20}`. Bin *i* is (*i* + 0.5) × 360 / bins degrees clockwise from the LiDAR's zero mark; 0 mm means no return. Full scans reach the phone over **USB** (the `x` line) or the **car's Wi-Fi** (UDP port 9900). Over Bluetooth only the 8 `sectors` arrive, so `ranges` is empty.
- More endpoints: `GET /camera.mjpg` (live MJPEG; works in an `<img>` tag, VLC or OpenCV `cv2.VideoCapture`), `GET /camera.jpg` (one picture), `WS /control` (settings; the phone sends `{"screen","settings":[...],"camera"}`, the page sends `{"set":"drive.speed","value":"slow"}`).
- **Why MJPEG?** Every browser shows it with a plain `<img>`, with no video player or codec. It uses more data than a video codec, so lower *Stream quality* or *frames per second* on a weak hotspot.
- In Client mode, GPS Mission goes back to the phone's own GPS and compass if the other phone stops sending for **3 seconds**.
- The other phone must be mounted like the car's phone: standing up with its back camera facing forward, or lying flat with its top edge forward. Otherwise its compass heading is wrong.
- The link has no password. Use it only on a Wi-Fi or hotspot you trust.
- *Source:* Android `sensorlink/` (`MiniWebServer.java` is the built-in HTTP + WebSocket server, `SensorLinkClient.java` uses OkHttp, the page is `assets/sensorlink/index.html`). iOS `Controllers/GpsNav/SensorLink*.swift` and `SensorLinkDashboard.html` (CI checks the two pages are identical). The protocol is written from SensorServer's documentation; no SensorServer code is copied (SensorServer is GPL-3.0, this project is MIT).

**Why WebSocket?** One small, always-open connection per sensor. It works from Python, a web page, Unity or ROS with no extra software on the phone.

---

## 2d. LiDAR robot: patrol, follow, avoid, track, warn (Android and iOS)

Open it from the **LiDAR robot** tile on the home screen. The screen shows a **face** beside the **LiDAR map**, then the five behaviours, a big **Start / Stop** button and three sliders (speed, distance, angle). The behaviours come from the hoverboard robot's ROS 2 package `hoverboard_laser` (github.com/quangvu16/hoverboard-robot, `src/hoverboard_laser`: `laser_Avoidance`, `laser_Tracker`, `laser_Warning`). That package has no patrol node, so Patrol and Tracking were designed here on the same helpers.

| Behaviour | What the car does | Face | Defaults |
|---|---|---|---|
| **Patrol** | Drives down the middle between walls. At a wall it turns round (back up, then forward, repeated) towards the side with more room, then carries on. | Focused blue eyes; worried at a wall | 35 % speed, turns at 0.9 m (1.5 × 0.6 m) |
| **Follow** | Follows the nearest object in front, keeping the set distance: forward when it's far, backs off when it's too close (as `laser_Tracker`). | **Pink heart eyes** and a smile; searching eyes when there's nothing to follow | keeps 0.55 m, looks ±30° |
| **Avoid** | Wanders and steers away from obstacles using front / left / right sectors (as `laser_Avoidance`). It backs out when boxed in. | Smiling when clear; worried amber eyes when turning away; scared red when boxed in | 35 % speed, reacts inside 0.8 m |
| **Track** | Turns to face the nearest object without chasing it, creeping forward with the wheels turned until the object is straight ahead. | Focused eyes that look at the target | looks ±90°, never closer than 0.4 m |
| **Warn** | Stays still. When something comes within the distance in front, the phone beeps and vibrates and the wheels point at it (as `laser_Warning`, without the buzzer). | Calm while watching; **wide red eyes and an open mouth** at the warning distance | 0.5 m, ±20° |

- **The eyes look where the LiDAR is looking:** at the person being followed, the obstacle being avoided, the target being tracked.
- **Ackermann, not hoverboard:** the hoverboard turns on the spot; this car can't. Wherever the original spins, this one backs out with the nose swinging the chosen way, or creeps forward with the wheels turned.
- **Safety:** every command goes through the same path as the other screens, so the **STOP** button and **object avoidance** still apply. If object avoidance is on with a 1 m margin, Follow will hold at 1 m rather than 0.55 m. If the LiDAR stops sending (more than 0.5 s), the car stops and the face shows **X eyes**. Leaving the screen stops the car. **Run each behaviour with the wheels off the ground first.**
- **LiDAR source:** a full scan over USB (Android) or the car's Wi-Fi works best. Over Bluetooth only the car's 8 sectors arrive; Avoid and Warn still work, but Follow and Track get coarse (45° steps).
- *Example:* choose **Follow**, press Start and walk slowly in front of the car. The face shows hearts and the car keeps about half a metre behind you. Stop walking and it holds; step towards it and it backs off.
- *Source:* Android `lidarbot/` (`LidarBehaviors.java`, `RobotFaceView.java`, `LidarBotFragment.java`, 9 unit tests). iOS `Controllers/LidarBot/` (`LidarBehaviors.swift`, `LidarBotController.swift`).

**Why a face?** A person near the car can't read a status line, but they can see at a glance whether the car is happy, following them, worried or alarmed.

---

## 2e. Indoor SLAM and navigation (Android and iOS)

Open it from **More features > General > Indoor SLAM**. The car draws a map of the room with its LiDAR (**SLAM** = Simultaneous Localization And Mapping: it builds the map and works out where it is on that map at the same time). Then you **tap the map** and the car drives there on its own, going round walls and furniture.

| Control | What it does |
|---|---|
| **Build map / Localize only** | *Build map* adds every scan to the map. *Localize only* keeps the map fixed and only works out where the car is (use it after **Load map**). |
| **Explore** | The car wanders by itself (the LiDAR robot's Avoid behaviour) while the map grows. You can also drive by RC transmitter or game controller. |
| **Tap** the map | Sets a goal (pink dot). The car plans a path (green line) and follows it. **Cancel goal** stops it. |
| **Long-press** the map | "The car is here, facing this way": use it if **Find me** fails or the map slips. |
| **Find me** | Finds the car on the map with no hint: it searches the whole map and every heading. It runs by itself after **Load map** and when tracking is lost for 3 s. |
| **Double-tap** / **Centre** | Keeps the view on the car. Pinch to zoom, drag to pan. |
| Layers **Map / Scan / Path / Trail** | Show or hide the grid, the live scan (red dots), the planned path and where the car has been. |
| **Save / Load / Clear map** | Keeps one map on the phone, so you don't have to map the flat every time. |
| **Export for ROS** | Shares `map.pgm` + `map.yaml`, the same files `map_saver` writes. They open in ROS 2 `map_server`, Nav2 and RViz. |

The status line shows the state, the **match %** (how many scan points land on known walls: above about 60 % the position is trustworthy), the scan count, the number of **submaps** and **loop closures**.

### Sensor link on the Indoor SLAM screen

The Indoor SLAM screen now has the **Sensor link** (WebSocket) control, like GPS Mission, Object Tracking and Autopilot: the row at the bottom on Android, the antenna button in the top bar on iOS.

- **Server mode:** open `http://<phone>:8080/` in a browser. The new **Indoor SLAM** card shows the map, the car (blue arrow), the path (green) and the goal (pink). **Tap the map to send the car there.** It also has **Explore**, **Cancel goal** and **Find me**. The settings card adds the SLAM screen's settings: mode (build map / localize only), the four algorithm choices, Explore and Find me.
- **WebSocket `/slam`**, for your own code:
  - The phone sends `{"type":"slam","active":true,"engine":..,"pose":[x,y,θ],"goal":[x,y],"path":[[x,y],..],"status":..,"map":{..}}` twice a second.
  - The map is included only when it changed (at most every 2 s): one byte per 5 cm cell (0 unknown, 1 free, 2 wall), base64, cropped to the known area.
  - Send `{"goal":[x,y]}`, `{"cancel":true}`, `{"explore":true}`, `{"findMe":true}`, `{"place":[x,y,θ]}` or `{"mapping":false}`.
- *Example (Python):* `ws = websocket.create_connection("ws://192.168.1.20:8080/slam"); ws.send('{"goal":[2.0,1.5]}')` sends the car to x 2.0 m, y 1.5 m on its map.
- *Why:* you can watch the map from a laptop while the phone rides on the car, and a ROS node or script can send goals.
- *Source:* Android `slam/SlamWire.java`, `sensorlink/SlamHub.java`, `SensorLinkServer` (`/slam`); iOS `SlamWire` / `SlamHub` in `SlamAlgorithms.swift`, `SensorLinkServer.swift`; the page `SensorLinkDashboard.html`.

### Choosing the algorithms

Tap **Algorithms** (next to Find me) and pick one option in each of the four lists. The choice is saved on the phone. Changing the SLAM engine keeps the current map and the car's position.

| Step | Option | How it works (simple) | Good for | Source |
|---|---|---|---|---|
| **SLAM** | **Cartographer** (default) | Small local maps; a branch-and-bound search finds old places again (loop closure) and straightens the map | Whole flats, loops | Hess et al., ICRA 2016; cartographer_ros |
| | **slam_toolbox (Karto)** | A graph of scans every 0.5 m; a new scan is matched to the last 10 scans; chains of old scans close loops | Medium flats; the ROS 2 default | Macenski, JOSS 2021; Konolige, IROS 2010 |
| | **Hector SLAM** | Every scan is matched to the whole map by Gauss-Newton, coarse to fine (20 → 10 → 5 cm); no odometry, no loop closure | One or two rooms; the fastest | Kohlbrecher et al., SSRR 2011 |
| | **GMapping** | 12 particles, each a guess of the path with its own map; bad guesses die, good ones copy | Rooms where scan matching alone gets confused | Grisetti et al., T-RO 2007 |
| | **CoreSLAM / BreezySLAM** | One map; random "hill climbing" tries; no loop closure | Single rooms, oldest phones | Steux, ICARCV 2010 |
| **Localizer** (Localize only) | **Scan matching** (default) | The chosen engine matches each scan to the saved map | Most cases | — |
| | **AMCL** | 500–2000 particles moved by LiDAR odometry and weighted by a likelihood field | Busy rooms (people, moved chairs); Nav2's default | Nav2 amcl; Thrun, *Probabilistic Robotics* |
| **Planner** | **NavFn (A\*)** (default) | Shortest cheap path over 10 cm cells, then straightened | General use | Nav2 nav2_navfn_planner |
| | **Theta\*** | A\* that may draw straight lines past corners, so it takes fewer, longer straight segments | Open rooms | Nash et al., AAAI 2007; Nav2 |
| | **Smac Hybrid-A\*** | Searches car poses with arcs this car can really drive (0.49 m minimum radius), forward **and reverse** | Tight spots, U-turns, goals behind the car | Dolgov et al., IJRR 2010; Nav2 Smac |
| **Controller** | **Regulated Pure Pursuit** (default) | Steer towards a point 0.55 m ahead; slow in turns and near walls; follows reverse parts of Hybrid-A\* paths | Smooth, cheap | Nav2 RPP |
| | **DWB** | Tries 66 (speed, steer) pairs for 1.7 s and scores them by path distance, goal distance and obstacles | Quick reactions | Fox et al., IEEE RAM 1997; Nav2 DWB |
| | **MPPI** | 300 noisy variations of its plan, 5.6 s ahead; averages the good ones, weighted by score | Smooth avoiding; chooses to reverse by itself | Williams et al., ICRA 2017; Nav2 MPPI |

The default settings come from each project: Cartographer's `trajectory_builder_2d.lua` / `pose_graph.lua`, slam_toolbox's `mapper_params_online_async.yaml`, gmapping and hector_mapping source, and the Nav2 configuration guide. Where the phone or the car needed a change, it is listed:
- GMapping: 12 particles instead of 30, and updates every 0.5 m instead of 1 m.
- Cartographer: a smaller loop-search window indoors.
- MPPI: 300 samples instead of 1000, 0.1 s steps, and a doubled PathFollow weight, because this car drives slowly.

*Example:* the default combination works for a flat. If the car must squeeze between a sofa and a table, or reach a goal behind it, choose **Smac Hybrid-A\*** with **Regulated Pure Pursuit**: the path then includes the reversing the car needs.

**Simulation results** (unit tests and a simulator, not the real car yet):

| SLAM, corridor ring, 2 laps, noisy LiDAR | End error | Worst error | Time per scan (PC) |
|---|---|---|---|
| Cartographer | 0.03 m | 0.17 m | 10 ms |
| slam_toolbox | 0.02 m | 0.11 m | 5 ms |
| Hector | 0.03 m | 0.09 m | 0.7 ms |
| GMapping | 0.02 m | 0.08 m | 4 ms |
| CoreSLAM | 0.09 m | 0.23 m | 1.5 ms |

All 9 planner/controller pairs reached the goal in 4 test drives in a room with a box: round the box, goal behind, U-turn, goal to the side. None came closer than 0.28 m to a wall.
- Pure Pursuit and DWB took about 20 s.
- MPPI took 30–70 s: it is careful and slow with the defaults, and is the heaviest (about 8 ms per step on a PC).
- With **Hybrid-A\***, every controller was quickest on the U-turn.

**Why give a choice?** Each method fails in a different place. Hector is fast but can't fix a bent map on a long loop. Cartographer and slam_toolbox fix loops but cost more. A planner that only draws straight lines can't plan a three-point turn; Hybrid-A\* can. Trying them on your own flat is the best way to learn which suits it.

**How it works and where each idea comes from.** The SLAM part follows Google **Cartographer** (`cartographer_ros`, Apache-2.0) with its default settings from `trajectory_builder_2d.lua` and `pose_graph.lua`. The ideas were rewritten in Java and Swift from the Cartographer paper (Hess et al., *Real-Time Loop Closure in 2D LIDAR SLAM*, ICRA 2016). BreezySLAM and slam_toolbox are LGPL, so no code was copied from any of these projects:

- **Submaps** (Cartographer): each scan goes into a small local map of about 90 scans (`num_range_data = 90`), and two submaps are active at a time. The **motion filter** adds a scan only after the car moves 0.2 m, turns 1° or 5 s pass. *Example:* one lap of a living room makes about 5 submaps. *Why:* a small map that never changes again can be moved as one piece later.
- **Occupancy grid** (BreezySLAM / CoreSLAM, slam_toolbox): cells are 5 cm. Each beam lowers the "wall" odds along its path and raises them at the end. The global 30 × 30 m map is painted from the submaps.
- **Scan matching** (Cartographer local SLAM): first a **correlative search** tries every shift (±15 cm) and every turn (±20°) cell by cell. Then a **least-squares polish** on a smooth (bicubic) map finds the pose below one cell, the job Ceres does in Cartographer. *Why:* the car has no wheel encoders, so the scan itself is the odometry. The brute-force first step can't get stuck the way hill climbing (the old RMHC method) can.
- **Phone gyro as a hint** (Cartographer uses an IMU the same way; SmartphoneGraphSLAM uses phone sensors): the gyro gives the turn between scans, so the search starts close to the right answer.
- **Loop closure** (Cartographer global SLAM): old submaps are searched for each new scan with **branch and bound**. The map is also kept at coarser levels where each cell holds the best value of the cells under it, so whole blocks of poses are skipped (`branch_and_bound_depth = 7`, 30°, min score 0.55). A good match adds a link to a **pose graph**, and **sparse pose adjustment** then moves whole submaps until all the links agree. *Example:* after a lap of a corridor the start and end walls are drawn twice, 15 cm apart; one loop closure pulls them into one line.
- **Indoor safety checks** (not in Cartographer): rooms and corridors repeat, so the search window grows with the distance driven (0.5 m + 10 %, at most 7 m). A match that could slide along a bare corridor is skipped. A link the solved graph still misses by more than 0.3 m or 5° is thrown away. *Why:* in the simulated corridor ring, a 7 m window joined the top corridor to the bottom one (4.4 m apart) and bent the map. These checks stopped that.
- **Global localization** (Cartographer pure localization): the same branch and bound searches the whole saved map at every heading (min score 0.6). That is how **Find me** works.
- **Costmap with inflation** (Nav2 `inflation_layer`): cells within 0.22 m of a wall are forbidden; cost fades out over 0.45 m more; unknown cells cost a little. *Why:* the path keeps to the middle of a doorway instead of scraping the frame.
- **A\* planner** (Nav2 `NavFn` / Smac idea) on a 10 cm grid, then shortened with straight-line shortcuts.
- **Regulated Pure Pursuit** (Nav2 controller): steers towards a point 0.55 m ahead on the path and slows in tight turns and near obstacles. Steering uses the car's real geometry (0.26 m wheelbase, 28° lock).
- **Ackermann extras:** if the path starts behind the car it does a **multi-point turn** (back, forward, back) on one side, like a person turning a car in a narrow street. If something blocks it within 0.35 m it **backs up** and re-plans (Nav2 BackUp recovery).
- **The map view** (PyRoboViz, RViz): grey = unknown, white = free, black = wall, the car as an arrow, scan dots, path and trail. *Example:* in RViz you would add Map, LaserScan, Path and RobotModel displays; here they are the four layer buttons.

**Limits:**
- **Loop closure moves whole submaps** (Cartographer also moves every scan), so walls can stay a few cm thick where submaps overlap. Drive slowly (about 30 % speed) and close loops often, for example back to the start of each room.
- **Look-alike places:** a symmetric room can fool **Find me**. Check the car arrow, and long-press if it's wrong.
- **Needs full scans:** USB LiDAR (Android) or the car's Wi-Fi. The 8 Bluetooth sectors are too coarse; the screen says so.
- **Glass and mirrors** don't reflect the LiDAR well, so they may show as open space.
- **Tested in simulation, not yet on the real car:**
  - Two-room flat (explore, then drive through the door): 0.03 m worst position error.
  - Corridor ring, 2 laps, noisy LiDAR, drifting gyro: the end error was 0.17 m without loop closure and 0.03 m with it; the old RMHC engine ended at 0.09 m.
  - Each scan takes about 10 ms on a PC, so expect several times that on a phone.
  - Try it with the wheels off the ground first, then in an open room. STOP and object avoidance still apply.
- *Source:* Android `slam/`: `Slam`, `SlamEngine` (Cartographer), `KartoSlam`, `HectorSlam`, `GMappingSlam`, `CoreSlam`, `Amcl`, `SlamSystem`, `Costmap`, `Planners`, `Controllers`, `Navigator`, `SlamFragment` (30 unit tests). iOS `Controllers/Slam/`: `SlamCore.swift`, `SlamAlgorithms.swift`, `NavAlgorithms.swift`, `SlamController.swift`.

**Why build the map first?** A planner can only go round walls it knows about. With a map, a goal in the next room becomes a path through the door; without one, the car would only see the wall in front of it.

---

## 2f. Car setup page (Android and iOS)

The page lists only what this car really has. The firmware reports its parts when the app sends `f` (reply `fRC_S3:i:lf:lb:ls:s:p:x:`). This car has **no battery sensor, sonar, bumpers or wheel encoders**, so those rows are gone.

| Section | What it shows | Where it comes from |
|---|---|---|
| Header | Robot type (`RC_S3`), refresh, USB / Bluetooth | `f` reply |
| Drive | "ESC throttle · steering servo (Ackermann car)" | type starting `RC_` |
| **What the car reports** | LiDAR obstacle ahead (`s`), LiDAR 8 sectors (`p`), LiDAR scans by USB (`x`), RC receiver (`m` messages), indicators (`i`), status LEDs (`ls`), front / rear lights (`lf`, `lb`) | firmware features |
| **Readings** | Nearest obstacle ahead in cm ("Clear" above 3 m, "No LiDAR data" when the scan is stale: the car then holds still); RC switch MANUAL / AUTO / HOLD / lost; link USB or Bluetooth | `s<cm>`, `m<letter>` |
| **Tests** | Motors forward / back / stop; **steering left / centre / right**; **indicators left / off / right**; lights slider (only shown if the car has lights) | `c`, `i`, `l` commands |
| LiDAR plan view | The live scan and the avoidance ring | USB or the car's Wi-Fi |
| Avoidance | Distance and back-up speed | as before |

- *Example:* tap **Steering → Left** with the wheels off the ground. The servo turns left for about a second, then the car's 1 s heartbeat straightens it.
- *Why remove the old rows?* A box that can never be ticked (bumpers, wheel odometry) makes a working car look broken. "Sonar" was really the LiDAR's forward cone, so it is now named that.
- *Source:* Android `robot/RobotInfoFragment.java` and `fragment_robot_info.xml`; iOS `Controllers/RobotInfo/RobotInfoFrame.swift`; firmware `process_feature_msg()` in `openbot-esp32s3.cpp`.

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

**LiDAR map in the information panel.** On GPS Mission, Object Tracking and Autopilot, ticking **Object avoidance** opens a LiDAR map right under the switch; unticking hides it. On iPhone the switch and map sit in GPS Mission's data panel, and in a small card at the top-left of Object Tracking and Autopilot.
- The car is in the middle, facing up. Rings are at 1, 2 and 4 m.
- The outline is the LiDAR scan. Coloured wedges are the car's 8 obstacle sectors: red under 0.5 m, amber under 1 m, green further.
- The ring is the avoidance distance: the car stops before anything comes inside it.
- The line below the map says what is arriving. It shows bins and returns for a live scan, or *only the 8 sectors* over Bluetooth, where full scans don't fit. A grey picture means the scan stopped and is frozen.
- *Example:* you tick Object avoidance in GPS Mission and a red wedge appears ahead-left. A box is 40 cm away, so the car will hold there until you move it.
- *Why show it here?* You can see what the car is avoiding at the moment you switch avoidance on, without going to Robot Info.


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
