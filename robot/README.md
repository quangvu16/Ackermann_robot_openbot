# OpenBot Robot App

<p align="center">
  <span>English</span> |
  <a href="README.zh-CN.md">简体中文</a> |
  <a href="README.de-DE.md">Deutsch</a> |
  <a href="README.fr-FR.md">Français</a> |
  <a href="README.es-ES.md">Español</a> |
  <a href="README.ko-KR.md">한국어</a>
</p>

The robot app runs on the phone or computer that sits on the robot. It talks to the robot body, streams video to a controller, and runs the driving policies.

All three platforms share the same feature set:

- **Driving** — joystick, free roam, and neural-network policies.
- **GPS Mission Navigation** — Mission Planner-style waypoint missions: `MISSION` drives a `.waypoints` file in order, `GUIDED` goes to one point, `RTL` returns to launch. `.waypoints` files (QGC WPL 110) round-trip with Mission Planner, and KML export writes both the planned path and the track actually driven.
- **LiDAR plan view** — a live 360° polar plot of one revolution, robot at the centre pointing up, fed from the LD14 / LD14P on the robot body over the "LDS1" UDP stream on port 9900. The firmware sends scans to a single address, so set `LIDAR_UDP_DEST_IP` in its settings to the IP of the phone or computer running the app.
- **Object avoidance** — follows [ArduPilot Rover Simple Object Avoidance](https://ardupilot.org/rover/docs/common-object-avoidance-landing-page.html): always stops at an obstacle (no sliding sideways on a car), `AVOID_MARGIN` sets the standoff, `AVOID_BACKUP_SPD` actively backs away, and there is a quick-disable switch. It sits in the one drive path every mode uses, so nothing escapes it.

### Using the phone's own GPS

The Android and iOS apps navigate with the phone's built-in GPS; no external receiver is needed. (The Linux app has no built-in GPS, so it reads an NMEA receiver instead.)

- **Mount the phone upright, back camera facing forward.** When the car is moving, its heading comes from GPS course-over-ground; when it is standing still (at the start of a mission) the phone's compass is used, and the apps take "forward" to be the direction the back camera looks. Keep the phone away from the motors, the ESC and steel: if the compass needs calibration or the magnetic field is not Earth-like, the GPS screen shows **COMPASS UNRELIABLE** until the car is moving.
- **Phone GPS is good to about 3–5 m under open sky,** so on the phone apps a waypoint counts as reached within **4 m** (the Linux app, which may read an RTK receiver, keeps 2.5 m). Missions hold whenever the reported accuracy is worse than 15 m or no fix has arrived for 3 s. Keep waypoints well apart (10 m or more) and away from tall buildings and trees.
- **Android + USB cable:** plug the phone (USB-C OTG) into the ESP32-S3 port that the firmware's `Serial` goes to, then turn on USB in Robot Info. With the Arduino default *USB CDC On Boot: Disabled*, that is the **COM/UART** port (CH343, CP2102 or CH340 chip); with *Enabled*, it is the native **USB** port. The speed is 115200. The phone powers the ESP32 and LiDAR through the cable; if the board is also fed 5 V from the car's BEC, make sure the board has a diode on USB VBUS so the car does not back-feed the phone.
- **iPhone:** iOS does not let apps talk to USB serial devices such as the ESP32-S3, so the iOS app drives the car over **Bluetooth**. The cable can still power the board, and GPS, LiDAR avoidance and missions all work the same over Bluetooth.
- **LiDAR plan view without Wi-Fi (Android + USB):** the firmware sends each revolution down the cable as an `x` line once the app asks for it, which the Android app does automatically on a USB connection. No Wi-Fi setup is needed. `SERIAL_SCAN_STEP` in the firmware settings sets the resolution: 2 (2° bins) for the 115200-baud COM port, 1 (1° bins) for the native USB port. Over Bluetooth (iPhone) the scan is too large for the link, so the plan view still uses Wi-Fi (UDP): put the phone and the ESP32 on the same network and set `LIDAR_UDP_DEST_IP` to the phone's IP. Object avoidance never needs Wi-Fi: it uses the `p` line on whichever link drives the car.

- [Android](android/README.md): install the APK from the latest release, or build it in Android Studio (open `robot/android`).
- [iOS](ios/README.md) (beta): build it with Xcode; it isn't on the App Store yet.
- [Linux](linux/README.md): a Python version for a Linux computer with a camera.
