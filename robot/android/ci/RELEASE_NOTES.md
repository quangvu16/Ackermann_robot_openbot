OpenBot robot app for the ESP32-S3 Ackermann car.

## Install

1. Download the `.apk` file below on your Android phone (Android 7.0 or later).
2. Open it and allow "install unknown apps" for your browser or file manager when Android asks.
3. Plug the phone into the ESP32-S3 with a USB-C OTG cable, then turn on USB in Robot Info.

If the file name ends in `-debug.apk`, it is signed with a debug key, the same way upstream OpenBot ships its APKs. Android may warn that the app is from an unknown developer. To update from an APK signed with a different key, uninstall the old app first.

## What works

- Driving over the USB cable, with the 1-second failsafe in the firmware
- GPS missions with the phone's own GPS (4 m waypoint radius), with `.waypoints` import and export
- LiDAR plan view over the USB cable, no Wi-Fi needed
- Object avoidance, with backup speed and a quick-disable switch

Google sign-in and Google Drive projects need your own Firebase config. Without it, this build uses a dummy config and those features do not work.

## Guides

- [App guide](https://github.com/quangvu16/Ackermann_robot_openbot/blob/main/docs/APPS_GUIDE.md)
- [Firmware guide](https://github.com/quangvu16/Ackermann_robot_openbot/blob/main/docs/FIRMWARE_GUIDE.md)

Test with the wheels off the ground first.
