# OpenBot Ackermann Robot: Install Guide

This guide takes you from a fresh download to a car that drives: **flash the ESP32-S3**, **install the app on an Android phone or an iPhone**, **set it up**, and **upload your changes to GitHub**. It is written in simple words. Each point gives its **Source** (the file that proves it), an **Example**, and **Why** where it helps.

For the details behind each step, see the [Firmware guide](FIRMWARE_GUIDE.md) (wiring, every setting, RC transmitter) and the [App guide](APPS_GUIDE.md) (every screen).

**Contents**

1. [What you need](#1-what-you-need)
2. [Get the code](#2-get-the-code)
3. [Install the firmware on the ESP32-S3](#3-install-the-firmware-on-the-esp32-s3)
4. [Install the app on an Android phone](#4-install-the-app-on-an-android-phone)
5. [Install the app on an iPhone](#5-install-the-app-on-an-iphone)
6. [Settings after installing](#6-settings-after-installing)
7. [Upload your changes to GitHub](#7-upload-your-changes-to-github)
8. [Troubleshooting](#8-troubleshooting)
9. [Questions to understand deeply](#9-questions-to-understand-deeply)

---

## 1. What you need

| Item | Needed for | Notes |
|---|---|---|
| The car, wired as in the [pinout map](ESP32S3_PINOUT.svg) | Everything | ESP32-S3 N16R8, ESC, steering servo, LD14/LD14P LiDAR, LEDs; FlySky receiver optional |
| A computer with **Arduino IDE 2** | Flashing the ESP32 | Windows, macOS or Linux |
| A USB-C data cable | Flashing, and the Android link | Some cables only charge: they will not work |
| **Android phone**, Android 7.0 or newer | Android app | Plus a **USB-C OTG** cable to the car |
| **iPhone**, iOS 15.5 or newer, plus a **Mac with Xcode** | iOS app | An Apple ID is enough; a paid developer account is optional |
| **Android Studio** (optional) | Building the Android app yourself | Not needed if you install the ready APK |
| A **GitHub account** and **git** | Uploading changes | GitHub Desktop is the easy option |

- *Source:* `docs/FIRMWARE_GUIDE.md` section 2; `robot/android/app/build.gradle` (`minSdkVersion 24` = Android 7.0); iOS deployment target 15.5 in `robot/ios/OpenBot/OpenBot.xcodeproj/project.pbxproj`.
- *Why does Android use a cable and the iPhone use Bluetooth?* iOS does not let apps talk to USB serial devices such as the ESP32-S3. So the iPhone drives the car over Bluetooth LE. *Source:* `robot/README.md`.

---

## 2. Get the code

**Option A: GitHub Desktop (easiest)**
1. Install [GitHub Desktop](https://desktop.github.com/) and sign in.
2. **File → Clone repository → URL**, paste `https://github.com/quangvu16/Ackermann_robot_openbot`, choose a folder, and click **Clone**.

**Option B: command line**
```bash
git clone https://github.com/quangvu16/Ackermann_robot_openbot.git
cd Ackermann_robot_openbot
```

**Option C: just download.** On the GitHub page, click **Code → Download ZIP**. This works for flashing and building, but you cannot push changes back from it.

- *Example:* on the Mac, clone into `~/Documents/Ackermann_robot_openbot`. Later you update it with one **Fetch / Pull** instead of copying files by hand.
- *Why clone instead of copying?* A clone remembers where it came from. One `git pull` brings every new file and change, so the phone apps and the firmware never get out of step.

The folders you will use:

```
firmware/openbot-esp32s3/   ESP32-S3 firmware (open the .ino in Arduino IDE)
robot/android/              Android app (open this folder in Android Studio)
robot/ios/OpenBot/          iOS app (open OpenBot.xcworkspace in Xcode)
docs/                       Guides and the pinout map
```

---

## 3. Install the firmware on the ESP32-S3

### 3.1 Install Arduino IDE and the ESP32 package

1. Install **Arduino IDE 2** from [arduino.cc](https://www.arduino.cc/en/software).
2. Open **File → Preferences**. In *Additional boards manager URLs*, add:
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
3. Open **Tools → Board → Boards Manager**, search **esp32**, and install **esp32 by Espressif Systems**. Version **3.3.12** is the one GitHub tests; any 3.x or 2.x also builds.

- *Source:* `.github/workflows/firmware-build.yml` (`ESP32_CORE: esp32:esp32@3.3.12` and the same board-manager URL); `docs/FIRMWARE_GUIDE.md` section 5 (core 2.x and 3.x both work).
- *Why use the tested version?* GitHub compiles the firmware with exactly this version on every change. If yours matches, a build that passes there will pass for you.

**No extra libraries are needed.** Bluetooth, Wi-Fi and the motor pulses all come with the ESP32 package.
- *Source:* `#include` lines in `firmware/openbot-esp32s3/openbot-esp32s3.cpp` (`BLEDevice.h` and the others are part of the ESP32 core).

### 3.2 Put your own settings in a separate file

1. Open `firmware/openbot-esp32s3/openbot-esp32s3.cpp` and find the **SETTINGS** block at the top.
2. Copy the whole block into a new file **`openbot_settings.h`** in the same folder.
3. Change the values in **your** file. The firmware uses it instead of the defaults.

| Setting | Change it when | Example |
|---|---|---|
| `WIFI_SSID`, `WIFI_PASS` | The iPhone LiDAR view or SLAM uses Wi-Fi | your home router name and password |
| `LIDAR_UDP_DEST_IP` | iPhone over Wi-Fi: set it to the iPhone's IP | `192.168.1.23` |
| `LIDAR_BAUD`, `LIDAR_MEASURE_FREQ` | You have an **LD14P** | 230400 and 4000 (LD14: 115200 and 2300) |
| `SERIAL_SCAN_STEP` | Phone on the **native USB** port | 1 (COM port: 2) |
| `SERVO_REVERSED` | The car steers the wrong way | 1 |
| `HAS_RC`, `RC_PROTOCOL` | No receiver, or a PWM receiver | `HAS_RC 0`, or `RC_PWM` |

- *Source:* `openbot-esp32s3.cpp` settings block (`__has_include("openbot_settings.h")`); `docs/FIRMWARE_GUIDE.md` section 4.
- *Why a separate file?* `openbot_settings.h` is listed in `.gitignore`. So your Wi-Fi password **never goes to GitHub**, and a `git pull` never overwrites your calibration. *Source:* `.gitignore` ("local and secret files: keep these out of GitHub").

### 3.3 Choose the board settings and upload

1. Open `firmware/openbot-esp32s3/openbot-esp32s3.ino` in Arduino IDE. The real code is in the `.cpp` next to it, which compiles too.
2. In the **Tools** menu choose:

| Tools menu | Choose | Why |
|---|---|---|
| Board | **ESP32S3 Dev Module** | The firmware stops with an error on other boards |
| Flash Size | **16MB (128Mb)** | The N16R8 has 16 MB flash |
| PSRAM | **OPI PSRAM** | The N16R8 has 8 MB *octal* PSRAM |
| USB CDC On Boot | **Disabled** if the phone plugs into the **COM/UART** port; **Enabled** for the **native USB** port | It decides which USB socket the phone must use |
| Port | The board's port | Plug the board in first |

3. Click **Upload** (→). If upload does not start, hold the **BOOT** button, tap **RST**, release BOOT, and upload again.

- *Source:* `.github/workflows/firmware-build.yml` (`FQBN: esp32:esp32:esp32s3:PSRAM=opi,FlashSize=16M`); `docs/FIRMWARE_GUIDE.md` section 5; `robot/README.md` (USB CDC On Boot). The BOOT/RST step is general ESP32-S3 practice (it forces the download mode).
- *Example:* the phone will plug into the socket marked **COM**. Choose *USB CDC On Boot: Disabled* and `SERIAL_SCAN_STEP 2`.

**Command-line alternative** (the same build GitHub runs):
```bash
arduino-cli core install esp32:esp32@3.3.12
arduino-cli compile --fqbn esp32:esp32:esp32s3:PSRAM=opi,FlashSize=16M firmware/openbot-esp32s3
arduino-cli upload  --fqbn esp32:esp32:esp32s3:PSRAM=opi,FlashSize=16M -p <port> firmware/openbot-esp32s3
```
- *Example port:* `COM5` on Windows, `/dev/cu.usbmodem1101` on a Mac, `/dev/ttyACM0` on Linux.

### 3.4 Check it with the wheels off the ground

Open **Tools → Serial Monitor** at **115200** baud and press **RST**.

| You should see | It proves |
|---|---|
| Left indicator blinks, then the right one | The LEDs are wired right |
| `OpenBot RC_S3 (ESP32-S3): ...` | The firmware started |
| `LiDAR SN ... hw ... fw ...` | The LiDAR talks |
| `Waiting a client connection to notify...` | Bluetooth is on, named **OpenBot** |
| The ESC beeps and arms | It saw neutral (1500 µs) |

- *Source:* `docs/FIRMWARE_GUIDE.md` section 6 (`setup()`, `setup_ble()`).
- *Example:* you see `not a single byte received` on GPIO13. The LiDAR sends nothing: check its 5 V power, then its TX wire.
- **Safety:** keep the wheels off the ground until you have calibrated steering and throttle ([Firmware guide section 7](FIRMWARE_GUIDE.md#7-calibrate-throttle-and-steering)). *Why:* an ESC on a wrong range can go to full throttle. The firmware stops the car if no command arrives for 1 second, but only once it is talking to an app.

---

## 4. Install the app on an Android phone

Pick **one** of the two ways.

### 4.1 Way A: install the ready APK (no computer needed)

1. On the phone, open one of these:
   - **Latest build** (made after every change to `main`): `https://github.com/quangvu16/Ackermann_robot_openbot/raw/apk-latest/OpenBot-Ackermann-debug.apk`
   - **Releases** (versions you tagged): `https://github.com/quangvu16/Ackermann_robot_openbot/releases`
2. Open the downloaded file. Android asks to **allow installs from this source**: tap **Settings**, turn it on, go back, tap **Install**.
3. If it says **"App not installed"** or **"conflicts with an existing package"**, uninstall the old OpenBot first, then install again.

- *Source:* `.github/workflows/android-build.yml` (publishes `OpenBot-Ackermann-debug.apk` to the `apk-latest` branch on each push to `main`); `.github/workflows/android-release.yml` (builds a release on a `v*.*.*` tag).
- *Why uninstall first?* Android only replaces an app with one signed by the same key. A debug build from GitHub and one from your Mac have different keys. Uninstalling deletes saved maps and settings, so export SLAM maps and missions first.

### 4.2 Way B: build and run from Android Studio

1. Install [Android Studio](https://developer.android.com/studio). **File → Open** the folder **`robot/android`** (not the repo root).
2. Put your **`google-services.json`** in `robot/android/app/` *and* `robot/android/app/src/main/assets/`. If you have none, copy `robot/android/ci/google-services.placeholder.json` to both places with that name.
3. Wait for **Gradle sync** to finish. Click **Sync Now** if a banner asks.
4. On the phone: **Settings → About phone →** tap **Build number** 7 times. Then **Developer options → USB debugging** on. Plug the phone into the computer and tap **Allow**.
5. At the top of Android Studio, choose run configuration **robot** and your phone (for example *vsmart Joy 4*), then click **▶ Run**.

- *Source:* `robot/android/README.md` (Build process); `docs/APPS_GUIDE.md` section 3.1; `.github/workflows/android-build.yml` ("Add the dummy google-services.json"); `.gitignore` (keeps `google-services.json` out of GitHub). The Build number and USB debugging steps are standard Android.
- *Why the placeholder file?* The app's build plugin needs a Firebase file to exist. Your real one holds API keys, so it is never uploaded. The placeholder lets the app build; only Firebase features (cloud sign-in) stay off.
- *Example:* the device list shows only "Medium Phone API" (an emulator). The phone is not connected: check the cable carries data and USB debugging is allowed.

### 4.3 First start on Android

1. Allow **Nearby devices / Bluetooth, Location, Camera** when asked. *Why:* Location is needed for GPS Mission and, on older Android, for Bluetooth scans; the camera is for driving views and the sensor link.
2. Plug the phone into the car's ESP32-S3 with the **USB-C OTG** cable, into the socket chosen in section 3.3. Tap **OK** on "Open OpenBot when this USB device is connected".
3. On **Home**, tap **🔗 Connect**. The status should turn green. *Source:* `docs/APPS_GUIDE.md` sections 2a and 3.2.

---

## 5. Install the app on an iPhone

You need a **Mac with Xcode**. Apple does not allow installing your own iPhone app any other way without the App Store.

### 5.1 Prepare the Mac

1. Install **Xcode** from the Mac App Store and open it once to finish the setup.
2. Install **CocoaPods**: `sudo gem install cocoapods` (or `brew install cocoapods`).
3. In Terminal:
   ```bash
   cd Ackermann_robot_openbot/robot/ios/OpenBot
   pod install
   open OpenBot.xcworkspace
   ```
   Always open **`OpenBot.xcworkspace`**, not the `.xcodeproj`.

- *Source:* `docs/APPS_GUIDE.md` section 4.1; `robot/ios/README.md`.
- *Why the workspace?* `pod install` adds the libraries the app uses (in the `Pods/` folder). Only the workspace includes them, so the `.xcodeproj` alone fails to build.

### 5.2 Signing: your Apple ID and your own bundle ID

1. **Xcode → Settings → Accounts →** **+**, and sign in with your Apple ID.
2. Click the **OpenBot** project → target **OpenBot** → **Signing & Capabilities**:
   - Turn on **Automatically manage signing**.
   - **Team:** your name *(Personal Team)*.
   - **Bundle Identifier:** change `org.openbot.app.robot` to your own, such as `com.yourname.openbot.ackermann`.
3. Do the same for targets **OpenBotTests** and **OpenBotUITests** if Xcode complains (use `.tests` and `.uitests` at the end).

- *Example:* the error *"Failed Registering Bundle Identifier org.openbot.app.robot"* means that name belongs to another developer. A new, unique name fixes it.
- *Why must it be unique?* Apple ties each bundle ID to one developer team, like a car number plate.

### 5.3 Prepare the iPhone and run

1. Connect the iPhone to the Mac by cable and tap **Trust This Computer** on the phone.
2. On iOS 16 or newer: **Settings → Privacy & Security → Developer Mode → On**, and restart the phone.
3. In Xcode's top bar, choose your iPhone as the destination, then click **▶ Run**.
4. The first time, the app may not open (*"Untrusted Developer"*). On the iPhone: **Settings → General → VPN & Device Management →** your Apple ID → **Trust**. Then open the app.
5. Allow **Bluetooth, Camera, Location, Local Network** and **Motion** when asked.

- *Source:* Apple's guide [Running your app on a device](https://developer.apple.com/documentation/xcode/running-your-app-in-simulator-or-on-a-device) and [Enabling Developer Mode](https://developer.apple.com/documentation/xcode/enabling-developer-mode-on-a-device); the permission texts are in `robot/ios/OpenBot/OpenBot/Info.plist`.
- *Why Local Network and Motion?* Local Network lets the app receive LiDAR scans over Wi-Fi and serve the sensor-link page; Motion reads the barometer and tilt. Without the Motion text in `Info.plist`, iOS closes the app as soon as it reads those sensors.

**Free Apple ID limits** (no paid developer account):
- The app **stops opening after 7 days**. Plug in and press **Run** again to renew it; your data stays.
- At most **3** of your own apps on the phone at once.
- *Source:* Apple Developer, [Choosing a membership](https://developer.apple.com/support/compare-memberships/). A paid account ($99/year) gives one-year apps and TestFlight.

### 5.4 First start on iPhone

1. On **Home**, tap **🔗 Connect** and choose **OpenBot** from the Bluetooth list.
2. For the LiDAR view and Indoor SLAM on iPhone, put the iPhone and the car on the **same Wi-Fi**, and set `LIDAR_UDP_DEST_IP` in the firmware to the iPhone's address (**Settings → Wi-Fi → ⓘ → IP Address**).

- *Source:* `docs/APPS_GUIDE.md` sections 4.2 and 4.5.
- *Why Wi-Fi for scans?* One LiDAR turn is too big for Bluetooth. Driving and avoidance still work over Bluetooth alone, because they only need the small 8-sector summary.

---

## 6. Settings after installing

Do these once, in this order, with the wheels off the ground.

| Step | Where | What to check | Example |
|---|---|---|---|
| 1. Parts list | **Car setup** | *What the car reports* lists LiDAR, indicators, lights, RC receiver | No LiDAR row → check firmware `HAS_LIDAR` and wiring |
| 2. Steering | Car setup → Tests → ↰ / steering wheel / ↱ | Wheels turn the way the icon shows | Turns the wrong way → `SERVO_REVERSED 1` |
| 3. Motors | Car setup → Tests → ↑ / ↓ / ⏹ | Wheels spin forward on ↑ | Spins backwards → swap two motor wires or check the ESC |
| 4. LiDAR | Car setup → LiDAR plan view | Objects in front show in front | Front shows at the side → set `LIDAR_FORWARD_DEG` |
| 5. Avoidance | Car setup → 🛡 (iPhone) or avoidance sliders | Distance 0.3–3 m, back-up speed 0–0.5 | Small room: 0.4 m, back-up 0.1 |
| 6. RC switch | Move the FS-i6 switch | Car setup shows MANUAL / AUTO / HOLD | AUTO is needed for the app to drive |
| 7. GPS | GPS Mission | Phone upright, camera forward, 15–20 cm from motors | *COMPASS UNRELIABLE* → move the phone away from metal |
| 8. Indoor SLAM | Indoor SLAM → **Algorithms** (sliders icon) | Defaults: Cartographer · scan matching · NavFn · RPP | Start with the defaults |
| 9. Sensor link | Antenna icon (iPhone) or Sensor link button (Android) | Mode **Server**, then open `http://<phone address>:8080` | A laptop on the same Wi-Fi shows the car live |

- *Source:* `docs/APPS_GUIDE.md` sections 2c, 2e, 2f, 3.3, 3.5, 4.4; `docs/FIRMWARE_GUIDE.md` sections 7 and 9; SLAM defaults in `robot/android/app/src/main/java/org/openbot/app/robot/slam/SlamFragment.java` (`applySavedAlgorithms`).
- **Icon buttons:** buttons show icons. **Long-press any icon** to see its name. *Example:* long-press ▶ on the LiDAR robot screen and it says *Start*.
- *Why this order?* Each step relies on the one before. Steering and motors must be right before avoidance can stop the car correctly, and the LiDAR must face forward before SLAM can draw a true map.

---

## 7. Upload your changes to GitHub

### 7.1 One-time setup

1. **Who you are** (shown on each change):
   ```bash
   git config --global user.name  "Your Name"
   git config --global user.email "you@example.com"
   ```
2. **Sign in.** The easy ways: **GitHub Desktop** (it signs in for you), or `gh auth login` with the [GitHub CLI](https://cli.github.com/).
   If git asks for a password on the command line, use a **personal access token**, not your GitHub password (**GitHub → Settings → Developer settings → Personal access tokens**).

- *Source:* GitHub Docs, [About authentication to GitHub](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/about-authentication-to-github).
- **Never paste a token into a file, a chat or a commit.** If one leaks, delete it on GitHub at once and make a new one. *Why:* anyone with the token can change or delete your repositories.

### 7.2 Each time you change something

**GitHub Desktop:** it lists the changed files → type a short summary → **Commit to main** → **Push origin**.

**Command line:**
```bash
git pull                      # 1. get others' changes first
git status                    # 2. see what you changed
git add docs/ firmware/       # 3. choose the files to upload
git commit -m "Tune steering trim and stop distance"   # 4. describe the change
git push                      # 5. upload to GitHub
```

- *Example message:* `fix: LiDAR front points the right way (LIDAR_FORWARD_DEG 90)`. *Why:* a clear message tells you in six months what changed and why.
- *Why pull first?* If GitHub has newer changes (for example from another computer), pushing without them is refused. Pulling first merges them in.

### 7.3 What stays off GitHub on purpose

These are in `.gitignore`, so git skips them even with `git add .`:

| File | Why it stays private |
|---|---|
| `firmware/openbot-esp32s3/openbot_settings.h` | Wi-Fi password, your calibration |
| `google-services.json`, `GoogleService-Info.plist` | Firebase API keys |
| `local.properties` | Path to your Android SDK, different on each computer |
| `*.jks`, `*.keystore` | App signing keys |
| `Pods/`, `build/`, `DerivedData/` | Rebuilt automatically; large |

- *Source:* `.gitignore` in the repo root.
- *Example:* run `git status` after editing `openbot_settings.h`. It is not listed, which proves it will not be uploaded.

### 7.4 What GitHub does after you push

| You push | GitHub runs | Result |
|---|---|---|
| A change in `firmware/openbot-esp32s3/` | **Firmware build** | Compiles 3 versions: iBUS, PWM and no receiver |
| A change in `robot/android/` | **Android build** | Builds, runs the unit tests, publishes the APK to `apk-latest` |
| A change in `robot/ios/` | **iOS build** | Builds the app and tests |
| A tag such as `v1.0.1` | **Android release APK** | A new entry on the Releases page with the APK |

- *Source:* `.github/workflows/firmware-build.yml`, `android-build.yml`, `ios-build.yml`, `android-release.yml`.
- **See the result:** the **Actions** tab on GitHub. ✅ green = OK, ❌ red = open it to read the error.
- **Make a release:**
  ```bash
  git tag v1.0.1
  git push origin v1.0.1
  ```
  *Why tags?* A tag marks one exact version. Testers install "v1.0.1" and you know exactly which code they have.

---

## 8. Troubleshooting

| Problem | Likely cause | Fix |
|---|---|---|
| Arduino: *"A fatal error occurred: Failed to connect"* | Board not in download mode | Hold **BOOT**, tap **RST**, release, upload again |
| Arduino: *sketch too big* | Bluetooth + Wi-Fi are large | **Tools → Partition Scheme**: a bigger app partition, or `LIDAR_UDP 0` if you don't need Wi-Fi |
| Android app does not see the car | Wrong USB socket, or a charge-only cable | Use the socket that matches *USB CDC On Boot*; try another cable |
| APK: *"App not installed"* | Different signing key | Uninstall the old app, then install |
| Android Studio: *google-services.json is missing* | The file is private | Copy the placeholder (section 4.2 step 2) |
| Xcode: *Failed Registering Bundle Identifier* | Bundle ID taken | Use your own (section 5.2) |
| Xcode: *No such module* | Opened the `.xcodeproj` or skipped `pod install` | Run `pod install`, open `OpenBot.xcworkspace` |
| iPhone app stops opening after a week | Free Apple ID: 7-day limit | Press **Run** in Xcode again |
| iPhone app closes as soon as it starts | Missing permission text in `Info.plist` | Pull the latest code (the Motion text was added on 27 Oct 2026) |
| No LiDAR view on iPhone | Wrong IP or different Wi-Fi | Set `LIDAR_UDP_DEST_IP` to the iPhone's IP; same network |
| `git push` refused: *rejected (fetch first)* | GitHub has newer changes | `git pull`, then `git push` |
| `git push` asks for a password and fails | Passwords are not accepted | Use GitHub Desktop, `gh auth login`, or a token |

More: [Firmware guide section 12](FIRMWARE_GUIDE.md#12-troubleshooting) and [App guide section 7](APPS_GUIDE.md#7-troubleshooting).

---

## 9. Questions to understand deeply

1. **Why does the firmware read settings from `openbot_settings.h` and not straight from the `.cpp`?** Think about what happens to your calibration when you `git pull` a new version.
2. **Why can the Android phone drive over a cable but the iPhone cannot?** What does Apple's rule on USB accessories protect, and what does it cost this project?
3. **Why does a debug APK from GitHub refuse to install over one built on your Mac?** What would one shared signing key change, and why must that key never go to GitHub?
4. **Why does the free Apple ID limit an app to 7 days?** Who is Apple protecting, and when does $99 a year become worth it for you?
5. **Why push small commits with clear messages rather than one big one at the end?** Imagine the car starts steering wrongly next month: how would you find which change caused it?
6. **Why let GitHub build the firmware and apps on every push?** What mistakes would it catch before the code ever reaches the car or the phone?
7. **Why calibrate steering and motors before turning on avoidance and SLAM?** What would a wrong `SERVO_REVERSED` do to a path that SLAM planned?
