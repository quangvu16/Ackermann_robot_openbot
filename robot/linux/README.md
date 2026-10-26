# Python

<p align="center">
  <span>English</span> |
  <a href="README.zh-CN.md">简体中文</a> |
  <a href="README.de-DE.md">Deutsch</a> |
  <a href="README.fr-FR.md">Français</a> |
  <a href="README.es-ES.md">Español</a> |
  <a href="README.ko-KR.md">한국어</a>
</p>

This module is an embedded Linux alternative to the smartphone control of an OpenBot vehicle. Written in Python, the OpenBot can be controlled using a linux-based computer and a camera for sensing.

The robot can be controlled in two ways: through inference of a Neural Network policy or via joystick. It can also drive waypoint missions from an NMEA GPS receiver, and stop itself at obstacles using a LiDAR.

```
├── __init__.py
├── README.md
├── requirements.txt
├── run.py              # policy inference / joystick driving
├── run_mission.py      # GPS waypoint missions (MISSION / GUIDED / RTL)
├── lidar_view.py       # live LiDAR plan view window
├── motors.py           # serial motor output, the object-avoidance chokepoint
├── generate_data_for_training.py
├── export_openvino.py
├── infer.py
├── joystick.py
├── realsense.py
├── gpsnav/             # GPS mission navigation (pure, hardware-free)
│   ├── geo.py          # haversine, bearings, destination point
│   ├── mission.py      # waypoints, Mission Planner .waypoints codec
│   ├── guidance.py     # differential-drive "go to waypoint" law
│   ├── navigator.py    # MISSION / GUIDED / RTL state machine
│   ├── gps.py          # NMEA receiver and heading fusion
│   └── kml.py          # KML export of the plan and the flown track
├── lidar/              # LiDAR scanning and object avoidance (pure)
│   ├── scan.py         # "LDS1" datagram codec and bin geometry
│   ├── proximity.py    # the eight p-channel octants
│   ├── avoidance.py    # ArduPilot Rover stop law
│   ├── scanner.py      # UDP listener for "LDS1" revolutions
│   ├── drive.py        # DriveGate: every drive command passes through here
│   └── plan_view.py    # polar plot of one revolution
└── tests
    ├── test_data
    │   └── logs1
    │       └── ...
    ├── test_models
    │   ├── openvino
    │   ├── tflite
    │   └── tf.zip
    ├── test_export_openvino.py
    ├── test_gpsnav.py
    ├── test_gpsnav_navigator.py
    ├── test_infer.py
    ├── test_joystick.py
    ├── test_lidar.py
    ├── test_motor.py
    └── test_realsense.py
```

## Running Robot

To operate the robot, run the `run.py`, which is the main Python script. The robot can be run in 3 modes:
- Debug: This mode runs the policy off-line. I.e., instead of real camera images and joystick input commands, it uses data (command and images) loaded from a dataset (see `tests/test_data/logs1/data`) as input to the policy.
- Inference: This mode runs the policy on-line. It uses real camera images and joystick input commands as input to the policy. This mode can be toggled to Joystick mode by pressing the `A` key on the joystick.
- Joystick: This mode operates the robot via joystick command in either "Dual" (controlling left and right wheel via left and right joystick) or "Joystick" (controlling forward, backward, left, right direction via one joystick) `control_mode`. Data collection for training is conducted in Joystick mode. This mode can be toggled to Inference mode by pressing the `A` key on the joystick.

The run.py script accepts these arguments (further details, see `run.py`):
```
--policy_path: Path to policy file.
--dataset_path: Path to dataset. Only used for debug mode.
--log_path: Path to log folder, where runs are saved.
--inference_backend: Backend to use. Consider exporting all models as openvino model for maximum performance. Options: tf, tflite, openvino.
--run_mode: Running mode. Options: debug, inference, joystick.
--control_mode: Control mode during joystick mode. Options: dual, joystick.
--avoid_margin, --avoid_backup, --no_avoid: object avoidance (see LiDAR and Object Avoidance below).
--lidar_port, --no_lidar, --lidar_forward_deg: the LiDAR scan stream that feeds it.
```

## GPS Mission Navigation

`run_mission.py` drives waypoint missions from an NMEA GPS receiver, with the same feature set as the phone app and [Mission Planner](https://ardupilot.org/planner/docs/mission-planner-features.html) itself:

- `MISSION` - AUTO: drive a `.waypoints` file in order
- `GUIDED` - drive to one point ("go to here")
- `RTL` - return to the launch position
- `.waypoints` (QGC WPL 110) save/load, so missions move between platforms
- KML export of the plan and the track actually driven

The guidance, mission and geo code live in `gpsnav/` and are free of hardware and UI dependencies, so they can be unit tested and replayed from a logged track. See `tests/test_gpsnav.py` and `tests/test_gpsnav_navigator.py`.

Drive a saved mission and export the flown track:
```
python run_mission.py --mission_path missions/field.waypoints \
    --gps_device /dev/ttyUSB1 --export_kml field.kml
```

Go to one point:
```
python run_mission.py --goto 47.3977,8.5456 --gps_device /dev/ttyUSB1
```

Build a mission on the command line, save it, then run it:
```
python run_mission.py --add_waypoints 47.3977,8.5456;47.3980,8.5460 \
    --home 47.3977,8.5456 --save_mission missions/demo.waypoints --dry_run
```

Useful flags: `--dry_run` (print the HUD without touching the motors), `--duration` (safety net for unattended runs), `--motor_device`, `--log_path`.

## LiDAR and Object Avoidance

The firmware on the robot body can carry an LD14 / LD14P LiDAR. It reports two ways:

- the **`p` channel**: eight proximity octants in centimetres, matching the MAVLink `DISTANCE_SENSOR` layout used by ArduPilot's proximity library (0 = front, clockwise, one sector every 45 degrees)
- **"LDS1" UDP scans**: full 360-degree revolutions on port 9900, which is the same stream a ROS 2 node would join

`lidar/` consumes both. The plan view (`lidar/plan_view.py`) draws one revolution as a polar plot with the robot at the centre pointing up; bins with no return are *gaps*, not zeros. Watch it live with:
```
python lidar_view.py                       # OpenCV window, q or Esc to quit
python lidar_view.py --snapshot scan.png   # headless robot: write one frame
```
The firmware sends scans to one address, `LIDAR_UDP_DEST_IP` in its settings, so set that to this computer's IP.

Object avoidance follows [ArduPilot Rover Simple Object Avoidance](https://ardupilot.org/rover/docs/common-object-avoidance-landing-page.html):

- Rover always **stops** - there is no sliding sideways past an obstacle on a car
- applies in every mode except manual, by sitting inside the one drive path every caller uses (`motors.apply_action_serial`)
- `AVOID_MARGIN` is the standoff at which the vehicle stops
- `AVOID_BACKUP_SPD` actively backs away to restore the margin; `0` disables it
- a **quick-disable switch** before first use, as ArduPilot insists

Two deliberate deviations from ArduPilot, both toward stopping rather than driving:

- a **stale** sensor (reports arrived, then stopped) blocks forward motion; reverse stays available. A sensor that has **never** reported is treated as "not fitted" and never blocks, so a robot without a LiDAR stays drivable
- blocking zeroes the throttle but **keeps the steering**, so the wheels stay pointed where the driver put them

Both `run.py` (joystick and policy driving) and `run_mission.py` take the same avoidance flags, so no mode escapes the law. Drive with the joystick and a 0.8 m standoff:
```
python run.py --run_mode joystick --avoid_margin 0.8
```

Run the mission with avoidance active and a 1 m standoff:
```
python run_mission.py --mission_path missions/field.waypoints \
    --gps_device /dev/ttyUSB1 --avoid_margin 1.0
```

Other flags: `--avoid_backup` (reverse command used to back away), `--no_avoid` (the quick-disable), `--lidar_port`, `--no_lidar`, `--lidar_forward_deg` (LiDAR mounting offset in degrees clockwise from the car's nose, matching the firmware's `LIDAR_FORWARD_DEG`).

Without a LiDAR the robot drives exactly as before and avoidance has no ranging.

The mission runner also holds the robot (sends `c0,0`) whenever it has no GPS fix newer than 3 seconds, so a lost receiver never leaves the motors on the last command.

## Generating Training Data
The script `generate_data_for_training.py` generates a log data folder that is required for training a policy via the `OpenBot/policy/openbot/train.py` script. The log data folder contains an `images` and a `sensor_data` folder in the format required by `train.py`.

See `tests/test_generate_data.py` for an example.

## OpenVino: Optimising Policy Inference Performance
To optimise the inference speed on supported Intel hardware (such as the [Up Core Plus](https://up-board.org/upcoreplus/specifications/) board), the trained model needs to be exported to OpenVino.

The `export_openvino.py` script exports a trained TensorFlow model to an OpenVino model. This OpenVino model is then loaded via `get_openvino_interpreter()` in `infer.py`.

See `tests/test_export_openvino.py` for an example.

## Tests and example code

**Note:** For testing the code, the test data and test model called `test_data` and `test_model` respectively are required to be in `OpenBot/robot/linux/tests`. The function `get_data()` in `download_data.py` provides download functionality and is called at the beginning of `test_infer.py`, `test_export_openvino.py`, and `test_generate_data.py`. Alternatively,
 please run the script `get_test_data.sh` (unix systems only) that downloads and unzips a zip file containing `test_data` and `test_models` with the data for debug mode and models for inference respectively.

If a ROS 2 environment is on `PYTHONPATH`, disable plugin autoload so it does not hijack pytest:
```
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest tests
```

Run `pytest` in the folder `tests` or run the `test_*.py` files individually to test the functionalities of

- downloading test data and test model from the cloud via `test_download_data.py`
- export to OpenVino via `test_export_openvino.py`
- generating training data via `test_generate_data.py`.
- inference in debug mode for OpenVino, Tensorflow, and Tflite via `test_infer.py`.
    - *Note*: The test data in logs1 is generated using the `associate_frames.py` script in `OpenBot.policy.openbot`, where the path to the images is hardcoded in `logs1/data/sensor_data/matched_frame_ctrl_cmd_processed.txt`.
    - `test_infer.py` replaces it with the path to your copy of the `OpenBot` repository.
- joystick connection via `test_joystick.py`
- motor connection from serial port to Arduino via `test_motor.py`.
- video stream to Realsense camera via `test_realsense.py`.
- GPS mission navigation (geo math, mission codec, guidance law, state machine) via `test_gpsnav.py` and `test_gpsnav_navigator.py`. These are pure and need no hardware.
- LiDAR decoding, proximity octants and the object-avoidance law via `test_lidar.py`. Pure, with an injected clock so staleness is tested without sleeping.

# Installation
The installation process is detailed in the following.

The python implementation for controlling OpenBot requires a few Python modules for inference, joystick control, sensing, and actuation.
Further, drivers for the camera or controller might be required.

## Setup
Currently, the code is tested on:
- Board: [Up Core Plus](https://up-board.org/upcoreplus/specifications/)
- Camera: [Realsense D435i](https://www.intelrealsense.com/depth-camera-d435i/)
- Controller: [Xbox One](https://www.microsoft.com/en-gb/store/collections/xboxcontrollers?source=lp)
- Arduino: [OpenBot Firmware](https://github.com/ob-f/OpenBot/blob/master/firmware/README.md)

## Python modules

The code is tested with Python 3.9. Using Anaconda3:
```
conda create --name openbot python==3.9
```

First, install the requirements of OpenBot.policy via
```
cd ../../policy && pip install -r requirements.txt && cd ../robot/linux
```

Then, install the required modules via
```
pip install -r requirements.txt
```

In particular,
- `pyserial` communicates with the Arduino and thus motors via serial port
- `pyrealsense2` and `opencv-python` are required for camera image processing.
- `pygame` is used for joystick control and processing the joystick inputs
- `openvino-dev[tensorflow2,extras]` is used for boosted performance on supported Intel hardware. For further details on optimised AI inference on Intel hardware, please see [OpenVino](https://docs.openvino.ai/latest/home.html). OpenVino is the recommended inference backend. Tensorflow and Tflite are also supported (see Tests). For running PyTorch modules, please consider converting PyTorch to an OpenVino backend (see [this Tutorial](https://docs.openvino.ai/latest/openvino_docs_MO_DG_prepare_model_convert_model_Convert_Model_From_PyTorch.html)).

`gpsnav` and `lidar` add no new dependencies: `gpsnav` uses only the standard library plus `pyserial` for the NMEA receiver, and `lidar`'s plan view draws with `opencv-python`, which is already required for the camera.

## Drivers
If the code is executed on Ubuntu, the Xbox One controller USB Wireless Dongle requires a driver, which can be found at [this link](https://github.com/medusalix/xone).

## Tensorflow for Inference
If TensorFlow is used for inference, please add the Python `policy` module to `PYTHONPATH` via `export PYTHONPATH=$PYTHONPATH:/path/to/OpenBot/policy`. This workaround avoids having to install openbot as module and to find `openbot.utils.load_model()`, which is required to load the tensorflow model. Further details, see `get_tf_interpreter()` in `infer.py` and the test code `tests/test_infer.py`.

## Support for non-linux distributions (MacOS, Windows)

Please note that the code is intended to run on Linux-based computers, e.g., Up Core Plus. Some python modules may not be available for MacOs or Windows.

The code can run on MacOS for debugging purposes with the following changes:
- Use `pyrealsense2-macosx` instead of `pyrealsense2` in requirements.txt
- For tflite follow [these instructions](https://github.com/milinddeore/TfLite-Standalone-build-Linux-MacOS)
