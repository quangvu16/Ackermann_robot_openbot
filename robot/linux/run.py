import os
import argparse
from infer import InferNetwork
from lidar import DriveGate, LidarScanner, Proximity

CUR_DIR = os.path.join(os.path.dirname(__file__))

# POLICY_PATH = f"{CUR_DIR}/tests/test_models/tf/"
# POLICY_PATH = f"{CUR_DIR}/tests/test_models/tflite/"
POLICY_PATH = f"{CUR_DIR}/tests/test_models/openvino/"

DATASET_PATH = f"{CUR_DIR}/tests/test_data/logs1/data/"

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Running the OpenBot Intel version via Python script"
    )
    parser.add_argument(
        "--policy_path", "-p", default=POLICY_PATH, help="Path to policy file."
    )
    parser.add_argument(
        "--dataset_path",
        "-d",
        default=DATASET_PATH,
        help="Path to dataset. Only used for debug mode",
    )
    parser.add_argument(
        "--log_path",
        "-l",
        default=None,
        help="Path to log folder, where runs are saved.",
    )
    parser.add_argument(
        "--inference_backend",
        "-i",
        default="openvino",
        help="Backend to use. Consider exporting all models as openvino model for maximum performance. Options: tf, tflite, openvino",
    )
    parser.add_argument(
        "--run_mode",
        "-m",
        default="joystick",
        help="Running mode. Options: debug, inference, joystick",
    )
    parser.add_argument(
        "--control_mode",
        "-c",
        default="dual",
        help="Control mode during joystick mode. Options: dual, joystick.",
    )

    # Object avoidance, ArduPilot Rover style - the same gate run_mission.py
    # uses, so joystick and policy driving cannot escape it either.
    parser.add_argument(
        "--avoid_margin",
        type=float,
        default=1.0,
        help="Object-avoidance standoff in metres (ArduPilot AVOID_MARGIN). 0 disables the law.",
    )
    parser.add_argument(
        "--avoid_backup",
        type=float,
        default=0.0,
        help="Reverse command used to back away from an obstacle (ArduPilot AVOID_BACKUP_SPD). 0 disables backing away.",
    )
    parser.add_argument(
        "--no_avoid",
        action="store_true",
        help="Quick-disable the object-avoidance law for this run.",
    )
    parser.add_argument(
        "--lidar_port",
        type=int,
        default=9900,
        help="UDP port carrying 'LDS1' LiDAR revolutions from the robot.",
    )
    parser.add_argument(
        "--no_lidar",
        action="store_true",
        help="Do not listen for LiDAR scans; avoidance then has no ranging.",
    )
    parser.add_argument(
        "--lidar_forward_deg",
        type=float,
        default=0.0,
        help="LiDAR mounting offset in degrees clockwise from the nose (firmware LIDAR_FORWARD_DEG).",
    )

    args = parser.parse_args()
    options = vars(args)
    avoid_margin = options.pop("avoid_margin")
    avoid_backup = options.pop("avoid_backup")
    no_avoid = options.pop("no_avoid")
    lidar_port = options.pop("lidar_port")
    no_lidar = options.pop("no_lidar")

    gate = DriveGate(
        proximity=Proximity(),
        margin_meters=avoid_margin,
        backup_speed=avoid_backup,
        enabled=not no_avoid and avoid_margin > 0,
    )
    scanner = None
    if not no_lidar and args.run_mode != "debug":
        scanner = LidarScanner()
        if scanner.start(port=lidar_port):
            print(f"Listening for LiDAR scans on UDP {lidar_port}")
        else:
            print(f"Could not bind UDP port {lidar_port}; continuing without LiDAR.")
            scanner = None

    policy = InferNetwork(gate=gate, scanner=scanner, **options)

    try:
        policy.run()
    finally:
        if scanner is not None:
            scanner.stop()
    if args.run_mode != "debug":
        print("Finished run. Logging")
        policy.write_log()
        print("Finished writing log")
