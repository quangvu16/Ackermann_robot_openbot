"""GPS mission runner - Mission Planner's flight modes on the Linux robot.

Drives waypoint missions from an NMEA GPS receiver, with the same feature set
as the phone app and Mission Planner itself:

- ``MISSION``  - AUTO: drive a ``.waypoints`` file in order
- ``GUIDED``   - drive to one point ("go to here")
- ``RTL``      - return to the launch position
- ``.waypoints`` (QGC WPL 110) save/load, so missions move between platforms
- KML export of the plan and the track actually driven

Examples
--------
Drive a saved mission and export the flown track::

    python run_mission.py --mission_path missions/field.waypoints \\
        --gps_device /dev/ttyUSB1 --export_kml field.kml

Go to one point::

    python run_mission.py --goto 47.3977,8.5456 --gps_device /dev/ttyUSB1

Build a mission on the command line, save it, then run it::

    python run_mission.py --add_waypoints 47.3977,8.5456;47.3980,8.5460 \\
        --home 47.3977,8.5456 --save_mission missions/demo.waypoints --dry_run
"""

import argparse
import csv
import os
import time
from datetime import datetime

from gpsnav import Mission, NavMode, Navigator, Waypoint
from gpsnav.gps import Gps
from lidar import DriveGate, LidarScanner, Proximity
from motors import apply_action_serial, get_arduino_serial, stop_robot

CUR_DIR = os.path.join(os.path.dirname(__file__))

#: Guidance loop rate; GPS is ~1 Hz so 5 Hz just re-steers on the latest fix.
NAV_PERIOD_S = 0.2

#: A fix older than this is not steered on. NMEA receivers report at 1-10 Hz,
#: so 3 s is several missed fixes - a cable pulled or the receiver lost lock -
#: and the robot holds rather than driving on a position it no longer has.
FIX_TIMEOUT_S = 3.0


def parse_coord_pair(text):
    """Parses ``"lat,lon"`` into a ``(lat, lon)`` float pair."""
    parts = text.split(",")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError(
            f"expected LAT,LON but got {text!r}"
        )
    try:
        return float(parts[0]), float(parts[1])
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"expected LAT,LON but got {text!r}") from exc


def parse_waypoint_list(text):
    """Parses ``"lat,lon;lat,lon;..."`` into a list of waypoints."""
    waypoints = []
    for chunk in text.split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        lat, lon = parse_coord_pair(chunk)
        waypoints.append(Waypoint(lat, lon))
    return waypoints


class MissionRunner:
    """Binds a :class:`~gpsnav.navigator.Navigator` to GPS and motor hardware."""

    def __init__(
        self,
        navigator,
        gps=None,
        motor_serial=None,
        log_path=None,
        dry_run=False,
        gate=None,
        scanner=None,
        forward_offset_deg=0.0,
    ):
        self.navigator = navigator
        self.gps = gps
        self.motor_serial = motor_serial
        self.log_path = log_path
        self.dry_run = dry_run
        self.gate = gate
        self.scanner = scanner
        self.forward_offset_deg = float(forward_offset_deg)
        self.fix = None
        self.fix_time = None
        self.log_rows = []
        self._clock = time.monotonic

    def on_fix(self, fix):
        self.fix = fix
        self.fix_time = self._clock()
        self.navigator.update_heading(fix.course_degrees, fix.speed_mps)

    def fix_is_stale(self):
        """True when no fix has arrived within :data:`FIX_TIMEOUT_S`."""
        return self.fix_time is None or self._clock() - self.fix_time > FIX_TIMEOUT_S

    def apply_control(self, left, right):
        if self.dry_run or self.motor_serial is None:
            return 0, 0
        return apply_action_serial(self.motor_serial, left, right, gate=self.gate)

    def refresh_lidar(self):
        """Fold the newest revolution into octants and let the law re-evaluate.

        The scan arrives on its own UDP socket, so it never rides the navigation
        loop; each tick simply looks at whatever is newest. A missing scan
        leaves the gate at whatever the last report said.
        """
        if self.scanner is None or self.gate is None:
            return
        scan = self.scanner.latest_scan
        if scan is not None:
            self.gate.update_from_scan(scan, self.forward_offset_deg)

    def tick(self):
        """Runs one navigation step against the latest fix."""
        self.refresh_lidar()
        fix = self.fix
        if fix is None or not fix.has_position or self.fix_is_stale():
            # No usable position: hold. Sending nothing here would leave the
            # motors on the last command, driving blind on a fix that is gone.
            self.apply_control(0.0, 0.0)
            return None

        status = self.navigator.step(
            fix.latitude, fix.longitude, accuracy_meters=fix.accuracy_meters
        )
        self.apply_control(status.left, status.right)
        self._log(fix, status)
        return status

    def _log(self, fix, status):
        self.log_rows.append(
            {
                "timestamp": time.time(),
                "latitude": fix.latitude,
                "longitude": fix.longitude,
                "speed_mps": fix.speed_mps,
                "course_degrees": fix.course_degrees,
                "accuracy_meters": fix.accuracy_meters,
                "heading_degrees": self.navigator.heading_degrees,
                "mode": status.mode.value,
                "left": status.left,
                "right": status.right,
                "distance_meters": status.distance_meters,
                "bearing_degrees": status.bearing_degrees,
                "message": status.message,
                "avoidance": self.gate.status_text() if self.gate is not None else "",
            }
        )

    def write_log(self):
        """Writes the tick log as CSV next to the other run logs."""
        if not self.log_rows or not self.log_path:
            return None
        os.makedirs(self.log_path, exist_ok=True)
        now = datetime.now().strftime("%m%d%Y-%H%M%S")
        path = os.path.join(self.log_path, f"mission_log_{now}.csv")
        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(self.log_rows[0].keys()))
            writer.writeheader()
            writer.writerows(self.log_rows)
        return path

    def run(self, duration_s=None):
        """Runs the mission loop until it completes or is interrupted.

        Returns:
            int: process exit code (0 on a clean finish)
        """
        deadline = None if duration_s is None else time.time() + duration_s
        try:
            while self.navigator.is_navigating:
                if deadline is not None and time.time() >= deadline:
                    print("Time limit reached, stopping.")
                    break
                status = self.tick()
                if status is not None:
                    print(format_hud(status, self.gate), end="\r", flush=True)
                time.sleep(NAV_PERIOD_S)
        except KeyboardInterrupt:
            print("\nInterrupted, stopping.")
        finally:
            self.navigator.stop()
            stop_robot(self.motor_serial)
            if self.scanner is not None:
                self.scanner.stop()
            print()
        return 0


def format_status(status):
    """One-line HUD, in the spirit of Mission Planner's flight data strip."""
    parts = [status.mode.value.upper()]
    if status.active_waypoint_index is not None and status.mode is NavMode.MISSION:
        parts.append("wp %d" % (status.active_waypoint_index + 1))
    if status.distance_meters is not None:
        parts.append("%6.1f m" % status.distance_meters)
    if status.bearing_degrees is not None:
        parts.append("brg %03.0f" % status.bearing_degrees)
    if status.turn_error_degrees is not None:
        parts.append("err %+04.0f" % status.turn_error_degrees)
    parts.append("L%+.2f R%+.2f" % (status.left, status.right))
    if status.message:
        parts.append(status.message)
    return "  ".join(parts)


def format_hud(status, gate):
    """One-line HUD with the avoidance state folded in.

    ArduPilot's flight data strip shows proximity when it is acting; the same
    is worth a glance here, but only when the law is actually holding the car
    back, so a clear run keeps the strip short.
    """
    line = format_status(status)
    if gate is None:
        return line
    state = gate.status_text()
    if state in ("Holding", "Backing", "Blind"):
        return f"{line}  [{state}]"
    return line


def build_mission(args):
    """Assembles the mission from CLI flags, Mission Planner style."""
    mission = Mission()

    if args.mission_path:
        with open(args.mission_path, encoding="utf-8") as handle:
            mission = Mission.parse_waypoints_file(handle.read())

    for waypoint in args.add_waypoints:
        mission.add_waypoint(waypoint)

    if args.home:
        mission.set_home(*args.home)

    return mission


def main():
    parser = argparse.ArgumentParser(
        description="Run a GPS waypoint mission on the OpenBot Linux robot."
    )
    parser.add_argument(
        "--mission_path",
        "-f",
        default=None,
        help="Mission Planner .waypoints file (QGC WPL 110) to load.",
    )
    parser.add_argument(
        "--add_waypoints",
        "-w",
        type=parse_waypoint_list,
        default=[],
        help='Waypoints as "lat,lon;lat,lon;...", appended to the loaded mission.',
    )
    parser.add_argument(
        "--home",
        type=parse_coord_pair,
        default=None,
        help='Launch position as "lat,lon". Defaults to the file home or first fix.',
    )
    parser.add_argument(
        "--goto",
        type=parse_coord_pair,
        default=None,
        help='GUIDED mode: drive to a single "lat,lon" and stop.',
    )
    parser.add_argument(
        "--rtl",
        action="store_true",
        help="RTL mode: drive back to the home position.",
    )
    parser.add_argument(
        "--gps_device",
        default="/dev/ttyUSB1",
        help="Serial device of the NMEA GPS receiver.",
    )
    parser.add_argument(
        "--gps_baudrate",
        type=int,
        default=9600,
        help="Baudrate of the NMEA GPS receiver.",
    )
    parser.add_argument(
        "--motor_device",
        default="/dev/ttyUSB0",
        help="Serial device of the Arduino motor driver.",
    )
    parser.add_argument(
        "--log_path",
        "-l",
        default=f"{CUR_DIR}/logs/",
        help="Folder for the per-run CSV log.",
    )
    parser.add_argument(
        "--export_kml",
        default=None,
        help="Write the plan and driven track to this KML file on exit.",
    )
    parser.add_argument(
        "--save_mission",
        default=None,
        help="Write the assembled mission to this .waypoints file and exit.",
    )
    parser.add_argument(
        "--dry_run",
        action="store_true",
        help="Navigate and print the HUD without touching the motors.",
    )
    parser.add_argument(
        "--avoid_margin",
        type=float,
        default=1.0,
        help="Object-avoidance standoff in metres (ArduPilot AVOID_MARGIN). "
        "0 disables the law.",
    )
    parser.add_argument(
        "--avoid_backup",
        type=float,
        default=0.0,
        help="Reverse command used to back away from an obstacle "
        "(ArduPilot AVOID_BACKUP_SPD). 0 disables backing away.",
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
        help="LiDAR mounting offset in degrees clockwise from the car's nose, "
        "matching the firmware's LIDAR_FORWARD_DEG.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Stop after this many seconds (safety net for unattended runs).",
    )
    args = parser.parse_args()

    navigator = Navigator(mission=build_mission(args))

    if args.save_mission:
        with open(args.save_mission, "w", encoding="utf-8") as handle:
            handle.write(navigator.save_mission_file())
        print(f"Wrote mission to {args.save_mission}")
        return 0

    motor_serial = (
        None if args.dry_run else get_arduino_serial(args.motor_device)
    )

    # Object avoidance, ArduPilot Rover style: a single gate on the drive path
    # fed by the LiDAR's UDP stream. The quick-disable is --no_avoid, matching
    # ArduPilot's insistence on an RCx_OPTION = 40 switch before first use.
    proximity = Proximity()
    gate = DriveGate(
        proximity=proximity,
        margin_meters=args.avoid_margin,
        backup_speed=args.avoid_backup,
        enabled=not args.no_avoid and args.avoid_margin > 0,
    )
    scanner = None
    if not args.no_lidar:
        scanner = LidarScanner()
        if not scanner.start(port=args.lidar_port):
            print(
                f"Could not bind UDP port {args.lidar_port} for LiDAR scans; "
                "continuing without a ranging source."
            )
            scanner = None
        else:
            print(f"Listening for LiDAR scans on UDP {args.lidar_port}")

    runner = MissionRunner(
        navigator,
        motor_serial=motor_serial,
        log_path=args.log_path,
        dry_run=args.dry_run,
        gate=gate,
        scanner=scanner,
        forward_offset_deg=args.lidar_forward_deg,
    )
    # Started only when the receiver is present; fixes land in runner.on_fix.
    runner.gps = Gps(
        callback=runner.on_fix,
        device_address=args.gps_device,
        baudrate=args.gps_baudrate,
    )

    if not args.dry_run and motor_serial is None:
        print("No motor controller found. Use --dry_run to navigate without one.")
        return 1

    print("Waiting 2s for the GPS thread to start")
    time.sleep(2)

    if runner.fix is not None and not navigator.mission.has_home:
        # Home is where the robot is standing when the run starts.
        navigator.mark_home(runner.fix.latitude, runner.fix.longitude)
        print(
            "Home set to %.7f, %.7f" % (runner.fix.latitude, runner.fix.longitude)
        )

    if args.goto:
        navigator.start_guided(*args.goto)
        print(f"GUIDED to {args.goto[0]:.7f}, {args.goto[1]:.7f}")
    elif args.rtl:
        if not navigator.start_rtl():
            print("RTL needs a home position. Pass --home or load a mission file.")
            return 1
        print("RTL to home")
    else:
        if not navigator.start_mission():
            print(
                "No waypoints. Pass --mission_path, --add_waypoints or --goto."
            )
            return 1
        print(f"MISSION with {len(navigator.mission)} waypoint(s)")

    exit_code = runner.run(duration_s=args.duration)

    log_file = runner.write_log()
    if log_file:
        print(f"Wrote mission log to {log_file}")

    if args.export_kml:
        navigator.save_kml(args.export_kml)
        print(f"Wrote KML to {args.export_kml}")

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
