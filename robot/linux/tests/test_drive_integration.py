"""Integration tests for the drive path: plan view rendering, the mission
runner's lost-fix hold and the avoidance gate on every entry point.

No hardware: serial ports are replaced with an in-memory writer and the LiDAR
with hand-built scans.
"""

import os
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from gpsnav import Mission, Navigator, Waypoint  # noqa: E402
from lidar import DriveGate, LidarScan, Proximity  # noqa: E402
from lidar.plan_view import close_seam, scan_to_points, split_strokes  # noqa: E402
import run_mission  # noqa: E402
from motors import apply_action_serial  # noqa: E402


class FakeSerial:
    def __init__(self):
        self.lines = []

    def write(self, data):
        self.lines.append(data.decode("utf-8"))


def _scan(ranges):
    return LidarScan.parse(LidarScan.encode(1, ranges))


def _wall_ahead_scan(mm=400):
    # 360 bins, everything clear except a wall across the nose.
    ranges = [0] * 360
    for b in list(range(0, 20)) + list(range(340, 360)):
        ranges[b] = mm
    return _scan(ranges)


# region plan view


def test_draw_plan_view_renders_a_frame():
    pytest.importorskip("cv2")
    from lidar.plan_view import draw_plan_view

    image, strokes = draw_plan_view(_scan([1000] * 360), size=200)
    assert image.shape == (200, 200, 3)
    assert len(strokes) == 1  # a full ring is one closed outline


def test_draw_plan_view_without_a_scan():
    pytest.importorskip("cv2")
    from lidar.plan_view import draw_plan_view

    image, strokes = draw_plan_view(None, size=120)
    assert image.shape == (120, 120, 3)
    assert strokes == []


def test_seam_joins_a_wall_across_the_zero_mark():
    scan = _wall_ahead_scan()
    points = scan_to_points(scan)
    strokes = split_strokes(points)
    assert len(strokes) == 2  # bins 0..19 and 340..359
    joined = close_seam(strokes, points, scan.bin_count)
    assert len(joined) == 1
    assert len(joined[0]) == 40


def test_seam_left_open_when_an_end_has_no_return():
    ranges = [1000] * 360
    ranges[0] = 0
    ranges[180] = 0
    scan = _scan(ranges)
    points = scan_to_points(scan)
    strokes = split_strokes(points)
    assert close_seam(strokes, points, scan.bin_count) == strokes


# endregion

# region drive gate on the serial chokepoint


def test_gate_stops_forward_into_a_wall_on_the_wire():
    gate = DriveGate(proximity=Proximity(), margin_meters=1.0)
    gate.update_from_scan(_wall_ahead_scan(400))
    serial = FakeSerial()
    left, right = apply_action_serial(serial, 1.0, 1.0, gate=gate)
    assert left + right == 0
    assert serial.lines[-1] == "c0,0\n"


def test_gate_lets_reverse_out_of_a_wall():
    gate = DriveGate(proximity=Proximity(), margin_meters=1.0)
    gate.update_from_scan(_wall_ahead_scan(400))
    serial = FakeSerial()
    left, right = apply_action_serial(serial, -0.5, -0.5, gate=gate)
    assert (left, right) == (-127, -127)


def test_infer_apply_action_goes_through_the_gate():
    infer = pytest.importorskip("infer", reason="needs the inference stack")
    gate = DriveGate(proximity=Proximity(), margin_meters=1.0)
    gate.update_from_scan(_wall_ahead_scan(400))
    serial = FakeSerial()
    fake = SimpleNamespace(run_mode="joystick", serial=serial, gate=gate)
    infer.InferNetwork.apply_action(fake, 1.0, 1.0)
    assert serial.lines[-1] == "c0,0\n"


# endregion

# region mission runner lost-fix hold


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


def _fix(lat, lon):
    return SimpleNamespace(
        latitude=lat,
        longitude=lon,
        has_position=True,
        accuracy_meters=1.0,
        course_degrees=0.0,
        speed_mps=1.0,
    )


def _runner():
    mission = Mission()
    mission.set_home(10.0, 106.0)
    mission.add_waypoint(Waypoint(10.001, 106.0))
    navigator = Navigator(mission=mission)
    serial = FakeSerial()
    runner = run_mission.MissionRunner(navigator, motor_serial=serial)
    clock = FakeClock()
    runner._clock = clock
    return runner, serial, clock, navigator


def test_runner_holds_before_the_first_fix():
    runner, serial, _clock, navigator = _runner()
    navigator.start_mission()
    assert runner.tick() is None
    assert serial.lines[-1] == "c0,0\n"


def test_runner_holds_when_the_fix_goes_stale():
    runner, serial, clock, navigator = _runner()
    navigator.start_mission()
    runner.on_fix(_fix(10.0, 106.0))
    runner.on_fix(_fix(10.00001, 106.0))
    status = runner.tick()
    assert status is not None
    clock.now += run_mission.FIX_TIMEOUT_S + 0.1
    assert runner.tick() is None
    assert serial.lines[-1] == "c0,0\n"


# endregion
