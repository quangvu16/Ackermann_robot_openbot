"""Tests for the LiDAR scan codec, proximity octants and object avoidance.

Pure-python, no hardware and no display required. Mirrors the Android
``LidarScanTest`` / ``ObjectAvoidanceTest`` so the geometry and the stop law
behave identically on every platform.

Staleness is tested with an injected clock rather than ``sleep``, so the suite
stays fast and cannot flake on a loaded machine.
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from lidar import (  # noqa: E402
    FRONT,
    FRONT_LEFT,
    FRONT_RIGHT,
    HEADER_LEN,
    MAX_BINS,
    SECTORS,
    DriveGate,
    LidarScan,
    Proximity,
    apply,
    evaluate,
    feed_from_scan,
    format_status,
    parse_sectors_cm,
    scan_to_points,
    split_strokes,
    wrap_deg,
)
from lidar.scanner import LidarScanner  # noqa: E402

INFINITY = float("inf")


class FakeClock:
    """A clock the tests can advance without sleeping."""

    def __init__(self, start=1000.0):
        self.now = start

    def __call__(self):
        return self.now

    def advance(self, ms):
        self.now += ms


def make_scan(ranges_mm, scan_id=1):
    """Build a scan the way the firmware would have sent it."""
    datagram = LidarScan.encode(scan_id, ranges_mm)
    return LidarScan.parse(datagram)


# ---------------------------------------------------------------------------
# scan codec
# ---------------------------------------------------------------------------


class TestLidarScanParse:
    def test_round_trip_preserves_bins_and_id(self):
        ranges = [100, 200, 300, 0, 500]
        datagram = LidarScan.encode(42, ranges)
        scan = LidarScan.parse(datagram)
        assert scan is not None
        assert scan.bin_count == 5
        assert scan.scan_id == 42
        assert scan.ranges_mm == ranges

    def test_rejects_none(self):
        assert LidarScan.parse(None) is None

    def test_rejects_short_datagram(self):
        assert LidarScan.parse(b"LDS1") is None
        assert LidarScan.parse(b"") is None

    def test_rejects_wrong_magic(self):
        datagram = bytearray(LidarScan.encode(1, [100, 200]))
        datagram[0] = ord("X")
        assert LidarScan.parse(bytes(datagram)) is None

    def test_rejects_zero_bins(self):
        datagram = b"LDS1" + b"\x00\x00" + b"\x00\x00\x00\x00" + b"\x00\x00\x00\x00"
        assert LidarScan.parse(datagram) is None

    def test_rejects_bin_count_past_max(self):
        datagram = bytearray(LidarScan.encode(1, [1] * 4))
        datagram[4] = (MAX_BINS + 1) & 0xFF
        datagram[5] = ((MAX_BINS + 1) >> 8) & 0xFF
        assert LidarScan.parse(bytes(datagram)) is None

    def test_rejects_truncated_ranges(self):
        full = LidarScan.encode(1, [100, 200, 300])
        assert LidarScan.parse(full[: HEADER_LEN + 2]) is None

    def test_uses_length_not_buffer_capacity(self):
        """An oversized receive buffer must not turn stale bytes into ranges."""
        full = LidarScan.encode(1, [100, 200, 300, 400])
        padded = full + b"\xff" * 64
        scan = LidarScan.parse(padded, length=len(full))
        assert scan is not None
        assert scan.ranges_mm == [100, 200, 300, 400]

    def test_accepts_max_bins(self):
        ranges = [1] * MAX_BINS
        scan = LidarScan.parse(LidarScan.encode(1, ranges))
        assert scan is not None
        assert scan.bin_count == MAX_BINS

    def test_full_16bit_range(self):
        scan = make_scan([0xFFFF, 1])
        assert scan.get_range_mm(0) == 0xFFFF


class TestLidarScanGeometry:
    def test_zero_means_no_return_not_obstacle_at_zero(self):
        scan = make_scan([0, 500])
        assert not scan.has_return(0)
        assert scan.get_range_meters(0) == INFINITY
        assert scan.get_range_meters(1) == pytest.approx(0.5)
        assert scan.valid_points == 1

    def test_out_of_range_bin_is_safe(self):
        scan = make_scan([100])
        assert scan.get_range_mm(-1) == 0
        assert scan.get_range_mm(1) == 0
        assert scan.get_range_meters(5) == INFINITY

    def test_bin_angles_are_centred(self):
        scan = make_scan([0] * 360)
        assert scan.get_bin_angle_deg(0) == pytest.approx(0.5)
        assert scan.get_bin_angle_deg(180) == pytest.approx(180.5)
        assert scan.get_bin_angle_deg(359) == pytest.approx(359.5)

    def test_forward_offset_rotates_vehicle_angle(self):
        scan = make_scan([0] * 360)
        assert scan.get_vehicle_angle_deg(0, 0.0) == pytest.approx(0.5)
        assert scan.get_vehicle_angle_deg(0, 10.0) == pytest.approx(350.5)
        assert scan.get_vehicle_angle_deg(10, 10.0) == pytest.approx(0.5)

    def test_wrap_deg(self):
        assert wrap_deg(0) == 0
        assert wrap_deg(360) == 0
        assert wrap_deg(-10) == pytest.approx(350)
        assert wrap_deg(370) == pytest.approx(10)

    def test_sector_folding_picks_nearest(self):
        # Bin 0 at 0.5 deg and bin 1 at 1.5 deg are both in the front octant;
        # bin 90 at 90.5 deg lands in the right octant (centred on 90 deg).
        ranges = [0] * 360
        ranges[0] = 2000
        ranges[1] = 1500  # nearer, same front sector
        ranges[90] = 3000
        scan = make_scan(ranges)
        sectors = scan.compute_sectors_mm(8, 0.0)
        assert sectors[FRONT] == 1500
        assert sectors[2] == 3000
        assert sectors[4] == 0

    def test_sector_folding_honours_offset(self):
        ranges = [0] * 360
        ranges[0] = 1000  # 0.5 deg on the LiDAR, 90.5 deg on the car after offset
        scan = make_scan(ranges)
        sectors = scan.compute_sectors_mm(8, -90.0)
        assert sectors[2] == 1000  # right
        assert sectors[FRONT] == 0

    def test_sector_folding_keeps_empty_sectors_at_zero(self):
        scan = make_scan([0] * 360)
        sectors = scan.compute_sectors_mm(8, 0.0)
        assert sectors == [0] * 8

    def test_sector_folding_accepts_fewer_sectors(self):
        ranges = [0] * 360
        ranges[0] = 800
        scan = make_scan(ranges)
        assert scan.compute_sectors_mm(4, 0.0) == [800, 0, 0, 0]

    def test_sector_folding_never_returns_null(self):
        scan = make_scan([0] * 4)
        assert scan.compute_sectors_mm(8, 0.0) == [0] * 8
        assert scan.compute_sectors_mm(0, 0.0) == [0]  # floored to 1


# ---------------------------------------------------------------------------
# proximity
# ---------------------------------------------------------------------------


class TestProximityParse:
    def test_parses_eight_values(self):
        assert parse_sectors_cm("10,20,30,40,50,60,70,80") == [
            10, 20, 30, 40, 50, 60, 70, 80
        ]

    def test_rejects_wrong_arity(self):
        assert parse_sectors_cm("1,2,3") is None
        assert parse_sectors_cm("1,2,3,4,5,6,7,8,9") is None

    def test_rejects_malformed(self):
        assert parse_sectors_cm("1,2,3,4,5,6,7,x") is None
        assert parse_sectors_cm(None) is None
        assert parse_sectors_cm("") is None


class TestProximity:
    def test_zero_cm_is_clear_not_obstacle_at_zero(self):
        clock = FakeClock()
        prox = Proximity(clock=clock)
        assert prox.set_sectors_cm([0] * 8)
        assert prox.get_sectors_m() == [INFINITY] * 8
        assert prox.get_sector_m(FRONT) == INFINITY

    def test_converts_cm_to_m(self):
        prox = Proximity(clock=FakeClock())
        prox.set_sectors_cm([100, 0, 250, 0, 0, 0, 0, 0])
        assert prox.get_sector_m(FRONT) == pytest.approx(1.0)
        assert prox.get_sector_m(1) == INFINITY
        assert prox.get_sector_m(2) == pytest.approx(2.5)

    def test_rejects_wrong_arity_without_clobbering(self):
        prox = Proximity(clock=FakeClock())
        prox.set_sectors_cm([100] * 8)
        assert not prox.set_sectors_cm([1, 2, 3])
        assert prox.get_sector_m(FRONT) == pytest.approx(1.0)

    def test_rejects_negative_and_nan(self):
        prox = Proximity(clock=FakeClock())
        assert not prox.set_sectors_cm([-1] + [0] * 7)
        assert not prox.set_sectors_cm([float("nan")] + [0] * 7)
        assert not prox.set_sectors_cm(None)

    def test_state_is_never_before_first_report(self):
        prox = Proximity(clock=FakeClock())
        assert not prox.has_ever_reported()
        assert prox.get_data_state() == "never"
        assert prox.get_age_ms() == 1000

    def test_state_goes_stale_when_reports_stop(self):
        clock = FakeClock()
        prox = Proximity(clock=clock)
        prox.set_sectors_cm([100] * 8)
        assert prox.get_data_state() == "fresh"
        clock.advance(499)
        assert prox.get_data_state() == "fresh"
        clock.advance(2)
        assert prox.get_data_state() == "stale"
        assert prox.has_ever_reported()

    def test_age_is_time_since_last_report_not_gap(self):
        """A dead sensor must not look permanently fresh."""
        clock = FakeClock()
        prox = Proximity(clock=clock)
        prox.set_sectors_cm([100] * 8)
        clock.advance(100)
        prox.set_sectors_cm([100] * 8)
        clock.advance(600)
        assert prox.get_age_ms() == pytest.approx(600)
        assert prox.get_data_state() == "stale"

    def test_reset_returns_to_never(self):
        prox = Proximity(clock=FakeClock())
        prox.set_sectors_cm([100] * 8)
        prox.reset()
        assert prox.get_data_state() == "never"
        assert prox.get_sectors_m() == [INFINITY] * 8

    def test_out_of_range_sector_is_clear(self):
        prox = Proximity(clock=FakeClock())
        assert prox.get_sector_m(-1) == INFINITY
        assert prox.get_sector_m(8) == INFINITY


# ---------------------------------------------------------------------------
# avoidance law
# ---------------------------------------------------------------------------


class TestObjectAvoidance:
    def test_clear_road_passes_through(self):
        d = evaluate(0.5, 0.5, [INFINITY] * 8, "fresh", 1.0, 0.0, True)
        assert d.left == pytest.approx(0.5)
        assert d.right == pytest.approx(0.5)
        assert not d.blocked
        assert not d.backing
        assert not d.is_holding()

    def test_disabled_passes_through(self):
        d = evaluate(0.5, 0.5, [0.2] * 8, "fresh", 1.0, 0.0, False)
        assert d.left == pytest.approx(0.5)
        assert not d.blocked

    def test_zero_margin_passes_through(self):
        d = evaluate(0.5, 0.5, [0.2] * 8, "fresh", 0.0, 0.0, True)
        assert not d.blocked

    def test_never_fitted_never_blocks(self):
        """A robot with no LiDAR must stay drivable."""
        d = evaluate(0.5, 0.5, [INFINITY] * 8, "never", 1.0, 0.0, True)
        assert d.left == pytest.approx(0.5)
        assert not d.blocked

    def test_stale_blocks_forward_keeps_reverse(self):
        d = evaluate(0.5, 0.5, [INFINITY] * 8, "stale", 1.0, 0.0, True)
        assert d.blocked
        assert not d.backing
        assert d.is_holding()
        d = evaluate(-0.5, -0.5, [INFINITY] * 8, "stale", 1.0, 0.0, True)
        assert not d.blocked
        assert d.left == pytest.approx(-0.5)

    def test_front_obstacle_blocks_forward(self):
        sectors = [INFINITY] * 8
        sectors[FRONT] = 0.4
        d = evaluate(0.5, 0.5, sectors, "fresh", 1.0, 0.0, True)
        assert d.blocked
        assert not d.backing
        assert d.nearest_sector == FRONT
        assert d.nearest_meters == pytest.approx(0.4)

    def test_front_arc_covers_corners(self):
        for sector in (FRONT_LEFT, FRONT_RIGHT):
            sectors = [INFINITY] * 8
            sectors[sector] = 0.3
            d = evaluate(0.5, 0.5, sectors, "fresh", 1.0, 0.0, True)
            assert d.blocked, f"sector {sector} should block forward motion"

    def test_rear_obstacle_blocks_reverse(self):
        sectors = [INFINITY] * 8
        sectors[4] = 0.3
        d = evaluate(-0.5, -0.5, sectors, "fresh", 1.0, 0.0, True)
        assert d.blocked
        assert d.nearest_sector == 4

    def test_rear_obstacle_does_not_block_forward(self):
        sectors = [INFINITY] * 8
        sectors[4] = 0.3
        d = evaluate(0.5, 0.5, sectors, "fresh", 1.0, 0.0, True)
        assert not d.blocked

    def test_blocking_keeps_steering(self):
        sectors = [INFINITY] * 8
        sectors[FRONT] = 0.2
        d = evaluate(0.8, 0.2, sectors, "fresh", 1.0, 0.0, True)
        assert d.blocked
        # throttle zeroed, steering (left-right) preserved as +/- half
        assert d.left == pytest.approx(0.3)
        assert d.right == pytest.approx(-0.3)

    def test_backs_away_when_enabled_and_rear_clear(self):
        sectors = [INFINITY] * 8
        sectors[FRONT] = 0.3
        d = evaluate(0.5, 0.5, sectors, "fresh", 1.0, 0.4, True)
        assert d.blocked
        assert d.backing
        assert d.left == pytest.approx(-0.4)
        assert d.right == pytest.approx(-0.4)

    def test_does_not_back_into_a_wall(self):
        sectors = [INFINITY] * 8
        sectors[FRONT] = 0.3
        sectors[4] = 0.3
        d = evaluate(0.5, 0.5, sectors, "fresh", 1.0, 0.4, True)
        assert d.blocked
        assert not d.backing
        assert d.left == pytest.approx(0.0)
        assert d.right == pytest.approx(0.0)

    def test_no_return_is_not_an_obstacle(self):
        """0 on the wire means clear; it must never read as an obstacle at zero."""
        d = evaluate(0.5, 0.5, [0.0] * 8, "fresh", 1.0, 0.0, True)
        assert not d.blocked

    def test_commands_are_clamped(self):
        d = evaluate(5.0, -5.0, [INFINITY] * 8, "fresh", 1.0, 0.0, True)
        assert d.left == pytest.approx(1.0)
        assert d.right == pytest.approx(-1.0)

    def test_apply_uses_proximity_state(self):
        clock = FakeClock()
        prox = Proximity(clock=clock)
        prox.set_sectors_cm([40] + [0] * 7)  # 40 cm ahead, inside a 1 m margin
        d = apply(0.5, 0.5, prox, 1.0, 0.0, True)
        assert d.blocked
        clock.advance(600)
        d = apply(0.5, 0.5, prox, 1.0, 0.0, True)
        assert d.blocked  # stale still blocks forward


# ---------------------------------------------------------------------------
# drive gate
# ---------------------------------------------------------------------------


class TestDriveGate:
    def test_no_proximity_passes_through(self):
        gate = DriveGate()
        d = gate.apply(0.6, 0.6)
        assert d.left == pytest.approx(0.6)
        assert not d.blocked

    def test_stores_requested_control(self):
        gate = DriveGate()
        gate.apply(0.4, -0.2)
        assert gate.control == (0.4, -0.2)

    def test_stored_request_re_evaluated_on_proximity_update(self):
        """A cleared blockage releases without the driver re-moving the stick."""
        clock = FakeClock()
        prox = Proximity(clock=clock)
        prox.set_sectors_cm([30] + [0] * 7)
        gate = DriveGate(proximity=prox, margin_meters=1.0)
        d = gate.apply(0.5, 0.5)
        assert d.blocked
        prox.set_sectors_cm([0] * 8)  # the way ahead clears
        d = gate.on_proximity_update()
        assert not d.blocked
        assert d.left == pytest.approx(0.5)

    def test_quick_disable_releases_immediately(self):
        prox = Proximity(clock=FakeClock())
        prox.set_sectors_cm([30] + [0] * 7)
        gate = DriveGate(proximity=prox, margin_meters=1.0, enabled=True)
        gate.apply(0.5, 0.5)
        d = gate.set_enabled(False)
        assert not d.blocked
        assert d.left == pytest.approx(0.5)

    def test_update_from_scan_feeds_proximity(self):
        ranges = [0] * 360
        ranges[0] = 400  # 40 cm dead ahead
        scan = make_scan(ranges)
        prox = Proximity(clock=FakeClock())
        gate = DriveGate(proximity=prox, margin_meters=1.0)
        gate.apply(0.5, 0.5)  # request forward motion first; (0, 0) never blocks
        d = gate.update_from_scan(scan)
        assert d is not None
        assert d.blocked
        assert gate.status_text() == "Holding"

    def test_status_text(self):
        clock = FakeClock()
        prox = Proximity(clock=clock)
        gate = DriveGate(proximity=prox, margin_meters=1.0)
        assert gate.status_text() == "Idle"
        gate.set_enabled(False)
        assert gate.status_text() == "Off"
        gate.set_enabled(True)
        prox.set_sectors_cm([0] * 8)
        assert gate.status_text() == "Clear"
        clock.advance(600)
        assert gate.status_text() == "Blind"

    def test_feed_from_scan_none_safe(self):
        prox = Proximity(clock=FakeClock())
        assert not feed_from_scan(prox, None)
        assert not feed_from_scan(None, make_scan([100]))

    def test_feed_from_scan_converts_mm_to_cm(self):
        ranges = [0] * 360
        ranges[0] = 1500  # 150 cm
        scan = make_scan(ranges)
        prox = Proximity(clock=FakeClock())
        assert feed_from_scan(prox, scan)
        assert prox.get_sector_m(FRONT) == pytest.approx(1.5)


# ---------------------------------------------------------------------------
# plan view geometry
# ---------------------------------------------------------------------------


class TestPlanView:
    def test_no_scan_gives_no_points(self):
        assert scan_to_points(None) == []

    def test_zero_range_max_is_safe(self):
        assert scan_to_points(make_scan([100]), range_max_meters=0) == []

    def test_omits_bins_without_returns(self):
        scan = make_scan([0, 1000, 0])
        points = scan_to_points(scan)
        assert len(points) == 1
        assert points[0][2] == 1

    def test_nose_is_up(self):
        """Bin 0 sits at 0.5 deg from the LiDAR zero, i.e. essentially straight up."""
        scan = make_scan([1000] + [0] * 359)
        x, y, _ = scan_to_points(scan)[0]
        assert y < 0
        assert abs(x) < 0.05

    def test_range_is_clamped_to_outer_circle(self):
        scan = make_scan([100000] + [0] * 359)  # 100 m, way past 8 m max
        x, y, _ = scan_to_points(scan)[0]
        r = math.hypot(x, y)
        assert r == pytest.approx(0.5, abs=1e-6)

    def test_forward_offset_rotates_the_picture(self):
        scan = make_scan([1000] + [0] * 359)
        x0, y0, _ = scan_to_points(scan, forward_offset_deg=0.0)[0]
        x1, y1, _ = scan_to_points(scan, forward_offset_deg=90.0)[0]
        assert (x0, y0) != (x1, y1)
        # A positive offset means the LiDAR's zero sits 90 deg clockwise of the
        # nose, so that return is 90 deg counter-clockwise of the nose: left.
        assert x1 < 0
        assert abs(y1) < 0.05

    def test_split_strokes_lifts_the_pen_on_gaps(self):
        points = [
            (0.0, 0.0, 0),
            (0.1, 0.0, 1),
            (0.5, 0.0, 3),  # gap: bin 2 had no return
            (0.6, 0.0, 4),
        ]
        strokes = split_strokes(points)
        assert len(strokes) == 2
        assert strokes[0] == [(0.0, 0.0), (0.1, 0.0)]
        assert strokes[1] == [(0.5, 0.0), (0.6, 0.0)]

    def test_split_strokes_empty(self):
        assert split_strokes([]) == []

    def test_single_point_is_a_stroke_of_one(self):
        assert split_strokes([(0.1, 0.2, 0)]) == [[(0.1, 0.2)]]

    def test_format_status(self):
        assert format_status(None, True) == "Waiting for scans"
        scan = make_scan([100, 0, 300])
        assert format_status(scan, True) == "3 bins · 2 returns · live"
        assert format_status(scan, False) == "3 bins · 2 returns · stale"


# ---------------------------------------------------------------------------
# scanner (no socket required - just the state machine)
# ---------------------------------------------------------------------------


class TestScannerState:
    def test_never_reported_is_stale_but_not_fresh(self):
        scanner = LidarScanner(clock=FakeClock())
        assert not scanner.has_ever_reported()
        assert not scanner.is_fresh
        assert not scanner.is_stale
        assert scanner.get_age_ms() == 1000

    def test_latest_scan_none_until_accepted(self):
        scanner = LidarScanner(clock=FakeClock())
        assert scanner.latest_scan is None
        assert scanner.get_latest_sectors_mm(8) is None

    def test_start_and_stop_are_idempotent(self):
        scanner = LidarScanner(clock=FakeClock())
        # Binding a privileged port would fail; use a high port and stop twice.
        started = scanner.start(port=0)
        if started:
            assert scanner.is_running
        scanner.stop()
        scanner.stop()
        assert not scanner.is_running
