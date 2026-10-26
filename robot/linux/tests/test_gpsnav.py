"""Tests for the GPS mission navigation core.

Pure-python, no hardware required. Mirrors the Android ``gpsNav`` test suite so
the guidance law, mission codec and geo math behave identically on every
platform.
"""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from gpsnav import (  # noqa: E402
    GuidanceController,
    Mission,
    Waypoint,
    bearing_degrees,
    destination_point,
    distance_meters,
    turn_error_degrees,
    wrap_degrees,
)

# Keep tolerances in metres / degrees identical to the Android tests.
TOLERANCE_M = 1.0
TOLERANCE_DEG = 0.01


# ---------------------------------------------------------------------------
# geo
# ---------------------------------------------------------------------------


class TestGeoMath:
    def test_distance_is_zero_for_same_point(self):
        assert distance_meters(47.397742, 8.545594, 47.397742, 8.545594) == pytest.approx(0.0, abs=1e-6)

    def test_one_degree_of_latitude_is_about_111km(self):
        assert distance_meters(0.0, 0.0, 1.0, 0.0) == pytest.approx(111194.9, abs=50.0)

    def test_distance_is_symmetric(self):
        forward = distance_meters(47.39, 8.54, 47.40, 8.55)
        backward = distance_meters(47.40, 8.55, 47.39, 8.54)
        assert forward == pytest.approx(backward, abs=1e-6)

    def test_cardinal_bearings(self):
        assert bearing_degrees(0.0, 0.0, 1.0, 0.0) == pytest.approx(0.0, abs=TOLERANCE_DEG)
        assert bearing_degrees(0.0, 0.0, 0.0, 1.0) == pytest.approx(90.0, abs=TOLERANCE_DEG)
        assert bearing_degrees(0.0, 0.0, 0.0, -1.0) == pytest.approx(-90.0, abs=TOLERANCE_DEG)
        # Due south is antipodal in longitude terms; atan2 gives -180.
        assert bearing_degrees(0.0, 0.0, -1.0, 0.0) == pytest.approx(-180.0, abs=TOLERANCE_DEG)

    def test_wrap_degrees_stays_in_range(self):
        assert wrap_degrees(0.0) == pytest.approx(0.0, abs=1e-9)
        assert wrap_degrees(370.0) == pytest.approx(10.0, abs=1e-9)
        assert wrap_degrees(-370.0) == pytest.approx(-10.0, abs=1e-9)
        assert wrap_degrees(180.0) == pytest.approx(-180.0, abs=1e-9)
        assert wrap_degrees(170.0) == pytest.approx(170.0, abs=1e-9)

        for angle in (-720, -360, -181, -180, -90, -1, 0, 1, 90, 179, 180, 359, 720):
            wrapped = wrap_degrees(angle)
            assert -180.0 <= wrapped < 180.0, "wrap_degrees(%r) = %r" % (angle, wrapped)

    def test_turn_error_is_positive_when_target_is_to_the_right(self):
        # Facing north (0 deg), target to the east (90 deg) -> turn right by 90.
        assert turn_error_degrees(0.0, 90.0) == pytest.approx(90.0, abs=TOLERANCE_DEG)
        # Facing north, target to the west -> turn left.
        assert turn_error_degrees(0.0, -90.0) == pytest.approx(-90.0, abs=TOLERANCE_DEG)

    def test_turn_error_picks_short_way_around(self):
        # Facing 170 deg, target at -170 deg: 20 deg to the right, not 340 left.
        assert turn_error_degrees(170.0, -170.0) == pytest.approx(20.0, abs=TOLERANCE_DEG)
        # Facing -170 deg, target at 170 deg: 20 deg to the left.
        assert turn_error_degrees(-170.0, 170.0) == pytest.approx(-20.0, abs=TOLERANCE_DEG)

    def test_destination_round_trips_with_bearing_and_distance(self):
        start_lat, start_lon = 47.397742, 8.545594
        bearing, distance = 37.0, 250.0

        end_lat, end_lon = destination_point(start_lat, start_lon, bearing, distance)
        assert distance_meters(start_lat, start_lon, end_lat, end_lon) == pytest.approx(
            distance, abs=TOLERANCE_M
        )
        assert bearing_degrees(start_lat, start_lon, end_lat, end_lon) == pytest.approx(
            bearing, abs=TOLERANCE_DEG
        )


# ---------------------------------------------------------------------------
# guidance
# ---------------------------------------------------------------------------

ROBOT_LAT = 0.0
ROBOT_LON = 0.0


def target_at(bearing, distance):
    lat, lon = destination_point(ROBOT_LAT, ROBOT_LON, bearing, distance)
    return Waypoint(lat, lon)


class TestGuidanceController:
    def setup_method(self, _):
        self.controller = GuidanceController()

    def test_target_straight_ahead_drives_both_wheels_equally(self):
        result = self.controller.compute(ROBOT_LAT, ROBOT_LON, 0.0, target_at(0.0, 20.0))

        assert not result.arrived
        assert result.turn_error_degrees == pytest.approx(0.0, abs=0.5)
        assert result.left == pytest.approx(result.right, abs=1e-4)
        assert result.left > 0.1

    def test_target_to_the_right_leads_with_left_wheel(self):
        result = self.controller.compute(ROBOT_LAT, ROBOT_LON, 0.0, target_at(30.0, 20.0))

        assert result.turn_error_degrees > 0
        assert result.left > result.right

    def test_target_to_the_left_leads_with_right_wheel(self):
        result = self.controller.compute(ROBOT_LAT, ROBOT_LON, 0.0, target_at(-30.0, 20.0))

        assert result.turn_error_degrees < 0
        assert result.right > result.left

    def test_large_heading_error_pivots_in_place(self):
        result = self.controller.compute(ROBOT_LAT, ROBOT_LON, 0.0, target_at(90.0, 20.0))

        # Hard right: left wheel forward, right wheel backward.
        assert result.left > 0.0
        assert result.right < 0.0
        assert result.left + result.right == pytest.approx(0.0, abs=1e-3)

    def test_large_negative_heading_error_pivots_the_other_way(self):
        result = self.controller.compute(ROBOT_LAT, ROBOT_LON, 0.0, target_at(-90.0, 20.0))

        assert result.left < 0.0
        assert result.right > 0.0
        assert result.left + result.right == pytest.approx(0.0, abs=1e-3)

    def test_inside_acceptance_radius_arrives_and_stops(self):
        result = self.controller.compute(ROBOT_LAT, ROBOT_LON, 0.0, target_at(0.0, 1.0))

        assert result.arrived
        assert result.left == 0.0
        assert result.right == 0.0

    def test_waypoint_acceptance_radius_overrides_default(self):
        loose = Waypoint(0.0, 0.0, "", 20.0)
        result = self.controller.compute(ROBOT_LAT, ROBOT_LON, 0.0, loose)

        assert result.arrived

    def test_slows_down_when_close_to_the_waypoint(self):
        far = self.controller.compute(ROBOT_LAT, ROBOT_LON, 0.0, target_at(0.0, 20.0)).left
        near = self.controller.compute(ROBOT_LAT, ROBOT_LON, 0.0, target_at(0.0, 3.0)).left

        assert near < far

    def test_commands_stay_inside_normalised_range(self):
        bearings = (0, 15, 45, 55, 70, 90, 135, 170, -170, -90, -45, -15)
        distances = (1.5, 2.6, 3, 5, 8, 12, 30, 100)

        for bearing in bearings:
            for distance in distances:
                result = self.controller.compute(
                    ROBOT_LAT, ROBOT_LON, 0.0, target_at(bearing, distance)
                )
                assert -1.0 <= result.left <= 1.0
                assert -1.0 <= result.right <= 1.0

    def test_reports_distance_and_bearing_to_target(self):
        result = self.controller.compute(ROBOT_LAT, ROBOT_LON, 0.0, target_at(45.0, 50.0))

        assert result.distance_meters == pytest.approx(50.0, abs=1.0)
        assert result.bearing_degrees == pytest.approx(45.0, abs=0.5)
        assert result.turn_error_degrees == pytest.approx(45.0, abs=0.5)


# ---------------------------------------------------------------------------
# mission
# ---------------------------------------------------------------------------


class TestMission:
    def test_waypoints_round_trip_preserves_home_and_order(self):
        original = Mission()
        original.set_home(47.3977420, 8.5455940)
        original.add_waypoint(Waypoint(47.3981000, 8.5460000))
        original.add_waypoint(Waypoint(47.3985000, 8.5470000, "", 5.0))
        original.add_waypoint(Waypoint(47.3990000, 8.5480000))

        parsed = Mission.parse_waypoints_file(original.to_waypoints_file())

        assert parsed.has_home
        assert parsed.home_latitude == pytest.approx(original.home_latitude, abs=1e-7)
        assert parsed.home_longitude == pytest.approx(original.home_longitude, abs=1e-7)
        assert len(parsed) == len(original)

        for expected, actual in zip(original, parsed):
            assert actual.latitude == pytest.approx(expected.latitude, abs=1e-7)
            assert actual.longitude == pytest.approx(expected.longitude, abs=1e-7)

    def test_file_header_matches_mission_planner(self):
        mission = Mission()
        mission.set_home(47.0, 8.0)
        mission.add_waypoint(Waypoint(47.1, 8.1))

        text = mission.to_waypoints_file()
        assert text.startswith("QGC WPL 110\n")
        lines = text.split("\n")
        assert len(lines) == 4  # header, home, wp, trailing empty
        assert lines[1].startswith("0\t1\t")
        assert lines[2].startswith("1\t0\t")

    def test_non_navigation_commands_are_skipped_on_read(self):
        text = (
            "QGC WPL 110\n"
            "0\t1\t3\t16\t0\t0\t0\t0\t47.3977420\t8.5455940\t0\t1\n"
            # DO_JUMP (177) is not a waypoint and must not become one.
            "1\t0\t3\t177\t0\t0\t0\t0\t47.3980000\t8.5460000\t0\t1\n"
            "2\t0\t3\t16\t0\t0\t0\t0\t47.3985000\t8.5470000\t0\t1\n"
        )

        mission = Mission.parse_waypoints_file(text)

        assert len(mission) == 1
        assert mission.get_waypoint(0).latitude == pytest.approx(47.3985000, abs=1e-7)

    def test_rejects_foreign_header(self):
        with pytest.raises(ValueError, match="header"):
            Mission.parse_waypoints_file("not a waypoint file\n")

    def test_rejects_malformed_row(self):
        with pytest.raises(ValueError, match="Malformed"):
            Mission.parse_waypoints_file("QGC WPL 110\n0\t1\t3\n")

    def test_edits_maintain_order(self):
        mission = Mission()
        mission.add_waypoint(Waypoint(1.0, 1.0))
        mission.add_waypoint(Waypoint(2.0, 2.0))
        mission.add_waypoint(Waypoint(3.0, 3.0))

        mission.move_waypoint(2, 0)
        assert [w.latitude for w in mission] == [3.0, 1.0, 2.0]

        mission.remove_waypoint(1)
        assert [w.latitude for w in mission] == [3.0, 2.0]

        mission.clear_waypoints()
        assert mission.is_empty

    def test_total_distance_runs_home_through_waypoints(self):
        mission = Mission()
        # 1 degree of latitude is about 111.19 km.
        mission.set_home(0.0, 0.0)
        mission.add_waypoint(Waypoint(1.0, 0.0))
        mission.add_waypoint(Waypoint(2.0, 0.0))

        assert mission.total_distance_meters() == pytest.approx(222389.9, abs=100.0)

    def test_total_distance_without_home_starts_at_first_waypoint(self):
        mission = Mission()
        mission.add_waypoint(Waypoint(0.0, 0.0))
        mission.add_waypoint(Waypoint(1.0, 0.0))

        assert mission.total_distance_meters() == pytest.approx(111194.9, abs=50.0)
