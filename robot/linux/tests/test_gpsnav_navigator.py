"""Tests for the GPS mission navigator: NMEA parsing, heading fusion, state machine.

Pure-python, no hardware required. The NMEA sentences are built with correct
checksums so the parser is exercised exactly as a receiver would drive it.
"""

import os
import sys
import threading

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from gpsnav import (  # noqa: E402
    Mission,
    NavMode,
    Navigator,
    Waypoint,
    destination_point,
)
from gpsnav.gps import (  # noqa: E402
    COURSE_SPEED_THRESHOLD_MPS,
    Gps,
    HeadingFuser,
    parse_nmea_line,
)

# ---------------------------------------------------------------------------
# NMEA helpers
# ---------------------------------------------------------------------------


def nmea(body):
    """Wraps a sentence body with a valid ``$...*CC`` checksum."""
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return "$%s*%02X" % (body, checksum)


# 4807.038 N -> 48 + 7.038/60; 01131.000 E -> 11 + 31/60
LAT = 48.0 + 7.038 / 60.0
LON = 11.0 + 31.0 / 60.0

GGA = "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"
RMC = "GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W"


class TestNmea:
    def test_gga_reports_position_and_accuracy(self):
        update = parse_nmea_line(nmea(GGA))

        assert update["valid"] is True
        assert update["latitude"] == pytest.approx(LAT, abs=1e-6)
        assert update["longitude"] == pytest.approx(LON, abs=1e-6)
        assert update["accuracy_meters"] == pytest.approx(0.9 * 4.0, abs=1e-6)

    def test_rmc_reports_speed_and_course(self):
        update = parse_nmea_line(nmea(RMC))

        assert update["valid"] is True
        assert update["latitude"] == pytest.approx(LAT, abs=1e-6)
        # 022.4 kn * 0.514444
        assert update["speed_mps"] == pytest.approx(22.4 * 0.514444, abs=1e-3)
        assert update["course_degrees"] == pytest.approx(84.4, abs=1e-6)

    def test_southern_western_hemispheres_are_negative(self):
        body = "GPGGA,123519,4807.038,S,01131.000,W,1,08,0.9,545.4,M,46.9,M,,"
        update = parse_nmea_line(nmea(body))

        assert update["latitude"] == pytest.approx(-LAT, abs=1e-6)
        assert update["longitude"] == pytest.approx(-LON, abs=1e-6)

    def test_accepts_any_talker_prefix(self):
        body = "GNGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"
        assert parse_nmea_line(nmea(body))["valid"] is True

    def test_rejects_a_bad_checksum(self):
        corrupted = nmea(GGA).replace("*", "*0")
        assert parse_nmea_line(corrupted) is None

    def test_ignores_unsupported_sentences(self):
        assert parse_nmea_line(nmea("GPGSA,A,3,04,05,,,,,,,,,,,1.8,1.0,1.5")) is None
        assert parse_nmea_line("not nmea at all") is None
        assert parse_nmea_line("") is None

    def test_no_fix_is_reported_so_stale_position_can_be_dropped(self):
        no_fix = "GPGGA,123519,4807.038,N,01131.000,E,0,08,0.9,545.4,M,46.9,M,,"
        assert parse_nmea_line(nmea(no_fix)) == {"valid": False}

        void_rmc = "GPRMC,123519,V,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W"
        assert parse_nmea_line(nmea(void_rmc)) == {"valid": False}

    def test_course_wraps_into_range(self):
        # 359.9 deg is almost due north; wrap_degrees maps it just above -180.
        body = "GPRMC,123519,A,4807.038,N,01131.000,E,022.4,359.9,230394,003.1,W"
        course = parse_nmea_line(nmea(body))["course_degrees"]
        assert course == pytest.approx(-0.1, abs=1e-6)
        assert -180.0 <= course < 180.0

        body = "GPRMC,123519,A,4807.038,N,01131.000,E,022.4,270.0,230394,003.1,W"
        assert parse_nmea_line(nmea(body))["course_degrees"] == pytest.approx(-90.0)


class TestHeadingFuser:
    def setup_method(self, _):
        self.fuser = HeadingFuser()

    def test_no_heading_until_the_robot_moves(self):
        assert self.fuser.heading_degrees is None
        assert not self.fuser.is_ready

        # Stationary course is noise, not a heading.
        self.fuser.update(90.0, speed_mps=0.1)
        assert self.fuser.heading_degrees is None

    def test_course_is_used_once_moving(self):
        self.fuser.update(90.0, speed_mps=COURSE_SPEED_THRESHOLD_MPS)
        assert self.fuser.heading_degrees == pytest.approx(90.0)

    def test_holds_last_heading_when_it_slows_down(self):
        self.fuser.update(45.0, speed_mps=2.0)
        self.fuser.update(120.0, speed_mps=0.1)

        assert self.fuser.heading_degrees == pytest.approx(45.0)

    def test_none_course_leaves_heading_alone(self):
        self.fuser.update(45.0, speed_mps=2.0)
        assert self.fuser.update(None, speed_mps=2.0) == pytest.approx(45.0)

    def test_reset_forgets_the_heading(self):
        self.fuser.update(45.0, speed_mps=2.0)
        self.fuser.reset()
        assert self.fuser.heading_degrees is None


def make_bare_gps(callback):
    """Builds a Gps without opening serial, for driving the parser in tests."""
    gps = Gps.__new__(Gps)
    gps.callback = callback
    gps.latest = None
    gps._pending = {}
    gps._lock = threading.Lock()
    return gps


class TestGpsFeed:
    def test_feed_merges_gga_and_rmc_into_one_fix(self):
        received = []
        gps = make_bare_gps(received.append)

        gps.feed(nmea(GGA))
        gps.feed(nmea(RMC))

        # Each sentence yields a fix; the RMC one is the merged, complete one.
        assert len(received) == 2
        fix = received[-1]
        assert fix.latitude == pytest.approx(LAT, abs=1e-6)
        assert fix.course_degrees == pytest.approx(84.4)
        assert fix.accuracy_meters == pytest.approx(0.9 * 4.0)  # kept from the GGA
        assert fix.has_course  # 022.4 kn is well above the speed threshold

    def test_no_fix_clears_stale_position(self):
        gps = make_bare_gps(lambda fix: None)

        gps.feed(nmea(GGA))
        assert gps.latest is not None

        gps.feed(nmea("GPGGA,123519,4807.038,N,01131.000,E,0,08,0.9,545.4,M,46.9,M,,"))
        assert gps.latest is None


# ---------------------------------------------------------------------------
# Navigator
# ---------------------------------------------------------------------------

HOME = (0.0, 0.0)


def wp_at(bearing, distance, **kwargs):
    lat, lon = destination_point(HOME[0], HOME[1], bearing, distance)
    return Waypoint(lat, lon, **kwargs)


def make_navigator(*waypoints, home=HOME):
    mission = Mission()
    if home is not None:
        mission.set_home(*home)
    for waypoint in waypoints:
        mission.add_waypoint(waypoint)
    return Navigator(mission=mission)


class TestNavigatorModes:
    def test_idle_reports_zero_command(self):
        nav = Navigator()
        status = nav.step(0.0, 0.0)

        assert status.mode is NavMode.IDLE
        assert status.left == 0.0
        assert status.right == 0.0
        assert not nav.is_navigating

    def test_start_mission_refuses_an_empty_plan(self):
        assert Navigator().start_mission() is False

    def test_mission_needs_a_heading_before_it_will_steer(self):
        nav = make_navigator(wp_at(0.0, 20.0))
        assert nav.start_mission()

        status = nav.step(*HOME)
        assert status.mode is NavMode.MISSION
        assert status.message == "acquiring heading"
        assert status.left == status.right > 0.0  # creep to establish course

    def test_mission_drives_toward_the_first_waypoint(self):
        nav = make_navigator(wp_at(0.0, 20.0))
        nav.start_mission()
        nav.update_heading(0.0, 1.0)

        status = nav.step(*HOME)

        assert status.mode is NavMode.MISSION
        assert status.left > 0.0
        assert status.left == pytest.approx(status.right, abs=1e-3)
        assert status.distance_meters == pytest.approx(20.0, abs=1.0)

    def test_mission_advances_to_the_next_waypoint_on_arrival(self):
        nav = make_navigator(wp_at(0.0, 1.0), wp_at(90.0, 20.0))
        nav.start_mission()
        nav.update_heading(0.0, 1.0)

        status = nav.step(*HOME)

        assert status.arrived
        assert status.active_waypoint_index == 0
        assert nav.active_index == 1
        assert nav.is_navigating

    def test_mission_completes_after_the_last_waypoint(self):
        nav = make_navigator(wp_at(0.0, 1.0))
        nav.start_mission()
        nav.update_heading(0.0, 1.0)

        status = nav.step(*HOME)

        assert status.arrived
        assert status.active_waypoint_index == 0
        assert not nav.is_navigating
        assert nav.mode is NavMode.IDLE

    def test_guided_drives_to_one_point_then_disarms(self):
        nav = make_navigator(home=None)
        nav.mark_home(*HOME)
        nav.start_guided(*destination_point(0.0, 0.0, 0.0, 1.0))
        nav.update_heading(0.0, 1.0)

        status = nav.step(*HOME)

        assert status.mode is NavMode.IDLE  # arrival already disarmed it
        assert status.arrived
        assert not nav.is_navigating

    def test_rtl_needs_a_home_position(self):
        nav = Navigator()
        assert nav.start_rtl() is False

    def test_rtl_drives_back_to_home(self):
        far = wp_at(0.0, 30.0)
        nav = make_navigator(far)
        nav.start_rtl()
        nav.update_heading(180.0, 1.0)

        # Standing at the far waypoint, facing away: turn hard back toward home.
        status = nav.step(far.latitude, far.longitude)

        assert status.mode is NavMode.RTL
        assert status.distance_meters == pytest.approx(30.0, abs=1.0)
        assert not status.arrived

    def test_rtl_arrives_and_disarms(self):
        nav = make_navigator(wp_at(0.0, 30.0))
        nav.start_rtl()
        nav.update_heading(0.0, 1.0)

        status = nav.step(*HOME)

        assert status.arrived
        assert nav.mode is NavMode.IDLE

    def test_stop_disarms_navigation(self):
        nav = make_navigator(wp_at(0.0, 20.0))
        nav.start_mission()
        nav.stop()

        assert not nav.is_navigating
        assert nav.step(*HOME).left == 0.0

    def test_pause_holds_the_robot(self):
        nav = make_navigator(wp_at(0.0, 20.0))
        nav.start_mission()
        nav.update_heading(0.0, 1.0)
        nav.pause()

        status = nav.step(*HOME)

        assert status.mode is NavMode.MISSION
        assert status.message == "paused"
        assert status.left == 0.0

        nav.resume()
        assert nav.step(*HOME).is_moving


class TestNavigatorSafety:
    def test_holds_when_the_fix_is_untrustworthy(self):
        nav = make_navigator(wp_at(0.0, 20.0))
        nav.start_mission()
        nav.update_heading(0.0, 1.0)

        status = nav.step(0.0, 0.0, accuracy_meters=25.0)

        assert status.left == 0.0
        assert status.right == 0.0
        assert "holding" in status.message

    def test_drives_on_a_fix_inside_the_accuracy_budget(self):
        nav = make_navigator(wp_at(0.0, 20.0))
        nav.start_mission()
        nav.update_heading(0.0, 1.0)

        status = nav.step(0.0, 0.0, accuracy_meters=4.0)

        assert status.is_moving

    def test_records_a_track_of_every_fix(self):
        nav = make_navigator(wp_at(0.0, 20.0))
        nav.start_mission()
        nav.update_heading(0.0, 1.0)

        nav.step(0.0, 0.0)
        nav.step(0.0, 0.001)

        assert nav.track == [(0.0, 0.0), (0.0, 0.001)]

    def test_kml_export_contains_the_plan_and_the_track(self):
        nav = make_navigator(wp_at(0.0, 20.0), wp_at(90.0, 20.0))
        nav.start_mission()
        nav.update_heading(0.0, 1.0)
        nav.step(*HOME)

        kml = nav.export_kml("Test Mission")

        assert "<name>Test Mission</name>" in kml
        assert "<name>Home</name>" in kml
        assert "<name>WP1</name>" in kml
        assert "<name>WP2</name>" in kml
        assert "<name>Track</name>" in kml


class TestMissionFiles:
    def test_save_and_load_round_trip_through_the_navigator(self):
        nav = make_navigator(wp_at(0.0, 20.0), wp_at(90.0, 20.0))
        text = nav.save_mission_file()

        reloaded = Navigator()
        reloaded.load_mission_file(text)

        assert len(reloaded.mission) == 2
        assert reloaded.mission.has_home

    def test_rejects_a_foreign_file(self):
        with pytest.raises(ValueError, match="header"):
            Navigator().load_mission_file("garbage\n")
