"""GPS mission navigation state machine.

The ground-robot equivalent of Mission Planner's flight modes:

- ``MISSION`` - AUTO: drive the loaded waypoint list in order
- ``GUIDED``  - "go to here": drive to one clicked target
- ``RTL``     - return to launch: drive back to home

``step()`` is a pure function of the latest fix and the mission state, so the
whole thing can be unit tested and replayed from a logged track without any
hardware attached.
"""

from enum import Enum

from gpsnav.geo import bearing_degrees, wrap_degrees
from gpsnav.gps import COURSE_SPEED_THRESHOLD_MPS
from gpsnav.guidance import GuidanceController
from gpsnav.kml import build_kml
from gpsnav.mission import Mission, Waypoint

#: Hold position rather than drive on a fix worse than this (matches Android).
MAX_POSITION_ACCURACY_M = 15.0
#: Forward creep used until GPS course-over-ground gives us a heading.
HEADING_WARMUP_THROTTLE = 0.25


class NavMode(Enum):
    IDLE = "idle"
    MISSION = "mission"
    GUIDED = "guided"
    RTL = "rtl"


class NavStatus:
    """Outcome of one navigation tick."""

    def __init__(
        self,
        mode,
        left,
        right,
        distance_meters=None,
        bearing_degrees=None,
        turn_error_degrees=None,
        active_waypoint_index=None,
        arrived=False,
        message="",
    ):
        self.mode = mode
        self.left = left
        self.right = right
        self.distance_meters = distance_meters
        self.bearing_degrees = bearing_degrees
        self.turn_error_degrees = turn_error_degrees
        self.active_waypoint_index = active_waypoint_index
        self.arrived = arrived
        self.message = message

    @property
    def is_moving(self):
        return self.left != 0.0 or self.right != 0.0

    def __repr__(self):
        return "NavStatus(%s L=%.2f R=%.2f dist=%s err=%s %s)" % (
            self.mode.value,
            self.left,
            self.right,
            self.distance_meters,
            self.turn_error_degrees,
            self.message,
        )


class Navigator:
    """Runs waypoint missions against a live GPS fix."""

    def __init__(self, mission=None, guidance=None):
        """
        Args:
            mission: a :class:`~gpsnav.mission.Mission`, created empty if omitted
            guidance: a :class:`~gpsnav.guidance.GuidanceController`
        """
        self.mission = mission if mission is not None else Mission()
        self.guidance = guidance if guidance is not None else GuidanceController()
        self.mode = NavMode.IDLE
        self.paused = False
        self.active_index = 0
        self.guided_target = None
        self.track = []
        self._heading_degrees = None
        self.last_status = None

    # region mission bookkeeping

    @property
    def is_navigating(self):
        return self.mode is not NavMode.IDLE

    @property
    def heading_degrees(self):
        """Heading used for steering, or ``None`` before the first moving fix."""
        return self._heading_degrees

    def mark_home(self, latitude, longitude):
        """Records the launch position RTL returns to."""
        self.mission.set_home(latitude, longitude)

    def load_mission(self, mission):
        self.mission = mission if mission is not None else Mission()
        self.active_index = 0

    def load_mission_file(self, contents):
        """Loads a Mission Planner ``.waypoints`` document."""
        self.load_mission(Mission.parse_waypoints_file(contents))
        return self.mission

    def save_mission_file(self):
        """Serialises the current mission to the ``.waypoints`` format."""
        return self.mission.to_waypoints_file()

    def current_target(self):
        """The waypoint being sought right now, or ``None`` when idle."""
        if self.mode is NavMode.MISSION:
            return self.mission.get_waypoint(self.active_index)
        return self.guided_target

    # endregion

    # region mode control

    def start_mission(self):
        """AUTO: drive every waypoint in order. Returns False if there are none."""
        if self.mission.is_empty:
            return False
        self.active_index = 0
        self.guided_target = None
        self.mode = NavMode.MISSION
        self.paused = False
        return True

    def start_guided(self, latitude, longitude, acceptance_radius_meters=2.5):
        """GUIDED: drive to a single target ("go to here")."""
        self.guided_target = Waypoint(
            latitude, longitude, "", acceptance_radius_meters
        )
        self.mode = NavMode.GUIDED
        self.paused = False
        return True

    def start_rtl(self, acceptance_radius_meters=2.5):
        """RTL: return to the recorded home position."""
        if not self.mission.has_home:
            return False
        self.guided_target = Waypoint(
            self.mission.home_latitude,
            self.mission.home_longitude,
            "Home",
            acceptance_radius_meters,
        )
        self.mode = NavMode.RTL
        self.paused = False
        return True

    def pause(self):
        if self.is_navigating:
            self.paused = True

    def resume(self):
        self.paused = False

    def stop(self):
        """Disarms navigation and returns the robot to manual/idle control."""
        self.mode = NavMode.IDLE
        self.paused = False
        self.guided_target = None
        self.active_index = 0

    # endregion

    # region guidance

    def update_heading(self, course_degrees, speed_mps=0.0):
        """Fuses GPS course-over-ground into the steering heading.

        Mirrors the Android provider: the course is only trusted while moving.
        Returns the heading, which is ``None`` until the robot has moved enough
        for the receiver to report a real course.
        """
        if course_degrees is None:
            return self._heading_degrees
        if speed_mps >= COURSE_SPEED_THRESHOLD_MPS:
            self._heading_degrees = wrap_degrees(course_degrees)
        return self._heading_degrees

    def step(self, latitude, longitude, accuracy_meters=None):
        """Computes one control tick from the current fix.

        Args:
            latitude: current latitude in degrees
            longitude: current longitude in degrees
            accuracy_meters: horizontal accuracy of the fix, if known

        Returns:
            NavStatus: wheel commands plus distance/bearing/progress state
        """
        if latitude is not None and longitude is not None:
            self.track.append((latitude, longitude))

        if not self.is_navigating or self.paused:
            message = "paused" if self.paused else "idle"
            return self._record(NavStatus(self.mode, 0.0, 0.0, message=message))

        if accuracy_meters is not None and accuracy_meters > MAX_POSITION_ACCURACY_M:
            # Driving on a bad fix is worse than waiting: hold and say so.
            return self._record(
                NavStatus(
                    self.mode,
                    0.0,
                    0.0,
                    active_waypoint_index=self.active_index,
                    message="GPS accuracy %.1f m, holding" % accuracy_meters,
                )
            )

        target = self.current_target()
        if target is None:
            self.stop()
            return self._record(NavStatus(NavMode.IDLE, 0.0, 0.0, message="no target"))

        if self._heading_degrees is None:
            # No compass on Linux, so heading only exists once we have moved.
            # Creep forward to establish course-over-ground, then steer.
            return self._record(
                NavStatus(
                    self.mode,
                    HEADING_WARMUP_THROTTLE,
                    HEADING_WARMUP_THROTTLE,
                    distance_meters=target.distance_from(latitude, longitude),
                    bearing_degrees=bearing_degrees(
                        latitude, longitude, target.latitude, target.longitude
                    ),
                    active_waypoint_index=self.active_index,
                    message="acquiring heading",
                )
            )

        result = self.guidance.compute(
            latitude, longitude, self._heading_degrees, target
        )

        # Snapshot before advancing, so the status names the waypoint just hit.
        reached_index = self.active_index

        if result.arrived:
            self._on_arrived()
            return self._record(
                NavStatus(
                    self.mode,
                    0.0,
                    0.0,
                    distance_meters=result.distance_meters,
                    bearing_degrees=result.bearing_degrees,
                    turn_error_degrees=result.turn_error_degrees,
                    active_waypoint_index=reached_index,
                    arrived=True,
                    message="waypoint reached",
                )
            )

        return self._record(
            NavStatus(
                self.mode,
                result.left,
                result.right,
                distance_meters=result.distance_meters,
                bearing_degrees=result.bearing_degrees,
                turn_error_degrees=result.turn_error_degrees,
                active_waypoint_index=reached_index,
            )
        )

    def _on_arrived(self):
        if self.mode is NavMode.MISSION:
            self.active_index += 1
            if self.active_index >= len(self.mission):
                # Last waypoint: mission complete, back to idle.
                self.stop()
        else:
            # GUIDED and RTL are one-shot: arriving disarms them.
            self.stop()

    def _record(self, status):
        self.last_status = status
        return status

    # endregion

    # region export

    def export_kml(self, document_name="OpenBot Mission"):
        """Builds a KML document of the plan plus the track driven so far."""
        return build_kml(document_name, self.mission, self.track)

    def save_kml(self, path, document_name="OpenBot Mission"):
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(self.export_kml(document_name))

    # endregion
