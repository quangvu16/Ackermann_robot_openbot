"""Differential-drive "go to waypoint" guidance law.

Mirrors ``org.openbot.app.robot.gpsNav.GuidanceController`` on Android: turn
proportionally onto the bearing of the target, bleed speed as the acceptance
radius is approached, and stop once inside it. When the heading error is large
the robot pivots on the spot rather than carving a wide arc.

The controller is a pure function of ``(position, heading, target)`` so it can be
unit tested and replayed from a logged track without touching the vehicle.
"""

from gpsnav.geo import turn_error_degrees


class GuidanceResult:
    """Outcome of one guidance step."""

    def __init__(
        self,
        left,
        right,
        distance_meters,
        bearing_degrees,
        turn_error_degrees_,
        arrived,
    ):
        #: Normalised left wheel command in ``[-1, 1]``.
        self.left = left
        #: Normalised right wheel command in ``[-1, 1]``.
        self.right = right
        self.distance_meters = distance_meters
        self.bearing_degrees = bearing_degrees
        #: Signed turn needed to point at the target; positive is to the right.
        self.turn_error_degrees = turn_error_degrees_
        self.arrived = arrived

    def __repr__(self):
        return "GuidanceResult(l=%.3f r=%.3f dist=%.2f brg=%.1f err=%.1f arrived=%s)" % (
            self.left,
            self.right,
            self.distance_meters,
            self.bearing_degrees,
            self.turn_error_degrees,
            self.arrived,
        )


class GuidanceController:
    """Steers a differential-drive robot toward a GPS waypoint.

    This is the ground-robot equivalent of Mission Planner's "go to waypoint" /
    guided mode.
    """

    def __init__(
        self,
        acceptance_radius_meters=2.5,
        slowdown_radius_meters=8.0,
        full_lock_bearing_error_degrees=70.0,
        pivot_bearing_error_degrees=55.0,
        cruise_throttle=0.70,
        min_throttle=0.28,
    ):
        """
        Args:
            acceptance_radius_meters: distance at which a waypoint counts as reached
            slowdown_radius_meters: distance at which the approach speed ramp begins
            full_lock_bearing_error_degrees: heading error that saturates steering
            pivot_bearing_error_degrees: heading error above which we pivot, not arc
            cruise_throttle: throttle used far from the waypoint, in ``[0, 1]``
            min_throttle: throttle floor so the motors keep turning near the waypoint
        """
        self.acceptance_radius_meters = max(0.5, acceptance_radius_meters)
        self.slowdown_radius_meters = max(self.acceptance_radius_meters, slowdown_radius_meters)
        self.full_lock_bearing_error_degrees = max(1.0, full_lock_bearing_error_degrees)
        self.pivot_bearing_error_degrees = max(1.0, pivot_bearing_error_degrees)
        self.cruise_throttle = _clamp(cruise_throttle, 0.05, 1.0)
        self.min_throttle = _clamp(min_throttle, 0.0, self.cruise_throttle)

    def compute(self, latitude, longitude, heading_degrees, target):
        """Computes the wheel commands that move the robot toward ``target``.

        Args:
            latitude: current latitude in degrees
            longitude: current longitude in degrees
            heading_degrees: current heading, degrees clockwise from true north
            target: the :class:`~gpsnav.mission.Waypoint` being sought

        Returns:
            GuidanceResult: wheel commands plus distance/bearing/arrival state
        """
        distance_meters = target.distance_from(latitude, longitude)
        bearing_degrees = target.bearing_from(latitude, longitude)
        turn_error = turn_error_degrees(heading_degrees, bearing_degrees)

        radius = max(self.acceptance_radius_meters, target.acceptance_radius_meters)
        if distance_meters <= radius:
            return GuidanceResult(0.0, 0.0, distance_meters, bearing_degrees, turn_error, True)

        # Positive turn error means the target lies to the right of the nose.
        turn = _clamp(turn_error / self.full_lock_bearing_error_degrees, -1.0, 1.0)

        # Ramp up from min_throttle at the acceptance radius to cruise_throttle at
        # the slowdown radius.
        ramp = _clamp(
            (distance_meters - radius) / (self.slowdown_radius_meters - radius), 0.0, 1.0
        )
        throttle = self.min_throttle + (self.cruise_throttle - self.min_throttle) * ramp

        # Bleed a little more speed while the nose is well off the bearing so we
        # do not overshoot.
        throttle *= 1.0 - 0.35 * abs(turn)

        if abs(turn_error) > self.pivot_bearing_error_degrees:
            # Pivot on the spot: right turn means left wheel forward, right wheel back.
            pivot = max(throttle, self.min_throttle)
            sign = 1.0 if turn_error >= 0 else -1.0
            left = _clamp(sign * pivot, -1.0, 1.0)
            right = _clamp(-sign * pivot, -1.0, 1.0)
        else:
            turn_component = throttle * turn
            left = _clamp(throttle + turn_component, -1.0, 1.0)
            right = _clamp(throttle - turn_component, -1.0, 1.0)

        return GuidanceResult(left, right, distance_meters, bearing_degrees, turn_error, False)


def _clamp(value, low, high):
    return max(low, min(high, value))
