"""ArduPilot Rover-style Simple Object Avoidance.

Python port of ``org.openbot.app.robot.vehicle.ObjectAvoidance`` on Android.

The model follows ArduPilot's *Rover* behaviour rather than Copter's:

- **Rover always stops.** Copter's ``AVOID_BEHAVE`` (Stop vs Slide) has no Rover
  equivalent - there is no sliding sideways past an obstacle on a car.
- **Applies in every mode except manual.** ArduPilot applies this in the
  firmware's navigation stack so no flight mode escapes it. Here the equivalent
  chokepoint is the caller's drive path (:func:`apply` wraps whatever the source
  asked for), so gamepad, mission, inference and Blockly all pass through it.
- **``AVOID_MARGIN``** is the standoff at which the vehicle stops, in metres.
- **``AVOID_BACKUP_SPD``** actively backs away to restore the margin. ``0``
  disables it, matching ArduPilot's default of 0.
- **"SAFETY FIRST!"** ArduPilot insists on a quick-disable switch
  (``RCx_OPTION = 40``) before first use. The same switch is the ``enabled``
  argument here.

The eight sectors are the MAVLink ``DISTANCE_SENSOR`` octant layout; see
:mod:`lidar.proximity`.

Two deliberate deviations from ArduPilot, both toward stopping rather than
driving:

- A **stale** sensor (reports arrived, then stopped) blocks forward motion.
  ArduPilot simply fails to avoid when it has no data; for a robot with no other
  ranging that means driving blind, which is the one outcome the feature exists
  to prevent. Reverse stays available so the driver can back out. A sensor that
  has **never** reported is treated as "not fitted" and never blocks - otherwise
  a robot without a LiDAR would be undrivable.
- Blocking zeroes the throttle but **keeps the steering**, so the wheels stay
  pointed where the driver put them while the robot is held.
"""

import math

SECTORS = 8

FRONT = 0
FRONT_RIGHT = 1
RIGHT = 2
BACK_RIGHT = 3
BACK = 4
BACK_LEFT = 5
LEFT = 6
FRONT_LEFT = 7

#: The front arc checked against forward motion: front-left, front, front-right.
#: A 135 degree sweep - a finer-grained version of the forward quadrant ArduPilot
#: checks, and it covers the corners an Ackermann car swings through as it steers.
FRONT_ARC = (FRONT_LEFT, FRONT, FRONT_RIGHT)

#: The rear arc checked against reverse motion, the mirror of ``FRONT_ARC``.
REAR_ARC = (BACK_LEFT, BACK, BACK_RIGHT)

#: No proximity report has ever arrived: no LiDAR fitted.
NEVER = "never"
#: Reports arrived but have stopped. Fail toward stopping.
STALE = "stale"
#: A recent report arrived and can be acted on.
FRESH = "fresh"


class Decision:
    """What the law decided.

    ``left`` and ``right`` are the commands to actually drive, in ``[-1, 1]``.
    """

    def __init__(self, left, right, blocked, backing, nearest_sector, nearest_meters):
        self.left = left
        self.right = right
        #: True when an obstacle vetoed motion the driver or autopilot actually asked for.
        self.blocked = blocked
        #: True when the robot is actively reversing to restore the margin.
        self.backing = backing
        #: Sector index of the nearest obstacle in the checked arc, or -1 when clear.
        self.nearest_sector = nearest_sector
        #: Distance to that obstacle in metres, or ``inf`` when clear.
        self.nearest_meters = nearest_meters

    def is_holding(self):
        """True when this decision is holding the robot back, stopped or backing."""
        return self.blocked or self.backing


def evaluate(
    left,
    right,
    sector_meters,
    state,
    margin_meters,
    backup_speed,
    enabled,
):
    """Evaluate one control request against the current proximity picture.

    Parameters
    ----------
    left, right:
        Requested wheel commands in ``[-1, 1]``.
    sector_meters:
        The eight octant distances in metres; anything ``<= 0`` or non-finite is
        treated as "no return" (clear), matching the firmware's 0-means-clear
        wire convention.
    state:
        One of :data:`NEVER`, :data:`STALE`, :data:`FRESH`.
    margin_meters:
        Standoff in metres, like ArduPilot's ``AVOID_MARGIN``. ``<= 0`` disables
        the law.
    backup_speed:
        Reverse command in ``[0, 1]`` used to back away, like
        ``AVOID_BACKUP_SPD``. ``0`` disables backing away.
    enabled:
        The quick-disable switch. ``False`` always passes the request through
        untouched.
    """
    left = _clamp(left)
    right = _clamp(right)

    if not enabled or margin_meters <= 0 or state == NEVER:
        return Decision(left, right, False, False, -1, math.inf)

    if state == STALE:
        # The last picture we had is too old to trust. A frozen "clear road" is
        # exactly the reading that drives a robot into a wall, so block the
        # direction we can no longer see. Reverse stays open - backing out of
        # trouble is what the driver wants, and the firmware's own
        # stop_at_obstacle() still holds the nose off whatever is in front.
        if _throttle(left, right) > 0:
            return _stop_keep_steering(left, right, -1, math.inf)
        return Decision(left, right, False, False, -1, math.inf)

    throttle = _throttle(left, right)
    if throttle > 0:
        obstacle = _nearest(sector_meters, FRONT_ARC)
        if obstacle[1] < margin_meters:
            if backup_speed > 0:
                rear = _nearest(sector_meters, REAR_ARC)
                if rear[1] >= margin_meters:
                    # ArduPilot's AVOID_BACKUP_SPD: actively back away to restore
                    # the margin rather than sitting against the barrier. Only
                    # when the way behind is clear - backing into a wall to get
                    # away from a wall is not an improvement.
                    b = _clamp(backup_speed)
                    return Decision(-b, -b, True, True, obstacle[0], obstacle[1])
            return _stop_keep_steering(left, right, obstacle[0], obstacle[1])
    elif throttle < 0:
        obstacle = _nearest(sector_meters, REAR_ARC)
        if obstacle[1] < margin_meters:
            return _stop_keep_steering(left, right, obstacle[0], obstacle[1])

    return Decision(left, right, False, False, -1, math.inf)


def apply(left, right, proximity, margin_meters, backup_speed, enabled):
    """Convenience wrapper: evaluate and return the ``Decision`` to drive.

    ``proximity`` is anything exposing ``get_sectors_m()`` and
    ``get_data_state()``, i.e. a :class:`lidar.proximity.Proximity`.
    """
    return evaluate(
        left,
        right,
        proximity.get_sectors_m(),
        proximity.get_data_state(),
        margin_meters,
        backup_speed,
        enabled,
    )


def _throttle(left, right):
    """Throttle of a tank-style pair: the same mix the firmware uses to drive the ESC."""
    return left + right


def _stop_keep_steering(left, right, nearest_sector, nearest_meters):
    """Zero the throttle but keep the steering angle the driver asked for.

    With throttle 0 and ``steering = left - right`` preserved, the two tracks
    become ``+/- steering/2`` - the wheels stay pointed, the robot does not move.
    """
    steering = left - right
    half = _clamp(steering / 2.0)
    return Decision(half, -half, True, False, nearest_sector, nearest_meters)


def _nearest(sector_meters, arc):
    """Nearest return across the given sectors.

    A sector with no return is infinite here, so a clear arc never looks like an
    obstacle sitting at zero.
    """
    best_sector = -1
    best = math.inf
    if sector_meters is None or len(sector_meters) < SECTORS:
        return best_sector, best
    for sector in arc:
        if sector < 0 or sector >= len(sector_meters):
            continue
        try:
            d = float(sector_meters[sector])
        except (TypeError, ValueError):
            continue
        if not (d > 0) or math.isinf(d) or math.isnan(d):
            continue
        if d < best:
            best = d
            best_sector = sector
    return best_sector, best


def _clamp(value):
    try:
        v = float(value)
    except (TypeError, ValueError):
        return 0.0
    if math.isnan(v):
        return 0.0
    return max(-1.0, min(1.0, v))
