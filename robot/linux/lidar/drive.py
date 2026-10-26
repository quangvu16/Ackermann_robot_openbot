"""The single chokepoint every drive command passes through.

Python counterpart of the object-avoidance gate inside ``Vehicle.sendControl()``
on Android. ArduPilot applies Simple Object Avoidance inside the navigation
stack so no flight mode escapes it; on this robot the equivalent is the one
function every caller already uses to reach the motors, and that is
``motors.apply_action_serial``. Wrapping that is enough - gamepad, mission,
inference and anything else all pass through it.

``control`` is what the driver or autopilot asked for; ``last_decision`` is what
actually went to the wheels after the law had its say. Keeping both is what lets
a cleared blockage release without the driver re-moving the stick: every new
proximity report re-evaluates the stored request.
"""

from lidar.avoidance import Decision, SECTORS, evaluate
from lidar.proximity import Proximity


def feed_from_scan(proximity, scan, forward_offset_deg=0.0):
    """Fold one revolution into octants and ingest them as a ``p`` report.

    The UDP scan is the richer source on this host - a full 360 degrees rather
    than the eight numbers the firmware also sends on the serial ``p`` channel -
    so a mission runner with no telemetry reader can still feed the law.
    Millimetres are converted to the centimetres ``set_sectors_cm`` expects, and
    a clear sector stays 0 so it lands as "no return" exactly as the wire
    convention intends.
    """
    if proximity is None or scan is None:
        return False
    sectors_mm = scan.compute_sectors_mm(SECTORS, forward_offset_deg)
    return proximity.set_sectors_cm([mm / 10.0 for mm in sectors_mm])


class DriveGate:
    """Applies the object-avoidance law to every control request."""

    def __init__(
        self,
        proximity=None,
        margin_meters=1.0,
        backup_speed=0.0,
        enabled=True,
    ):
        self.proximity = proximity
        self.margin_meters = float(margin_meters)
        self.backup_speed = float(backup_speed)
        #: The quick-disable switch, ArduPilot's ``RCx_OPTION = 40``.
        self.enabled = bool(enabled)
        #: The last request the driver or autopilot made, before avoidance.
        self.control = (0.0, 0.0)
        self._last_decision = Decision(0.0, 0.0, False, False, -1, float("inf"))

    @property
    def last_decision(self):
        """What actually went out on the last call to :meth:`apply`."""
        return self._last_decision

    def set_enabled(self, enabled):
        """Toggle the quick-disable switch and re-evaluate the stored request.

        Returns the new decision, so a caller can re-send immediately without
        waiting for the next stick move.
        """
        self.enabled = bool(enabled)
        return self.apply(*self.control)

    def set_margin_meters(self, margin_meters):
        self.margin_meters = float(margin_meters)
        return self.apply(*self.control)

    def set_backup_speed(self, backup_speed):
        self.backup_speed = float(backup_speed)
        return self.apply(*self.control)

    def set_proximity(self, proximity):
        self.proximity = proximity

    def on_proximity_update(self):
        """Re-evaluate after a new ``p`` report.

        Called whenever proximity data changes so a cleared blockage releases
        the robot without the driver re-moving the stick. Returns the new
        decision.
        """
        return self.apply(*self.control)

    def update_from_scan(self, scan, forward_offset_deg=0.0):
        """Ingest one revolution and re-evaluate the stored request.

        Returns the new decision, or ``None`` when there is no scan or no
        proximity sink to feed.
        """
        if self.proximity is None or scan is None:
            return None
        if not feed_from_scan(self.proximity, scan, forward_offset_deg):
            return None
        return self.on_proximity_update()

    def apply(self, left, right):
        """Run one control request through the law.

        Returns the :class:`~lidar.avoidance.Decision` to actually drive.
        """
        self.control = (left, right)
        proximity = self.proximity
        if proximity is None:
            state = "never"
            sectors = [float("inf")] * 8
        else:
            state = proximity.get_data_state()
            sectors = proximity.get_sectors_m()
        self._last_decision = evaluate(
            left,
            right,
            sectors,
            state,
            self.margin_meters,
            self.backup_speed,
            self.enabled,
        )
        return self._last_decision

    def status_text(self):
        """One-line status for a HUD, matching the Android app's wording."""
        if not self.enabled:
            return "Off"
        if self.proximity is None or not self.proximity.has_ever_reported():
            return "Idle"
        state = self.proximity.get_data_state()
        if state == "stale":
            return "Blind"
        decision = self._last_decision
        if decision.backing:
            return "Backing"
        if decision.blocked:
            return "Holding"
        return "Clear"
