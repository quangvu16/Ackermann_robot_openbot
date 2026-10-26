"""The eight proximity octants the car reports on the OpenBot ``p`` channel.

Python port of ``org.openbot.app.robot.vehicle.Proximity`` on Android.

The layout is the MAVLink ``DISTANCE_SENSOR`` octant arrangement that
ArduPilot's proximity library uses, and the same one the firmware's
``PROX_SECTORS`` folding produces: sector 0 is straight ahead and the numbering
goes clockwise, one sector every 45 degrees::

           7   0   1        0 = front        4 = back
         6   +   2          1 = front-right  5 = back-left
           5   4   3        2 = right        6 = left
                             3 = back-right   7 = front-left

**Units and the zero convention.** The wire carries centimetres to match the
``s`` sonar channel, and the firmware spells "nothing measured in this sector" as
``0``. This module converts on ingest: metres internally, and a 0 becomes
``math.inf`` so an empty sector can never be mistaken for an obstacle sitting on
top of the robot. That is the *opposite* of the ``s`` channel, where 0 means
stale and the car stops - on ``p``, staleness is signalled by the report simply
not arriving, which is what :meth:`Proximity.get_age` and
:meth:`Proximity.get_data_state` watch for.

Staleness is measured as time since the *last accepted report*, not as the gap
between the last two reports: that gap freezes at the reporting period when
reports stop, which would make a dead sensor look permanently fresh.
"""

import math
import time

SECTORS = 8

#: How old the last report may be before it is considered stale.
STALE_AFTER_MS = 500

#: Age reported before any reading has arrived, so a never-fed instance reads as
#: stale rather than fresh.
NEVER_FED_AGE_MS = 1000


def parse_sectors_cm(body):
    """Parse the body of a ``p`` message: eight comma-separated distances in cm.

    Returns ``None`` for anything malformed, so a truncated BLE notification
    cannot be mistaken for a scan of eight clear sectors.
    """
    if body is None:
        return None
    parts = body.split(",")
    if len(parts) != SECTORS:
        return None
    out = []
    for part in parts:
        try:
            out.append(float(part.strip()))
        except ValueError:
            return None
    return out


class Proximity:
    """Eight octant distances with the staleness tracking object avoidance needs."""

    def __init__(self, clock=None):
        self._sectors_m = [math.inf] * SECTORS
        self._timestamp_ms = 0
        # Injectable clock so the tests can age the reading without sleeping.
        self._clock = clock if clock is not None else _monotonic_ms

    def set_sectors_cm(self, sector_cm):
        """Ingest one ``p`` report.

        ``sector_cm`` is the eight octant distances in centimetres as received;
        ``0`` means "no return" and is stored as infinite. A ``None``, short or
        otherwise unusable list is rejected and the previous reading is left
        alone rather than being replaced with zeros that look like clear road.

        Returns True if the report was accepted.
        """
        if sector_cm is None or len(sector_cm) != SECTORS:
            return False
        for cm in sector_cm:
            if cm is None or cm != cm or cm < 0:  # NaN check: NaN != NaN
                return False
        for i, cm in enumerate(sector_cm):
            # 0 cm on the wire means "nothing measured here", not "an obstacle at
            # zero". A real return at 0 is outside the LD14's 20 mm minimum
            # anyway, so the two cannot be confused.
            self._sectors_m[i] = math.inf if cm == 0 else cm / 100.0
        self._timestamp_ms = self._clock()
        return True

    def get_sectors_m(self):
        """The eight octant distances in metres, clear sectors as ``inf``."""
        return list(self._sectors_m)

    def get_sector_m(self, sector):
        """Distance of one sector in metres, or ``inf`` when there is no return."""
        if sector < 0 or sector >= SECTORS:
            return math.inf
        return self._sectors_m[sector]

    def get_age_ms(self):
        """Milliseconds since the last accepted report, or ``NEVER_FED_AGE_MS`` before it."""
        if self._timestamp_ms == 0:
            return NEVER_FED_AGE_MS
        return self._clock() - self._timestamp_ms

    @property
    def timestamp_ms(self):
        """Monotonic milliseconds of the last accepted report, or 0 before the first."""
        return self._timestamp_ms

    def has_ever_reported(self):
        """True once at least one report has been ingested."""
        return self._timestamp_ms > 0

    def get_data_state(self):
        """How fresh the picture is.

        ``"never"`` never blocks driving - a robot with no LiDAR must stay
        drivable. ``"stale"`` blocks forward motion - a LiDAR that died mid-drive
        must not leave the robot driving blind. ``"fresh"`` can be acted on.
        """
        if self._timestamp_ms == 0:
            return "never"
        return "stale" if self.get_age_ms() > STALE_AFTER_MS else "fresh"

    def reset(self):
        """Reset to "nothing has ever been measured"."""
        self._sectors_m = [math.inf] * SECTORS
        self._timestamp_ms = 0


def _monotonic_ms():
    return time.monotonic() * 1000.0
