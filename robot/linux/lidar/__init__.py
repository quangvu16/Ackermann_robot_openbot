"""LiDAR scanning, proximity and ArduPilot-style object avoidance.

A Python port of the Android ``vehicle`` + ``customview`` lidar stack, for the
OpenBot Linux robot. The firmware already folds one LD14/LD14P revolution into
eight octants on the ``p`` channel and streams full "LDS1" scans over UDP; this
package consumes both so the operator can see what the LiDAR sees and the drive
path stops the car at an obstacle the way ArduPilot Rover does.

The submodules are deliberately free of hardware and UI dependencies so they can
be unit tested with a fake clock and a hand-built datagram:

- :mod:`lidar.scan` - the "LDS1" datagram codec and bin geometry
- :mod:`lidar.proximity` - the eight octants of the ``p`` channel
- :mod:`lidar.avoidance` - the Rover object-avoidance law
- :mod:`lidar.scanner` - UDP listener holding the newest revolution
- :mod:`lidar.drive` - the single chokepoint every drive command passes through
- :mod:`lidar.plan_view` - polar plot of one revolution

Entry points: :mod:`motors` (drive) and ``run_mission.py`` (mission runner) in
the parent directory.
"""

from lidar.avoidance import (
    BACK,
    BACK_LEFT,
    BACK_RIGHT,
    Decision,
    FRONT,
    FRONT_ARC,
    FRONT_LEFT,
    FRONT_RIGHT,
    FRESH,
    LEFT,
    NEVER,
    REAR_ARC,
    RIGHT,
    SECTORS,
    STALE,
    apply,
    evaluate,
)
from lidar.drive import DriveGate, feed_from_scan
from lidar.plan_view import format_status, scan_to_points, split_strokes
from lidar.proximity import Proximity, parse_sectors_cm
from lidar.scan import HEADER_LEN, MAGIC, MAX_BINS, LidarScan, wrap_deg
from lidar.scanner import DEFAULT_PORT, NEVER_FED_AGE_MS, STALE_AFTER_MS, LidarScanner

__all__ = [
    "BACK",
    "BACK_LEFT",
    "BACK_RIGHT",
    "DEFAULT_PORT",
    "Decision",
    "DriveGate",
    "FRONT",
    "FRONT_ARC",
    "FRONT_LEFT",
    "FRONT_RIGHT",
    "FRESH",
    "HEADER_LEN",
    "LEFT",
    "LidarScan",
    "LidarScanner",
    "MAGIC",
    "MAX_BINS",
    "NEVER",
    "NEVER_FED_AGE_MS",
    "Proximity",
    "REAR_ARC",
    "RIGHT",
    "SECTORS",
    "STALE",
    "STALE_AFTER_MS",
    "apply",
    "evaluate",
    "feed_from_scan",
    "format_status",
    "parse_sectors_cm",
    "scan_to_points",
    "split_strokes",
    "wrap_deg",
]
