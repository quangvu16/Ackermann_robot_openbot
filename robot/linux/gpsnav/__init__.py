"""GPS mission navigation for the OpenBot Linux robot.

A Python port of the Android ``gpsNav`` package: waypoint missions driven by a
GPS receiver, with the Mission Planner feature set (mission plan, guided
"go here", return-to-launch, ``.waypoints`` interchange, KML export).

The submodules are deliberately free of hardware and UI dependencies so they can
be unit tested and replayed from a logged track:

- :mod:`gpsnav.geo` - geographic math
- :mod:`gpsnav.mission` - waypoints, mission, Mission Planner file codec
- :mod:`gpsnav.guidance` - differential-drive "go to waypoint" law
- :mod:`gpsnav.navigator` - MISSION / GUIDED / RTL state machine
- :mod:`gpsnav.gps` - NMEA receiver and heading fusion
- :mod:`gpsnav.kml` - KML export

Entry point: ``run_mission.py`` in the parent directory.
"""

from gpsnav.geo import (
    bearing_degrees,
    destination_point,
    distance_meters,
    turn_error_degrees,
    wrap_degrees,
)
from gpsnav.gps import (
    COURSE_SPEED_THRESHOLD_MPS,
    Gps,
    GpsFix,
    HeadingFuser,
    get_gps_serial,
    parse_nmea_line,
)
from gpsnav.guidance import GuidanceController, GuidanceResult
from gpsnav.kml import build_kml
from gpsnav.mission import Mission, Waypoint
from gpsnav.navigator import (
    MAX_POSITION_ACCURACY_M,
    NavMode,
    Navigator,
    NavStatus,
)

__all__ = [
    "COURSE_SPEED_THRESHOLD_MPS",
    "GuidanceController",
    "GuidanceResult",
    "Gps",
    "GpsFix",
    "HeadingFuser",
    "MAX_POSITION_ACCURACY_M",
    "Mission",
    "NavMode",
    "NavStatus",
    "Navigator",
    "Waypoint",
    "bearing_degrees",
    "build_kml",
    "destination_point",
    "distance_meters",
    "get_gps_serial",
    "parse_nmea_line",
    "turn_error_degrees",
    "wrap_degrees",
]
