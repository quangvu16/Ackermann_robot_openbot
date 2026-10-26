"""Geographic helpers for GPS navigation.

Pure Python so the math can be unit tested without hardware. All angles are in
degrees and all distances in metres.

Mirrors ``org.openbot.app.robot.gpsNav.GeoMath`` on Android so a mission
planned on one platform behaves the same on the others.
"""

import math

#: Mean earth radius used by the haversine formula.
EARTH_RADIUS_M = 6371008.8

_DEG_TO_RAD = math.pi / 180.0
_RAD_TO_DEG = 180.0 / math.pi


def distance_meters(lat1, lon1, lat2, lon2):
    """Great-circle distance between two WGS-84 points, in metres."""
    d_lat = (lat2 - lat1) * _DEG_TO_RAD
    d_lon = (lon2 - lon1) * _DEG_TO_RAD
    a = (
        math.sin(d_lat / 2) ** 2
        + math.cos(lat1 * _DEG_TO_RAD)
        * math.cos(lat2 * _DEG_TO_RAD)
        * math.sin(d_lon / 2) ** 2
    )
    return 2 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(a)))


def bearing_degrees(lat1, lon1, lat2, lon2):
    """Initial great-circle bearing from the first point to the second.

    Result is in degrees clockwise from true north, wrapped to ``[-180, 180)``.
    """
    phi1 = lat1 * _DEG_TO_RAD
    phi2 = lat2 * _DEG_TO_RAD
    d_lon = (lon2 - lon1) * _DEG_TO_RAD
    y = math.sin(d_lon) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(d_lon)
    return wrap_degrees(math.atan2(y, x) * _RAD_TO_DEG)


def wrap_degrees(degrees):
    """Wraps an angle to the range ``[-180, 180)``."""
    return ((degrees + 180.0) % 360.0 + 360.0) % 360.0 - 180.0


def turn_error_degrees(heading_degrees, target_bearing_degrees):
    """Smallest signed turn from a heading onto a target bearing.

    Positive means the target is to the right (clockwise), negative to the left.
    """
    return wrap_degrees(target_bearing_degrees - heading_degrees)


def destination_point(latitude, longitude, bearing_degrees_, distance_meters_):
    """Moves a point along a great circle by the given distance and initial bearing.

    Returns ``(latitude, longitude)`` in degrees. Used for track replay and tests.
    """
    angular = distance_meters_ / EARTH_RADIUS_M
    theta = bearing_degrees_ * _DEG_TO_RAD
    phi1 = latitude * _DEG_TO_RAD
    lambda1 = longitude * _DEG_TO_RAD

    sin_phi2 = math.sin(phi1) * math.cos(angular) + math.cos(phi1) * math.sin(angular) * math.cos(theta)
    phi2 = math.asin(max(-1.0, min(1.0, sin_phi2)))
    lambda2 = lambda1 + math.atan2(
        math.sin(theta) * math.sin(angular) * math.cos(phi1),
        math.cos(angular) - math.sin(phi1) * sin_phi2,
    )

    return phi2 * _RAD_TO_DEG, wrap_degrees(lambda2 * _RAD_TO_DEG)
