"""GPS waypoints and missions, with Mission Planner ``.waypoints`` interchange.

Mirrors ``org.openbot.app.robot.gpsNav.Waypoint`` and ``Mission`` on Android.
"""

from gpsnav.geo import bearing_degrees, distance_meters

WPL_HEADER = "QGC WPL 110"
CMD_NAV_WAYPOINT = 16


class Waypoint:
    """A single GPS waypoint, the ground-robot equivalent of a Mission Planner entry.

    Waypoints only carry the horizontal target plus an acceptance radius; unlike
    an aircraft mission there is no altitude leg.
    """

    def __init__(self, latitude, longitude, name="", acceptance_radius_meters=2.5):
        self.latitude = float(latitude)
        self.longitude = float(longitude)
        self.name = name or ""
        self.acceptance_radius_meters = max(0.5, float(acceptance_radius_meters))

    def distance_from(self, latitude, longitude):
        """Distance in metres from the given position to this waypoint."""
        return distance_meters(latitude, longitude, self.latitude, self.longitude)

    def bearing_from(self, latitude, longitude):
        """Initial bearing in degrees from the given position to this waypoint."""
        return bearing_degrees(latitude, longitude, self.latitude, self.longitude)

    def __repr__(self):
        label = self.name or "-"
        return "Waypoint(%s lat=%.7f lon=%.7f r=%.1fm)" % (
            label,
            self.latitude,
            self.longitude,
            self.acceptance_radius_meters,
        )


class Mission:
    """An ordered GPS mission: a home position plus a list of waypoints to visit.

    Serialises to the QGroundControl / Mission Planner ``.waypoints`` format
    (``QGC WPL 110``) so missions can be planned in Mission Planner and driven
    here, or the other way round. Only the horizontal component is meaningful for
    a ground robot, so the altitude column is written as zero and ignored on read.
    """

    def __init__(self):
        self.home_latitude = None
        self.home_longitude = None
        self._waypoints = []

    # region bookkeeping

    def clear_waypoints(self):
        self._waypoints = []

    def add_waypoint(self, waypoint, index=None):
        if waypoint is None:
            return
        if index is None:
            self._waypoints.append(waypoint)
        else:
            self._waypoints.insert(max(0, min(index, len(self._waypoints))), waypoint)

    def remove_waypoint(self, index):
        if 0 <= index < len(self._waypoints):
            del self._waypoints[index]

    def move_waypoint(self, from_index, to_index):
        if not (0 <= from_index < len(self._waypoints)):
            return
        if not (0 <= to_index < len(self._waypoints)):
            return
        self._waypoints.insert(to_index, self._waypoints.pop(from_index))

    def set_waypoints(self, replacement):
        self._waypoints = [w for w in (replacement or []) if w is not None]

    @property
    def waypoints(self):
        return list(self._waypoints)

    def get_waypoint(self, index):
        return self._waypoints[index] if 0 <= index < len(self._waypoints) else None

    def __len__(self):
        return len(self._waypoints)

    def __iter__(self):
        return iter(self._waypoints)

    @property
    def is_empty(self):
        return not self._waypoints

    @property
    def has_home(self):
        return self.home_latitude is not None and self.home_longitude is not None

    def set_home(self, latitude, longitude):
        self.home_latitude = float(latitude)
        self.home_longitude = float(longitude)

    def total_distance_meters(self):
        """Total along-track length in metres, home to last waypoint."""
        if not self._waypoints:
            return 0.0

        total = 0.0
        if self.has_home:
            prev_lat, prev_lon = self.home_latitude, self.home_longitude
            has_prev = True
        else:
            prev_lat = prev_lon = 0.0
            has_prev = False

        for waypoint in self._waypoints:
            if has_prev:
                total += distance_meters(prev_lat, prev_lon, waypoint.latitude, waypoint.longitude)
            prev_lat, prev_lon = waypoint.latitude, waypoint.longitude
            has_prev = True
        return total

    # endregion

    # region Mission Planner .waypoints codec

    def to_waypoints_file(self):
        """Serialises this mission to the Mission Planner / QGC waypoint format.

        Index 0 is the home position (``current`` flag set), matching what Mission
        Planner writes. When no home has been recorded yet the first waypoint is
        used so the file stays valid.
        """
        lines = [WPL_HEADER]
        if self.has_home or self._waypoints:
            if self.has_home:
                lat, lon = self.home_latitude, self.home_longitude
            else:
                lat, lon = self._waypoints[0].latitude, self._waypoints[0].longitude
            lines.append(_format_wpl_line(0, 1, lat, lon))
        for i, waypoint in enumerate(self._waypoints):
            lines.append(_format_wpl_line(i + 1, 0, waypoint.latitude, waypoint.longitude))
        return "\n".join(lines) + "\n"

    @staticmethod
    def parse_waypoints_file(contents):
        """Parses a Mission Planner / QGroundControl waypoint file.

        Row 0 is treated as the home position; every subsequent ``NAV_WAYPOINT``
        row becomes a mission waypoint. Non-navigation commands are skipped rather
        than rejected so files planned for a rover with extra ``DO_`` commands
        still load their waypoints.
        """
        mission = Mission()
        if contents is None:
            return mission

        lines = contents.splitlines()
        if not lines:
            raise ValueError("Empty waypoint file")

        header = lines[0].strip()
        if not header.startswith("QGC WPL"):
            raise ValueError("Unsupported waypoint file header: %s" % header)

        for row_index, line in enumerate(lines[1:]):
            line = line.strip()
            if not line:
                continue

            fields = line.split("\t")
            if len(fields) < 11:
                fields = line.split()
            if len(fields) < 11:
                raise ValueError("Malformed waypoint row: %s" % line)

            try:
                command = int(fields[3])
                lat = float(fields[8])
                lon = float(fields[9])
            except ValueError as exc:
                raise ValueError("Malformed waypoint row: %s" % line) from exc

            if row_index == 0:
                mission.set_home(lat, lon)
            elif command == CMD_NAV_WAYPOINT:
                mission.add_waypoint(Waypoint(lat, lon))
        return mission

    # endregion


def _format_wpl_line(index, current, lat, lon):
    # index, current, frame, command, p1, p2, p3, p4, x, y, z, autocontinue
    return "%d\t%d\t3\t%d\t0\t0\t0\t0\t%.8f\t%.8f\t0\t1" % (
        index,
        current,
        CMD_NAV_WAYPOINT,
        lat,
        lon,
    )
