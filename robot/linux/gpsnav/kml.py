"""KML export for a planned mission and the driven track.

Mirrors ``org.openbot.app.robot.gpsNav.KmlWriter`` on Android so an export from
any platform drops straight into Google Earth the same way.
"""

from xml.sax.saxutils import escape as _xml_escape


def build_kml(document_name, mission, track):
    """Builds a KML document.

    Args:
        document_name: name given to the KML ``Document``
        mission: a :class:`~gpsnav.mission.Mission`
        track: iterable of ``(latitude, longitude)`` pairs in visit order

    Returns:
        str: complete KML text
    """
    parts = [
        '<?xml version="1.0" encoding="UTF-8"?>\n',
        '<kml xmlns="http://www.opengis.net/kml/2.2">\n',
        "  <Document>\n",
        "    <name>%s</name>\n" % _xml_escape(document_name or ""),
    ]

    if mission.has_home:
        _append_placemark(parts, "Home", mission.home_latitude, mission.home_longitude)

    waypoints = mission.waypoints
    for i, waypoint in enumerate(waypoints):
        label = "WP%d" % (i + 1)
        if waypoint.name:
            label += " " + waypoint.name
        _append_placemark(parts, label, waypoint.latitude, waypoint.longitude)

    if len(waypoints) > 1 or mission.has_home:
        parts.append("    <Placemark>\n")
        parts.append("      <name>Mission Path</name>\n")
        parts.append(
            "      <Style><LineStyle><color>ff00aaff</color><width>3</width></LineStyle></Style>\n"
        )
        parts.append("      <LineString><tessellate>1</tessellate><coordinates>")
        if mission.has_home:
            parts.append(_coord(mission.home_latitude, mission.home_longitude))
        for waypoint in waypoints:
            parts.append(_coord(waypoint.latitude, waypoint.longitude))
        parts.append("</coordinates></LineString>\n")
        parts.append("    </Placemark>\n")

    track = list(track or [])
    if track:
        parts.append("    <Placemark>\n")
        parts.append("      <name>Track</name>\n")
        parts.append(
            "      <Style><LineStyle><color>ff00ff00</color><width>3</width></LineStyle></Style>\n"
        )
        parts.append("      <LineString><tessellate>1</tessellate><coordinates>")
        for position in track:
            if position is not None and len(position) >= 2:
                parts.append(_coord(position[0], position[1]))
        parts.append("</coordinates></LineString>\n")
        parts.append("    </Placemark>\n")

    parts.append("  </Document>\n")
    parts.append("</kml>\n")
    return "".join(parts)


def _append_placemark(parts, name, lat, lon):
    if lat is None or lon is None:
        return
    parts.append("    <Placemark>\n")
    parts.append("      <name>%s</name>\n" % _xml_escape(name))
    parts.append(
        "      <Point><coordinates>%s</coordinates></Point>\n" % _coord(lat, lon)
    )
    parts.append("    </Placemark>\n")


def _coord(lat, lon):
    return "%.8f,%.8f,0 " % (lon, lat)
