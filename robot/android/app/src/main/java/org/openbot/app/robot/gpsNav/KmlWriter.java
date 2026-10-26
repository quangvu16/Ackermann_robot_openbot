package org.openbot.app.robot.gpsNav;

import java.util.List;
import java.util.Locale;

/**
 * Writes a mission and the driven track as KML, matching the export Mission Planner produces for a
 * flown mission so the result can be dropped straight into Google Earth.
 */
public final class KmlWriter {

  private KmlWriter() {}

  /**
   * @param mission the planned mission (home plus waypoints)
   * @param track positions as {@code {lat, lon}} pairs in visit order; may be empty
   */
  public static String build(String documentName, Mission mission, List<double[]> track) {
    StringBuilder kml = new StringBuilder();
    kml.append("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    kml.append("<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n");
    kml.append("  <Document>\n");
    kml.append("    <name>").append(escape(documentName)).append("</name>\n");

    appendPlacemark(kml, "Home", mission.getHomeLatitude(), mission.getHomeLongitude());
    List<Waypoint> waypoints = mission.getWaypoints();
    for (int i = 0; i < waypoints.size(); i++) {
      Waypoint waypoint = waypoints.get(i);
      String label =
          waypoint.getName().isEmpty() ? "WP" + (i + 1) : "WP" + (i + 1) + " " + waypoint.getName();
      appendPlacemark(kml, label, waypoint.getLatitude(), waypoint.getLongitude());
    }

    if (waypoints.size() > 1 || mission.hasHome()) {
      kml.append("    <Placemark>\n");
      kml.append("      <name>Mission Path</name>\n");
      kml.append(
          "      <Style><LineStyle><color>ff00aaff</color><width>3</width></LineStyle></Style>\n");
      kml.append("      <LineString><tessellate>1</tessellate><coordinates>");
      if (mission.hasHome()) {
        kml.append(coord(mission.getHomeLatitude(), mission.getHomeLongitude()));
      }
      for (Waypoint waypoint : waypoints) {
        kml.append(coord(waypoint.getLatitude(), waypoint.getLongitude()));
      }
      kml.append("</coordinates></LineString>\n");
      kml.append("    </Placemark>\n");
    }

    if (track != null && !track.isEmpty()) {
      kml.append("    <Placemark>\n");
      kml.append("      <name>Track</name>\n");
      kml.append(
          "      <Style><LineStyle><color>ff00ff00</color><width>3</width></LineStyle></Style>\n");
      kml.append("      <LineString><tessellate>1</tessellate><coordinates>");
      for (double[] position : track) {
        if (position != null && position.length >= 2) {
          kml.append(coord(position[0], position[1]));
        }
      }
      kml.append("</coordinates></LineString>\n");
      kml.append("    </Placemark>\n");
    }

    kml.append("  </Document>\n");
    kml.append("</kml>\n");
    return kml.toString();
  }

  private static void appendPlacemark(StringBuilder kml, String name, double lat, double lon) {
    if (Double.isNaN(lat) || Double.isNaN(lon)) {
      return;
    }
    kml.append("    <Placemark>\n");
    kml.append("      <name>").append(escape(name)).append("</name>\n");
    kml.append("      <Point><coordinates>")
        .append(coord(lat, lon))
        .append("</coordinates></Point>\n");
    kml.append("    </Placemark>\n");
  }

  private static String coord(double lat, double lon) {
    return String.format(Locale.US, "%.8f,%.8f,0 ", lon, lat);
  }

  private static String escape(String text) {
    return text == null ? "" : text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;");
  }
}
