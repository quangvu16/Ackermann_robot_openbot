package org.openbot.app.robot.gpsNav;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.StringReader;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/**
 * An ordered GPS mission: a home position plus a list of waypoints to visit.
 *
 * <p>Serialises to the QGroundControl / Mission Planner {@code .waypoints} format ({@code QGC WPL
 * 110}) so missions can be planned in Mission Planner and flown here, or the other way round. Only
 * the horizontal component is meaningful for a ground robot, so the altitude column is written as
 * zero and ignored when reading.
 */
public class Mission {

  /** Home / launch position. {@code null} until the first GPS fix of a mission run. */
  private double homeLatitude = Double.NaN;

  private double homeLongitude = Double.NaN;

  private final List<Waypoint> waypoints = new ArrayList<>();

  /** Removes every waypoint but keeps the home position. */
  public void clearWaypoints() {
    waypoints.clear();
  }

  public void addWaypoint(Waypoint waypoint) {
    if (waypoint != null) {
      waypoints.add(waypoint);
    }
  }

  public void addWaypoint(int index, Waypoint waypoint) {
    if (waypoint == null) {
      return;
    }
    waypoints.add(Math.max(0, Math.min(index, waypoints.size())), waypoint);
  }

  public void removeWaypoint(int index) {
    if (index >= 0 && index < waypoints.size()) {
      waypoints.remove(index);
    }
  }

  public void moveWaypoint(int fromIndex, int toIndex) {
    if (fromIndex < 0
        || fromIndex >= waypoints.size()
        || toIndex < 0
        || toIndex >= waypoints.size()) {
      return;
    }
    Waypoint moved = waypoints.remove(fromIndex);
    waypoints.add(toIndex, moved);
  }

  public void setWaypoints(List<Waypoint> replacement) {
    waypoints.clear();
    if (replacement != null) {
      for (Waypoint waypoint : replacement) {
        addWaypoint(waypoint);
      }
    }
  }

  public List<Waypoint> getWaypoints() {
    return Collections.unmodifiableList(waypoints);
  }

  public Waypoint getWaypoint(int index) {
    return (index < 0 || index >= waypoints.size()) ? null : waypoints.get(index);
  }

  public int size() {
    return waypoints.size();
  }

  public boolean isEmpty() {
    return waypoints.isEmpty();
  }

  public boolean hasHome() {
    return !Double.isNaN(homeLatitude) && !Double.isNaN(homeLongitude);
  }

  public double getHomeLatitude() {
    return homeLatitude;
  }

  public double getHomeLongitude() {
    return homeLongitude;
  }

  public void setHome(double latitude, double longitude) {
    this.homeLatitude = latitude;
    this.homeLongitude = longitude;
  }

  /** Total along-track length in metres, home to last waypoint. */
  public double totalDistanceMeters() {
    if (waypoints.isEmpty()) {
      return 0.0;
    }
    double total = 0.0;
    double prevLat = homeLatitude;
    double prevLon = homeLongitude;
    boolean hasPrev = hasHome();
    for (Waypoint waypoint : waypoints) {
      if (hasPrev) {
        total +=
            GeoMath.distanceMeters(
                prevLat, prevLon, waypoint.getLatitude(), waypoint.getLongitude());
      }
      prevLat = waypoint.getLatitude();
      prevLon = waypoint.getLongitude();
      hasPrev = true;
    }
    return total;
  }

  // region Mission Planner .waypoints codec

  private static final String WPL_HEADER = "QGC WPL 110";
  private static final int CMD_NAV_WAYPOINT = 16;

  /**
   * Serialises this mission to the Mission Planner / QGroundControl waypoint file format.
   *
   * <p>Index 0 is the home position ({@code current} flag set), matching what Mission Planner
   * writes. When no home has been recorded yet the first waypoint is used so the file stays valid.
   */
  public String toWaypointsFile() {
    StringBuilder builder = new StringBuilder();
    builder.append(WPL_HEADER).append('\n');

    if (hasHome() || !waypoints.isEmpty()) {
      double lat = hasHome() ? homeLatitude : waypoints.get(0).getLatitude();
      double lon = hasHome() ? homeLongitude : waypoints.get(0).getLongitude();
      builder.append(formatWplLine(0, 1, lat, lon));
    }
    for (int i = 0; i < waypoints.size(); i++) {
      Waypoint waypoint = waypoints.get(i);
      builder.append(formatWplLine(i + 1, 0, waypoint.getLatitude(), waypoint.getLongitude()));
    }
    return builder.toString();
  }

  private static String formatWplLine(int index, int current, double lat, double lon) {
    // index, current, frame, command, p1, p2, p3, p4, x, y, z, autocontinue
    return String.format(
        Locale.US,
        "%d\t%d\t3\t%d\t0\t0\t0\t0\t%.8f\t%.8f\t0\t1\n",
        index,
        current,
        CMD_NAV_WAYPOINT,
        lat,
        lon);
  }

  /**
   * Parses a Mission Planner / QGroundControl waypoint file.
   *
   * <p>Row 0 is treated as the home position; every subsequent {@code NAV_WAYPOINT} row becomes a
   * mission waypoint. Non-navigation commands are skipped rather than rejected so files planned for
   * a rover with extra DO_ commands still load their waypoints.
   */
  public static Mission parseWaypointsFile(String contents) throws IOException {
    Mission mission = new Mission();
    if (contents == null) {
      return mission;
    }

    try (BufferedReader reader = new BufferedReader(new StringReader(contents))) {
      String line = reader.readLine();
      if (line == null) {
        throw new IOException("Empty waypoint file");
      }
      if (!line.trim().startsWith("QGC WPL")) {
        throw new IOException("Unsupported waypoint file header: " + line.trim());
      }

      int rowIndex = 0;
      while ((line = reader.readLine()) != null) {
        line = line.trim();
        if (line.isEmpty()) {
          continue;
        }
        String[] fields = line.split("\\t");
        if (fields.length < 11) {
          fields = line.split("\\s+");
        }
        if (fields.length < 11) {
          throw new IOException("Malformed waypoint row: " + line);
        }

        int command;
        double lat;
        double lon;
        try {
          command = Integer.parseInt(fields[3].trim());
          lat = Double.parseDouble(fields[8].trim());
          lon = Double.parseDouble(fields[9].trim());
        } catch (NumberFormatException e) {
          throw new IOException("Malformed waypoint row: " + line, e);
        }

        if (rowIndex == 0) {
          mission.setHome(lat, lon);
        } else if (command == CMD_NAV_WAYPOINT) {
          mission.addWaypoint(new Waypoint(lat, lon));
        }
        rowIndex++;
      }
    }
    return mission;
  }

  // endregion
}
