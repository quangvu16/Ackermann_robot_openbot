package org.openbot.app.robot.gpsNav;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.io.IOException;
import org.junit.Test;

/** Pure-JVM tests for the Mission Planner {@code .waypoints} codec and mission bookkeeping. */
public class MissionTest {

  @Test
  public void waypointsRoundTripPreservesHomeAndOrder() throws IOException {
    Mission original = new Mission();
    original.setHome(47.3977420, 8.5455940);
    original.addWaypoint(new Waypoint(47.3981000, 8.5460000));
    original.addWaypoint(new Waypoint(47.3985000, 8.5470000, "", 5f));
    original.addWaypoint(new Waypoint(47.3990000, 8.5480000));

    String file = original.toWaypointsFile();
    Mission parsed = Mission.parseWaypointsFile(file);

    assertTrue(parsed.hasHome());
    assertEquals(original.getHomeLatitude(), parsed.getHomeLatitude(), 1e-7);
    assertEquals(original.getHomeLongitude(), parsed.getHomeLongitude(), 1e-7);
    assertEquals(original.size(), parsed.size());

    for (int i = 0; i < original.size(); i++) {
      Waypoint expected = original.getWaypoint(i);
      Waypoint actual = parsed.getWaypoint(i);
      assertEquals(expected.getLatitude(), actual.getLatitude(), 1e-7);
      assertEquals(expected.getLongitude(), actual.getLongitude(), 1e-7);
    }
  }

  @Test
  public void fileHeaderMatchesMissionPlanner() {
    Mission mission = new Mission();
    mission.setHome(47.0, 8.0);
    mission.addWaypoint(new Waypoint(47.1, 8.1));

    String file = mission.toWaypointsFile();
    assertTrue(file.startsWith("QGC WPL 110\n"));
    // Home is row 0 with the current flag set; waypoints follow in order.
    String[] lines = file.split("\n");
    assertEquals(3, lines.length);
    assertTrue(lines[1].startsWith("0\t1\t"));
    assertTrue(lines[2].startsWith("1\t0\t"));
  }

  @Test
  public void nonNavigationCommandsAreSkippedOnRead() throws IOException {
    String file =
        "QGC WPL 110\n"
            + "0\t1\t3\t16\t0\t0\t0\t0\t47.3977420\t8.5455940\t0\t1\n"
            // DO_JUMP (177) is not a waypoint and must not become one.
            + "1\t0\t3\t177\t0\t0\t0\t0\t47.3980000\t8.5460000\t0\t1\n"
            + "2\t0\t3\t16\t0\t0\t0\t0\t47.3985000\t8.5470000\t0\t1\n";

    Mission mission = Mission.parseWaypointsFile(file);

    assertEquals(1, mission.size());
    assertEquals(47.3985000, mission.getWaypoint(0).getLatitude(), 1e-7);
  }

  @Test
  public void rejectsForeignHeader() {
    try {
      Mission.parseWaypointsFile("not a waypoint file\n");
      fail("expected IOException for a foreign header");
    } catch (IOException expected) {
      assertTrue(expected.getMessage().contains("header"));
    }
  }

  @Test
  public void rejectsMalformedRow() {
    try {
      Mission.parseWaypointsFile("QGC WPL 110\n0\t1\t3\n");
      fail("expected IOException for a malformed row");
    } catch (IOException expected) {
      assertTrue(expected.getMessage().contains("Malformed"));
    }
  }

  @Test
  public void editsMaintainOrder() {
    Mission mission = new Mission();
    mission.addWaypoint(new Waypoint(1.0, 1.0));
    mission.addWaypoint(new Waypoint(2.0, 2.0));
    mission.addWaypoint(new Waypoint(3.0, 3.0));

    mission.moveWaypoint(2, 0);
    assertEquals(3.0, mission.getWaypoint(0).getLatitude(), 1e-9);
    assertEquals(1.0, mission.getWaypoint(1).getLatitude(), 1e-9);
    assertEquals(2.0, mission.getWaypoint(2).getLatitude(), 1e-9);

    mission.removeWaypoint(1);
    assertEquals(2, mission.size());
    assertEquals(3.0, mission.getWaypoint(0).getLatitude(), 1e-9);
    assertEquals(2.0, mission.getWaypoint(1).getLatitude(), 1e-9);

    mission.clearWaypoints();
    assertTrue(mission.isEmpty());
  }

  @Test
  public void totalDistanceRunsHomeThroughWaypoints() {
    Mission mission = new Mission();
    // 1 degree of latitude ≈ 111.19 km.
    mission.setHome(0.0, 0.0);
    mission.addWaypoint(new Waypoint(1.0, 0.0));
    mission.addWaypoint(new Waypoint(2.0, 0.0));

    assertEquals(222389.9, mission.totalDistanceMeters(), 100.0);
  }

  @Test
  public void totalDistanceWithoutHomeStartsAtFirstWaypoint() {
    Mission mission = new Mission();
    mission.addWaypoint(new Waypoint(0.0, 0.0));
    mission.addWaypoint(new Waypoint(1.0, 0.0));

    assertEquals(111194.9, mission.totalDistanceMeters(), 50.0);
  }
}
