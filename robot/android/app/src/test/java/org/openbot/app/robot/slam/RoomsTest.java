package org.openbot.app.robot.slam;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.junit.Test;

/** Rooms & Zones (Valetudo-style): rooms, restrictions, lanes and the job runner. */
public class RoomsTest {

  /**
   * Two rooms joined by a 1 m doorway: A is 4 x 4 m (x 0..4), B is 3 x 4 m (x 4..7). The doorway
   * is in the wall at x = 4, from y = 1.5 to 2.5.
   */
  static OccupancyGrid twoRooms() {
    OccupancyGrid g = OccupancyGrid.indoorDefault();
    for (int cy = g.cellY(0); cy <= g.cellY(4); cy++)
      for (int cx = g.cellX(0); cx <= g.cellX(7); cx++) g.set(cx, cy, -2f);
    double[][] walls = {
      {0, 0, 7, 0}, {7, 0, 7, 4}, {7, 4, 0, 4}, {0, 4, 0, 0}, {4, 0, 4, 1.5}, {4, 2.5, 4, 4},
    };
    for (double[] s : walls) {
      int n = (int) (Math.hypot(s[2] - s[0], s[3] - s[1]) / 0.02) + 1;
      for (int k = 0; k <= n; k++) {
        double t = (double) k / n;
        g.set(g.cellX(s[0] + (s[2] - s[0]) * t), g.cellY(s[1] + (s[3] - s[1]) * t), 3f);
      }
    }
    return g;
  }

  @Test
  public void jsonReadsWhatValetudoSends() {
    Object o = Json.parse("{\"action\":\"goto\",\"coordinates\":{\"x\":1200,\"y\":-3.5e1},\"ids\":[\"1\",2],\"ok\":true,\"n\":null}");
    assertEquals("goto", Json.str(Json.get(o, "action")));
    assertEquals(1200, Json.num(Json.get(o, "coordinates", "x"), 0), 1e-9);
    assertEquals(-35, Json.num(Json.get(o, "coordinates", "y"), 0), 1e-9);
    assertEquals(2, Json.arr(Json.get(o, "ids")).size());
    assertEquals(Boolean.TRUE, Json.get(o, "ok"));
    assertEquals("Küche \"1\"", Json.str(Json.parse(Json.quote("Küche \"1\""))));
    assertNull(Json.tryParse("{bad"));
  }

  @Test
  public void splitsTwoRoomsAtTheDoorway() {
    OccupancyGrid g = twoRooms();
    RoomSegmenter.Result r = RoomSegmenter.segment(g, new RoomSegmenter.Params());
    assertEquals(2, r.count);
    RoomMap m = new RoomMap();
    m.setRooms(r, RoomMap.stampOf(g));
    assertEquals("largest room first", 1, m.roomAt(2, 2));
    assertEquals(2, m.roomAt(5.5, 2));
    assertEquals(0, m.roomAt(10, 10));
    assertEquals(15.0, m.area(1), 1.5); // 4 x 4 minus the walls
    assertEquals(11.0, m.area(2), 1.5);
    double[] c = m.centroid(1);
    assertEquals(2.0, c[0], 0.3);
    assertEquals("Room 1", m.name(1));
  }

  @Test
  public void roomsAndRestrictionsSurviveJson() {
    OccupancyGrid g = twoRooms();
    RoomMap m = new RoomMap();
    m.setRooms(RoomSegmenter.segment(g, new RoomSegmenter.Params()), RoomMap.stampOf(g));
    m.names.put(1, "Living \"room\"");
    m.zones.add(RoomMap.box(0.5, 0.5, 1.5, 1.5));
    m.noGo.add(RoomMap.box(5, 3, 6, 3.5));
    m.walls.add(new double[] {1, 3, 2, 3});
    m.home = new double[] {0.5, 0.5};
    RoomMap n = RoomMap.fromJson(m.toJson());
    assertEquals(m.mapStamp, n.mapStamp);
    assertEquals("Living \"room\"", n.name(1));
    assertEquals(2, n.roomAt(5.5, 2));
    assertEquals(1, n.zones.size());
    assertEquals(1.5, n.zones.get(0)[4], 1e-9);
    assertEquals(0.5, n.home[1], 1e-9);
    assertTrue(n.blocked(5.5, 3.2));
    assertTrue("near the wall line", n.blocked(1.5, 3.1));
    assertTrue(!n.blocked(1.5, 2.0));
  }

  @Test
  public void noGoAreaAndVirtualWallBlockTheDoorway() {
    OccupancyGrid g = twoRooms();
    Navigator.Params p = new Navigator.Params();
    assertNotNull("open door: a path", Navigator.plan(g, 2, 2, 5.5, 2, p));
    RoomMap m = new RoomMap();
    m.noGo.add(RoomMap.box(3.6, 1.3, 4.4, 2.7));
    p.keepOut = m;
    assertNull("no-go area over the doorway", Navigator.plan(g, 2, 2, 5.5, 2, p));
    m.noGo.clear();
    m.walls.add(new double[] {4, 1.4, 4, 2.6});
    assertNull("virtual wall across the doorway", Navigator.plan(g, 2, 2, 5.5, 2, p));
    m.walls.clear();
    m.noGo.add(RoomMap.box(1.5, 1.0, 2.5, 3.0)); // a box in the middle of room A
    List<double[]> path = Navigator.plan(g, 0.8, 2, 3.4, 2, p);
    assertNotNull("goes round the no-go box", path);
    for (double[] q : path) assertTrue("inside no-go: " + q[0] + "," + q[1], !RoomMap.inside(m.noGo.get(0), q[0], q[1]));
  }

  @Test
  public void lanesCoverTheRoomInsideItsWalls() {
    OccupancyGrid g = twoRooms();
    RoomMap m = new RoomMap();
    m.setRooms(RoomSegmenter.segment(g, new RoomSegmenter.Params()), RoomMap.stampOf(g));
    Navigator.Params p = new Navigator.Params();
    List<double[]> wps = CoveragePlanner.plan(g, p, (x, y) -> m.roomAt(x, y) == 1, 0.5, new Pose(0.6, 0.6, 0));
    assertTrue("lanes: " + wps.size(), wps.size() >= 8 && wps.size() <= 20);
    for (double[] q : wps) {
      assertEquals(1, m.roomAt(q[0], q[1]));
      assertTrue("off the walls", q[0] > 0.2 && q[0] < 3.8 && q[1] > 0.2 && q[1] < 3.8);
    }
    double[] first = wps.get(0);
    assertTrue("starts near the car", Math.hypot(first[0] - 0.6, first[1] - 0.6) < 1.0);
    // A zone gives lanes only inside it.
    double[] zone = RoomMap.box(5, 0.5, 6.5, 2);
    List<double[]> z = CoveragePlanner.plan(g, p, (x, y) -> RoomMap.inside(zone, x, y), 0.5, null);
    assertTrue(z.size() >= 4);
    for (double[] q : z) assertTrue(RoomMap.inside(zone, q[0], q[1]));
  }

  @Test
  public void missionPatrolsAZoneThenIsIdleAndReturnsHome() {
    OccupancyGrid g = twoRooms();
    Navigator nav = new Navigator(new Navigator.Params());
    CleaningMission job = new CleaningMission(nav);
    double[] zone = RoomMap.box(0.6, 0.6, 3.0, 2.0);
    List<double[]> wps = CoveragePlanner.plan(g, nav.getParams(), (x, y) -> RoomMap.inside(zone, x, y), 0.7, new Pose(1, 1, 0));
    job.start(wps, CleaningMission.Flag.ZONE, "Zone 1");
    assertEquals(CleaningMission.Status.CLEANING, job.getStatus(null));
    assertEquals(CleaningMission.Flag.ZONE, job.getFlag());
    double[] st = {1, 1, 0};
    int k = 0;
    for (; k < 12000 && job.isBusy(); k++) {
      Navigator.Command c = job.step(new Pose(st[0], st[1], st[2]), g, Double.POSITIVE_INFINITY, k * 0.1);
      if (c != null) st = SlamTest.drive(st, c.throttle, c.steer);
      if (k == 50) { // pause and resume keep the place in the list
        int at = job.getIndex();
        job.pause();
        assertEquals(CleaningMission.Status.PAUSED, job.getStatus(null));
        assertEquals(CleaningMission.Flag.RESUMABLE, job.getFlag());
        assertNull(job.step(new Pose(st[0], st[1], st[2]), g, Double.POSITIVE_INFINITY, k * 0.1));
        assertTrue(job.resume());
        assertEquals(at, job.getIndex());
        assertEquals(CleaningMission.Flag.ZONE, job.getFlag());
      }
    }
    assertEquals("finished in " + k + " steps", CleaningMission.Status.IDLE, job.getStatus(null));
    assertTrue("most lanes driven, skipped " + job.getSkipped(), job.getSkipped() <= wps.size() / 4);

    job.returnHome();
    assertEquals(CleaningMission.Status.ERROR, job.getStatus(null)); // no home yet
    job.clearError();
    job.setHome(new double[] {0.7, 3.2});
    job.returnHome();
    assertEquals(CleaningMission.Status.RETURNING, job.getStatus(null));
    for (k = 0; k < 3000 && job.isBusy(); k++) {
      Navigator.Command c = job.step(new Pose(st[0], st[1], st[2]), g, Double.POSITIVE_INFINITY, k * 0.1);
      if (c != null) st = SlamTest.drive(st, c.throttle, c.steer);
    }
    assertEquals(CleaningMission.Status.DOCKED, job.getStatus(new Pose(st[0], st[1], st[2])));
  }

  @Test
  public void unreachableTargetIsAnError() {
    OccupancyGrid g = twoRooms();
    Navigator.Params p = new Navigator.Params();
    RoomMap m = new RoomMap();
    m.walls.add(new double[] {4, 1.4, 4, 2.6});
    p.keepOut = m;
    Navigator nav = new Navigator(p);
    CleaningMission job = new CleaningMission(nav);
    job.goTo(5.5, 2);
    for (int k = 0; k < 50 && job.isBusy(); k++) job.step(new Pose(2, 2, 0), g, Double.POSITIVE_INFINITY, k * 0.1);
    assertEquals(CleaningMission.Status.ERROR, job.getStatus(null));
    assertTrue(job.getError().length() > 0);
  }

  @Test
  public void itemNearestATapIsFound() {
    RoomMap m = new RoomMap();
    m.zones.add(RoomMap.box(0, 0, 1, 1));
    m.noGo.add(RoomMap.box(3, 3, 4, 4));
    m.walls.add(new double[] {6, 0, 6, 2});
    assertEquals(0, m.itemAt(0.5, 0.5, 0.2)[0]);
    assertEquals(1, m.itemAt(4.1, 3.5, 0.2)[0]);
    int[] w = m.itemAt(6.1, 1, 0.2);
    assertEquals(2, w[0]);
    assertEquals(0, w[1]);
    assertNull(m.itemAt(9, 9, 0.2));
  }

  static Map<String, Object> obj(Object o) {
    return Json.obj(o);
  }

  static List<double[]> list(double[]... a) {
    List<double[]> l = new ArrayList<>();
    for (double[] x : a) l.add(x);
    return l;
  }
}
