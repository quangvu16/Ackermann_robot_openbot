package org.openbot.app.robot.slam;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.util.List;
import java.util.Random;
import org.junit.Test;

/** SLAM and navigation in a simulated 6 m x 4 m room with a box in it. */
public class SlamTest {

  /** Walls as segments {x0, y0, x1, y1}. */
  static final double[][] ROOM = {
    {-2, -1.5, 4, -1.5}, {4, -1.5, 4, 2.5}, {4, 2.5, -2, 2.5}, {-2, 2.5, -2, -1.5},
    {1.5, 0.5, 2.0, 0.5}, {2.0, 0.5, 2.0, 1.5}, {2.0, 1.5, 1.5, 1.5}, {1.5, 1.5, 1.5, 0.5}, // box
  };

  /** One 360-beam revolution from the true pose; angles + left, like the app passes them. */
  static double[][] scan(double x, double y, double th, Random noise) {
    double[] a = new double[360];
    double[] r = new double[360];
    for (int i = 0; i < 360; i++) {
      a[i] = -Math.toRadians(i + 0.5); // bins go clockwise
      double g = th + a[i];
      double dx = Math.cos(g), dy = Math.sin(g);
      double best = 9;
      for (double[] s : ROOM) {
        double ex = s[2] - s[0], ey = s[3] - s[1];
        double den = dx * ey - dy * ex;
        if (Math.abs(den) < 1e-9) continue;
        double t = ((s[0] - x) * ey - (s[1] - y) * ex) / den;
        double u = ((s[0] - x) * dy - (s[1] - y) * dx) / den;
        if (t > 0 && u >= 0 && u <= 1) best = Math.min(best, t);
      }
      r[i] = best + noise.nextGaussian() * 0.01;
    }
    return new double[][] {a, r};
  }

  @Test
  public void slamTracksAPathAroundTheRoom() {
    SlamEngine slam = new SlamEngine(OccupancyGrid.indoorDefault());
    Random noise = new Random(1);
    double x = 0, y = 0, th = 0, drift = 0;
    for (int step = 0; step < 100; step++) { // stays inside the room
      double[][] s = scan(x, y, th, noise);
      slam.update(s[0], s[1], th + drift); // gyro yaw with a slow drift
      // Drive: forward 4 cm per scan, turning gently left for a while.
      double turn = step > 30 && step < 70 ? 0.03 : 0;
      th += turn;
      x += 0.04 * Math.cos(th);
      y += 0.04 * Math.sin(th);
      drift += 0.002;
    }
    double[][] s = scan(x, y, th, noise);
    Pose p = slam.update(s[0], s[1], th + drift);
    assertEquals("x", x, p.x, 0.15);
    assertEquals("y", y, p.y, 0.15);
    assertEquals("heading", 0, Math.toDegrees(Pose.wrap(p.theta - th)), 5);
    assertTrue("walls were mapped", slam.getGrid().knownCells() > 2000);
  }

  @Test
  public void slamWorksWithoutTheGyroToo() {
    SlamEngine slam = new SlamEngine(OccupancyGrid.indoorDefault());
    Random noise = new Random(3);
    double x = 0, y = 0, th = 0;
    for (int step = 0; step < 80; step++) {
      double[][] s = scan(x, y, th, noise);
      slam.update(s[0], s[1], Double.NaN);
      th += step > 20 && step < 50 ? 0.03 : 0;
      x += 0.04 * Math.cos(th);
      y += 0.04 * Math.sin(th);
    }
    double[][] s = scan(x, y, th, noise);
    Pose p = slam.update(s[0], s[1], Double.NaN);
    assertEquals(x, p.x, 0.2);
    assertEquals(y, p.y, 0.2);
    assertEquals(0, Math.toDegrees(Pose.wrap(p.theta - th)), 8);
  }

  @Test
  public void localizationModeLeavesTheMapAlone() {
    SlamEngine slam = new SlamEngine(OccupancyGrid.indoorDefault());
    Random noise = new Random(2);
    double[][] s = scan(0, 0, 0, noise);
    slam.update(s[0], s[1], 0);
    slam.update(s[0], s[1], 0);
    slam.setMapping(false);
    int v = slam.getGrid().getVersion();
    s = scan(0.1, 0, 0, noise);
    Pose p = slam.update(s[0], s[1], 0);
    assertEquals(v, slam.getGrid().getVersion());
    assertEquals(0.1, p.x, 0.08);
  }

  static OccupancyGrid roomGrid() {
    OccupancyGrid g = OccupancyGrid.indoorDefault();
    for (int cy = g.cellY(-1.5); cy <= g.cellY(2.5); cy++)
      for (int cx = g.cellX(-2); cx <= g.cellX(4); cx++) g.set(cx, cy, -2f);
    for (double[] s : ROOM) {
      int n = (int) (Math.hypot(s[2] - s[0], s[3] - s[1]) / 0.02) + 1;
      for (int k = 0; k <= n; k++) {
        double t = (double) k / n;
        g.set(g.cellX(s[0] + (s[2] - s[0]) * t), g.cellY(s[1] + (s[3] - s[1]) * t), 3f);
      }
    }
    return g;
  }

  @Test
  public void plannerGoesAroundTheBoxAndKeepsOffWalls() {
    OccupancyGrid g = roomGrid();
    List<double[]> path = Navigator.plan(g, -1, 1.0, 3.2, 1.0, new Navigator.Params());
    assertNotNull(path);
    for (double[] q : path) {
      boolean inBox = q[0] > 1.35 && q[0] < 2.15 && q[1] > 0.35 && q[1] < 1.65;
      assertTrue("path enters the box: " + q[0] + "," + q[1], !inBox);
    }
    assertEquals(3.2, path.get(path.size() - 1)[0], 1e-9);
    assertNull("goal inside a wall", Navigator.plan(g, -1, 1, 1.75, 1.0, new Navigator.Params()));
  }

  @Test
  public void purePursuitSteersTowardsTheGoalAndReversesWhenItIsBehind() {
    OccupancyGrid g = roomGrid();
    Navigator nav = new Navigator(new Navigator.Params());
    nav.setGoal(0.5, 1.5);
    Navigator.Command c = nav.step(new Pose(-1, 0, 0), g, Double.POSITIVE_INFINITY, 0);
    assertEquals(Navigator.State.DRIVING, c.state);
    assertTrue(c.throttle > 0);
    assertTrue("goal is ahead-left", c.steer > 0);

    nav.setGoal(-1.4, 0);
    c = nav.step(new Pose(0, 0, 0), g, Double.POSITIVE_INFINITY, 0);
    assertTrue("goal behind: reverse", c.throttle < 0);

    nav.setGoal(0.5, 1.5);
    c = nav.step(new Pose(0.45, 1.45, 0), g, Double.POSITIVE_INFINITY, 0);
    assertEquals(Navigator.State.ARRIVED, c.state);
  }

  @Test
  public void blockedThenBacksUp() {
    OccupancyGrid g = roomGrid();
    Navigator nav = new Navigator(new Navigator.Params());
    nav.setGoal(3, 0);
    Pose p = new Pose(0, 0, 0);
    assertEquals(Navigator.State.BLOCKED, nav.step(p, g, 0.2, 0).state);
    assertEquals(Navigator.State.BLOCKED, nav.step(p, g, 0.2, 2.0).state);
    nav.step(p, g, 0.2, 2.6);
    Navigator.Command c = nav.step(p, g, 0.2, 2.7);
    assertEquals(Navigator.State.BACKING_UP, c.state);
    assertTrue(c.throttle < 0);
  }

  @Test
  public void mapSavesAndLoads() throws Exception {
    OccupancyGrid g = roomGrid();
    ByteArrayOutputStream bo = new ByteArrayOutputStream();
    g.write(new DataOutputStream(bo));
    OccupancyGrid h = OccupancyGrid.read(new DataInputStream(new ByteArrayInputStream(bo.toByteArray())));
    assertEquals(g.width, h.width);
    assertTrue(h.isOccupied(h.cellX(4), h.cellY(0)));
    assertTrue(h.get(h.cellX(0), h.cellY(0)) < OccupancyGrid.FREE_THRESHOLD);
    byte[] pgm = g.toPgm();
    assertTrue(new String(pgm, 0, 2, "US-ASCII").equals("P5"));
  }

  /** Bicycle model with the firmware's servo flip in reverse (update_vehicle): returns x, y, th. */
  static double[] drive(double[] st, double throttle, double steer) {
    double v = throttle * 0.6, delta = steer * Math.toRadians(28);
    if (throttle < 0) delta = -delta;
    double th = st[2] + v / 0.26 * Math.tan(delta) * 0.1;
    return new double[] {st[0] + v * Math.cos(th) * 0.1, st[1] + v * Math.sin(th) * 0.1, th};
  }

  @Test
  public void turnsRoundAndReachesAGoalBehindIt() {
    OccupancyGrid g = roomGrid();
    Navigator nav = new Navigator(new Navigator.Params());
    nav.setGoal(-1.0, 0.0); // behind the car
    double[] st = {1.0, 0.0, 0.0};
    for (int k = 0; k < 900; k++) {
      Navigator.Command c = nav.step(new Pose(st[0], st[1], st[2]), g, Double.POSITIVE_INFINITY, k * 0.1);
      if (c.state == Navigator.State.ARRIVED) return;
      st = drive(st, c.throttle, c.steer);
    }
    throw new AssertionError("did not arrive; ended at " + st[0] + ", " + st[1]);
  }
}
