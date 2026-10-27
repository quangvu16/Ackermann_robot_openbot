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

  // region Cartographer-style parts

  static double[][] points(double[][] s) {
    int n = 0;
    for (double r : s[1]) if (r < SlamEngine.MAX_RANGE) n++;
    double[] px = new double[n], py = new double[n];
    int k = 0;
    for (int i = 0; i < s[0].length; i++) {
      if (s[1][i] >= SlamEngine.MAX_RANGE) continue;
      px[k] = s[1][i] * Math.cos(s[0][i]);
      py[k] = s[1][i] * Math.sin(s[0][i]);
      k++;
    }
    return new double[][] {px, py};
  }

  @Test
  public void correlativeAndCeresMatcherRecoverASmallOffset() {
    OccupancyGrid g = roomGrid();
    double[][] p = points(scan(0.3, 0.2, 0.4, new Random(4)));
    ScanMatcher m = new ScanMatcher();
    Pose est = m.match(g, new Pose(0.42, 0.11, 0.4 + Math.toRadians(9)), p[0], p[1]);
    assertEquals(0.3, est.x, 0.03);
    assertEquals(0.2, est.y, 0.03);
    assertEquals(0, Math.toDegrees(Pose.wrap(est.theta - 0.4)), 1.0);
    assertTrue("well constrained in a room", m.lastConstraint > 0.3);
  }

  @Test
  public void branchAndBoundFindsAFarOffsetLikeBruteForce() {
    OccupancyGrid g = roomGrid();
    double[][] p = points(scan(1.0, 0.0, 0.3, new Random(5)));
    double[][] lp = SlamEngine.adaptiveVoxel(p[0], p[1], 0.9, 100);
    FastCorrelativeMatcher f = new FastCorrelativeMatcher(g);
    FastCorrelativeMatcher.Result r = f.match(new Pose(-0.5, 1.0, 0.0), lp[0], lp[1], 3, Math.toRadians(30), 0.5);
    assertNotNull(r);
    assertEquals(1.0, r.pose.x, 0.06);
    assertEquals(0.0, r.pose.y, 0.06);
    assertEquals(0.3, r.pose.theta, Math.toRadians(1.5));
  }

  @Test
  public void globalLocalizationFindsTheCarOnASavedMapWithNoHint() {
    SlamEngine slam = new SlamEngine(OccupancyGrid.indoorDefault());
    slam.setGrid(roomGrid());
    slam.setMapping(false);
    slam.requestGlobalLocalization();
    Random noise = new Random(6);
    double[][] s = scan(3.0, -0.6, 2.2, noise);
    Pose p = slam.update(s[0], s[1], Double.NaN);
    assertTrue(slam.wasRelocalizeFound());
    assertEquals(1, slam.getRelocalizeSeq());
    assertEquals(3.0, p.x, 0.1);
    assertEquals(-0.6, p.y, 0.1);
    assertEquals(0, Math.toDegrees(Pose.wrap(p.theta - 2.2)), 3);
  }

  @Test
  public void poseGraphSpreadsTheDriftAndClosesTheLoop() {
    // 12 submaps round a 3 m circle; local SLAM over-turns 2 degrees per link (drift).
    PoseGraph g = new PoseGraph();
    int n = 12;
    double step = 2 * Math.PI / n, chord = 2 * 3 * Math.sin(step / 2);
    Pose est = new Pose(3, 0, Math.PI / 2 + step / 2);
    Pose measured = new Pose(chord, 0, step + Math.toRadians(2));
    g.add(est, true);
    for (int i = 1; i < n; i++) {
      est = est.compose(measured);
      g.add(est, false);
      g.addConstraint(new PoseGraph.Constraint(i - 1, i, measured, PoseGraph.SEQ_T, PoseGraph.SEQ_R, false));
    }
    double before = g.get(n - 1).distanceTo(g.get(0).compose(new Pose(chord, 0, step).inverse()));
    // Loop closure: the last submap seen from the first, as it truly is.
    Pose truthLast = new Pose(3 * Math.cos(-step), 3 * Math.sin(-step), Math.PI / 2 - step / 2);
    g.addConstraint(new PoseGraph.Constraint(0, n - 1, g.get(0).inverse().compose(truthLast),
        PoseGraph.LOOP_T, PoseGraph.LOOP_R, true));
    g.optimize();
    double after = g.get(n - 1).distanceTo(truthLast);
    assertTrue("drift before " + before, before > 1.0);
    assertTrue("error after " + after, after < 0.05);
  }

  @Test
  public void loopClosureKeepsTheMapStraightAfterLapsOfTheRoom() {
    SlamEngine slam = new SlamEngine(OccupancyGrid.indoorDefault());
    Random noise = new Random(8);
    double drift = 0, maxErr = 0;
    double[][] last = null;
    Pose p = null;
    slam.resetPose(new Pose(1.3, 0.5, Math.PI / 2)); // where the lap starts
    for (int step = 0; step < 330; step++) { // ~2 laps of a 1.1 m circle round the middle of the room
      double t = step * 0.04 / 1.1;
      double x = 0.2 + 1.1 * Math.cos(t), y = 0.5 + 1.1 * Math.sin(t), th = t + Math.PI / 2;
      double[][] s = scan(x, y, th, noise);
      drift += 0.003;
      p = slam.update(s[0], s[1], th + drift, step * 0.1);
      maxErr = Math.max(maxErr, Math.hypot(p.x - x, p.y - y));
    }
    assertTrue("submaps were made", slam.getSubmapCount() >= 3);
    assertTrue("loops were closed", slam.getLoopClosureCount() > 0);
    assertTrue("worst error " + maxErr, maxErr < 0.15);
  }

  @Test
  public void mappingOnALoadedMapLinksNewSubmapsToIt() {
    SlamEngine slam = new SlamEngine(OccupancyGrid.indoorDefault());
    slam.setGrid(roomGrid());
    slam.setMapping(false);
    Random noise = new Random(9);
    slam.resetPose(new Pose(-1, 0, 0));
    slam.setMapping(true); // extend the saved map
    double x = -1, y = 0, th = 0;
    for (int step = 0; step < 120; step++) {
      double[][] s = scan(x, y, th, noise);
      slam.update(s[0], s[1], th, step * 0.1);
      th += step > 40 && step < 80 ? 0.03 : 0;
      x += 0.03 * Math.cos(th);
      y += 0.03 * Math.sin(th);
    }
    assertTrue("matched the loaded map", slam.getLoopClosureCount() > 0);
    Pose p = slam.getPose();
    assertEquals(x, p.x, 0.1);
    assertEquals(y, p.y, 0.1);
  }

  // region Algorithm choices

  @Test
  public void everySlamEngineTracksADriveRoundTheRoom() {
    for (Slam.Type t : Slam.Type.values()) {
      Slam slam = Slam.create(t, OccupancyGrid.indoorDefault());
      Random noise = new Random(1);
      double x = 0, y = 0, th = 0;
      for (int step = 0; step < 100; step++) {
        double[][] s = scan(x, y, th, noise);
        slam.update(s[0], s[1], th + step * 0.002, step * 0.1);
        th += step > 30 && step < 70 ? 0.03 : 0;
        x += 0.04 * Math.cos(th);
        y += 0.04 * Math.sin(th);
      }
      double[][] s = scan(x, y, th, noise);
      Pose p = slam.update(s[0], s[1], th + 0.2, 10.0);
      assertEquals(t + " x", x, p.x, 0.15);
      assertEquals(t + " y", y, p.y, 0.15);
      assertEquals(t + " heading", 0, Math.toDegrees(Pose.wrap(p.theta - th)), 5);
      assertTrue(t + " mapped", slam.getGrid().knownCells() > 1500);
      assertEquals(t, slam.type());
    }
  }

  @Test
  public void switchingEngineKeepsTheMapAndPose() {
    SlamSystem sys = new SlamSystem(Slam.Type.CARTOGRAPHER, OccupancyGrid.indoorDefault());
    Random noise = new Random(2);
    for (int i = 0; i < 10; i++) {
      double[][] s = scan(0.03 * i, 0, 0, noise);
      sys.update(s[0], s[1], 0, i * 0.1);
    }
    int known = sys.getGrid().knownCells();
    Pose before = sys.getPose();
    sys.setEngine(Slam.Type.HECTOR);
    assertEquals(Slam.Type.HECTOR, sys.type());
    assertTrue(sys.getGrid().knownCells() >= known * 0.9);
    assertEquals(before.x, sys.getPose().x, 1e-9);
    double[][] s = scan(0.33, 0, 0, noise);
    Pose p = sys.update(s[0], s[1], 0, 1.0);
    assertEquals(0.33, p.x, 0.08);
  }

  @Test
  public void amclTracksTheCarOnASavedMap() {
    SlamSystem sys = new SlamSystem(Slam.Type.CARTOGRAPHER, OccupancyGrid.indoorDefault());
    sys.setGrid(roomGrid());
    sys.setLocalizer(SlamSystem.Localizer.AMCL);
    sys.setMapping(false);
    sys.resetPose(new Pose(-1, 0, 0));
    Random noise = new Random(4);
    double x = -1, y = 0, th = 0;
    Pose p = null;
    for (int step = 0; step < 120; step++) {
      double[][] s = scan(x, y, th, noise);
      p = sys.update(s[0], s[1], th, step * 0.1);
      th += step > 40 && step < 70 ? 0.025 : 0;
      x += 0.03 * Math.cos(th);
      y += 0.03 * Math.sin(th);
    }
    assertEquals(x, p.x, 0.15);
    assertEquals(y, p.y, 0.15);
    assertEquals(SlamSystem.Localizer.AMCL, sys.getLocalizer());
  }

  @Test
  public void everyPlannerGoesAroundTheBox() {
    OccupancyGrid g = roomGrid();
    for (Navigator.Planner pl : Navigator.Planner.values()) {
      Navigator.Params p = new Navigator.Params();
      p.planner = pl;
      List<double[]> path = Navigator.plan(g, new Pose(-1, 1.0, 0), 3.2, 1.0, p);
      assertNotNull(pl.label, path);
      for (double[] q : path) {
        boolean inBox = q[0] > 1.35 && q[0] < 2.15 && q[1] > 0.35 && q[1] < 1.65;
        assertTrue(pl + " enters the box", !inBox);
      }
      double[] end = path.get(path.size() - 1);
      assertEquals(3.2, end[0], 1e-9);
    }
  }

  @Test
  public void hybridAStarPlansACarLikePathWithAReverse() {
    // Facing the right wall from close by, goal behind: a car needs to reverse or loop.
    OccupancyGrid g = roomGrid();
    Navigator.Params p = new Navigator.Params();
    p.planner = Navigator.Planner.HYBRID_ASTAR;
    List<double[]> path = Navigator.plan(g, new Pose(3.3, -0.8, 0), 2.6, -0.8, p);
    assertNotNull(path);
    double minR = p.wheelbase / Math.tan(Math.toRadians(p.maxSteerDeg));
    for (int i = 1; i + 1 < path.size() - 1; i++) {
      double[] a = path.get(i - 1), b = path.get(i), c = path.get(i + 1);
      if (a[2] != b[2] || b[2] != c[2]) continue; // cusp
      double h1 = Math.atan2(b[1] - a[1], b[0] - a[0]), h2 = Math.atan2(c[1] - b[1], c[0] - b[0]);
      double turn = Math.abs(Pose.wrap(h2 - h1)), len = Math.hypot(c[0] - b[0], c[1] - b[1]);
      assertTrue("tighter than the car can turn at " + i, turn <= len / minR * 1.3 + 1e-6);
    }
  }

  @Test
  public void everyPlannerAndControllerArrives() {
    OccupancyGrid g = roomGrid();
    for (Navigator.Planner pl : Navigator.Planner.values()) {
      for (Navigator.Controller co : Navigator.Controller.values()) {
        Navigator.Params p = new Navigator.Params();
        p.planner = pl;
        p.controller = co;
        Navigator nav = new Navigator(p);
        nav.setGoal(3.2, 1.0);
        double[] st = {-1, 0, 0};
        boolean arrived = false;
        for (int k = 0; k < 900 && !arrived; k++) {
          Navigator.Command c = nav.step(new Pose(st[0], st[1], st[2]), g, Double.POSITIVE_INFINITY, k * 0.1);
          arrived = c.state == Navigator.State.ARRIVED;
          st = drive(st, c.throttle, c.steer);
          assertTrue(pl + "/" + co + " left the room", st[0] > -1.85 && st[0] < 3.85 && st[1] > -1.35 && st[1] < 2.35);
        }
        assertTrue(pl + "/" + co + " did not arrive", arrived);
      }
    }
  }

  // endregion

  // endregion
}
