package org.openbot.app.robot.lidarbot;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;
import org.openbot.app.robot.lidarbot.LidarBehaviors.Decision;
import org.openbot.app.robot.lidarbot.LidarBehaviors.Mode;
import org.openbot.app.robot.lidarbot.LidarBehaviors.Mood;
import org.openbot.app.robot.lidarbot.LidarBehaviors.Points;

public class LidarBehaviorsTest {

  /** 360 bins, all far (5 m), with an object of {@code mm} spanning clockwise bins [from, to]. */
  private static int[] scanWith(int from, int to, int mm) {
    int[] r = new int[360];
    for (int i = 0; i < 360; i++) r[i] = 5000;
    for (int i = from; i <= to; i++) r[(i + 360) % 360] = mm;
    return r;
  }

  private static Points pts(int[] r) {
    return LidarBehaviors.fromScan(r, 0, 0.15);
  }

  private static Decision run(Mode m, int[] scan) {
    return new LidarBehaviors(m, LidarBehaviors.Params.defaults(m)).step(pts(scan), 0);
  }

  @Test
  public void scanAnglesArePositiveToTheLeft() {
    // Bin 90 is 90.5 degrees clockwise = to the RIGHT = -90.5 in the ROS convention.
    Points p = LidarBehaviors.fromScan(new int[] {0, 0, 0, 1000}, 0, 0.1); // 4 bins: bin 3 at 315 cw
    assertEquals(1, p.size());
    assertEquals(45.0 / 2 + 22.5, p.angleDeg[0], 1e-9); // 3.5 * 90 = 315 cw -> +45 left
    assertEquals(1.0, p.rangeM[0], 1e-9);
  }

  @Test
  public void wheelsMixBackToThrottleAndSteering() {
    float[] w = LidarBehaviors.toWheels(0.4, 1.0); // forward, nose left
    assertEquals(0.4, w[0] + w[1], 1e-6); // throttle = l + r
    assertTrue("nose left means right wheel ahead of left (firmware steering l - r < 0)", w[0] < w[1]);
  }

  @Test
  public void avoidanceDrivesWhenClearAndTurnsAwayFromBlockedSide() {
    Decision clear = run(Mode.AVOID, scanWith(0, -1, 5000));
    assertEquals(Mood.HAPPY, clear.mood);
    assertTrue(clear.throttle > 0);
    assertEquals(0, clear.steer, 1e-9);

    // Obstacle on the right (clockwise 25..40 deg) at 0.6 m: veer left.
    Decision right = run(Mode.AVOID, scanWith(25, 40, 600));
    assertTrue(right.steer > 0);
    assertTrue(right.throttle > 0);

    // Wall all round the front at 0.4 m: boxed in, back out.
    Decision boxed = run(Mode.AVOID, scanWith(-45, 45, 400));
    assertEquals(Mood.SCARED, boxed.mood);
    assertTrue(boxed.throttle < 0);
  }

  @Test
  public void followerHoldsTheDistance() {
    Decision far = run(Mode.FOLLOW, scanWith(-3, 3, 1200));
    assertTrue("drives towards a target 1.2 m away", far.throttle > 0);
    assertEquals(Mood.LOVE, far.mood);

    Decision near = run(Mode.FOLLOW, scanWith(-3, 3, 300));
    assertTrue("backs off when closer than 0.55 m", near.throttle < 0);

    Decision right = run(Mode.FOLLOW, scanWith(18, 22, 900)); // 20 deg to the right
    assertTrue("steers towards a target on the right", right.steer < 0);
    assertTrue(right.gazeDeg < 0);

    Decision none = run(Mode.FOLLOW, scanWith(0, -1, 5000));
    assertEquals(Mood.SEARCHING, none.mood);
    assertEquals(0, none.throttle, 1e-9);
  }

  @Test
  public void trackingTurnsTowardsAndStopsWhenFacing() {
    Decision left = run(Mode.TRACK, scanWith(-62, -58, 1500)); // 60 deg left
    assertTrue(left.steer > 0);
    assertTrue(left.throttle > 0);
    Decision facing = run(Mode.TRACK, scanWith(-2, 2, 1500));
    assertEquals(0, facing.throttle, 1e-9);
    assertEquals(Mood.FOCUSED, facing.mood);
  }

  @Test
  public void warningNeverDrivesAndAlarmsWhenClose() {
    Decision ok = run(Mode.WARN, scanWith(-2, 2, 2000));
    assertFalse(ok.alarm);
    Decision close = run(Mode.WARN, scanWith(-2, 2, 400));
    assertTrue(close.alarm);
    assertEquals(Mood.SCARED, close.mood);
    assertEquals(0, close.throttle, 1e-9);
  }

  @Test
  public void patrolTurnsRoundAtAWallThenCarriesOn() {
    LidarBehaviors b = new LidarBehaviors(Mode.PATROL, LidarBehaviors.Params.defaults(Mode.PATROL));
    Decision go = b.step(pts(scanWith(0, -1, 5000)), 0);
    assertTrue(go.throttle > 0);
    Decision wall = b.step(pts(scanWith(-25, 25, 700)), 1);
    assertEquals(0, wall.throttle, 1e-9);
    Decision back = b.step(pts(scanWith(-25, 25, 700)), 2);
    assertTrue("first turn step backs up", back.throttle < 0);
    Decision fwd = b.step(pts(scanWith(-25, 25, 700)), 3);
    assertTrue("then forward", fwd.throttle > 0);
    Decision clear = b.step(pts(scanWith(0, -1, 5000)), 4);
    assertTrue(clear.throttle > 0);
    assertEquals(1, b.getPatrolTurns());
  }

  @Test
  public void noLidarStops() {
    Decision d = new LidarBehaviors(Mode.AVOID, LidarBehaviors.Params.defaults(Mode.AVOID)).step(null, 0);
    assertEquals(Mood.BLIND, d.mood);
    assertEquals(0, d.throttle, 1e-9);
  }

  @Test
  public void sectorsWorkAsCoarsePoints() {
    double inf = Double.POSITIVE_INFINITY;
    Points p = LidarBehaviors.fromSectors(new double[] {0.4, inf, inf, inf, inf, inf, inf, inf});
    assertTrue(p.coarse);
    assertEquals(1, p.size());
    Decision d = new LidarBehaviors(Mode.WARN, LidarBehaviors.Params.defaults(Mode.WARN)).step(p, 0);
    assertTrue(d.alarm);
  }
}
