package org.openbot.app.robot.vehicle;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;
import org.openbot.app.robot.vehicle.ObjectAvoidance.DataState;
import org.openbot.app.robot.vehicle.ObjectAvoidance.Decision;

/**
 * Tests for the ArduPilot Rover-style Simple Object Avoidance law.
 *
 * <p>The behavioural claims under test come from ArduPilot's own documentation: Rover always stops
 * rather than sliding past, the margin is a standoff in metres, {@code AVOID_BACKUP_SPD} of 0 means
 * stop and stay stopped, and there has to be a quick-disable that passes commands through untouched.
 */
public class ObjectAvoidanceTest {

  private static final float MARGIN = 1.0f;

  /** Eight clear sectors: no return anywhere. */
  private static float[] clear() {
    return new float[] {
      Float.POSITIVE_INFINITY,
      Float.POSITIVE_INFINITY,
      Float.POSITIVE_INFINITY,
      Float.POSITIVE_INFINITY,
      Float.POSITIVE_INFINITY,
      Float.POSITIVE_INFINITY,
      Float.POSITIVE_INFINITY,
      Float.POSITIVE_INFINITY
    };
  }

  /** Eight clear sectors with one obstacle placed, in metres. 0 means no return on this array. */
  private static float[] withObstacle(int sector, float meters) {
    float[] s = clear();
    s[sector] = meters;
    return s;
  }

  // ---------------------------------------------------------------- pass-through

  @Test
  public void clearRoadAhead_passesCommandThrough() {
    Decision d = ObjectAvoidance.evaluate(0.6f, 0.6f, clear(), DataState.FRESH, MARGIN, 0f, true);
    assertEquals(0.6f, d.left, 1e-6);
    assertEquals(0.6f, d.right, 1e-6);
    assertFalse(d.blocked);
    assertFalse(d.backing);
    assertFalse(d.isHolding());
  }

  @Test
  public void disabledSwitch_passesThroughEvenWithObstacleAhead() {
    // The quick-disable has to be absolute: when the car is misreading the room, the driver needs
    // to be able to drive it out by hand.
    float[] s = withObstacle(ObjectAvoidance.FRONT, 0.3f);
    Decision d = ObjectAvoidance.evaluate(1.0f, 1.0f, s, DataState.FRESH, MARGIN, 0.5f, false);
    assertEquals(1.0f, d.left, 1e-6);
    assertEquals(1.0f, d.right, 1e-6);
    assertFalse(d.isHolding());
  }

  @Test
  public void nonPositiveMargin_disablesTheLaw() {
    float[] s = withObstacle(ObjectAvoidance.FRONT, 0.3f);
    Decision d = ObjectAvoidance.evaluate(1.0f, 1.0f, s, DataState.FRESH, 0f, 0f, true);
    assertEquals(1.0f, d.left, 1e-6);
    assertFalse(d.isHolding());
  }

  // ---------------------------------------------------------------- Rover always stops

  @Test
  public void obstacleAhead_stopsForwardMotion() {
    float[] s = withObstacle(ObjectAvoidance.FRONT, MARGIN - 0.1f);
    Decision d = ObjectAvoidance.evaluate(0.8f, 0.8f, s, DataState.FRESH, MARGIN, 0f, true);
    // Rover stops - it does not slide past, which a car cannot do anyway.
    assertTrue(d.blocked);
    assertEquals(0f, d.left + d.right, 1e-6);
  }

  @Test
  public void obstacleAhead_stopsEvenAtLowSpeed() {
    float[] s = withObstacle(ObjectAvoidance.FRONT, 0.4f);
    Decision d = ObjectAvoidance.evaluate(0.15f, 0.15f, s, DataState.FRESH, MARGIN, 0f, true);
    assertTrue(d.blocked);
    assertEquals(0f, d.left + d.right, 1e-6);
  }

  @Test
  public void obstacleInFrontCorners_stopsForwardMotion() {
    // The swept path of an Ackermann car includes the front corners, not just the nose.
    for (int corner : new int[] {ObjectAvoidance.FRONT_LEFT, ObjectAvoidance.FRONT_RIGHT}) {
      float[] s = withObstacle(corner, 0.5f);
      Decision d = ObjectAvoidance.evaluate(1.0f, 1.0f, s, DataState.FRESH, MARGIN, 0f, true);
      assertTrue("sector " + corner + " should block forward", d.blocked);
      assertEquals(0f, d.left + d.right, 1e-6);
    }
  }

  @Test
  public void obstacleAtExactlyMargin_doesNotBlock() {
    // AVOID_MARGIN is the standoff at which the vehicle stops: at exactly that distance the road
    // is still open.
    float[] s = withObstacle(ObjectAvoidance.FRONT, MARGIN);
    Decision d = ObjectAvoidance.evaluate(0.5f, 0.5f, s, DataState.FRESH, MARGIN, 0f, true);
    assertFalse(d.blocked);
    assertEquals(0.5f, d.left, 1e-6);
  }

  @Test
  public void obstacleJustInsideMargin_blocks() {
    float[] s = withObstacle(ObjectAvoidance.FRONT, MARGIN - 0.01f);
    Decision d = ObjectAvoidance.evaluate(0.5f, 0.5f, s, DataState.FRESH, MARGIN, 0f, true);
    assertTrue(d.blocked);
  }

  @Test
  public void obstacleBehind_doesNotBlockForwardMotion() {
    float[] s = withObstacle(ObjectAvoidance.BACK, 0.2f);
    Decision d = ObjectAvoidance.evaluate(0.7f, 0.7f, s, DataState.FRESH, MARGIN, 0f, true);
    assertFalse(d.blocked);
    assertEquals(0.7f, d.left, 1e-6);
  }

  @Test
  public void obstacleToTheSide_doesNotBlockForwardMotion() {
    float[] s = withObstacle(ObjectAvoidance.RIGHT, 0.2f);
    Decision d = ObjectAvoidance.evaluate(0.7f, 0.7f, s, DataState.FRESH, MARGIN, 0f, true);
    assertFalse(d.blocked);
  }

  // ---------------------------------------------------------------- reverse

  @Test
  public void obstacleBehind_stopsReverseMotion() {
    float[] s = withObstacle(ObjectAvoidance.BACK, 0.4f);
    Decision d = ObjectAvoidance.evaluate(-0.6f, -0.6f, s, DataState.FRESH, MARGIN, 0f, true);
    assertTrue(d.blocked);
    assertEquals(0f, d.left + d.right, 1e-6);
  }

  @Test
  public void obstacleBehind_stopsReverseFromRearCorners() {
    for (int corner : new int[] {ObjectAvoidance.BACK_LEFT, ObjectAvoidance.BACK_RIGHT}) {
      float[] s = withObstacle(corner, 0.5f);
      Decision d = ObjectAvoidance.evaluate(-1.0f, -1.0f, s, DataState.FRESH, MARGIN, 0f, true);
      assertTrue("sector " + corner + " should block reverse", d.blocked);
    }
  }

  @Test
  public void obstacleAhead_doesNotBlockReverse() {
    // Backing away from a wall is exactly what the driver wants, and ArduPilot's AVOID_BACKUP_SPD
    // exists to do it deliberately.
    float[] s = withObstacle(ObjectAvoidance.FRONT, 0.3f);
    Decision d = ObjectAvoidance.evaluate(-0.6f, -0.6f, s, DataState.FRESH, MARGIN, 0f, true);
    assertFalse(d.blocked);
    assertEquals(-0.6f, d.left, 1e-6);
  }

  // ---------------------------------------------------------------- steering preserved

  @Test
  public void blocking_keepsSteeringAngle() {
    // A car held against a barrier should keep its wheels pointed where the driver put them.
    // Steering is left - right, throttle is left + right.
    float[] s = withObstacle(ObjectAvoidance.FRONT, 0.4f);
    Decision d = ObjectAvoidance.evaluate(0.9f, 0.3f, s, DataState.FRESH, MARGIN, 0f, true);
    assertTrue(d.blocked);
    assertEquals(0f, d.left + d.right, 1e-6);
    assertEquals(0.6f, d.left - d.right, 1e-5);
  }

  // ---------------------------------------------------------------- AVOID_BACKUP_SPD

  @Test
  public void backupSpeedZero_stopsAndStaysStopped() {
    // ArduPilot's AVOID_BACKUP_SPD default of 0: stop and stay stopped, no active backing away.
    float[] s = withObstacle(ObjectAvoidance.FRONT, 0.4f);
    Decision d = ObjectAvoidance.evaluate(1.0f, 1.0f, s, DataState.FRESH, MARGIN, 0f, true);
    assertTrue(d.blocked);
    assertFalse(d.backing);
    assertEquals(0f, d.left + d.right, 1e-6);
  }

  @Test
  public void backupSpeedPositive_backsAwayFromForwardObstacle() {
    float[] s = withObstacle(ObjectAvoidance.FRONT, 0.4f);
    Decision d = ObjectAvoidance.evaluate(1.0f, 1.0f, s, DataState.FRESH, MARGIN, 0.3f, true);
    assertTrue(d.backing);
    assertTrue(d.isHolding());
    assertEquals(-0.3f, d.left, 1e-6);
    assertEquals(-0.3f, d.right, 1e-6);
  }

  @Test
  public void backupSpeedPositive_doesNotBackWhenRearIsBlocked() {
    // Backing into a wall to get away from a wall is not an improvement.
    float[] s = withObstacle(ObjectAvoidance.FRONT, 0.4f);
    s[ObjectAvoidance.BACK] = 0.5f;
    Decision d = ObjectAvoidance.evaluate(1.0f, 1.0f, s, DataState.FRESH, MARGIN, 0.3f, true);
    assertFalse(d.backing);
    assertTrue(d.blocked);
    assertEquals(0f, d.left + d.right, 1e-6);
  }

  @Test
  public void backupSpeedPositive_doesNotOverrideReverseRequest() {
    // The driver is already reversing; do not fight them by inserting an avoidance reverse.
    float[] s = withObstacle(ObjectAvoidance.FRONT, 0.4f);
    Decision d = ObjectAvoidance.evaluate(-0.5f, -0.5f, s, DataState.FRESH, MARGIN, 0.3f, true);
    assertFalse(d.backing);
    assertFalse(d.blocked);
    assertEquals(-0.5f, d.left, 1e-6);
  }

  @Test
  public void backupSpeedPositive_doesNotOverrideStopRequest() {
    float[] s = withObstacle(ObjectAvoidance.FRONT, 0.4f);
    Decision d = ObjectAvoidance.evaluate(0f, 0f, s, DataState.FRESH, MARGIN, 0.3f, true);
    assertFalse(d.backing);
    assertFalse(d.blocked);
    assertEquals(0f, d.left, 1e-6);
  }

  @Test
  public void backupSpeedIsClampedToUnitRange() {
    float[] s = withObstacle(ObjectAvoidance.FRONT, 0.4f);
    Decision d = ObjectAvoidance.evaluate(1.0f, 1.0f, s, DataState.FRESH, MARGIN, 2.5f, true);
    assertEquals(-1.0f, d.left, 1e-6);
    assertEquals(-1.0f, d.right, 1e-6);
  }

  // ---------------------------------------------------------------- sensor freshness

  @Test
  public void neverReported_passesThroughEvenWithObstacleAhead() {
    // A car with no LiDAR must stay drivable. Treating "no sensor" as "sensor says stop" would
    // brick every vehicle that never had one.
    float[] s = withObstacle(ObjectAvoidance.FRONT, 0.3f);
    Decision d = ObjectAvoidance.evaluate(1.0f, 1.0f, s, DataState.NEVER, MARGIN, 0f, true);
    assertFalse(d.isHolding());
    assertEquals(1.0f, d.left, 1e-6);
  }

  @Test
  public void staleSensor_blocksForwardMotion() {
    // The last picture is too old to trust. A frozen "clear road" is exactly the reading that
    // drives a car into a wall, so hold rather than drive blind.
    float[] s = clear();
    Decision d = ObjectAvoidance.evaluate(1.0f, 1.0f, s, DataState.STALE, MARGIN, 0f, true);
    assertTrue(d.blocked);
    assertEquals(0f, d.left + d.right, 1e-6);
  }

  @Test
  public void staleSensor_stillAllowsReverse() {
    // Backing out of trouble when the forward sensor has died is what the driver wants.
    float[] s = clear();
    Decision d = ObjectAvoidance.evaluate(-0.7f, -0.7f, s, DataState.STALE, MARGIN, 0f, true);
    assertFalse(d.isHolding());
    assertEquals(-0.7f, d.left, 1e-6);
  }

  // ---------------------------------------------------------------- sector semantics

  @Test
  public void zeroDistanceSector_isClearNotAnObstacleAtZero() {
    // The firmware's wire convention: 0 means "nothing measured here". Reading it as an obstacle
    // at zero would stop the car permanently in open space.
    float[] s = new float[] {0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f};
    Decision d = ObjectAvoidance.evaluate(1.0f, 1.0f, s, DataState.FRESH, MARGIN, 0f, true);
    assertFalse(d.isHolding());
    assertEquals(1.0f, d.left, 1e-6);
  }

  @Test
  public void nullSectorArray_passesThrough() {
    Decision d = ObjectAvoidance.evaluate(0.5f, 0.5f, null, DataState.FRESH, MARGIN, 0f, true);
    assertFalse(d.isHolding());
    assertEquals(0.5f, d.left, 1e-6);
  }

  @Test
  public void shortSectorArray_passesThrough() {
    Decision d =
        ObjectAvoidance.evaluate(0.5f, 0.5f, new float[] {1f, 2f}, DataState.FRESH, MARGIN, 0f, true);
    assertFalse(d.isHolding());
  }

  // ---------------------------------------------------------------- nearest reporting

  @Test
  public void reportsNearestObstacleInCheckedArc() {
    float[] s = withObstacle(ObjectAvoidance.FRONT, 2.0f);
    s[ObjectAvoidance.FRONT_RIGHT] = 0.45f;
    s[ObjectAvoidance.FRONT_LEFT] = 0.9f;
    Decision d = ObjectAvoidance.evaluate(1.0f, 1.0f, s, DataState.FRESH, MARGIN, 0f, true);
    assertTrue(d.blocked);
    assertEquals(ObjectAvoidance.FRONT_RIGHT, d.nearestSector);
    assertEquals(0.45f, d.nearestMeters, 1e-6);
  }

  @Test
  public void clearArc_reportsNoNearestObstacle() {
    Decision d = ObjectAvoidance.evaluate(1.0f, 1.0f, clear(), DataState.FRESH, MARGIN, 0f, true);
    assertFalse(d.blocked);
    assertEquals(-1, d.nearestSector);
    assertEquals(Float.POSITIVE_INFINITY, d.nearestMeters, 0);
  }

  // ---------------------------------------------------------------- input hygiene

  @Test
  public void outOfRangeCommandsAreClamped() {
    Decision d = ObjectAvoidance.evaluate(3.0f, -2.0f, clear(), DataState.FRESH, MARGIN, 0f, true);
    assertEquals(1.0f, d.left, 1e-6);
    assertEquals(-1.0f, d.right, 1e-6);
  }

  @Test
  public void nanCommandsBecomeZero() {
    Decision d =
        ObjectAvoidance.evaluate(Float.NaN, 0.5f, clear(), DataState.FRESH, MARGIN, 0f, true);
    assertEquals(0f, d.left, 1e-6);
    assertEquals(0.5f, d.right, 1e-6);
  }

  // ---------------------------------------------------------------- constants

  @Test
  public void sectorsMatchTheMavlinkOctantLayout() {
    // Sector 0 is straight ahead, numbering clockwise every 45 degrees. This is the MAVLink
    // DISTANCE_SENSOR orientation layout ArduPilot's proximity library uses, and the same one the
    // firmware's PROX_SECTORS folding produces. The indices are the contract between the two.
    assertEquals(8, ObjectAvoidance.SECTORS);
    assertEquals(0, ObjectAvoidance.FRONT);
    assertEquals(1, ObjectAvoidance.FRONT_RIGHT);
    assertEquals(2, ObjectAvoidance.RIGHT);
    assertEquals(3, ObjectAvoidance.BACK_RIGHT);
    assertEquals(4, ObjectAvoidance.BACK);
    assertEquals(5, ObjectAvoidance.BACK_LEFT);
    assertEquals(6, ObjectAvoidance.LEFT);
    assertEquals(7, ObjectAvoidance.FRONT_LEFT);
    assertEquals(ObjectAvoidance.SECTORS, Proximity.SECTORS);
  }
}
