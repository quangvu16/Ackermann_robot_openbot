package org.openbot.app.robot.gpsNav;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/**
 * Pure-JVM tests for the differential-drive "go to waypoint" law.
 *
 * <p>Targets are placed with {@link GeoMath#destinationPoint} from a robot sitting at (0, 0) so the
 * distance and bearing of each case are exact by construction.
 */
public class GuidanceControllerTest {

  private static final double ROBOT_LAT = 0.0;
  private static final double ROBOT_LON = 0.0;

  private final GuidanceController controller = new GuidanceController();

  private static Waypoint targetAt(double bearingDegrees, double distanceMeters) {
    double[] point = GeoMath.destinationPoint(ROBOT_LAT, ROBOT_LON, bearingDegrees, distanceMeters);
    return new Waypoint(point[0], point[1]);
  }

  @Test
  public void targetStraightAheadDrivesBothWheelsEqually() {
    Waypoint target = targetAt(0.0, 20.0);
    GuidanceController.Result result = controller.compute(ROBOT_LAT, ROBOT_LON, 0.0, target);

    assertFalse(result.isArrived());
    assertEquals(0.0, result.getTurnErrorDegrees(), 0.5);
    assertEquals(result.getLeft(), result.getRight(), 1e-4);
    assertTrue("should drive forward", result.getLeft() > 0.1f);
  }

  @Test
  public void targetToTheRightLeadsWithLeftWheel() {
    Waypoint target = targetAt(30.0, 20.0);
    GuidanceController.Result result = controller.compute(ROBOT_LAT, ROBOT_LON, 0.0, target);

    assertTrue("turn error should be positive", result.getTurnErrorDegrees() > 0);
    assertTrue("left wheel must lead to turn right", result.getLeft() > result.getRight());
  }

  @Test
  public void targetToTheLeftLeadsWithRightWheel() {
    Waypoint target = targetAt(-30.0, 20.0);
    GuidanceController.Result result = controller.compute(ROBOT_LAT, ROBOT_LON, 0.0, target);

    assertTrue("turn error should be negative", result.getTurnErrorDegrees() < 0);
    assertTrue("right wheel must lead to turn left", result.getRight() > result.getLeft());
  }

  @Test
  public void largeHeadingErrorPivotsInPlace() {
    Waypoint target = targetAt(90.0, 20.0);
    GuidanceController.Result result = controller.compute(ROBOT_LAT, ROBOT_LON, 0.0, target);

    // Hard right: left wheel forward, right wheel backward, so the robot spins on the spot.
    assertTrue("left wheel forward", result.getLeft() > 0f);
    assertTrue("right wheel backward", result.getRight() < 0f);
    assertEquals(0f, result.getLeft() + result.getRight(), 1e-3);
  }

  @Test
  public void largeNegativeHeadingErrorPivotsTheOtherWay() {
    Waypoint target = targetAt(-90.0, 20.0);
    GuidanceController.Result result = controller.compute(ROBOT_LAT, ROBOT_LON, 0.0, target);

    assertTrue("left wheel backward", result.getLeft() < 0f);
    assertTrue("right wheel forward", result.getRight() > 0f);
    assertEquals(0f, result.getLeft() + result.getRight(), 1e-3);
  }

  @Test
  public void defaultRadiusSuitsPhoneGps() {
    // 3.5 m out is inside the phone-GPS default of 4 m: arrived, not circling.
    assertTrue(controller.compute(ROBOT_LAT, ROBOT_LON, 0.0, targetAt(0.0, 3.5)).isArrived());
    assertFalse(controller.compute(ROBOT_LAT, ROBOT_LON, 0.0, targetAt(0.0, 4.5)).isArrived());
  }

  @Test
  public void insideAcceptanceRadiusArrivesAndStops() {
    Waypoint target = targetAt(0.0, 1.0); // default radius is 4 m
    GuidanceController.Result result = controller.compute(ROBOT_LAT, ROBOT_LON, 0.0, target);

    assertTrue(result.isArrived());
    assertEquals(0f, result.getLeft(), 0f);
    assertEquals(0f, result.getRight(), 0f);
  }

  @Test
  public void waypointAcceptanceRadiusOverridesDefault() {
    Waypoint loose = new Waypoint(0.0, 0.0, "", 20f);
    // The robot is already inside the generous 20 m radius.
    GuidanceController.Result result = controller.compute(ROBOT_LAT, ROBOT_LON, 0.0, loose);

    assertTrue(result.isArrived());
  }

  @Test
  public void slowsDownWhenCloseToTheWaypoint() {
    float far = controller.compute(ROBOT_LAT, ROBOT_LON, 0.0, targetAt(0.0, 20.0)).getLeft();
    float near = controller.compute(ROBOT_LAT, ROBOT_LON, 0.0, targetAt(0.0, 5.0)).getLeft();

    assertTrue("closer target must be approached more slowly", near < far);
  }

  @Test
  public void commandsStayInsideNormalisedRange() {
    double[] bearings = {0, 15, 45, 55, 70, 90, 135, 170, -170, -90, -45, -15};
    double[] distances = {1.5, 2.6, 3, 5, 8, 12, 30, 100};

    for (double bearing : bearings) {
      for (double distance : distances) {
        GuidanceController.Result result =
            controller.compute(ROBOT_LAT, ROBOT_LON, 0.0, targetAt(bearing, distance));
        assertTrue("left out of range", result.getLeft() >= -1f && result.getLeft() <= 1f);
        assertTrue("right out of range", result.getRight() >= -1f && result.getRight() <= 1f);
      }
    }
  }

  @Test
  public void reportsDistanceAndBearingToTarget() {
    Waypoint target = targetAt(45.0, 50.0);
    GuidanceController.Result result = controller.compute(ROBOT_LAT, ROBOT_LON, 0.0, target);

    assertEquals(50.0, result.getDistanceMeters(), 1.0);
    assertEquals(45.0, result.getBearingDegrees(), 0.5);
    assertEquals(45.0, result.getTurnErrorDegrees(), 0.5);
  }
}
