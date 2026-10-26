package org.openbot.app.robot.gpsNav;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** Pure-JVM tests for the geographic helpers used by GPS navigation. */
public class GeoMathTest {

  private static final double TOLERANCE_M = 1.0;
  private static final double TOLERANCE_DEG = 0.01;

  @Test
  public void distanceIsZeroForSamePoint() {
    assertEquals(0.0, GeoMath.distanceMeters(47.397742, 8.545594, 47.397742, 8.545594), 1e-6);
  }

  @Test
  public void oneDegreeOfLatitudeIsAbout111Km() {
    double distance = GeoMath.distanceMeters(0.0, 0.0, 1.0, 0.0);
    assertEquals(111194.9, distance, 50.0);
  }

  @Test
  public void distanceIsSymmetric() {
    double forward = GeoMath.distanceMeters(47.39, 8.54, 47.40, 8.55);
    double backward = GeoMath.distanceMeters(47.40, 8.55, 47.39, 8.54);
    assertEquals(forward, backward, 1e-6);
  }

  @Test
  public void cardinalBearings() {
    assertEquals(0.0, GeoMath.bearingDegrees(0.0, 0.0, 1.0, 0.0), TOLERANCE_DEG);
    assertEquals(90.0, GeoMath.bearingDegrees(0.0, 0.0, 0.0, 1.0), TOLERANCE_DEG);
    assertEquals(-90.0, GeoMath.bearingDegrees(0.0, 0.0, 0.0, -1.0), TOLERANCE_DEG);
    // Due south is antipodal in longitude terms; atan2 gives -180.
    assertEquals(-180.0, GeoMath.bearingDegrees(0.0, 0.0, -1.0, 0.0), TOLERANCE_DEG);
  }

  @Test
  public void wrapDegreesStaysInRange() {
    assertEquals(0.0, GeoMath.wrapDegrees(0.0), 1e-9);
    assertEquals(10.0, GeoMath.wrapDegrees(370.0), 1e-9);
    assertEquals(-10.0, GeoMath.wrapDegrees(-370.0), 1e-9);
    assertEquals(-180.0, GeoMath.wrapDegrees(180.0), 1e-9);
    assertEquals(170.0, GeoMath.wrapDegrees(170.0), 1e-9);

    for (double angle :
        new double[] {-720, -360, -181, -180, -90, -1, 0, 1, 90, 179, 180, 359, 720}) {
      double wrapped = GeoMath.wrapDegrees(angle);
      assertTrue("wrapDegrees(" + angle + ") = " + wrapped + " out of range", wrapped >= -180.0);
      assertTrue("wrapDegrees(" + angle + ") = " + wrapped + " out of range", wrapped < 180.0);
    }
  }

  @Test
  public void turnErrorIsPositiveWhenTargetIsToTheRight() {
    // Facing north (0°), target to the east (90°) → turn right by 90°.
    assertEquals(90.0, GeoMath.turnErrorDegrees(0.0, 90.0), TOLERANCE_DEG);
    // Facing north, target to the west → turn left.
    assertEquals(-90.0, GeoMath.turnErrorDegrees(0.0, -90.0), TOLERANCE_DEG);
  }

  @Test
  public void turnErrorPicksShortWayAround() {
    // Facing 170°, target at -170°: 20° to the right, not 340° to the left.
    assertEquals(20.0, GeoMath.turnErrorDegrees(170.0, -170.0), TOLERANCE_DEG);
    // Facing -170°, target at 170°: 20° to the left.
    assertEquals(-20.0, GeoMath.turnErrorDegrees(-170.0, 170.0), TOLERANCE_DEG);
  }

  @Test
  public void destinationRoundTripsWithBearingAndDistance() {
    double startLat = 47.397742;
    double startLon = 8.545594;
    double bearing = 37.0;
    double distance = 250.0;

    double[] end = GeoMath.destinationPoint(startLat, startLon, bearing, distance);
    assertEquals(distance, GeoMath.distanceMeters(startLat, startLon, end[0], end[1]), TOLERANCE_M);
    assertEquals(
        bearing, GeoMath.bearingDegrees(startLat, startLon, end[0], end[1]), TOLERANCE_DEG);
  }
}
