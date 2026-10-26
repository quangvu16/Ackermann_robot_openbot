package org.openbot.app.robot.gpsNav;

/**
 * Geographic helpers for GPS navigation.
 *
 * <p>Pure Java so the math can be unit tested without an Android device. All angles are in degrees
 * and all distances in metres.
 */
public final class GeoMath {

  /** Mean earth radius used by the haversine formula. */
  private static final double EARTH_RADIUS_M = 6371008.8;

  private static final double DEG_TO_RAD = Math.PI / 180.0;
  private static final double RAD_TO_DEG = 180.0 / Math.PI;

  private GeoMath() {}

  /** Great-circle distance between two WGS-84 points, in metres. */
  public static double distanceMeters(double lat1, double lon1, double lat2, double lon2) {
    double dLat = (lat2 - lat1) * DEG_TO_RAD;
    double dLon = (lon2 - lon1) * DEG_TO_RAD;
    double a =
        Math.sin(dLat / 2) * Math.sin(dLat / 2)
            + Math.cos(lat1 * DEG_TO_RAD)
                * Math.cos(lat2 * DEG_TO_RAD)
                * Math.sin(dLon / 2)
                * Math.sin(dLon / 2);
    return 2 * EARTH_RADIUS_M * Math.asin(Math.min(1.0, Math.sqrt(a)));
  }

  /**
   * Initial great-circle bearing from the first point to the second.
   *
   * <p>Result is in degrees clockwise from true north, wrapped to {@code [-180, 180)}.
   */
  public static double bearingDegrees(double lat1, double lon1, double lat2, double lon2) {
    double phi1 = lat1 * DEG_TO_RAD;
    double phi2 = lat2 * DEG_TO_RAD;
    double dLon = (lon2 - lon1) * DEG_TO_RAD;
    double y = Math.sin(dLon) * Math.cos(phi2);
    double x = Math.cos(phi1) * Math.sin(phi2) - Math.sin(phi1) * Math.cos(phi2) * Math.cos(dLon);
    return wrapDegrees(Math.atan2(y, x) * RAD_TO_DEG);
  }

  /** Wraps an angle to the range {@code [-180, 180)}. */
  public static double wrapDegrees(double degrees) {
    return ((degrees + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
  }

  /**
   * Smallest signed turn from a heading onto a target bearing.
   *
   * <p>Positive means the target is to the right (clockwise), negative to the left.
   */
  public static double turnErrorDegrees(double headingDegrees, double targetBearingDegrees) {
    return wrapDegrees(targetBearingDegrees - headingDegrees);
  }

  /**
   * Moves a point along a great circle by the given distance and initial bearing.
   *
   * <p>Returned as {@code {latitude, longitude}} in degrees. Used for track replay and tests.
   */
  public static double[] destinationPoint(
      double latitude, double longitude, double bearingDegrees, double distanceMeters) {
    double angular = distanceMeters / EARTH_RADIUS_M;
    double theta = bearingDegrees * DEG_TO_RAD;
    double phi1 = latitude * DEG_TO_RAD;
    double lambda1 = longitude * DEG_TO_RAD;

    double sinPhi2 =
        Math.sin(phi1) * Math.cos(angular) + Math.cos(phi1) * Math.sin(angular) * Math.cos(theta);
    double phi2 = Math.asin(Math.max(-1.0, Math.min(1.0, sinPhi2)));
    double lambda2 =
        lambda1
            + Math.atan2(
                Math.sin(theta) * Math.sin(angular) * Math.cos(phi1),
                Math.cos(angular) - Math.sin(phi1) * sinPhi2);

    return new double[] {phi2 * RAD_TO_DEG, wrapDegrees(lambda2 * RAD_TO_DEG)};
  }
}
