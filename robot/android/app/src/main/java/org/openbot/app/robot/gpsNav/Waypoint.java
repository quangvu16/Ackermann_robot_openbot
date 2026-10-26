package org.openbot.app.robot.gpsNav;

/**
 * A single GPS waypoint, the ground-robot equivalent of a Mission Planner waypoint list entry.
 *
 * <p>Waypoints only carry the horizontal target plus an acceptance radius; unlike an aircraft
 * mission there is no altitude leg. Immutable so a mission can hand them to the guidance loop
 * without defensive copies.
 */
public final class Waypoint {

  private final double latitude;
  private final double longitude;
  private final String name;
  private final float acceptanceRadiusMeters;

  /**
   * Default acceptance radius. The phone's own GPS is good to about 3-5 m under open sky, so a
   * tighter radius than that leaves the car circling a waypoint it can never quite "reach". (The
   * Linux robot, which reads an external receiver that may be RTK, keeps 2.5 m.)
   */
  public static final float DEFAULT_ACCEPTANCE_RADIUS_M = 4.0f;

  public Waypoint(double latitude, double longitude) {
    this(latitude, longitude, "", DEFAULT_ACCEPTANCE_RADIUS_M);
  }

  public Waypoint(double latitude, double longitude, String name, float acceptanceRadiusMeters) {
    this.latitude = latitude;
    this.longitude = longitude;
    this.name = name == null ? "" : name;
    this.acceptanceRadiusMeters = Math.max(0.5f, acceptanceRadiusMeters);
  }

  public double getLatitude() {
    return latitude;
  }

  public double getLongitude() {
    return longitude;
  }

  public String getName() {
    return name;
  }

  public float getAcceptanceRadiusMeters() {
    return acceptanceRadiusMeters;
  }

  /** Distance in metres from the given position to this waypoint. */
  public double distanceFrom(double latitude, double longitude) {
    return GeoMath.distanceMeters(latitude, longitude, this.latitude, this.longitude);
  }

  /** Initial bearing in degrees from the given position to this waypoint. */
  public double bearingFrom(double latitude, double longitude) {
    return GeoMath.bearingDegrees(latitude, longitude, this.latitude, this.longitude);
  }

  @Override
  public String toString() {
    return String.format(
        java.util.Locale.US,
        "Waypoint{%s lat=%.7f lon=%.7f r=%.1fm}",
        name.isEmpty() ? "-" : name,
        latitude,
        longitude,
        acceptanceRadiusMeters);
  }
}
