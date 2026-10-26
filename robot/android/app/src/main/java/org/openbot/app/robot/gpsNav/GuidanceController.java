package org.openbot.app.robot.gpsNav;

/**
 * Steers a differential-drive robot toward a GPS waypoint.
 *
 * <p>This is the ground-robot equivalent of Mission Planner's "go to waypoint" / guided mode: turn
 * proportionally onto the bearing of the target, bleed speed as the acceptance radius is
 * approached, and stop once inside it. When the heading error is large the robot pivots on the spot
 * rather than carving a wide arc.
 *
 * <p>The controller is a pure function of (position, heading, target) so it can be unit tested and
 * replayed from a logged track without touching the vehicle.
 */
public class GuidanceController {

  /** Convenience result of one guidance step. */
  public static final class Result {
    private final float left;
    private final float right;
    private final double distanceMeters;
    private final double bearingDegrees;
    private final double turnErrorDegrees;
    private final boolean arrived;

    private Result(
        float left,
        float right,
        double distanceMeters,
        double bearingDegrees,
        double turnErrorDegrees,
        boolean arrived) {
      this.left = left;
      this.right = right;
      this.distanceMeters = distanceMeters;
      this.bearingDegrees = bearingDegrees;
      this.turnErrorDegrees = turnErrorDegrees;
      this.arrived = arrived;
    }

    static Result of(
        float left,
        float right,
        double distanceMeters,
        double bearingDegrees,
        double turnErrorDegrees) {
      return new Result(left, right, distanceMeters, bearingDegrees, turnErrorDegrees, false);
    }

    static Result arrived(double distanceMeters, double bearingDegrees, double turnErrorDegrees) {
      return new Result(0f, 0f, distanceMeters, bearingDegrees, turnErrorDegrees, true);
    }

    /** Normalised left wheel command in {@code [-1, 1]}. */
    public float getLeft() {
      return left;
    }

    /** Normalised right wheel command in {@code [-1, 1]}. */
    public float getRight() {
      return right;
    }

    public double getDistanceMeters() {
      return distanceMeters;
    }

    public double getBearingDegrees() {
      return bearingDegrees;
    }

    /** Signed turn needed to point at the target; positive is to the right. */
    public double getTurnErrorDegrees() {
      return turnErrorDegrees;
    }

    public boolean isArrived() {
      return arrived;
    }
  }

  private final double acceptanceRadiusMeters;
  private final double slowdownRadiusMeters;
  private final double fullLockBearingErrorDegrees;
  private final double pivotBearingErrorDegrees;
  private final double cruiseThrottle;
  private final double minThrottle;

  /** Guidance with the default tuning for an OpenBot-class ground robot. */
  public GuidanceController() {
    this(Waypoint.DEFAULT_ACCEPTANCE_RADIUS_M, 8.0, 70.0, 55.0, 0.70, 0.28);
  }

  /**
   * @param acceptanceRadiusMeters distance at which a waypoint counts as reached
   * @param slowdownRadiusMeters distance at which the approach speed ramp begins
   * @param fullLockBearingErrorDegrees heading error that saturates the steering command
   * @param pivotBearingErrorDegrees heading error above which the robot pivots instead of arcing
   * @param cruiseThrottle throttle used far from the waypoint, in {@code [0, 1]}
   * @param minThrottle throttle floor so the motors keep turning inside the slowdown radius
   */
  public GuidanceController(
      double acceptanceRadiusMeters,
      double slowdownRadiusMeters,
      double fullLockBearingErrorDegrees,
      double pivotBearingErrorDegrees,
      double cruiseThrottle,
      double minThrottle) {
    this.acceptanceRadiusMeters = Math.max(0.5, acceptanceRadiusMeters);
    this.slowdownRadiusMeters = Math.max(this.acceptanceRadiusMeters, slowdownRadiusMeters);
    this.fullLockBearingErrorDegrees = Math.max(1.0, fullLockBearingErrorDegrees);
    this.pivotBearingErrorDegrees = Math.max(1.0, pivotBearingErrorDegrees);
    this.cruiseThrottle = clamp(cruiseThrottle, 0.05, 1.0);
    this.minThrottle = clamp(minThrottle, 0.0, this.cruiseThrottle);
  }

  /**
   * Computes the wheel commands that move the robot toward {@code target}.
   *
   * @param latitude current latitude in degrees
   * @param longitude current longitude in degrees
   * @param headingDegrees current heading in degrees clockwise from true north
   * @param target the waypoint being sought
   */
  public Result compute(double latitude, double longitude, double headingDegrees, Waypoint target) {
    double distanceMeters = target.distanceFrom(latitude, longitude);
    double bearingDegrees = target.bearingFrom(latitude, longitude);
    double turnErrorDegrees = GeoMath.turnErrorDegrees(headingDegrees, bearingDegrees);

    double radius = Math.max(acceptanceRadiusMeters, target.getAcceptanceRadiusMeters());
    if (distanceMeters <= radius) {
      return Result.arrived(distanceMeters, bearingDegrees, turnErrorDegrees);
    }

    // Positive turn error means the target lies to the right of the nose.
    double turn = clamp(turnErrorDegrees / fullLockBearingErrorDegrees, -1.0, 1.0);

    // Ramp up from minThrottle at the acceptance radius to cruiseThrottle at the slowdown radius.
    double ramp = clamp((distanceMeters - radius) / (slowdownRadiusMeters - radius), 0.0, 1.0);
    double throttle = minThrottle + (cruiseThrottle - minThrottle) * ramp;

    // Bleed a little more speed while the nose is well off the bearing so we do not overshoot.
    throttle *= 1.0 - 0.35 * Math.abs(turn);

    float left;
    float right;
    if (Math.abs(turnErrorDegrees) > pivotBearingErrorDegrees) {
      // Pivot on the spot: right turn means left wheel forward, right wheel backward.
      double pivot = Math.max(throttle, minThrottle);
      double sign = turnErrorDegrees >= 0 ? 1.0 : -1.0;
      left = (float) clamp(sign * pivot, -1.0, 1.0);
      right = (float) clamp(-sign * pivot, -1.0, 1.0);
    } else {
      double turnComponent = throttle * turn;
      left = (float) clamp(throttle + turnComponent, -1.0, 1.0);
      right = (float) clamp(throttle - turnComponent, -1.0, 1.0);
    }

    return Result.of(left, right, distanceMeters, bearingDegrees, turnErrorDegrees);
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}
