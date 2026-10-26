package org.openbot.app.robot.vehicle;

/**
 * ArduPilot Rover-style Simple Object Avoidance, applied to the app's tank-style drive commands.
 *
 * <p>The model follows ArduPilot's Rover behaviour rather than Copter's:
 *
 * <ul>
 *   <li><b>Rover always stops.</b> Copter's {@code AVOID_BEHAVE} (Stop vs Slide) has no Rover
 *       equivalent - there is no sliding sideways past an obstacle on a car.
 *   <li><b>Applies in every mode except manual.</b> ArduPilot applies this in the firmware's
 *       navigation stack so no flight mode escapes it. Here the equivalent chokepoint is {@code
 *       Vehicle.setControl()}, which every drive source (gamepad, phone, GPS mission, point-goal,
 *       Blockly, BotFunctions) passes through.
 *   <li><b>{@code AVOID_MARGIN}</b> is the standoff at which the vehicle stops. Ours is in metres,
 *       like the parameter.
 *   <li><b>{@code AVOID_BACKUP_SPD}</b> actively backs away to restore the margin. {@code 0}
 *       disables it, matching ArduPilot's default of 0.
 *   <li><b>"SAFETY FIRST!"</b> ArduPilot insists on a quick-disable switch ({@code RCx_OPTION =
 *       40}) before first use. The same switch lives in the app's FreeRoam screen.
 * </ul>
 *
 * <p>The eight sectors are the MAVLink {@code DISTANCE_SENSOR} octant layout that ArduPilot's
 * proximity library uses: 0 is straight ahead, then every 45 degrees clockwise. See {@link
 * Proximity}.
 *
 * <p>Two deliberate deviations from ArduPilot, both in the direction of stopping rather than
 * driving:
 *
 * <ul>
 *   <li>A <b>stale</b> sensor (messages arrived, then stopped) blocks forward motion. ArduPilot
 *       simply fails to avoid when it has no data; for a car with no other ranging that means
 *       driving blind, which is the one outcome the feature exists to prevent. Reverse stays
 *       available so the driver can back out. A sensor that has <b>never</b> reported is treated as
 *       "not fitted" and never blocks - otherwise a car without a LiDAR would be undrivable.
 *   <li>Blocking zeroes the throttle but <b>keeps the steering</b>, so the wheels stay pointed
 *       where the driver put them while the car is held. On an Ackermann car that is a stationary
 *       steer, which is exactly what you want against a barrier.
 * </ul>
 *
 * <p>This class is deliberately pure Java with no Android imports so the law can be unit tested on
 * the JVM without a device or Robolectric.
 */
public final class ObjectAvoidance {

  private ObjectAvoidance() {}

  /**
   * Number of proximity octants. Matches {@link Proximity#SECTORS} and the firmware's {@code
   * PROX_SECTORS}.
   */
  public static final int SECTORS = 8;

  /** Straight ahead. */
  public static final int FRONT = 0;
  /** 45 degrees to the right of the nose. */
  public static final int FRONT_RIGHT = 1;
  /** Directly to the right. */
  public static final int RIGHT = 2;
  /** 45 degrees behind on the right. */
  public static final int BACK_RIGHT = 3;
  /** Directly behind. */
  public static final int BACK = 4;
  /** 45 degrees behind on the left. */
  public static final int BACK_LEFT = 5;
  /** Directly to the left. */
  public static final int LEFT = 6;
  /** 45 degrees to the left of the nose. */
  public static final int FRONT_LEFT = 7;

  /**
   * The front arc checked against forward motion: front-left, front, front-right. A 135 degree
   * sweep, which is a finer-grained version of the forward quadrant ArduPilot checks, and covers
   * the corners an Ackermann car swings through as it steers.
   */
  private static final int[] FRONT_ARC = {FRONT_LEFT, FRONT, FRONT_RIGHT};

  /** The rear arc checked against reverse motion, the mirror of {@link #FRONT_ARC}. */
  private static final int[] REAR_ARC = {BACK_LEFT, BACK, BACK_RIGHT};

  /** How fresh the proximity data is. See the class comment for how each is treated. */
  public enum DataState {
    /** No proximity message has ever arrived: no LiDAR fitted, or it has never reported. */
    NEVER,
    /** Messages arrived but have stopped. Fail toward stopping. */
    STALE,
    /** A recent message arrived and can be acted on. */
    FRESH
  }

  /**
   * What the law decided. {@code left} and {@code right} are the commands to actually drive, in
   * {@code [-1, 1]} like {@link Control}.
   */
  public static final class Decision {
    public final float left;
    public final float right;

    /** True when an obstacle vetoed motion the driver or autopilot actually asked for. */
    public final boolean blocked;

    /** True when the car is actively reversing to restore the margin ({@code backupSpeed > 0}). */
    public final boolean backing;

    /** Sector index of the nearest obstacle in the checked arc, or -1 when the arc is clear. */
    public final int nearestSector;

    /** Distance to that obstacle in metres, or {@link Float#POSITIVE_INFINITY} when clear. */
    public final float nearestMeters;

    Decision(
        float left,
        float right,
        boolean blocked,
        boolean backing,
        int nearestSector,
        float nearestMeters) {
      this.left = left;
      this.right = right;
      this.blocked = blocked;
      this.backing = backing;
      this.nearestSector = nearestSector;
      this.nearestMeters = nearestMeters;
    }

    /** True when this decision is holding the car back, either stopped or backing away. */
    public boolean isHolding() {
      return blocked || backing;
    }
  }

  /**
   * Evaluate one control request against the current proximity picture.
   *
   * @param left requested left track, {@code [-1, 1]}
   * @param right requested right track, {@code [-1, 1]}
   * @param sectorMeters the eight octant distances in metres; anything {@code <= 0} or non-finite
   *     is treated as "no return" (clear), matching the firmware's 0-means-clear wire convention
   * @param state how fresh {@code sectorMeters} is
   * @param marginMeters standoff in metres, like ArduPilot's {@code AVOID_MARGIN}; {@code <= 0}
   *     disables the law
   * @param backupSpeed reverse command in {@code [0, 1]} used to back away, like {@code
   *     AVOID_BACKUP_SPD}; {@code 0} disables backing away
   * @param enabled the quick-disable switch; {@code false} always passes the request through
   *     untouched
   */
  public static Decision evaluate(
      float left,
      float right,
      float[] sectorMeters,
      DataState state,
      float marginMeters,
      float backupSpeed,
      boolean enabled) {

    left = clamp(left);
    right = clamp(right);

    if (!enabled || marginMeters <= 0f || state == DataState.NEVER) {
      return new Decision(left, right, false, false, -1, Float.POSITIVE_INFINITY);
    }

    if (state == DataState.STALE) {
      // The last picture we had is too old to trust. A frozen "clear road" is exactly the reading
      // that drives a car into a wall, so block the direction we can no longer see. Reverse stays
      // open - backing out of trouble is what the driver wants, and the firmware's own
      // stop_at_obstacle() still holds the nose off whatever is in front.
      if (throttle(left, right) > 0f) {
        return stopKeepSteering(left, right, -1, Float.POSITIVE_INFINITY);
      }
      return new Decision(left, right, false, false, -1, Float.POSITIVE_INFINITY);
    }

    float throttle = throttle(left, right);
    if (throttle > 0f) {
      Obstacle o = nearest(sectorMeters, FRONT_ARC);
      if (o.meters < marginMeters) {
        if (backupSpeed > 0f) {
          Obstacle rear = nearest(sectorMeters, REAR_ARC);
          if (rear.meters >= marginMeters) {
            // ArduPilot's AVOID_BACKUP_SPD: actively back away to restore the margin rather than
            // sitting against the barrier. Only when the way behind is clear - backing into a wall
            // to get away from a wall is not an improvement.
            float b = clamp(backupSpeed);
            return new Decision(-b, -b, true, true, o.sector, o.meters);
          }
        }
        return stopKeepSteering(left, right, o.sector, o.meters);
      }
    } else if (throttle < 0f) {
      Obstacle o = nearest(sectorMeters, REAR_ARC);
      if (o.meters < marginMeters) {
        return stopKeepSteering(left, right, o.sector, o.meters);
      }
    }

    return new Decision(left, right, false, false, -1, Float.POSITIVE_INFINITY);
  }

  /** Throttle of a tank-style pair: the same mix the firmware uses to drive the ESC. */
  private static float throttle(float left, float right) {
    return left + right;
  }

  /**
   * Zero the throttle but keep the steering angle the driver asked for. With {@code throttle = 0}
   * and {@code steering = left - right} preserved, the two tracks become {@code +/- steering/2} -
   * the wheels stay pointed, the car does not move.
   */
  private static Decision stopKeepSteering(
      float left, float right, int nearestSector, float nearestMeters) {
    float steering = left - right;
    float half = clamp(steering / 2f);
    return new Decision(half, -half, true, false, nearestSector, nearestMeters);
  }

  private static final class Obstacle {
    final int sector;
    final float meters;

    Obstacle(int sector, float meters) {
      this.sector = sector;
      this.meters = meters;
    }
  }

  /**
   * Nearest return across the given sectors. A sector with no return is infinite here, so a clear
   * arc never looks like an obstacle sitting at zero.
   */
  private static Obstacle nearest(float[] sectorMeters, int[] arc) {
    int bestSector = -1;
    float best = Float.POSITIVE_INFINITY;
    if (sectorMeters == null || sectorMeters.length < SECTORS) {
      return new Obstacle(-1, Float.POSITIVE_INFINITY);
    }
    for (int i = 0; i < arc.length; i++) {
      int sector = arc[i];
      if (sector < 0 || sector >= sectorMeters.length) {
        continue;
      }
      float d = sectorMeters[sector];
      if (!(d > 0f) || Float.isInfinite(d)) {
        continue; // no return, or already infinite: clear
      }
      if (Float.isNaN(d)) {
        continue;
      }
      if (d < best) {
        best = d;
        bestSector = sector;
      }
    }
    return new Obstacle(bestSector, best);
  }

  private static float clamp(float v) {
    if (Float.isNaN(v)) {
      return 0f;
    }
    return Math.max(-1f, Math.min(1f, v));
  }
}
