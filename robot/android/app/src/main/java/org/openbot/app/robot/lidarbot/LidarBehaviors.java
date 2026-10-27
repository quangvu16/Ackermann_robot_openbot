package org.openbot.app.robot.lidarbot;

import java.util.Locale;

/**
 * LiDAR behaviours for the Ackermann car, after the hoverboard robot's ROS 2 package
 * hoverboard_laser (github.com/quangvu16/hoverboard-robot, src/hoverboard_laser): laser_Avoidance,
 * laser_Tracker and laser_Warning, plus Patrol and Tracking built on the same helpers.
 *
 * <p>The hoverboard turns on the spot; this car cannot. So every "turn on the spot" becomes either a
 * forward turn or backing out with the wheels turned, and Tracking points the car by creeping
 * forward with the wheels turned rather than spinning.
 *
 * <p>Angles here follow the hoverboard code (ROS): degrees from the nose, positive to the LEFT,
 * wrapped to (-180, 180]. Output is a throttle in [-1, 1] and a steering in [-1, 1] with positive =
 * the NOSE turns left. That holds in reverse too: the firmware flips the servo when backing (see
 * update_vehicle()), so +1 while reversing still swings the nose left. {@link #toWheels} turns them
 * into the app's left/right pair, which the firmware mixes back (throttle = l + r, steering = l - r).
 *
 * <p>Pure Java, so it is unit tested on the JVM. The same code exists in Swift (LidarBehaviors.swift).
 */
public final class LidarBehaviors {

  public enum Mode {
    PATROL,
    FOLLOW,
    AVOID,
    TRACK,
    WARN
  }

  /** What the face on screen shows. */
  public enum Mood {
    IDLE, // stopped, waiting
    HAPPY, // all clear, driving
    LOVE, // following someone at the right distance
    FOCUSED, // patrolling / tracking a target
    SEARCHING, // nothing to follow or track
    WORRIED, // something close, steering away
    SCARED, // boxed in, or the warning alarm
    BLIND // no LiDAR data
  }

  /** Tunable parameters; defaults are the hoverboard_laser ones where they exist. */
  public static final class Params {
    /** Top speed as a fraction of the speed mode (0..1). */
    public double speed = 0.35;
    /** Distance to keep or react at, metres (ResponseDist). */
    public double responseDist = 0.55;
    /** Half-width of the side / search window, degrees (LaserAngle). */
    public double laserAngle = 45;
    /** Returns closer than this are the car itself, metres (min_range). */
    public double minRange = 0.15;

    public static Params defaults(Mode mode) {
      Params p = new Params();
      switch (mode) {
        case FOLLOW:
          p.responseDist = 0.55; // laser_Tracker
          p.laserAngle = 30;
          p.speed = 0.4;
          break;
        case TRACK:
          p.responseDist = 0.4;
          p.laserAngle = 90;
          p.speed = 0.25;
          break;
        case WARN:
          p.responseDist = 0.5; // laser_Warning
          p.laserAngle = 20;
          p.speed = 0;
          break;
        case PATROL:
          p.responseDist = 0.6;
          p.laserAngle = 45;
          p.speed = 0.35;
          break;
        case AVOID:
        default:
          p.responseDist = 0.55; // laser_Avoidance
          p.laserAngle = 45;
          p.speed = 0.35;
          break;
      }
      return p;
    }
  }

  /** One decision: what to drive and what to show. */
  public static final class Decision {
    public double throttle; // -1..1, + forward
    public double steer; // -1..1, + nose turns left (forward or reverse)
    public Mood mood = Mood.IDLE;
    /** Where the face looks, degrees, + left; 0 = straight ahead. */
    public double gazeDeg;
    /** Distance to the object that matters, metres; NaN when none. */
    public double targetDist = Double.NaN;
    /** True while Warning's alarm should sound. */
    public boolean alarm;
    /** Plain-English reason, for the status line. */
    public String reason = "";

    Decision set(double throttle, double steer, Mood mood, String reason) {
      this.throttle = clamp(throttle, -1, 1);
      this.steer = clamp(steer, -1, 1);
      this.mood = mood;
      this.reason = reason;
      return this;
    }
  }

  /** Valid returns: angle (deg, + left) and range (m). */
  public static final class Points {
    public final double[] angleDeg;
    public final double[] rangeM;
    /** True when only the car's 8 proximity sectors are known (coarse). */
    public final boolean coarse;

    public Points(double[] angleDeg, double[] rangeM, boolean coarse) {
      this.angleDeg = angleDeg;
      this.rangeM = rangeM;
      this.coarse = coarse;
    }

    public int size() {
      return angleDeg.length;
    }
  }

  /**
   * A LiDAR revolution as points. Bin {@code i} is {@code (i + 0.5) * 360 / bins} degrees
   * CLOCKWISE from the LiDAR's zero mark (the firmware's convention); 0 mm = no return.
   */
  public static Points fromScan(int[] rangesMm, double forwardOffsetDeg, double minRange) {
    int n = 0;
    for (int mm : rangesMm) if (mm > 0 && mm / 1000.0 >= minRange) n++;
    double[] a = new double[n];
    double[] r = new double[n];
    int k = 0;
    for (int i = 0; i < rangesMm.length; i++) {
      int mm = rangesMm[i];
      if (mm <= 0 || mm / 1000.0 < minRange) continue;
      double cw = (i + 0.5) * 360.0 / rangesMm.length - forwardOffsetDeg;
      a[k] = wrap(-cw);
      r[k] = mm / 1000.0;
      k++;
    }
    return new Points(a, r, false);
  }

  /**
   * The car's 8 proximity sectors (metres, clockwise from the nose, infinite = clear) as points at
   * the sector centres. Coarse, but enough to avoid and warn over Bluetooth.
   */
  public static Points fromSectors(double[] sectorsM) {
    int n = 0;
    for (double m : sectorsM) if (isFinitePositive(m)) n++;
    double[] a = new double[n];
    double[] r = new double[n];
    int k = 0;
    double step = 360.0 / sectorsM.length;
    for (int i = 0; i < sectorsM.length; i++) {
      if (!isFinitePositive(sectorsM[i])) continue;
      a[k] = wrap(-i * step);
      r[k] = sectorsM[i];
      k++;
    }
    return new Points(a, r, true);
  }

  // region State kept between ticks (hold times, patrol turn)

  private final Mode mode;
  private Params params;
  private Decision held; // a manoeuvre being held
  private double holdUntil;
  private int patrolTurns;
  private int patrolTurnStep; // 0 = not turning
  private int turnSide = 1; // +1 left, -1 right
  // Avoid: the side to escape to, chosen when first blocked and kept for a while, so backing out
  // and driving forward both swing the nose the same way (otherwise a car rocks in a corner).
  private int escapeSide;
  private double escapeUntil;

  public LidarBehaviors(Mode mode, Params params) {
    this.mode = mode;
    this.params = params;
  }

  public Mode getMode() {
    return mode;
  }

  public void setParams(Params params) {
    this.params = params;
  }

  public Params getParams() {
    return params;
  }

  /** Number of turn-arounds Patrol has made. */
  public int getPatrolTurns() {
    return patrolTurns;
  }

  // endregion

  /**
   * One step (call at about 10 Hz). {@code points} null = no LiDAR: the car stops. {@code nowSec}
   * is any monotonic clock in seconds.
   */
  public Decision step(Points points, double nowSec) {
    Decision d = new Decision();
    if (points == null) {
      held = null;
      patrolTurnStep = 0;
      return d.set(0, 0, Mood.BLIND, "No LiDAR data: stopped");
    }
    if (held != null && nowSec < holdUntil) {
      return copy(held);
    }
    held = null;
    switch (mode) {
      case AVOID:
        return avoid(points, nowSec, d);
      case FOLLOW:
        return follow(points, d);
      case TRACK:
        return track(points, d);
      case WARN:
        return warn(points, d);
      case PATROL:
      default:
        return patrol(points, nowSec, d);
    }
  }

  // region Avoidance (laser_Avoidance)

  private Decision avoid(Points p, double now, Decision d) {
    double near = params.responseDist * 1.5;
    int minPoints = p.coarse ? 0 : 5;
    double a = params.laserAngle;
    int front = 0;
    int left = 0;
    int right = 0;
    double frontMin = Double.POSITIVE_INFINITY;
    double nearestAng = 0;
    double nearest = Double.POSITIVE_INFINITY;
    for (int i = 0; i < p.size(); i++) {
      double ang = p.angleDeg[i];
      double r = p.rangeM[i];
      if (Math.abs(ang) <= a && r < nearest) {
        nearest = r;
        nearestAng = ang;
      }
      if (r >= near) continue;
      if (Math.abs(ang) <= 20) {
        front++;
        frontMin = Math.min(frontMin, r);
      } else if (ang > 20 && ang < a) left++;
      else if (ang < -20 && ang > -a) right++;
    }
    boolean f = front > minPoints;
    boolean l = left > minPoints;
    boolean r = right > minPoints;
    double v = params.speed;
    d.gazeDeg = nearest < near ? nearestAng : 0;
    d.targetDist = nearest;
    // Same decision table as the hoverboard's, with Ackermann turns: where the hoverboard spins on
    // the spot, this car backs out with the nose swinging the chosen way.
    if (f && (escapeSide == 0 || now > escapeUntil)) {
      escapeSide = left <= right ? 1 : -1; // towards the side with fewer close returns
    }
    if (f) escapeUntil = now + 4.0;
    String side = escapeSide > 0 ? "left" : "right";
    if (f && l && r) {
      d.set(-v * 0.6, escapeSide, Mood.SCARED, "Boxed in: backing out, nose turning " + side);
      return hold(d, now, 1.2);
    } else if (f && frontMin < params.responseDist) {
      d.set(-v * 0.6, escapeSide, Mood.WORRIED, "Too close ahead: backing out, nose turning " + side);
      return hold(d, now, 0.8);
    } else if (f && (r || l)) {
      d.set(v * 0.5, escapeSide, Mood.WORRIED,
          (r && !l ? "Front and right" : l && !r ? "Front and left" : "Front") + " blocked: turning " + side);
      return hold(d, now, 0.4);
    } else if (f) {
      d.set(v * 0.5, escapeSide, Mood.WORRIED, "Front blocked: turning " + side);
      return hold(d, now, 0.4);
    } else if (l && r) {
      return d.set(v * 0.5, 0, Mood.FOCUSED, "Narrow gap: slowly straight through");
    } else if (l) {
      return d.set(v, -0.6, Mood.FOCUSED, "Left blocked: veering right");
    } else if (r) {
      return d.set(v, +0.6, Mood.FOCUSED, "Right blocked: veering left");
    }
    return d.set(v, 0, Mood.HAPPY, "All clear: forward");
  }

  // endregion

  // region Follower (laser_Tracker)

  private Decision follow(Points p, Decision d) {
    double dist = params.responseDist;
    int best = -1;
    // Something straight ahead and near wins (priorityAngle 10); otherwise the nearest in the window.
    for (int i = 0; i < p.size(); i++) {
      if (Math.abs(p.angleDeg[i]) < 10 && p.rangeM[i] < dist + 0.5
          && (best < 0 || p.rangeM[i] < p.rangeM[best])) best = i;
    }
    if (best < 0) {
      for (int i = 0; i < p.size(); i++) {
        if (Math.abs(p.angleDeg[i]) < params.laserAngle && (best < 0 || p.rangeM[i] < p.rangeM[best])) {
          best = i;
        }
      }
    }
    if (best < 0 || p.rangeM[best] > 3.0) {
      return d.set(0, 0, Mood.SEARCHING, "Nothing to follow in front");
    }
    double r = p.rangeM[best];
    double ang = p.angleDeg[best];
    d.gazeDeg = ang;
    d.targetDist = r;
    double err = r - dist;
    if (Math.abs(err) < 0.1) err = 0; // dead band, as laser_Tracker
    double throttle = clamp(err * 2.0, -1, 1) * params.speed;
    double steer = clamp(ang / 45.0, -1, 1); // nose towards the target, also while backing off
    if (throttle == 0) {
      return d.set(0, steer, Mood.LOVE, String.format(Locale.US, "Holding %.2f m from the target", r));
    }
    return d.set(throttle, steer, throttle > 0 ? Mood.LOVE : Mood.WORRIED,
        String.format(Locale.US, throttle > 0 ? "Following: %.2f m ahead" : "Too close (%.2f m): backing off", r));
  }

  // endregion

  // region Tracking

  /**
   * Turn to face the nearest object without chasing it. The car cannot spin, so it creeps forward
   * with the wheels turned until the object is within 10 degrees of the nose, and never closer than
   * ResponseDist.
   */
  private Decision track(Points p, Decision d) {
    int best = nearestIn(p, params.laserAngle, 3.0);
    if (best < 0) return d.set(0, 0, Mood.SEARCHING, "Looking for something to track");
    double r = p.rangeM[best];
    double ang = p.angleDeg[best];
    d.gazeDeg = ang;
    d.targetDist = r;
    double steer = clamp(ang / 30.0, -1, 1);
    if (Math.abs(ang) <= 10) {
      return d.set(0, 0, Mood.FOCUSED, String.format(Locale.US, "Facing the target, %.2f m", r));
    }
    if (r <= params.responseDist) {
      return d.set(0, steer, Mood.WORRIED, "Target too close to turn towards: pointing the wheels only");
    }
    return d.set(params.speed, steer, Mood.FOCUSED,
        String.format(Locale.US, "Turning towards the target (%.0f° %s)", Math.abs(ang), ang > 0 ? "left" : "right"));
  }

  // endregion

  // region Warning (laser_Warning)

  /** Stands still; sounds the alarm when something comes within ResponseDist in front. */
  private Decision warn(Points p, Decision d) {
    int best = nearestIn(p, params.laserAngle, Double.POSITIVE_INFINITY);
    if (best < 0) return d.set(0, 0, Mood.HAPPY, "Watching: nothing in front");
    double r = p.rangeM[best];
    double ang = p.angleDeg[best];
    d.gazeDeg = ang;
    d.targetDist = r;
    if (r <= params.responseDist) {
      d.alarm = true;
      // Point the wheels at it (laser_Warning turns to face it; this car can only aim its wheels).
      return d.set(0, clamp(ang / 30.0, -1, 1), Mood.SCARED,
          String.format(Locale.US, "WARNING: object %.2f m away", r));
    }
    return d.set(0, 0, r <= params.responseDist * 2 ? Mood.WORRIED : Mood.FOCUSED,
        String.format(Locale.US, "Watching: nearest %.2f m", r));
  }

  // endregion

  // region Patrol

  /**
   * Drive down the middle between the walls; at a wall, turn round with a back-and-forward
   * manoeuvre (no spinning on an Ackermann car) towards the side with more room, then carry on.
   * Turns repeat until the way ahead is clear for twice ResponseDist.
   */
  private Decision patrol(Points p, double now, Decision d) {
    double front = minIn(p, -20, 20);
    double left = minIn(p, 60, 120);
    double right = minIn(p, -120, -60);
    double v = params.speed;
    d.targetDist = front;

    if (patrolTurnStep > 0) {
      if (front > params.responseDist * 2.5) {
        patrolTurnStep = 0;
        patrolTurns++;
      } else {
        // A multi-point turn: back up, then forward, the nose swinging to turnSide both times.
        boolean back = patrolTurnStep % 2 == 1;
        patrolTurnStep++;
        d.gazeDeg = turnSide * 60;
        if (back) {
          d.set(-v * 0.7, turnSide, Mood.FOCUSED, "Patrol: turning round (backing)");
        } else {
          d.set(v * 0.6, turnSide, Mood.FOCUSED, "Patrol: turning round (forward)");
        }
        return hold(d, now, 0.9);
      }
    }
    if (front < params.responseDist * 1.5) {
      turnSide = left >= right ? 1 : -1;
      patrolTurnStep = 1;
      d.set(0, 0, Mood.WORRIED, "Patrol: wall ahead, turning round " + (turnSide > 0 ? "left" : "right"));
      return hold(d, now, 0.3);
    }
    // Keep to the middle: steer towards the side with more room.
    double l = Math.min(left, 3.0);
    double r = Math.min(right, 3.0);
    double steer = clamp((l - r) * 0.6, -0.6, 0.6);
    d.gazeDeg = steer * 30;
    return d.set(v, steer, Mood.FOCUSED, String.format(Locale.US, "Patrolling: %.1f m clear ahead", Math.min(front, 9.9)));
  }

  // endregion

  // region helpers

  private Decision hold(Decision d, double now, double seconds) {
    held = copy(d);
    holdUntil = now + seconds;
    return d;
  }

  private static Decision copy(Decision s) {
    Decision d = new Decision();
    d.throttle = s.throttle;
    d.steer = s.steer;
    d.mood = s.mood;
    d.gazeDeg = s.gazeDeg;
    d.targetDist = s.targetDist;
    d.alarm = s.alarm;
    d.reason = s.reason;
    return d;
  }

  private static int nearestIn(Points p, double halfAngle, double maxRange) {
    int best = -1;
    for (int i = 0; i < p.size(); i++) {
      if (Math.abs(p.angleDeg[i]) <= halfAngle && p.rangeM[i] <= maxRange
          && (best < 0 || p.rangeM[i] < p.rangeM[best])) best = i;
    }
    return best;
  }

  static double minIn(Points p, double fromDeg, double toDeg) {
    double m = Double.POSITIVE_INFINITY;
    for (int i = 0; i < p.size(); i++) {
      if (p.angleDeg[i] >= fromDeg && p.angleDeg[i] <= toDeg) m = Math.min(m, p.rangeM[i]);
    }
    return m;
  }

  /** Throttle and steering (+ nose left) to the app's left/right pair: l + r = throttle, l - r = -steer. */
  public static float[] toWheels(double throttle, double steer) {
    // The firmware's steering is l - r with + meaning RIGHT (tank-style), so nose-left is r > l.
    double l = (throttle - steer) / 2.0;
    double r = (throttle + steer) / 2.0;
    return new float[] {(float) clamp(l, -1, 1), (float) clamp(r, -1, 1)};
  }

  static double wrap(double deg) {
    double a = ((deg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    return a == -180.0 ? 180.0 : a;
  }

  private static boolean isFinitePositive(double m) {
    return !Double.isNaN(m) && !Double.isInfinite(m) && m > 0;
  }

  static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  // endregion
}
