package org.openbot.app.robot.slam;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Goal navigation on the SLAM map, a small version of Nav2's pieces for this car:
 *
 * <ul>
 *   <li><b>Costmap</b> ({@link Costmap}): walls inflated by the car's radius plus a fading cost.
 *   <li><b>Global planner</b>, chosen by the user ({@link Planners}): NavFn (A*), Theta* or Smac
 *       Hybrid-A* (car-like arcs, forward and reverse).
 *   <li><b>Controller</b>, chosen by the user: Regulated Pure Pursuit (here; it follows reversing
 *       parts of a Hybrid-A* path too), DWB or MPPI ({@link Controllers}).
 *   <li><b>Recovery</b>: blocked in front for 2.5 s, back up for a second and re-plan (Nav2's
 *       BackUp behaviour; no Spin, a car cannot spin).
 * </ul>
 *
 * Output uses the same throttle / steer convention as the LiDAR robot (steer + = nose left). Pure
 * Java, unit tested. Same logic in Swift (SlamCore.swift).
 */
public final class Navigator {

  public enum State {
    IDLE,
    DRIVING,
    BLOCKED,
    BACKING_UP,
    ARRIVED,
    NO_PATH
  }

  public enum Planner {
    NAVFN("NavFn (A*)"),
    THETA_STAR("Theta*"),
    HYBRID_ASTAR("Smac Hybrid-A*");

    public final String label;

    Planner(String label) {
      this.label = label;
    }
  }

  public enum Controller {
    RPP("Regulated Pure Pursuit"),
    DWB("DWB"),
    MPPI("MPPI");

    public final String label;

    Controller(String label) {
      this.label = label;
    }
  }

  /** Car and controller settings. */
  public static final class Params {
    public Planner planner = Planner.NAVFN;
    public Controller controller = Controller.RPP;
    public double speed = 0.35; // fraction of the speed mode
    public double wheelbase = 0.26; // m
    public double maxSteerDeg = 28; // front-wheel lock
    public double lookahead = 0.55; // m
    public double robotRadius = 0.22; // m: inflated as lethal
    public double inflation = 0.45; // m: soft cost beyond the radius
    public double goalTolerance = 0.25; // m
    public double stopDistance = 0.35; // m in front: stop
  }

  public static final class Command {
    public double throttle;
    public double steer;
    public State state = State.IDLE;
    public String message = "";
  }

  private final Params params;
  private double[] goal; // x, y
  private List<double[]> path = new ArrayList<>();
  private State state = State.IDLE;
  private double lastPlan = -1e9;
  private double blockedSince = -1;
  private double backUntil = -1;
  private double backSteer;
  // Turning round when the path is behind: a multi-point turn to one side (chosen once).
  private int turnSide; // 0 = not turning, +1 nose left, -1 nose right
  private double turnPhaseStart;
  private boolean turnBacking;
  private Costmap costmap;
  private int progress; // path index reached (paths with reversing parts are followed in order)
  private final Controllers.Dwb dwb = new Controllers.Dwb();
  private final Controllers.Mppi mppi = new Controllers.Mppi();

  public Navigator(Params params) {
    this.params = params;
  }

  public Params getParams() {
    return params;
  }

  public void setGoal(double x, double y) {
    turnSide = 0;
    goal = new double[] {x, y};
    path = new ArrayList<>();
    lastPlan = -1e9;
    blockedSince = -1;
    backUntil = -1;
    progress = 0;
    mppi.reset();
    state = State.DRIVING;
  }

  public void cancel() {
    goal = null;
    path = new ArrayList<>();
    state = State.IDLE;
  }

  public double[] getGoal() {
    return goal;
  }

  public List<double[]> getPath() {
    return path;
  }

  public State getState() {
    return state;
  }

  /**
   * One control step (about 10 Hz).
   *
   * @param frontMin nearest LiDAR return within 25° of the nose, metres (infinite if none)
   */
  public Command step(Pose pose, OccupancyGrid grid, double frontMin, double now) {
    Command c = new Command();
    if (goal == null) {
      state = State.IDLE;
      c.state = state;
      return c;
    }
    double toGoal = pose.distanceTo(goal[0], goal[1]);
    if (toGoal < params.goalTolerance) {
      state = State.ARRIVED;
      goal = null;
      c.state = state;
      c.message = "Arrived at the goal";
      return c;
    }

    // Recovery: backing up.
    if (now < backUntil) {
      state = State.BACKING_UP;
      c.state = state;
      c.throttle = -params.speed * 0.7;
      c.steer = backSteer;
      c.message = "Blocked: backing up to try again";
      return c;
    }

    // Blocked in front: wait, then back up.
    if (frontMin < params.stopDistance) {
      if (blockedSince < 0) blockedSince = now;
      if (now - blockedSince > 2.5) {
        backUntil = now + 1.0;
        blockedSince = -1;
        lastPlan = -1e9; // re-plan after backing up
        double[] la = lookaheadPoint(pose);
        backSteer = la == null ? 0 : -Math.signum(angleTo(pose, la)); // swing the nose away
      }
      state = State.BLOCKED;
      c.state = state;
      c.message = String.format(Locale.US, "Obstacle %.2f m ahead: waiting", frontMin);
      return c;
    }
    blockedSince = -1;

    // Re-plan every 2 s (the map keeps growing; Hybrid-A* every 4 s, it costs more) or when there is no path.
    double replan = params.planner == Planner.HYBRID_ASTAR ? 4.0 : 2.0;
    if (path.isEmpty() || now - lastPlan > replan) {
      costmap = new Costmap(grid, params);
      List<double[]> p = Planners.plan(costmap, pose, goal[0], goal[1], params);
      lastPlan = now;
      progress = 0;
      if (p == null) {
        state = State.NO_PATH;
        path = new ArrayList<>();
        c.state = state;
        c.message = "No path to the goal on this map";
        return c;
      }
      path = p;
    }

    if (params.controller != Controller.RPP) {
      double[] u = params.controller == Controller.DWB
          ? dwb.step(pose, path, costmap, params)
          : mppi.step(pose, path, goal, costmap, params);
      state = State.DRIVING;
      c.state = state;
      c.throttle = u[0];
      c.steer = u[1];
      if (frontMin < params.stopDistance * 2.5 && c.throttle > 0) c.throttle *= 0.55;
      c.message = String.format(Locale.US, "%s: %.1f m to go", params.controller.label, toGoal);
      return c;
    }
    if (hasReverse(path)) return followWithReverse(pose, toGoal, frontMin, c);

    double[] la = lookaheadPoint(pose);
    if (la == null) la = goal;
    double alpha = angleTo(pose, la);
    double ld = Math.max(0.15, pose.distanceTo(la[0], la[1]));
    state = State.DRIVING;
    c.state = state;
    double maxSteer = Math.toRadians(params.maxSteerDeg);
    // Path behind: a multi-point turn (a car cannot turn on the spot). The side is chosen once, so
    // the turn does not flip left/right as the angle wobbles around 180°; back up and go forward in
    // turns, the nose swinging the same way both times, until the path is within 60° of the nose.
    if (turnSide == 0 && Math.abs(alpha) > Math.toRadians(100)) {
      turnSide = alpha >= 0 ? 1 : -1;
      turnBacking = true;
      turnPhaseStart = now;
    } else if (turnSide != 0 && Math.abs(alpha) < Math.toRadians(60)) {
      turnSide = 0;
    }
    if (turnSide != 0) {
      boolean roomAhead = frontMin > params.stopDistance * 1.6;
      if (now - turnPhaseStart > 1.6 || (!turnBacking && !roomAhead)) {
        turnBacking = !turnBacking || !roomAhead;
        turnPhaseStart = now;
      }
      c.throttle = (turnBacking ? -0.6 : 0.5) * params.speed;
      c.steer = turnSide;
      c.message = "Turning round towards the path";
      return c;
    }
    double delta = Math.atan2(2 * params.wheelbase * Math.sin(alpha), ld);
    double steer = clamp(delta / maxSteer, -1, 1);
    double v = params.speed * clamp(1 - 0.5 * Math.abs(steer), 0.4, 1); // regulate on curvature
    if (toGoal < 0.8) v *= Math.max(0.45, toGoal / 0.8); // slow for the goal
    if (frontMin < params.stopDistance * 2.5) v *= 0.55; // slow near obstacles
    c.throttle = v;
    c.steer = steer;
    c.message = String.format(Locale.US, "Driving to the goal: %.1f m to go", toGoal);
    return c;
  }

  private static boolean hasReverse(List<double[]> p) {
    for (double[] q : p) if (q.length > 2 && q[2] < 0) return true;
    return false;
  }

  private static int dirOf(double[] q) {
    return q.length > 2 && q[2] < 0 ? -1 : 1;
  }

  /**
   * Regulated Pure Pursuit with reversing (Nav2's allow_reversing): the path is followed in order,
   * one direction at a time. In reverse the car aims its rear at the lookahead point; the lookahead
   * never passes a cusp (where the direction changes), and the car moves on at the cusp.
   */
  private Command followWithReverse(Pose pose, double toGoal, double frontMin, Command c) {
    // Nearest point, searching forward from the progress made (never back across a cusp).
    int end = progress;
    while (end < path.size() - 1 && dirOf(path.get(end + 1)) == dirOf(path.get(progress))) end++;
    int near = progress;
    double bd = Double.MAX_VALUE;
    for (int i = progress; i <= end; i++) {
      double d = pose.distanceTo(path.get(i)[0], path.get(i)[1]);
      if (d < bd) {
        bd = d;
        near = i;
      }
    }
    progress = near;
    if (end < path.size() - 1 && pose.distanceTo(path.get(end)[0], path.get(end)[1]) < 0.15) {
      progress = end + 1; // reached the cusp: switch direction
      return followWithReverse(pose, toGoal, frontMin, c);
    }
    int dir = dirOf(path.get(progress));
    double[] la = path.get(end);
    for (int i = near; i <= end; i++) {
      if (pose.distanceTo(path.get(i)[0], path.get(i)[1]) >= params.lookahead) {
        la = path.get(i);
        break;
      }
    }
    // Angle to the point from the side the car drives towards (the rear when reversing).
    double heading = dir > 0 ? pose.theta : pose.theta + Math.PI;
    double alpha = Pose.wrap(Math.atan2(la[1] - pose.y, la[0] - pose.x) - heading);
    double ld = Math.max(0.15, pose.distanceTo(la[0], la[1]));
    double delta = Math.atan2(2 * params.wheelbase * Math.sin(alpha), ld);
    double steer = clamp(delta / Math.toRadians(params.maxSteerDeg), -1, 1);
    double v = params.speed * clamp(1 - 0.5 * Math.abs(steer), 0.4, 1);
    if (toGoal < 0.8) v *= Math.max(0.45, toGoal / 0.8);
    if (dir > 0 && frontMin < params.stopDistance * 2.5) v *= 0.55;
    if (dir < 0) v *= 0.7;
    state = State.DRIVING;
    c.state = state;
    c.throttle = dir * v;
    c.steer = steer; // + turns the nose left in both directions, which swings the rear's path left in reverse
    c.message = String.format(Locale.US, "%s to the goal: %.1f m to go", dir > 0 ? "Driving" : "Reversing", toGoal);
    return c;
  }

  /** First path point at least one lookahead away, searching from the point nearest the car. */
  private double[] lookaheadPoint(Pose pose) {
    if (path.isEmpty()) return null;
    int nearest = 0;
    double best = Double.MAX_VALUE;
    for (int i = 0; i < path.size(); i++) {
      double d = pose.distanceTo(path.get(i)[0], path.get(i)[1]);
      if (d < best) {
        best = d;
        nearest = i;
      }
    }
    for (int i = nearest; i < path.size(); i++) {
      if (pose.distanceTo(path.get(i)[0], path.get(i)[1]) >= params.lookahead) return path.get(i);
    }
    return path.get(path.size() - 1);
  }

  private static double angleTo(Pose pose, double[] p) {
    return Pose.wrap(Math.atan2(p[1] - pose.y, p[0] - pose.x) - pose.theta);
  }

  /** Plans with the chosen planner (start heading 0 for Hybrid-A*); null when unreachable. */
  public static List<double[]> plan(OccupancyGrid grid, double sx, double sy, double gx, double gy, Params p) {
    return Planners.plan(new Costmap(grid, p), new Pose(sx, sy, 0), gx, gy, p);
  }

  public static List<double[]> plan(OccupancyGrid grid, Pose start, double gx, double gy, Params p) {
    return Planners.plan(new Costmap(grid, p), start, gx, gy, p);
  }

  static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }
}
