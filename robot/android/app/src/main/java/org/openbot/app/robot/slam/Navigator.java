package org.openbot.app.robot.slam;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.PriorityQueue;

/**
 * Goal navigation on the SLAM map, a small version of Nav2's pieces for this car:
 *
 * <ul>
 *   <li><b>Costmap</b>: walls inflated by the car's radius (lethal) plus a cost that fades further
 *       out, so paths keep off walls (Nav2's inflation layer). Unknown space is allowed but costs
 *       more (allow_unknown).
 *   <li><b>Global planner</b>: A* on a 10 cm grid, then shortcuts where the straight line is clear
 *       (like NavFn / Smac 2D with smoothing).
 *   <li><b>Controller</b>: Regulated Pure Pursuit for an Ackermann car: steer = atan(2 L sin(a) /
 *       lookahead), slower on sharp turns, near obstacles and near the goal.
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

  /** Car and controller settings. */
  public static final class Params {
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

    // Re-plan every 2 s (the map keeps growing) or when there is no path.
    if (path.isEmpty() || now - lastPlan > 2.0) {
      List<double[]> p = plan(grid, pose.x, pose.y, goal[0], goal[1], params);
      lastPlan = now;
      if (p == null) {
        state = State.NO_PATH;
        path = new ArrayList<>();
        c.state = state;
        c.message = "No path to the goal on this map";
        return c;
      }
      path = p;
    }

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

  // region Planning: inflated costmap + A*

  /** Costs on a coarse grid: -1 lethal, otherwise 1 + extra. */
  static final class Costmap {
    final int w;
    final int h;
    final int factor;
    final double res;
    final OccupancyGrid grid;
    final float[] cost;

    Costmap(OccupancyGrid grid, Params p) {
      this.grid = grid;
      factor = Math.max(1, (int) Math.round(0.10 / grid.resolution));
      res = grid.resolution * factor;
      w = grid.width / factor;
      h = grid.height / factor;
      cost = new float[w * h];
      boolean[] occ = new boolean[w * h];
      boolean[] unknown = new boolean[w * h];
      for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
          boolean o = false;
          boolean known = false;
          for (int dy = 0; dy < factor && !o; dy++) {
            for (int dx = 0; dx < factor; dx++) {
              float v = grid.get(x * factor + dx, y * factor + dy);
              if (v > OccupancyGrid.OCC_THRESHOLD) {
                o = true;
                break;
              }
              if (v < OccupancyGrid.FREE_THRESHOLD) known = true;
            }
          }
          occ[y * w + x] = o;
          unknown[y * w + x] = !o && !known;
        }
      }
      int rLethal = (int) Math.ceil(p.robotRadius / res);
      int rSoft = (int) Math.ceil((p.robotRadius + p.inflation) / res);
      for (int i = 0; i < cost.length; i++) cost[i] = unknown[i] ? 3f : 1f;
      for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
          if (!occ[y * w + x]) continue;
          for (int dy = -rSoft; dy <= rSoft; dy++) {
            for (int dx = -rSoft; dx <= rSoft; dx++) {
              int nx = x + dx;
              int ny = y + dy;
              if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
              double d = Math.hypot(dx, dy);
              int i = ny * w + nx;
              if (d <= rLethal) cost[i] = -1;
              else if (d <= rSoft && cost[i] >= 0) {
                float extra = (float) (8 * (1 - (d - rLethal) / (rSoft - rLethal + 1e-9)));
                cost[i] = Math.max(cost[i], 1 + extra);
              }
            }
          }
        }
      }
    }

    int cx(double x) {
      return (int) Math.floor((x - grid.originX) / res);
    }

    int cy(double y) {
      return (int) Math.floor((y - grid.originY) / res);
    }

    boolean free(int x, int y) {
      return x >= 0 && y >= 0 && x < w && y < h && cost[y * w + x] >= 0;
    }

    double wx(int x) {
      return grid.originX + (x + 0.5) * res;
    }

    double wy(int y) {
      return grid.originY + (y + 0.5) * res;
    }
  }

  /** A* from start to goal; null when the goal cannot be reached. */
  public static List<double[]> plan(OccupancyGrid grid, double sx, double sy, double gx, double gy, Params p) {
    Costmap m = new Costmap(grid, p);
    int x0 = m.cx(sx);
    int y0 = m.cy(sy);
    int x1 = m.cx(gx);
    int y1 = m.cy(gy);
    if (!m.free(x1, y1)) return null;
    // The car itself may sit in an inflated cell next to a wall: let it start from there.
    if (x0 < 0 || y0 < 0 || x0 >= m.w || y0 >= m.h) return null;
    int n = m.w * m.h;
    float[] g = new float[n];
    java.util.Arrays.fill(g, Float.MAX_VALUE);
    int[] from = new int[n];
    java.util.Arrays.fill(from, -1);
    boolean[] closed = new boolean[n];
    int start = y0 * m.w + x0;
    int target = y1 * m.w + x1;
    g[start] = 0;
    PriorityQueue<double[]> open = new PriorityQueue<>((a, b) -> Double.compare(a[0], b[0]));
    open.add(new double[] {octile(x0, y0, x1, y1), start});
    int[] dxs = {1, -1, 0, 0, 1, 1, -1, -1};
    int[] dys = {0, 0, 1, -1, 1, -1, 1, -1};
    int expanded = 0;
    while (!open.isEmpty() && expanded < 400_000) {
      int cur = (int) open.poll()[1];
      if (closed[cur]) continue;
      closed[cur] = true;
      expanded++;
      if (cur == target) break;
      int cx = cur % m.w;
      int cy = cur / m.w;
      for (int k = 0; k < 8; k++) {
        int nx = cx + dxs[k];
        int ny = cy + dys[k];
        if (!m.free(nx, ny)) continue;
        int ni = ny * m.w + nx;
        if (closed[ni]) continue;
        double step = (k < 4 ? 1.0 : Math.sqrt(2)) * m.cost[ni];
        float ng = (float) (g[cur] + step);
        if (ng < g[ni]) {
          g[ni] = ng;
          from[ni] = cur;
          open.add(new double[] {ng + octile(nx, ny, x1, y1), ni});
        }
      }
    }
    if (from[target] < 0 && target != start) return null;
    List<int[]> cells = new ArrayList<>();
    for (int c = target; c != -1; c = from[c]) {
      cells.add(0, new int[] {c % m.w, c / m.w});
      if (c == start) break;
    }
    // Shortcut: keep a point only when the straight line to the next kept point is not clear.
    List<double[]> out = new ArrayList<>();
    int i = 0;
    out.add(new double[] {sx, sy});
    while (i < cells.size() - 1) {
      int j = cells.size() - 1;
      while (j > i + 1 && !lineFree(m, cells.get(i), cells.get(j))) j--;
      out.add(new double[] {m.wx(cells.get(j)[0]), m.wy(cells.get(j)[1])});
      i = j;
    }
    out.set(out.size() - 1, new double[] {gx, gy});
    return densify(out, 0.1);
  }

  private static double octile(int x0, int y0, int x1, int y1) {
    int dx = Math.abs(x1 - x0);
    int dy = Math.abs(y1 - y0);
    return Math.max(dx, dy) + (Math.sqrt(2) - 1) * Math.min(dx, dy);
  }

  /** Clear and cheap all the way (soft cost keeps shortcuts from hugging walls). */
  private static boolean lineFree(Costmap m, int[] a, int[] b) {
    int steps = Math.max(Math.abs(b[0] - a[0]), Math.abs(b[1] - a[1]));
    for (int s = 0; s <= steps; s++) {
      double t = steps == 0 ? 0 : (double) s / steps;
      int x = (int) Math.round(a[0] + (b[0] - a[0]) * t);
      int y = (int) Math.round(a[1] + (b[1] - a[1]) * t);
      if (!m.free(x, y) || m.cost[y * m.w + x] > 4) return false;
    }
    return true;
  }

  /** Points every {@code step} metres, so pure pursuit has a smooth line to follow. */
  private static List<double[]> densify(List<double[]> pts, double step) {
    List<double[]> out = new ArrayList<>();
    for (int i = 0; i < pts.size() - 1; i++) {
      double[] a = pts.get(i);
      double[] b = pts.get(i + 1);
      double d = Math.hypot(b[0] - a[0], b[1] - a[1]);
      int n = Math.max(1, (int) Math.ceil(d / step));
      for (int k = 0; k < n; k++) {
        double t = (double) k / n;
        out.add(new double[] {a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t});
      }
    }
    out.add(pts.get(pts.size() - 1));
    return out;
  }

  // endregion

  static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }
}
