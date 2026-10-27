package org.openbot.app.robot.slam;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.PriorityQueue;

/**
 * The global planners the user can choose, on the same {@link Costmap}. Paths are lists of {x, y} or
 * {x, y, direction} points (direction -1 = reversing, only from Hybrid-A*).
 *
 * <ul>
 *   <li><b>NavFn</b> (Nav2 nav2_navfn_planner with use_astar): A* over 8-connected cells weighted by
 *       cost, then straight-line shortcuts through cheap cells.
 *   <li><b>Theta*</b> (Nav2 nav2_theta_star_planner; Nash et al., AAAI 2007): A* that may link a cell
 *       straight to its grandparent when the line between them is clear, giving any-angle paths.
 *       Cost = w_euc_cost 1.0 x length + w_traversal_cost 2.0 x cost along the line.
 *   <li><b>Smac Hybrid-A*</b> (Nav2 nav2_smac_planner; Dolgov et al., "Path Planning for Autonomous
 *       Vehicles in Unknown Semi-structured Environments", IJRR 2010): searches car poses (x, y,
 *       heading in 72 bins) with arcs the car can really drive, forward and reverse (Reeds-Shepp
 *       model) at its minimum turning radius. Penalties as Nav2: non_straight 1.2, reverse 2.0,
 *       change 0.0, cost 2.0. Heuristic: obstacle-aware distance to the goal (Dijkstra).
 * </ul>
 *
 * Rewritten for this project from the papers and Nav2's documentation (Nav2 is Apache-2.0 / LGPL).
 */
final class Planners {

  private Planners() {}

  static List<double[]> plan(Costmap m, Pose start, double gx, double gy, Navigator.Params p) {
    switch (p.planner) {
      case THETA_STAR:
        return thetaStar(m, start.x, start.y, gx, gy);
      case HYBRID_ASTAR:
        return hybridAStar(m, start, gx, gy, p);
      default:
        return navfn(m, start.x, start.y, gx, gy);
    }
  }

  // region NavFn (A* + shortcuts)

  static List<double[]> navfn(Costmap m, double sx, double sy, double gx, double gy) {
    int x0 = m.cx(sx), y0 = m.cy(sy), x1 = m.cx(gx), y1 = m.cy(gy);
    if (!m.free(x1, y1) || x0 < 0 || y0 < 0 || x0 >= m.w || y0 >= m.h) return null;
    int n = m.w * m.h;
    float[] g = new float[n];
    Arrays.fill(g, Float.MAX_VALUE);
    int[] from = new int[n];
    Arrays.fill(from, -1);
    boolean[] closed = new boolean[n];
    int start = y0 * m.w + x0, target = y1 * m.w + x1;
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
      int cx = cur % m.w, cy = cur / m.w;
      for (int k = 0; k < 8; k++) {
        int nx = cx + dxs[k], ny = cy + dys[k];
        if (!m.free(nx, ny)) continue;
        int ni = ny * m.w + nx;
        if (closed[ni]) continue;
        float ng = (float) (g[cur] + (k < 4 ? 1.0 : Math.sqrt(2)) * m.cost[ni]);
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
    List<double[]> out = new ArrayList<>();
    out.add(new double[] {sx, sy});
    int i = 0;
    while (i < cells.size() - 1) {
      int j = cells.size() - 1;
      while (j > i + 1 && !lineCheap(m, cells.get(i), cells.get(j))) j--;
      out.add(new double[] {m.wx(cells.get(j)[0]), m.wy(cells.get(j)[1])});
      i = j;
    }
    out.set(out.size() - 1, new double[] {gx, gy});
    return densify(out, 0.1);
  }

  static double octile(int x0, int y0, int x1, int y1) {
    int dx = Math.abs(x1 - x0), dy = Math.abs(y1 - y0);
    return Math.max(dx, dy) + (Math.sqrt(2) - 1) * Math.min(dx, dy);
  }

  private static boolean lineCheap(Costmap m, int[] a, int[] b) {
    int steps = Math.max(Math.abs(b[0] - a[0]), Math.abs(b[1] - a[1]));
    for (int s = 0; s <= steps; s++) {
      double t = steps == 0 ? 0 : (double) s / steps;
      int x = (int) Math.round(a[0] + (b[0] - a[0]) * t), y = (int) Math.round(a[1] + (b[1] - a[1]) * t);
      if (!m.free(x, y) || m.cost[y * m.w + x] > 4) return false;
    }
    return true;
  }

  static List<double[]> densify(List<double[]> pts, double step) {
    List<double[]> out = new ArrayList<>();
    for (int i = 0; i < pts.size() - 1; i++) {
      double[] a = pts.get(i), b = pts.get(i + 1);
      int n = Math.max(1, (int) Math.ceil(Math.hypot(b[0] - a[0], b[1] - a[1]) / step));
      for (int k = 0; k < n; k++) {
        double t = (double) k / n;
        out.add(new double[] {a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t});
      }
    }
    out.add(pts.get(pts.size() - 1));
    return out;
  }

  // endregion

  // region Theta*

  static final double W_EUC = 1.0;
  static final double W_TRAVERSAL = 2.0;

  /** Cost of a straight segment: length plus how expensive the cells it crosses are. */
  private static double segmentCost(Costmap m, int x0, int y0, int x1, int y1) {
    int steps = Math.max(Math.abs(x1 - x0), Math.abs(y1 - y0));
    double len = Math.hypot(x1 - x0, y1 - y0), sum = 0;
    for (int s = 1; s <= steps; s++) {
      double t = (double) s / steps;
      int x = (int) Math.round(x0 + (x1 - x0) * t), y = (int) Math.round(y0 + (y1 - y0) * t);
      double c = (m.cost[y * m.w + x] - 1) / 8.0;
      sum += c * c;
    }
    return W_EUC * len + W_TRAVERSAL * len * (steps == 0 ? 0 : sum / steps);
  }

  private static boolean lineOfSight(Costmap m, int x0, int y0, int x1, int y1) {
    int steps = Math.max(Math.abs(x1 - x0), Math.abs(y1 - y0));
    for (int s = 0; s <= steps; s++) {
      double t = steps == 0 ? 0 : (double) s / steps;
      if (!m.free((int) Math.round(x0 + (x1 - x0) * t), (int) Math.round(y0 + (y1 - y0) * t))) return false;
    }
    return true;
  }

  static List<double[]> thetaStar(Costmap m, double sx, double sy, double gx, double gy) {
    int x0 = m.cx(sx), y0 = m.cy(sy), x1 = m.cx(gx), y1 = m.cy(gy);
    if (!m.free(x1, y1) || x0 < 0 || y0 < 0 || x0 >= m.w || y0 >= m.h) return null;
    int n = m.w * m.h;
    double[] g = new double[n];
    Arrays.fill(g, Double.MAX_VALUE);
    int[] parent = new int[n];
    Arrays.fill(parent, -1);
    boolean[] closed = new boolean[n];
    int start = y0 * m.w + x0, target = y1 * m.w + x1;
    g[start] = 0;
    parent[start] = start;
    PriorityQueue<double[]> open = new PriorityQueue<>((a, b) -> Double.compare(a[0], b[0]));
    open.add(new double[] {Math.hypot(x1 - x0, y1 - y0) * W_EUC, start});
    int[] dxs = {1, -1, 0, 0, 1, 1, -1, -1};
    int[] dys = {0, 0, 1, -1, 1, -1, 1, -1};
    int expanded = 0;
    while (!open.isEmpty() && expanded < 400_000) {
      int cur = (int) open.poll()[1];
      if (closed[cur]) continue;
      closed[cur] = true;
      expanded++;
      if (cur == target) break;
      int cx = cur % m.w, cy = cur / m.w;
      int par = parent[cur], px = par % m.w, py = par / m.w;
      for (int k = 0; k < 8; k++) {
        int nx = cx + dxs[k], ny = cy + dys[k];
        if (!m.free(nx, ny)) continue;
        int ni = ny * m.w + nx;
        if (closed[ni]) continue;
        double ng;
        int np;
        if (par != cur && lineOfSight(m, px, py, nx, ny)) { // path 2: straight from the grandparent
          ng = g[par] + segmentCost(m, px, py, nx, ny);
          np = par;
        } else {
          ng = g[cur] + segmentCost(m, cx, cy, nx, ny);
          np = cur;
        }
        if (ng < g[ni]) {
          g[ni] = ng;
          parent[ni] = np;
          open.add(new double[] {ng + Math.hypot(x1 - nx, y1 - ny) * W_EUC, ni});
        }
      }
    }
    if (parent[target] < 0) return null;
    List<double[]> out = new ArrayList<>();
    for (int c = target; ; c = parent[c]) {
      out.add(0, new double[] {m.wx(c % m.w), m.wy(c / m.w)});
      if (c == start || parent[c] == c) break;
    }
    out.set(0, new double[] {sx, sy});
    if (out.size() == 1) out.add(new double[] {gx, gy});
    else out.set(out.size() - 1, new double[] {gx, gy});
    return densify(out, 0.1);
  }

  // endregion

  // region Smac Hybrid-A*

  static final int ANGLE_BINS = 72;
  static final double NON_STRAIGHT_PENALTY = 1.2;
  static final double REVERSE_PENALTY = 2.0;
  static final double CHANGE_PENALTY = 0.0;
  static final double COST_PENALTY = 2.0;

  private static final class HNode {
    final double x;
    final double y;
    final double th;
    final int dir;
    final double g;
    final HNode parent;

    HNode(double x, double y, double th, int dir, double g, HNode parent) {
      this.x = x;
      this.y = y;
      this.th = th;
      this.dir = dir;
      this.g = g;
      this.parent = parent;
    }
  }

  /** 2-D Dijkstra distance (metres) from the goal over non-lethal cells: Smac's obstacle heuristic. */
  private static float[] obstacleHeuristic(Costmap m, int gx, int gy) {
    float[] d = new float[m.w * m.h];
    Arrays.fill(d, Float.MAX_VALUE);
    PriorityQueue<double[]> q = new PriorityQueue<>((a, b) -> Double.compare(a[0], b[0]));
    d[gy * m.w + gx] = 0;
    q.add(new double[] {0, gy * m.w + gx});
    int[] dxs = {1, -1, 0, 0, 1, 1, -1, -1};
    int[] dys = {0, 0, 1, -1, 1, -1, 1, -1};
    while (!q.isEmpty()) {
      double[] e = q.poll();
      int cur = (int) e[1];
      if (e[0] > d[cur]) continue;
      int cx = cur % m.w, cy = cur / m.w;
      for (int k = 0; k < 8; k++) {
        int nx = cx + dxs[k], ny = cy + dys[k];
        if (!m.free(nx, ny)) continue;
        int ni = ny * m.w + nx;
        float nd = (float) (d[cur] + (k < 4 ? 1 : Math.sqrt(2)) * m.res);
        if (nd < d[ni]) {
          d[ni] = nd;
          q.add(new double[] {nd, ni});
        }
      }
    }
    return d;
  }

  static List<double[]> hybridAStar(Costmap m, Pose start, double gx, double gy, Navigator.Params p) {
    int x1 = m.cx(gx), y1 = m.cy(gy);
    if (!m.free(x1, y1)) return null;
    int sx = m.cx(start.x), sy = m.cy(start.y);
    if (sx < 0 || sy < 0 || sx >= m.w || sy >= m.h) return null;
    float[] heur = obstacleHeuristic(m, x1, y1);
    double radius = p.wheelbase / Math.tan(Math.toRadians(p.maxSteerDeg)); // minimum turning radius
    double step = Math.sqrt(2) * m.res; // leaves the cell (Nav2's primitive length)
    double dTheta = step / radius;
    double binSize = 2 * Math.PI / ANGLE_BINS;
    boolean[] closed = new boolean[m.w * m.h * ANGLE_BINS];
    PriorityQueue<Object[]> open = new PriorityQueue<>((a, b) -> Double.compare((double) a[0], (double) b[0]));
    HNode root = new HNode(start.x, start.y, start.theta, 1, 0, null);
    open.add(new Object[] {(double) heurAt(m, heur, start.x, start.y), root});
    HNode goalNode = null;
    int iterations = 0;
    while (!open.isEmpty() && iterations++ < 200_000) {
      HNode cur = (HNode) open.poll()[1];
      int cx = m.cx(cur.x), cy = m.cy(cur.y);
      int bin = Math.floorMod((int) Math.round(cur.th / binSize), ANGLE_BINS);
      int key = (cy * m.w + cx) * ANGLE_BINS + bin;
      if (closed[key]) continue;
      closed[key] = true;
      if (Math.hypot(cur.x - gx, cur.y - gy) <= 1.5 * m.res) {
        goalNode = cur;
        break;
      }
      for (int dir = 1; dir >= -1; dir -= 2) {
        for (int turn = -1; turn <= 1; turn++) {
          // Arc of length `step` driving in `dir` with the wheels at full lock (or straight).
          double dth = turn * dTheta; // + = nose turns left, both directions
          double nx, ny, nth = cur.th + dth;
          if (turn == 0) {
            nx = cur.x + dir * step * Math.cos(cur.th);
            ny = cur.y + dir * step * Math.sin(cur.th);
          } else {
            // Exact arc: heading changes by dth over a signed distance dir*step.
            double rSigned = dir * step / dth;
            nx = cur.x + rSigned * (Math.sin(nth) - Math.sin(cur.th));
            ny = cur.y - rSigned * (Math.cos(nth) - Math.cos(cur.th));
          }
          double mx = (cur.x + nx) / 2, my = (cur.y + ny) / 2;
          float cEnd = m.at(nx, ny), cMid = m.at(mx, my);
          if (cEnd < 0 || cMid < 0) continue;
          int nb = Math.floorMod((int) Math.round(nth / binSize), ANGLE_BINS);
          int nkey = (m.cy(ny) * m.w + m.cx(nx)) * ANGLE_BINS + nb;
          if (closed[nkey]) continue;
          double travel = step * (1 + COST_PENALTY * (cEnd - 1) / 8.0);
          if (turn != 0) travel *= NON_STRAIGHT_PENALTY;
          if (dir < 0) travel *= REVERSE_PENALTY;
          if (dir != cur.dir) travel += CHANGE_PENALTY;
          HNode next = new HNode(nx, ny, nth, dir, cur.g + travel, cur);
          open.add(new Object[] {next.g + heurAt(m, heur, nx, ny), next});
        }
      }
    }
    if (goalNode == null) return null;
    List<double[]> out = new ArrayList<>();
    for (HNode k = goalNode; k != null; k = k.parent) {
      // Each point carries the direction used to reach it; the start takes the first move's.
      int dir = k.parent == null ? (out.isEmpty() ? 1 : (int) out.get(0)[2]) : k.dir;
      out.add(0, new double[] {k.x, k.y, dir});
    }
    out.add(new double[] {gx, gy, out.get(out.size() - 1)[2]});
    return out;
  }

  private static float heurAt(Costmap m, float[] heur, double x, double y) {
    int cx = m.cx(x), cy = m.cy(y);
    if (cx < 0 || cy < 0 || cx >= m.w || cy >= m.h) return Float.MAX_VALUE / 4;
    float v = heur[cy * m.w + cx];
    return v == Float.MAX_VALUE ? Float.MAX_VALUE / 4 : v;
  }

  // endregion
}
