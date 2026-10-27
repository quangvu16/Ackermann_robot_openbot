package org.openbot.app.robot.slam;

/**
 * Cartographer's local scan matching, in two steps (cartographer/mapping/internal/2d/scan_matching):
 *
 * <ol>
 *   <li><b>Real-time correlative scan matcher</b>: tries every pose in a small window (here ±0.15 m,
 *       ±20°) on the grid's cells and angle steps, and keeps the one whose points land on the most
 *       likely-occupied cells. Brute force, so it can't get stuck in a local minimum.
 *   <li><b>"Ceres" scan matcher</b>: polishes that pose below the cell size by least squares on a
 *       smooth (bicubic) version of the map, with springs that keep it near the first answer.
 * </ol>
 *
 * <p>Rewritten in Java for this project from the algorithm in Hess et al., "Real-Time Loop Closure in
 * 2D LIDAR SLAM" (ICRA 2016) and Cartographer's defaults (Apache-2.0). Same code in Swift (SlamCore).
 */
public final class ScanMatcher {

  /** Cartographer real_time_correlative_scan_matcher: linear window 0.1 m (0.15 m here: no wheel odometry). */
  public double linearWindow = 0.15;

  public double angularWindow = Math.toRadians(20);
  public double translationDeltaCostWeight = 0.1;
  public double rotationDeltaCostWeight = 0.1;

  /**
   * Cartographer ceres_scan_matcher (local SLAM) uses occupied 1, translation 10, rotation 40, pulling
   * towards a pose predicted from odometry/IMU. Ours pulls towards the correlative answer, which is
   * only good to a cell, so the springs are 10x softer (same 1:4 ratio): in a simulated corridor loop
   * this roughly halved the worst drift.
   */
  public double occupiedSpaceWeight = 1;

  public double translationWeight = 1;
  public double rotationWeight = 4;
  public int maxIterations = 20;

  /** Mean probability of the points at the last answer (0.1..0.9). */
  public double lastScore;

  /**
   * How well the last refined match pins down position in its weakest direction, 0..1: the smaller
   * over the larger eigenvalue of the 2x2 position information matrix. Near 0 means the scan could
   * slide (a bare corridor). Not in Cartographer; the degeneracy test of Zhang et al. (ICRA 2016).
   */
  public double lastConstraint;

  /** Both steps. Points are in the robot frame (metres). */
  public Pose match(OccupancyGrid grid, Pose initial, double[] px, double[] py) {
    Pose coarse = correlative(grid, initial, px, py);
    return refine(grid, coarse, px, py);
  }

  /** Angle step so the farthest point moves about one cell (Cartographer's SearchParameters). */
  static double angularStep(double resolution, double[] px, double[] py) {
    double maxR = 0;
    for (int i = 0; i < px.length; i++) maxR = Math.max(maxR, Math.hypot(px[i], py[i]));
    if (maxR < resolution) return Math.toRadians(5);
    double v = 1 - resolution * resolution / (2 * maxR * maxR);
    return Math.acos(Math.max(-1, Math.min(1, v)));
  }

  /** Step 1: brute-force search of the window. */
  public Pose correlative(OccupancyGrid g, Pose initial, double[] px, double[] py) {
    int n = px.length;
    if (n == 0) return initial;
    float[] prob = g.probabilities();
    double res = g.resolution;
    double step = angularStep(res, px, py);
    int nAng = Math.min(80, (int) Math.ceil(angularWindow / step));
    int nLin = (int) Math.ceil(linearWindow / res);
    int[] cx = new int[n];
    int[] cy = new int[n];
    double bestScore = -1;
    Pose best = initial;
    for (int k = -nAng; k <= nAng; k++) {
      double th = initial.theta + k * step;
      double c = Math.cos(th), s = Math.sin(th);
      for (int i = 0; i < n; i++) {
        cx[i] = g.cellX(initial.x + c * px[i] - s * py[i]);
        cy[i] = g.cellY(initial.y + s * px[i] + c * py[i]);
      }
      for (int ox = -nLin; ox <= nLin; ox++) {
        for (int oy = -nLin; oy <= nLin; oy++) {
          double sum = 0;
          for (int i = 0; i < n; i++) {
            int x = cx[i] + ox, y = cy[i] + oy;
            sum += (x >= 0 && y >= 0 && x < g.width && y < g.height) ? prob[y * g.width + x] : OccupancyGrid.P_MIN;
          }
          double score = sum / n;
          double pen = translationDeltaCostWeight * Math.hypot(ox * res, oy * res)
              + rotationDeltaCostWeight * Math.abs(k * step);
          score *= Math.exp(-pen * pen);
          if (score > bestScore) {
            bestScore = score;
            best = new Pose(initial.x + ox * res, initial.y + oy * res, th);
          }
        }
      }
    }
    lastScore = bestScore;
    return best;
  }

  /** Step 2: Gauss-Newton / Levenberg-Marquardt on the bicubic map (what Ceres does in Cartographer). */
  public Pose refine(OccupancyGrid g, Pose target, double[] px, double[] py) {
    return refine(g, target, px, py, occupiedSpaceWeight, translationWeight, rotationWeight);
  }

  public Pose refine(OccupancyGrid g, Pose target, double[] px, double[] py, double wOcc, double wT, double wR) {
    int n = px.length;
    if (n == 0) return target;
    float[] prob = g.probabilities();
    double x = target.x, y = target.y, th = target.theta;
    double lambda = 1e-3;
    double[] out = new double[3];
    double cost = cost(g, prob, px, py, x, y, th, target, wOcc, wT, wR, out);
    double scale = wOcc / Math.sqrt(n);
    for (int it = 0; it < maxIterations; it++) {
      // Normal equations J^T J and J^T r for the 3 unknowns.
      double[][] h = new double[3][3];
      double[] b = new double[3];
      double c = Math.cos(th), s = Math.sin(th);
      for (int i = 0; i < n; i++) {
        double wx = x + c * px[i] - s * py[i];
        double wy = y + s * px[i] + c * py[i];
        bicubic(g, prob, wx, wy, out);
        double r = scale * (1 - out[0]);
        double dpx = out[1], dpy = out[2]; // dP/dwx, dP/dwy
        double dwxdt = -s * px[i] - c * py[i];
        double dwydt = c * px[i] - s * py[i];
        double[] j = {-scale * dpx, -scale * dpy, -scale * (dpx * dwxdt + dpy * dwydt)};
        accumulate(h, b, j, r);
      }
      accumulate(h, b, new double[] {wT, 0, 0}, wT * (x - target.x));
      accumulate(h, b, new double[] {0, wT, 0}, wT * (y - target.y));
      accumulate(h, b, new double[] {0, 0, wR}, wR * Pose.wrap(th - target.theta));
      boolean accepted = false;
      boolean converged = false;
      for (int tries = 0; tries < 6 && !accepted; tries++) {
        double[][] a = new double[3][3];
        for (int r = 0; r < 3; r++) {
          for (int k = 0; k < 3; k++) a[r][k] = h[r][k];
          a[r][r] += lambda * (h[r][r] + 1e-9);
        }
        double[] d = solve3(a, new double[] {-b[0], -b[1], -b[2]});
        if (d == null) break;
        double nx = x + d[0], ny = y + d[1], nth = th + d[2];
        double nc = cost(g, prob, px, py, nx, ny, nth, target, wOcc, wT, wR, out);
        if (nc < cost) {
          converged = cost - nc < 1e-10;
          x = nx;
          y = ny;
          th = nth;
          cost = nc;
          lambda = Math.max(1e-7, lambda * 0.3);
          accepted = true;
        } else {
          lambda *= 10;
        }
      }
      if (!accepted || converged) break;
    }
    Pose result = new Pose(x, y, th);
    lastScore = meanProbability(g, result, px, py);
    lastConstraint = translationConstraint(g, prob, result, px, py);
    return result;
  }

  private static double cost(OccupancyGrid g, float[] prob, double[] px, double[] py, double x, double y,
      double th, Pose t, double wOcc, double wT, double wR, double[] out) {
    double c = Math.cos(th), s = Math.sin(th);
    double sum = 0;
    for (int i = 0; i < px.length; i++) {
      bicubic(g, prob, x + c * px[i] - s * py[i], y + s * px[i] + c * py[i], out);
      double r = 1 - out[0];
      sum += r * r;
    }
    sum *= wOcc * wOcc / px.length;
    double dx = x - t.x, dy = y - t.y, dt = Pose.wrap(th - t.theta);
    return sum + wT * wT * (dx * dx + dy * dy) + wR * wR * dt * dt;
  }

  static double translationConstraint(OccupancyGrid g, float[] prob, Pose p, double[] px, double[] py) {
    double c = Math.cos(p.theta), s = Math.sin(p.theta);
    double hxx = 0, hxy = 0, hyy = 0;
    double[] out = new double[3];
    for (int i = 0; i < px.length; i++) {
      bicubic(g, prob, p.x + c * px[i] - s * py[i], p.y + s * px[i] + c * py[i], out);
      hxx += out[1] * out[1];
      hxy += out[1] * out[2];
      hyy += out[2] * out[2];
    }
    double tr = hxx + hyy, det = hxx * hyy - hxy * hxy;
    double disc = Math.sqrt(Math.max(0, tr * tr / 4 - det));
    double hi = tr / 2 + disc, lo = tr / 2 - disc;
    return hi <= 1e-12 ? 0 : Math.max(0, lo) / hi;
  }

  /** Mean cell probability of the points (no interpolation), the score shown as "match". */
  public static double meanProbability(OccupancyGrid g, Pose p, double[] px, double[] py) {
    if (px.length == 0) return 0;
    float[] prob = g.probabilities();
    double c = Math.cos(p.theta), s = Math.sin(p.theta), sum = 0;
    for (int i = 0; i < px.length; i++) {
      int x = g.cellX(p.x + c * px[i] - s * py[i]);
      int y = g.cellY(p.y + s * px[i] + c * py[i]);
      sum += g.inBounds(x, y) ? prob[y * g.width + x] : OccupancyGrid.P_MIN;
    }
    return sum / px.length;
  }

  private static void accumulate(double[][] h, double[] b, double[] j, double r) {
    for (int a = 0; a < 3; a++) {
      b[a] += j[a] * r;
      for (int k = 0; k < 3; k++) h[a][k] += j[a] * j[k];
    }
  }

  static double[] solve3(double[][] a, double[] b) {
    double det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    if (Math.abs(det) < 1e-300) return null;
    double[] x = new double[3];
    for (int col = 0; col < 3; col++) {
      double[][] m = {a[0].clone(), a[1].clone(), a[2].clone()};
      for (int r = 0; r < 3; r++) m[r][col] = b[r];
      x[col] = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
          - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
          + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])) / det;
    }
    return x;
  }

  private static float cell(OccupancyGrid g, float[] prob, int x, int y) {
    return (x >= 0 && y >= 0 && x < g.width && y < g.height) ? prob[y * g.width + x] : OccupancyGrid.P_MIN;
  }

  /**
   * Bicubic (Catmull-Rom) interpolation of the probability grid at world (wx, wy), as Ceres'
   * BiCubicInterpolator. out = {value, d/dwx, d/dwy}.
   */
  static void bicubic(OccupancyGrid g, float[] prob, double wx, double wy, double[] out) {
    double u = (wx - g.originX) / g.resolution - 0.5; // cell centres at integers
    double v = (wy - g.originY) / g.resolution - 0.5;
    int x0 = (int) Math.floor(u), y0 = (int) Math.floor(v);
    double tx = u - x0, ty = v - y0;
    double[] f = new double[4];
    double[] fd = new double[4];
    double[] tmp = new double[2];
    for (int j = 0; j < 4; j++) {
      int yy = y0 - 1 + j;
      cubic(cell(g, prob, x0 - 1, yy), cell(g, prob, x0, yy), cell(g, prob, x0 + 1, yy), cell(g, prob, x0 + 2, yy), tx, tmp);
      f[j] = tmp[0];
      fd[j] = tmp[1];
    }
    cubic(f[0], f[1], f[2], f[3], ty, tmp);
    double val = tmp[0], dv = tmp[1];
    cubic(fd[0], fd[1], fd[2], fd[3], ty, tmp);
    double du = tmp[0];
    out[0] = val;
    out[1] = du / g.resolution;
    out[2] = dv / g.resolution;
  }

  /** Catmull-Rom cubic through p1 (t = 0) and p2 (t = 1): {value, derivative}. */
  private static void cubic(double p0, double p1, double p2, double p3, double t, double[] out) {
    double a = (-p0 + 3 * p1 - 3 * p2 + p3) / 2;
    double b = (2 * p0 - 5 * p1 + 4 * p2 - p3) / 2;
    double c = (-p0 + p2) / 2;
    out[0] = ((a * t + b) * t + c) * t + p1;
    out[1] = (3 * a * t + 2 * b) * t + c;
  }
}
