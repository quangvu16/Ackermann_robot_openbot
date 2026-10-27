package org.openbot.app.robot.slam;

import java.util.ArrayList;
import java.util.List;

/**
 * The global part of Cartographer's 2-D SLAM: a graph of submap poses joined by constraints, solved
 * by sparse pose adjustment (Konolige et al. 2010, as in cartographer/mapping/internal/optimization).
 *
 * <p>Each constraint says "seen from pose i, pose j is at (x, y, θ)". Local SLAM gives a chain of
 * them between consecutive submaps; loop closure adds one whenever a scan matches an old submap.
 * The solver moves every submap a little so all constraints agree as well as possible. Because a
 * single loop closure is much stiffer than each link of the chain, the drift is spread back along the
 * chain and the two ends of the loop meet.
 *
 * <p>Weights are Cartographer's pose_graph.lua defaults; a Huber loss (scale 10) on loop closures
 * limits the damage of a wrong match. Levenberg-Marquardt with a dense solve: a flat has tens of
 * submaps, so 3n x 3n stays small. Same code in Swift (SlamCore).
 */
public final class PoseGraph {

  /** Cartographer matcher_translation_weight / matcher_rotation_weight (local SLAM links). */
  public static final double SEQ_T = 5e2;

  public static final double SEQ_R = 1.6e3;
  /** Cartographer loop_closure_translation_weight / loop_closure_rotation_weight. */
  public static final double LOOP_T = 1.1e4;

  public static final double LOOP_R = 1e5;
  /** Cartographer optimization_problem.huber_scale. */
  public static final double HUBER = 10;
  /** Cartographer fixed_frame_pose weights: a soft anchor for a session's first submap. */
  public static final double PRIOR_T = 1e1;

  public static final double PRIOR_R = 1e2;

  public static final class Constraint {
    public final int i;
    public final int j; // -1: a prior on i
    public final Pose relative; // pose j seen from pose i (or the prior pose of i)
    public final double wT;
    public final double wR;
    public final boolean robust;

    public Constraint(int i, int j, Pose relative, double wT, double wR, boolean robust) {
      this.i = i;
      this.j = j;
      this.relative = relative;
      this.wT = wT;
      this.wR = wR;
      this.robust = robust;
    }
  }

  private final List<Pose> poses = new ArrayList<>();
  private final List<Boolean> fixed = new ArrayList<>();
  private final List<Constraint> constraints = new ArrayList<>();
  public int maxIterations = 50;

  public int add(Pose initial, boolean isFixed) {
    poses.add(initial);
    fixed.add(isFixed);
    return poses.size() - 1;
  }

  public int size() {
    return poses.size();
  }

  public Pose get(int i) {
    return poses.get(i);
  }

  public void set(int i, Pose p) {
    poses.set(i, p);
  }

  public void addConstraint(Constraint c) {
    constraints.add(c);
  }

  public List<Constraint> getConstraints() {
    return constraints;
  }

  public void clear() {
    poses.clear();
    fixed.clear();
    constraints.clear();
  }

  /**
   * Drops robust (loop-closure) constraints that the current poses still miss by more than {@code
   * maxT} metres or {@code maxR} radians: they joined two places that only look alike. Not in
   * Cartographer, which relies on the Huber loss alone. Returns how many were dropped.
   */
  public int removeOutliers(double maxT, double maxR) {
    int before = constraints.size();
    double[] e = new double[3];
    java.util.Iterator<Constraint> it = constraints.iterator();
    while (it.hasNext()) {
      Constraint c = it.next();
      if (!c.robust) continue;
      error(c, poses, e);
      if (Math.hypot(e[0], e[1]) > maxT || Math.abs(e[2]) > maxR) it.remove();
    }
    return before - constraints.size();
  }

  /** Total weighted squared error (with Huber), for tests and to accept LM steps. */
  public double cost() {
    return cost(poses);
  }

  private double cost(List<Pose> p) {
    double sum = 0;
    double[] e = new double[3];
    for (Constraint c : constraints) {
      error(c, p, e);
      double s = sq(c.wT * e[0]) + sq(c.wT * e[1]) + sq(c.wR * e[2]);
      sum += c.robust && s > HUBER * HUBER ? 2 * HUBER * Math.sqrt(s) - HUBER * HUBER : s;
    }
    return sum;
  }

  private static double sq(double v) {
    return v * v;
  }

  /** e = (T_i^-1 T_j) - measured, or T_i - prior. */
  private static void error(Constraint c, List<Pose> p, double[] e) {
    Pose a = p.get(c.i);
    if (c.j < 0) {
      e[0] = a.x - c.relative.x;
      e[1] = a.y - c.relative.y;
      e[2] = Pose.wrap(a.theta - c.relative.theta);
      return;
    }
    Pose b = p.get(c.j);
    double co = Math.cos(a.theta), si = Math.sin(a.theta);
    double dx = b.x - a.x, dy = b.y - a.y;
    e[0] = co * dx + si * dy - c.relative.x;
    e[1] = -si * dx + co * dy - c.relative.y;
    e[2] = Pose.wrap(b.theta - a.theta - c.relative.theta);
  }

  /** Levenberg-Marquardt. Returns the number of iterations used. */
  public int optimize() {
    int n = poses.size();
    int[] index = new int[n];
    int m = 0;
    for (int k = 0; k < n; k++) index[k] = fixed.get(k) ? -1 : m++;
    if (m == 0) return 0;
    int dim = 3 * m;
    double lambda = 1e-4;
    double cost = cost(poses);
    double[] e = new double[3];
    int it;
    for (it = 0; it < maxIterations; it++) {
      double[][] h = new double[dim][dim];
      double[] g = new double[dim];
      for (Constraint c : constraints) {
        error(c, poses, e);
        double[][] ji = new double[3][3];
        double[][] jj = new double[3][3];
        if (c.j < 0) {
          ji[0][0] = 1;
          ji[1][1] = 1;
          ji[2][2] = 1;
        } else {
          Pose a = poses.get(c.i), b = poses.get(c.j);
          double co = Math.cos(a.theta), si = Math.sin(a.theta);
          double dx = b.x - a.x, dy = b.y - a.y;
          ji[0][0] = -co; ji[0][1] = -si; ji[0][2] = -si * dx + co * dy;
          ji[1][0] = si;  ji[1][1] = -co; ji[1][2] = -co * dx - si * dy;
          ji[2][2] = -1;
          jj[0][0] = co;  jj[0][1] = si;
          jj[1][0] = -si; jj[1][1] = co;
          jj[2][2] = 1;
        }
        double[] w = {c.wT, c.wT, c.wR};
        double[] r = {w[0] * e[0], w[1] * e[1], w[2] * e[2]};
        double rho = 1; // IRLS weight for the Huber loss
        if (c.robust) {
          double s = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
          if (s > HUBER * HUBER) rho = HUBER / Math.sqrt(s);
        }
        int bi = index[c.i], bj = c.j < 0 ? -1 : index[c.j];
        for (int row = 0; row < 3; row++) {
          for (int col = 0; col < 3; col++) { ji[row][col] *= w[row]; jj[row][col] *= w[row]; }
        }
        addBlock(h, g, bi, ji, bi, ji, r, rho);
        if (bj >= 0) {
          addBlock(h, g, bj, jj, bj, jj, r, rho);
          addCross(h, bi, ji, bj, jj, rho);
        }
      }
      boolean accepted = false;
      boolean converged = false;
      for (int tries = 0; tries < 8 && !accepted; tries++) {
        double[][] a = new double[dim][];
        for (int k = 0; k < dim; k++) {
          a[k] = h[k].clone();
          a[k][k] += lambda * h[k][k] + 1e-9;
        }
        double[] negG = new double[dim];
        for (int k = 0; k < dim; k++) negG[k] = -g[k];
        double[] d = cholSolve(a, negG);
        if (d == null) {
          lambda *= 10;
          continue;
        }
        List<Pose> trial = new ArrayList<>(poses);
        for (int k = 0; k < n; k++) {
          int b = index[k];
          if (b < 0) continue;
          Pose p = poses.get(k);
          trial.set(k, new Pose(p.x + d[3 * b], p.y + d[3 * b + 1], p.theta + d[3 * b + 2]));
        }
        double nc = cost(trial);
        if (nc < cost) {
          converged = (cost - nc) < 1e-9 * Math.max(1, cost);
          for (int k = 0; k < n; k++) poses.set(k, trial.get(k));
          cost = nc;
          lambda = Math.max(1e-9, lambda * 0.3);
          accepted = true;
        } else {
          lambda *= 10;
        }
      }
      if (!accepted || converged) break;
    }
    return it;
  }

  /** H[bi][bj] += rho * Ja^T Jb, and g[bi] += rho * Ja^T r when bi == bj via the same call. */
  private static void addBlock(double[][] h, double[] g, int bi, double[][] ja, int bj, double[][] jb, double[] r, double rho) {
    if (bi < 0) return;
    for (int a = 0; a < 3; a++) {
      double gs = 0;
      for (int row = 0; row < 3; row++) gs += ja[row][a] * r[row];
      g[3 * bi + a] += rho * gs;
      for (int b = 0; b < 3; b++) {
        double s = 0;
        for (int row = 0; row < 3; row++) s += ja[row][a] * jb[row][b];
        h[3 * bi + a][3 * bj + b] += rho * s;
      }
    }
  }

  private static void addCross(double[][] h, int bi, double[][] ji, int bj, double[][] jj, double rho) {
    if (bi < 0 || bj < 0) return;
    for (int a = 0; a < 3; a++) {
      for (int b = 0; b < 3; b++) {
        double s = 0;
        for (int row = 0; row < 3; row++) s += ji[row][a] * jj[row][b];
        h[3 * bi + a][3 * bj + b] += rho * s;
        h[3 * bj + b][3 * bi + a] += rho * s;
      }
    }
  }

  /** Solves A x = b for symmetric positive-definite A (Cholesky). Null if A isn't. */
  static double[] cholSolve(double[][] a, double[] b) {
    int n = b.length;
    double[][] l = new double[n][n];
    for (int i = 0; i < n; i++) {
      for (int j = 0; j <= i; j++) {
        double s = a[i][j];
        for (int k = 0; k < j; k++) s -= l[i][k] * l[j][k];
        if (i == j) {
          if (s <= 0) return null;
          l[i][i] = Math.sqrt(s);
        } else {
          l[i][j] = s / l[j][j];
        }
      }
    }
    double[] y = new double[n];
    for (int i = 0; i < n; i++) {
      double s = b[i];
      for (int k = 0; k < i; k++) s -= l[i][k] * y[k];
      y[i] = s / l[i][i];
    }
    double[] x = new double[n];
    for (int i = n - 1; i >= 0; i--) {
      double s = y[i];
      for (int k = i + 1; k < n; k++) s -= l[k][i] * x[k];
      x[i] = s / l[i][i];
    }
    return x;
  }
}
