package org.openbot.app.robot.slam;

import java.util.Random;

/**
 * Nav2's AMCL (Adaptive Monte Carlo Localization; Thrun, Burgard and Fox, "Probabilistic Robotics",
 * ch. 8): hundreds of guesses ("particles") of where the car is on a fixed map. Each update they move
 * with the measured motion plus noise, are weighted by how well the scan fits the map there, and are
 * resampled so good guesses multiply. Defaults from Nav2's amcl parameters:
 *
 * <ul>
 *   <li>Odometry motion model (differential, alpha1..alpha4 = 0.2), updates after 0.25 m or 0.2 rad.
 *   <li>Likelihood-field laser model: 60 beams, sigma_hit 0.2, z_hit 0.5, z_rand 0.5, distances up
 *       to 2 m, combined as amcl does (p += pz^3).
 *   <li>500 particles (min_particles); 2000 (max_particles) while spread out after a global search.
 * </ul>
 *
 * <p>The car has no wheel encoders, so the odometry comes from {@link ScanOdometry} (scan-to-scan
 * matching, as with laser_scan_matcher in ROS). Rewritten for this project from the book and Nav2's
 * documented behaviour (nav2_amcl is LGPL).
 */
final class Amcl {

  static final double ALPHA1 = 0.2;
  static final double ALPHA2 = 0.2;
  static final double ALPHA3 = 0.2;
  static final double ALPHA4 = 0.2;
  static final double UPDATE_MIN_D = 0.25;
  static final double UPDATE_MIN_A = 0.2;
  static final int MAX_BEAMS = 60;
  static final double SIGMA_HIT = 0.2;
  static final double Z_HIT = 0.5;
  static final double Z_RAND = 0.5;
  static final double LIKELIHOOD_MAX_DIST = 2.0;
  static final int MIN_PARTICLES = 500;
  static final int MAX_PARTICLES = 2000;

  private double[] px = new double[0];
  private double[] py = new double[0];
  private double[] pt = new double[0];
  private double[] w = new double[0];
  private final Random random = new Random(11);
  private OccupancyGrid fieldGrid;
  private int fieldVersion;
  private float[] field; // distance to the nearest wall, metres (capped)
  private Pose lastOdom;
  private Pose estimate = Pose.ZERO;
  private boolean globalSearch;
  private int updates;

  Pose getEstimate() {
    return estimate;
  }

  int size() {
    return px.length;
  }

  /** Spread of the particles (m), for "converged?". */
  double spread() {
    double mx = estimate.x, my = estimate.y, s = 0;
    for (int i = 0; i < px.length; i++) s += w[i] * ((px[i] - mx) * (px[i] - mx) + (py[i] - my) * (py[i] - my));
    return Math.sqrt(s);
  }

  boolean isGlobalSearch() {
    return globalSearch;
  }

  int getUpdates() {
    return updates;
  }

  /** Particles around a known pose (Nav2's initial pose: here 0.25 m and 0.2 rad). */
  void init(Pose p) {
    resize(MIN_PARTICLES);
    for (int i = 0; i < px.length; i++) {
      px[i] = p.x + random.nextGaussian() * 0.25;
      py[i] = p.y + random.nextGaussian() * 0.25;
      pt[i] = p.theta + random.nextGaussian() * 0.2;
      w[i] = 1.0 / px.length;
    }
    estimate = p;
    lastOdom = null;
    globalSearch = false;
    updates = 0;
  }

  /** Nav2's reinitialize_global_localization: particles everywhere free, any heading. */
  void initGlobal(OccupancyGrid g) {
    resize(MAX_PARTICLES);
    int tries = 0;
    for (int i = 0; i < px.length; ) {
      int cx = random.nextInt(g.width), cy = random.nextInt(g.height);
      if (g.get(cx, cy) < OccupancyGrid.FREE_THRESHOLD || ++tries > 2_000_000) {
        px[i] = g.worldX(cx);
        py[i] = g.worldY(cy);
        pt[i] = (random.nextDouble() * 2 - 1) * Math.PI;
        w[i] = 1.0 / px.length;
        i++;
      }
    }
    lastOdom = null;
    globalSearch = true;
    updates = 0;
  }

  private void resize(int n) {
    px = new double[n];
    py = new double[n];
    pt = new double[n];
    w = new double[n];
  }

  /** Returns true if the filter updated (moved enough). */
  boolean update(Pose odom, double[] angles, double[] ranges, OccupancyGrid g) {
    if (px.length == 0) init(odom);
    if (lastOdom == null) {
      lastOdom = odom;
      weigh(angles, ranges, g);
      resample();
      return true;
    }
    Pose d = lastOdom.inverse().compose(odom);
    if (Math.hypot(d.x, d.y) < UPDATE_MIN_D && Math.abs(d.theta) < UPDATE_MIN_A) return false;
    motion(lastOdom, odom);
    lastOdom = odom;
    weigh(angles, ranges, g);
    resample();
    updates++;
    return true;
  }

  /** Probabilistic Robotics table 5.6 (sample_motion_model_odometry), as nav2_amcl's DiffMotionModel. */
  private void motion(Pose o0, Pose o1) {
    double dx = o1.x - o0.x, dy = o1.y - o0.y;
    double trans = Math.hypot(dx, dy);
    double rot1 = trans < 0.01 ? 0 : Pose.wrap(Math.atan2(dy, dx) - o0.theta);
    double rot2 = Pose.wrap(o1.theta - o0.theta - rot1);
    // Driving backwards: measure the turn from the rear, as amcl does.
    double r1 = Math.min(Math.abs(rot1), Math.abs(Pose.wrap(rot1 - Math.PI)));
    double r2 = Math.min(Math.abs(rot2), Math.abs(Pose.wrap(rot2 - Math.PI)));
    double sRot1 = Math.sqrt(ALPHA1 * r1 * r1 + ALPHA2 * trans * trans);
    double sTrans = Math.sqrt(ALPHA3 * trans * trans + ALPHA4 * (r1 * r1 + r2 * r2));
    double sRot2 = Math.sqrt(ALPHA1 * r2 * r2 + ALPHA2 * trans * trans);
    for (int i = 0; i < px.length; i++) {
      double a1 = rot1 - random.nextGaussian() * sRot1;
      double t = trans - random.nextGaussian() * sTrans;
      double a2 = rot2 - random.nextGaussian() * sRot2;
      px[i] += t * Math.cos(pt[i] + a1);
      py[i] += t * Math.sin(pt[i] + a1);
      pt[i] = Pose.wrap(pt[i] + a1 + a2);
    }
  }

  /** Distance to the nearest wall, capped at LIKELIHOOD_MAX_DIST (two-pass chamfer transform). */
  private void buildField(OccupancyGrid g) {
    int w = g.width, h = g.height;
    float big = (float) LIKELIHOOD_MAX_DIST;
    float res = (float) g.resolution, diag = (float) (g.resolution * Math.sqrt(2));
    float[] f = new float[w * h];
    for (int y = 0; y < h; y++) for (int x = 0; x < w; x++) f[y * w + x] = g.isOccupied(x, y) ? 0 : big;
    for (int y = 0; y < h; y++) {
      for (int x = 0; x < w; x++) {
        int i = y * w + x;
        float v = f[i];
        if (x > 0) v = Math.min(v, f[i - 1] + res);
        if (y > 0) {
          v = Math.min(v, f[i - w] + res);
          if (x > 0) v = Math.min(v, f[i - w - 1] + diag);
          if (x < w - 1) v = Math.min(v, f[i - w + 1] + diag);
        }
        f[i] = v;
      }
    }
    for (int y = h - 1; y >= 0; y--) {
      for (int x = w - 1; x >= 0; x--) {
        int i = y * w + x;
        float v = f[i];
        if (x < w - 1) v = Math.min(v, f[i + 1] + res);
        if (y < h - 1) {
          v = Math.min(v, f[i + w] + res);
          if (x < w - 1) v = Math.min(v, f[i + w + 1] + diag);
          if (x > 0) v = Math.min(v, f[i + w - 1] + diag);
        }
        f[i] = Math.min(v, big);
      }
    }
    field = f;
    fieldGrid = g;
    fieldVersion = g.getVersion();
  }

  private void weigh(double[] angles, double[] ranges, OccupancyGrid g) {
    if (field == null || fieldGrid != g || fieldVersion != g.getVersion()) buildField(g);
    int n = angles.length;
    int step = Math.max(1, n / MAX_BEAMS);
    double denom = 2 * SIGMA_HIT * SIGMA_HIT;
    double zRandMult = 1.0 / SlamEngine.MAX_RANGE;
    double total = 0;
    for (int i = 0; i < px.length; i++) {
      double c = Math.cos(pt[i]), s = Math.sin(pt[i]);
      double p = 1.0;
      for (int b = 0; b < n; b += step) {
        double r = ranges[b];
        if (r < SlamEngine.MIN_RANGE || r >= SlamEngine.MAX_RANGE) continue;
        double lx = r * Math.cos(angles[b]), ly = r * Math.sin(angles[b]);
        int cx = g.cellX(px[i] + c * lx - s * ly), cy = g.cellY(py[i] + s * lx + c * ly);
        double z = g.inBounds(cx, cy) ? field[cy * g.width + cx] : LIKELIHOOD_MAX_DIST;
        double pz = Z_HIT * Math.exp(-(z * z) / denom) + Z_RAND * zRandMult;
        p += pz * pz * pz;
      }
      w[i] *= p;
      total += w[i];
    }
    if (total <= 0) {
      java.util.Arrays.fill(w, 1.0 / w.length);
    } else {
      for (int i = 0; i < w.length; i++) w[i] /= total;
    }
    // Estimate: weighted mean (circular mean for the heading).
    double mx = 0, my = 0, sc = 0, ss = 0;
    for (int i = 0; i < px.length; i++) {
      mx += w[i] * px[i];
      my += w[i] * py[i];
      sc += w[i] * Math.cos(pt[i]);
      ss += w[i] * Math.sin(pt[i]);
    }
    estimate = new Pose(mx, my, Math.atan2(ss, sc));
  }

  /** Low-variance resampling (resample_interval 1); fewer particles once they agree (KLD idea). */
  private void resample() {
    double spread = spread();
    int n = spread > 1.0 ? MAX_PARTICLES : MIN_PARTICLES;
    if (globalSearch && spread < 0.5) globalSearch = false;
    double[] nx = new double[n], ny = new double[n], nt = new double[n], nw = new double[n];
    double r = random.nextDouble() / n, c = w[0];
    int i = 0;
    for (int m = 0; m < n; m++) {
      double u = r + (double) m / n;
      while (u > c && i < w.length - 1) c += w[++i];
      nx[m] = px[i];
      ny[m] = py[i];
      nt[m] = pt[i];
      nw[m] = 1.0 / n;
    }
    px = nx;
    py = ny;
    pt = nt;
    w = nw;
  }
}
