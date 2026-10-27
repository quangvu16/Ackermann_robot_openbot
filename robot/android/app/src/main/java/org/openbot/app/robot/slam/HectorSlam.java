package org.openbot.app.robot.slam;

/**
 * Hector SLAM (Kohlbrecher et al., "A Flexible and Scalable SLAM System with Full 3D Motion
 * Estimation", SSRR 2011; hector_mapping): no odometry at all. Each scan is matched to the map by
 * Gauss-Newton on a bilinearly interpolated occupancy map, coarse to fine over three map
 * resolutions (5, 10, 20 cm here) so it can't fall into a nearby wrong minimum. The map is updated
 * after 0.4 m or 0.9 rad (hector_mapping defaults) with update factors 0.9 occupied / 0.4 free. No
 * loop closure. Rewritten for this project from the paper (hector_slam is BSD).
 */
final class HectorSlam extends BaseSlam {

  static final int LEVELS = 3; // map_multi_res_levels
  static final double UPDATE_DISTANCE = 0.4; // map_update_distance_thresh
  static final double UPDATE_ANGLE = 0.9; // map_update_angle_thresh (rad)
  static final float L_OCC = (float) Math.log(0.9 / 0.1); // update_factor_occupied
  static final float L_FREE = (float) Math.log(0.4 / 0.6); // update_factor_free

  private OccupancyGrid[] levels;
  private float[][] prob;
  private int[] probVersion;
  private Pose lastMapPose;

  HectorSlam(OccupancyGrid grid) {
    super(grid);
    buildLevels();
  }

  @Override
  public Type type() {
    return Type.HECTOR;
  }

  @Override
  protected void onMapReplaced() {
    buildLevels();
  }

  @Override
  protected void onPoseReset(Pose p) {
    lastMapPose = null;
  }

  /** Level 0 is the global map; coarser levels are rebuilt from it (2x2 averages of log-odds). */
  private void buildLevels() {
    OccupancyGrid g = grid;
    levels = new OccupancyGrid[LEVELS];
    prob = new float[LEVELS][];
    probVersion = new int[] {-1, -1, -1};
    levels[0] = g;
    for (int l = 1; l < LEVELS; l++) {
      OccupancyGrid f = levels[l - 1];
      OccupancyGrid c = new OccupancyGrid(f.width / 2, f.height / 2, f.resolution * 2, f.originX, f.originY);
      for (int y = 0; y < c.height; y++) {
        for (int x = 0; x < c.width; x++) {
          float s = f.get(2 * x, 2 * y) + f.get(2 * x + 1, 2 * y) + f.get(2 * x, 2 * y + 1) + f.get(2 * x + 1, 2 * y + 1);
          c.set(x, y, s / 4);
        }
      }
      levels[l] = c;
    }
  }

  /** Hector's map value: occupancy probability, 0.5 where unknown. */
  private float[] probabilities(int l) {
    OccupancyGrid g = levels[l];
    if (prob[l] == null || probVersion[l] != g.getVersion()) {
      float[] p = new float[g.width * g.height];
      for (int i = 0; i < p.length; i++) p[i] = (float) (1 / (1 + Math.exp(-g.logOdds[i])));
      prob[l] = p;
      probVersion[l] = g.getVersion();
    }
    return prob[l];
  }

  @Override
  protected Pose step(Scan s, Pose prior, double time) {
    Pose best = prior;
    if (scans > 0) {
      for (int l = LEVELS - 1; l >= 0; l--) best = gaussNewton(l, best, s.mx, s.my, l == 0 ? 5 : 3);
      quality = SlamEngine.hitFraction(grid, best, s.px, s.py);
    }
    if (mapping && (lastMapPose == null || lastMapPose.distanceTo(best) > UPDATE_DISTANCE
        || Math.abs(Pose.wrap(best.theta - lastMapPose.theta)) > UPDATE_ANGLE)) {
      for (OccupancyGrid g : levels) g.integrate(best, s.a, s.r, SlamEngine.MAX_RANGE, L_OCC, L_FREE);
      lastMapPose = best;
    }
    return best;
  }

  /** Hector's scan matching: minimise sum (1 - M(S_i(xi)))^2 by Gauss-Newton (paper eq. 7-13). */
  private Pose gaussNewton(int l, Pose start, double[] px, double[] py, int iterations) {
    OccupancyGrid g = levels[l];
    float[] p = probabilities(l);
    double x = start.x, y = start.y, th = start.theta;
    double cost = cost(g, p, px, py, x, y, th);
    double[] m = new double[3];
    for (int it = 0; it < iterations; it++) {
      double[][] h = new double[3][3];
      double[] b = new double[3];
      double c = Math.cos(th), sn = Math.sin(th);
      for (int i = 0; i < px.length; i++) {
        bilinear(g, p, x + c * px[i] - sn * py[i], y + sn * px[i] + c * py[i], m);
        double r = 1 - m[0];
        double[] j = {m[1], m[2], m[1] * (-sn * px[i] - c * py[i]) + m[2] * (c * px[i] - sn * py[i])};
        for (int a = 0; a < 3; a++) {
          b[a] += j[a] * r;
          for (int k = 0; k < 3; k++) h[a][k] += j[a] * j[k];
        }
      }
      for (int a = 0; a < 3; a++) h[a][a] += 1e-6;
      double[] d = ScanMatcher.solve3(h, b);
      if (d == null) break;
      double nc = cost(g, p, px, py, x + d[0], y + d[1], th + d[2]);
      if (nc >= cost) break; // Hector keeps the last good estimate
      x += d[0];
      y += d[1];
      th += d[2];
      cost = nc;
    }
    return new Pose(x, y, th);
  }

  private static double cost(OccupancyGrid g, float[] p, double[] px, double[] py, double x, double y, double th) {
    double c = Math.cos(th), sn = Math.sin(th), sum = 0;
    double[] m = new double[3];
    for (int i = 0; i < px.length; i++) {
      bilinear(g, p, x + c * px[i] - sn * py[i], y + sn * px[i] + c * py[i], m);
      sum += (1 - m[0]) * (1 - m[0]);
    }
    return sum;
  }

  /** Bilinear map value and gradient (paper eq. 4-6); outside the map counts as unknown (0.5). */
  private static void bilinear(OccupancyGrid g, float[] p, double wx, double wy, double[] out) {
    double u = (wx - g.originX) / g.resolution - 0.5, v = (wy - g.originY) / g.resolution - 0.5;
    int x0 = (int) Math.floor(u), y0 = (int) Math.floor(v);
    double tx = u - x0, ty = v - y0;
    double p00 = at(g, p, x0, y0), p10 = at(g, p, x0 + 1, y0), p01 = at(g, p, x0, y0 + 1), p11 = at(g, p, x0 + 1, y0 + 1);
    out[0] = ty * (tx * p11 + (1 - tx) * p01) + (1 - ty) * (tx * p10 + (1 - tx) * p00);
    out[1] = (ty * (p11 - p01) + (1 - ty) * (p10 - p00)) / g.resolution;
    out[2] = (tx * (p11 - p10) + (1 - tx) * (p01 - p00)) / g.resolution;
  }

  private static double at(OccupancyGrid g, float[] p, int x, int y) {
    return (x >= 0 && y >= 0 && x < g.width && y < g.height) ? p[y * g.width + x] : 0.5;
  }
}
