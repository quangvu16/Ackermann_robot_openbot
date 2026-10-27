package org.openbot.app.robot.slam;

import java.util.ArrayList;
import java.util.List;

/**
 * What every non-Cartographer engine shares: the global map, pose, trail, gyro prior, a constant-
 * velocity prediction, and global localization (branch and bound over the whole map, as in {@link
 * SlamEngine}). Subclasses only say how one scan updates the pose and the map ({@link #step}).
 */
abstract class BaseSlam implements Slam {

  /** One revolution, filtered: valid beams (a, r), their points (px, py) and a thinned copy (mx, my). */
  static final class Scan {
    final double[] a;
    final double[] r;
    final double[] px;
    final double[] py;
    final double[] mx;
    final double[] my;
    final int n;

    Scan(double[] angles, double[] ranges) {
      int c = 0;
      for (double v : ranges) if (v >= SlamEngine.MIN_RANGE && v < SlamEngine.MAX_RANGE) c++;
      a = new double[c];
      r = new double[c];
      px = new double[c];
      py = new double[c];
      int k = 0;
      for (int i = 0; i < ranges.length; i++) {
        if (ranges[i] >= SlamEngine.MIN_RANGE && ranges[i] < SlamEngine.MAX_RANGE) {
          a[k] = angles[i];
          r[k] = ranges[i];
          px[k] = r[k] * Math.cos(a[k]);
          py[k] = r[k] * Math.sin(a[k]);
          k++;
        }
      }
      n = c;
      double[][] m = SlamEngine.adaptiveVoxel(px, py, 0.5, 200);
      mx = m[0];
      my = m[1];
    }
  }

  protected volatile OccupancyGrid grid;
  protected volatile Pose pose = Pose.ZERO;
  protected boolean mapping = true;
  protected volatile int scans;
  protected volatile double quality;
  private double lastImuYaw = Double.NaN;
  private double stepX;
  private double stepY;
  private final List<double[]> trail = new ArrayList<>();
  protected final ScanMatcher matcher = new ScanMatcher();
  private volatile boolean relocalizeRequested;
  private volatile int relocalizeSeq;
  private volatile boolean relocalizeFound;
  private final GlobalLocalizer globalLocalizer = new GlobalLocalizer();

  BaseSlam(OccupancyGrid grid) {
    this.grid = grid;
  }

  @Override
  public OccupancyGrid getGrid() {
    return grid;
  }

  @Override
  public Pose getPose() {
    return pose;
  }

  @Override
  public int getScanCount() {
    return scans;
  }

  @Override
  public double getMatchQuality() {
    return quality;
  }

  @Override
  public List<double[]> getTrail() {
    synchronized (trail) {
      return new ArrayList<>(trail);
    }
  }

  protected void setTrail(List<double[]> t) {
    synchronized (trail) {
      trail.clear();
      trail.addAll(t);
    }
  }

  @Override
  public boolean isMapping() {
    return mapping;
  }

  @Override
  public void setMapping(boolean m) {
    mapping = m;
  }

  @Override
  public void reset() {
    OccupancyGrid g = grid;
    grid = new OccupancyGrid(g.width, g.height, g.resolution, g.originX, g.originY);
    scans = 0;
    onMapReplaced();
    resetPose(Pose.ZERO);
    setTrail(new ArrayList<>());
  }

  @Override
  public void setGrid(OccupancyGrid loaded) {
    grid = loaded.copy();
    scans = 1;
    onMapReplaced();
    resetPose(Pose.ZERO);
    setTrail(new ArrayList<>());
  }

  @Override
  public void resetPose(Pose p) {
    pose = p;
    stepX = 0;
    stepY = 0;
    lastImuYaw = Double.NaN;
    onPoseReset(p);
  }

  @Override
  public void requestGlobalLocalization() {
    relocalizeRequested = true;
  }

  @Override
  public int getRelocalizeSeq() {
    return relocalizeSeq;
  }

  @Override
  public boolean wasRelocalizeFound() {
    return relocalizeFound;
  }

  /** The map was cleared or replaced (grid already set). */
  protected void onMapReplaced() {}

  /** The pose jumped (long press, global localization, reset). */
  protected void onPoseReset(Pose p) {}

  /** One scan: return the new pose, updating the map if mapping. */
  protected abstract Pose step(Scan s, Pose prior, double time);

  @Override
  public final Pose update(double[] angles, double[] ranges, double imuYaw, double time) {
    Scan s = new Scan(angles, ranges);
    double dTheta = 0;
    if (!Double.isNaN(imuYaw)) {
      if (!Double.isNaN(lastImuYaw)) dTheta = Pose.wrap(imuYaw - lastImuYaw);
      lastImuYaw = imuYaw;
    }
    if (relocalizeRequested && s.n >= 20) {
      relocalizeRequested = false;
      Pose found = globalLocalizer.find(grid, s.mx, s.my, matcher);
      relocalizeFound = found != null;
      if (found != null) {
        double keep = lastImuYaw;
        resetPose(found);
        lastImuYaw = keep;
        dTheta = 0;
      }
      relocalizeSeq++;
    }
    Pose prior = new Pose(pose.x + stepX * 0.6, pose.y + stepY * 0.6, pose.theta + dTheta);
    Pose best = s.n >= 20 ? step(s, prior, time) : prior;
    stepX = best.x - pose.x;
    stepY = best.y - pose.y;
    pose = best;
    scans++;
    synchronized (trail) {
      if (trail.isEmpty()
          || Math.hypot(trail.get(trail.size() - 1)[0] - best.x, trail.get(trail.size() - 1)[1] - best.y) > 0.05) {
        trail.add(new double[] {best.x, best.y});
        if (trail.size() > 5000) trail.remove(0);
      }
    }
    return best;
  }

  /** Branch and bound over the whole map and every heading (Cartographer's MatchFullSubmap). */
  static final class GlobalLocalizer {
    private FastCorrelativeMatcher fcm;
    private OccupancyGrid key;
    private int version;

    Pose find(OccupancyGrid g, double[] px, double[] py, ScanMatcher refiner) {
      if (fcm == null || key != g || version != g.getVersion()) {
        fcm = new FastCorrelativeMatcher(g);
        key = g;
        version = g.getVersion();
      }
      double cx = g.originX + g.width * g.resolution / 2, cy = g.originY + g.height * g.resolution / 2;
      double window = Math.max(g.width, g.height) * g.resolution / 2;
      FastCorrelativeMatcher.Result res =
          fcm.match(new Pose(cx, cy, 0), px, py, window, Math.PI, SlamEngine.GLOBAL_LOCALIZATION_MIN_SCORE);
      return res == null ? null : refiner.refine(g, res.pose, px, py, 20, 10, 1);
    }
  }
}
