package org.openbot.app.robot.slam;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * 2-D LiDAR SLAM for the car: scan matching against the map being built, CoreSLAM style (as in
 * BreezySLAM's RMHC_SLAM), with the phone's gyroscope as the heading prior (the smartphone-sensor
 * idea of SmartphoneGraphSLAM) and slam_toolbox's two modes: <b>mapping</b> (match, then add the
 * scan to the map) and <b>localization</b> (match against a saved map without changing it).
 *
 * <p>Each scan: predict the pose (last pose, plus the gyro's change in heading, plus a damped share
 * of the last step), then search around that prediction with random-mutation hill climbing (RMHC) for
 * the pose whose beam ends land on the most wall, then add the scan to the map from that pose. No
 * loop closure: on a long loop the map can drift; reset or re-localize on a saved map.
 *
 * <p>Pure Java, unit tested on a simulated room. Same algorithm in Swift (SlamCore.swift).
 */
public final class SlamEngine {

  /** Beams beyond this are treated as "nothing seen". LD14: 8 m. */
  public static final double MAX_RANGE = 8.0;
  /** Closer than this is the car itself. */
  public static final double MIN_RANGE = 0.15;

  private OccupancyGrid grid;
  private Pose pose = Pose.ZERO;
  private Pose lastMapPose;
  private double lastStepX;
  private double lastStepY;
  private double lastImuYaw = Double.NaN;
  private boolean mapping = true;
  private int scans;
  private double lastScore;
  private double lastQuality;
  private final List<double[]> trail = new ArrayList<>();
  private final Random random = new Random(42);

  /** RMHC search: iterations and starting step sizes (shrink as the search improves). */
  int iterations = 250;

  double sigmaXY = 0.08;
  double sigmaTheta = Math.toRadians(4);

  public SlamEngine(OccupancyGrid grid) {
    this.grid = grid;
  }

  public OccupancyGrid getGrid() {
    return grid;
  }

  public Pose getPose() {
    return pose;
  }

  public int getScanCount() {
    return scans;
  }

  /** 0..1: share of beam ends that landed on a known wall in the last match. */
  public double getMatchQuality() {
    return lastQuality;
  }

  /** A copy of the trail driven (safe to read from another thread). */
  public List<double[]> getTrail() {
    synchronized (trail) {
      return new ArrayList<>(trail);
    }
  }

  public boolean isMapping() {
    return mapping;
  }

  /** Mapping adds scans to the map; localization only matches against it (slam_toolbox modes). */
  public void setMapping(boolean mapping) {
    this.mapping = mapping;
  }

  /** Start again: empty map, pose at the origin. */
  public void reset() {
    grid.clear();
    resetPose(Pose.ZERO);
    scans = 0;
  }

  /** Use a loaded map; the car is assumed to start where the map's origin is unless told otherwise. */
  public void setGrid(OccupancyGrid grid) {
    this.grid = grid;
    resetPose(Pose.ZERO);
    scans = 1; // a loaded map counts as "has walls": match from the first scan
  }

  /** Put the car somewhere on the map (e.g. a long press on the map in localization mode). */
  public void resetPose(Pose p) {
    pose = p;
    lastMapPose = null;
    lastStepX = 0;
    lastStepY = 0;
    lastImuYaw = Double.NaN;
    synchronized (trail) {
      trail.clear();
    }
  }

  /**
   * One LiDAR revolution.
   *
   * @param angles beam angles, radians, + left of the nose
   * @param ranges metres (beams outside MIN_RANGE..MAX_RANGE are skipped)
   * @param imuYaw the phone's integrated yaw in radians (+ counter-clockwise), NaN if unknown
   */
  public Pose update(double[] angles, double[] ranges, double imuYaw) {
    // Keep the usable beams.
    int n = 0;
    for (double r : ranges) if (r >= MIN_RANGE && r < MAX_RANGE) n++;
    double[] a = new double[n];
    double[] r = new double[n];
    int k = 0;
    for (int i = 0; i < ranges.length; i++) {
      if (ranges[i] >= MIN_RANGE && ranges[i] < MAX_RANGE) {
        a[k] = angles[i];
        r[k] = ranges[i];
        k++;
      }
    }

    // Predict: gyro heading change, plus a damped repeat of the last step.
    double dTheta = 0;
    if (!Double.isNaN(imuYaw)) {
      if (!Double.isNaN(lastImuYaw)) dTheta = Pose.wrap(imuYaw - lastImuYaw);
      lastImuYaw = imuYaw;
    }
    Pose prior = new Pose(pose.x + lastStepX * 0.6, pose.y + lastStepY * 0.6, pose.theta + dTheta);

    Pose best = prior;
    if (scans > 0 && n >= 20) {
      best = match(prior, a, r);
    }
    lastStepX = best.x - pose.x;
    lastStepY = best.y - pose.y;
    pose = best;
    scans++;

    // Add the scan to the map when mapping, and only every few cm / degrees (fewer smeared walls).
    if (mapping && n >= 20) {
      boolean moved = lastMapPose == null
          || lastMapPose.distanceTo(pose) > 0.04
          || Math.abs(Pose.wrap(pose.theta - lastMapPose.theta)) > Math.toRadians(2);
      if (moved) {
        grid.integrate(pose, a, r, MAX_RANGE);
        lastMapPose = pose;
      }
    }
    synchronized (trail) {
      if (trail.isEmpty()
          || Math.hypot(trail.get(trail.size() - 1)[0] - pose.x, trail.get(trail.size() - 1)[1] - pose.y) > 0.05) {
        trail.add(new double[] {pose.x, pose.y});
        if (trail.size() > 5000) trail.remove(0);
      }
    }
    return pose;
  }

  /** Random-mutation hill climbing around {@code prior} (CoreSLAM's search). */
  Pose match(Pose prior, double[] a, double[] r) {
    double bestScore = score(prior, a, r);
    Pose best = prior;
    double sxy = sigmaXY;
    double sth = sigmaTheta;
    int sinceImprove = 0;
    for (int i = 0; i < iterations; i++) {
      Pose cand = new Pose(
          best.x + random.nextGaussian() * sxy,
          best.y + random.nextGaussian() * sxy,
          best.theta + random.nextGaussian() * sth);
      double s = score(cand, a, r);
      if (s > bestScore) {
        bestScore = s;
        best = cand;
        sinceImprove = 0;
      } else if (++sinceImprove > 25) {
        sxy *= 0.5; // refine
        sth *= 0.5;
        sinceImprove = 0;
        if (sxy < grid.resolution * 0.1) break;
      }
    }
    lastScore = bestScore;
    lastQuality = hitFraction(best, a, r);
    return best;
  }

  /** Share of beam ends that land on (or next to) a wall cell. */
  double hitFraction(Pose p, double[] a, double[] r) {
    int hits = 0;
    double c = Math.cos(p.theta);
    double sn = Math.sin(p.theta);
    for (int i = 0; i < a.length; i++) {
      double lx = r[i] * Math.cos(a[i]);
      double ly = r[i] * Math.sin(a[i]);
      int cx = grid.cellX(p.x + c * lx - sn * ly);
      int cy = grid.cellY(p.y + sn * lx + c * ly);
      if (grid.isOccupied(cx, cy) || grid.isOccupied(cx + 1, cy) || grid.isOccupied(cx - 1, cy)
          || grid.isOccupied(cx, cy + 1) || grid.isOccupied(cx, cy - 1)) hits++;
    }
    return a.length == 0 ? 0 : (double) hits / a.length;
  }

  double score(Pose p, double[] a, double[] r) {
    double s = 0;
    double c = Math.cos(p.theta);
    double sn = Math.sin(p.theta);
    for (int i = 0; i < a.length; i++) {
      double lx = r[i] * Math.cos(a[i]);
      double ly = r[i] * Math.sin(a[i]);
      double wx = p.x + c * lx - sn * ly;
      double wy = p.y + sn * lx + c * ly;
      s += grid.hitScore(grid.cellX(wx), grid.cellY(wy));
    }
    return s;
  }
}
