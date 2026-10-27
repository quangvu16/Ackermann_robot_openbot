package org.openbot.app.robot.slam;

import java.util.ArrayList;
import java.util.List;

/**
 * What the Indoor SLAM screen talks to: the chosen SLAM engine plus the chosen localizer for a saved
 * map ("Localize only"). Switching engine keeps the map and the car's pose: the new engine starts on
 * top of the current map. Same class in Swift (SlamCore.swift).
 */
public final class SlamSystem implements Slam {

  public enum Localizer {
    SCAN_MATCHING("Scan matching"),
    AMCL("AMCL");

    public final String label;

    Localizer(String label) {
      this.label = label;
    }
  }

  private volatile Slam engine;
  private volatile Localizer localizer = Localizer.SCAN_MATCHING;
  private boolean mapping = true;
  private final Amcl amcl = new Amcl();
  private final ScanOdometry odometry = new ScanOdometry();
  private volatile Pose amclPose = Pose.ZERO;
  private double lastImuYaw = Double.NaN;
  private boolean amclSearchPending;
  private volatile int amclSeq;
  private volatile boolean amclFound;
  private volatile boolean lastFromAmcl;
  private volatile double amclQuality;
  private volatile int amclScans;
  private final List<double[]> amclTrail = new ArrayList<>();

  public SlamSystem(Type type, OccupancyGrid grid) {
    engine = Slam.create(type, grid);
  }

  private boolean amclActive() {
    return !mapping && localizer == Localizer.AMCL;
  }

  public Slam getEngine() {
    return engine;
  }

  public Localizer getLocalizer() {
    return localizer;
  }

  /** Change the SLAM engine, keeping the map and pose. Call on the SLAM thread. */
  public void setEngine(Type t) {
    Slam old = engine;
    if (old.type() == t) return;
    Pose p = getPose();
    Slam next = Slam.create(t, old.getGrid());
    if (old.getScanCount() > 0) next.setGrid(old.getGrid());
    next.setMapping(mapping);
    next.resetPose(p);
    engine = next;
  }

  /** Change the localizer used in "Localize only". Call on the SLAM thread. */
  public void setLocalizer(Localizer l) {
    if (l == localizer) return;
    Pose p = getPose();
    localizer = l;
    if (amclActive()) startAmcl(p);
    else engine.resetPose(p);
  }

  private void startAmcl(Pose p) {
    amcl.init(p);
    odometry.reset(p);
    amclPose = p;
    lastImuYaw = Double.NaN;
  }

  @Override
  public Type type() {
    return engine.type();
  }

  @Override
  public OccupancyGrid getGrid() {
    return engine.getGrid();
  }

  @Override
  public Pose getPose() {
    return amclActive() ? amclPose : engine.getPose();
  }

  @Override
  public int getScanCount() {
    return engine.getScanCount() + amclScans;
  }

  @Override
  public double getMatchQuality() {
    return amclActive() ? amclQuality : engine.getMatchQuality();
  }

  @Override
  public List<double[]> getTrail() {
    if (!amclActive()) return engine.getTrail();
    synchronized (amclTrail) {
      return new ArrayList<>(amclTrail);
    }
  }

  @Override
  public boolean isMapping() {
    return mapping;
  }

  @Override
  public void setMapping(boolean m) {
    if (m == mapping) return;
    Pose p = getPose();
    mapping = m;
    if (m) {
      engine.setMapping(true);
      engine.resetPose(p);
    } else {
      engine.setMapping(false);
      if (localizer == Localizer.AMCL) startAmcl(p);
    }
  }

  @Override
  public void reset() {
    engine.reset();
    startAmcl(Pose.ZERO);
    synchronized (amclTrail) {
      amclTrail.clear();
    }
  }

  @Override
  public void setGrid(OccupancyGrid loaded) {
    engine.setGrid(loaded);
    startAmcl(Pose.ZERO);
  }

  @Override
  public void resetPose(Pose p) {
    engine.resetPose(p);
    if (amclActive()) startAmcl(p);
  }

  @Override
  public void requestGlobalLocalization() {
    if (amclActive()) amclSearchPending = true;
    else engine.requestGlobalLocalization();
  }

  @Override
  public int getRelocalizeSeq() {
    return engine.getRelocalizeSeq() + amclSeq;
  }

  @Override
  public boolean wasRelocalizeFound() {
    return lastFromAmcl ? amclFound : engine.wasRelocalizeFound();
  }

  @Override
  public int getSubmapCount() {
    return amclActive() ? amcl.size() : engine.getSubmapCount();
  }

  @Override
  public int getLoopClosureCount() {
    return amclActive() ? 0 : engine.getLoopClosureCount();
  }

  @Override
  public Pose update(double[] angles, double[] ranges, double imuYaw, double time) {
    if (!amclActive()) {
      int before = engine.getRelocalizeSeq();
      Pose p = engine.update(angles, ranges, imuYaw, time);
      if (engine.getRelocalizeSeq() != before) lastFromAmcl = false;
      return p;
    }
    BaseSlam.Scan s = new BaseSlam.Scan(angles, ranges);
    OccupancyGrid g = engine.getGrid();
    if (amclSearchPending && s.n >= 20) {
      amclSearchPending = false;
      amcl.initGlobal(g);
    }
    double dTheta = 0;
    if (!Double.isNaN(imuYaw)) {
      if (!Double.isNaN(lastImuYaw)) dTheta = Pose.wrap(imuYaw - lastImuYaw);
      lastImuYaw = imuYaw;
    }
    Pose odo = odometry.getPose();
    Pose odom = odometry.update(s, new Pose(odo.x, odo.y, odo.theta + dTheta));
    boolean wasSearching = amcl.isGlobalSearch();
    if (amcl.update(odom, angles, ranges, g)) {
      amclPose = amcl.getEstimate();
      if (wasSearching && !amcl.isGlobalSearch()) finishSearch(true);
      else if (wasSearching && amcl.getUpdates() > 60) finishSearch(false);
    } else {
      // Between filter updates, carry the estimate along with the odometry.
      amclPose = amclPose.compose(odo.inverse().compose(odom));
    }
    amclQuality = SlamEngine.hitFraction(g, amclPose, s.px, s.py);
    amclScans++;
    synchronized (amclTrail) {
      if (amclTrail.isEmpty() || Math.hypot(amclTrail.get(amclTrail.size() - 1)[0] - amclPose.x,
          amclTrail.get(amclTrail.size() - 1)[1] - amclPose.y) > 0.05) {
        amclTrail.add(new double[] {amclPose.x, amclPose.y});
        if (amclTrail.size() > 5000) amclTrail.remove(0);
      }
    }
    return amclPose;
  }

  private void finishSearch(boolean found) {
    amclFound = found;
    lastFromAmcl = true;
    amclSeq++;
  }
}
