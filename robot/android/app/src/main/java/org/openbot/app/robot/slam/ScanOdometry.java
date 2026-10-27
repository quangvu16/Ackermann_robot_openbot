package org.openbot.app.robot.slam;

/**
 * Odometry from the LiDAR alone, for AMCL (the car has no wheel encoders): each scan is matched to a
 * small rolling map of the last few scans, like ROS's laser_scan_matcher. It drifts slowly, which is
 * fine: AMCL only uses the motion between its updates and corrects the rest against the saved map.
 */
final class ScanOdometry {

  private final ScanMatcher matcher = new ScanMatcher();
  private OccupancyGrid local;
  private Pose pose = Pose.ZERO;
  private Pose lastInsert;
  private double cx;
  private double cy;

  void reset(Pose p) {
    pose = p;
    local = null;
    lastInsert = null;
  }

  Pose getPose() {
    return pose;
  }

  Pose update(BaseSlam.Scan s, Pose prior) {
    pose = local != null && s.n >= 20 ? matcher.match(local, prior, s.mx, s.my) : prior;
    if (local == null || Math.hypot(pose.x - cx, pose.y - cy) > 3) {
      // (Re)start a 14 m map centred here with the current scan in it.
      cx = pose.x;
      cy = pose.y;
      local = new OccupancyGrid(280, 280, 0.05, cx - 7, cy - 7);
      local.integrate(pose, s.a, s.r, SlamEngine.MAX_RANGE);
      lastInsert = pose;
      return pose;
    }
    if (lastInsert.distanceTo(pose) > 0.1 || Math.abs(Pose.wrap(pose.theta - lastInsert.theta)) > Math.toRadians(5)) {
      local.integrate(pose, s.a, s.r, SlamEngine.MAX_RANGE);
      lastInsert = pose;
    }
    return pose;
  }
}
