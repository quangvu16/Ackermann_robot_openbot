package org.openbot.app.robot.slam;

import java.util.List;

/**
 * One 2-D LiDAR SLAM algorithm the user can choose on the Indoor SLAM screen. Every engine builds an
 * {@link OccupancyGrid} in the same world frame, so the navigator, the map view, save/load and the
 * ROS export work the same whichever is chosen. Same interface in Swift (SlamCore.swift).
 */
public interface Slam {

  /** The choices, in the order shown. */
  enum Type {
    CARTOGRAPHER("Cartographer"),
    SLAM_TOOLBOX("slam_toolbox (Karto)"),
    HECTOR("Hector SLAM"),
    GMAPPING("GMapping"),
    CORESLAM("CoreSLAM / BreezySLAM");

    public final String label;

    Type(String label) {
      this.label = label;
    }
  }

  static Slam create(Type t, OccupancyGrid grid) {
    switch (t) {
      case SLAM_TOOLBOX:
        return new KartoSlam(grid);
      case HECTOR:
        return new HectorSlam(grid);
      case GMAPPING:
        return new GMappingSlam(grid);
      case CORESLAM:
        return new CoreSlam(grid);
      default:
        return new SlamEngine(grid);
    }
  }

  Type type();

  OccupancyGrid getGrid();

  Pose getPose();

  int getScanCount();

  /** 0..1: share of beam ends that landed on a known wall in the last match. */
  double getMatchQuality();

  /** A copy of the trail driven (safe to read from another thread). */
  List<double[]> getTrail();

  boolean isMapping();

  void setMapping(boolean mapping);

  void reset();

  /** Use a loaded map (the engine keeps mapping on top of it if mapping is on). */
  void setGrid(OccupancyGrid loaded);

  /** Put the car somewhere on the map. */
  void resetPose(Pose p);

  /**
   * One LiDAR revolution: angles in radians (+ left of the nose), ranges in metres, the phone's
   * integrated yaw (radians, + counter-clockwise, NaN if unknown) and a time in seconds.
   */
  Pose update(double[] angles, double[] ranges, double imuYaw, double time);

  /** Find the car on the map from the next scan with no hint. */
  void requestGlobalLocalization();

  int getRelocalizeSeq();

  boolean wasRelocalizeFound();

  /** Submaps (Cartographer), scan nodes (slam_toolbox), particles (GMapping); 0 if none. */
  default int getSubmapCount() {
    return 0;
  }

  default int getLoopClosureCount() {
    return 0;
  }
}
