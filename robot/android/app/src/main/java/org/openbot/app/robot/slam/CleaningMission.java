package org.openbot.app.robot.slam;

import java.util.ArrayList;
import java.util.List;

/**
 * Runs a Rooms &amp; Zones job on top of the {@link Navigator}: drive a list of waypoints in order
 * (lanes over rooms or zones), go to one point, or return home. Its status and flag use Valetudo's
 * words (StatusStateAttribute: idle, docked, cleaning, moving, returning, paused, error; flags
 * segment, zone, target, resumable), so the REST API and MQTT can report them as they are.
 *
 * <p>A waypoint the planner cannot reach (behind a no-go area, say) is skipped and counted rather
 * than stopping the whole job. Pure Java, unit tested; same in Swift (RoomsCore.swift).
 */
public final class CleaningMission {

  public enum Status {
    IDLE("idle"),
    DOCKED("docked"),
    CLEANING("cleaning"),
    MOVING("moving"),
    RETURNING("returning"),
    PAUSED("paused"),
    ERROR("error");

    public final String valetudo;

    Status(String v) {
      valetudo = v;
    }
  }

  public enum Flag {
    NONE("none"),
    SEGMENT("segment"),
    ZONE("zone"),
    TARGET("target"),
    RESUMABLE("resumable");

    public final String valetudo;

    Flag(String v) {
      valetudo = v;
    }
  }

  /** Seconds a waypoint may stay unreachable before it is skipped. */
  public static final double NO_PATH_GRACE = 1.0;

  /** Within this distance of home and not busy: "docked". */
  public static final double DOCKED_RADIUS = 0.35;

  private final Navigator navigator;
  private List<double[]> waypoints = new ArrayList<>();
  private int index;
  private int skipped;
  private Status status = Status.IDLE;
  private Status before = Status.IDLE; // what PAUSED will resume
  private Flag beforeFlag = Flag.NONE;
  private Flag flag = Flag.NONE;
  private String label = "";
  private String error = "";
  private double[] home;
  private double noPathSince = -1;

  public CleaningMission(Navigator navigator) {
    this.navigator = navigator;
  }

  public void setHome(double[] home) {
    this.home = home;
  }

  /** Patrol these waypoints (from {@link CoveragePlanner}); {@code label} names the rooms or zones. */
  public void start(List<double[]> wps, Flag f, String label) {
    if (wps.isEmpty()) {
      fail("Nothing to drive: the area has no free space the car fits in");
      return;
    }
    waypoints = new ArrayList<>(wps);
    index = 0;
    skipped = 0;
    flag = f;
    this.label = label;
    error = "";
    status = Status.CLEANING;
    goToCurrent();
  }

  public void goTo(double x, double y) {
    waypoints = new ArrayList<>();
    waypoints.add(new double[] {x, y});
    index = 0;
    skipped = 0;
    flag = Flag.TARGET;
    label = "";
    error = "";
    status = Status.MOVING;
    goToCurrent();
  }

  public void returnHome() {
    if (home == null) {
      fail("No home set: choose the home tool and tap the map");
      return;
    }
    waypoints = new ArrayList<>();
    waypoints.add(home.clone());
    index = 0;
    skipped = 0;
    flag = Flag.NONE;
    label = "";
    error = "";
    status = Status.RETURNING;
    goToCurrent();
  }

  public void pause() {
    if (!isBusy()) return;
    before = status;
    beforeFlag = flag;
    status = Status.PAUSED;
    flag = Flag.RESUMABLE;
    navigator.cancel();
  }

  /** Resume after a pause (Valetudo's "start" while paused). */
  public boolean resume() {
    if (status != Status.PAUSED) return false;
    status = before;
    flag = beforeFlag;
    goToCurrent();
    return true;
  }

  public void stop() {
    navigator.cancel();
    waypoints = new ArrayList<>();
    index = 0;
    status = Status.IDLE;
    flag = Flag.NONE;
    label = "";
  }

  private void fail(String why) {
    navigator.cancel();
    waypoints = new ArrayList<>();
    status = Status.ERROR;
    flag = Flag.NONE;
    error = why;
  }

  public void clearError() {
    if (status == Status.ERROR) {
      status = Status.IDLE;
      error = "";
    }
  }

  private void goToCurrent() {
    noPathSince = -1;
    double[] p = waypoints.get(index);
    navigator.setGoal(p[0], p[1]);
  }

  public boolean isBusy() {
    return status == Status.CLEANING || status == Status.MOVING || status == Status.RETURNING;
  }

  /**
   * One step at about 10 Hz. Returns the drive command, or null when nothing should drive (then
   * the caller stops the car).
   */
  public Navigator.Command step(Pose pose, OccupancyGrid grid, double frontMin, double now) {
    if (!isBusy()) return null;
    Navigator.Command c = navigator.step(pose, grid, frontMin, now);
    if (navigator.getGoal() != null) {
      if (c.state != Navigator.State.NO_PATH) {
        noPathSince = -1;
        return c;
      }
      // No path: wait a moment (the pose may settle), then give this waypoint up.
      if (noPathSince < 0) noPathSince = now;
      if (now - noPathSince < NO_PATH_GRACE) return c;
      navigator.cancel();
      skipped++;
    }
    noPathSince = -1;
    index++;
    if (index < waypoints.size()) {
      goToCurrent();
      return c;
    }
    boolean wasMoving = status == Status.MOVING;
    boolean allSkipped = skipped == waypoints.size();
    waypoints = new ArrayList<>();
    index = 0;
    flag = Flag.NONE;
    label = "";
    if (allSkipped) {
      fail(wasMoving ? "No path to that point on this map" : "No lane of the area could be reached");
    } else {
      status = Status.IDLE;
    }
    return null;
  }

  /** Valetudo status, with "docked" for idle at home. */
  public Status getStatus(Pose pose) {
    if (status == Status.IDLE && home != null && pose != null
        && Math.hypot(pose.x - home[0], pose.y - home[1]) < DOCKED_RADIUS) {
      return Status.DOCKED;
    }
    return status;
  }

  public Flag getFlag() {
    return flag;
  }

  public String getLabel() {
    return label;
  }

  public String getError() {
    return error;
  }

  public int getIndex() {
    return index;
  }

  public int getTotal() {
    return waypoints.size();
  }

  public int getSkipped() {
    return skipped;
  }

  /** Waypoints still to drive (for drawing the plan). */
  public List<double[]> remaining() {
    if (waypoints.isEmpty()) return new ArrayList<>();
    return new ArrayList<>(waypoints.subList(Math.min(index, waypoints.size()), waypoints.size()));
  }

  public double[] target() {
    return index < waypoints.size() ? waypoints.get(index) : null;
  }
}
