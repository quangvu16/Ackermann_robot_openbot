package org.openbot.app.robot.slam;

import java.util.Locale;

/** Robot pose in the map: metres, and heading in radians counter-clockwise from +x. */
public final class Pose {
  public final double x;
  public final double y;
  public final double theta;

  public Pose(double x, double y, double theta) {
    this.x = x;
    this.y = y;
    this.theta = wrap(theta);
  }

  public static final Pose ZERO = new Pose(0, 0, 0);

  public double distanceTo(Pose o) {
    return Math.hypot(o.x - x, o.y - y);
  }

  public double distanceTo(double px, double py) {
    return Math.hypot(px - x, py - y);
  }

  /** Radians in (-pi, pi]. */
  public static double wrap(double a) {
    double r = Math.IEEEremainder(a, 2 * Math.PI);
    return r <= -Math.PI ? r + 2 * Math.PI : r;
  }

  @Override
  public String toString() {
    return String.format(Locale.US, "x %.2f m, y %.2f m, %.0f°", x, y, Math.toDegrees(theta));
  }
}
