package org.openbot.app.robot.slam;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Lanes that cover a room or a zone, the way a vacuum's "clean this room" drives: a boustrophedon
 * (lawn-mower) pattern, Choset and Pignon, "Coverage path planning: the boustrophedon cellular
 * decomposition" (FSR 1997), simplified to straight lanes on the costmap.
 *
 * <p>Lanes run along the longer side of the area, {@code spacing} apart. Each lane is cut where a
 * wall, a no-go area or a virtual wall crosses it, and the pieces are joined in a snake order,
 * starting from the corner nearest the car. The result is a list of waypoints for {@link
 * CleaningMission}; the navigator plans the turns between lanes (this car cannot turn on the spot,
 * so lanes closer than its turning circle are joined by multi-point turns).
 */
public final class CoveragePlanner {

  private CoveragePlanner() {}

  /** Which world points belong to the area to cover. */
  public interface Region {
    boolean contains(double x, double y);
  }

  /** Costmap cells cheaper than this count as drivable for lanes (1 free .. 9 next to a wall). */
  static final float MAX_LANE_COST = 6f;
  /** Lanes shorter than this are skipped (metres). */
  static final double MIN_RUN = 0.3;
  /** Lane ends stop this far inside the drivable part (metres). */
  static final double END_INSET = 0.1;

  public static List<double[]> plan(OccupancyGrid grid, Navigator.Params p, Region region, double spacing, Pose start) {
    Costmap cm = new Costmap(grid, p);
    int w = cm.w, h = cm.h;
    boolean[] in = new boolean[w * h];
    int minX = w, minY = h, maxX = -1, maxY = -1;
    for (int y = 0; y < h; y++) {
      for (int x = 0; x < w; x++) {
        float c = cm.cost[y * w + x];
        if (c < 0 || c > MAX_LANE_COST || !region.contains(cm.wx(x), cm.wy(y))) continue;
        in[y * w + x] = true;
        minX = Math.min(minX, x);
        maxX = Math.max(maxX, x);
        minY = Math.min(minY, y);
        maxY = Math.max(maxY, y);
      }
    }
    List<double[]> out = new ArrayList<>();
    if (maxX < 0) return out;
    boolean alongX = (maxX - minX) >= (maxY - minY); // lanes along the longer side
    int step = Math.max(1, (int) Math.round(spacing / cm.res));
    int lo = alongX ? minY : minX, hi = alongX ? maxY : maxX;
    int runLo = alongX ? minX : minY, runHi = alongX ? maxX : maxY;
    int minRun = (int) Math.ceil(MIN_RUN / cm.res);
    int inset = (int) Math.round(END_INSET / cm.res);

    List<List<double[]>> lanes = new ArrayList<>();
    for (int lane = lo + step / 2; lane <= hi; lane += step) {
      List<double[]> pieces = new ArrayList<>();
      int s = -1, gap = 0;
      for (int t = runLo; t <= runHi + 3; t++) {
        boolean ok = t <= runHi && (alongX ? in[lane * w + t] : in[t * w + lane]);
        if (ok) {
          if (s < 0) s = t;
          gap = 0;
        } else if (s >= 0 && ++gap > 2) { // short gaps (a chair leg) do not cut the lane
          int e = t - gap;
          if (e - s + 1 >= minRun) {
            int a = Math.min(s + inset, e), b = Math.max(e - inset, a);
            pieces.add(alongX ? new double[] {cm.wx(a), cm.wy(lane), cm.wx(b), cm.wy(lane)}
                : new double[] {cm.wx(lane), cm.wy(a), cm.wx(lane), cm.wy(b)});
          }
          s = -1;
          gap = 0;
        }
      }
      if (!pieces.isEmpty()) lanes.add(pieces);
    }
    if (lanes.isEmpty()) return out;

    // Snake order. Try the four corners and start from the one nearest the car.
    double best = Double.MAX_VALUE;
    List<double[]> bestRoute = null;
    for (int corner = 0; corner < 4; corner++) {
      List<double[]> route = snake(lanes, (corner & 1) != 0, (corner & 2) != 0);
      double d = start == null ? 0 : Math.hypot(route.get(0)[0] - start.x, route.get(0)[1] - start.y);
      if (d < best) {
        best = d;
        bestRoute = route;
      }
    }
    // Drop points that are almost on top of each other.
    for (double[] q : bestRoute) {
      if (!out.isEmpty()) {
        double[] last = out.get(out.size() - 1);
        if (Math.hypot(last[0] - q[0], last[1] - q[1]) < 0.15) continue;
      }
      out.add(q);
    }
    return out;
  }

  private static List<double[]> snake(List<List<double[]>> lanes, boolean reverseLanes, boolean firstBackwards) {
    List<List<double[]>> order = new ArrayList<>(lanes);
    if (reverseLanes) Collections.reverse(order);
    List<double[]> route = new ArrayList<>();
    boolean backwards = firstBackwards;
    for (List<double[]> pieces : order) {
      List<double[]> ps = new ArrayList<>(pieces);
      if (backwards) Collections.reverse(ps);
      for (double[] seg : ps) {
        if (backwards) {
          route.add(new double[] {seg[2], seg[3]});
          route.add(new double[] {seg[0], seg[1]});
        } else {
          route.add(new double[] {seg[0], seg[1]});
          route.add(new double[] {seg[2], seg[3]});
        }
      }
      backwards = !backwards;
    }
    return route;
  }
}
