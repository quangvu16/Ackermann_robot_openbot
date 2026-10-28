package org.openbot.app.robot.slam;

/**
 * Nav2-style costmap on a 10 cm grid: walls inflated by the car's radius are lethal (-1); beyond that
 * a cost fades out over the inflation distance (Nav2's inflation layer); unknown space is allowed but
 * costs 3 (allow_unknown). Free space costs 1, the most expensive non-lethal cell 9.
 */
final class Costmap {
  final int w;
  final int h;
  final int factor;
  final double res;
  final OccupancyGrid grid;
  final float[] cost;

  Costmap(OccupancyGrid grid, Navigator.Params p) {
    this.grid = grid;
    factor = Math.max(1, (int) Math.round(0.10 / grid.resolution));
    res = grid.resolution * factor;
    w = grid.width / factor;
    h = grid.height / factor;
    cost = new float[w * h];
    boolean[] occ = new boolean[w * h];
    boolean[] unknown = new boolean[w * h];
    for (int y = 0; y < h; y++) {
      for (int x = 0; x < w; x++) {
        boolean o = false;
        boolean known = false;
        for (int dy = 0; dy < factor && !o; dy++) {
          for (int dx = 0; dx < factor; dx++) {
            float v = grid.get(x * factor + dx, y * factor + dy);
            if (v > OccupancyGrid.OCC_THRESHOLD) {
              o = true;
              break;
            }
            if (v < OccupancyGrid.FREE_THRESHOLD) known = true;
          }
        }
        occ[y * w + x] = o;
        unknown[y * w + x] = !o && !known;
      }
    }
    int rLethal = (int) Math.ceil(p.robotRadius / res);
    int rSoft = (int) Math.ceil((p.robotRadius + p.inflation) / res);
    for (int i = 0; i < cost.length; i++) cost[i] = unknown[i] ? 3f : 1f;
    for (int y = 0; y < h; y++) {
      for (int x = 0; x < w; x++) {
        if (!occ[y * w + x]) continue;
        for (int dy = -rSoft; dy <= rSoft; dy++) {
          for (int dx = -rSoft; dx <= rSoft; dx++) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
            double d = Math.hypot(dx, dy);
            int i = ny * w + nx;
            if (d <= rLethal) cost[i] = -1;
            else if (d <= rSoft && cost[i] >= 0) {
              float extra = (float) (8 * (1 - (d - rLethal) / (rSoft - rLethal + 1e-9)));
              cost[i] = Math.max(cost[i], 1 + extra);
            }
          }
        }
      }
    }
    // No-go areas and virtual walls (Rooms & Zones): lethal, like Nav2's keepout filter.
    Navigator.KeepOut keepOut = p.keepOut;
    if (keepOut != null) {
      for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
          if (keepOut.blocked(wx(x), wy(y))) cost[y * w + x] = -1;
        }
      }
    }
  }

  int cx(double x) {
    return (int) Math.floor((x - grid.originX) / res);
  }

  int cy(double y) {
    return (int) Math.floor((y - grid.originY) / res);
  }

  boolean free(int x, int y) {
    return x >= 0 && y >= 0 && x < w && y < h && cost[y * w + x] >= 0;
  }

  double wx(int x) {
    return grid.originX + (x + 0.5) * res;
  }

  double wy(int y) {
    return grid.originY + (y + 0.5) * res;
  }

  /** Cost at a world point; -1 lethal or outside. */
  float at(double x, double y) {
    int i = cx(x), j = cy(y);
    return free(i, j) ? cost[j * w + i] : -1;
  }

  /** 0 (free) .. 1 (next to lethal), like Nav2's cost / 252; unknown 0.25. */
  double normalized(double x, double y) {
    float c = at(x, y);
    return c < 0 ? 1 : (c - 1) / 8.0;
  }
}
