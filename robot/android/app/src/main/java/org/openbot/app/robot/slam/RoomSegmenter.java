package org.openbot.app.robot.slam;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Splits a SLAM map into rooms, like the "segments" a Valetudo vacuum shows. Distance-transform room
 * segmentation, one of the methods compared by Bormann et al., "Room segmentation: Survey,
 * implementation, and analysis" (ICRA 2016, the ipa_room_segmentation package):
 *
 * <ol>
 *   <li>Free space on a 10 cm grid; walls and unknown space are not free.
 *   <li>Distance of each free cell to the nearest non-free cell (3-4 chamfer transform).
 *   <li><b>Room cores</b>: cells further than {@link Params#coreClearance} from any wall. A doorway
 *       is narrower than twice that, so it cuts the cores apart.
 *   <li>Each core grows back over the free space it is nearest to (breadth-first), so the split
 *       falls in the doorways.
 *   <li>Rooms smaller than {@link Params#minRoomArea} join the neighbour they share most border with.
 * </ol>
 *
 * Rooms are numbered by size, largest first. Pure Java, unit tested; same in Swift (RoomsCore.swift).
 */
public final class RoomSegmenter {

  private RoomSegmenter() {}

  public static final class Params {
    /** Metres from the walls a room's core must be. Doorways up to twice this split rooms. */
    public double coreClearance = 0.55;
    /** Cores smaller than this (m²) are ignored (furniture gaps, corners). */
    public double minCoreArea = 0.25;
    /** Rooms smaller than this (m²) are joined to a neighbour. */
    public double minRoomArea = 1.0;
  }

  public static final class Result {
    public int w;
    public int h;
    public int factor;
    public double res;
    public double originX;
    public double originY;
    public int[] labels;
    public int count;
  }

  public static Result segment(OccupancyGrid grid, Params p) {
    Result r = new Result();
    r.factor = Math.max(1, (int) Math.round(0.10 / grid.resolution));
    r.res = grid.resolution * r.factor;
    r.w = grid.width / r.factor;
    r.h = grid.height / r.factor;
    r.originX = grid.originX;
    r.originY = grid.originY;
    int w = r.w, h = r.h, n = w * h;

    // 1. Free cells.
    boolean[] free = new boolean[n];
    for (int y = 0; y < h; y++) {
      for (int x = 0; x < w; x++) {
        boolean occ = false, known = false;
        for (int dy = 0; dy < r.factor && !occ; dy++) {
          for (int dx = 0; dx < r.factor; dx++) {
            float v = grid.get(x * r.factor + dx, y * r.factor + dy);
            if (v > OccupancyGrid.OCC_THRESHOLD) {
              occ = true;
              break;
            }
            if (v < OccupancyGrid.FREE_THRESHOLD) known = true;
          }
        }
        free[y * w + x] = known && !occ;
      }
    }

    // 2. Chamfer distance (3 straight, 4 diagonal; divide by 3 for cells).
    int[] dist = chamfer(free, w, h);

    // 3. Cores: connected components of cells far from walls.
    int coreCells = (int) Math.ceil(p.coreClearance / r.res * 3);
    int[] labels = new int[n];
    int next = 0;
    int minCore = (int) Math.ceil(p.minCoreArea / (r.res * r.res));
    ArrayDeque<Integer> q = new ArrayDeque<>();
    List<Integer> comp = new ArrayList<>();
    for (int i = 0; i < n; i++) {
      if (labels[i] != 0 || dist[i] < coreCells) continue;
      next++;
      comp.clear();
      labels[i] = next;
      q.add(i);
      while (!q.isEmpty()) {
        int c = q.poll();
        comp.add(c);
        int cx = c % w, cy = c / w;
        for (int k = 0; k < 4; k++) {
          int nx = cx + DX[k], ny = cy + DY[k];
          if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
          int j = ny * w + nx;
          if (labels[j] == 0 && dist[j] >= coreCells) {
            labels[j] = next;
            q.add(j);
          }
        }
      }
      if (comp.size() < minCore) {
        for (int c : comp) labels[c] = -1; // too small: grown over later
        next--;
      }
    }
    for (int i = 0; i < n; i++) if (labels[i] < 0) labels[i] = 0;

    // 4. Grow every core over the free space nearest to it.
    q.clear();
    for (int i = 0; i < n; i++) if (labels[i] > 0) q.add(i);
    while (!q.isEmpty()) {
      int c = q.poll();
      int cx = c % w, cy = c / w;
      for (int k = 0; k < 4; k++) {
        int nx = cx + DX[k], ny = cy + DY[k];
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        int j = ny * w + nx;
        if (free[j] && labels[j] == 0) {
          labels[j] = labels[c];
          q.add(j);
        }
      }
    }

    // 5. Join small rooms to their neighbour with the longest shared border.
    int minRoom = (int) Math.ceil(p.minRoomArea / (r.res * r.res));
    boolean changed = true;
    while (changed && next > 0) {
      changed = false;
      int[] area = new int[next + 1];
      for (int l : labels) if (l > 0) area[l]++;
      for (int id = 1; id <= next; id++) {
        if (area[id] == 0 || area[id] >= minRoom) continue;
        int[] border = new int[next + 1];
        for (int i = 0; i < n; i++) {
          if (labels[i] != id) continue;
          int cx = i % w, cy = i / w;
          for (int k = 0; k < 4; k++) {
            int nx = cx + DX[k], ny = cy + DY[k];
            if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
            int l = labels[ny * w + nx];
            if (l > 0 && l != id) border[l]++;
          }
        }
        int best = 0;
        for (int l = 1; l <= next; l++) if (border[l] > border[best]) best = l;
        for (int i = 0; i < n; i++) if (labels[i] == id) labels[i] = best; // 0 = drop it
        changed = true;
        break; // areas changed: recount
      }
    }

    // Number rooms by size, largest first, without gaps.
    int[] area = new int[next + 1];
    for (int l : labels) if (l > 0) area[l]++;
    Integer[] order = new Integer[next];
    for (int i = 0; i < next; i++) order[i] = i + 1;
    Arrays.sort(order, (a, b) -> area[b] != area[a] ? area[b] - area[a] : a - b);
    int[] map = new int[next + 1];
    int count = 0;
    for (Integer id : order) {
      if (area[id] > 0) map[id] = ++count;
    }
    for (int i = 0; i < n; i++) labels[i] = labels[i] > 0 ? map[labels[i]] : 0;
    r.labels = labels;
    r.count = count;
    return r;
  }

  private static final int[] DX = {1, -1, 0, 0};
  private static final int[] DY = {0, 0, 1, -1};

  /** Two-pass 3-4 chamfer distance from each free cell to the nearest non-free cell (in thirds). */
  static int[] chamfer(boolean[] free, int w, int h) {
    int inf = Integer.MAX_VALUE / 4;
    int[] d = new int[w * h];
    for (int i = 0; i < d.length; i++) d[i] = free[i] ? inf : 0;
    for (int y = 0; y < h; y++) {
      for (int x = 0; x < w; x++) {
        int i = y * w + x;
        if (d[i] == 0) continue;
        int v = d[i];
        if (x > 0) v = Math.min(v, d[i - 1] + 3);
        if (y > 0) {
          v = Math.min(v, d[i - w] + 3);
          if (x > 0) v = Math.min(v, d[i - w - 1] + 4);
          if (x < w - 1) v = Math.min(v, d[i - w + 1] + 4);
        }
        if (x == 0 || y == 0 || x == w - 1 || y == h - 1) v = Math.min(v, 3); // map edge: not free beyond
        d[i] = v;
      }
    }
    for (int y = h - 1; y >= 0; y--) {
      for (int x = w - 1; x >= 0; x--) {
        int i = y * w + x;
        if (d[i] == 0) continue;
        int v = d[i];
        if (x < w - 1) v = Math.min(v, d[i + 1] + 3);
        if (y < h - 1) {
          v = Math.min(v, d[i + w] + 3);
          if (x < w - 1) v = Math.min(v, d[i + w + 1] + 4);
          if (x > 0) v = Math.min(v, d[i + w - 1] + 4);
        }
        d[i] = v;
      }
    }
    return d;
  }
}
