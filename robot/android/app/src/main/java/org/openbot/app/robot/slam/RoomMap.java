package org.openbot.app.robot.slam;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.ConcurrentSkipListMap;
import java.util.concurrent.CopyOnWriteArrayList;

/**
 * What the user adds on top of a saved SLAM map, the way Valetudo adds it on top of a vacuum's map:
 *
 * <ul>
 *   <li><b>Rooms</b> (Valetudo "segments"): a label per 10 cm cell, from {@link RoomSegmenter}, with
 *       a name each.
 *   <li><b>Zones</b>: 4-corner areas to patrol (Valetudo zone cleaning).
 *   <li><b>No-go areas</b> and <b>virtual walls</b> (Valetudo CombinedVirtualRestrictions): the
 *       planner never goes there ({@link #blocked}).
 *   <li><b>Home</b>: where "return home" drives to (Valetudo's dock / charger location).
 * </ul>
 *
 * Points are world metres (x right, y up), as in the SLAM map. Saved as JSON next to the map. Pure
 * Java; the same class is in Swift (RoomsCore.swift).
 */
public final class RoomMap implements Navigator.KeepOut {

  /** Extra room around a virtual wall's line and a no-go edge, on top of the car's radius. */
  public static final double WALL_MARGIN = 0.05;

  // Room labels (0 = no room) on a coarse grid aligned with the SLAM grid.
  public int w;
  public int h;
  public int factor = 1;
  public double res = 0.1;
  public double originX;
  public double originY;
  public int[] labels = new int[0];
  public final ConcurrentSkipListMap<Integer, String> names = new ConcurrentSkipListMap<>();

  // Copy-on-write: the screen edits them while the planner and the REST API read them.
  public final List<double[]> zones = new CopyOnWriteArrayList<>(); // x0,y0,x1,y1,x2,y2,x3,y3
  public final List<double[]> noGo = new CopyOnWriteArrayList<>(); // x0,y0,...,x3,y3
  public final List<double[]> walls = new CopyOnWriteArrayList<>(); // xA,yA,xB,yB
  public double[] home; // x, y or null

  /** Which map the rooms were made for (see {@link #stampOf}); empty = none yet. */
  public String mapStamp = "";

  /** Car radius used by {@link #blocked}; set from the navigator's settings. */
  public double robotRadius = 0.22;

  // region Rooms

  public boolean hasRooms() {
    return !names.isEmpty();
  }

  public int roomAt(double x, double y) {
    int[] l = labels;
    if (l.length == 0) return 0;
    int cx = (int) Math.floor((x - originX) / res);
    int cy = (int) Math.floor((y - originY) / res);
    if (cx < 0 || cy < 0 || cx >= w || cy >= h) return 0;
    int i = cy * w + cx;
    return i < l.length ? l[i] : 0;
  }

  public String name(int id) {
    String n = names.get(id);
    return n == null ? "Room " + id : n;
  }

  /** Area of a room in square metres. */
  public double area(int id) {
    int n = 0;
    for (int l : labels) if (l == id) n++;
    return n * res * res;
  }

  /** The room's centre of mass, where its name is drawn; null if the room has no cells. */
  public double[] centroid(int id) {
    double sx = 0, sy = 0;
    int n = 0;
    for (int y = 0; y < h; y++) {
      for (int x = 0; x < w; x++) {
        if (labels[y * w + x] != id) continue;
        sx += x;
        sy += y;
        n++;
      }
    }
    if (n == 0) return null;
    return new double[] {originX + (sx / n + 0.5) * res, originY + (sy / n + 0.5) * res};
  }

  /** Take a fresh segmentation; names are reset to "Room 1", "Room 2" ... (largest first). */
  public void setRooms(RoomSegmenter.Result r, String stamp) {
    w = r.w;
    h = r.h;
    factor = r.factor;
    res = r.res;
    originX = r.originX;
    originY = r.originY;
    labels = r.labels;
    names.clear();
    for (int id = 1; id <= r.count; id++) names.put(id, "Room " + id);
    mapStamp = stamp;
  }

  /** A short fingerprint of a map, so rooms made for another map are not used on this one. */
  public static String stampOf(OccupancyGrid g) {
    int occ = 0, free = 0;
    for (float v : g.logOdds) {
      if (v > OccupancyGrid.OCC_THRESHOLD) occ++;
      else if (v < OccupancyGrid.FREE_THRESHOLD) free++;
    }
    return String.format(Locale.ROOT, "%dx%d@%.3f:%.2f,%.2f:%d/%d", g.width, g.height, g.resolution,
        g.originX, g.originY, occ, free);
  }

  // endregion

  // region Restrictions

  @Override
  public boolean blocked(double x, double y) {
    for (double[] p : noGo) {
      if (inside(p, x, y)) return true;
    }
    double clear = robotRadius + WALL_MARGIN;
    for (double[] s : walls) {
      if (segmentDistance(s[0], s[1], s[2], s[3], x, y) < clear) return true;
    }
    return false;
  }

  public boolean hasRestrictions() {
    return !noGo.isEmpty() || !walls.isEmpty();
  }

  /** An axis-aligned box from two opposite corners, as the 4 corners Valetudo uses. */
  public static double[] box(double ax, double ay, double bx, double by) {
    double x0 = Math.min(ax, bx), x1 = Math.max(ax, bx), y0 = Math.min(ay, by), y1 = Math.max(ay, by);
    return new double[] {x0, y0, x1, y0, x1, y1, x0, y1};
  }

  /** Point in a polygon given as x0,y0,x1,y1,... (even-odd rule). */
  public static boolean inside(double[] poly, double x, double y) {
    boolean in = false;
    int n = poly.length / 2;
    for (int i = 0, j = n - 1; i < n; j = i++) {
      double xi = poly[2 * i], yi = poly[2 * i + 1], xj = poly[2 * j], yj = poly[2 * j + 1];
      if ((yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi) in = !in;
    }
    return in;
  }

  public static double segmentDistance(double ax, double ay, double bx, double by, double x, double y) {
    double dx = bx - ax, dy = by - ay;
    double l2 = dx * dx + dy * dy;
    double t = l2 < 1e-12 ? 0 : Math.max(0, Math.min(1, ((x - ax) * dx + (y - ay) * dy) / l2));
    return Math.hypot(ax + t * dx - x, ay + t * dy - y);
  }

  /** Bounding box of a polygon: minX, minY, maxX, maxY. */
  public static double[] bounds(double[] poly) {
    double[] b = {Double.MAX_VALUE, Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE};
    for (int i = 0; i + 1 < poly.length; i += 2) {
      b[0] = Math.min(b[0], poly[i]);
      b[1] = Math.min(b[1], poly[i + 1]);
      b[2] = Math.max(b[2], poly[i]);
      b[3] = Math.max(b[3], poly[i + 1]);
    }
    return b;
  }

  /**
   * The zone, no-go area or wall nearest to (x, y) within {@code within} metres, to select or delete
   * it: returns {kind, index} with kind 0 zone, 1 no-go, 2 wall; or null.
   */
  public int[] itemAt(double x, double y, double within) {
    int[] best = null;
    double bd = within;
    for (int k = 0; k < 2; k++) {
      List<double[]> list = k == 0 ? zones : noGo;
      for (int i = 0; i < list.size(); i++) {
        double[] p = list.get(i);
        double d = inside(p, x, y) ? 0 : polygonEdgeDistance(p, x, y);
        if (d <= bd) {
          bd = d;
          best = new int[] {k, i};
        }
      }
    }
    for (int i = 0; i < walls.size(); i++) {
      double[] s = walls.get(i);
      double d = segmentDistance(s[0], s[1], s[2], s[3], x, y);
      if (d <= bd) {
        bd = d;
        best = new int[] {2, i};
      }
    }
    return best;
  }

  static double polygonEdgeDistance(double[] p, double x, double y) {
    double d = Double.MAX_VALUE;
    int n = p.length / 2;
    for (int i = 0, j = n - 1; i < n; j = i++) {
      d = Math.min(d, segmentDistance(p[2 * j], p[2 * j + 1], p[2 * i], p[2 * i + 1], x, y));
    }
    return d;
  }

  // endregion

  // region JSON

  public String toJson() {
    StringBuilder b = new StringBuilder(1024 + labels.length / 4);
    b.append("{\"version\":1,\"mapStamp\":").append(Json.quote(mapStamp));
    b.append(String.format(Locale.ROOT, ",\"rooms\":{\"w\":%d,\"h\":%d,\"factor\":%d,\"res\":%.4f,\"originX\":%.4f,\"originY\":%.4f",
        w, h, factor, res, originX, originY));
    b.append(",\"rle\":[");
    for (int i = 0; i < labels.length; ) { // value,count pairs
      int v = labels[i], j = i;
      while (j < labels.length && labels[j] == v) j++;
      if (i > 0) b.append(',');
      b.append(v).append(',').append(j - i);
      i = j;
    }
    b.append("],\"names\":{");
    boolean first = true;
    for (Map.Entry<Integer, String> e : names.entrySet()) {
      if (!first) b.append(',');
      first = false;
      b.append('"').append(e.getKey()).append("\":").append(Json.quote(e.getValue()));
    }
    b.append("}}");
    appendList(b, "zones", zones);
    appendList(b, "noGo", noGo);
    appendList(b, "walls", walls);
    b.append(",\"home\":");
    if (home == null) b.append("null");
    else b.append(String.format(Locale.ROOT, "[%.3f,%.3f]", home[0], home[1]));
    return b.append('}').toString();
  }

  private static void appendList(StringBuilder b, String key, List<double[]> list) {
    b.append(",\"").append(key).append("\":[");
    for (int i = 0; i < list.size(); i++) {
      if (i > 0) b.append(',');
      b.append('[');
      double[] p = list.get(i);
      for (int k = 0; k < p.length; k++) {
        if (k > 0) b.append(',');
        b.append(String.format(Locale.ROOT, "%.3f", p[k]));
      }
      b.append(']');
    }
    b.append(']');
  }

  public static RoomMap fromJson(String text) {
    RoomMap m = new RoomMap();
    Map<String, Object> o = Json.obj(Json.parse(text));
    if (o == null) return m;
    m.mapStamp = Json.str(o.get("mapStamp")) == null ? "" : Json.str(o.get("mapStamp"));
    Map<String, Object> r = Json.obj(o.get("rooms"));
    if (r != null) {
      m.w = (int) Json.num(r.get("w"), 0);
      m.h = (int) Json.num(r.get("h"), 0);
      m.factor = (int) Json.num(r.get("factor"), 1);
      m.res = Json.num(r.get("res"), 0.1);
      m.originX = Json.num(r.get("originX"), 0);
      m.originY = Json.num(r.get("originY"), 0);
      List<Object> rle = Json.arr(r.get("rle"));
      int[] labels = new int[Math.max(0, m.w * m.h)];
      int at = 0;
      if (rle != null) {
        for (int i = 0; i + 1 < rle.size(); i += 2) {
          int v = (int) Json.num(rle.get(i), 0), n = (int) Json.num(rle.get(i + 1), 0);
          for (int k = 0; k < n && at < labels.length; k++) labels[at++] = v;
        }
      }
      m.labels = at == labels.length ? labels : new int[0];
      Map<String, Object> names = Json.obj(r.get("names"));
      if (names != null && m.labels.length > 0) {
        for (Map.Entry<String, Object> e : names.entrySet()) {
          try {
            m.names.put(Integer.parseInt(e.getKey()), Json.str(e.getValue()));
          } catch (NumberFormatException ignored) {
            // skip a bad key
          }
        }
      }
    }
    readList(o.get("zones"), m.zones, 8);
    readList(o.get("noGo"), m.noGo, 8);
    readList(o.get("walls"), m.walls, 4);
    List<Object> home = Json.arr(o.get("home"));
    if (home != null && home.size() == 2) m.home = new double[] {Json.num(home.get(0), 0), Json.num(home.get(1), 0)};
    return m;
  }

  private static void readList(Object o, List<double[]> out, int len) {
    List<Object> l = Json.arr(o);
    if (l == null) return;
    for (Object e : l) {
      List<Object> p = Json.arr(e);
      if (p == null || p.size() != len) continue;
      double[] v = new double[len];
      for (int i = 0; i < len; i++) v[i] = Json.num(p.get(i), 0);
      out.add(v);
    }
  }

  // endregion
}
