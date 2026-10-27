package org.openbot.app.robot.slam;

import java.util.List;
import java.util.Locale;

/**
 * The Indoor SLAM screen on the Sensor link {@code /slam} WebSocket (same format in Swift,
 * SlamWire in SlamAlgorithms.swift). The phone sends, about twice a second:
 *
 * <pre>
 * {"type":"slam","active":true,"engine":"..","localizer":"..","planner":"..","controller":"..",
 *  "mapping":true,"exploring":false,"pose":[x,y,theta],"quality":0.83,"scans":120,"submaps":4,
 *  "loops":2,"goal":[x,y]|null,"path":[[x,y],..],"status":"..",
 *  "map":{"version":7,"res":0.05,"x0":-3.2,"y0":-2.1,"w":120,"h":90,"cells":"<base64>"}}
 * </pre>
 *
 * "map" only when it changed (at most every 2 s) or for a new viewer: one byte per cell, row 0 at
 * y0 (bottom), 0 unknown, 1 free, 2 wall, cropped to the known part. Metres and radians, x right,
 * y up, theta counter-clockwise. With no SLAM screen open: {"type":"slam","active":false}.
 *
 * <p>The page sends commands: {"goal":[x,y]}, {"cancel":true}, {"explore":true|false},
 * {"findMe":true}, {"place":[x,y,theta]}, {"mapping":true|false}.
 */
public final class SlamWire {

  private SlamWire() {}

  public static final String INACTIVE = "{\"type\":\"slam\",\"active\":false}";

  /** A parsed command from the page; kind is null when the text isn't one. */
  public static final class Command {
    public String kind; // goal, cancel, explore, findMe, place, mapping
    public double x;
    public double y;
    public double theta;
    public boolean flag;
  }

  public static String json(String engine, String localizer, String planner, String controller, boolean mapping,
      boolean exploring, Pose pose, double quality, int scans, int submaps, int loops, double[] goal,
      List<double[]> path, String status, OccupancyGrid mapOrNull) {
    StringBuilder b = new StringBuilder(256);
    b.append("{\"type\":\"slam\",\"active\":true");
    b.append(",\"engine\":\"").append(esc(engine)).append('"');
    b.append(",\"localizer\":\"").append(esc(localizer)).append('"');
    b.append(",\"planner\":\"").append(esc(planner)).append('"');
    b.append(",\"controller\":\"").append(esc(controller)).append('"');
    b.append(",\"mapping\":").append(mapping);
    b.append(",\"exploring\":").append(exploring);
    b.append(",\"pose\":[").append(f(pose.x)).append(',').append(f(pose.y)).append(',').append(f(pose.theta)).append(']');
    b.append(",\"quality\":").append(String.format(Locale.US, "%.2f", quality));
    b.append(",\"scans\":").append(scans);
    b.append(",\"submaps\":").append(submaps);
    b.append(",\"loops\":").append(loops);
    b.append(",\"goal\":");
    if (goal == null) b.append("null");
    else b.append('[').append(f(goal[0])).append(',').append(f(goal[1])).append(']');
    b.append(",\"path\":[");
    if (path != null) {
      int step = Math.max(1, path.size() / 150); // at most ~150 points
      boolean first = true;
      for (int i = 0; i < path.size(); i += step) {
        if (!first) b.append(',');
        first = false;
        b.append('[').append(f(path.get(i)[0])).append(',').append(f(path.get(i)[1])).append(']');
      }
      if (!path.isEmpty() && (path.size() - 1) % step != 0) {
        double[] last = path.get(path.size() - 1);
        b.append(first ? "" : ",").append('[').append(f(last[0])).append(',').append(f(last[1])).append(']');
      }
    }
    b.append(']');
    b.append(",\"status\":\"").append(esc(status == null ? "" : status)).append('"');
    if (mapOrNull != null) b.append(",\"map\":").append(mapJson(mapOrNull));
    b.append('}');
    return b.toString();
  }

  /** The known part of the map: 0 unknown, 1 free, 2 wall, base64. */
  public static String mapJson(OccupancyGrid g) {
    int x0 = g.width, y0 = g.height, x1 = -1, y1 = -1;
    for (int y = 0; y < g.height; y++) {
      for (int x = 0; x < g.width; x++) {
        float v = g.logOdds[y * g.width + x];
        if (v > OccupancyGrid.OCC_THRESHOLD || v < OccupancyGrid.FREE_THRESHOLD) {
          if (x < x0) x0 = x;
          if (x > x1) x1 = x;
          if (y < y0) y0 = y;
          if (y > y1) y1 = y;
        }
      }
    }
    if (x1 < 0) {
      x0 = 0;
      y0 = 0;
      x1 = -1;
      y1 = -1;
    }
    int w = x1 - x0 + 1, h = y1 - y0 + 1;
    byte[] cells = new byte[Math.max(0, w * h)];
    for (int y = 0; y < h; y++) {
      for (int x = 0; x < w; x++) {
        float v = g.logOdds[(y + y0) * g.width + x + x0];
        cells[y * w + x] = (byte) (v > OccupancyGrid.OCC_THRESHOLD ? 2 : v < OccupancyGrid.FREE_THRESHOLD ? 1 : 0);
      }
    }
    return "{\"version\":" + g.getVersion()
        + ",\"res\":" + g.resolution
        + ",\"x0\":" + f(g.originX + x0 * g.resolution)
        + ",\"y0\":" + f(g.originY + y0 * g.resolution)
        + ",\"w\":" + Math.max(0, w) + ",\"h\":" + Math.max(0, h)
        + ",\"cells\":\"" + base64(cells) + "\"}";
  }

  /** Parses a page command; never throws (bad input gives kind == null). */
  public static Command parse(String text) {
    Command c = new Command();
    if (text == null) return c;
    String t = text.replaceAll("\\s+", "");
    try {
      if (t.startsWith("{\"goal\":[")) {
        double[] v = numbers(t, "\"goal\":[");
        if (v.length >= 2) {
          c.kind = "goal";
          c.x = v[0];
          c.y = v[1];
        }
      } else if (t.startsWith("{\"place\":[")) {
        double[] v = numbers(t, "\"place\":[");
        if (v.length >= 3) {
          c.kind = "place";
          c.x = v[0];
          c.y = v[1];
          c.theta = v[2];
        }
      } else if (t.equals("{\"cancel\":true}")) {
        c.kind = "cancel";
      } else if (t.equals("{\"findMe\":true}")) {
        c.kind = "findMe";
      } else if (t.startsWith("{\"explore\":")) {
        c.kind = "explore";
        c.flag = t.contains("true");
      } else if (t.startsWith("{\"mapping\":")) {
        c.kind = "mapping";
        c.flag = t.contains("true");
      }
    } catch (RuntimeException e) {
      c.kind = null;
    }
    if (c.kind != null && (Double.isNaN(c.x) || Double.isInfinite(c.x) || Double.isNaN(c.y) || Double.isInfinite(c.y)
        || Double.isNaN(c.theta) || Double.isInfinite(c.theta))) {
      c.kind = null;
    }
    return c;
  }

  private static double[] numbers(String t, String key) {
    int s = t.indexOf(key) + key.length(), e = t.indexOf(']', s);
    String[] parts = t.substring(s, e).split(",");
    double[] out = new double[parts.length];
    for (int i = 0; i < parts.length; i++) out[i] = Double.parseDouble(parts[i]);
    return out;
  }

  private static String f(double v) {
    return String.format(Locale.US, "%.3f", v);
  }

  private static String esc(String s) {
    StringBuilder b = new StringBuilder();
    for (char ch : s.toCharArray()) {
      if (ch == '"' || ch == '\\') b.append('\\').append(ch);
      else if (ch < 0x20) b.append(String.format(Locale.US, "\\u%04x", (int) ch));
      else b.append(ch);
    }
    return b.toString();
  }

  private static final char[] B64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/".toCharArray();

  /** Plain base64 (no android.util, so it runs in unit tests). */
  static String base64(byte[] d) {
    StringBuilder b = new StringBuilder((d.length + 2) / 3 * 4);
    for (int i = 0; i < d.length; i += 3) {
      int n = (d[i] & 0xFF) << 16 | (i + 1 < d.length ? (d[i + 1] & 0xFF) << 8 : 0) | (i + 2 < d.length ? d[i + 2] & 0xFF : 0);
      b.append(B64[n >> 18 & 63]).append(B64[n >> 12 & 63]);
      b.append(i + 1 < d.length ? B64[n >> 6 & 63] : '=');
      b.append(i + 2 < d.length ? B64[n & 63] : '=');
    }
    return b.toString();
  }
}
