package org.openbot.app.robot.slam;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

/**
 * A log-odds occupancy grid, the map the SLAM screen builds and navigates on.
 *
 * <p>World frame: metres, x to the right and y up on screen, origin where mapping started. Cell
 * (0, 0) is the bottom-left corner; {@link #originX}/{@link #originY} are its world position. Each
 * cell holds log-odds of being occupied: 0 = unknown, positive = wall, negative = free.
 *
 * <p>Pure Java (tested on the JVM); the iOS app has the same class in Swift (SlamCore.swift). Written
 * for this project, after the occupancy-grid mapping in BreezySLAM/CoreSLAM and slam_toolbox; no code
 * from them is used (they are LGPL, this project is MIT).
 */
public final class OccupancyGrid {

  /** Log-odds added for a hit, for a pass-through, and the clamps (keeps the map changeable). */
  public static final float L_OCC = 0.9f;

  public static final float L_FREE = -0.35f;
  public static final float L_MIN = -3.5f;
  public static final float L_MAX = 3.5f;
  /** Above this a cell counts as a wall for planning; below {@link #FREE_THRESHOLD} as free. */
  public static final float OCC_THRESHOLD = 0.6f;

  public static final float FREE_THRESHOLD = -0.6f;

  public final int width;
  public final int height;
  public final double resolution;
  public final double originX;
  public final double originY;
  final float[] logOdds;
  private int version;

  public OccupancyGrid(int width, int height, double resolution) {
    this(width, height, resolution, -width * resolution / 2, -height * resolution / 2);
  }

  public OccupancyGrid(int width, int height, double resolution, double originX, double originY) {
    this.width = width;
    this.height = height;
    this.resolution = resolution;
    this.originX = originX;
    this.originY = originY;
    this.logOdds = new float[width * height];
  }

  /** 30 m x 30 m at 5 cm: enough for a flat or an office floor. */
  public static OccupancyGrid indoorDefault() {
    return new OccupancyGrid(600, 600, 0.05);
  }

  public int cellX(double x) {
    return (int) Math.floor((x - originX) / resolution);
  }

  public int cellY(double y) {
    return (int) Math.floor((y - originY) / resolution);
  }

  public double worldX(int cx) {
    return originX + (cx + 0.5) * resolution;
  }

  public double worldY(int cy) {
    return originY + (cy + 0.5) * resolution;
  }

  public boolean inBounds(int cx, int cy) {
    return cx >= 0 && cy >= 0 && cx < width && cy < height;
  }

  public float get(int cx, int cy) {
    return inBounds(cx, cy) ? logOdds[cy * width + cx] : 0f;
  }

  public void set(int cx, int cy, float v) {
    if (inBounds(cx, cy)) logOdds[cy * width + cx] = v;
  }

  public boolean isOccupied(int cx, int cy) {
    return get(cx, cy) > OCC_THRESHOLD;
  }

  /** Bumped on every change, so a view knows when to redraw. */
  public int getVersion() {
    return version;
  }

  public void clear() {
    java.util.Arrays.fill(logOdds, 0f);
    version++;
  }

  private void add(int cx, int cy, float delta) {
    if (!inBounds(cx, cy)) return;
    int i = cy * width + cx;
    float v = logOdds[i] + delta;
    logOdds[i] = v < L_MIN ? L_MIN : (v > L_MAX ? L_MAX : v);
  }

  /**
   * Adds one scan seen from {@code pose}: every cell a beam passes through becomes more free, the
   * cell it ends in more occupied. Beams at {@code maxRange} or beyond only clear (no wall seen).
   *
   * @param angles beam angles in radians in the robot frame, + to the left
   * @param ranges metres
   */
  public void integrate(Pose pose, double[] angles, double[] ranges, double maxRange) {
    int x0 = cellX(pose.x);
    int y0 = cellY(pose.y);
    for (int i = 0; i < angles.length; i++) {
      double r = ranges[i];
      boolean hit = r < maxRange;
      double d = Math.min(r, maxRange);
      double a = pose.theta + angles[i];
      int x1 = cellX(pose.x + d * Math.cos(a));
      int y1 = cellY(pose.y + d * Math.sin(a));
      traceFree(x0, y0, x1, y1);
      if (hit) add(x1, y1, L_OCC - L_FREE); // undo the free step on the end cell, then mark it
    }
    version++;
  }

  /** Bresenham from (x0, y0) to (x1, y1), marking every cell free (the end cell included). */
  private void traceFree(int x0, int y0, int x1, int y1) {
    int dx = Math.abs(x1 - x0);
    int dy = -Math.abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    int x = x0;
    int y = y0;
    for (int guard = 0; guard < 4096; guard++) {
      add(x, y, L_FREE);
      if (x == x1 && y == y1) return;
      int e2 = 2 * err;
      if (e2 >= dy) {
        err += dy;
        x += sx;
      }
      if (e2 <= dx) {
        err += dx;
        y += sy;
      }
    }
  }

  /**
   * How well a cell explains a beam end: its wall-ness, plus a little from its four neighbours so
   * the scan matcher can feel its way towards a wall from one cell away.
   */
  double hitScore(int cx, int cy) {
    if (!inBounds(cx, cy)) return 0;
    double s = Math.max(0, get(cx, cy));
    s += 0.35 * (Math.max(0, get(cx + 1, cy)) + Math.max(0, get(cx - 1, cy))
        + Math.max(0, get(cx, cy + 1)) + Math.max(0, get(cx, cy - 1)));
    return s;
  }

  // region Probabilities for the scan matchers (Cartographer style)

  /** Cartographer's kMinProbability / kMaxProbability: unknown and outside count as 0.1. */
  public static final float P_MIN = 0.1f;

  public static final float P_MAX = 0.9f;
  private static final float[] PROB_LUT = new float[449]; // log-odds -3.5..3.5 in 1/64 steps

  static {
    for (int i = 0; i < PROB_LUT.length; i++) {
      double l = L_MIN + i / 64.0;
      float p = (float) (1 / (1 + Math.exp(-l)));
      PROB_LUT[i] = Math.max(P_MIN, Math.min(P_MAX, p));
    }
  }

  private float[] probCache;
  private int probVersion = -1;

  /** Probability a cell is occupied, 0.1..0.9; never-seen cells are 0.1, as in Cartographer. */
  public static float probabilityOf(float l) {
    if (l == 0f) return P_MIN;
    int i = Math.round((l - L_MIN) * 64f);
    return PROB_LUT[i < 0 ? 0 : (i >= PROB_LUT.length ? PROB_LUT.length - 1 : i)];
  }

  /** All cells as probabilities (row-major), cached until the map changes. One thread only. */
  public float[] probabilities() {
    if (probCache == null || probVersion != version) {
      if (probCache == null) probCache = new float[logOdds.length];
      for (int i = 0; i < logOdds.length; i++) probCache[i] = probabilityOf(logOdds[i]);
      probVersion = version;
    }
    return probCache;
  }

  /** A copy with the same size and origin. */
  public OccupancyGrid copy() {
    OccupancyGrid g = new OccupancyGrid(width, height, resolution, originX, originY);
    System.arraycopy(logOdds, 0, g.logOdds, 0, logOdds.length);
    g.version = version + 1;
    return g;
  }

  /**
   * Paints {@code src} (a submap) into this grid: every cell of this grid under the submap gets
   * {@code scale} times the submap cell's log-odds. {@code srcToThis} maps the submap's own
   * coordinates (its world coordinates minus {@code srcOrigin}) into this grid's coordinates.
   */
  public void addSubmap(OccupancyGrid src, double srcOriginX, double srcOriginY, Pose srcToThis, float scale) {
    // Bounding box of the submap's corners in this grid.
    double minX = Double.MAX_VALUE, minY = Double.MAX_VALUE, maxX = -Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
    double[][] corners = {
      {src.originX, src.originY}, {src.originX + src.width * src.resolution, src.originY},
      {src.originX, src.originY + src.height * src.resolution},
      {src.originX + src.width * src.resolution, src.originY + src.height * src.resolution}
    };
    double c = Math.cos(srcToThis.theta), s = Math.sin(srcToThis.theta);
    for (double[] k : corners) {
      double lx = k[0] - srcOriginX, ly = k[1] - srcOriginY;
      double gx = srcToThis.x + c * lx - s * ly, gy = srcToThis.y + s * lx + c * ly;
      minX = Math.min(minX, gx);
      maxX = Math.max(maxX, gx);
      minY = Math.min(minY, gy);
      maxY = Math.max(maxY, gy);
    }
    int x0 = Math.max(0, cellX(minX)), x1 = Math.min(width - 1, cellX(maxX));
    int y0 = Math.max(0, cellY(minY)), y1 = Math.min(height - 1, cellY(maxY));
    for (int cy = y0; cy <= y1; cy++) {
      double wy = worldY(cy) - srcToThis.y;
      for (int cx = x0; cx <= x1; cx++) {
        double wx = worldX(cx) - srcToThis.x;
        // Inverse rotation back into the submap, then nearest submap cell.
        double lx = c * wx + s * wy + srcOriginX, ly = -s * wx + c * wy + srcOriginY;
        int sx = src.cellX(lx), sy = src.cellY(ly);
        if (!src.inBounds(sx, sy)) continue;
        float v = src.logOdds[sy * src.width + sx];
        if (v == 0f) continue;
        int i = cy * width + cx;
        float t = logOdds[i] + scale * v;
        logOdds[i] = t < L_MIN ? L_MIN : (t > L_MAX ? L_MAX : t);
      }
    }
    version++;
  }

  // endregion

  /** Number of cells that are known (free or wall), for the status line. */
  public int knownCells() {
    int n = 0;
    for (float v : logOdds) if (v > OCC_THRESHOLD || v < FREE_THRESHOLD) n++;
    return n;
  }

  // region Save / load ("OBMAP1" + header + int8 log-odds x 32)

  private static final int MAGIC = 0x4F424D31; // "OBM1"

  public void write(DataOutputStream out) throws IOException {
    out.writeInt(MAGIC);
    out.writeInt(width);
    out.writeInt(height);
    out.writeDouble(resolution);
    out.writeDouble(originX);
    out.writeDouble(originY);
    byte[] b = new byte[logOdds.length];
    for (int i = 0; i < b.length; i++) b[i] = (byte) Math.round(logOdds[i] * 32f);
    out.write(b);
  }

  public static OccupancyGrid read(DataInputStream in) throws IOException {
    if (in.readInt() != MAGIC) throw new IOException("Not an OpenBot map");
    int w = in.readInt();
    int h = in.readInt();
    if (w <= 0 || h <= 0 || (long) w * h > 4_000_000L) throw new IOException("Bad map size");
    OccupancyGrid g = new OccupancyGrid(w, h, in.readDouble(), in.readDouble(), in.readDouble());
    byte[] b = new byte[w * h];
    in.readFully(b);
    for (int i = 0; i < b.length; i++) g.logOdds[i] = b[i] / 32f;
    g.version++;
    return g;
  }

  /**
   * The map as a ROS map_server PGM (P5): 254 free, 0 occupied, 205 unknown, top row first. Pair it
   * with {@link #rosYaml} to load the map in Nav2 / RViz.
   */
  public byte[] toPgm() {
    String head = "P5\n# OpenBot SLAM map\n" + width + " " + height + "\n255\n";
    byte[] h = head.getBytes(java.nio.charset.StandardCharsets.US_ASCII);
    byte[] out = new byte[h.length + width * height];
    System.arraycopy(h, 0, out, 0, h.length);
    int k = h.length;
    for (int y = height - 1; y >= 0; y--) {
      for (int x = 0; x < width; x++) {
        float v = logOdds[y * width + x];
        out[k++] = (byte) (v > OCC_THRESHOLD ? 0 : v < FREE_THRESHOLD ? 254 : 205);
      }
    }
    return out;
  }

  public String rosYaml(String pgmName) {
    return "image: " + pgmName + "\nresolution: " + resolution + "\norigin: [" + originX + ", " + originY
        + ", 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
  }

  // endregion
}
