package org.openbot.app.robot.slam;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Cartographer's fast correlative scan matcher (branch and bound), used for <b>loop closure</b> and
 * for finding the car on a saved map with no hint (<b>global localization</b>).
 *
 * <p>Idea (Hess et al., ICRA 2016, section V): searching metres and tens of degrees cell by cell is
 * too slow, so the map is also stored at coarser levels where each cell holds the <i>best</i>
 * probability of the 2^h x 2^h cells it covers. A coarse score is therefore never lower than any fine
 * score under it, so whole blocks of poses can be skipped once a better answer is known. The answer
 * is exactly what a full brute-force search would find, just much faster.
 *
 * <p>Probabilities are stored as bytes, as in Cartographer's PrecomputationGrid2D. Written for this
 * project from the paper and Cartographer's defaults (Apache-2.0); same code in Swift (SlamCore).
 */
public final class FastCorrelativeMatcher {

  /** Cartographer pose_graph.lua: branch_and_bound_depth = 7. */
  public static final int DEPTH = 7;

  private final OccupancyGrid grid;
  private final int depth;
  /** levels[h]: width (W + 2^h - 1), offset 2^h - 1 so negative cell indices work. */
  private final byte[][] levels;

  private final int[] levelWidth;
  private final int[] levelHeight;

  public static final class Result {
    public final Pose pose;
    public final double score;

    Result(Pose pose, double score) {
      this.pose = pose;
      this.score = score;
    }
  }

  public FastCorrelativeMatcher(OccupancyGrid grid) {
    this(grid, DEPTH);
  }

  public FastCorrelativeMatcher(OccupancyGrid grid, int depth) {
    this.grid = grid;
    this.depth = depth;
    levels = new byte[depth][];
    levelWidth = new int[depth];
    levelHeight = new int[depth];
    float[] prob = grid.probabilities();
    int w = grid.width, h = grid.height;
    int[] base = new int[w * h];
    for (int i = 0; i < base.length; i++) base[i] = Math.round(prob[i] * 255f);
    for (int lev = 0; lev < depth; lev++) {
      int win = 1 << lev;
      int lw = w + win - 1, lh = h + win - 1;
      levelWidth[lev] = lw;
      levelHeight[lev] = lh;
      // Max over [x, x + win) along rows, then columns (sliding-window maximum).
      int[] rows = new int[lw * h];
      for (int y = 0; y < h; y++) slidingMax(base, y * w, 1, w, win, rows, y * lw, 1);
      byte[] out = new byte[lw * lh];
      int[] col = new int[lh];
      int[] src = new int[h];
      for (int x = 0; x < lw; x++) {
        for (int y = 0; y < h; y++) src[y] = rows[y * lw + x];
        slidingMax(src, 0, 1, h, win, col, 0, 1);
        for (int y = 0; y < lh; y++) out[y * lw + x] = (byte) col[y];
      }
      levels[lev] = out;
    }
  }

  /**
   * out[k] for k = 0 .. n + win - 2 is the max of in[j] over j in [k - win + 1, k] clipped to 0..n-1,
   * i.e. the max over cells [x, x + win) with x = k - (win - 1). Cells outside count as 0.
   */
  private static void slidingMax(int[] in, int inOff, int inStride, int n, int win, int[] out, int outOff, int outStride) {
    int m = n + win - 1;
    int[] dq = new int[n];
    int head = 0, tail = 0;
    for (int k = 0; k < m; k++) {
      if (k < n) {
        int v = in[inOff + k * inStride];
        while (tail > head && in[inOff + dq[tail - 1] * inStride] <= v) tail--;
        dq[tail++] = k;
      }
      while (tail > head && dq[head] < k - win + 1) head++;
      out[outOff + k * outStride] = tail > head ? in[inOff + dq[head] * inStride] : 0;
    }
  }

  private int value(int lev, int cx, int cy) {
    int off = (1 << lev) - 1;
    int x = cx + off, y = cy + off;
    if (x < 0 || y < 0 || x >= levelWidth[lev] || y >= levelHeight[lev]) return 0;
    return levels[lev][y * levelWidth[lev] + x] & 0xFF;
  }

  private static final class Candidate implements Comparable<Candidate> {
    final int angle;
    final int ox;
    final int oy;
    double score;

    Candidate(int angle, int ox, int oy) {
      this.angle = angle;
      this.ox = ox;
      this.oy = oy;
    }

    @Override
    public int compareTo(Candidate o) {
      return Double.compare(o.score, score); // best first
    }
  }

  /**
   * Searches poses within {@code linearWindow} metres and {@code angularWindow} radians of {@code
   * initial}. Returns null if nothing scores above {@code minScore} (0..1, mean probability).
   */
  public Result match(Pose initial, double[] px, double[] py, double linearWindow, double angularWindow, double minScore) {
    int n = px.length;
    if (n == 0) return null;
    double res = grid.resolution;
    double step = ScanMatcher.angularStep(res, px, py);
    int nAng = (int) Math.ceil(angularWindow / step);
    int angles = 2 * nAng + 1;
    if (angularWindow >= Math.PI - 1e-9) { // full turn: don't search the same heading twice
      angles = (int) Math.ceil(2 * Math.PI / step);
      nAng = angles / 2;
    }
    int[][] sx = new int[angles][n];
    int[][] sy = new int[angles][n];
    double[] th = new double[angles];
    for (int k = 0; k < angles; k++) {
      th[k] = initial.theta + (k - nAng) * step;
      double c = Math.cos(th[k]), s = Math.sin(th[k]);
      for (int i = 0; i < n; i++) {
        sx[k][i] = grid.cellX(initial.x + c * px[i] - s * py[i]);
        sy[k][i] = grid.cellY(initial.y + s * px[i] + c * py[i]);
      }
    }
    int w = (int) Math.ceil(linearWindow / res);
    int top = depth - 1;
    int stepTop = 1 << top;
    List<Candidate> cands = new ArrayList<>();
    for (int k = 0; k < angles; k++) {
      for (int ox = -w; ox <= w; ox += stepTop) {
        for (int oy = -w; oy <= w; oy += stepTop) cands.add(new Candidate(k, ox, oy));
      }
    }
    score(cands, top, sx, sy, n);
    Collections.sort(cands);
    double[] best = {minScore * 255};
    Candidate[] found = {null};
    search(cands, top, sx, sy, n, w, best, found);
    if (found[0] == null) return null;
    Candidate c = found[0];
    return new Result(new Pose(initial.x + c.ox * res, initial.y + c.oy * res, th[c.angle]), c.score / 255.0);
  }

  private void score(List<Candidate> cands, int lev, int[][] sx, int[][] sy, int n) {
    for (Candidate c : cands) {
      int[] xs = sx[c.angle], ys = sy[c.angle];
      long sum = 0;
      for (int i = 0; i < n; i++) sum += value(lev, xs[i] + c.ox, ys[i] + c.oy);
      c.score = (double) sum / n;
    }
  }

  private void search(List<Candidate> cands, int lev, int[][] sx, int[][] sy, int n, int w, double[] best, Candidate[] found) {
    for (Candidate c : cands) {
      if (c.score <= best[0]) return; // sorted: nothing further can beat it
      if (lev == 0) {
        best[0] = c.score;
        found[0] = c;
        return;
      }
      int half = 1 << (lev - 1);
      List<Candidate> kids = new ArrayList<>(4);
      for (int dx = 0; dx <= half; dx += half) {
        if (c.ox + dx > w) break;
        for (int dy = 0; dy <= half; dy += half) {
          if (c.oy + dy > w) break;
          kids.add(new Candidate(c.angle, c.ox + dx, c.oy + dy));
        }
      }
      score(kids, lev - 1, sx, sy, n);
      Collections.sort(kids);
      search(kids, lev - 1, sx, sy, n, w, best, found);
    }
  }
}
