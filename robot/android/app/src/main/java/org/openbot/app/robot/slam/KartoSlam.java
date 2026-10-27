package org.openbot.app.robot.slam;

import java.util.ArrayList;
import java.util.List;

/**
 * slam_toolbox's graph SLAM (Macenski and Jambrecic, "SLAM Toolbox: SLAM for the dynamic world",
 * JOSS 2021), built on Open Karto (Konolige et al., "Efficient Sparse Pose Adjustment for 2D
 * Mapping", IROS 2010). Defaults from slam_toolbox's mapper_params_online_async.yaml.
 *
 * <ul>
 *   <li>A scan becomes a graph <b>node</b> after the car moves 0.5 m or turns 0.5 rad
 *       (minimum_travel_distance / heading).
 *   <li>Between nodes each scan is matched against the <b>running buffer</b>: the last 10 node
 *       scans (scan_buffer_size) drawn into a small map. Each new node is linked to the previous one.
 *   <li><b>Loop closure</b>: old nodes within 3 m (loop_search_maximum_distance) that are not in the
 *       buffer, in a chain of at least 10 (loop_match_minimum_chain_size), are drawn into a map and
 *       searched ±4 m (loop_search_space_dimension 8) and ±20° (coarse_search_angle_offset); the match
 *       needs a fine response of 0.45. Unlike Cartographer, every scan node is optimized, and the
 *       map is redrawn from all node scans at their optimized poses.
 * </ul>
 *
 * <p>The same indoor safety checks as our Cartographer (slide test, outlier removal) stand in for
 * Karto's covariance test. Rewritten for this project (slam_toolbox is LGPL).
 */
final class KartoSlam extends BaseSlam {

  static final double MIN_TRAVEL_DISTANCE = 0.5;
  static final double MIN_TRAVEL_HEADING = 0.5;
  static final int SCAN_BUFFER_SIZE = 10;
  static final double LOOP_SEARCH_MAX_DISTANCE = 3.0;
  static final int LOOP_MIN_CHAIN = 10;
  static final double LOOP_SEARCH_WINDOW = 4.0; // half of loop_search_space_dimension
  static final double LOOP_ANGLE_WINDOW = 0.349; // coarse_search_angle_offset
  static final double LOOP_MIN_RESPONSE_FINE = 0.45;
  static final int BUFFER_CELLS = 320; // 16 m running-buffer map

  private static final class Node {
    final double[] a;
    final double[] r;
    final double[] mx;
    final double[] my;
    final int index;

    Node(Scan s, int index) {
      a = s.a;
      r = s.r;
      mx = s.mx;
      my = s.my;
      this.index = index;
    }
  }

  private final PoseGraph graph = new PoseGraph();
  private final List<Node> nodes = new ArrayList<>();
  private OccupancyGrid base; // a loaded map
  private OccupancyGrid buffer; // running buffer
  private Pose lastNodePose;
  private volatile int loops;
  private boolean chainBreak = true; // the next node starts a new chain (no link to the previous)

  KartoSlam(OccupancyGrid grid) {
    super(grid);
  }

  @Override
  public Type type() {
    return Type.SLAM_TOOLBOX;
  }

  @Override
  public int getSubmapCount() {
    return nodes.size();
  }

  @Override
  public int getLoopClosureCount() {
    return loops;
  }

  @Override
  protected void onMapReplaced() {
    graph.clear();
    nodes.clear();
    buffer = null;
    loops = 0;
    base = scans > 0 ? grid.copy() : null; // setGrid sets scans = 1 first
  }

  @Override
  protected void onPoseReset(Pose p) {
    lastNodePose = null;
    buffer = null;
    chainBreak = true;
  }

  @Override
  protected Pose step(Scan s, Pose prior, double time) {
    if (!mapping) {
      Pose best = scans > 0 ? matcher.match(grid, prior, s.mx, s.my) : prior;
      quality = SlamEngine.hitFraction(grid, best, s.px, s.py);
      return best;
    }
    Pose best = prior;
    OccupancyGrid against = buffer != null ? buffer : (base != null ? grid : null);
    if (against != null) {
      best = matcher.match(against, prior, s.mx, s.my);
      quality = SlamEngine.hitFraction(against, best, s.px, s.py);
    }
    if (lastNodePose == null || lastNodePose.distanceTo(best) >= MIN_TRAVEL_DISTANCE
        || Math.abs(Pose.wrap(best.theta - lastNodePose.theta)) >= MIN_TRAVEL_HEADING) {
      best = addNode(s, best);
    }
    return best;
  }

  private Pose addNode(Scan s, Pose p) {
    Node nd = new Node(s, nodes.size());
    int gi = graph.add(p, nodes.isEmpty() && base == null);
    if (!chainBreak && !nodes.isEmpty()) {
      Pose prev = graph.get(gi - 1);
      graph.addConstraint(new PoseGraph.Constraint(gi - 1, gi, prev.inverse().compose(p), PoseGraph.SEQ_T, PoseGraph.SEQ_R, false));
    } else if (!(nodes.isEmpty() && base == null)) {
      graph.addConstraint(new PoseGraph.Constraint(gi, -1, p, PoseGraph.PRIOR_T, PoseGraph.PRIOR_R, false));
    }
    chainBreak = false;
    nodes.add(nd);
    grid.integrate(p, s.a, s.r, SlamEngine.MAX_RANGE);
    lastNodePose = p;
    if (tryLoopClosure(nd)) {
      optimizeAndRedraw();
      p = graph.get(gi);
      lastNodePose = p;
    }
    rebuildBuffer();
    return p;
  }

  private void rebuildBuffer() {
    int last = nodes.size() - 1;
    Pose c = graph.get(last);
    double half = BUFFER_CELLS * grid.resolution / 2;
    OccupancyGrid b = new OccupancyGrid(BUFFER_CELLS, BUFFER_CELLS, grid.resolution, c.x - half, c.y - half);
    for (int i = Math.max(0, last - SCAN_BUFFER_SIZE + 1); i <= last; i++) {
      Node nd = nodes.get(i);
      b.integrate(graph.get(i), nd.a, nd.r, SlamEngine.MAX_RANGE);
    }
    buffer = b;
  }

  /** Karto's loop closure: a chain of old nodes near the new one, matched as one map. */
  private boolean tryLoopClosure(Node nd) {
    int ni = nd.index;
    Pose p = graph.get(ni);
    int nearest = -1;
    double nearestD = Double.MAX_VALUE;
    for (int i = 0; i < ni - SCAN_BUFFER_SIZE; i++) {
      double d = graph.get(i).distanceTo(p);
      if (d <= LOOP_SEARCH_MAX_DISTANCE && d < nearestD) {
        nearestD = d;
        nearest = i;
      }
    }
    if (nearest < 0) return false;
    // The chain: consecutive old nodes around the nearest one that are still within reach.
    int lo = nearest, hi = nearest;
    while (lo > 0 && graph.get(lo - 1).distanceTo(p) <= LOOP_SEARCH_MAX_DISTANCE * 2) lo--;
    while (hi < ni - SCAN_BUFFER_SIZE - 1 && graph.get(hi + 1).distanceTo(p) <= LOOP_SEARCH_MAX_DISTANCE * 2) hi++;
    if (hi - lo + 1 < LOOP_MIN_CHAIN) return false;
    double minX = Double.MAX_VALUE, minY = Double.MAX_VALUE, maxX = -Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
    for (int i = lo; i <= hi; i++) {
      Pose q = graph.get(i);
      minX = Math.min(minX, q.x);
      maxX = Math.max(maxX, q.x);
      minY = Math.min(minY, q.y);
      maxY = Math.max(maxY, q.y);
    }
    double pad = SlamEngine.MAX_RANGE;
    double res = grid.resolution;
    int w = (int) Math.ceil((maxX - minX + 2 * pad) / res), h = (int) Math.ceil((maxY - minY + 2 * pad) / res);
    OccupancyGrid chain = new OccupancyGrid(w, h, res, minX - pad, minY - pad);
    for (int i = lo; i <= hi; i++) chain.integrate(graph.get(i), nodes.get(i).a, nodes.get(i).r, SlamEngine.MAX_RANGE);
    FastCorrelativeMatcher.Result r =
        new FastCorrelativeMatcher(chain).match(p, nd.mx, nd.my, LOOP_SEARCH_WINDOW, LOOP_ANGLE_WINDOW, 0.1 + 0.8 * 0.35);
    if (r == null) return false;
    Pose refined = matcher.refine(chain, r.pose, nd.mx, nd.my, 20, 10, 1);
    double response = (ScanMatcher.meanProbability(chain, refined, nd.mx, nd.my) - 0.1) / 0.8;
    if (response < LOOP_MIN_RESPONSE_FINE || matcher.lastConstraint < SlamEngine.MIN_CONSTRAINT) return false;
    Pose ref = graph.get(nearest);
    graph.addConstraint(new PoseGraph.Constraint(nearest, ni, ref.inverse().compose(refined),
        PoseGraph.LOOP_T, PoseGraph.LOOP_R, true));
    return true;
  }

  private void optimizeAndRedraw() {
    graph.optimize();
    for (int round = 0; round < 2 && graph.removeOutliers(SlamEngine.OUTLIER_T, SlamEngine.OUTLIER_R) > 0; round++) {
      graph.optimize();
    }
    int kept = 0;
    for (PoseGraph.Constraint c : graph.getConstraints()) if (c.robust) kept++;
    loops = kept;
    OccupancyGrid old = grid;
    OccupancyGrid g = base != null ? base.copy()
        : new OccupancyGrid(old.width, old.height, old.resolution, old.originX, old.originY);
    List<double[]> t = new ArrayList<>();
    for (int i = 0; i < nodes.size(); i++) {
      Pose q = graph.get(i);
      g.integrate(q, nodes.get(i).a, nodes.get(i).r, SlamEngine.MAX_RANGE);
      t.add(new double[] {q.x, q.y});
    }
    grid = g;
    setTrail(t);
  }
}
