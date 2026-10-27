package org.openbot.app.robot.slam;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;

/**
 * 2-D LiDAR SLAM for the car, built the way Google's Cartographer (cartographer_ros) builds it:
 *
 * <ul>
 *   <li><b>Local SLAM</b>: each scan is matched against the current <i>submap</i> (a small local map
 *       of the last ~90 scans) with the real-time correlative + "Ceres" scan matcher ({@link
 *       ScanMatcher}). A <b>motion filter</b> only inserts a scan when the car moved 0.2 m, turned
 *       1° or 5 s passed. Two submaps are active at a time; when a new one starts, the oldest is
 *       finished and never changes again.
 *   <li><b>Global SLAM</b>: finished submaps are searched for each new scan, and each finished
 *       submap is searched for old scans, with the branch-and-bound matcher ({@link
 *       FastCorrelativeMatcher}). A good match is a <b>loop closure</b>: a constraint in the {@link
 *       PoseGraph}. Optimizing the graph moves whole submaps so the map lines up again.
 *   <li><b>Global localization</b>: on a saved map, the car can find itself with no hint (the same
 *       branch-and-bound search over the whole map and every heading), and does it again by itself
 *       when tracking is lost.
 * </ul>
 *
 * <p>Simplification: nodes are rigid inside their submap and only submaps are optimized (Cartographer
 * optimizes every node too). The phone's gyroscope gives the turn between scans, as the IMU does in
 * Cartographer. Defaults are Cartographer's trajectory_builder_2d.lua / pose_graph.lua unless noted.
 * Pure Java, unit tested; the same engine is in Swift (SlamCore.swift).
 */
public final class SlamEngine {

  /** Beams beyond this are treated as "nothing seen". LD14: 8 m. */
  public static final double MAX_RANGE = 8.0;
  /** Closer than this is the car itself. */
  public static final double MIN_RANGE = 0.15;

  // Cartographer trajectory_builder_2d.lua
  /** submaps.num_range_data: scans per submap. */
  public static final int NUM_RANGE_DATA = 90;

  static final double MOTION_MAX_DISTANCE = 0.2;
  static final double MOTION_MAX_ANGLE = Math.toRadians(1);
  static final double MOTION_MAX_TIME = 5;
  /** Submaps are 14 m squares at 5 cm (Cartographer's grow; ours start a new one 2 m from the last). */
  static final int SUBMAP_CELLS = 280;

  static final double NEW_SUBMAP_DISTANCE = 2.0;

  // Cartographer pose_graph.lua
  static final double SAMPLING_RATIO = 0.3;
  static final double MAX_CONSTRAINT_DISTANCE = 15;
  static final double MIN_SCORE = 0.55;
  static final double GLOBAL_LOCALIZATION_MIN_SCORE = 0.6;
  static final double LOOP_LINEAR_WINDOW = 7;
  static final double LOOP_ANGULAR_WINDOW = Math.toRadians(30);
  /**
   * Not in Cartographer: indoors, corridors and rooms repeat every few metres, so the 7 m search
   * finds look-alike places. The window here grows with the distance driven since the submap was
   * made (drift grows with it): 0.5 m + 10 %, at most 7 m.
   */
  static double loopWindow(double drivenSince) {
    return Math.min(LOOP_LINEAR_WINDOW, 0.5 + 0.1 * Math.abs(drivenSince));
  }

  /** A loop closure the optimized graph still disagrees with by more than this is dropped as wrong. */
  static final double OUTLIER_T = 0.3;

  static final double OUTLIER_R = Math.toRadians(5);

  /** Loop matches that pin position down less than this (see ScanMatcher.lastConstraint) are skipped. */
  static final double MIN_CONSTRAINT = 0.3;
  /** Cartographer optimizes every 90 nodes; a small flat has fewer, so 30 here. */
  int optimizeEveryNNodes = 30;
  /** Loop-closure searches per scan (spreads the work; Cartographer uses background threads). */
  int loopSearchesPerScan = 2;

  static final class Submap {
    final OccupancyGrid grid;
    final double ox; // origin in its session's local frame (submaps are axis-aligned there)
    final double oy;
    final int session;
    final boolean frozen;
    double driven; // distance driven when it was made
    int inserts;
    boolean finished;
    int graphIndex;

    Submap(OccupancyGrid grid, double ox, double oy, int session, boolean frozen) {
      this.grid = grid;
      this.ox = ox;
      this.oy = oy;
      this.session = session;
      this.frozen = frozen;
    }
  }

  static final class Node {
    final Pose local;
    final int submap; // newest submap it went into
    final int[] inSubmaps;
    final double[] lx; // loop-closure points, robot frame
    final double[] ly;
    double driven;

    Node(Pose local, int submap, int[] inSubmaps, double[] lx, double[] ly) {
      this.local = local;
      this.submap = submap;
      this.inSubmaps = inSubmaps;
      this.lx = lx;
      this.ly = ly;
    }
  }

  private volatile OccupancyGrid grid; // the global map everyone reads
  private volatile Pose pose = Pose.ZERO; // global
  private Pose localPose = Pose.ZERO;
  private Pose lastInsertPose;
  private double lastInsertTime = -1e9;
  private double lastStepX;
  private double lastStepY;
  private double driven;
  private double lastImuYaw = Double.NaN;
  private boolean mapping = true;
  private volatile int scans;
  private volatile double lastQuality;
  private int lowQualityScans;
  private final List<double[]> trail = new ArrayList<>();
  private final Random random = new Random(42);

  final ScanMatcher matcher = new ScanMatcher();
  private final PoseGraph graph = new PoseGraph();
  private final List<Submap> submaps = new ArrayList<>();
  private final List<Submap> active = new ArrayList<>();
  private final List<Node> nodes = new ArrayList<>();
  private final ArrayDeque<int[]> loopQueue = new ArrayDeque<>();
  private Submap base; // a loaded map, frozen
  private int session;
  private int nodesSinceOptimize;
  private int newLoops;
  private volatile int loopClosures;
  private volatile boolean relocalizeRequested;
  private volatile int relocalizeSeq;
  private volatile boolean relocalizeFound;
  private FastCorrelativeMatcher globalMatcher;
  private OccupancyGrid globalMatcherGrid;
  private int globalMatcherVersion;
  /** Branch-and-bound tables for the most recently used finished submaps. */
  private final Map<Submap, FastCorrelativeMatcher> matcherCache =
      new LinkedHashMap<Submap, FastCorrelativeMatcher>(8, 0.75f, true) {
        @Override
        protected boolean removeEldestEntry(Map.Entry<Submap, FastCorrelativeMatcher> e) {
          return size() > 6;
        }
      };

  public SlamEngine(OccupancyGrid grid) {
    this.grid = grid;
  }

  public OccupancyGrid getGrid() {
    return grid;
  }

  public Pose getPose() {
    return pose;
  }

  public int getScanCount() {
    return scans;
  }

  /** 0..1: share of beam ends that landed on a known wall in the last match. */
  public double getMatchQuality() {
    return lastQuality;
  }

  public int getSubmapCount() {
    return submaps.size();
  }

  public int getLoopClosureCount() {
    return loopClosures;
  }

  /** A copy of the trail driven (safe to read from another thread). */
  public List<double[]> getTrail() {
    synchronized (trail) {
      return new ArrayList<>(trail);
    }
  }

  public boolean isMapping() {
    return mapping;
  }

  /** Mapping builds submaps; localization only matches against the map (Cartographer pure localization). */
  public void setMapping(boolean m) {
    if (m == mapping) return;
    mapping = m;
    if (m) startSession();
    else finishActive();
  }

  /** Find the car on the map from the next scan, with no hint (branch and bound over the whole map). */
  public void requestGlobalLocalization() {
    relocalizeRequested = true;
  }

  /** Bumped after each global localization attempt; see {@link #wasRelocalizeFound()}. */
  public int getRelocalizeSeq() {
    return relocalizeSeq;
  }

  public boolean wasRelocalizeFound() {
    return relocalizeFound;
  }

  /** Start again: empty map, pose at the origin. */
  public void reset() {
    OccupancyGrid g = grid;
    clearGraph();
    grid = new OccupancyGrid(g.width, g.height, g.resolution, g.originX, g.originY);
    resetPose(Pose.ZERO);
    scans = 0;
    synchronized (trail) {
      trail.clear();
    }
  }

  /** Use a loaded map: it becomes a frozen base the car localizes on (and new submaps attach to). */
  public void setGrid(OccupancyGrid loaded) {
    clearGraph();
    base = new Submap(loaded, 0, 0, -1, true);
    base.finished = true;
    base.graphIndex = graph.add(Pose.ZERO, true);
    grid = loaded.copy();
    resetPose(Pose.ZERO);
    scans = 1; // a loaded map counts as "has walls": match from the first scan
    synchronized (trail) {
      trail.clear();
    }
  }

  /** Put the car somewhere on the map (a long press on the map). */
  public void resetPose(Pose p) {
    pose = p;
    localPose = p;
    lastStepX = 0;
    lastStepY = 0;
    lastImuYaw = Double.NaN;
    lowQualityScans = 0;
    if (mapping) startSession();
  }

  private void clearGraph() {
    graph.clear();
    submaps.clear();
    active.clear();
    nodes.clear();
    loopQueue.clear();
    matcherCache.clear();
    base = null;
    nodesSinceOptimize = 0;
    newLoops = 0;
    loopClosures = 0;
    globalMatcher = null;
  }

  /** A new local frame starting at the current global pose (after a jump, a mode change or a reset). */
  private void startSession() {
    finishActive();
    session++;
    localPose = pose;
    lastInsertPose = null;
  }

  private void finishActive() {
    for (Submap s : active) finish(s);
    active.clear();
  }

  // region Update

  public Pose update(double[] angles, double[] ranges, double imuYaw) {
    return update(angles, ranges, imuYaw, scans * 0.1);
  }

  /**
   * One LiDAR revolution.
   *
   * @param angles beam angles, radians, + left of the nose
   * @param ranges metres (beams outside MIN_RANGE..MAX_RANGE are skipped)
   * @param imuYaw the phone's integrated yaw in radians (+ counter-clockwise), NaN if unknown
   * @param time seconds (any origin), for the motion filter
   */
  public Pose update(double[] angles, double[] ranges, double imuYaw, double time) {
    int n = 0;
    for (double r : ranges) if (r >= MIN_RANGE && r < MAX_RANGE) n++;
    double[] a = new double[n];
    double[] r = new double[n];
    double[] px = new double[n];
    double[] py = new double[n];
    int k = 0;
    for (int i = 0; i < ranges.length; i++) {
      if (ranges[i] >= MIN_RANGE && ranges[i] < MAX_RANGE) {
        a[k] = angles[i];
        r[k] = ranges[i];
        px[k] = r[k] * Math.cos(a[k]);
        py[k] = r[k] * Math.sin(a[k]);
        k++;
      }
    }
    // Cartographer adaptive_voxel_filter (local matching) and loop_closure_adaptive_voxel_filter.
    double[][] mp = adaptiveVoxel(px, py, 0.5, 200);
    double[][] lp = adaptiveVoxel(px, py, 0.9, 100);

    double dTheta = 0;
    if (!Double.isNaN(imuYaw)) {
      if (!Double.isNaN(lastImuYaw)) dTheta = Pose.wrap(imuYaw - lastImuYaw);
      lastImuYaw = imuYaw;
    }

    if (relocalizeRequested && n >= 20) {
      relocalizeRequested = false;
      globalLocalize(mp[0], mp[1]);
    }

    if (mapping) {
      Pose prior = new Pose(localPose.x + lastStepX * 0.6, localPose.y + lastStepY * 0.6, localPose.theta + dTheta);
      Pose best = prior;
      if (!active.isEmpty() && n >= 20) {
        Submap ms = active.get(0); // Cartographer matches against the oldest active submap
        best = matcher.match(ms.grid, prior, mp[0], mp[1]);
        lastQuality = hitFraction(ms.grid, best, px, py);
      }
      lastStepX = best.x - localPose.x;
      lastStepY = best.y - localPose.y;
      driven += Math.hypot(lastStepX, lastStepY);
      localPose = best;
      pose = localToGlobal().compose(localPose);
      if (n >= 20 && motionFilterPasses(time)) insert(a, r, mp[0], mp[1], time);
      for (int i = 0; i < loopSearchesPerScan && !loopQueue.isEmpty(); i++) {
        int[] job = loopQueue.pollFirst();
        tryLoopClosure(nodes.get(job[0]), job[1] < 0 ? base : submaps.get(job[1]));
      }
      if (newLoops > 0 && nodesSinceOptimize >= optimizeEveryNNodes) optimize();
    } else {
      Pose prior = new Pose(pose.x + lastStepX * 0.6, pose.y + lastStepY * 0.6, pose.theta + dTheta);
      Pose best = prior;
      OccupancyGrid g = grid;
      if (scans > 0 && n >= 20) {
        best = matcher.match(g, prior, mp[0], mp[1]);
        lastQuality = hitFraction(g, best, px, py);
        // Lost? Cartographer's pure localization keeps searching the whole map; here: after 3 s.
        lowQualityScans = lastQuality < 0.3 ? lowQualityScans + 1 : 0;
        if (lowQualityScans >= 30) {
          lowQualityScans = 0;
          relocalizeRequested = true;
        }
      }
      lastStepX = best.x - pose.x;
      lastStepY = best.y - pose.y;
      pose = best;
      localPose = best;
    }
    scans++;
    appendTrail(pose);
    return pose;
  }

  /** Cartographer's motion filter: insert after 0.2 m, 1° or 5 s. */
  private boolean motionFilterPasses(double time) {
    if (lastInsertPose == null
        || lastInsertPose.distanceTo(localPose) > MOTION_MAX_DISTANCE
        || Math.abs(Pose.wrap(localPose.theta - lastInsertPose.theta)) > MOTION_MAX_ANGLE
        || time - lastInsertTime > MOTION_MAX_TIME) {
      lastInsertPose = localPose;
      lastInsertTime = time;
      return true;
    }
    return false;
  }

  /** Global transform of the current session's local frame (from its newest submap). */
  private Pose localToGlobal() {
    Submap last = null;
    for (int i = submaps.size() - 1; i >= 0; i--) {
      Submap s = submaps.get(i);
      if (s.session == session && !s.frozen) {
        last = s;
        break;
      }
    }
    if (last == null) return Pose.ZERO; // a new session starts with local == global
    return graph.get(last.graphIndex).compose(new Pose(last.ox, last.oy, 0).inverse());
  }

  private void insert(double[] a, double[] r, double[] lx, double[] ly, double time) {
    Submap newest = active.isEmpty() ? null : active.get(active.size() - 1);
    if (newest == null || newest.inserts >= NUM_RANGE_DATA
        || Math.hypot(localPose.x - newest.ox, localPose.y - newest.oy) > NEW_SUBMAP_DISTANCE) {
      addSubmap(newest);
    }
    int[] in = new int[active.size()];
    for (int i = 0; i < active.size(); i++) {
      Submap s = active.get(i);
      s.grid.integrate(localPose, a, r, MAX_RANGE);
      s.inserts++;
      in[i] = submaps.indexOf(s);
    }
    grid.integrate(pose, a, r, MAX_RANGE); // live global map; rebuilt from submaps after optimizing
    Node node = new Node(localPose, in[in.length - 1], in, lx, ly);
    node.driven = driven;
    nodes.add(node);
    int ni = nodes.size() - 1;
    nodesSinceOptimize++;
    // Search the finished submaps for this scan (Cartographer: ComputeConstraintsForNode).
    for (int si = 0; si < submaps.size(); si++) {
      Submap s = submaps.get(si);
      if (s.finished && isNear(node, s) && random.nextDouble() < SAMPLING_RATIO) enqueue(ni, si);
    }
    if (base != null && random.nextDouble() < SAMPLING_RATIO) enqueue(ni, -1); // the loaded map
  }

  private void addSubmap(Submap previous) {
    int cells = SUBMAP_CELLS;
    double res = grid.resolution;
    double half = cells * res / 2;
    OccupancyGrid g = new OccupancyGrid(cells, cells, res, localPose.x - half, localPose.y - half);
    Submap s = new Submap(g, localPose.x, localPose.y, session, false);
    s.driven = driven;
    Pose global = localToGlobal().compose(new Pose(s.ox, s.oy, 0));
    boolean first = previous == null || previous.session != session;
    boolean fix = first && base == null && submaps.isEmpty();
    s.graphIndex = graph.add(global, fix);
    if (!first) {
      graph.addConstraint(new PoseGraph.Constraint(previous.graphIndex, s.graphIndex,
          new Pose(s.ox - previous.ox, s.oy - previous.oy, 0), PoseGraph.SEQ_T, PoseGraph.SEQ_R, false));
    } else if (!fix) {
      graph.addConstraint(new PoseGraph.Constraint(s.graphIndex, -1, global, PoseGraph.PRIOR_T, PoseGraph.PRIOR_R, false));
    }
    submaps.add(s);
    active.add(s);
    if (active.size() > 2) finish(active.remove(0));
  }

  /** A finished submap never changes; search it for older scans (Cartographer: ComputeConstraintsForSubmap). */
  private void finish(Submap s) {
    if (s.finished) return;
    s.finished = true;
    int si = submaps.indexOf(s);
    for (int ni = 0; ni < nodes.size(); ni++) {
      Node nd = nodes.get(ni);
      boolean inside = false;
      for (int k : nd.inSubmaps) inside |= k == si;
      if (!inside && isNear(nd, s) && random.nextDouble() < SAMPLING_RATIO) enqueue(ni, si);
    }
  }

  private void enqueue(int node, int submap) {
    loopQueue.addLast(new int[] {node, submap});
    while (loopQueue.size() > 400) loopQueue.pollFirst();
  }

  private Pose nodeGlobal(Node nd) {
    Submap a = submaps.get(nd.submap);
    return graph.get(a.graphIndex).compose(new Pose(nd.local.x - a.ox, nd.local.y - a.oy, nd.local.theta));
  }

  private boolean isNear(Node nd, Submap s) {
    if (s.frozen) return true;
    Pose g = nodeGlobal(nd);
    Pose sg = graph.get(s.graphIndex);
    return Math.hypot(g.x - sg.x, g.y - sg.y) <= MAX_CONSTRAINT_DISTANCE;
  }

  private FastCorrelativeMatcher matcherFor(Submap s) {
    FastCorrelativeMatcher m = matcherCache.get(s);
    if (m == null) {
      m = new FastCorrelativeMatcher(s.grid);
      matcherCache.put(s, m);
    }
    return m;
  }

  /** Branch-and-bound match of one old scan in one finished submap; adds a loop closure if good. */
  boolean tryLoopClosure(Node nd, Submap s) {
    if (nd.lx.length < 20) return false;
    Pose inSub = graph.get(s.graphIndex).inverse().compose(nodeGlobal(nd)); // node in submap frame
    Pose initial = new Pose(inSub.x + s.ox, inSub.y + s.oy, inSub.theta); // in the submap grid
    double window = s.frozen ? LOOP_LINEAR_WINDOW : loopWindow(nd.driven - s.driven);
    FastCorrelativeMatcher.Result res = matcherFor(s).match(initial, nd.lx, nd.ly, window, LOOP_ANGULAR_WINDOW, MIN_SCORE);
    if (res == null) return false;
    // Cartographer's constraint_builder ceres_scan_matcher: occupied 20, translation 10, rotation 1.
    Pose refined = matcher.refine(s.grid, res.pose, nd.lx, nd.ly, 20, 10, 1);
    if (matcher.lastConstraint < MIN_CONSTRAINT) return false; // could slide (bare corridor): unsafe
    Pose measured = new Pose(refined.x - s.ox, refined.y - s.oy, refined.theta);
    Submap a = submaps.get(nd.submap);
    Pose relAN = new Pose(nd.local.x - a.ox, nd.local.y - a.oy, nd.local.theta);
    graph.addConstraint(new PoseGraph.Constraint(s.graphIndex, a.graphIndex, measured.compose(relAN.inverse()),
        PoseGraph.LOOP_T, PoseGraph.LOOP_R, true));
    loopClosures++;
    newLoops++;
    return true;
  }

  /** Solve the pose graph, then rebuild the global map from the submaps at their new poses. */
  void optimize() {
    graph.optimize();
    for (int round = 0; round < 2 && graph.removeOutliers(OUTLIER_T, OUTLIER_R) > 0; round++) graph.optimize();
    int kept = 0;
    for (PoseGraph.Constraint c : graph.getConstraints()) if (c.robust) kept++;
    loopClosures = kept;
    newLoops = 0;
    nodesSinceOptimize = 0;
    pose = localToGlobal().compose(localPose);
    OccupancyGrid old = grid;
    OccupancyGrid g = base != null
        ? base.grid.copy()
        : new OccupancyGrid(old.width, old.height, old.resolution, old.originX, old.originY);
    for (Submap s : submaps) {
      if (s.frozen) continue;
      // Each scan is in two submaps, so each counts half (Cartographer blends submaps when painting).
      g.addSubmap(s.grid, s.ox, s.oy, graph.get(s.graphIndex), 0.5f);
    }
    grid = g;
    synchronized (trail) {
      trail.clear();
      for (Node nd : nodes) {
        Pose p = nodeGlobal(nd);
        trail.add(new double[] {p.x, p.y});
      }
      while (trail.size() > 5000) trail.remove(0);
    }
  }

  /** Branch and bound over the whole map and every heading (Cartographer: MatchFullSubmap). */
  boolean globalLocalize(double[] px, double[] py) {
    OccupancyGrid g = base != null ? base.grid : grid;
    if (globalMatcher == null || globalMatcherGrid != g || globalMatcherVersion != g.getVersion()) {
      globalMatcher = new FastCorrelativeMatcher(g);
      globalMatcherGrid = g;
      globalMatcherVersion = g.getVersion();
    }
    double cx = g.originX + g.width * g.resolution / 2, cy = g.originY + g.height * g.resolution / 2;
    double window = Math.max(g.width, g.height) * g.resolution / 2;
    FastCorrelativeMatcher.Result res =
        globalMatcher.match(new Pose(cx, cy, 0), px, py, window, Math.PI, GLOBAL_LOCALIZATION_MIN_SCORE);
    relocalizeFound = res != null;
    if (res != null) {
      Pose refined = matcher.refine(g, res.pose, px, py, 20, 10, 1);
      pose = refined;
      localPose = refined;
      lastStepX = 0;
      lastStepY = 0;
      lowQualityScans = 0;
      if (mapping) startSession();
    }
    relocalizeSeq++;
    return relocalizeFound;
  }

  // endregion

  private void appendTrail(Pose p) {
    synchronized (trail) {
      if (trail.isEmpty()
          || Math.hypot(trail.get(trail.size() - 1)[0] - p.x, trail.get(trail.size() - 1)[1] - p.y) > 0.05) {
        trail.add(new double[] {p.x, p.y});
        if (trail.size() > 5000) trail.remove(0);
      }
    }
  }

  /**
   * Cartographer's adaptive voxel filter: the largest voxel (up to {@code maxLength}) that still
   * leaves {@code minPoints} points, so far walls aren't thinned more than near ones need to be.
   */
  static double[][] adaptiveVoxel(double[] px, double[] py, double maxLength, int minPoints) {
    if (px.length <= minPoints) return new double[][] {px, py};
    double[][] f = voxel(px, py, maxLength);
    if (f[0].length >= minPoints) return f;
    double high = maxLength;
    double low = maxLength / 2;
    f = voxel(px, py, low);
    while (f[0].length < minPoints && low > 0.01) {
      high = low;
      low /= 2;
      f = voxel(px, py, low);
    }
    double[][] best = f;
    for (int i = 0; i < 5; i++) { // binary search between low (enough points) and high (too few)
      double mid = (low + high) / 2;
      double[][] m = voxel(px, py, mid);
      if (m[0].length >= minPoints) {
        low = mid;
        best = m;
      } else {
        high = mid;
      }
    }
    return best;
  }

  static double[][] voxel(double[] px, double[] py, double size) {
    Set<Long> seen = new HashSet<>();
    double[] ox = new double[px.length];
    double[] oy = new double[px.length];
    int n = 0;
    for (int i = 0; i < px.length; i++) {
      long kx = (long) Math.floor(px[i] / size), ky = (long) Math.floor(py[i] / size);
      if (seen.add((kx << 32) ^ (ky & 0xFFFFFFFFL))) {
        ox[n] = px[i];
        oy[n] = py[i];
        n++;
      }
    }
    return new double[][] {java.util.Arrays.copyOf(ox, n), java.util.Arrays.copyOf(oy, n)};
  }

  /** Share of beam ends that land on (or next to) a wall cell. */
  static double hitFraction(OccupancyGrid g, Pose p, double[] px, double[] py) {
    int hits = 0;
    double c = Math.cos(p.theta);
    double sn = Math.sin(p.theta);
    for (int i = 0; i < px.length; i++) {
      int cx = g.cellX(p.x + c * px[i] - sn * py[i]);
      int cy = g.cellY(p.y + sn * px[i] + c * py[i]);
      if (g.isOccupied(cx, cy) || g.isOccupied(cx + 1, cy) || g.isOccupied(cx - 1, cy)
          || g.isOccupied(cx, cy + 1) || g.isOccupied(cx, cy - 1)) hits++;
    }
    return px.length == 0 ? 0 : (double) hits / px.length;
  }
}
