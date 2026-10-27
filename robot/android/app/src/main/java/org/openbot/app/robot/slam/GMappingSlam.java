package org.openbot.app.robot.slam;

import java.util.Random;

/**
 * GMapping (Grisetti, Stachniss and Burgard, "Improved Techniques for Grid Mapping with Rao-
 * Blackwellized Particle Filters", IEEE T-RO 2007; slam_gmapping): a particle filter where every
 * particle is a guess of the whole path and carries its own map.
 *
 * <p>At each update every particle moves by the measured motion plus noise (gmapping's motion model:
 * srr 0.1, srt 0.2, str 0.1, stt 0.2), is improved by scan matching against its own map (the
 * "improved proposal"), and is weighted by how well the scan fits that map (smoothed by ogain 3).
 * When the effective number of particles drops below half (resampleThreshold 0.5), good particles
 * are copied and bad ones dropped. The map shown is the best particle's.
 *
 * <p>Phone adjustments: 12 particles (gmapping uses 30: each carries a 1.4 MB map here), updates
 * every 0.5 m or 0.25 rad (gmapping 1.0 m / 0.5 rad, for bigger robots), and since the car has no
 * wheel odometry the "odometry" is scan matching against the best map between updates. Rewritten for
 * this project from the paper (gmapping is BSD).
 */
final class GMappingSlam extends BaseSlam {

  static final int PARTICLES = 12;
  static final double LINEAR_UPDATE = 0.5;
  static final double ANGULAR_UPDATE = 0.25;
  static final double SRR = 0.1;
  static final double SRT = 0.2;
  static final double STR = 0.1;
  static final double STT = 0.2;
  static final double OGAIN = 3;
  static final double RESAMPLE_THRESHOLD = 0.5;

  private static final class Particle {
    Pose pose;
    OccupancyGrid map;
    double logWeight;

    Particle(Pose pose, OccupancyGrid map) {
      this.pose = pose;
      this.map = map;
    }
  }

  private Particle[] particles = new Particle[0];
  private int best;
  private Pose odomAtUpdate;
  private boolean started;
  private final Random random = new Random(7);
  private volatile int resamples;

  GMappingSlam(OccupancyGrid grid) {
    super(grid);
  }

  @Override
  public Type type() {
    return Type.GMAPPING;
  }

  @Override
  public int getSubmapCount() {
    return particles.length;
  }

  /** How many times the particles were resampled (shown where Cartographer shows loop closures). */
  @Override
  public int getLoopClosureCount() {
    return resamples;
  }

  @Override
  protected void onMapReplaced() {
    started = false;
    particles = new Particle[0];
  }

  @Override
  protected void onPoseReset(Pose p) {
    // A jump: every particle restarts at the new pose, keeping its map.
    if (particles.length == 0) return;
    for (Particle q : particles) {
      q.pose = p;
      q.logWeight = 0;
    }
    odomAtUpdate = p;
  }

  private void start(Pose p) {
    particles = new Particle[PARTICLES];
    for (int i = 0; i < PARTICLES; i++) particles[i] = new Particle(p, i == 0 ? grid : grid.copy());
    best = 0;
    odomAtUpdate = p;
    started = true;
  }

  @Override
  protected Pose step(Scan s, Pose prior, double time) {
    if (!started) start(prior);
    OccupancyGrid bestMap = particles[best].map;
    // "Odometry": track against the best particle's map every scan.
    Pose odom = scans > 0 ? matcher.match(bestMap, prior, s.mx, s.my) : prior;
    quality = SlamEngine.hitFraction(bestMap, odom, s.px, s.py);
    if (!mapping) return odom;
    Pose delta = odomAtUpdate.inverse().compose(odom);
    boolean first = scans <= 1 && odomAtUpdate.distanceTo(odom) < 1e-9;
    if (!first && Math.hypot(delta.x, delta.y) < LINEAR_UPDATE && Math.abs(delta.theta) < ANGULAR_UPDATE) return odom;

    double d = Math.hypot(delta.x, delta.y), a = Math.abs(delta.theta);
    double sT = SRR * d + SRT * a, sR = STR * d + STT * a;
    for (Particle q : particles) {
      if (!first) {
        Pose noisy = new Pose(delta.x + random.nextGaussian() * sT, delta.y + random.nextGaussian() * sT,
            delta.theta + random.nextGaussian() * sR);
        Pose proposal = q.pose.compose(noisy);
        q.pose = matcher.refine(q.map, proposal, s.mx, s.my); // improved proposal
        q.logWeight += logLikelihood(q.map, q.pose, s.mx, s.my) / OGAIN;
      }
    }
    if (!first) resampleIfNeeded();
    for (Particle q : particles) q.map.integrate(q.pose, s.a, s.r, SlamEngine.MAX_RANGE);
    best = 0;
    for (int i = 1; i < particles.length; i++) if (particles[i].logWeight > particles[best].logWeight) best = i;
    grid = particles[best].map;
    odomAtUpdate = particles[best].pose;
    return odomAtUpdate;
  }

  private static double logLikelihood(OccupancyGrid g, Pose p, double[] px, double[] py) {
    float[] prob = g.probabilities();
    double c = Math.cos(p.theta), sn = Math.sin(p.theta), sum = 0;
    for (int i = 0; i < px.length; i++) {
      int x = g.cellX(p.x + c * px[i] - sn * py[i]), y = g.cellY(p.y + sn * px[i] + c * py[i]);
      sum += Math.log(g.inBounds(x, y) ? prob[y * g.width + x] : OccupancyGrid.P_MIN);
    }
    return sum;
  }

  /** Normalise, and low-variance resample when N_eff < threshold * N. */
  private void resampleIfNeeded() {
    int n = particles.length;
    double max = Double.NEGATIVE_INFINITY;
    for (Particle q : particles) max = Math.max(max, q.logWeight);
    double[] w = new double[n];
    double sum = 0;
    for (int i = 0; i < n; i++) {
      w[i] = Math.exp(particles[i].logWeight - max);
      sum += w[i];
    }
    double sq = 0;
    for (int i = 0; i < n; i++) {
      w[i] /= sum;
      sq += w[i] * w[i];
      particles[i].logWeight = Math.log(Math.max(w[i], 1e-300));
    }
    double neff = 1 / sq;
    if (neff >= RESAMPLE_THRESHOLD * n) return;
    Particle[] next = new Particle[n];
    boolean[] used = new boolean[n];
    double r = random.nextDouble() / n, c = w[0];
    int i = 0;
    for (int m = 0; m < n; m++) {
      double u = r + (double) m / n;
      while (u > c && i < n - 1) c += w[++i];
      Particle src = particles[i];
      // The first copy keeps the map; further copies get their own.
      Particle q = new Particle(src.pose, used[i] ? src.map.copy() : src.map);
      used[i] = true;
      next[m] = q;
    }
    particles = next;
    resamples++;
  }
}
