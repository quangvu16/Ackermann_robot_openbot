package org.openbot.app.robot.slam;

import java.util.Random;

/**
 * CoreSLAM, as in BreezySLAM's RMHC_SLAM (Steux and El Hamzaoui, "tinySLAM", ICARCV 2010): one map,
 * random-mutation hill climbing around the predicted pose for the pose whose beam ends land on the
 * most wall, then the scan goes into the map. No loop closure. The lightest engine: good for a single
 * room, it drifts on long loops. Rewritten for this project (BreezySLAM is LGPL).
 */
final class CoreSlam extends BaseSlam {

  /** RMHC search: iterations and starting step sizes (they halve as the search stalls). */
  int iterations = 250;

  double sigmaXY = 0.08;
  double sigmaTheta = Math.toRadians(4);
  private final Random random = new Random(42);
  private Pose lastMapPose;

  CoreSlam(OccupancyGrid grid) {
    super(grid);
  }

  @Override
  public Type type() {
    return Type.CORESLAM;
  }

  @Override
  protected void onPoseReset(Pose p) {
    lastMapPose = null;
  }

  @Override
  protected Pose step(Scan s, Pose prior, double time) {
    OccupancyGrid g = grid;
    Pose best = prior;
    if (scans > 0) {
      best = rmhc(g, prior, s.px, s.py);
      quality = SlamEngine.hitFraction(g, best, s.px, s.py);
    }
    if (mapping) {
      boolean moved = lastMapPose == null || lastMapPose.distanceTo(best) > 0.04
          || Math.abs(Pose.wrap(best.theta - lastMapPose.theta)) > Math.toRadians(2);
      if (moved) {
        g.integrate(best, s.a, s.r, SlamEngine.MAX_RANGE);
        lastMapPose = best;
      }
    }
    return best;
  }

  private Pose rmhc(OccupancyGrid g, Pose prior, double[] px, double[] py) {
    double bestScore = score(g, prior, px, py);
    Pose best = prior;
    double sxy = sigmaXY, sth = sigmaTheta;
    int since = 0;
    for (int i = 0; i < iterations; i++) {
      Pose cand = new Pose(best.x + random.nextGaussian() * sxy, best.y + random.nextGaussian() * sxy,
          best.theta + random.nextGaussian() * sth);
      double sc = score(g, cand, px, py);
      if (sc > bestScore) {
        bestScore = sc;
        best = cand;
        since = 0;
      } else if (++since > 25) {
        sxy *= 0.5;
        sth *= 0.5;
        since = 0;
        if (sxy < g.resolution * 0.1) break;
      }
    }
    return best;
  }

  private static double score(OccupancyGrid g, Pose p, double[] px, double[] py) {
    double c = Math.cos(p.theta), sn = Math.sin(p.theta), sum = 0;
    for (int i = 0; i < px.length; i++) {
      sum += g.hitScore(g.cellX(p.x + c * px[i] - sn * py[i]), g.cellY(p.y + sn * px[i] + c * py[i]));
    }
    return sum;
  }
}
