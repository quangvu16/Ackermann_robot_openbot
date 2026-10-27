package org.openbot.app.robot.slam;

import java.util.List;
import java.util.Random;

/**
 * The local controllers the user can choose besides Regulated Pure Pursuit (which lives in {@link
 * Navigator}). Both try many short drives with a bicycle model of this car and pick by scores, the
 * way Nav2 does. Throttle is a fraction of the speed mode (assumed 0.6 m/s at full throttle for the
 * model), steer -1..1 with + = nose turns left whichever way the car drives (the firmware flips the
 * servo in reverse).
 *
 * <ul>
 *   <li><b>DWB</b> (Nav2 dwb_controller, the Dynamic Window Approach of Fox, Burgard and Thrun, IEEE
 *       RAM 1997): a grid of (throttle, steer) pairs, each driven for sim_time 1.7 s, scored by the
 *       critics BaseObstacle 0.02, PathDist 32, PathAlign 32, GoalDist 24, GoalAlign 24 (Nav2's
 *       example), plus Oscillation. Lowest total wins.
 *   <li><b>MPPI</b> (Nav2 nav2_mppi_controller; Williams et al., "Information Theoretic MPC", ICRA
 *       2017): 300 noisy versions (Nav2: 1000) of the last plan, 56 steps of 0.1 s, scored by
 *       Constraint 4.0, Cost 3.81, Goal 5.0, PathAlign 14.0, PathFollow 10.0 (Nav2 5.0), PathAngle 2.2,
 *       PreferForward 5.0 critics, then averaged with weights exp(-cost / temperature 0.3). Smooth,
 *       and it chooses to reverse by itself when that is cheaper.
 * </ul>
 *
 * Rewritten for this project from the papers and Nav2's documentation.
 */
final class Controllers {

  private Controllers() {}

  static final double V_MAX = 0.6; // m/s at throttle 1 (model)

  /** One step of the car model: returns {x, y, theta}. */
  static double[] simulate(double x, double y, double th, double throttle, double steer, Navigator.Params p, double dt) {
    double v = throttle * V_MAX;
    double delta = steer * Math.toRadians(p.maxSteerDeg);
    double nth = th + Math.abs(v) / p.wheelbase * Math.tan(delta) * dt;
    return new double[] {x + v * Math.cos(nth) * dt, y + v * Math.sin(nth) * dt, nth};
  }

  /** Nearest path index to (x, y), searching all points. */
  static int nearest(List<double[]> path, double x, double y) {
    int best = 0;
    double bd = Double.MAX_VALUE;
    for (int i = 0; i < path.size(); i++) {
      double d = Math.hypot(path.get(i)[0] - x, path.get(i)[1] - y);
      if (d < bd) {
        bd = d;
        best = i;
      }
    }
    return best;
  }

  /** Distance to the path points lo..hi (a window keeps MPPI cheap). */
  static double distToPath(List<double[]> path, double x, double y, int lo, int hi) {
    double bd = Double.MAX_VALUE;
    for (int i = Math.max(0, lo); i <= Math.min(path.size() - 1, hi); i++) {
      bd = Math.min(bd, Math.hypot(path.get(i)[0] - x, path.get(i)[1] - y));
    }
    return bd;
  }

  static double distToPath(List<double[]> path, double x, double y) {
    double[] q = path.get(nearest(path, x, y));
    return Math.hypot(q[0] - x, q[1] - y);
  }

  /** Path length from index i to the end. */
  static double remaining(List<double[]> path, int i) {
    double s = 0;
    for (int k = i; k < path.size() - 1; k++) {
      s += Math.hypot(path.get(k + 1)[0] - path.get(k)[0], path.get(k + 1)[1] - path.get(k)[1]);
    }
    return s;
  }

  // region DWB

  static final double DWB_SIM_TIME = 1.7;
  static final double BASE_OBSTACLE = 0.02;
  static final double PATH_DIST = 32;
  static final double PATH_ALIGN = 32;
  static final double GOAL_DIST = 24;
  static final double GOAL_ALIGN = 24;
  static final double OSCILLATION = 5;
  static final double FORWARD_POINT = 0.1; // forward_point_distance

  static final class Dwb {
    private double lastThrottle;

    double[] step(Pose pose, List<double[]> path, Costmap m, Navigator.Params p) {
      double[] throttles = {-0.5, -0.25, 0.25, 0.5, 0.75, 1.0};
      int steers = 11;
      double best = Double.MAX_VALUE;
      double[] cmd = {0, 0};
      double res = m.res;
      for (double tf : throttles) {
        double t = tf * p.speed;
        for (int k = 0; k < steers; k++) {
          double s = -1 + 2.0 * k / (steers - 1);
          double x = pose.x, y = pose.y, th = pose.theta, obstacle = 0;
          boolean hit = false;
          for (double tt = 0; tt < DWB_SIM_TIME; tt += 0.1) {
            double[] q = simulate(x, y, th, t, s, p, 0.1);
            x = q[0];
            y = q[1];
            th = q[2];
            float c = m.at(x, y);
            if (c < 0) {
              hit = true;
              break;
            }
            obstacle = Math.max(obstacle, (c - 1) / 8.0 * 252); // Nav2 BaseObstacle: footprint cost 0..252
          }
          if (hit) continue;
          double fx = x + FORWARD_POINT * Math.cos(th) * Math.signum(t), fy = y + FORWARD_POINT * Math.sin(th) * Math.signum(t);
          int end = nearest(path, x, y);
          int fend = nearest(path, fx, fy);
          double score = BASE_OBSTACLE * obstacle
              + PATH_DIST * distToPath(path, x, y) / res
              + PATH_ALIGN * distToPath(path, fx, fy) / res
              + GOAL_DIST * remaining(path, end) / res
              + GOAL_ALIGN * remaining(path, fend) / res
              + (Math.signum(t) != Math.signum(lastThrottle) && lastThrottle != 0 ? OSCILLATION * 10 : 0);
          if (score < best) {
            best = score;
            cmd = new double[] {t, s};
          }
        }
      }
      lastThrottle = cmd[0];
      return cmd;
    }
  }

  // endregion

  // region MPPI

  static final int BATCH = 300;
  static final int STEPS = 56; // time_steps (2.8 s)
  static final double DT = 0.1; // Nav2 model_dt 0.05 s for 0.5 m/s robots; this car is slower, so a longer look ahead
  static final double V_STD = 0.4; // of the speed: Nav2's vx_std 0.2 m/s over vx_max 0.5 m/s
  static final double STEER_STD = 0.3;
  static final double TEMPERATURE = 0.3;
  static final double W_CONSTRAINT = 4.0;
  static final double W_COST = 3.81;
  static final double W_GOAL = 5.0;
  static final double W_PATH_ALIGN = 14.0;
  static final double W_PATH_FOLLOW = 10.0; // Nav2 5.0: doubled so this slow car keeps moving
  static final double W_PATH_ANGLE = 2.2;
  static final double W_PREFER_FORWARD = 5.0;
  static final double COLLISION_COST = 1e6;

  static final class Mppi {
    private final double[] uv = new double[STEPS];
    private final double[] us = new double[STEPS];
    private final Random random = new Random(3);

    void reset() {
      java.util.Arrays.fill(uv, 0);
      java.util.Arrays.fill(us, 0);
    }

    double[] step(Pose pose, List<double[]> path, double[] goal, Costmap m, Navigator.Params p) {
      double vMax = p.speed, vMin = -0.6 * p.speed;
      double[][] ev = new double[BATCH][STEPS];
      double[][] es = new double[BATCH][STEPS];
      double[] cost = new double[BATCH];
      int near = nearest(path, pose.x, pose.y);
      // PathFollow: a point ahead on the path about as far as the horizon reaches (Nav2 uses
      // offset_from_furthest; this car is slow, so the point is set by distance).
      int ahead = (int) Math.ceil(p.speed * V_MAX * STEPS * DT / 0.1);
      int follow = Math.min(path.size() - 1, near + Math.max(6, ahead));
      double[] fp = path.get(follow);
      for (int b = 0; b < BATCH; b++) {
        double x = pose.x, y = pose.y, th = pose.theta, c = 0, align = 0;
        double nv = 0, ns = 0;
        for (int t = 0; t < STEPS; t++) {
          // Smooth noise (Nav2 smooths its samples too); the first sample is the plan itself.
          if (b != 0) {
            nv = 0.8 * nv + 0.6 * random.nextGaussian() * V_STD * p.speed;
            ns = 0.8 * ns + 0.6 * random.nextGaussian() * STEER_STD;
          }
          ev[b][t] = nv;
          es[b][t] = ns;
          double v = uv[t] + nv, s = us[t] + ns;
          if (v > vMax) c += W_CONSTRAINT * (v - vMax) * 10;
          if (v < vMin) c += W_CONSTRAINT * (vMin - v) * 10;
          v = Navigator.clamp(v, vMin, vMax);
          s = Navigator.clamp(s, -1, 1);
          double[] q = simulate(x, y, th, v, s, p, DT);
          x = q[0];
          y = q[1];
          th = q[2];
          double nc = m.normalized(x, y);
          if (m.at(x, y) < 0) c += COLLISION_COST;
          else c += W_COST * nc / STEPS; // CostCritic: average cost along the trajectory
          align += distToPath(path, x, y, near - 5, near + 40);
          if (v < 0) c += W_PREFER_FORWARD * -v * V_MAX * DT; // PreferForward: metres driven backwards
        }
        c += W_PATH_ALIGN * align / STEPS;
        double toGoal = Math.hypot(goal[0] - x, goal[1] - y);
        if (Math.hypot(goal[0] - pose.x, goal[1] - pose.y) < 1.4) c += W_GOAL * toGoal; // threshold_to_consider
        else c += W_PATH_FOLLOW * Math.hypot(fp[0] - x, fp[1] - y);
        double ang = Math.abs(Pose.wrap(Math.atan2(fp[1] - y, fp[0] - x) - th));
        if (ang > Math.PI / 2) c += W_PATH_ANGLE * ang; // PathAngle: facing away from the path ahead
        cost[b] = c;
      }
      double min = Double.MAX_VALUE;
      for (double c : cost) min = Math.min(min, c);
      double[] w = new double[BATCH];
      double sum = 0;
      for (int b = 0; b < BATCH; b++) {
        w[b] = Math.exp(-(cost[b] - min) / TEMPERATURE);
        sum += w[b];
      }
      for (int t = 0; t < STEPS; t++) {
        double dv = 0, ds = 0;
        for (int b = 0; b < BATCH; b++) {
          dv += w[b] * ev[b][t];
          ds += w[b] * es[b][t];
        }
        uv[t] = Navigator.clamp(uv[t] + dv / sum, vMin, vMax);
        us[t] = Navigator.clamp(us[t] + ds / sum, -1, 1);
      }
      // Smooth the plan (Nav2 runs a Savitzky-Golay filter): 3-point moving average.
      double[] sv = uv.clone(), ss = us.clone();
      for (int t = 1; t < STEPS - 1; t++) {
        uv[t] = (sv[t - 1] + sv[t] + sv[t + 1]) / 3;
        us[t] = (ss[t - 1] + ss[t] + ss[t + 1]) / 3;
      }
      double[] cmd = {uv[0], us[0]};
      // Shift the plan one step for next time (warm start).
      System.arraycopy(uv, 1, uv, 0, STEPS - 1);
      System.arraycopy(us, 1, us, 0, STEPS - 1);
      return cmd;
    }
  }

  // endregion
}
