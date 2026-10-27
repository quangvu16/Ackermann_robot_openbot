package org.openbot.app.robot.lidarbot;

import android.content.Context;
import android.content.SharedPreferences;
import android.media.AudioManager;
import android.media.ToneGenerator;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.os.VibrationEffect;
import android.os.Vibrator;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.view.WindowManager;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.preference.PreferenceManager;
import java.util.Locale;
import org.openbot.app.robot.R;
import org.openbot.app.robot.common.ControlsFragment;
import org.openbot.app.robot.databinding.FragmentLidarBotBinding;
import org.openbot.app.robot.vehicle.LidarScan;
import org.openbot.app.robot.vehicle.Proximity;
import timber.log.Timber;

/**
 * LiDAR robot: Patrol, Follow, Avoid, Track and Warn, after the hoverboard robot's
 * hoverboard_laser package, with a face that shows each behaviour's mood next to the LiDAR map.
 *
 * <p>A 10 Hz tick reads the newest LiDAR data (full scan over USB or the car's WiFi, otherwise the
 * car's 8 proximity sectors), asks {@link LidarBehaviors} what to do, sends it through {@code
 * Vehicle.setControl()} - so the STOP latch and object avoidance still apply - and updates the
 * face, map and status. Stale or missing LiDAR stops the car.
 */
public class LidarBotFragment extends ControlsFragment {

  private static final long TICK_MS = 100;
  private static final String PREF_MODE = "lidarbot_mode";

  private FragmentLidarBotBinding binding;
  private final Handler handler = new Handler(Looper.getMainLooper());
  private LidarBehaviors.Mode mode = LidarBehaviors.Mode.FOLLOW;
  private LidarBehaviors behaviour;
  private final java.util.EnumMap<LidarBehaviors.Mode, LidarBehaviors.Params> params =
      new java.util.EnumMap<>(LidarBehaviors.Mode.class);
  private boolean running;
  private ToneGenerator tone;
  private long lastBeepMs;

  private final Runnable tick =
      new Runnable() {
        @Override
        public void run() {
          step();
          handler.postDelayed(this, TICK_MS);
        }
      };

  @Override
  public View onCreateView(
      @NonNull LayoutInflater inflater, @Nullable ViewGroup container, @Nullable Bundle state) {
    binding = FragmentLidarBotBinding.inflate(inflater, container, false);
    return binding.getRoot();
  }

  @Override
  public void onViewCreated(@NonNull View view, @Nullable Bundle savedInstanceState) {
    super.onViewCreated(view, savedInstanceState);
    for (LidarBehaviors.Mode m : LidarBehaviors.Mode.values()) {
      params.put(m, LidarBehaviors.Params.defaults(m));
    }
    SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(requireContext());
    try {
      mode = LidarBehaviors.Mode.valueOf(prefs.getString(PREF_MODE, mode.name()));
    } catch (IllegalArgumentException ignored) {
      // keep the default
    }
    binding.lidarMap.setRangeMaxMeters(3f);
    binding.modeGroup.check(buttonFor(mode));
    binding.modeGroup.addOnButtonCheckedListener(
        (group, id, checked) -> {
          if (!checked) return;
          LidarBehaviors.Mode m = modeFor(id);
          if (m == mode) return;
          stopBehaviour();
          mode = m;
          prefs.edit().putString(PREF_MODE, m.name()).apply();
          showMode();
        });
    setupSensorLinkControls(binding.sensorLink.getRoot());
    binding.startStop.setOnClickListener(v -> {
      if (running) stopBehaviour();
      else startBehaviour();
    });
    binding.speed.addOnChangeListener((s, value, fromUser) -> {
      if (fromUser) current().speed = value;
      showParams();
    });
    binding.distance.addOnChangeListener((s, value, fromUser) -> {
      if (fromUser) current().responseDist = value;
      showParams();
    });
    binding.angle.addOnChangeListener((s, value, fromUser) -> {
      if (fromUser) current().laserAngle = value;
      showParams();
    });
    showMode();
  }

  @Override
  protected String remoteScreenName() {
    return "LiDAR robot";
  }

  /** Browser page settings (Sensor link, server mode): behaviour, start/stop and its three sliders. */
  @Override
  protected void addScreenSettings(java.util.List<org.openbot.app.robot.sensorlink.RemoteSettings.Setting> out) {
    if (binding == null) return;
    java.util.List<String> modes = new java.util.ArrayList<>();
    for (LidarBehaviors.Mode m : LidarBehaviors.Mode.values()) modes.add(m.name().toLowerCase(Locale.ROOT));
    out.add(org.openbot.app.robot.sensorlink.RemoteSettings.Setting.choice(
        "lidarbot.mode", "LiDAR robot", "Behaviour", modes,
        () -> mode.name().toLowerCase(Locale.ROOT),
        v -> binding.modeGroup.check(buttonFor(LidarBehaviors.Mode.valueOf(((String) v).toUpperCase(Locale.ROOT))))));
    out.add(org.openbot.app.robot.sensorlink.RemoteSettings.Setting.action(
        "lidarbot.run", "LiDAR robot", "Run the behaviour", "START", "STOP", () -> running,
        v -> {
          boolean on = (Boolean) v;
          if (on && !running) startBehaviour();
          else if (!on && running) stopBehaviour();
        }));
    out.add(org.openbot.app.robot.sensorlink.RemoteSettings.Setting.range(
        "lidarbot.speed", "LiDAR robot", "Speed (fraction)", 0, 1, 0.05,
        () -> current().speed,
        v -> {
          current().speed = (Double) v;
          binding.speed.setValue(snap(((Double) v).floatValue(), 0f, 1f, 0.05f));
          showParams();
        }));
    out.add(org.openbot.app.robot.sensorlink.RemoteSettings.Setting.range(
        "lidarbot.distance", "LiDAR robot", "Distance (m)", 0.2, 2.0, 0.05,
        () -> current().responseDist,
        v -> {
          current().responseDist = (Double) v;
          binding.distance.setValue(snap(((Double) v).floatValue(), 0.2f, 2.0f, 0.05f));
          showParams();
        }));
    out.add(org.openbot.app.robot.sensorlink.RemoteSettings.Setting.range(
        "lidarbot.angle", "LiDAR robot", "Angle (± degrees)", 10, 120, 5,
        () -> current().laserAngle,
        v -> {
          current().laserAngle = (Double) v;
          binding.angle.setValue(snap(((Double) v).floatValue(), 10f, 120f, 5f));
          showParams();
        }));
  }

  @Override
  public void onResume() {
    super.onResume();
    try {
      tone = new ToneGenerator(AudioManager.STREAM_ALARM, 80);
    } catch (RuntimeException e) {
      Timber.w(e, "No tone generator; the warning will only vibrate");
      tone = null;
    }
    handler.removeCallbacks(tick);
    handler.post(tick);
  }

  @Override
  public synchronized void onPause() {
    stopBehaviour(); // never leave the car driving itself off screen
    handler.removeCallbacks(tick);
    if (tone != null) {
      tone.release();
      tone = null;
    }
    super.onPause();
  }

  @Override
  public void onDestroyView() {
    handler.removeCallbacks(tick);
    binding = null;
    super.onDestroyView();
  }

  // region Behaviour

  private LidarBehaviors.Params current() {
    return params.get(mode);
  }

  private void startBehaviour() {
    behaviour = new LidarBehaviors(mode, current());
    running = true;
    // Keep the screen on while the car drives itself: the face is the display.
    requireActivity().getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
    showRunning();
  }

  private void stopBehaviour() {
    boolean was = running;
    running = false;
    behaviour = null;
    if (vehicle != null) vehicle.setControl(0, 0);
    if (getActivity() != null) {
      getActivity().getWindow().clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
    }
    if (was && binding != null) showRunning();
  }

  /** Newest LiDAR data as points: a fresh full scan, else fresh proximity sectors, else null. */
  private LidarBehaviors.Points readLidar(double minRange) {
    LidarScan scan = lidarScanner.getLatestScan();
    if (scan != null && lidarScanner.isFresh()) {
      return LidarBehaviors.fromScan(scan.getRangesMm(), 0, minRange);
    }
    Proximity p = vehicle == null ? null : vehicle.getProximity();
    if (p != null && p.hasEverReported() && p.getAge() < Proximity.STALE_AFTER_MS) {
      float[] m = p.getSectorMeters();
      double[] d = new double[m.length];
      for (int i = 0; i < m.length; i++) d[i] = m[i];
      return LidarBehaviors.fromSectors(d);
    }
    return null;
  }

  private void step() {
    if (binding == null) return;
    LidarBehaviors.Points points = readLidar(current().minRange);
    refreshLidar(binding.lidarMap, null);
    showSource(points);

    if (!running) {
      LidarBehaviors.Mood idle = points == null ? LidarBehaviors.Mood.BLIND : LidarBehaviors.Mood.IDLE;
      binding.face.setMood(idle, 0, moodCaption(idle));
      return;
    }
    LidarBehaviors.Decision d = behaviour.step(points, SystemClock.elapsedRealtime() / 1000.0);
    float[] wheels = LidarBehaviors.toWheels(d.throttle, d.steer);
    if (vehicle != null) vehicle.setControl(wheels[0], wheels[1]);
    binding.face.setMood(d.mood, (float) d.gazeDeg, moodCaption(d.mood));
    binding.status.setText(d.reason);
    binding.status.setTextColor(darker(RobotFaceView.moodColor(d.mood)));
    if (d.alarm) alarm();
  }

  /** Warning's alarm: a short beep and buzz, at most every 400 ms. */
  private void alarm() {
    long now = SystemClock.elapsedRealtime();
    if (now - lastBeepMs < 400) return;
    lastBeepMs = now;
    if (tone != null) tone.startTone(ToneGenerator.TONE_CDMA_ALERT_CALL_GUARD, 200);
    Vibrator v = (Vibrator) requireContext().getSystemService(Context.VIBRATOR_SERVICE);
    if (v != null && v.hasVibrator()) {
      if (android.os.Build.VERSION.SDK_INT >= 26) {
        v.vibrate(VibrationEffect.createOneShot(150, VibrationEffect.DEFAULT_AMPLITUDE));
      } else {
        v.vibrate(150);
      }
    }
  }

  // endregion

  // region UI

  private void showMode() {
    int help;
    switch (mode) {
      case PATROL:
        help = R.string.lidarbot_patrol_help;
        break;
      case AVOID:
        help = R.string.lidarbot_avoid_help;
        break;
      case TRACK:
        help = R.string.lidarbot_track_help;
        break;
      case WARN:
        help = R.string.lidarbot_warn_help;
        break;
      case FOLLOW:
      default:
        help = R.string.lidarbot_follow_help;
        break;
    }
    binding.modeHelp.setText(help);
    LidarBehaviors.Params p = current();
    binding.speed.setValue(snap((float) p.speed, 0f, 1f, 0.05f));
    binding.distance.setValue(snap((float) p.responseDist, 0.2f, 2.0f, 0.05f));
    binding.angle.setValue(snap((float) p.laserAngle, 10f, 120f, 5f));
    // Warning never drives: its speed slider means nothing.
    binding.speed.setEnabled(mode != LidarBehaviors.Mode.WARN);
    showParams();
    showRunning();
  }

  private void showParams() {
    if (binding == null) return;
    binding.speedLabel.setText(
        String.format(Locale.getDefault(), "%s  %d%%", getString(R.string.lidarbot_speed), Math.round(binding.speed.getValue() * 100)));
    binding.distanceLabel.setText(
        String.format(Locale.getDefault(), "%s  %.2f", getString(R.string.lidarbot_distance), binding.distance.getValue()));
    binding.angleLabel.setText(
        String.format(Locale.getDefault(), "%s  ±%.0f", getString(R.string.lidarbot_angle), binding.angle.getValue()));
    if (behaviour != null) behaviour.setParams(current());
  }

  private void showRunning() {
    binding.startStop.setText(running ? R.string.lidarbot_stop : R.string.lidarbot_start);
    binding.startStop.setBackgroundTintList(
        android.content.res.ColorStateList.valueOf(
            running ? android.graphics.Color.parseColor("#D32F2F") : requireContext().getColor(R.color.openBotBlue)));
    if (!running) {
      binding.status.setText("");
    }
  }

  private void showSource(LidarBehaviors.Points points) {
    if (points == null) {
      binding.sourceInfo.setText(R.string.lidarbot_source_none);
    } else if (points.coarse) {
      binding.sourceInfo.setText(R.string.lidarbot_source_sectors);
    } else {
      binding.sourceInfo.setText(getString(R.string.lidarbot_source_scan, points.size()));
    }
  }

  private String moodCaption(LidarBehaviors.Mood mood) {
    switch (mood) {
      case HAPPY:
        return getString(R.string.lidarbot_mood_happy);
      case LOVE:
        return getString(R.string.lidarbot_mood_love);
      case FOCUSED:
        return getString(R.string.lidarbot_mood_focused);
      case SEARCHING:
        return getString(R.string.lidarbot_mood_searching);
      case WORRIED:
        return getString(R.string.lidarbot_mood_worried);
      case SCARED:
        return getString(R.string.lidarbot_mood_scared);
      case BLIND:
        return getString(R.string.lidarbot_mood_blind);
      case IDLE:
      default:
        return getString(R.string.lidarbot_mood_idle);
    }
  }

  private static int darker(int color) {
    float[] hsv = new float[3];
    android.graphics.Color.colorToHSV(color, hsv);
    hsv[2] *= 0.65f;
    return android.graphics.Color.HSVToColor(hsv);
  }

  private static float snap(float v, float lo, float hi, float step) {
    float c = Math.max(lo, Math.min(hi, v));
    return lo + Math.round((c - lo) / step) * step;
  }

  private int buttonFor(LidarBehaviors.Mode m) {
    switch (m) {
      case PATROL:
        return R.id.mode_patrol;
      case AVOID:
        return R.id.mode_avoid;
      case TRACK:
        return R.id.mode_track;
      case WARN:
        return R.id.mode_warn;
      case FOLLOW:
      default:
        return R.id.mode_follow;
    }
  }

  private LidarBehaviors.Mode modeFor(int id) {
    if (id == R.id.mode_patrol) return LidarBehaviors.Mode.PATROL;
    if (id == R.id.mode_avoid) return LidarBehaviors.Mode.AVOID;
    if (id == R.id.mode_track) return LidarBehaviors.Mode.TRACK;
    if (id == R.id.mode_warn) return LidarBehaviors.Mode.WARN;
    return LidarBehaviors.Mode.FOLLOW;
  }

  // endregion

  @Override
  protected void processControllerKeyData(String command) {}

  @Override
  protected void processUSBData(String data) {
    if (vehicle != null && !vehicle.isReady()) {
      vehicle.setReady(true);
      vehicle.requestVehicleConfig();
    }
  }
}
