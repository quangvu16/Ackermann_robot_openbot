package org.openbot.app.robot.slam;

import android.content.Context;
import android.content.Intent;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.net.Uri;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Looper;
import android.os.SystemClock;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.view.WindowManager;
import android.widget.Toast;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.core.content.FileProvider;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.util.ArrayList;
import java.util.List;
import org.openbot.app.robot.R;
import org.openbot.app.robot.common.ControlsFragment;
import org.openbot.app.robot.databinding.FragmentSlamBinding;
import org.openbot.app.robot.lidarbot.LidarBehaviors;
import org.openbot.app.robot.vehicle.LidarScan;
import timber.log.Timber;

/**
 * Indoor SLAM and navigation with the LiDAR (see {@link SlamEngine}, {@link Navigator},
 * {@link SlamMapView}).
 *
 * <p>Every new LiDAR revolution goes to a background thread: SLAM updates the pose and the map,
 * then the navigator (goal set by tapping the map) or Explore (the LiDAR robot's Avoid behaviour)
 * decides what to drive. Commands go through {@code Vehicle.setControl()}, so STOP and object
 * avoidance still apply. The phone's gyroscope gives the heading prior. Stale LiDAR stops the car;
 * leaving the screen stops it too.
 */
public class SlamFragment extends ControlsFragment implements SensorEventListener {

  private static final long TICK_MS = 100;

  private FragmentSlamBinding binding;
  private final Handler main = new Handler(Looper.getMainLooper());
  private HandlerThread worker;
  private Handler work;

  private final SlamEngine slam = new SlamEngine(OccupancyGrid.indoorDefault());
  private final Navigator navigator = new Navigator(new Navigator.Params());
  private final LidarBehaviors explorer =
      new LidarBehaviors(LidarBehaviors.Mode.AVOID, LidarBehaviors.Params.defaults(LidarBehaviors.Mode.AVOID));
  private volatile boolean exploring;
  private volatile boolean driving; // we sent a non-zero command and owe a stop
  private long lastScanId = -1;
  private volatile double[] lastAngles;
  private volatile double[] lastRanges;
  private volatile String statusText = "";

  // Gyro yaw: rotation rate about the gravity axis, integrated (works however the phone is mounted).
  private SensorManager sensors;
  private final float[] gravity = {0, 0, 9.81f};
  private volatile double imuYaw = Double.NaN;
  private long lastGyroNs;

  private final Runnable tick =
      new Runnable() {
        @Override
        public void run() {
          step();
          main.postDelayed(this, TICK_MS);
        }
      };

  @Override
  public View onCreateView(@NonNull LayoutInflater inflater, @Nullable ViewGroup c, @Nullable Bundle s) {
    binding = FragmentSlamBinding.inflate(inflater, c, false);
    return binding.getRoot();
  }

  @Override
  public void onViewCreated(@NonNull View view, @Nullable Bundle savedInstanceState) {
    super.onViewCreated(view, savedInstanceState);
    binding.map.setGrid(slam.getGrid());
    binding.map.setListener(
        new SlamMapView.Listener() {
          @Override
          public void onMapTap(double x, double y) {
            exploring = false;
            work(() -> navigator.setGoal(x, y));
            showButtons(true);
          }

          @Override
          public void onMapLongPress(double x, double y) {
            work(() -> slam.resetPose(new Pose(x, y, slam.getPose().theta)));
            toast(R.string.slam_pose_set);
          }
        });
    binding.modeGroup.check(R.id.mode_map);
    binding.modeGroup.addOnButtonCheckedListener(
        (g, id, checked) -> {
          if (checked) work(() -> slam.setMapping(id == R.id.mode_map));
        });
    binding.exploreButton.setOnClickListener(v -> {
      exploring = !exploring;
      if (exploring) work(navigator::cancel);
      else stopCar();
      showButtons(false);
    });
    binding.cancelButton.setOnClickListener(v -> {
      exploring = false;
      work(navigator::cancel);
      stopCar();
      showButtons(false);
    });
    binding.centerButton.setOnClickListener(v -> binding.map.recentre());
    binding.saveButton.setOnClickListener(v -> work(this::saveMap));
    binding.loadButton.setOnClickListener(v -> work(this::loadMap));
    binding.clearButton.setOnClickListener(v -> work(() -> {
      navigator.cancel();
      slam.reset();
    }));
    binding.exportButton.setOnClickListener(v -> work(this::exportMap));
    binding.layerMap.setOnCheckedChangeListener((b, on) -> binding.map.showMap = on);
    binding.layerScan.setOnCheckedChangeListener((b, on) -> binding.map.showScan = on);
    binding.layerPath.setOnCheckedChangeListener((b, on) -> binding.map.showPath = on);
    binding.layerTrail.setOnCheckedChangeListener((b, on) -> binding.map.showTrail = on);
    showButtons(false);
  }

  @Override
  protected String remoteScreenName() {
    return "Indoor SLAM";
  }

  @Override
  public void onResume() {
    super.onResume();
    worker = new HandlerThread("slam");
    worker.start();
    work = new Handler(worker.getLooper());
    sensors = (SensorManager) requireContext().getSystemService(Context.SENSOR_SERVICE);
    if (sensors != null) {
      Sensor gyro = sensors.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
      Sensor grav = sensors.getDefaultSensor(Sensor.TYPE_GRAVITY);
      if (gyro != null) sensors.registerListener(this, gyro, SensorManager.SENSOR_DELAY_GAME);
      if (grav != null) sensors.registerListener(this, grav, SensorManager.SENSOR_DELAY_GAME);
    }
    requireActivity().getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
    main.post(tick);
  }

  @Override
  public synchronized void onPause() {
    main.removeCallbacks(tick);
    exploring = false;
    navigator.cancel();
    stopCar();
    if (sensors != null) sensors.unregisterListener(this);
    if (worker != null) worker.quitSafely();
    worker = null;
    work = null;
    if (getActivity() != null) {
      getActivity().getWindow().clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
    }
    super.onPause();
  }

  @Override
  public void onDestroyView() {
    binding = null;
    super.onDestroyView();
  }

  private void work(Runnable r) {
    Handler h = work;
    if (h != null) h.post(r);
  }

  // region Loop

  private void step() {
    if (binding == null) return;
    LidarScan scan = lidarScanner.getLatestScan();
    boolean fresh = scan != null && lidarScanner.isFresh();
    if (!fresh) {
      binding.sourceInfo.setText(R.string.slam_needs_scan);
      if (driving) {
        stopCar();
        statusText = getString(R.string.lidarbot_source_none);
      }
    } else {
      binding.sourceInfo.setText(getString(R.string.lidarbot_source_scan, scan.getValidPoints()));
      if (scan.getScanId() != lastScanId) {
        lastScanId = scan.getScanId();
        int[] mm = scan.getRangesMm();
        double yaw = imuYaw;
        work(() -> onScan(mm, yaw));
      }
    }
    // Draw the newest state.
    Pose pose = slam.getPose();
    binding.map.setGrid(slam.getGrid());
    binding.map.setState(pose, lastAngles, lastRanges, slam.getTrail(),
        new ArrayList<>(navigator.getPath()), navigator.getGoal());
    binding.mapStatus.setText(getString(R.string.slam_status, pose.toString(),
        (int) Math.round(slam.getMatchQuality() * 100), slam.getScanCount()));
    binding.status.setText(statusText);
    showButtons(navigator.getGoal() != null);
  }

  /** On the SLAM thread: update the map and pose, then decide what to drive. */
  private void onScan(int[] rangesMm, double yaw) {
    int n = rangesMm.length;
    double[] a = new double[n];
    double[] r = new double[n];
    for (int i = 0; i < n; i++) {
      a[i] = -Math.toRadians((i + 0.5) * 360.0 / n); // bins run clockwise; + left here
      r[i] = rangesMm[i] == 0 ? 99 : rangesMm[i] / 1000.0; // 0 = no return: skipped
    }
    Pose pose = slam.update(a, r, yaw);
    lastAngles = a;
    lastRanges = r;

    double frontMin = Double.POSITIVE_INFINITY;
    for (int i = 0; i < n; i++) {
      if (r[i] >= SlamEngine.MIN_RANGE && r[i] < SlamEngine.MAX_RANGE && Math.abs(a[i]) < Math.toRadians(25)) {
        frontMin = Math.min(frontMin, r[i]);
      }
    }
    double now = SystemClock.elapsedRealtime() / 1000.0;
    if (navigator.getGoal() != null) {
      Navigator.Command c = navigator.step(pose, slam.getGrid(), frontMin, now);
      statusText = c.message;
      drive(c.throttle, c.steer);
      if (navigator.getGoal() == null) stopCar(); // arrived
    } else if (exploring) {
      LidarBehaviors.Decision d = explorer.step(LidarBehaviors.fromScan(rangesMm, 0, 0.15), now);
      statusText = "Exploring: " + d.reason;
      drive(d.throttle, d.steer);
    } else if (navigator.getState() == Navigator.State.ARRIVED) {
      statusText = "Arrived at the goal";
    } else if (navigator.getState() == Navigator.State.NO_PATH) {
      statusText = "No path to the goal on this map";
    } else {
      statusText = "";
    }
  }

  private void drive(double throttle, double steer) {
    float[] w = LidarBehaviors.toWheels(throttle, steer);
    driving = throttle != 0 || steer != 0;
    main.post(() -> {
      if (vehicle != null) vehicle.setControl(w[0], w[1]);
    });
  }

  private void stopCar() {
    driving = false;
    main.post(() -> {
      if (vehicle != null) vehicle.setControl(0, 0);
    });
  }

  // endregion

  // region Gyro

  @Override
  public void onSensorChanged(SensorEvent e) {
    if (e.sensor.getType() == Sensor.TYPE_GRAVITY) {
      System.arraycopy(e.values, 0, gravity, 0, 3);
      return;
    }
    if (e.sensor.getType() != Sensor.TYPE_GYROSCOPE) return;
    if (lastGyroNs != 0) {
      double dt = (e.timestamp - lastGyroNs) / 1e9;
      double g = Math.sqrt(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
      if (g > 1 && dt > 0 && dt < 0.2) {
        // Android's gravity sensor points up; turning counter-clockwise seen from above is +.
        double rate = (e.values[0] * gravity[0] + e.values[1] * gravity[1] + e.values[2] * gravity[2]) / g;
        double y = Double.isNaN(imuYaw) ? 0 : imuYaw;
        imuYaw = y + rate * dt;
      }
    }
    lastGyroNs = e.timestamp;
  }

  @Override
  public void onAccuracyChanged(Sensor sensor, int accuracy) {}

  // endregion

  // region Map files

  private File mapFile() {
    File dir = new File(requireContext().getFilesDir(), "slam");
    if (!dir.exists() && !dir.mkdirs()) Timber.w("Could not create %s", dir);
    return new File(dir, "map.obmap");
  }

  private void saveMap() {
    try (DataOutputStream out = new DataOutputStream(new FileOutputStream(mapFile()))) {
      slam.getGrid().write(out);
      toast(R.string.slam_saved);
    } catch (Exception e) {
      Timber.e(e, "Saving the SLAM map failed");
      toastText(e.getMessage());
    }
  }

  private void loadMap() {
    File f = mapFile();
    if (!f.exists()) {
      toast(R.string.slam_no_saved);
      return;
    }
    try (DataInputStream in = new DataInputStream(new FileInputStream(f))) {
      OccupancyGrid g = OccupancyGrid.read(in);
      navigator.cancel();
      slam.setGrid(g);
      slam.setMapping(false); // localize on the saved map, as slam_toolbox's localization mode
      main.post(() -> {
        if (binding != null) binding.modeGroup.check(R.id.mode_localize);
      });
      toast(R.string.slam_loaded);
    } catch (Exception e) {
      Timber.e(e, "Loading the SLAM map failed");
      toastText(e.getMessage());
    }
  }

  /** map.pgm + map.yaml, the ROS map_server format: open them in RViz or Nav2. */
  private void exportMap() {
    try {
      File dir = new File(requireContext().getFilesDir(), "slam/export");
      if (!dir.exists() && !dir.mkdirs()) throw new java.io.IOException("Cannot create " + dir);
      File pgm = new File(dir, "openbot_map.pgm");
      File yaml = new File(dir, "openbot_map.yaml");
      try (FileOutputStream o = new FileOutputStream(pgm)) {
        o.write(slam.getGrid().toPgm());
      }
      try (FileOutputStream o = new FileOutputStream(yaml)) {
        o.write(slam.getGrid().rosYaml(pgm.getName()).getBytes(java.nio.charset.StandardCharsets.UTF_8));
      }
      String auth = requireContext().getPackageName() + ".provider";
      ArrayList<Uri> uris = new ArrayList<>();
      uris.add(FileProvider.getUriForFile(requireContext(), auth, pgm));
      uris.add(FileProvider.getUriForFile(requireContext(), auth, yaml));
      Intent share = new Intent(Intent.ACTION_SEND_MULTIPLE);
      share.setType("application/octet-stream");
      share.putParcelableArrayListExtra(Intent.EXTRA_STREAM, uris);
      share.addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION);
      main.post(() -> startActivity(Intent.createChooser(share, getString(R.string.slam_export))));
    } catch (Exception e) {
      Timber.e(e, "Exporting the SLAM map failed");
      toastText(e.getMessage());
    }
  }

  // endregion

  private void showButtons(boolean goal) {
    if (binding == null) return;
    binding.exploreButton.setText(exploring ? R.string.slam_stop : R.string.slam_explore);
    binding.exploreButton.setBackgroundTintList(android.content.res.ColorStateList.valueOf(
        exploring ? android.graphics.Color.parseColor("#D32F2F") : requireContext().getColor(R.color.openBotBlue)));
    binding.cancelButton.setEnabled(goal || exploring);
  }

  private void toast(int res) {
    main.post(() -> {
      if (getContext() != null) Toast.makeText(getContext(), res, Toast.LENGTH_SHORT).show();
    });
  }

  private void toastText(String text) {
    main.post(() -> {
      if (getContext() != null) Toast.makeText(getContext(), String.valueOf(text), Toast.LENGTH_LONG).show();
    });
  }

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
