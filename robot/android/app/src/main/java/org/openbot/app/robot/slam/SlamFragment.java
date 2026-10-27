package org.openbot.app.robot.slam;

import org.openbot.app.robot.common.IconButtons;
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
import android.widget.ArrayAdapter;
import android.widget.LinearLayout;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;
import androidx.appcompat.app.AlertDialog;
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
public class SlamFragment extends ControlsFragment
    implements SensorEventListener, org.openbot.app.robot.sensorlink.SlamHub.Source {

  private static final long TICK_MS = 100;

  private FragmentSlamBinding binding;
  private final Handler main = new Handler(Looper.getMainLooper());
  private HandlerThread worker;
  private Handler work;

  private final SlamSystem slam = new SlamSystem(Slam.Type.CARTOGRAPHER, OccupancyGrid.indoorDefault());
  private final Navigator.Params navParams = new Navigator.Params();
  private final Navigator navigator = new Navigator(navParams);
  private volatile String algorithmsText = "";
  private final LidarBehaviors explorer =
      new LidarBehaviors(LidarBehaviors.Mode.AVOID, LidarBehaviors.Params.defaults(LidarBehaviors.Mode.AVOID));
  private volatile boolean exploring;
  private volatile boolean driving; // we sent a non-zero command and owe a stop
  private long lastScanId = -1;
  private final java.util.concurrent.atomic.AtomicBoolean busy = new java.util.concurrent.atomic.AtomicBoolean();
  private int shownRelocalizeSeq;
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
    binding.algorithmsButton.setOnClickListener(v -> showAlgorithms());
    setupSensorLinkControls(binding.sensorLink.getRoot());
    binding.findButton.setOnClickListener(v -> {
      slam.requestGlobalLocalization();
      toast(R.string.slam_finding);
    });
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
    applySavedAlgorithms();
    org.openbot.app.robot.sensorlink.SlamHub.get().setSource(this);
    main.post(tick);
  }

  @Override
  public synchronized void onPause() {
    org.openbot.app.robot.sensorlink.SlamHub.get().clearSource(this);
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
        if (busy.compareAndSet(false, true)) { // drop scans while the last one is still being matched
          work(() -> {
            try {
              onScan(mm, yaw);
            } finally {
              busy.set(false);
            }
          });
        }
      }
    }
    // Draw the newest state.
    Pose pose = slam.getPose();
    binding.map.setGrid(slam.getGrid());
    binding.map.setState(pose, lastAngles, lastRanges, slam.getTrail(),
        new ArrayList<>(navigator.getPath()), navigator.getGoal());
    binding.mapStatus.setText(getString(R.string.slam_status, pose.toString(),
        (int) Math.round(slam.getMatchQuality() * 100), slam.getScanCount())
        + " · " + getString(R.string.slam_graph, slam.getSubmapCount(), slam.getLoopClosureCount())
        + "\n" + algorithmsText);
    int seq = slam.getRelocalizeSeq();
    if (seq != shownRelocalizeSeq) {
      shownRelocalizeSeq = seq;
      toast(slam.wasRelocalizeFound() ? R.string.slam_found : R.string.slam_not_found);
    }
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
    Pose pose = slam.update(a, r, yaw, SystemClock.elapsedRealtime() / 1000.0);
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
      slam.setMapping(false); // localize on the saved map (Cartographer's pure localization)
      slam.requestGlobalLocalization(); // find the car on it with no hint
      main.post(() -> {
        if (binding != null) binding.modeGroup.check(R.id.mode_localize);
      });
      toast(R.string.slam_finding);
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

  // region Sensor link: /slam WebSocket and browser settings

  private volatile Slam.Type curSlam = Slam.Type.CARTOGRAPHER;
  private volatile SlamSystem.Localizer curLoc = SlamSystem.Localizer.SCAN_MATCHING;
  private volatile Navigator.Planner curPlanner = Navigator.Planner.NAVFN;
  private volatile Navigator.Controller curController = Navigator.Controller.RPP;

  @Override
  public String json(boolean withMap) {
    return SlamWire.json(curSlam.label, curLoc.label, curPlanner.label, curController.label, slam.isMapping(),
        exploring, slam.getPose(), slam.getMatchQuality(), slam.getScanCount(), slam.getSubmapCount(),
        slam.getLoopClosureCount(), navigator.getGoal(), new ArrayList<>(navigator.getPath()), statusText,
        withMap ? slam.getGrid() : null);
  }

  @Override
  public int mapVersion() {
    OccupancyGrid g = slam.getGrid();
    return System.identityHashCode(g) * 31 + g.getVersion();
  }

  /** A command from the browser page, as if the same button had been pressed here. */
  @Override
  public void command(String text) {
    SlamWire.Command c = SlamWire.parse(text);
    if (c.kind == null) return;
    main.post(() -> {
      if (binding == null) return;
      switch (c.kind) {
        case "goal":
          exploring = false;
          work(() -> navigator.setGoal(c.x, c.y));
          break;
        case "cancel":
          exploring = false;
          work(navigator::cancel);
          stopCar();
          break;
        case "explore":
          exploring = c.flag;
          if (exploring) work(navigator::cancel);
          else stopCar();
          break;
        case "findMe":
          slam.requestGlobalLocalization();
          break;
        case "place":
          work(() -> slam.resetPose(new Pose(c.x, c.y, c.theta)));
          break;
        case "mapping":
          binding.modeGroup.check(c.flag ? R.id.mode_map : R.id.mode_localize);
          break;
        default:
          break;
      }
      showButtons(navigator.getGoal() != null);
    });
  }

  @Override
  protected void addScreenSettings(List<org.openbot.app.robot.sensorlink.RemoteSettings.Setting> out) {
    out.add(org.openbot.app.robot.sensorlink.RemoteSettings.Setting.choice(
        "slam.mode", "Indoor SLAM", "Mode", java.util.Arrays.asList("build map", "localize only"),
        () -> slam.isMapping() ? "build map" : "localize only",
        v -> binding.modeGroup.check("build map".equals(v) ? R.id.mode_map : R.id.mode_localize)));
    out.add(org.openbot.app.robot.sensorlink.RemoteSettings.Setting.choice(
        "slam.engine", "Indoor SLAM", "SLAM", java.util.Arrays.asList(labels(Slam.Type.values())),
        () -> curSlam.label, v -> chooseFromBrowser(Slam.Type.values(), (String) v, -1)));
    out.add(org.openbot.app.robot.sensorlink.RemoteSettings.Setting.choice(
        "slam.localizer", "Indoor SLAM", "Localizer (saved map)", java.util.Arrays.asList(labels(SlamSystem.Localizer.values())),
        () -> curLoc.label, v -> chooseFromBrowser(SlamSystem.Localizer.values(), (String) v, -1)));
    out.add(org.openbot.app.robot.sensorlink.RemoteSettings.Setting.choice(
        "slam.planner", "Indoor SLAM", "Planner", java.util.Arrays.asList(labels(Navigator.Planner.values())),
        () -> curPlanner.label, v -> chooseFromBrowser(Navigator.Planner.values(), (String) v, -1)));
    out.add(org.openbot.app.robot.sensorlink.RemoteSettings.Setting.choice(
        "slam.controller", "Indoor SLAM", "Controller", java.util.Arrays.asList(labels(Navigator.Controller.values())),
        () -> curController.label, v -> chooseFromBrowser(Navigator.Controller.values(), (String) v, -1)));
    out.add(org.openbot.app.robot.sensorlink.RemoteSettings.Setting.action(
        "slam.explore", "Indoor SLAM", "Explore", "EXPLORE", "STOP", () -> exploring,
        v -> command("{\"explore\":" + v + "}")));
    out.add(org.openbot.app.robot.sensorlink.RemoteSettings.Setting.action(
        "slam.findMe", "Indoor SLAM", "Find the car on the map", "FIND ME", "FIND ME", () -> false,
        v -> slam.requestGlobalLocalization()));
  }

  /** One algorithm picked in the browser: save it and apply it like the dialog does. */
  private void chooseFromBrowser(Enum<?>[] values, String label, int unused) {
    String[] ls = labels(values);
    int idx = java.util.Arrays.asList(ls).indexOf(label);
    if (idx < 0) return;
    Slam.Type t = curSlam;
    SlamSystem.Localizer l = curLoc;
    Navigator.Planner pl = curPlanner;
    Navigator.Controller co = curController;
    Object v = values[idx];
    if (v instanceof Slam.Type) t = (Slam.Type) v;
    else if (v instanceof SlamSystem.Localizer) l = (SlamSystem.Localizer) v;
    else if (v instanceof Navigator.Planner) pl = (Navigator.Planner) v;
    else co = (Navigator.Controller) v;
    prefs().edit().putString("slam", t.name()).putString("localizer", l.name()).putString("planner", pl.name())
        .putString("controller", co.name()).apply();
    applyAlgorithms(t, l, pl, co);
  }

  // endregion

  // region Algorithm choices (saved on the phone)

  private android.content.SharedPreferences prefs() {
    return requireContext().getSharedPreferences("slam_algorithms", Context.MODE_PRIVATE);
  }

  private static <E extends Enum<E>> E saved(android.content.SharedPreferences p, String key, E def) {
    try {
      return Enum.valueOf(def.getDeclaringClass(), p.getString(key, def.name()));
    } catch (IllegalArgumentException e) {
      return def;
    }
  }

  private void applySavedAlgorithms() {
    android.content.SharedPreferences p = prefs();
    applyAlgorithms(saved(p, "slam", Slam.Type.CARTOGRAPHER), saved(p, "localizer", SlamSystem.Localizer.SCAN_MATCHING),
        saved(p, "planner", Navigator.Planner.NAVFN), saved(p, "controller", Navigator.Controller.RPP));
  }

  private void applyAlgorithms(Slam.Type t, SlamSystem.Localizer l, Navigator.Planner pl, Navigator.Controller co) {
    algorithmsText = t.label + " · " + l.label + " · " + pl.label + " · " + co.label;
    curSlam = t;
    curLoc = l;
    curPlanner = pl;
    curController = co;
    work(() -> {
      slam.setEngine(t);
      slam.setLocalizer(l);
      navParams.planner = pl;
      navParams.controller = co;
      if (navigator.getGoal() != null) {
        double[] g = navigator.getGoal();
        navigator.setGoal(g[0], g[1]); // re-plan with the new planner
      }
    });
  }

  /** Four pickers: SLAM engine, localizer, planner, controller. */
  private void showAlgorithms() {
    android.content.SharedPreferences p = prefs();
    LinearLayout box = new LinearLayout(requireContext());
    box.setOrientation(LinearLayout.VERTICAL);
    int pad = (int) (16 * getResources().getDisplayMetrics().density);
    box.setPadding(pad, pad / 2, pad, 0);
    Spinner sSlam = picker(box, R.string.slam_algo_slam, labels(Slam.Type.values()), saved(p, "slam", Slam.Type.CARTOGRAPHER).ordinal());
    Spinner sLoc = picker(box, R.string.slam_algo_localizer, labels(SlamSystem.Localizer.values()),
        saved(p, "localizer", SlamSystem.Localizer.SCAN_MATCHING).ordinal());
    Spinner sPl = picker(box, R.string.slam_algo_planner, labels(Navigator.Planner.values()), saved(p, "planner", Navigator.Planner.NAVFN).ordinal());
    Spinner sCo = picker(box, R.string.slam_algo_controller, labels(Navigator.Controller.values()),
        saved(p, "controller", Navigator.Controller.RPP).ordinal());
    TextView note = new TextView(requireContext());
    note.setText(R.string.slam_algo_note);
    note.setPadding(0, pad / 2, 0, 0);
    box.addView(note);
    new AlertDialog.Builder(requireContext())
        .setTitle(R.string.slam_algorithms)
        .setView(box)
        .setPositiveButton(android.R.string.ok, (d, w) -> {
          Slam.Type t = Slam.Type.values()[sSlam.getSelectedItemPosition()];
          SlamSystem.Localizer l = SlamSystem.Localizer.values()[sLoc.getSelectedItemPosition()];
          Navigator.Planner pl = Navigator.Planner.values()[sPl.getSelectedItemPosition()];
          Navigator.Controller co = Navigator.Controller.values()[sCo.getSelectedItemPosition()];
          p.edit().putString("slam", t.name()).putString("localizer", l.name()).putString("planner", pl.name())
              .putString("controller", co.name()).apply();
          applyAlgorithms(t, l, pl, co);
        })
        .setNegativeButton(android.R.string.cancel, null)
        .show();
  }

  private static String[] labels(Enum<?>[] values) {
    String[] out = new String[values.length];
    for (int i = 0; i < values.length; i++) {
      Object v = values[i];
      if (v instanceof Slam.Type) out[i] = ((Slam.Type) v).label;
      else if (v instanceof SlamSystem.Localizer) out[i] = ((SlamSystem.Localizer) v).label;
      else if (v instanceof Navigator.Planner) out[i] = ((Navigator.Planner) v).label;
      else out[i] = ((Navigator.Controller) v).label;
    }
    return out;
  }

  private Spinner picker(LinearLayout box, int title, String[] items, int selected) {
    TextView t = new TextView(requireContext());
    t.setText(title);
    t.setTextSize(15);
    t.setTypeface(android.graphics.Typeface.DEFAULT_BOLD);
    t.setPadding(0, (int) (10 * getResources().getDisplayMetrics().density), 0, 0);
    box.addView(t);
    Spinner sp = new Spinner(requireContext());
    ArrayAdapter<String> a = new ArrayAdapter<>(requireContext(), android.R.layout.simple_spinner_item, items);
    a.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
    sp.setAdapter(a);
    sp.setSelection(selected);
    box.addView(sp);
    return sp;
  }

  // endregion

  private void showButtons(boolean goal) {
    if (binding == null) return;
    IconButtons.set(binding.exploreButton,
        exploring ? R.drawable.ic_btn_stop : R.drawable.ic_btn_travel_explore,
        exploring ? R.string.slam_stop : R.string.slam_explore);
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
