package org.openbot.app.robot.valetudo;

import android.content.Context;
import android.content.SharedPreferences;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.media.AudioManager;
import android.media.ToneGenerator;
import android.os.Bundle;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.Looper;
import android.os.SystemClock;
import android.text.InputType;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.view.WindowManager;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.Toast;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.widget.SwitchCompat;
import java.io.DataInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import org.openbot.app.robot.R;
import org.openbot.app.robot.common.ControlsFragment;
import org.openbot.app.robot.common.IconButtons;
import org.openbot.app.robot.databinding.FragmentRoomsBinding;
import org.openbot.app.robot.lidarbot.LidarBehaviors;
import org.openbot.app.robot.sensorlink.RemoteSettings;
import org.openbot.app.robot.sensorlink.ValetudoHub;
import org.openbot.app.robot.slam.CleaningMission;
import org.openbot.app.robot.slam.CoveragePlanner;
import org.openbot.app.robot.slam.Navigator;
import org.openbot.app.robot.slam.OccupancyGrid;
import org.openbot.app.robot.slam.Pose;
import org.openbot.app.robot.slam.RoomMap;
import org.openbot.app.robot.slam.RoomSegmenter;
import org.openbot.app.robot.slam.Slam;
import org.openbot.app.robot.slam.SlamEngine;
import org.openbot.app.robot.slam.SlamMapView;
import org.openbot.app.robot.slam.SlamSystem;
import org.openbot.app.robot.vehicle.LidarScan;
import timber.log.Timber;

/**
 * Rooms &amp; Zones: what Valetudo (github.com/Hypfer/Valetudo) gives a robot vacuum, for this car on
 * its own LiDAR map. It opens the map saved in Indoor SLAM, finds the car on it (pure
 * localization), and adds rooms ({@link RoomSegmenter}), zones, no-go areas, virtual walls and a
 * home point ({@link RoomMap}). Start patrols the selected rooms or zone in lanes ({@link
 * CoveragePlanner}, {@link CleaningMission}); the navigator never plans through restrictions.
 *
 * <p>Valetudo's REST API is served through the Sensor link's web server ({@link ValetudoApi}) and
 * its MQTT topics by {@link ValetudoMqtt}, so Home Assistant and Valetudo scripts can use the car.
 * Drive commands go through {@code Vehicle.setControl()}, so STOP and object avoidance still apply.
 */
public class RoomsFragment extends ControlsFragment
    implements SensorEventListener, ValetudoApi.Robot, ValetudoHub.Source, ValetudoMqtt.StatusListener {

  private static final long TICK_MS = 100;
  private static final String PREFS = "valetudo";

  private FragmentRoomsBinding binding;
  private final Handler main = new Handler(Looper.getMainLooper());
  private HandlerThread worker;
  private Handler work;

  // SLAM on the saved map (localize only), the navigator and the job runner.
  private final SlamSystem slam = new SlamSystem(Slam.Type.CARTOGRAPHER, OccupancyGrid.indoorDefault());
  private final Navigator.Params navParams = new Navigator.Params();
  private final Navigator navigator = new Navigator(navParams);
  private final CleaningMission mission = new CleaningMission(navigator);
  private volatile boolean mapLoaded;

  // Rooms, zones, restrictions; what is selected.
  private volatile RoomMap rooms = new RoomMap();
  private volatile int roomsVersion;
  private final Set<Integer> selectedRooms = Collections.synchronizedSet(new LinkedHashSet<>());
  private volatile int[] selectedItem; // {kind, index}: 0 zone, 1 no-go, 2 wall
  private volatile List<double[]> activeZones = new ArrayList<>();

  private final ValetudoApi api = new ValetudoApi(this);
  private ValetudoMqtt mqtt;
  private volatile String mqttStatus = "";
  private long lastMqttMs;

  private volatile boolean driving;
  private long lastScanId = -1;
  private final java.util.concurrent.atomic.AtomicBoolean busy = new java.util.concurrent.atomic.AtomicBoolean();
  private int shownRelocalizeSeq;
  private volatile double[] lastAngles;
  private volatile double[] lastRanges;
  private volatile String statusDetail = "";

  // Gyro yaw, as on Indoor SLAM.
  private SensorManager sensors;
  private final float[] gravity = {0, 0, 9.81f};
  private volatile double imuYaw = Double.NaN;
  private long lastGyroNs;

  private final Runnable tick = new Runnable() {
    @Override
    public void run() {
      step();
      main.postDelayed(this, TICK_MS);
    }
  };

  @Override
  public View onCreateView(@NonNull LayoutInflater inflater, @Nullable ViewGroup c, @Nullable Bundle s) {
    binding = FragmentRoomsBinding.inflate(inflater, c, false);
    return binding.getRoot();
  }

  @Override
  public void onViewCreated(@NonNull View view, @Nullable Bundle savedInstanceState) {
    super.onViewCreated(view, savedInstanceState);
    binding.map.showTrail = false;
    binding.map.setListener(new SlamMapView.Listener() {
      @Override
      public void onMapTap(double x, double y) {
        onTap(x, y);
      }

      @Override
      public void onMapLongPress(double x, double y) {
        work(() -> slam.resetPose(new Pose(x, y, slam.getPose().theta)));
        toast(R.string.slam_pose_set);
      }
    });
    binding.map.setEditListener(this::onShapeDrawn);
    binding.toolGroup.check(R.id.tool_select);
    binding.toolGroup.addOnButtonCheckedListener((g, id, checked) -> {
      if (!checked) return;
      binding.map.tool = id == R.id.tool_goto ? RoomsMapView.Tool.GO_TO
          : id == R.id.tool_zone ? RoomsMapView.Tool.ZONE
          : id == R.id.tool_nogo ? RoomsMapView.Tool.NO_GO
          : id == R.id.tool_wall ? RoomsMapView.Tool.WALL
          : id == R.id.tool_home ? RoomsMapView.Tool.HOME : RoomsMapView.Tool.SELECT;
    });
    binding.centerButton.setOnClickListener(v -> binding.map.recentre());
    binding.startButton.setOnClickListener(v -> {
      String err = basicControl(mission.isBusy() ? "pause" : "start"); // start also resumes after a pause
      if (err != null) toastText(err);
    });
    binding.stopButton.setOnClickListener(v -> basicControl("stop"));
    binding.homeButton.setOnClickListener(v -> {
      String err = basicControl("home");
      if (err != null) toastText(err);
    });
    binding.locateButton.setOnClickListener(v -> locate());
    binding.detectButton.setOnClickListener(v -> work(this::detectRooms));
    binding.renameButton.setOnClickListener(v -> showRename());
    binding.deleteButton.setOnClickListener(v -> deleteSelected());
    binding.settingsButton.setOnClickListener(v -> showSettings());
    setupSensorLinkControls(binding.sensorLink.getRoot());
  }

  @Override
  protected String remoteScreenName() {
    return "Rooms & Zones";
  }

  @Override
  public void onResume() {
    super.onResume();
    worker = new HandlerThread("rooms");
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
    work(this::loadMapAndRooms);
    ValetudoHub.get().setSource(this);
    startMqtt();
    main.post(tick);
  }

  @Override
  public synchronized void onPause() {
    ValetudoHub.get().clearSource(this);
    stopMqtt();
    main.removeCallbacks(tick);
    mission.stop();
    stopCar();
    if (sensors != null) sensors.unregisterListener(this);
    if (worker != null) worker.quitSafely();
    worker = null;
    work = null;
    if (getActivity() != null) getActivity().getWindow().clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
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

  // region Map and rooms files

  private File slamDir() {
    File dir = new File(requireContext().getFilesDir(), "slam");
    if (!dir.exists() && !dir.mkdirs()) Timber.w("Could not create %s", dir);
    return dir;
  }

  /** On the worker: the map from Indoor SLAM (Save), then this screen's rooms for it. */
  private void loadMapAndRooms() {
    File f = new File(slamDir(), "map.obmap");
    if (!f.exists()) {
      mapLoaded = false;
      statusDetail = getString(R.string.rooms_no_map);
      return;
    }
    try (DataInputStream in = new DataInputStream(new FileInputStream(f))) {
      OccupancyGrid g = OccupancyGrid.read(in);
      slam.setGrid(g);
      slam.setMapping(false); // pure localization: the map stays as saved
      slam.requestGlobalLocalization();
      mapLoaded = true;
      toast(R.string.slam_finding);
      RoomMap r = new RoomMap();
      File rf = new File(slamDir(), "rooms.json");
      if (rf.exists()) {
        byte[] b = new byte[(int) rf.length()];
        try (FileInputStream rin = new FileInputStream(rf)) {
          int at = 0;
          while (at < b.length) {
            int n = rin.read(b, at, b.length - at);
            if (n < 0) break;
            at += n;
          }
        }
        r = RoomMap.fromJson(new String(b, StandardCharsets.UTF_8));
      }
      r.robotRadius = navParams.robotRadius;
      String stamp = RoomMap.stampOf(g);
      boolean stale = !stamp.equals(r.mapStamp);
      setRooms(r);
      if (stale || !r.hasRooms()) detectRooms(); // a new or changed map: find its rooms again
    } catch (Exception e) {
      Timber.e(e, "Loading the map for Rooms & Zones failed");
      mapLoaded = false;
      statusDetail = e.getMessage();
    }
  }

  private void setRooms(RoomMap r) {
    rooms = r;
    navParams.keepOut = r.hasRestrictions() ? r : null;
    mission.setHome(r.home);
    roomsVersion++;
    if (navigator.getGoal() != null) { // re-plan around new restrictions
      double[] goal = navigator.getGoal();
      navigator.setGoal(goal[0], goal[1]);
    }
  }

  /** Call after any change to rooms, zones, restrictions or home. */
  private void roomsChanged() {
    RoomMap r = rooms;
    navParams.keepOut = r.hasRestrictions() ? r : null;
    mission.setHome(r.home);
    roomsVersion++;
    work(this::saveRooms);
  }

  private void saveRooms() {
    try (FileOutputStream out = new FileOutputStream(new File(slamDir(), "rooms.json"))) {
      out.write(rooms.toJson().getBytes(StandardCharsets.UTF_8));
    } catch (Exception e) {
      Timber.e(e, "Saving rooms failed");
    }
  }

  /** On the worker. Keeps zones, restrictions and home; names reset to Room 1, Room 2, ... */
  private void detectRooms() {
    if (!mapLoaded) {
      toast(R.string.rooms_no_map);
      return;
    }
    OccupancyGrid g = slam.getGrid();
    RoomSegmenter.Result res = RoomSegmenter.segment(g, new RoomSegmenter.Params());
    RoomMap r = rooms;
    r.setRooms(res, RoomMap.stampOf(g));
    selectedRooms.clear();
    roomsChanged();
    main.post(() -> {
      if (getContext() != null) {
        Toast.makeText(getContext(), getString(R.string.rooms_detected, res.count), Toast.LENGTH_SHORT).show();
      }
    });
  }

  // endregion

  // region Taps and drawing

  private void onTap(double x, double y) {
    RoomsMapView.Tool t = binding.map.tool;
    RoomMap r = rooms;
    if (t == RoomsMapView.Tool.GO_TO) {
      String err = goTo(x, y);
      if (err != null) toastText(err);
    } else if (t == RoomsMapView.Tool.HOME) {
      r.home = new double[] {x, y};
      roomsChanged();
      toast(R.string.rooms_home_set);
    } else if (t == RoomsMapView.Tool.SELECT) {
      int[] item = r.itemAt(x, y, 0.25);
      if (item != null) {
        int[] cur = selectedItem;
        selectedItem = cur != null && cur[0] == item[0] && cur[1] == item[1] ? null : item;
        selectedRooms.clear();
      } else {
        selectedItem = null;
        int id = r.roomAt(x, y);
        if (id > 0) {
          if (!selectedRooms.remove(id)) selectedRooms.add(id);
        } else {
          selectedRooms.clear();
        }
      }
    }
  }

  private void onShapeDrawn(RoomsMapView.Tool t, double ax, double ay, double bx, double by) {
    RoomMap r = rooms;
    if (t == RoomsMapView.Tool.ZONE) {
      r.zones.add(RoomMap.box(ax, ay, bx, by));
      selectedItem = new int[] {0, r.zones.size() - 1};
      selectedRooms.clear();
    } else if (t == RoomsMapView.Tool.NO_GO) {
      r.noGo.add(RoomMap.box(ax, ay, bx, by));
    } else if (t == RoomsMapView.Tool.WALL) {
      r.walls.add(new double[] {ax, ay, bx, by});
    }
    roomsChanged();
    if (t != RoomsMapView.Tool.ZONE) work(() -> setRooms(rooms)); // re-plan around it
  }

  private void deleteSelected() {
    int[] it = selectedItem;
    RoomMap r = rooms;
    if (it == null) {
      toast(R.string.rooms_delete_none);
      return;
    }
    List<double[]> list = it[0] == 0 ? r.zones : it[0] == 1 ? r.noGo : r.walls;
    if (it[1] < list.size()) list.remove(it[1]);
    selectedItem = null;
    roomsChanged();
  }

  private void showRename() {
    if (selectedRooms.size() != 1) {
      toast(R.string.rooms_pick_one);
      return;
    }
    int id = selectedRooms.iterator().next();
    EditText e = new EditText(requireContext());
    e.setText(rooms.name(id));
    e.setSelectAllOnFocus(true);
    new AlertDialog.Builder(requireContext())
        .setTitle(R.string.rooms_rename)
        .setView(e)
        .setPositiveButton(android.R.string.ok, (d, w) -> renameSegment(id, e.getText().toString()))
        .setNegativeButton(android.R.string.cancel, null)
        .show();
  }

  // endregion

  // region Loop

  private void step() {
    if (binding == null) return;
    LidarScan scan = lidarScanner.getLatestScan();
    boolean fresh = scan != null && lidarScanner.isFresh();
    if (!fresh) {
      binding.sourceInfo.setText(R.string.slam_needs_scan);
      if (driving) stopCar();
    } else {
      binding.sourceInfo.setText(getString(R.string.lidarbot_source_scan, scan.getValidPoints()));
      if (scan.getScanId() != lastScanId) {
        lastScanId = scan.getScanId();
        int[] mm = scan.getRangesMm();
        double yaw = imuYaw;
        if (busy.compareAndSet(false, true)) {
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
    Pose pose = slam.getPose();
    binding.map.setGrid(slam.getGrid());
    binding.map.setState(pose, lastAngles, lastRanges, slam.getTrail(), new ArrayList<>(navigator.getPath()), navigator.getGoal());
    binding.map.setRooms(rooms, roomsVersion, new LinkedHashSet<>(selectedRooms), selectedItem, mission.remaining());
    int seq = slam.getRelocalizeSeq();
    if (seq != shownRelocalizeSeq) {
      shownRelocalizeSeq = seq;
      toast(slam.wasRelocalizeFound() ? R.string.slam_found : R.string.slam_not_found);
    }
    showStatus(pose);
    long now = SystemClock.uptimeMillis();
    if (mqtt != null && now - lastMqttMs > 1000) {
      lastMqttMs = now;
      ValetudoMqtt m = mqtt;
      work(m::publishState);
    }
  }

  private void showStatus(Pose pose) {
    RoomMap r = rooms;
    CleaningMission.Status st = mission.getStatus(pose);
    String s = getString(statusText(st));
    if (!mission.getLabel().isEmpty()) s += " · " + mission.getLabel();
    if (st == CleaningMission.Status.ERROR) s += ": " + mission.getError();
    if (!mapLoaded) s = statusDetail;
    binding.status.setText(s);
    int here = r.roomAt(pose.x, pose.y);
    String where = here > 0 ? r.name(here) : "—";
    String lane = mission.getTotal() > 1 ? String.format(Locale.ROOT, " · %d / %d", mission.getIndex() + 1, mission.getTotal()) : "";
    binding.mapStatus.setText(where + lane + String.format(Locale.ROOT, " · %d%%", (int) Math.round(slam.getMatchQuality() * 100)));
    StringBuilder sel = new StringBuilder();
    synchronized (selectedRooms) {
      for (int id : selectedRooms) {
        if (sel.length() > 0) sel.append(", ");
        sel.append(r.name(id));
      }
    }
    int[] it = selectedItem;
    if (it != null) sel.append(getString(it[0] == 0 ? R.string.rooms_tool_zone : it[0] == 1 ? R.string.rooms_tool_nogo : R.string.rooms_tool_wall));
    binding.selection.setText(sel.length() == 0 ? (r.hasRooms() ? "" : getString(R.string.rooms_need_rooms)) : "▸ " + sel);
    binding.mqttInfo.setText(mqttStatus);
    boolean running = mission.isBusy();
    IconButtons.set(binding.startButton, running ? R.drawable.ic_btn_pause : R.drawable.ic_btn_play_arrow,
        running ? R.string.rooms_pause : st == CleaningMission.Status.PAUSED ? R.string.rooms_resume : R.string.rooms_start);
  }

  static int statusText(CleaningMission.Status s) {
    switch (s) {
      case DOCKED:
        return R.string.rooms_st_docked;
      case CLEANING:
        return R.string.rooms_st_cleaning;
      case MOVING:
        return R.string.rooms_st_moving;
      case RETURNING:
        return R.string.rooms_st_returning;
      case PAUSED:
        return R.string.rooms_st_paused;
      case ERROR:
        return R.string.rooms_st_error;
      default:
        return R.string.rooms_st_idle;
    }
  }

  /** On the worker: localize, then drive the job. */
  private void onScan(int[] rangesMm, double yaw) {
    int n = rangesMm.length;
    double[] a = new double[n];
    double[] r = new double[n];
    for (int i = 0; i < n; i++) {
      a[i] = -Math.toRadians((i + 0.5) * 360.0 / n);
      r[i] = rangesMm[i] == 0 ? 99 : rangesMm[i] / 1000.0;
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
    boolean wasBusy = mission.isBusy();
    Navigator.Command c = mission.step(pose, slam.getGrid(), frontMin, SystemClock.elapsedRealtime() / 1000.0);
    if (c != null) drive(c.throttle, c.steer);
    else if (wasBusy || driving) stopCar();
    if (!mission.isBusy()) activeZones = new ArrayList<>();
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

  // region Jobs (shared by the buttons, the REST API and MQTT)

  @Override
  public String basicControl(String action) {
    if (!mapLoaded) return getStringSafe(R.string.rooms_no_map);
    switch (action) {
      case "stop":
        work(() -> {
          mission.stop();
          stopCar();
        });
        return null;
      case "pause":
        work(() -> {
          mission.pause();
          stopCar();
        });
        return null;
      case "home":
        if (rooms.home == null) return "No home set: choose the home tool and tap the map";
        work(mission::returnHome);
        return null;
      case "start":
        if (mission.getStatus(null) == CleaningMission.Status.PAUSED) {
          work(mission::resume);
          return null;
        }
        int[] it = selectedItem;
        if (it != null && it[0] == 0 && it[1] < rooms.zones.size()) {
          List<double[]> z = new ArrayList<>();
          z.add(rooms.zones.get(it[1]));
          return cleanZones(z);
        }
        List<Integer> ids;
        synchronized (selectedRooms) {
          ids = new ArrayList<>(selectedRooms);
        }
        if (ids.isEmpty()) ids = new ArrayList<>(rooms.names.keySet()); // nothing chosen: every room
        if (ids.isEmpty()) return getStringSafe(R.string.rooms_need_rooms);
        return cleanSegments(ids);
      default:
        return "Unknown action";
    }
  }

  @Override
  public String goTo(double x, double y) {
    if (!mapLoaded) return getStringSafe(R.string.rooms_no_map);
    work(() -> {
      mission.clearError();
      mission.goTo(x, y);
    });
    return null;
  }

  @Override
  public String cleanSegments(List<Integer> ids) {
    if (!mapLoaded) return getStringSafe(R.string.rooms_no_map);
    RoomMap r = rooms;
    for (int id : ids) if (!r.names.containsKey(id)) return "No room " + id;
    double spacing = laneSpacing();
    work(() -> {
      List<double[]> all = new ArrayList<>();
      StringBuilder label = new StringBuilder();
      Pose from = slam.getPose();
      for (int id : ids) {
        List<double[]> wps = CoveragePlanner.plan(slam.getGrid(), navParams, (x, y) -> r.roomAt(x, y) == id, spacing, from);
        all.addAll(wps);
        if (!wps.isEmpty()) {
          double[] last = wps.get(wps.size() - 1);
          from = new Pose(last[0], last[1], 0);
        }
        if (label.length() > 0) label.append(", ");
        label.append(r.name(id));
      }
      mission.clearError();
      mission.start(all, CleaningMission.Flag.SEGMENT, label.toString());
    });
    return null;
  }

  @Override
  public String cleanZones(List<double[]> zones) {
    if (!mapLoaded) return getStringSafe(R.string.rooms_no_map);
    if (zones.isEmpty()) return "No zones";
    double spacing = laneSpacing();
    work(() -> {
      List<double[]> all = new ArrayList<>();
      Pose from = slam.getPose();
      for (double[] z : zones) {
        List<double[]> wps = CoveragePlanner.plan(slam.getGrid(), navParams, (x, y) -> RoomMap.inside(z, x, y), spacing, from);
        all.addAll(wps);
        if (!wps.isEmpty()) {
          double[] last = wps.get(wps.size() - 1);
          from = new Pose(last[0], last[1], 0);
        }
      }
      activeZones = new ArrayList<>(zones);
      mission.clearError();
      mission.start(all, CleaningMission.Flag.ZONE, zones.size() == 1 ? "Zone" : zones.size() + " zones");
    });
    return null;
  }

  @Override
  public String renameSegment(int id, String name) {
    RoomMap r = rooms;
    if (!r.names.containsKey(id)) return "No room " + id;
    String n = name.trim();
    r.names.put(id, n.isEmpty() ? "Room " + id : n);
    roomsChanged();
    return null;
  }

  @Override
  public String setRestrictions(List<double[]> walls, List<double[]> noGo) {
    RoomMap r = rooms;
    r.walls.clear();
    r.walls.addAll(walls);
    r.noGo.clear();
    r.noGo.addAll(noGo);
    selectedItem = null;
    roomsChanged();
    work(() -> setRooms(rooms));
    return null;
  }

  /** Valetudo's "locate": blink the indicators left / right and beep for about 3 s. */
  @Override
  public String locate() {
    main.post(() -> {
      ToneGenerator tone = null;
      try {
        tone = new ToneGenerator(AudioManager.STREAM_NOTIFICATION, 90);
      } catch (RuntimeException e) {
        Timber.w(e, "No tone generator");
      }
      final ToneGenerator t = tone;
      for (int k = 0; k < 6; k++) {
        final int side = k % 2 == 0 ? -1 : 1;
        main.postDelayed(() -> {
          if (vehicle != null) vehicle.setIndicator(side);
          if (t != null) t.startTone(ToneGenerator.TONE_PROP_BEEP, 150);
        }, k * 500L);
      }
      main.postDelayed(() -> {
        if (vehicle != null) vehicle.setIndicator(0);
        if (t != null) t.release();
      }, 3200);
    });
    return null;
  }

  private double laneSpacing() {
    return prefs().getFloat("lane_spacing", 0.5f);
  }

  private String getStringSafe(int res) {
    Context c = getContext();
    return c == null ? "Not ready" : c.getString(res);
  }

  // endregion

  // region ValetudoApi.Robot getters (any thread)

  @Override
  public OccupancyGrid grid() {
    return mapLoaded ? slam.getGrid() : null;
  }

  @Override
  public RoomMap rooms() {
    return rooms;
  }

  @Override
  public Pose pose() {
    return slam.getPose();
  }

  @Override
  public CleaningMission.Status status() {
    return mapLoaded ? mission.getStatus(slam.getPose()) : CleaningMission.Status.ERROR;
  }

  @Override
  public CleaningMission.Flag flag() {
    return mission.getFlag();
  }

  @Override
  public String error() {
    return mapLoaded ? mission.getError() : "No saved map: build one in Indoor SLAM";
  }

  @Override
  public List<double[]> plannedPath() {
    return new ArrayList<>(navigator.getPath());
  }

  @Override
  public double[] goToTarget() {
    return mission.getFlag() == CleaningMission.Flag.TARGET ? mission.target() : null;
  }

  @Override
  public List<double[]> activeZones() {
    return activeZones;
  }

  /** Valetudo REST request from the Sensor link server. */
  @Override
  public String[] handle(String method, String path, String body) {
    ValetudoApi.Response r = api.handle(method, path, body);
    return new String[] {String.valueOf(r.status), r.contentType, r.body};
  }

  // endregion

  // region Settings and MQTT

  private SharedPreferences prefs() {
    return requireContext().getSharedPreferences(PREFS, Context.MODE_PRIVATE);
  }

  private ValetudoMqtt.Settings mqttSettings() {
    SharedPreferences p = prefs();
    ValetudoMqtt.Settings s = new ValetudoMqtt.Settings();
    s.enabled = p.getBoolean("mqtt_enabled", false);
    s.host = p.getString("mqtt_host", "");
    s.port = p.getInt("mqtt_port", 1883);
    s.username = p.getString("mqtt_user", "");
    s.password = p.getString("mqtt_pass", "");
    s.prefix = p.getString("mqtt_prefix", "valetudo");
    s.identifier = p.getString("mqtt_id", "openbot");
    s.homeAssistant = p.getBoolean("mqtt_hass", true);
    return s;
  }

  private void startMqtt() {
    ValetudoMqtt.Settings s = mqttSettings();
    if (!s.enabled || s.host.trim().isEmpty()) return;
    mqtt = new ValetudoMqtt(this, api, s, this);
    mqtt.start();
  }

  private void stopMqtt() {
    ValetudoMqtt m = mqtt;
    mqtt = null;
    if (m != null) new Thread(m::stop, "mqtt-stop").start();
    mqttStatus = "";
  }

  @Override
  public void onMqttStatus(String text) {
    mqttStatus = text;
  }

  private void showSettings() {
    Context c = requireContext();
    ValetudoMqtt.Settings s = mqttSettings();
    int pad = (int) (16 * getResources().getDisplayMetrics().density);
    LinearLayout box = new LinearLayout(c);
    box.setOrientation(LinearLayout.VERTICAL);
    box.setPadding(pad, pad / 2, pad, 0);
    EditText spacing = field(box, R.string.rooms_lane_spacing, String.format(Locale.ROOT, "%.2f", laneSpacing()),
        InputType.TYPE_CLASS_NUMBER | InputType.TYPE_NUMBER_FLAG_DECIMAL);
    TextView head = new TextView(c);
    head.setText(R.string.rooms_mqtt);
    head.setTypeface(android.graphics.Typeface.DEFAULT_BOLD);
    head.setPadding(0, pad, 0, 0);
    box.addView(head);
    SwitchCompat enabled = new SwitchCompat(c);
    enabled.setText(R.string.rooms_mqtt_enable);
    enabled.setChecked(s.enabled);
    box.addView(enabled);
    EditText host = field(box, R.string.rooms_mqtt_host, s.host, InputType.TYPE_CLASS_TEXT | InputType.TYPE_TEXT_VARIATION_URI);
    EditText port = field(box, R.string.rooms_mqtt_port, String.valueOf(s.port), InputType.TYPE_CLASS_NUMBER);
    EditText user = field(box, R.string.rooms_mqtt_user, s.username, InputType.TYPE_CLASS_TEXT);
    EditText pass = field(box, R.string.rooms_mqtt_pass, s.password, InputType.TYPE_CLASS_TEXT | InputType.TYPE_TEXT_VARIATION_PASSWORD);
    EditText prefix = field(box, R.string.rooms_mqtt_prefix, s.prefix, InputType.TYPE_CLASS_TEXT);
    EditText id = field(box, R.string.rooms_mqtt_id, s.identifier, InputType.TYPE_CLASS_TEXT);
    SwitchCompat hass = new SwitchCompat(c);
    hass.setText(R.string.rooms_mqtt_hass);
    hass.setChecked(s.homeAssistant);
    box.addView(hass);
    android.widget.ScrollView scroll = new android.widget.ScrollView(c);
    scroll.addView(box);
    new AlertDialog.Builder(c)
        .setTitle(R.string.rooms_settings)
        .setView(scroll)
        .setPositiveButton(android.R.string.ok, (d, w) -> {
          float sp;
          try {
            sp = Float.parseFloat(spacing.getText().toString().trim());
          } catch (NumberFormatException e) {
            sp = 0.5f;
          }
          int pt;
          try {
            pt = Integer.parseInt(port.getText().toString().trim());
          } catch (NumberFormatException e) {
            pt = 1883;
          }
          prefs().edit()
              .putFloat("lane_spacing", Math.max(0.25f, Math.min(1.5f, sp)))
              .putBoolean("mqtt_enabled", enabled.isChecked())
              .putString("mqtt_host", host.getText().toString().trim())
              .putInt("mqtt_port", pt)
              .putString("mqtt_user", user.getText().toString())
              .putString("mqtt_pass", pass.getText().toString())
              .putString("mqtt_prefix", prefix.getText().toString().trim())
              .putString("mqtt_id", id.getText().toString().trim())
              .putBoolean("mqtt_hass", hass.isChecked())
              .apply();
          stopMqtt();
          startMqtt();
        })
        .setNegativeButton(android.R.string.cancel, null)
        .show();
  }

  private EditText field(LinearLayout box, int label, String value, int type) {
    TextView t = new TextView(requireContext());
    t.setText(label);
    t.setPadding(0, (int) (8 * getResources().getDisplayMetrics().density), 0, 0);
    box.addView(t);
    EditText e = new EditText(requireContext());
    e.setText(value);
    e.setInputType(type);
    e.setSingleLine(true);
    box.addView(e);
    return e;
  }

  /** The SLAM, planner and controller chosen on Indoor SLAM (Algorithms) apply here too. */
  private void applySavedAlgorithms() {
    SharedPreferences p = requireContext().getSharedPreferences("slam_algorithms", Context.MODE_PRIVATE);
    work(() -> {
      slam.setEngine(saved(p, "slam", Slam.Type.CARTOGRAPHER));
      slam.setLocalizer(saved(p, "localizer", SlamSystem.Localizer.SCAN_MATCHING));
      navParams.planner = saved(p, "planner", Navigator.Planner.NAVFN);
      navParams.controller = saved(p, "controller", Navigator.Controller.RPP);
    });
  }

  private static <E extends Enum<E>> E saved(SharedPreferences p, String key, E def) {
    try {
      return Enum.valueOf(def.getDeclaringClass(), p.getString(key, def.name()));
    } catch (IllegalArgumentException e) {
      return def;
    }
  }

  @Override
  protected void addScreenSettings(List<RemoteSettings.Setting> out) {
    out.add(RemoteSettings.Setting.action("rooms.start", "Rooms & Zones", "Patrol the selected rooms (or all)",
        "START", "PAUSE", mission::isBusy, v -> main.post(() -> basicControl(mission.isBusy() ? "pause" : "start"))));
    out.add(RemoteSettings.Setting.action("rooms.home", "Rooms & Zones", "Return home", "HOME", "HOME", () -> false,
        v -> main.post(() -> basicControl("home"))));
    out.add(RemoteSettings.Setting.action("rooms.locate", "Rooms & Zones", "Locate", "LOCATE", "LOCATE", () -> false,
        v -> locate()));
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
