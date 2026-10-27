package org.openbot.app.robot.gpsNav;

import android.content.Context;
import android.content.SharedPreferences;
import android.content.res.ColorStateList;
import android.graphics.Color;
import android.location.Location;
import android.net.Uri;
import android.net.ConnectivityManager;
import android.net.Network;
import android.net.NetworkCapabilities;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.preference.PreferenceManager;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Toast;
import androidx.activity.result.ActivityResultLauncher;
import androidx.activity.result.contract.ActivityResultContracts;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.recyclerview.widget.LinearLayoutManager;
import com.google.android.material.bottomsheet.BottomSheetBehavior;
import com.google.android.material.dialog.MaterialAlertDialogBuilder;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.openbot.app.robot.R;
import org.openbot.app.robot.common.ControlsFragment;
import org.openbot.app.robot.databinding.FragmentGpsNavigationBinding;
import org.openbot.app.robot.sensorlink.OnDemandCamera;
import org.openbot.app.robot.sensorlink.SensorLink;
import org.openbot.app.robot.utils.Constants;
import org.openbot.app.robot.utils.PermissionUtils;
import timber.log.Timber;

/**
 * Mission Planner style GPS navigation.
 *
 * <p>Plans a waypoint mission on the map (or at the current phone GPS position), then drives the
 * robot through it using the phone's GPS for position and heading. Supports the three navigation
 * modes a ground station needs: run the whole mission, guide to a single point, and return to
 * launch.
 *
 * <p>Unlike the ARCore {@code pointGoalNavigation} module this is purely geographic, so it works
 * outdoors without a camera feed.
 */
public class GpsNavigationFragment extends ControlsFragment
    implements WaypointAdapter.OnWaypointActionListener,
        GpsNavLocationProvider.Listener,
        MissionMapView.OnMapTapListener {

  /** Navigation modes, mirroring Mission Planner's AUTO / GUIDED / RTL. */
  private enum NavMode {
    IDLE,
    MISSION,
    GUIDED,
    RTL
  }

  /** Positions worse than this make the guidance loop hold instead of driving. */
  private static final float MAX_POSITION_ACCURACY_M = 15f;
  // A fix older than this is not steered on: the provider stopped reporting (GPS lost, permission
  // revoked, provider disabled) and the last position is no longer where the robot is.
  private static final long FIX_TIMEOUT_MS = 3000;
  private long lastFixElapsedMs = 0;
  private ActivityResultLauncher<String> exportWaypointsLauncher;
  private ActivityResultLauncher<String[]> importWaypointsLauncher;

  private static final long NAV_PERIOD_MS = 200L;
  private static final String MISSION_FOLDER = "missions";
  private static final String MISSION_FILE = "mission.waypoints";

  private FragmentGpsNavigationBinding binding;
  private final Mission mission = new Mission();
  private final GuidanceController guidance = new GuidanceController();
  private final List<double[]> track = new ArrayList<>();

  private GpsNavLocationProvider locationProvider;
  private WaypointAdapter adapter;
  private ActivityResultLauncher<String> locationPermissionLauncher;

  private NavMode navMode = NavMode.IDLE;
  private boolean paused = false;
  private int activeWaypointIndex = -1;
  private Waypoint guidedTarget = null;

  private Location lastLocation;
  private float headingDegrees = Float.NaN;

  private final Handler navHandler = new Handler(Looper.getMainLooper());
  private boolean navLoopRunning = false;

  private final Runnable navLoop =
      new Runnable() {
        @Override
        public void run() {
          stepNavigation();
          if (navLoopRunning) {
            navHandler.postDelayed(this, NAV_PERIOD_MS);
          }
        }
      };

  @Override
  public void onCreate(@Nullable Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    locationPermissionLauncher =
        registerForActivityResult(
            new ActivityResultContracts.RequestPermission(),
            granted -> {
              if (granted) {
                startLocationProvider();
              } else {
                Toast.makeText(
                        requireContext(), R.string.gps_nav_permission_needed, Toast.LENGTH_LONG)
                    .show();
              }
            });
    // Mission Planner interchange goes through the system file picker, so a .waypoints file can be
    // shared with a PC, Drive or another phone - the app's own files dir is not reachable from there.
    exportWaypointsLauncher =
        registerForActivityResult(
            new ActivityResultContracts.CreateDocument("application/octet-stream"),
            this::exportWaypointsTo);
    importWaypointsLauncher =
        registerForActivityResult(
            new ActivityResultContracts.OpenDocument(), this::importWaypointsFrom);
  }

  @Override
  public View onCreateView(
      @NonNull LayoutInflater inflater,
      @Nullable ViewGroup container,
      @Nullable Bundle savedInstanceState) {
    binding = FragmentGpsNavigationBinding.inflate(inflater, container, false);
    return binding.getRoot();
  }

  @Override
  public void onViewCreated(@NonNull View view, @Nullable Bundle savedInstanceState) {
    super.onViewCreated(view, savedInstanceState);

    adapter = new WaypointAdapter(this);
    binding.waypointList.setLayoutManager(new LinearLayoutManager(requireContext()));
    binding.waypointList.setAdapter(adapter);

    binding.mapView.setOnMapTapListener(this);
    binding.mapTypeButton.setOnClickListener(v -> showMapTypeDialog());
    restoreMapType();
    warnIfMapTilesUnavailable();

    locationProvider = new GpsNavLocationProvider(requireContext(), this);

    binding.startPauseButton.setOnClickListener(v -> onStartPausePressed());
    binding.stopButton.setOnClickListener(v -> stopNavigation(R.string.gps_nav_status_idle));
    binding.rtlButton.setOnClickListener(v -> startReturnToLaunch());
    binding.addWaypointButton.setOnClickListener(v -> addWaypointAtCurrentPosition());
    binding.clearButton.setOnClickListener(v -> clearMission());
    binding.saveButton.setOnClickListener(v -> saveMission());
    binding.exportButton.setOnClickListener(v -> exportKml());
    binding.importWaypointsButton.setOnClickListener(
        v -> importWaypointsLauncher.launch(new String[] {"*/*"}));
    binding.exportWaypointsButton.setOnClickListener(
        v -> exportWaypointsLauncher.launch(MISSION_FILE));

    loadMission();
    refreshMissionUi();
    updateHud(null);
    setupObjectAvoidanceControls(
        binding.objectAvoidance.avoidToggle, binding.objectAvoidance.avoidInfo);
    setupSensorLinkControls(binding.sensorLink.getRoot());
    setupPanel();
  }

  private static final String PREF_PANEL_OPEN = "gps_mission_panel_open";

  /**
   * The bottom sheet works like Object Tracking's: drag it, tap the handle, or tap "Hide data".
   * Collapsed, it still shows the header (status, connection, Start / Stop / Return home), so a
   * running mission can always be paused or stopped. The choice is remembered.
   */
  private void setupPanel() {
    SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(requireContext());
    BottomSheetBehavior<View> sheet = BottomSheetBehavior.from(binding.missionBottomSheet);
    binding.sheetHeader.post(
        () -> {
          if (binding != null) sheet.setPeekHeight(binding.sheetHeader.getHeight());
        });
    boolean open = prefs.getBoolean(PREF_PANEL_OPEN, true);
    sheet.setState(open ? BottomSheetBehavior.STATE_EXPANDED : BottomSheetBehavior.STATE_COLLAPSED);
    showPanelLabel(open);
    sheet.addBottomSheetCallback(
        new BottomSheetBehavior.BottomSheetCallback() {
          @Override
          public void onStateChanged(@NonNull View bottomSheet, int newState) {
            if (newState == BottomSheetBehavior.STATE_EXPANDED
                || newState == BottomSheetBehavior.STATE_COLLAPSED) {
              boolean nowOpen = newState == BottomSheetBehavior.STATE_EXPANDED;
              prefs.edit().putBoolean(PREF_PANEL_OPEN, nowOpen).apply();
              showPanelLabel(nowOpen);
            }
          }

          @Override
          public void onSlide(@NonNull View bottomSheet, float slideOffset) {}
        });
    View.OnClickListener toggle =
        v ->
            sheet.setState(
                sheet.getState() == BottomSheetBehavior.STATE_EXPANDED
                    ? BottomSheetBehavior.STATE_COLLAPSED
                    : BottomSheetBehavior.STATE_EXPANDED);
    binding.sheetHandleArea.setOnClickListener(toggle);
    binding.togglePanelButton.setOnClickListener(toggle);
    updateConnectionIcon();
  }

  private void showPanelLabel(boolean open) {
    if (binding == null) return;
    binding.togglePanelButton.setText(open ? R.string.gps_panel_hide : R.string.gps_panel_show);
  }

  /** USB or Bluetooth icon, green while the car is connected, grey otherwise. */
  private void updateConnectionIcon() {
    if (binding == null) return;
    boolean bluetooth = vehicle != null && "Bluetooth".equals(vehicle.getConnectionType());
    boolean connected = vehicle != null && vehicle.isConnected();
    binding.connectionIcon.setImageResource(bluetooth ? R.drawable.ic_bluetooth : R.drawable.ic_usb);
    binding.connectionIcon.setImageTintList(
        ColorStateList.valueOf(connected ? Color.rgb(0x2E, 0x7D, 0x32) : Color.rgb(0x9E, 0x9E, 0x9E)));
  }

  @Override
  public void onResume() {
    super.onResume();
    if (binding != null && binding.mapView != null) {
      binding.mapView.onResume();
    }
    requestLocationAndStart();
    SensorLink.get().addRemoteListener(remoteSensors);
    // Opens the camera only while a browser watches it (Sensor link page).
    if (browserCamera == null) browserCamera = new OnDemandCamera(requireContext(), this);
    browserCamera.start();
  }

  private OnDemandCamera browserCamera;

  @Override
  protected String remoteScreenName() {
    return "GPS Mission";
  }

  /** Sensor link client: GPS and compass from a SensorServer app on another phone. */
  private final SensorLink.RemoteListener remoteSensors =
      new SensorLink.RemoteListener() {
        @Override
        public void onRemoteLocation(Location location) {
          if (locationProvider != null) locationProvider.onRemoteLocation(location);
        }

        @Override
        public void onRemoteRotationVector(float[] values) {
          if (locationProvider != null) locationProvider.onRemoteRotationVector(values);
        }
      };

  @Override
  public void onPause() {
    SensorLink.get().removeRemoteListener(remoteSensors);
    if (browserCamera != null) browserCamera.stop();
    // Never leave the wheels spinning while the screen is away.
    stopNavigation(R.string.gps_nav_status_idle);
    if (locationProvider != null) {
      locationProvider.stop();
    }
    if (binding != null && binding.mapView != null) {
      binding.mapView.onPause();
    }
    super.onPause();
  }

  @Override
  public void onDestroyView() {
    navHandler.removeCallbacks(navLoop);
    if (locationProvider != null) {
      locationProvider.stop();
      locationProvider = null;
    }
    binding = null;
    super.onDestroyView();
  }

  // region ControlsFragment

  @Override
  protected void processControllerKeyData(String command) {
    // Manual input takes priority over autonomy, the way a joystick overrides a Mission Planner
    // auto mission. ControlsFragment has already applied the drive command by this point.
    if (navMode != NavMode.IDLE && Constants.CMD_DRIVE.equals(command)) {
      stopNavigation(R.string.gps_nav_status_idle);
    }
  }

  @Override
  protected void processUSBData(String data) {
    // The HUD is fed from GPS (distance, bearing, ground speed) rather than vehicle telemetry.
    // Object avoidance status still needs the telemetry stream, since that is where the LiDAR
    // proximity reports arrive.
    refreshObjectAvoidanceStatus(binding.objectAvoidance.avoidInfo);
  }

  // endregion

  // region Location

  private void requestLocationAndStart() {
    if (PermissionUtils.hasLocationPermission(requireActivity())) {
      startLocationProvider();
    } else {
      locationPermissionLauncher.launch(Constants.PERMISSION_LOCATION);
    }
  }

  private void startLocationProvider() {
    if (locationProvider != null) {
      locationProvider.start(requireContext());
    }
  }

  @Override
  public void onLocationUpdate(Location location) {
    if (location == null || binding == null) {
      return;
    }
    lastLocation = location;
    lastFixElapsedMs = SystemClock.elapsedRealtime();

    // Keep a breadcrumb trail of where we have actually been.
    if (track.isEmpty()
        || GeoMath.distanceMeters(
                track.get(track.size() - 1)[0],
                track.get(track.size() - 1)[1],
                location.getLatitude(),
                location.getLongitude())
            > 0.5) {
      track.add(new double[] {location.getLatitude(), location.getLongitude()});
    }

    binding.mapView.setRobot(location.getLatitude(), location.getLongitude(), headingDegrees);
    binding.mapView.setTrack(track);
    if (track.size() % 10 == 0) {
      adapter.setRobotPosition(location.getLatitude(), location.getLongitude());
    }
    updateHud(location);
  }

  @Override
  public void onHeadingUpdate(float newHeadingDegrees) {
    headingDegrees = newHeadingDegrees;
    if (binding != null && lastLocation != null) {
      binding.mapView.setRobot(
          lastLocation.getLatitude(), lastLocation.getLongitude(), headingDegrees);
    }
  }

  @Override
  public void onLocationUnavailable(String reason) {
    Timber.w("GPS navigation location unavailable: %s", reason);
    if (binding != null) {
      binding.hudStatus.setText(R.string.gps_nav_status_waiting_gps);
    }
  }

  // endregion

  // region Navigation

  private void onStartPausePressed() {
    if (navMode == NavMode.IDLE) {
      startMission();
    } else if (paused) {
      paused = false;
      binding.startPauseButton.setText(R.string.gps_nav_pause);
      binding.hudStatus.setText(R.string.gps_nav_status_navigating);
    } else {
      paused = true;
      haltMotors();
      binding.startPauseButton.setText(R.string.gps_nav_resume);
      binding.hudStatus.setText(R.string.gps_nav_status_paused);
    }
  }

  private void startMission() {
    if (!ensureReadyToNavigate()) {
      return;
    }
    if (mission.isEmpty()) {
      Toast.makeText(requireContext(), R.string.gps_nav_need_waypoints, Toast.LENGTH_SHORT).show();
      return;
    }
    if (!mission.hasHome() && lastLocation != null) {
      mission.setHome(lastLocation.getLatitude(), lastLocation.getLongitude());
      binding.mapView.setHome(mission.getHomeLatitude(), mission.getHomeLongitude());
    }

    navMode = NavMode.MISSION;
    paused = false;
    guidedTarget = null;
    activeWaypointIndex = 0;
    binding.startPauseButton.setText(R.string.gps_nav_pause);
    binding.hudStatus.setText(R.string.gps_nav_status_navigating);
    refreshMissionUi();
    startNavLoop();
  }

  private void startReturnToLaunch() {
    if (!ensureReadyToNavigate()) {
      return;
    }
    if (!mission.hasHome()) {
      Toast.makeText(requireContext(), R.string.gps_nav_need_fix, Toast.LENGTH_SHORT).show();
      return;
    }
    navMode = NavMode.RTL;
    paused = false;
    guidedTarget = null;
    activeWaypointIndex = -1;
    binding.startPauseButton.setText(R.string.gps_nav_pause);
    binding.hudStatus.setText(R.string.gps_nav_status_returning);
    refreshMissionUi();
    startNavLoop();
  }

  private void startGuided(Waypoint target) {
    if (!ensureReadyToNavigate() || target == null) {
      return;
    }
    if (!mission.hasHome() && lastLocation != null) {
      mission.setHome(lastLocation.getLatitude(), lastLocation.getLongitude());
      binding.mapView.setHome(mission.getHomeLatitude(), mission.getHomeLongitude());
    }
    navMode = NavMode.GUIDED;
    paused = false;
    guidedTarget = target;
    activeWaypointIndex = -1;
    binding.startPauseButton.setText(R.string.gps_nav_pause);
    binding.hudStatus.setText(R.string.gps_nav_status_navigating);
    refreshMissionUi();
    startNavLoop();
    Toast.makeText(requireContext(), R.string.gps_nav_guided_set, Toast.LENGTH_SHORT).show();
  }

  private boolean ensureReadyToNavigate() {
    if (lastLocation == null) {
      Toast.makeText(requireContext(), R.string.gps_nav_need_fix, Toast.LENGTH_SHORT).show();
      return false;
    }
    if (vehicle == null) {
      Toast.makeText(requireContext(), R.string.gps_nav_vehicle_not_ready, Toast.LENGTH_SHORT)
          .show();
      return false;
    }
    return true;
  }

  private void startNavLoop() {
    if (!navLoopRunning) {
      navLoopRunning = true;
      navHandler.post(navLoop);
    }
  }

  private void stopNavigation(int statusStringRes) {
    navMode = NavMode.IDLE;
    paused = false;
    guidedTarget = null;
    activeWaypointIndex = -1;
    navLoopRunning = false;
    navHandler.removeCallbacks(navLoop);
    haltMotors();
    if (binding != null) {
      binding.startPauseButton.setText(R.string.gps_nav_start);
      binding.hudStatus.setText(statusStringRes);
      binding.mapView.setWaypoints(mission.getWaypoints(), -1);
    }
    refreshMissionUi();
  }

  private void haltMotors() {
    if (vehicle != null) {
      vehicle.setControl(0, 0);
    }
  }

  /** One guidance tick: pick the target, steer toward it, advance on arrival. */
  private void stepNavigation() {
    if (binding == null || navMode == NavMode.IDLE || paused) {
      haltMotors();
      return;
    }
    if (lastLocation == null) {
      haltMotors();
      return;
    }
    if (SystemClock.elapsedRealtime() - lastFixElapsedMs > FIX_TIMEOUT_MS) {
      haltMotors();
      binding.hudStatus.setText(R.string.gps_nav_fix_lost);
      return;
    }

    // GPS that has drifted indoors or under trees will happily drive the robot into a wall.
    if (lastLocation.hasAccuracy() && lastLocation.getAccuracy() > MAX_POSITION_ACCURACY_M) {
      haltMotors();
      binding.hudStatus.setText(
          getString(R.string.gps_nav_accuracy_poor, lastLocation.getAccuracy()));
      return;
    }

    Waypoint target = currentTarget();
    if (target == null) {
      completeNavigation();
      return;
    }

    if (Float.isNaN(headingDegrees)) {
      // Wait for a heading rather than guessing; the robot would otherwise drive off at random.
      haltMotors();
      return;
    }

    GuidanceController.Result result =
        guidance.compute(
            lastLocation.getLatitude(), lastLocation.getLongitude(), headingDegrees, target);

    if (result.isArrived()) {
      onTargetReached();
      return;
    }

    if (vehicle != null) {
      vehicle.setControl(result.getLeft(), result.getRight());

      // Object avoidance can veto that command. The waypoint must not be marked complete while
      // the car is being held back - it has not actually got there. Flag the hold in the HUD so
      // the mission reads as stalled, and let the next tick try again once the obstacle clears.
      // This is the Rover behaviour from ArduPilot: the vehicle stops before the barrier rather
      // than pushing through to the WP. The distance detail lives on the avoidance status line
      // in the sheet, so the two do not say the same thing twice.
    }
    showDrivingStatus();
  }

  /**
   * Status line for a tick that is actually driving. Written every tick so a message that no longer
   * applies (obstacle gone, fix back, compass calibrated) clears by itself instead of sticking.
   */
  private void showDrivingStatus() {
    if (vehicle != null && vehicle.isObjectAvoidanceHolding()) {
      binding.hudStatus.setText(R.string.gps_nav_status_held);
    } else if (locationProvider != null
        && !locationProvider.isHeadingFromCourse()
        && !locationProvider.isCompassReliable()) {
      // Standing still or creeping, so steering is on the compass - and the compass is off.
      // Not a hold: once the car is moving GPS course takes over and this clears.
      binding.hudStatus.setText(R.string.gps_nav_compass_unreliable);
    } else {
      binding.hudStatus.setText(
          navMode == NavMode.RTL
              ? R.string.gps_nav_status_returning
              : R.string.gps_nav_status_navigating);
    }
  }

  @Nullable
  private Waypoint currentTarget() {
    switch (navMode) {
      case MISSION:
        return mission.getWaypoint(activeWaypointIndex);
      case GUIDED:
      case RTL:
        return guidedTarget;
      case IDLE:
      default:
        return null;
    }
  }

  private void onTargetReached() {
    haltMotors();
    if (navMode == NavMode.MISSION) {
      int reachedIndex = activeWaypointIndex;
      activeWaypointIndex++;
      if (activeWaypointIndex >= mission.size()) {
        completeNavigation();
      } else {
        Toast.makeText(
                requireContext(),
                getString(R.string.gps_nav_arrived, reachedIndex + 1),
                Toast.LENGTH_SHORT)
            .show();
        refreshMissionUi();
      }
      return;
    }
    completeNavigation();
  }

  private void completeNavigation() {
    NavMode finished = navMode;
    int status;
    int toast;
    switch (finished) {
      case RTL:
        status = R.string.gps_nav_rtl_complete;
        toast = R.string.gps_nav_rtl_complete;
        break;
      case MISSION:
        status = R.string.gps_nav_status_completed;
        toast = R.string.gps_nav_mission_complete;
        break;
      default:
        status = R.string.gps_nav_status_completed;
        toast = R.string.gps_nav_status_completed;
        break;
    }
    stopNavigation(status);
    Toast.makeText(requireContext(), toast, Toast.LENGTH_SHORT).show();
  }

  // endregion

  // region Mission editing

  @Override
  public void onMapTap(double latitude, double longitude) {
    String message =
        getString(R.string.gps_nav_tap_dialog_message, formatLat(latitude), formatLon(longitude));
    new MaterialAlertDialogBuilder(requireContext())
        .setTitle(R.string.gps_nav_tap_dialog_title)
        .setMessage(message)
        .setPositiveButton(
            R.string.gps_nav_tap_add_waypoint,
            (dialog, which) -> {
              mission.addWaypoint(new Waypoint(latitude, longitude));
              onMissionChanged();
              Toast.makeText(
                      requireContext(),
                      getString(R.string.gps_nav_waypoint_added, mission.size()),
                      Toast.LENGTH_SHORT)
                  .show();
            })
        .setNegativeButton(
            R.string.gps_nav_tap_go_here,
            (dialog, which) -> startGuided(new Waypoint(latitude, longitude)))
        .setNeutralButton(R.string.gps_nav_cancel, null)
        .show();
  }

  private void addWaypointAtCurrentPosition() {
    if (lastLocation == null) {
      Toast.makeText(requireContext(), R.string.gps_nav_no_fix, Toast.LENGTH_SHORT).show();
      return;
    }
    mission.addWaypoint(new Waypoint(lastLocation.getLatitude(), lastLocation.getLongitude()));
    onMissionChanged();
    Toast.makeText(
            requireContext(),
            getString(R.string.gps_nav_waypoint_added, mission.size()),
            Toast.LENGTH_SHORT)
        .show();
  }

  private void clearMission() {
    mission.clearWaypoints();
    onMissionChanged();
    Toast.makeText(requireContext(), R.string.gps_nav_mission_cleared, Toast.LENGTH_SHORT).show();
  }

  @Override
  public void onWaypointDelete(int position) {
    if (position < 0 || position >= mission.size()) {
      return;
    }
    mission.removeWaypoint(position);
    if (activeWaypointIndex >= mission.size()) {
      activeWaypointIndex = mission.size() - 1;
    }
    onMissionChanged();
    Toast.makeText(requireContext(), R.string.gps_nav_waypoint_removed, Toast.LENGTH_SHORT).show();
  }

  @Override
  public void onWaypointSelect(int position) {
    if (position < 0 || position >= mission.size()) {
      return;
    }
    activeWaypointIndex = position;
    refreshMissionUi();
  }

  // region Map type

  private static final String PREF_MAP_TYPE = "gps_nav_map_type";

  /** Mission Planner's map provider dropdown, as a single-choice dialog. */
  private void showMapTypeDialog() {
    final MissionMapView.MapType[] types = MissionMapView.MapType.values();
    final String[] labels = {
      getString(R.string.gps_nav_map_type_satellite),
      getString(R.string.gps_nav_map_type_hybrid),
      getString(R.string.gps_nav_map_type_street),
      getString(R.string.gps_nav_map_type_terrain)
    };
    int checked = binding.mapView.getMapType().ordinal();

    new MaterialAlertDialogBuilder(requireContext())
        .setTitle(R.string.gps_nav_map_type)
        .setSingleChoiceItems(
            labels,
            checked,
            (dialog, which) -> {
              applyMapType(types[which]);
              dialog.dismiss();
            })
        .setNegativeButton(R.string.gps_nav_cancel, null)
        .show();
  }

  private void applyMapType(MissionMapView.MapType type) {
    binding.mapView.setMapType(type);
    binding.mapTypeButton.setText(labelFor(type));
    PreferenceManager.getDefaultSharedPreferences(requireContext())
        .edit()
        .putString(PREF_MAP_TYPE, type.name())
        .apply();
  }

  private void restoreMapType() {
    SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(requireContext());
    MissionMapView.MapType type = MissionMapView.MapType.SATELLITE;
    String saved = prefs.getString(PREF_MAP_TYPE, null);
    if (saved != null) {
      try {
        type = MissionMapView.MapType.valueOf(saved);
      } catch (IllegalArgumentException ignored) {
        // Unknown value from an older build; fall back to satellite.
      }
    }
    binding.mapView.setMapType(type);
    binding.mapTypeButton.setText(labelFor(type));
  }

  private String labelFor(MissionMapView.MapType type) {
    switch (type) {
      case HYBRID:
        return getString(R.string.gps_nav_map_type_hybrid);
      case STREET:
        return getString(R.string.gps_nav_map_type_street);
      case TERRAIN:
        return getString(R.string.gps_nav_map_type_terrain);
      case SATELLITE:
      default:
        return getString(R.string.gps_nav_map_type_satellite);
    }
  }

  /**
   * Map tiles are fetched over HTTPS. When the network is not validated — a captive portal on guest
   * Wi-Fi is the common case — every tile request is refused and the map is left blank, so say why
   * rather than showing an empty grid.
   */
  private void warnIfMapTilesUnavailable() {
    Context context = requireContext();
    ConnectivityManager cm =
        (ConnectivityManager) context.getSystemService(Context.CONNECTIVITY_SERVICE);
    if (cm == null) {
      return;
    }
    Network network = cm.getActiveNetwork();
    NetworkCapabilities caps = network == null ? null : cm.getNetworkCapabilities(network);
    boolean validated =
        caps != null
            && caps.hasCapability(NetworkCapabilities.NET_CAPABILITY_INTERNET)
            && caps.hasCapability(NetworkCapabilities.NET_CAPABILITY_VALIDATED);
    if (!validated) {
      Toast.makeText(context, R.string.gps_nav_map_no_tiles, Toast.LENGTH_LONG).show();
    }
  }

  // endregion

  private void onMissionChanged() {
    saveMission();
    refreshMissionUi();
  }

  private void refreshMissionUi() {
    if (binding == null) {
      return;
    }
    adapter.setWaypoints(mission.getWaypoints());
    adapter.setActivePosition(activeWaypointIndex);
    if (lastLocation != null) {
      adapter.setRobotPosition(lastLocation.getLatitude(), lastLocation.getLongitude());
    }
    binding.mapView.setWaypoints(mission.getWaypoints(), activeWaypointIndex);
    if (mission.hasHome()) {
      binding.mapView.setHome(mission.getHomeLatitude(), mission.getHomeLongitude());
    }

    if (mission.isEmpty()) {
      binding.missionInfo.setText(R.string.gps_nav_mission_empty);
    } else if (navMode == NavMode.MISSION && activeWaypointIndex >= 0) {
      double remaining = 0;
      if (lastLocation != null) {
        Waypoint target = mission.getWaypoint(activeWaypointIndex);
        if (target != null) {
          remaining += target.distanceFrom(lastLocation.getLatitude(), lastLocation.getLongitude());
          for (int i = activeWaypointIndex + 1; i < mission.size(); i++) {
            Waypoint from = mission.getWaypoint(i - 1);
            Waypoint to = mission.getWaypoint(i);
            remaining += to.distanceFrom(from.getLatitude(), from.getLongitude());
          }
        }
      }
      binding.missionInfo.setText(
          getString(
              R.string.gps_nav_mission_progress,
              activeWaypointIndex + 1,
              mission.size(),
              remaining));
    } else {
      binding.missionInfo.setText(
          getString(R.string.gps_nav_mission_info, mission.size(), mission.totalDistanceMeters()));
    }
  }

  private void updateHud(@Nullable Location location) {
    if (binding == null) {
      return;
    }
    updateConnectionIcon();
    Waypoint target = currentTarget();
    if (target == null || lastLocation == null) {
      binding.hudDistance.setText(R.string.gps_nav_distance_placeholder);
      binding.hudBearing.setText(R.string.gps_nav_bearing_placeholder);
      binding.hudSpeed.setText(R.string.gps_nav_speed_placeholder);
      return;
    }

    double distance = target.distanceFrom(lastLocation.getLatitude(), lastLocation.getLongitude());
    double turnError =
        GeoMath.turnErrorDegrees(
            headingDegrees,
            target.bearingFrom(lastLocation.getLatitude(), lastLocation.getLongitude()));

    binding.hudDistance.setText(String.format(Locale.US, "Dist  %.0f m", distance));
    binding.hudBearing.setText(String.format(Locale.US, "Brng  %+.0f°", turnError));
    if (location != null && location.hasSpeed()) {
      binding.hudSpeed.setText(String.format(Locale.US, "Spd  %.1f m/s", location.getSpeed()));
    } else {
      binding.hudSpeed.setText(R.string.gps_nav_speed_placeholder);
    }
    if (location != null && location.hasAccuracy()) {
      binding.hudAccuracy.setText(String.format(Locale.US, "Acc  %.0f m", location.getAccuracy()));
    }
  }

  // endregion

  // region Persistence

  private File missionFile() {
    File folder = new File(requireContext().getFilesDir(), MISSION_FOLDER);
    //noinspection ResultOfMethodCallIgnored
    folder.mkdirs();
    return new File(folder, MISSION_FILE);
  }

  private void saveMission() {
    try {
      writeFile(missionFile(), mission.toWaypointsFile());
    } catch (IOException e) {
      Timber.e(e, "Could not save GPS mission");
    }
  }

  private void loadMission() {
    File file = missionFile();
    if (!file.exists()) {
      return;
    }
    try {
      Mission loaded = Mission.parseWaypointsFile(readFile(file));
      mission.setHome(loaded.getHomeLatitude(), loaded.getHomeLongitude());
      mission.setWaypoints(loaded.getWaypoints());
      Timber.d("Loaded GPS mission with %d waypoints", mission.size());
    } catch (IOException e) {
      Timber.e(e, "Could not load GPS mission");
    }
  }

  private void exportWaypointsTo(@Nullable Uri uri) {
    if (uri == null) {
      return; // picker cancelled
    }
    try (OutputStream out = requireContext().getContentResolver().openOutputStream(uri)) {
      if (out == null) {
        throw new IOException("Cannot open " + uri);
      }
      out.write(mission.toWaypointsFile().getBytes(StandardCharsets.UTF_8));
      Toast.makeText(requireContext(), R.string.gps_nav_waypoints_exported, Toast.LENGTH_SHORT)
          .show();
    } catch (IOException | SecurityException e) {
      Timber.e(e, "Could not export .waypoints");
      Toast.makeText(
              requireContext(),
              getString(R.string.gps_nav_waypoints_failed, String.valueOf(e.getMessage())),
              Toast.LENGTH_LONG)
          .show();
    }
  }

  private void importWaypointsFrom(@Nullable Uri uri) {
    if (uri == null || binding == null) {
      return; // picker cancelled, or the screen went away while it was open
    }
    if (navMode != NavMode.IDLE) {
      // Swapping the plan under a running mission would retarget the car mid-drive.
      stopNavigation(R.string.gps_nav_status_idle);
    }
    try (InputStream in = requireContext().getContentResolver().openInputStream(uri)) {
      if (in == null) {
        throw new IOException("Cannot open " + uri);
      }
      StringBuilder contents = new StringBuilder();
      try (BufferedReader reader =
          new BufferedReader(new InputStreamReader(in, StandardCharsets.UTF_8))) {
        char[] buffer = new char[4096];
        int read;
        while ((read = reader.read(buffer)) != -1) {
          contents.append(buffer, 0, read);
        }
      }
      Mission loaded = Mission.parseWaypointsFile(contents.toString());
      mission.setHome(loaded.getHomeLatitude(), loaded.getHomeLongitude());
      mission.setWaypoints(loaded.getWaypoints());
      saveMission();
      refreshMissionUi();
      binding.mapView.setWaypoints(mission.getWaypoints(), -1);
      Toast.makeText(
              requireContext(),
              getString(R.string.gps_nav_waypoints_imported, mission.size()),
              Toast.LENGTH_SHORT)
          .show();
    } catch (IOException | SecurityException e) {
      Timber.e(e, "Could not import .waypoints");
      Toast.makeText(
              requireContext(),
              getString(R.string.gps_nav_waypoints_failed, String.valueOf(e.getMessage())),
              Toast.LENGTH_LONG)
          .show();
    }
  }

  private void exportKml() {
    if (mission.isEmpty() && track.isEmpty()) {
      Toast.makeText(requireContext(), R.string.gps_nav_need_waypoints, Toast.LENGTH_SHORT).show();
      return;
    }
    try {
      File documents =
          requireContext().getExternalFilesDir(android.os.Environment.DIRECTORY_DOCUMENTS);
      if (documents == null) {
        throw new IOException("External documents directory unavailable");
      }
      //noinspection ResultOfMethodCallIgnored
      documents.mkdirs();
      File out = new File(documents, "openbot-mission.kml");
      writeFile(out, KmlWriter.build("OpenBot Mission", mission, track));
      Toast.makeText(
              requireContext(),
              getString(R.string.gps_nav_export_done, out.getAbsolutePath()),
              Toast.LENGTH_LONG)
          .show();
    } catch (IOException | SecurityException e) {
      Timber.e(e, "Could not export mission KML");
      Toast.makeText(
              requireContext(),
              getString(R.string.gps_nav_export_failed, String.valueOf(e.getMessage())),
              Toast.LENGTH_LONG)
          .show();
    }
  }

  /** Reads a UTF-8 text file. Uses java.io so it works across the full minSdk..targetSdk range. */
  private static String readFile(File file) throws IOException {
    StringBuilder contents = new StringBuilder();
    try (BufferedReader reader =
        new BufferedReader(
            new InputStreamReader(new FileInputStream(file), StandardCharsets.UTF_8))) {
      char[] buffer = new char[4096];
      int read;
      while ((read = reader.read(buffer)) != -1) {
        contents.append(buffer, 0, read);
      }
    }
    return contents.toString();
  }

  private static void writeFile(File file, String contents) throws IOException {
    try (Writer writer =
        new OutputStreamWriter(new FileOutputStream(file), StandardCharsets.UTF_8)) {
      writer.write(contents);
    }
  }

  // endregion

  private static String formatLat(double latitude) {
    return String.format(Locale.US, "%.7f", latitude);
  }

  private static String formatLon(double longitude) {
    return String.format(Locale.US, "%.7f", longitude);
  }
}
