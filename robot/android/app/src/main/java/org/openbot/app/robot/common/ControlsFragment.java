package org.openbot.app.robot.common;

import android.os.Bundle;
import android.view.KeyEvent;
import android.view.MotionEvent;
import android.view.View;
import android.view.animation.Animation;
import android.view.animation.AnimationUtils;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.CheckBox;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;
import androidx.activity.result.ActivityResultLauncher;
import androidx.activity.result.contract.ActivityResultContracts;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;
import androidx.lifecycle.ViewModelProvider;
import java.io.File;
import java.util.List;
import java.util.Set;
import java.util.function.Predicate;
import java.util.stream.Collectors;
import org.jetbrains.annotations.NotNull;
import org.json.JSONObject;
import org.openbot.app.robot.R;
import org.openbot.app.robot.env.AudioPlayer;
import org.openbot.app.robot.env.BotToControllerEventBus;
import org.openbot.app.robot.env.ControllerToBotEventBus;
import org.openbot.app.robot.env.PhoneController;
import org.openbot.app.robot.env.SharedPreferencesManager;
import org.openbot.app.robot.main.MainViewModel;
import org.openbot.app.robot.sensorlink.RemoteSettings;
import org.openbot.app.robot.sensorlink.SensorLink;
import org.openbot.app.robot.sensorlink.SensorLinkDialog;
import org.openbot.app.robot.server.ServerCommunication;
import org.openbot.app.robot.server.ServerListener;
import org.openbot.app.robot.tflite.Model;
import org.openbot.app.robot.utils.ConnectionUtils;
import org.openbot.app.robot.utils.Constants;
import org.openbot.app.robot.utils.Enums;
import org.openbot.app.robot.utils.FileUtils;
import org.openbot.app.robot.utils.FormatUtils;
import org.openbot.app.robot.utils.PermissionUtils;
import org.openbot.app.robot.vehicle.Control;
import org.openbot.app.robot.vehicle.LidarScanner;
import org.openbot.app.robot.vehicle.ObjectAvoidance;
import org.openbot.app.robot.vehicle.Proximity;
import org.openbot.app.robot.vehicle.Vehicle;
import timber.log.Timber;

public abstract class ControlsFragment extends Fragment implements ServerListener {
  private static final String NO_SERVER = "No server";

  protected MainViewModel mViewModel;
  protected Vehicle vehicle;
  protected Animation startAnimation;
  protected SharedPreferencesManager preferencesManager;
  protected PhoneController phoneController;

  protected AudioPlayer audioPlayer;

  /**
   * One UDP listener for the firmware's "LDS1" scan stream, shared by every screen. The car is
   * driven over BLE; this is only the side channel that lets the operator see what the LiDAR sees,
   * so it never sits on the control path and a phone off the robot's WiFi just shows "no scans".
   */
  protected final LidarScanner lidarScanner = new LidarScanner();

  /**
   * The LiDAR's scans arrive on their own UDP socket, not on the vehicle telemetry stream, so
   * {@code processUSBData()} never fires for them. This tick keeps any attached scan view moving.
   */
  private final android.os.Handler lidarUiHandler =
      new android.os.Handler(android.os.Looper.getMainLooper());

  private final Runnable lidarUiTick =
      new Runnable() {
        @Override
        public void run() {
          if (lidarViewRef != null) {
            refreshLidar(lidarViewRef, lidarStatusRef);
          }
          if (isResumed()) {
            lidarUiHandler.postDelayed(this, 100);
          }
        }
      };
  private org.openbot.app.robot.customview.LidarView lidarViewRef;
  private TextView lidarStatusRef;

  protected final String voice = "matthew";
  protected List<Model> masterList;

  protected ServerCommunication serverCommunication;

  private ArrayAdapter<String> modelAdapter;
  private ArrayAdapter<String> serverAdapter;
  private Spinner modelSpinner;
  private Spinner serverSpinner;

  @Override
  public void onCreate(@Nullable Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    // create before inflateFragment() to prevent npe when calling addCamera()
    preferencesManager = new SharedPreferencesManager(requireContext());
  }

  @Override
  public void onViewCreated(@NonNull View view, @Nullable Bundle savedInstanceState) {
    super.onViewCreated(view, savedInstanceState);
    requireActivity()
        .getWindow()
        .addFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

    phoneController = PhoneController.getInstance(requireContext());

    audioPlayer = new AudioPlayer(requireContext());
    masterList = FileUtils.loadConfigJSONFromAsset(requireActivity());
    serverCommunication = new ServerCommunication(requireContext(), this);

    requireActivity()
        .getSupportFragmentManager()
        .setFragmentResultListener(
            Constants.GENERIC_MOTION_EVENT,
            this,
            (requestKey, result) -> {
              if (vehicle == null) {
                return;
              }
              MotionEvent motionEvent = result.getParcelable(Constants.DATA);
              vehicle.setControl(vehicle.getGameController().processJoystickInput(motionEvent, -1));
              processControllerKeyData(Constants.CMD_DRIVE);
            });
    requireActivity()
        .getSupportFragmentManager()
        .setFragmentResultListener(
            Constants.KEY_EVENT,
            this,
            (requestKey, result) -> {
              if (vehicle == null) {
                return;
              }
              KeyEvent event = result.getParcelable(Constants.DATA);
              if (KeyEvent.ACTION_UP == event.getAction()) {
                processKeyEvent(result.getParcelable(Constants.DATA));
              }
              Control newControl =
                  vehicle
                      .getGameController()
                      .processButtonInput(result.getParcelable(Constants.DATA));
              if (vehicle.getControl().getLeft() != newControl.getLeft()
                  && vehicle.getControl().getRight() != newControl.getRight()) {
                vehicle.setControl(newControl);
              }
            });

    mViewModel = new ViewModelProvider(requireActivity()).get(MainViewModel.class);

    vehicle = mViewModel.getVehicle().getValue();
    if (vehicle == null) {
      mViewModel
          .getVehicle()
          .observe(
              getViewLifecycleOwner(),
              v -> {
                if (v != null) {
                  vehicle = v;
                }
              });
    }
    startAnimation = AnimationUtils.loadAnimation(requireContext(), R.anim.blink);

    mViewModel
        .getDeviceData()
        .observe(
            getViewLifecycleOwner(),
            data -> {
              if (vehicle == null || data == null || data.isEmpty()) {
                return;
              }
              char header = data.charAt(0);
              String body = data.substring(1);

              switch (header) {
                case 'r':
                  vehicle.setReady(true);
                  break;
                case 'f':
                  vehicle.processVehicleConfig(body);
                  break;
                case 'v':
                  if (FormatUtils.isNumeric(body)) {
                    vehicle.setBatteryVoltage(Float.parseFloat(body));
                  } else {
                    String[] msgParts = body.split(":");
                    switch (msgParts[0]) {
                      case "min":
                        vehicle.setMinMotorVoltage(Float.parseFloat(msgParts[1]));
                        break;
                      case "low":
                        vehicle.setLowBatteryVoltage(Float.parseFloat(msgParts[1]));
                        break;
                      case "max":
                        vehicle.setMaxBatteryVoltage(Float.parseFloat(msgParts[1]));
                        break;
                      default:
                        // A garbled battery reading is not something the driver can act on.
                        Timber.w("Ignored a garbled battery message from the car: %s", body);
                        break;
                    }
                  }
                  break;
                case 's':
                  if (FormatUtils.isNumeric(body)) {
                    vehicle.setSonarReading(Float.parseFloat(body));
                  }
                  break;
                case 'x':
                  // One LiDAR revolution down the USB cable. Feeds the same scanner the UDP
                  // stream does, so every screen's plan view picks it up with no WiFi at all.
                  org.openbot.app.robot.vehicle.LidarScan serialScan =
                      org.openbot.app.robot.vehicle.LidarScan.parseSerialLine(body);
                  if (serialScan != null) {
                    lidarScanner.offer(serialScan);
                  }
                  break;
                case 'p':
                  // Eight proximity octants for object avoidance. Malformed bodies are dropped by
                  // the parser rather than clearing the sectors - a truncated BLE notification must
                  // not read as eight sectors of open road.
                  float[] sectors = Proximity.parseSectorsCm(body);
                  if (sectors != null) {
                    vehicle.setProximitySectorsCm(sectors);
                  }
                  break;
                case 'w':
                  String[] itemList = body.split(",");
                  if (itemList.length == 2
                      && FormatUtils.isNumeric(itemList[0])
                      && FormatUtils.isNumeric(itemList[1])) {
                    vehicle.setLeftWheelRpm(Float.parseFloat(itemList[0]));
                    vehicle.setRightWheelRpm(Float.parseFloat(itemList[1]));
                  }
                  break;
                case 'b':
                  // do nothing
                  break;
              }

              processUSBData(data);
            });

    handlePhoneControllerEvents();
  }

  protected void processKeyEvent(KeyEvent keyCode) {
    if (vehicle == null) {
      return;
    }
    if (Enums.ControlMode.getByID(preferencesManager.getControlMode())
        == Enums.ControlMode.GAMEPAD) {
      switch (keyCode.getKeyCode()) {
        case KeyEvent.KEYCODE_BUTTON_X: // square
          toggleIndicatorEvent(Enums.VehicleIndicator.LEFT.getValue());
          processControllerKeyData(Constants.CMD_INDICATOR_LEFT);
          break;
        case KeyEvent.KEYCODE_BUTTON_Y: // triangle
          toggleIndicatorEvent(Enums.VehicleIndicator.STOP.getValue());
          processControllerKeyData(Constants.CMD_INDICATOR_STOP);
          break;
        case KeyEvent.KEYCODE_BUTTON_B: // circle
          toggleIndicatorEvent(Enums.VehicleIndicator.RIGHT.getValue());
          processControllerKeyData(Constants.CMD_INDICATOR_RIGHT);
          break;
        case KeyEvent.KEYCODE_BUTTON_A: // x
          processControllerKeyData(Constants.CMD_LOGS);
          break;
        case KeyEvent.KEYCODE_BUTTON_START: // options
          toggleNoise();
          processControllerKeyData(Constants.CMD_NOISE);
          break;
        case KeyEvent.KEYCODE_BUTTON_L1:
          processControllerKeyData(Constants.CMD_DRIVE_MODE);
          audioPlayer.playDriveMode(voice, vehicle.getDriveMode());
          break;
        case KeyEvent.KEYCODE_BUTTON_R1:
          handleNetworkCommand();
          break;
        case KeyEvent.KEYCODE_BUTTON_THUMBL:
          processControllerKeyData(Constants.CMD_SPEED_DOWN);
          audioPlayer.playSpeedMode(
              voice, Enums.SpeedMode.getByID(preferencesManager.getSpeedMode()));
          break;
        case KeyEvent.KEYCODE_BUTTON_THUMBR:
          processControllerKeyData(Constants.CMD_SPEED_UP);

          audioPlayer.playSpeedMode(
              voice, Enums.SpeedMode.getByID(preferencesManager.getSpeedMode()));
          break;

        default:
          break;
      }
    }
  }

  private void handlePhoneControllerEvents() {
    String subscriber = this.getClass().getSimpleName();
    ControllerToBotEventBus.unsubscribe(subscriber);
    ControllerToBotEventBus.subscribe(
        subscriber,
        event -> {
          if (getActivity() == null) {
            return;
          }
          getActivity().runOnUiThread(() -> handleControllerEvent(event));
        },
        error -> Timber.e(error, "ControllerToBotEventBus error in %s", getClass().getSimpleName()),
        event -> event.has("command") || event.has("driveCmd") || event.has("server"));
  }

  private void handleControllerEvent(JSONObject event) {
    try {
      if (event.has("server")) {
        updateServerSpinnerFromEvent(event);
        return;
      }

      String commandType = resolveCommandType(event);
      if (commandType == null) {
        return;
      }
      if (!ensureVehicleForCommand(commandType)) {
        return;
      }

      switch (commandType) {
        case Constants.CMD_DRIVE:
          applyDriveCommand(event.getJSONObject("driveCmd"));
          processControllerKeyData(Constants.CMD_DRIVE);
          break;

        case Constants.CMD_NOISE:
          toggleNoise();
          break;

        case Constants.CMD_NETWORK:
          handleNetworkCommand();
          break;

        case Constants.CMD_INDICATOR_LEFT:
          toggleIndicatorEvent(Enums.VehicleIndicator.LEFT.getValue());
          processControllerKeyData(Constants.CMD_INDICATOR_LEFT);
          break;

        case Constants.CMD_INDICATOR_RIGHT:
          toggleIndicatorEvent(Enums.VehicleIndicator.RIGHT.getValue());
          processControllerKeyData(Constants.CMD_INDICATOR_RIGHT);
          break;

        case Constants.CMD_INDICATOR_STOP:
          toggleIndicatorEvent(Enums.VehicleIndicator.STOP.getValue());
          processControllerKeyData(Constants.CMD_INDICATOR_STOP);
          break;

        case Constants.CMD_CONNECTED:
          sendConnectedStatus();
          break;

        case Constants.CMD_DISCONNECTED:
          vehicle.setControl(0, 0);
          processControllerKeyData(Constants.CMD_DISCONNECTED);
          break;

        default:
          processControllerKeyData(commandType);
          break;
      }
    } catch (Exception e) {
      Timber.e(e, "Error handling controller event in %s", getClass().getSimpleName());
    }
  }

  private String resolveCommandType(JSONObject event) throws org.json.JSONException {
    if (event.has("command")) {
      return event.getString("command");
    }
    if (event.has("driveCmd")) {
      return Constants.CMD_DRIVE;
    }
    return null;
  }

  private boolean ensureVehicleForCommand(String commandType) {
    if (vehicle == null && mViewModel != null) {
      vehicle = mViewModel.getVehicle().getValue();
    }
    if (vehicle == null) {
      Timber.w("Ignoring web command %s: vehicle not ready", commandType);
      return false;
    }
    return true;
  }

  private void applyDriveCommand(JSONObject driveValue) {
    vehicle.setControl(
        new Control((float) driveValue.optDouble("l", 0), (float) driveValue.optDouble("r", 0)));
  }

  private void sendConnectedStatus() {
    BotToControllerEventBus.emitEvent(
        ConnectionUtils.getStatus(
            isLoggingEnabledForStatus(),
            vehicle.isNoiseEnabled(),
            isNetworkModeEnabled(),
            vehicle.getDriveMode().toString(),
            vehicle.getIndicator()));
  }

  private void updateServerSpinnerFromEvent(JSONObject event) throws org.json.JSONException {
    if (serverSpinner == null || serverSpinner.getAdapter() == null) {
      return;
    }
    String server = event.getString("server");
    for (int i = 0; i < serverSpinner.getAdapter().getCount(); i++) {
      if (server.equals("noServerFound")) {
        serverSpinner.setSelection(0);
        return;
      }
      if (server.equals(serverSpinner.getAdapter().getItem(i))) {
        serverSpinner.setSelection(i);
        return;
      }
    }
  }

  // Override in Autopilot / Object Nav — they use network mode for ML, not just a toggle.
  protected void handleNetworkCommand() {}

  protected boolean isNetworkModeEnabled() {
    return false;
  }

  protected boolean isLoggingEnabledForStatus() {
    return false;
  }

  protected void emitNetworkStatus(boolean enabled) {
    BotToControllerEventBus.emitEvent(ConnectionUtils.createStatus("NETWORK", enabled));
  }

  protected void toggleNoise() {
    if (vehicle == null) {
      return;
    }
    vehicle.toggleNoise();
    BotToControllerEventBus.emitEvent(
        ConnectionUtils.createStatus("NOISE", vehicle.isNoiseEnabled()));
    audioPlayer.playNoise(voice, vehicle.isNoiseEnabled());
  }

  private void toggleIndicatorEvent(int value) {
    vehicle.setIndicator(value);
    BotToControllerEventBus.emitEvent(ConnectionUtils.createStatus("INDICATOR_LEFT", value == -1));
    BotToControllerEventBus.emitEvent(ConnectionUtils.createStatus("INDICATOR_RIGHT", value == 1));
    BotToControllerEventBus.emitEvent(ConnectionUtils.createStatus("INDICATOR_STOP", value == 0));
  }

  private boolean allGranted = true;
  protected final ActivityResultLauncher<String[]> requestPermissionLauncher =
      registerForActivityResult(
          new ActivityResultContracts.RequestMultiplePermissions(),
          result -> {
            result.forEach((permission, granted) -> allGranted = allGranted && granted);

            if (allGranted) phoneController.connect(requireContext());
            else {
              PermissionUtils.showControllerPermissionsToast(requireActivity());
            }
          });

  @NotNull
  protected List<String> getModelNames(Predicate<Model> filter) {
    return masterList.stream()
        .filter(filter)
        .map(f -> FileUtils.nameWithoutExtension(f.name))
        .collect(Collectors.toList());
  }

  @Override
  public void onResume() {
    serverCommunication.start();
    // Binding the scan port is best-effort: a ROS receiver may already hold it, or the phone may
    // not have the WiFi permission granted yet. The car is drivable either way.
    if (!lidarScanner.isRunning()) {
      lidarScanner.start(LidarScanner.DEFAULT_PORT);
    }
    lidarUiHandler.removeCallbacks(lidarUiTick);
    lidarUiHandler.post(lidarUiTick);
    if (sensorLinkStatusView != null && !sensorLinkAttached) {
      sensorLinkAttached = true;
      SensorLink.get().addStatusListener(sensorLinkStatusListener);
      SensorLink.get().setScreen(remoteScreenName(), browserSettings);
      SensorLink.get().attach(requireContext());
      refreshSensorLinkStatus();
    }
    super.onResume();
  }

  @Override
  public void onDestroy() {
    Timber.d("onDestroy");
    ControllerToBotEventBus.unsubscribe(this.getClass().getSimpleName());
    lidarScanner.stop();
    if (vehicle != null) {
      vehicle.setControl(0, 0);
    }
    super.onDestroy();
  }

  @Override
  public synchronized void onPause() {
    Timber.d("onPause");
    lidarUiHandler.removeCallbacks(lidarUiTick);
    serverCommunication.stop();
    if (sensorLinkAttached) {
      sensorLinkAttached = false;
      SensorLink.get().removeStatusListener(sensorLinkStatusListener);
      SensorLink.get().clearScreen(browserSettings);
      SensorLink.get().detach();
    }
    if (vehicle != null) {
      vehicle.setControl(0, 0);
    }
    super.onPause();
  }

  @Override
  public void onStop() {
    Timber.d("onStop");
    super.onStop();
  }

  /**
   * Wire up the object-avoidance quick-disable and status readout.
   *
   * <p>Every screen that drives the car gets the same pair, because the law itself is applied
   * centrally in {@code Vehicle.sendControl()} and therefore already covers every drive source.
   * What each screen owes the driver is a way to turn it off in a hurry and something that says
   * what it is doing - ArduPilot's "SAFETY FIRST!" point about having a quick-disable within reach
   * (their {@code RCx_OPTION = 40} switch) applies per screen, not once in the settings.
   *
   * <p>Pass the views from {@code object_avoidance_control.xml}, or any layout using the same IDs.
   */
  protected void setupObjectAvoidanceControls(CheckBox toggle, TextView status) {
    if (vehicle == null || toggle == null || status == null) {
      return;
    }
    vehicle.setObjectAvoidanceEnabled(preferencesManager.getObjectAvoidanceEnabled());
    vehicle.setObjectAvoidanceMargin(preferencesManager.getObjectAvoidanceMargin());
    vehicle.setObjectAvoidanceBackupSpeed(preferencesManager.getObjectAvoidanceBackup());
    toggle.setChecked(vehicle.isObjectAvoidanceEnabled());
    avoidToggleRef = toggle;
    avoidStatusRef = status;
    toggle.setOnClickListener(
        v -> {
          boolean enabled = toggle.isChecked();
          vehicle.setObjectAvoidanceEnabled(enabled);
          preferencesManager.setObjectAvoidanceEnabled(enabled);
          // Re-send immediately so a car currently held back either resumes (switched off) or
          // stops right now (switched on against a barrier).
          vehicle.sendControl();
          refreshObjectAvoidanceStatus(status);
        });
    refreshObjectAvoidanceStatus(status);
  }

  private TextView sensorLinkStatusView;
  private boolean sensorLinkAttached;
  private CheckBox avoidToggleRef;
  private TextView avoidStatusRef;

  // region Browser settings (Sensor link page, server mode)

  /** The name shown at the top of the browser page. */
  protected String remoteScreenName() {
    return getClass().getSimpleName();
  }

  /** Screens add their own settings here (e.g. tracking confidence). Main thread. */
  protected void addScreenSettings(java.util.List<RemoteSettings.Setting> out) {}

  /** Speed mode chosen in the browser; screens with a speed icon override to update it. */
  protected void onRemoteSpeedMode(Enums.SpeedMode mode) {
    preferencesManager.setSpeedMode(mode.getValue());
    if (vehicle != null) vehicle.setSpeedMultiplier(mode.getValue());
  }

  private static final java.util.List<String> SPEEDS = java.util.Arrays.asList("slow", "normal", "fast");

  /** Driving settings every driving screen offers, then the screen's own. */
  private final RemoteSettings.Provider browserSettings =
      out -> {
        if (vehicle != null) {
          out.add(
              RemoteSettings.Setting.action(
                  "drive.stop", "Driving", "Emergency stop", "STOP", "RESUME",
                  () -> vehicle.getEmergencyStop().isLatched(),
                  v -> {
                    if ((Boolean) v) vehicle.getEmergencyStop().latch();
                    else vehicle.getEmergencyStop().release();
                    vehicle.sendControl();
                  }));
          out.add(
              RemoteSettings.Setting.choice(
                  "drive.speed", "Driving", "Speed mode", SPEEDS,
                  () -> {
                    Enums.SpeedMode m = Enums.SpeedMode.getByID(preferencesManager.getSpeedMode());
                    return m == null ? "normal" : m.name().toLowerCase(java.util.Locale.ROOT);
                  },
                  v -> onRemoteSpeedMode(Enums.SpeedMode.valueOf(((String) v).toUpperCase(java.util.Locale.ROOT)))));
          out.add(
              RemoteSettings.Setting.bool(
                  "drive.avoidance", "Driving", "Object avoidance",
                  () -> vehicle.isObjectAvoidanceEnabled(),
                  v -> {
                    boolean on = (Boolean) v;
                    vehicle.setObjectAvoidanceEnabled(on);
                    preferencesManager.setObjectAvoidanceEnabled(on);
                    vehicle.sendControl();
                    if (avoidToggleRef != null) avoidToggleRef.setChecked(on);
                    refreshObjectAvoidanceStatus(avoidStatusRef);
                  }));
          out.add(
              RemoteSettings.Setting.range(
                  "drive.avoidanceMargin", "Driving", "Avoidance distance (m)", 0.3, 3.0, 0.1,
                  () -> preferencesManager.getObjectAvoidanceMargin(),
                  v -> {
                    float m = ((Double) v).floatValue();
                    preferencesManager.setObjectAvoidanceMargin(m);
                    vehicle.setObjectAvoidanceMargin(m);
                  }));
        }
        addScreenSettings(out);
      };

  // endregion
  private final Runnable sensorLinkStatusListener = this::refreshSensorLinkStatus;

  /**
   * Wire up the Sensor link row ({@code sensor_link_control.xml}): the SensorServer WebSocket link
   * runs while this screen is resumed, and the button opens its settings. Call from
   * onViewCreated.
   */
  protected void setupSensorLinkControls(View root) {
    if (root == null) {
      return;
    }
    sensorLinkStatusView = root.findViewById(R.id.sensor_link_status);
    root.findViewById(R.id.sensor_link_button)
        .setOnClickListener(v -> SensorLinkDialog.show(requireContext()));
    refreshSensorLinkStatus();
  }

  private void refreshSensorLinkStatus() {
    if (sensorLinkStatusView != null && isAdded()) {
      sensorLinkStatusView.setText(SensorLink.get().statusText(requireContext()));
    }
  }

  /** Refresh the avoidance status line. Call from the screen's regular UI update. */
  protected void refreshObjectAvoidanceStatus(TextView status) {
    if (vehicle == null || status == null) {
      return;
    }
    if (!vehicle.isObjectAvoidanceEnabled()) {
      status.setText(R.string.objectAvoidanceIdle);
      return;
    }
    ObjectAvoidance.Decision d = vehicle.getLastAvoidanceDecision();
    if (d == null) {
      status.setText(R.string.objectAvoidanceClear);
    } else if (d.backing) {
      status.setText(getString(R.string.objectAvoidanceBacking, d.nearestMeters));
    } else if (d.blocked) {
      // A stale LiDAR is its own message: the car is being held because it can no longer see,
      // not because something is in front of it. Worth saying which.
      if (vehicle.getProximity().getDataState() == ObjectAvoidance.DataState.STALE) {
        status.setText(R.string.objectAvoidanceBlind);
      } else {
        status.setText(getString(R.string.objectAvoidanceHolding, d.nearestMeters));
      }
    } else {
      status.setText(R.string.objectAvoidanceClear);
    }
  }

  /**
   * Register the scan view this screen wants kept up to date. The shared tick refreshes it while
   * the fragment is resumed, because scans arrive on their own UDP socket rather than on the
   * vehicle telemetry stream that drives {@code processUSBData()}.
   */
  protected void attachLidarView(org.openbot.app.robot.customview.LidarView view, TextView status) {
    this.lidarViewRef = view;
    this.lidarStatusRef = status;
    refreshLidar(view, status);
  }

  /**
   * Push the newest scan into a {@code LidarView} and, optionally, a status line. Call from the
   * screen's regular UI update so the picture tracks the sensor rather than freezing on the last
   * thing drawn before the user looked away.
   *
   * <p>A stale scan is drawn grey and labelled stale instead of being cleared: the last real
   * picture is still worth showing, but it must read as frozen rather than as the room right now.
   */
  protected void refreshLidar(org.openbot.app.robot.customview.LidarView view, TextView status) {
    if (view == null) {
      return;
    }
    boolean hasScan = lidarScanner.hasEverReported();
    boolean fresh = lidarScanner.isFresh();
    view.setStale(hasScan && !fresh);
    view.setScan(hasScan ? lidarScanner.getLatestScan() : null);
    if (vehicle != null) {
      view.setMarginMeters(vehicle.getObjectAvoidanceMargin());
    }
    if (status != null) {
      if (!hasScan) {
        status.setText(R.string.lidarIdle);
      } else {
        org.openbot.app.robot.vehicle.LidarScan scan = lidarScanner.getLatestScan();
        int returns = scan == null ? 0 : scan.getValidPoints();
        int bins = scan == null ? 0 : scan.getBinCount();
        status.setText(getString(fresh ? R.string.lidarLive : R.string.lidarStale, bins, returns));
      }
    }
  }

  protected void initModelSpinner(Spinner spinner, List<String> models, String selected) {
    modelAdapter = new ArrayAdapter<>(requireContext(), R.layout.spinner_item, models);
    modelAdapter.setDropDownViewResource(android.R.layout.simple_dropdown_item_1line);
    modelSpinner = spinner;
    modelSpinner.setAdapter(modelAdapter);
    if (!selected.isEmpty())
      modelSpinner.setSelection(
          Math.max(0, modelAdapter.getPosition(FileUtils.nameWithoutExtension(selected))));
    modelSpinner.setOnItemSelectedListener(
        new AdapterView.OnItemSelectedListener() {
          @Override
          public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
            String selected = parent.getItemAtPosition(position).toString();
            try {
              masterList.stream()
                  .filter(f -> f.name.contains(selected))
                  .findFirst()
                  .ifPresent(value -> setModel(value));

            } catch (IllegalArgumentException e) {
              e.printStackTrace();
            }
          }

          @Override
          public void onNothingSelected(AdapterView<?> parent) {}
        });
  }

  protected void initDeviceSpinner(Spinner spinner, int selected) {
    initArraySpinner(spinner, R.array.devices);
    spinner.setSelection(selected);
  }

  protected void initArraySpinner(Spinner spinner, int arrayResId) {
    ArrayAdapter<CharSequence> adapter =
        ArrayAdapter.createFromResource(requireContext(), arrayResId, R.layout.spinner_item);
    adapter.setDropDownViewResource(android.R.layout.simple_dropdown_item_1line);
    spinner.setAdapter(adapter);
  }

  protected void initServerSpinner(Spinner spinner) {
    serverAdapter = new ArrayAdapter<>(requireContext(), R.layout.spinner_item);
    serverAdapter.setDropDownViewResource(android.R.layout.simple_dropdown_item_1line);
    serverSpinner = spinner;
    serverSpinner.setAdapter(serverAdapter);
    serverSpinner.setOnItemSelectedListener(
        new AdapterView.OnItemSelectedListener() {
          @Override
          public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
            String selected = parent.getItemAtPosition(position).toString();
            if (selected.equals(NO_SERVER)) {
              serverCommunication.disconnect();
              if (serverAdapter.getPosition(preferencesManager.getServer()) > -1) {
                preferencesManager.setServer(selected);
              }
            } else {
              serverCommunication.connect(selected);
              preferencesManager.setServer(selected);
            }
          }

          @Override
          public void onNothingSelected(AdapterView<?> parent) {
            serverCommunication.disconnect();
          }
        });
    onServerListChange(serverCommunication.getServers());
  }

  @Override
  public void onServerListChange(Set<String> servers) {
    if (serverAdapter == null) {
      return;
    }
    requireActivity()
        .runOnUiThread(
            () -> {
              serverAdapter.clear();
              serverAdapter.add(NO_SERVER);
              serverAdapter.addAll(servers);
              if (!preferencesManager.getServer().isEmpty()) {
                serverSpinner.setSelection(
                    Math.max(0, serverAdapter.getPosition(preferencesManager.getServer())));
              }
            });
  }

  @Override
  public void onAddModel(String model) {
    Model item =
        new Model(
            masterList.size() + 1,
            Model.CLASS.AUTOPILOT,
            Model.TYPE.CMDNAV,
            model,
            Model.PATH_TYPE.FILE,
            requireActivity().getFilesDir() + File.separator + model,
            "256x96");

    if (modelAdapter != null && modelAdapter.getPosition(model) == -1) {
      modelAdapter.add(model);
      masterList.add(item);
      FileUtils.updateModelConfig(requireActivity(), requireContext(), masterList, false);
    } else {
      if (model.equals(modelSpinner.getSelectedItem())) {
        setModel(item);
      }
    }
    Toast.makeText(
            requireContext().getApplicationContext(),
            "AutopilotModel added: " + model,
            Toast.LENGTH_SHORT)
        .show();
  }

  @Override
  public void onRemoveModel(String model) {
    if (modelAdapter != null && modelAdapter.getPosition(model) != -1) {
      modelAdapter.remove(model);
    }
    Toast.makeText(
            requireContext().getApplicationContext(),
            "AutopilotModel removed: " + model,
            Toast.LENGTH_SHORT)
        .show();
  }

  @Override
  public void onConnectionEstablished(String ipAddress) {}

  protected void setModel(Model model) {}

  protected abstract void processControllerKeyData(String command);

  protected abstract void processUSBData(String data);
}
