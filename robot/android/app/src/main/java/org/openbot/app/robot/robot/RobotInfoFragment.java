package org.openbot.app.robot.robot;

import android.annotation.SuppressLint;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import androidx.annotation.Nullable;
import androidx.navigation.Navigation;
import java.util.Locale;
import org.jetbrains.annotations.NotNull;
import org.openbot.app.robot.R;
import org.openbot.app.robot.common.ControlsFragment;
import org.openbot.app.robot.databinding.FragmentRobotInfoBinding;
import org.openbot.app.robot.lidarbot.LidarBehaviors;
import org.openbot.app.robot.vehicle.RcMode;

/**
 * Car setup for the ESP32-S3 Ackermann car. Shows only what the firmware reports (its 'f' reply):
 * LiDAR obstacle distance (s), 8 proximity sectors (p), scans over USB (x), indicators (i), front
 * and rear lights (lf, lb), status LEDs (ls); the RC receiver shows up once 'm' messages arrive.
 * There is no battery sensor, sonar, bumper or wheel encoder on this car, so none are listed.
 */
public class RobotInfoFragment extends ControlsFragment {
  private FragmentRobotInfoBinding binding;
  private RcMode rcMode;

  @Override
  public View onCreateView(
      @NotNull LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
    binding = FragmentRobotInfoBinding.inflate(inflater, container, false);
    return binding.getRoot();
  }

  @SuppressLint("RestrictedApi")
  @Override
  public void onViewCreated(@NotNull View view, @Nullable Bundle savedInstanceState) {
    super.onViewCreated(view, savedInstanceState);

    if (vehicle == null) {
      mViewModel
          .getVehicle()
          .observe(
              getViewLifecycleOwner(),
              v -> {
                if (v != null && vehicle == null) {
                  vehicle = v;
                  setupRobotInfoUi();
                }
              });
      return;
    }
    setupRobotInfoUi();
  }

  private void setupRobotInfoUi() {
    if (vehicle == null || binding == null) {
      return;
    }

    String connectionType = vehicle.getConnectionType();
    if (connectionType != null && connectionType.equals("USB")) {
      binding.usbToggle.setVisibility(View.VISIBLE);
      binding.bleToggle.setVisibility(View.GONE);
    } else if (connectionType != null && connectionType.equals("Bluetooth")) {
      binding.bleToggle.setVisibility(View.VISIBLE);
      binding.usbToggle.setVisibility(View.GONE);
    }

    mViewModel
        .getUsbStatus()
        .observe(getViewLifecycleOwner(), status -> binding.usbToggle.setChecked(status));

    binding.usbToggle.setChecked(vehicle.isUsbConnected());
    binding.bleToggle.setChecked(vehicle.bleConnected());

    binding.usbToggle.setOnClickListener(
        v -> {
          binding.usbToggle.setChecked(vehicle.isUsbConnected());
          Navigation.findNavController(requireView()).navigate(R.id.open_usb_fragment);
        });

    binding.bleToggle.setOnClickListener(
        v -> {
          binding.bleToggle.setChecked(vehicle.bleConnected());
          Navigation.findNavController(requireView()).navigate(R.id.open_bluetooth_fragment);
        });

    binding.usbToggle.setOnCheckedChangeListener((buttonView, isChecked) -> refreshGui());
    binding.bleToggle.setOnCheckedChangeListener((buttonView, isChecked) -> refreshGui());
    binding.refreshToggle.setOnClickListener(v -> refreshGui());

    // Full-size LiDAR plan view with its own readout; FreeRoam carries the compact one.
    attachLidarView(binding.lidarView, binding.lidarInfo);

    binding.lightsSlider.addOnChangeListener(
        (slider, value, fromUser) -> {
          vehicle.sendLightIntensity(value / 100, value / 100);
        });

    binding.motorsForwardButton.setOnClickListener(v -> vehicle.setControl(0.75f, 0.75f));

    binding.motorsBackwardButton.setOnClickListener(v -> vehicle.setControl(-0.75f, -0.75f));

    binding.motorsStopButton.setOnClickListener(v -> vehicle.setControl(0.0f, 0.0f));

    // Steering test with the wheels still: the servo turns for about a second (the heartbeat then
    // stops and straightens the car). Same throttle/steer mixing as everywhere else in the app.
    binding.steerLeftButton.setOnClickListener(v -> steer(1));
    binding.steerCenterButton.setOnClickListener(v -> steer(0));
    binding.steerRightButton.setOnClickListener(v -> steer(-1));
    binding.indLeftButton.setOnClickListener(v -> vehicle.setIndicator(-1));
    binding.indOffButton.setOnClickListener(v -> vehicle.setIndicator(0));
    binding.indRightButton.setOnClickListener(v -> vehicle.setIndicator(1));

    mViewModel.getRcMode().observe(getViewLifecycleOwner(), m -> {
      rcMode = m;
      showRcMode();
    });

    // AVOID_MARGIN analog. The quick-disable switch is on FreeRoam where it can be reached in a
    // hurry; this is the standoff itself.
    float margin = preferencesManager.getObjectAvoidanceMargin();
    binding.avoidMarginSlider.setValue(margin);
    binding.avoidMarginInfo.setText(getString(R.string.avoidMarginValue, margin));
    vehicle.setObjectAvoidanceMargin(margin);
    binding.avoidMarginSlider.addOnChangeListener(
        (slider, value, fromUser) -> {
          vehicle.setObjectAvoidanceMargin(value);
          binding.avoidMarginInfo.setText(getString(R.string.avoidMarginValue, value));
          if (fromUser) {
            preferencesManager.setObjectAvoidanceMargin(value);
            vehicle.sendControl();
          }
        });

    // AVOID_BACKUP_SPD analog. Clamped into the slider's range so an old or hand-edited preference
    // can never crash the Slider (it throws on out-of-range values) and snapped to its 0.05 step.
    float backup = preferencesManager.getObjectAvoidanceBackup();
    backup = Math.max(0f, Math.min(0.5f, Math.round(backup * 20f) / 20f));
    binding.avoidBackupSlider.setValue(backup);
    vehicle.setObjectAvoidanceBackupSpeed(backup);
    showBackupSpeed(backup);
    binding.avoidBackupSlider.addOnChangeListener(
        (slider, value, fromUser) -> {
          vehicle.setObjectAvoidanceBackupSpeed(value);
          showBackupSpeed(value);
          if (fromUser) {
            preferencesManager.setObjectAvoidanceBackup(value);
            vehicle.sendControl();
          }
        });

    refreshGui();
  }

  private void steer(double s) {
    float[] w = LidarBehaviors.toWheels(0, s);
    vehicle.setControl(w[0], w[1]);
  }

  private void showRcMode() {
    if (binding == null) return;
    binding.featRc.setChecked(rcMode != null);
    if (rcMode == null) {
      binding.rcModeInfo.setText(R.string.n_a);
      return;
    }
    switch (rcMode) {
      case MANUAL:
        binding.rcModeInfo.setText(R.string.setup_rc_manual);
        break;
      case AUTO:
        binding.rcModeInfo.setText(R.string.setup_rc_auto);
        break;
      case HOLD:
        binding.rcModeInfo.setText(R.string.setup_rc_hold);
        break;
      default:
        binding.rcModeInfo.setText(R.string.setup_rc_lost);
        break;
    }
  }

  private void showLink() {
    if (vehicle.isUsbConnected()) binding.linkInfo.setText(R.string.setup_link_usb);
    else if (vehicle.bleConnected()) binding.linkInfo.setText(R.string.setup_link_ble);
    else binding.linkInfo.setText(R.string.setup_link_none);
  }

  private void showBackupSpeed(float value) {
    if (value <= 0f) {
      binding.avoidBackupInfo.setText(R.string.avoidBackupOff);
    } else {
      binding.avoidBackupInfo.setText(getString(R.string.avoidBackupValue, value * 100f));
    }
  }

  private void refreshGui() {
    if (vehicle == null || binding == null) {
      return;
    }
    updateGui(false);
    binding.refreshToggle.setChecked(false);
    if (vehicle.isReady()) {
      vehicle.requestVehicleConfig();
    }
  }

  private void updateGui(boolean isConnected) {
    showLink();
    if (isConnected) {
      String type = vehicle.getVehicleType();
      binding.robotTypeInfo.setText(type);
      boolean ackermann = type != null && type.startsWith("RC_");
      binding.robotIcon.setImageResource(ackermann ? R.drawable.rc_car : R.drawable.ic_openbot);
      binding.driveInfo.setText(ackermann ? R.string.setup_drive_ackermann : R.string.setup_drive_unknown);
    } else {
      binding.robotTypeInfo.setText(getString(R.string.n_a));
      binding.robotIcon.setImageResource(R.drawable.ic_openbot);
      binding.driveInfo.setText(R.string.setup_drive_unknown);
      binding.obstacleInfo.setText(R.string.n_a);
      vehicle.setHasSonar(false);
      vehicle.setHasProximity(false);
      vehicle.setHasSerialScans(false);
      vehicle.setHasIndicators(false);
      vehicle.setHasLedsFront(false);
      vehicle.setHasLedsBack(false);
      vehicle.setHasLedsStatus(false);
    }
    binding.featLidarObstacle.setChecked(vehicle.isHasSonar());
    binding.featLidarProximity.setChecked(vehicle.isHasProximity());
    binding.featLidarScans.setChecked(vehicle.isHasSerialScans());
    binding.featIndicators.setChecked(vehicle.isHasIndicators());
    binding.featLedsFront.setChecked(vehicle.isHasLedsFront());
    binding.featLedsBack.setChecked(vehicle.isHasLedsBack());
    binding.featLedsStatus.setChecked(vehicle.isHasLedsStatus());
    showRcMode();
    binding.indicatorsRow.setVisibility(vehicle.isHasIndicators() ? View.VISIBLE : View.GONE);
    boolean lights = vehicle.isHasLedsFront() || vehicle.isHasLedsBack();
    binding.ledsLabel.setVisibility(lights ? View.VISIBLE : View.GONE);
    binding.lightsSlider.setVisibility(lights ? View.VISIBLE : View.GONE);
  }

  @Override
  protected void processControllerKeyData(String command) {
    // Do nothing
  }

  @Override
  protected void processUSBData(String data) {
    if (vehicle == null || binding == null) {
      return;
    }
    if (!vehicle.isReady()) {
      vehicle.setReady(true);
      vehicle.requestVehicleConfig();
    }
    char header = data.charAt(0);
    // String body = data.substring(1);
    // int type = -1;
    switch (header) {
      case 'r':
        vehicle.requestVehicleConfig();
      case 'f':
        binding.refreshToggle.setChecked(vehicle.isReady());
        updateGui(vehicle.isReady());
        break;
      case 's':
        // The LiDAR's nearest return in the forward cone (the firmware sends it as 's').
        float cm = vehicle.getSonarReading();
        // 0 = no fresh scan (the car holds still), 300 = nothing within 3 m (MAX_SONAR_DISTANCE).
        if (cm <= 0) binding.obstacleInfo.setText(R.string.setup_obstacle_stale);
        else if (cm >= 300) binding.obstacleInfo.setText(R.string.setup_obstacle_clear);
        else binding.obstacleInfo.setText(String.format(Locale.US, "%.0f cm", cm));
        break;
    }
  }

  @Override
  public void onResume() {
    super.onResume();
    binding.bleToggle.setChecked(vehicle.bleConnected());
  }
}
