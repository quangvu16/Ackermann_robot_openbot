package org.openbot.app.robot.main;

import org.openbot.app.robot.common.IconButtons;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.res.ColorStateList;
import android.graphics.Color;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;
import androidx.lifecycle.ViewModelProvider;
import androidx.navigation.Navigation;
import androidx.preference.PreferenceManager;
import androidx.recyclerview.widget.LinearLayoutManager;
import org.openbot.app.robot.R;
import org.openbot.app.robot.common.FeatureList;
import org.openbot.app.robot.databinding.FragmentMainBinding;
import org.openbot.app.robot.databinding.ItemDashboardTileBinding;
import org.openbot.app.robot.env.SharedPreferencesManager;
import org.openbot.app.robot.model.SubCategory;
import org.openbot.app.robot.original.DefaultActivity;
import org.openbot.app.robot.vehicle.ObjectAvoidance;
import org.openbot.app.robot.vehicle.RcMode;
import org.openbot.app.robot.vehicle.Vehicle;
import timber.log.Timber;

public class MainFragment extends Fragment implements OnItemClickListener<SubCategory> {

  private static final String PREF_MORE_OPEN = "dashboard_more_open";
  private static final int GREEN = Color.rgb(0x2E, 0x7D, 0x32);
  private static final int AMBER = Color.rgb(0xF9, 0xA8, 0x25);
  private static final int RED = Color.rgb(0xC6, 0x28, 0x28);
  private static final int GREY = Color.rgb(0x9E, 0x9E, 0x9E);

  private MainViewModel mViewModel;
  private FragmentMainBinding binding;
  private SharedPreferencesManager preferencesManager;
  private RcMode rcMode;

  // The status card and chips refresh twice a second while the dashboard is on screen.
  private final Handler refreshHandler = new Handler(Looper.getMainLooper());
  private final Runnable refresh =
      new Runnable() {
        @Override
        public void run() {
          refreshStatus();
          refreshHandler.postDelayed(this, 500);
        }
      };

  @Nullable
  @Override
  public View onCreateView(
      @NonNull LayoutInflater inflater,
      @Nullable ViewGroup container,
      @Nullable Bundle savedInstanceState) {
    binding = FragmentMainBinding.inflate(inflater, container, false);
    return binding.getRoot();
  }

  @Override
  public void onViewCreated(@NonNull View view, @Nullable Bundle savedInstanceState) {
    super.onViewCreated(view, savedInstanceState);

    mViewModel = new ViewModelProvider(requireActivity()).get(MainViewModel.class);
    preferencesManager = new SharedPreferencesManager(requireContext());
    binding.list.setLayoutManager(new LinearLayoutManager(requireContext()));
    binding.list.setAdapter(new CategoryAdapter(FeatureList.getCategories(), this));

    setupTile(
        binding.tileDrive,
        R.drawable.ic_game,
        "#FF6D00",
        R.string.dash_drive_title,
        R.string.dash_drive_desc,
        R.id.action_mainFragment_to_freeRoamFragment);
    setupTile(
        binding.tileMission,
        R.drawable.ic_location,
        "#00ACC1",
        R.string.dash_mission_title,
        R.string.dash_mission_desc,
        R.id.action_mainFragment_to_gpsNavigationFragment);
    setupTile(
        binding.tileLidar,
        R.drawable.ic_lidar_robot,
        "#7E57C2",
        R.string.lidarbot_title,
        R.string.lidarbot_desc,
        R.id.action_mainFragment_to_lidarBotFragment);
    setupTile(
        binding.tileSetup,
        R.drawable.ic_settings,
        "#4B7BFF",
        R.string.dash_setup_title,
        R.string.dash_setup_desc,
        R.id.action_mainFragment_to_robotInfoFragment);

    // "More features": the full list, hidden until asked for; the choice is remembered.
    SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(requireContext());
    showMore(prefs.getBoolean(PREF_MORE_OPEN, false));
    binding.moreButton.setOnClickListener(
        v -> {
          boolean open = binding.list.getVisibility() != View.VISIBLE;
          prefs.edit().putBoolean(PREF_MORE_OPEN, open).apply();
          showMore(open);
        });

    binding.carConnectButton.setOnClickListener(v -> connect());
    binding.carConnectionSettings.setOnClickListener(v -> openConnectionScreen());
    binding.chipAvoidance.setOnClickListener(v -> toggleAvoidance());
    mViewModel
        .getRcMode()
        .observe(
            getViewLifecycleOwner(),
            mode -> {
              rcMode = mode;
              refreshStatus();
            });

    Vehicle vehicle = mViewModel.getVehicle().getValue();
    if (vehicle != null) {
      vehicle.setObjectAvoidanceEnabled(preferencesManager.getObjectAvoidanceEnabled());
    }
  }

  @Override
  public void onResume() {
    super.onResume();
    refreshHandler.post(refresh);
  }

  @Override
  public void onPause() {
    super.onPause();
    refreshHandler.removeCallbacks(refresh);
  }

  private void setupTile(
      ItemDashboardTileBinding tile,
      int icon,
      String color,
      int title,
      int description,
      int navigationAction) {
    tile.tileIcon.setImageResource(icon);
    tile.tileIcon.setBackgroundTintList(ColorStateList.valueOf(Color.parseColor(color)));
    tile.tileTitle.setText(title);
    tile.tileDescription.setText(description);
    tile.getRoot()
        .setOnClickListener(
            v -> Navigation.findNavController(requireView()).navigate(navigationAction));
  }

  private void showMore(boolean open) {
    binding.list.setVisibility(open ? View.VISIBLE : View.GONE);
    IconButtons.set(binding.moreButton,
        open ? R.drawable.ic_btn_expand_less : R.drawable.ic_btn_expand_more,
        open ? R.string.dash_more_hide : R.string.dash_more_show);
  }

  private boolean usesBluetooth(Vehicle vehicle) {
    return "Bluetooth".equals(vehicle.getConnectionType());
  }

  /** USB: connect straight away. Bluetooth: open the list of cars to pick from. */
  private void connect() {
    Vehicle vehicle = mViewModel.getVehicle().getValue();
    if (vehicle == null) return;
    if (usesBluetooth(vehicle)) {
      Navigation.findNavController(requireView()).navigate(R.id.open_bluetooth_fragment);
    } else {
      vehicle.connectUsb();
      mViewModel.setUsbStatus(vehicle.isUsbConnected());
      refreshStatus();
    }
  }

  private void openConnectionScreen() {
    Vehicle vehicle = mViewModel.getVehicle().getValue();
    if (vehicle == null) return;
    Navigation.findNavController(requireView())
        .navigate(usesBluetooth(vehicle) ? R.id.open_bluetooth_fragment : R.id.open_usb_fragment);
  }

  private void toggleAvoidance() {
    boolean enabled = !preferencesManager.getObjectAvoidanceEnabled();
    preferencesManager.setObjectAvoidanceEnabled(enabled);
    Vehicle vehicle = mViewModel.getVehicle().getValue();
    if (vehicle != null) {
      vehicle.setObjectAvoidanceEnabled(enabled);
      vehicle.sendControl(); // a car held at an obstacle resumes, or stops, right now
    }
    refreshStatus();
  }

  /** Connection card and status chips, from the vehicle's live state. */
  private void refreshStatus() {
    if (binding == null || !isAdded()) return;
    Vehicle vehicle = mViewModel.getVehicle().getValue();
    boolean connected = vehicle != null && vehicle.isConnected();
    boolean bluetooth = vehicle != null && usesBluetooth(vehicle);
    boolean stopped = vehicle != null && vehicle.getEmergencyStop().isLatched();

    binding.carStatusDot.setBackgroundTintList(
        ColorStateList.valueOf(stopped ? RED : connected ? GREEN : GREY));
    if (connected) {
      binding.carStatusTitle.setText(
          bluetooth ? R.string.dash_title_connected_ble : R.string.dash_title_connected_usb);
      binding.carStatusDetail.setText(
          stopped ? R.string.dash_detail_stopped : R.string.dash_detail_connected);
    } else {
      binding.carStatusTitle.setText(R.string.dash_title_disconnected);
      binding.carStatusDetail.setText(bluetooth ? R.string.dash_detail_ble : R.string.dash_detail_usb);
    }
    binding.carConnectButton.setVisibility(connected ? View.GONE : View.VISIBLE);

    // LiDAR: live (green), stopped reporting (red: forward driving is blocked), or none seen.
    ObjectAvoidance.DataState lidar =
        vehicle != null ? vehicle.getProximity().getDataState() : ObjectAvoidance.DataState.NEVER;
    if (lidar == ObjectAvoidance.DataState.FRESH) {
      setChip(binding.chipLidar, R.string.dash_chip_lidar_live, GREEN);
    } else if (lidar == ObjectAvoidance.DataState.STALE) {
      setChip(binding.chipLidar, R.string.dash_chip_lidar_stale, RED);
    } else {
      setChip(binding.chipLidar, R.string.dash_chip_lidar_none, GREY);
    }

    // RC switch, from the car's 'm' report.
    if (rcMode == null) {
      setChip(binding.chipRc, R.string.dash_rc_none, GREY);
    } else if (rcMode == RcMode.MANUAL) {
      setChip(binding.chipRc, R.string.dash_rc_manual, AMBER);
    } else if (rcMode == RcMode.AUTO) {
      setChip(binding.chipRc, R.string.dash_rc_auto, GREEN);
    } else if (rcMode == RcMode.HOLD) {
      setChip(binding.chipRc, R.string.dash_rc_hold, AMBER);
    } else {
      setChip(binding.chipRc, R.string.dash_rc_lost, RED);
    }

    boolean avoid = preferencesManager.getObjectAvoidanceEnabled();
    setChip(
        binding.chipAvoidance,
        avoid ? R.string.dash_chip_avoid_on : R.string.dash_chip_avoid_off,
        avoid ? GREEN : AMBER);
  }

  private void setChip(com.google.android.material.chip.Chip chip, int text, int color) {
    chip.setText(text);
    chip.setChipBackgroundColor(ColorStateList.valueOf(color));
    chip.setTextColor(color == AMBER ? Color.rgb(0x29, 0x29, 0x29) : Color.WHITE);
  }

  @Override
  public void onDestroyView() {
    super.onDestroyView();
    refreshHandler.removeCallbacks(refresh);
    binding = null;
  }

  @Override
  public void onItemClick(SubCategory subCategory) {

    Timber.d("onItemClick: %s", subCategory.getTitle());

    switch (subCategory.getTitle()) {
      case FeatureList.PROJECTS:
        Navigation.findNavController(requireView()).navigate(R.id.projectsFragment);
        break;
      case FeatureList.FREE_ROAM:
        Navigation.findNavController(requireView())
            .navigate(R.id.action_mainFragment_to_freeRoamFragment);
        break;

      case FeatureList.GPS_NAVIGATION:
        Navigation.findNavController(requireView())
            .navigate(R.id.action_mainFragment_to_gpsNavigationFragment);
        break;

      case FeatureList.LIDAR_ROBOT:
        Navigation.findNavController(requireView())
            .navigate(R.id.action_mainFragment_to_lidarBotFragment);
        break;

      case FeatureList.INDOOR_SLAM:
        Navigation.findNavController(requireView()).navigate(R.id.action_mainFragment_to_slamFragment);
        break;

      case FeatureList.ROOMS_ZONES:
        Navigation.findNavController(requireView()).navigate(R.id.action_mainFragment_to_roomsFragment);
        break;

      case FeatureList.DATA_COLLECTION:
        Navigation.findNavController(requireView())
            .navigate(R.id.action_mainFragment_to_loggerFragment);
        break;

      case FeatureList.CONTROLLER:
        // For a library module, uncomment the following line
        // intent = new Intent(this, ControllerActivity.class);
        // startActivity(intent);
        break;

      case FeatureList.CONTROLLER_MAPPING:
        Navigation.findNavController(requireView())
            .navigate(R.id.action_mainFragment_to_controllerMappingFragment);
        break;

      case FeatureList.ROBOT_INFO:
        Navigation.findNavController(requireView())
            .navigate(R.id.action_mainFragment_to_robotInfoFragment);
        break;

      case FeatureList.AUTOPILOT:
        Navigation.findNavController(requireView())
            .navigate(R.id.action_mainFragment_to_autopilotFragment);
        break;

      case FeatureList.OBJECT_NAV:
        Navigation.findNavController(requireView())
            .navigate(R.id.action_mainFragment_to_objectNavFragment);
        break;

      case FeatureList.POINT_GOAL_NAVIGATION:
        Navigation.findNavController(requireView())
            .navigate(R.id.action_mainFragment_to_pointGoalNavigationFragment);
        break;

      case FeatureList.MODEL_MANAGEMENT:
        Navigation.findNavController(requireView())
            .navigate(R.id.action_mainFragment_to_modelManagementFragment);
        break;

      case FeatureList.DEFAULT:
        Intent intent = new Intent(requireActivity(), DefaultActivity.class);
        startActivity(intent);
        break;
    }
  }
}
