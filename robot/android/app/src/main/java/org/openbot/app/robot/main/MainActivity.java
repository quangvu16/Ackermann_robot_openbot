package org.openbot.app.robot.main;

import static org.openbot.app.robot.utils.Constants.DEVICE_ACTION_DATA_RECEIVED;

import android.content.BroadcastReceiver;
import android.os.Handler;
import android.os.Looper;
import android.view.Gravity;
import android.view.View;
import android.view.ViewGroup;
import android.widget.FrameLayout;
import androidx.core.view.ViewCompat;
import androidx.core.view.WindowInsetsCompat;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.view.InputDevice;
import android.view.KeyEvent;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import androidx.appcompat.app.AppCompatActivity;
import androidx.appcompat.widget.Toolbar;
import androidx.core.content.ContextCompat;
import androidx.lifecycle.ViewModelProvider;
import androidx.localbroadcastmanager.content.LocalBroadcastManager;
import androidx.navigation.NavController;
import androidx.navigation.Navigation;
import androidx.navigation.fragment.NavHostFragment;
import androidx.navigation.ui.AppBarConfiguration;
import androidx.navigation.ui.NavigationUI;
import com.google.android.material.bottomnavigation.BottomNavigationView;
import java.util.Objects;
import org.openbot.app.robot.OpenBotApplication;
import org.openbot.app.robot.R;
import org.openbot.app.robot.utils.Constants;
import org.openbot.app.robot.vehicle.RcMode;
import org.openbot.app.robot.vehicle.UsbConnection;
import org.openbot.app.robot.vehicle.Vehicle;
import timber.log.Timber;

// For a library module, uncomment the following line
// import org.openbot.app.robot.controller.ControllerActivity;

public class MainActivity extends AppCompatActivity {

  private MainViewModel viewModel;
  private BroadcastReceiver localBroadcastReceiver;
  private Vehicle vehicle;
  private LocalBroadcastManager localBroadcastManager;
  private BottomNavigationView bottomNavigationView;
  private NavController navController;

  // Full-screen face for the car's RC switch: smile in MANUAL, cry in HOLD / LOST. The firmware
  // repeats its 'm' report every second; if reports stop (cable pulled, car off) the face goes.
  private static final long RC_REPORT_TIMEOUT_MS = 3000;
  private RcModeFaceView rcModeFace;
  private final Handler rcModeHandler = new Handler(Looper.getMainLooper());
  private final Runnable rcModeTimeout =
      () -> {
        rcModeFace.setMode(null);
        viewModel.setRcMode(null);
      };

  // The big red STOP button: on every screen while the car is connected (or while stopped).
  private StopButtonView stopButton;
  // Where STOP sits: bottom-right above the bottom bar on the home screens, top-right on driving
  // screens (no toolbar there, and the bottom belongs to their control sheets).
  private boolean stopOnHomeScreen = true;
  private int systemBarTop = 0;
  private int systemBarBottom = 0;
  private final Runnable stopButtonRefresh =
      new Runnable() {
        @Override
        public void run() {
          refreshStopButton();
          rcModeHandler.postDelayed(this, 1000);
        }
      };

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_main);
    // STOP first, the RC face after it: in MANUAL the face covers STOP, because then the RC
    // transmitter drives and the app's STOP does not apply.
    stopButton = new StopButtonView(this);
    float density = getResources().getDisplayMetrics().density;
    int stopSize = (int) (88 * density);
    FrameLayout.LayoutParams stopParams =
        new FrameLayout.LayoutParams(stopSize, stopSize, Gravity.END | Gravity.BOTTOM);
    stopParams.setMarginEnd((int) (16 * density));
    stopButton.setVisibility(View.GONE);
    stopButton.setOnClickListener(
        v -> {
          if (vehicle != null) vehicle.getEmergencyStop().toggle();
        });
    ((ViewGroup) getWindow().getDecorView()).addView(stopButton, stopParams);
    // The window is edge-to-edge, so the system bars' height is added to STOP's margins.
    ViewCompat.setOnApplyWindowInsetsListener(
        stopButton,
        (v, insets) -> {
          systemBarTop = insets.getInsets(WindowInsetsCompat.Type.systemBars()).top;
          systemBarBottom = insets.getInsets(WindowInsetsCompat.Type.systemBars()).bottom;
          placeStopButton();
          return insets;
        });

    // Added to the window's decor view, so it covers the toolbar, every screen and the bottom bar.
    rcModeFace = new RcModeFaceView(this);
    ((ViewGroup) getWindow().getDecorView())
        .addView(
            rcModeFace,
            new ViewGroup.LayoutParams(
                ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT));
    viewModel = new ViewModelProvider(this).get(MainViewModel.class);
    vehicle = OpenBotApplication.vehicle;
    bottomNavigationView = findViewById(R.id.bottomNavigationView);
    bottomNavigationView.setSelectedItemId(R.id.mainFragment);
    //    if (vehicle == null) {
    //      SharedPreferences sharedPreferences =
    // PreferenceManager.getDefaultSharedPreferences(this);
    //      int baudRate = Integer.parseInt(sharedPreferences.getString("baud_rate", "115200"));
    //      vehicle = new Vehicle(this, baudRate);
    //      vehicle.connectUsb();
    viewModel.setVehicle(vehicle);
    //    }
    if (vehicle != null) {
      vehicle
          .getEmergencyStop()
          .addListener(latched -> runOnUiThread(this::refreshStopButton));
    }
    rcModeHandler.post(stopButtonRefresh);

    localBroadcastReceiver =
        new BroadcastReceiver() {
          @Override
          public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();

            if (action != null) {
              switch (action) {
                case UsbManager.ACTION_USB_DEVICE_ATTACHED:
                  if (!vehicle.isUsbConnected()) {
                    vehicle.connectUsb();
                    viewModel.setUsbStatus(vehicle.isUsbConnected());
                  }
                  Timber.i("USB device attached");
                  break;

                  // Case activated when app is not set to open default when usb is connected
                case UsbConnection.ACTION_USB_PERMISSION:
                  synchronized (this) {
                    UsbDevice usbDevice = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);
                    if (intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) {
                      if (usbDevice != null) {
                        // call method to set up device communication
                        if (!vehicle.isUsbConnected()) {
                          vehicle.connectUsb();
                        }
                        viewModel.setUsbStatus(vehicle.isUsbConnected());
                        Timber.i("USB device attached");
                      }
                    }
                  }

                  break;
                case UsbManager.ACTION_USB_DEVICE_DETACHED:
                  vehicle.disconnectUsb();
                  viewModel.setUsbStatus(vehicle.isUsbConnected());
                  Timber.i("USB device detached");
                  break;
                case DEVICE_ACTION_DATA_RECEIVED:
                  String data = intent.getStringExtra("data");
                  viewModel.setDeviceData(data);
                  onRcModeMessage(data);
                  break;
              }
            }
          }
        };
    IntentFilter localIntentFilter = new IntentFilter();
    localIntentFilter.addAction(DEVICE_ACTION_DATA_RECEIVED);
    localIntentFilter.addAction(UsbManager.ACTION_USB_DEVICE_DETACHED);
    localIntentFilter.addAction(UsbManager.ACTION_USB_DEVICE_ATTACHED);
    localIntentFilter.addAction(UsbConnection.ACTION_USB_PERMISSION);

    localBroadcastManager = LocalBroadcastManager.getInstance(this);
    localBroadcastManager.registerReceiver(localBroadcastReceiver, localIntentFilter);

    ContextCompat.registerReceiver(
        this, localBroadcastReceiver, localIntentFilter, ContextCompat.RECEIVER_EXPORTED);

    NavHostFragment navHostFragment =
        (NavHostFragment) getSupportFragmentManager().findFragmentById(R.id.nav_host_fragment);
    navController = navHostFragment.getNavController();
    AppBarConfiguration appBarConfiguration =
        new AppBarConfiguration.Builder(navController.getGraph()).build();
    Toolbar toolbar = findViewById(R.id.toolbar);
    setSupportActionBar(toolbar);

    bottomNavigationView.setOnItemReselectedListener(
        item -> {
          // Do nothing when the selected item is already selected
        });

    NavigationUI.setupWithNavController(toolbar, navController, appBarConfiguration);
    NavigationUI.setupWithNavController(bottomNavigationView, navController);

    navController.addOnDestinationChangedListener(
        (controller, destination, arguments) -> {
          if (destination.getId() == R.id.mainFragment
              || destination.getId() == R.id.settingsFragment
              || destination.getId() == R.id.usbFragment
              || destination.getId() == R.id.projectsFragment
              || destination.getId() == R.id.profileFragment) {
            toolbar.setVisibility(View.VISIBLE);
            bottomNavigationView.setVisibility(View.VISIBLE);
            stopOnHomeScreen = true;
          } else {
            toolbar.setVisibility(View.GONE);
            bottomNavigationView.setVisibility(View.GONE);
            stopOnHomeScreen = false;
          }
          placeStopButton();

          // To update the toolbar icon according to the Fragment.
          Menu menu = toolbar.getMenu();
          if (destination.getId() == R.id.projectsFragment) {
            menu.findItem(R.id.settingsFragment).setVisible(false);
            menu.findItem(R.id.barCodeScannerFragment).setVisible(true);
          } else {
            menu.findItem(R.id.barCodeScannerFragment).setVisible(false);
            if (vehicle.getConnectionType().equals("Bluetooth")) {
              menu.findItem(R.id.bluetoothFragment).setVisible(true);
            }
            menu.findItem(R.id.settingsFragment).setVisible(true);
          }
        });

    //    if (savedInstanceState == null) {
    //      // Default to open this when app opens
    //      Intent intent = new Intent(this, DefaultActivity.class);
    //      startActivity(intent);
    //    }
  }

  @Override
  public boolean onCreateOptionsMenu(Menu menu) {
    // Inflate the menu; this adds items to the action bar if it is present.
    getMenuInflater().inflate(R.menu.menu_items, menu);
    // Get the current destination id.
    int currentDestinationId =
        Objects.requireNonNull(navController.getCurrentDestination()).getId();
    if (currentDestinationId == R.id.projectsFragment) {
      menu.findItem(R.id.barCodeScannerFragment).setVisible(true);
      menu.findItem(R.id.settingsFragment).setVisible(false);
    } else {
      menu.findItem(R.id.barCodeScannerFragment).setVisible(false);
    }
    if (vehicle.getConnectionType().equals("Bluetooth")) {
      menu.findItem(R.id.usbFragment).setVisible(false);
      menu.findItem(R.id.bluetoothFragment).setVisible(true);
    } else if (vehicle.getConnectionType().equals("USB")) {
      menu.findItem(R.id.usbFragment).setVisible(true);
      menu.findItem(R.id.bluetoothFragment).setVisible(false);
    }
    return true;
  }

  @Override
  public boolean onOptionsItemSelected(MenuItem item) {
    NavController navController = Navigation.findNavController(this, R.id.nav_host_fragment);
    if (item.getItemId() == R.id.barCodeScannerFragment) {
      navController.navigate(R.id.barCodeScannerFragment);
      return true;
    }

    return NavigationUI.onNavDestinationSelected(item, navController)
        || super.onOptionsItemSelected(item);
  }

  @Override
  public boolean dispatchGenericMotionEvent(MotionEvent event) {
    if ((event.getSource() & InputDevice.SOURCE_JOYSTICK) == InputDevice.SOURCE_JOYSTICK
        && event.getAction() == MotionEvent.ACTION_MOVE) {
      Bundle bundle = new Bundle();
      bundle.putParcelable(Constants.DATA, event);
      getSupportFragmentManager().setFragmentResult(Constants.GENERIC_MOTION_EVENT, bundle);
      return true;
    }
    return super.dispatchGenericMotionEvent(event);
  }

  @Override
  public boolean dispatchKeyEvent(KeyEvent event) {
    Bundle bundle = new Bundle();
    bundle.putParcelable(Constants.DATA_CONTINUOUS, event);
    getSupportFragmentManager().setFragmentResult(Constants.KEY_EVENT_CONTINUOUS, bundle);

    // Check that the event came from a game controller
    if ((event.getSource() & InputDevice.SOURCE_GAMEPAD) == InputDevice.SOURCE_GAMEPAD) {
      bundle.putParcelable(Constants.DATA, event);
      getSupportFragmentManager().setFragmentResult(Constants.KEY_EVENT, bundle);
      return true;
    }
    return super.dispatchKeyEvent(event);
  }

  /** Handles the car's {@code m<letter>} RC mode report (USB and Bluetooth both arrive here). */
  private void onRcModeMessage(String data) {
    if (data == null || data.length() < 2 || data.charAt(0) != 'm') return;
    RcMode mode = RcMode.parse(data.substring(1));
    if (mode == null) return;
    rcModeFace.setMode(mode);
    viewModel.setRcMode(mode);
    rcModeHandler.removeCallbacks(rcModeTimeout);
    rcModeHandler.postDelayed(rcModeTimeout, RC_REPORT_TIMEOUT_MS);
  }

  /**
   * Home screens: bottom-right, above the bottom navigation bar (the usual floating-button spot).
   * Driving screens: top-right, where the hidden toolbar was, clear of their bottom control sheets.
   */
  private void placeStopButton() {
    if (stopButton == null) return;
    float density = getResources().getDisplayMetrics().density;
    FrameLayout.LayoutParams lp = (FrameLayout.LayoutParams) stopButton.getLayoutParams();
    if (stopOnHomeScreen) {
      lp.gravity = Gravity.END | Gravity.BOTTOM;
      lp.topMargin = 0;
      lp.bottomMargin = (int) ((56 + 20) * density) + systemBarBottom;
    } else {
      lp.gravity = Gravity.END | Gravity.TOP;
      lp.topMargin = (int) (12 * density) + systemBarTop;
      lp.bottomMargin = 0;
    }
    stopButton.setLayoutParams(lp);
  }

  /** Shows STOP while connected, RESUME while stopped, nothing otherwise. */
  private void refreshStopButton() {
    if (stopButton == null || vehicle == null) return;
    boolean latched = vehicle.getEmergencyStop().isLatched();
    stopButton.setLatched(latched);
    stopButton.setVisibility(latched || vehicle.isConnected() ? View.VISIBLE : View.GONE);
  }

  @Override
  public synchronized void onDestroy() {
    rcModeHandler.removeCallbacks(rcModeTimeout);
    rcModeHandler.removeCallbacks(stopButtonRefresh);
    if (localBroadcastManager != null) {
      localBroadcastManager.unregisterReceiver(localBroadcastReceiver);
      localBroadcastManager = null;
    }

    unregisterReceiver(localBroadcastReceiver);
    if (localBroadcastReceiver != null) localBroadcastReceiver = null;
    if (!isChangingConfigurations()) vehicle.disconnectUsb();
    super.onDestroy();
  }

  @Override
  protected void onResume() {
    super.onResume();
  }

  @Override
  protected void onPause() {
    super.onPause();
  }
}
