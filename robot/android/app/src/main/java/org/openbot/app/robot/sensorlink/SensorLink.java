package org.openbot.app.robot.sensorlink;

import android.content.Context;
import android.location.Location;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import org.openbot.app.robot.R;

/**
 * The phone's SensorServer link, shared by GPS Mission, Object Tracking and Autopilot.
 *
 * <ul>
 *   <li><b>Server</b>: this phone streams its sensors and GPS over the SensorServer WebSocket API,
 *       so a laptop or another device can watch or log them while the car drives.
 *   <li><b>Client</b>: this phone reads GPS and compass from a SensorServer app on another phone
 *       (for example one mounted high on the car, away from the motors), and GPS Mission can drive
 *       on them.
 * </ul>
 *
 * The link runs only while one of those screens is open: each screen attaches in onResume and
 * detaches in onPause. Status and remote data are delivered on the main thread.
 */
public final class SensorLink {

  /** Remote data for GPS Mission (client mode with "use its GPS" ticked). */
  public interface RemoteListener {
    void onRemoteLocation(Location location);

    void onRemoteRotationVector(float[] values);
  }

  /** A remote fix older than this no longer counts, and the phone's own GPS takes over again. */
  public static final long REMOTE_STALE_MS = 3000L;

  private static final SensorLink INSTANCE = new SensorLink();

  public static SensorLink get() {
    return INSTANCE;
  }

  private final Handler main = new Handler(Looper.getMainLooper());
  private final List<Runnable> statusListeners = new CopyOnWriteArrayList<>();
  private final List<RemoteListener> remoteListeners = new CopyOnWriteArrayList<>();

  private int attachCount;
  private SensorLinkSettings settings = new SensorLinkSettings();
  private SensorLinkServer server;
  private SensorLinkClient client;

  // Status, main thread only.
  private boolean serverRunning;
  private String serverError;
  private int serverClients;
  private boolean clientConnected;
  private float remoteAccuracy = Float.NaN;
  private long remoteFixAtMs;

  // Browser settings: the camera everywhere, plus whatever the open screen adds.
  private final RemoteSettings remoteSettings = new RemoteSettings();
  private volatile String screenName = "OpenBot";
  private RemoteSettings.Provider screenProvider;

  private SensorLink() {
    remoteSettings.addProvider(CameraHub.get());
  }

  /** The open screen's settings for the browser page (driving, tracking...). Main thread. */
  public void setScreen(String name, RemoteSettings.Provider provider) {
    if (screenProvider != null) remoteSettings.removeProvider(screenProvider);
    screenName = name;
    screenProvider = provider;
    if (provider != null) remoteSettings.addProvider(provider);
  }

  public void clearScreen(RemoteSettings.Provider provider) {
    if (screenProvider == provider && provider != null) {
      remoteSettings.removeProvider(provider);
      screenProvider = null;
      screenName = "OpenBot";
    }
  }

  public void attach(Context context) {
    attachCount++;
    if (attachCount == 1) start(context.getApplicationContext());
  }

  public void detach() {
    if (attachCount == 0) return;
    attachCount--;
    if (attachCount == 0) stop();
  }

  /** Apply new settings now, if a screen has the link running. */
  public void restart(Context context) {
    if (attachCount == 0) return;
    stop();
    start(context.getApplicationContext());
  }

  public void addStatusListener(Runnable l) {
    statusListeners.add(l);
  }

  public void removeStatusListener(Runnable l) {
    statusListeners.remove(l);
  }

  public void addRemoteListener(RemoteListener l) {
    remoteListeners.add(l);
  }

  public void removeRemoteListener(RemoteListener l) {
    remoteListeners.remove(l);
  }

  /** True while GPS Mission is (or should be) driving on the other phone's GPS. */
  public boolean isRemoteGpsActive() {
    return client != null
        && settings.useRemoteGps
        && SystemClock.elapsedRealtime() - remoteFixAtMs < REMOTE_STALE_MS;
  }

  /** One line for the screens: what the link is doing right now. */
  public String statusText(Context context) {
    switch (settings.mode) {
      case SERVER:
        if (serverError != null) {
          return context.getString(R.string.sensor_link_server_failed, serverError);
        }
        String ip = localIpv4();
        if (ip == null) {
          return context.getString(R.string.sensor_link_server_no_ip, settings.port);
        }
        return context.getString(
            R.string.sensor_link_server, "http://" + ip + ":" + settings.port, serverClients);
      case CLIENT:
        String base = SensorLinkProtocol.baseUrl(settings.address);
        if (base == null) return context.getString(R.string.sensor_link_client_no_address);
        String host = base.substring("ws://".length());
        if (!clientConnected) return context.getString(R.string.sensor_link_client_connecting, host);
        if (Float.isNaN(remoteAccuracy)
            || SystemClock.elapsedRealtime() - remoteFixAtMs > REMOTE_STALE_MS) {
          return context.getString(R.string.sensor_link_client_connected, host);
        }
        return context.getString(R.string.sensor_link_client_gps, host, remoteAccuracy);
      default:
        return context.getString(R.string.sensor_link_off);
    }
  }

  private void start(Context app) {
    settings = SensorLinkSettings.load(app);
    resetStatus();
    if (settings.mode == SensorLinkSettings.Mode.SERVER) {
      server =
          new SensorLinkServer(
              app,
              settings.port,
              new SensorLinkServer.Callback() {
                @Override
                public void onServerStarted() {
                  main.post(() -> {
                    serverRunning = true;
                    serverError = null;
                    notifyStatus();
                  });
                }

                @Override
                public void onServerFailed(String reason) {
                  main.post(() -> {
                    serverRunning = false;
                    serverError = reason;
                    notifyStatus();
                  });
                }

                @Override
                public void onClientCountChanged(int count) {
                  main.post(() -> {
                    serverClients = count;
                    notifyStatus();
                  });
                }
              },
              remoteSettings,
              () -> screenName);
      server.start();
    } else if (settings.mode == SensorLinkSettings.Mode.CLIENT) {
      String base = SensorLinkProtocol.baseUrl(settings.address);
      if (base != null) {
        client =
            new SensorLinkClient(
                base,
                settings.useRemoteGps,
                new SensorLinkClient.Callback() {
                  @Override
                  public void onClientConnected(boolean connected) {
                    clientConnected = connected;
                    notifyStatus();
                  }

                  @Override
                  public void onRemoteFix(SensorLinkProtocol.GpsFix fix) {
                    remoteAccuracy = fix.accuracy;
                    remoteFixAtMs = SystemClock.elapsedRealtime();
                    if (settings.useRemoteGps) {
                      Location l = toLocation(fix);
                      for (RemoteListener r : remoteListeners) r.onRemoteLocation(l);
                    }
                    notifyStatus();
                  }

                  @Override
                  public void onRemoteRotationVector(float[] values) {
                    if (!settings.useRemoteGps) return;
                    for (RemoteListener r : remoteListeners) r.onRemoteRotationVector(values);
                  }
                });
        client.start();
      }
    }
    notifyStatus();
  }

  private void stop() {
    final SensorLinkServer s = server;
    server = null;
    if (s != null) {
      // stop() waits for the server thread; keep that off the main thread.
      new Thread(s::shutdown, "SensorLinkServerStop").start();
    }
    if (client != null) {
      client.stop();
      client = null;
    }
    resetStatus();
    notifyStatus();
  }

  private void resetStatus() {
    serverRunning = false;
    serverError = null;
    serverClients = 0;
    clientConnected = false;
    remoteAccuracy = Float.NaN;
    remoteFixAtMs = 0;
  }

  private void notifyStatus() {
    for (Runnable l : statusListeners) l.run();
  }

  static Location toLocation(SensorLinkProtocol.GpsFix fix) {
    Location l = new Location("sensorserver");
    l.setLatitude(fix.latitude);
    l.setLongitude(fix.longitude);
    l.setAltitude(fix.altitude);
    if (!Float.isNaN(fix.accuracy)) l.setAccuracy(fix.accuracy);
    if (!Float.isNaN(fix.bearing)) l.setBearing(fix.bearing);
    if (!Float.isNaN(fix.speed)) l.setSpeed(fix.speed);
    l.setTime(fix.timeMs > 0 ? fix.timeMs : System.currentTimeMillis());
    l.setElapsedRealtimeNanos(SystemClock.elapsedRealtimeNanos());
    return l;
  }

  /** The address other devices on the same Wi-Fi or hotspot can reach this phone at. */
  static String localIpv4() {
    try {
      String fallback = null;
      for (NetworkInterface nif : Collections.list(NetworkInterface.getNetworkInterfaces())) {
        if (!nif.isUp() || nif.isLoopback()) continue;
        for (InetAddress a : Collections.list(nif.getInetAddresses())) {
          if (!(a instanceof Inet4Address) || !a.isSiteLocalAddress()) continue;
          String name = nif.getName();
          // Wi-Fi or hotspot first; mobile data (rmnet, ccmni) is rarely reachable.
          if (name.startsWith("wlan") || name.startsWith("ap") || name.startsWith("swlan")) {
            return a.getHostAddress();
          }
          if (fallback == null) fallback = a.getHostAddress();
        }
      }
      return fallback;
    } catch (Exception e) {
      return null;
    }
  }
}
