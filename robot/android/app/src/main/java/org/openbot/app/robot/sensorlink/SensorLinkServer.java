package org.openbot.app.robot.sensorlink;

import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.os.Handler;
import android.os.HandlerThread;
import androidx.core.content.ContextCompat;
import com.google.android.gms.location.FusedLocationProviderClient;
import com.google.android.gms.location.LocationCallback;
import com.google.android.gms.location.LocationRequest;
import com.google.android.gms.location.LocationResult;
import com.google.android.gms.location.LocationServices;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.CopyOnWriteArraySet;
import timber.log.Timber;

/**
 * A SensorServer-compatible server for this phone's sensors and GPS, plus a live dashboard for
 * browsers at {@code http://<phone>:<port>/}. Any SensorServer client (Python scripts, Unity, ROS
 * bridges...) can read the same data. See {@link SensorLinkProtocol} for the API.
 *
 * <p>Sensors are only switched on while someone is connected to them, and off again when the last
 * client leaves, so an idle server costs no battery.
 */
final class SensorLinkServer extends MiniWebServer implements RemoteSettings.Provider {

  interface Callback {
    void onServerStarted();

    void onServerFailed(String reason);

    void onClientCountChanged(int count);
  }

  // Close codes in the private-use range (4000-4999).
  private static final int CLOSE_UNKNOWN_PATH = 4000;
  private static final int CLOSE_NO_SENSOR = 4001;
  private static final int CLOSE_NO_PERMISSION = 4002;

  private static final String DASHBOARD_ASSET = "sensorlink/index.html";

  private final Context context;
  private final Callback callback;
  private final SensorManager sensorManager;
  private final FusedLocationProviderClient locationClient;
  private final HandlerThread sensorThread = new HandlerThread("SensorLinkServer");
  private final Handler sensorHandler;
  private byte[] dashboard;

  /** What one connection asked for; {@code multi} adds the type to each message. */
  private static final class Subscription {
    final boolean multi;
    final boolean gps;

    Subscription(boolean multi, boolean gps) {
      this.multi = multi;
      this.gps = gps;
    }
  }

  // Browser control (/control WebSocket): settings for camera, driving and sensors.
  private final RemoteSettings remoteSettings;
  private final ScreenName screenName;
  private final Handler main = new Handler(android.os.Looper.getMainLooper());
  private final Set<Connection> controlClients = new CopyOnWriteArraySet<>();
  private final Runnable pushState = this::pushStateNow;
  private volatile String sensorRate = "game";
  static final java.util.List<String> SENSOR_RATES = java.util.Arrays.asList("ui", "game", "fastest");

  /** The name of the screen that has the link open, for the page header. */
  interface ScreenName {
    String get();
  }

  private final Map<String, TypeListener> listeners = new HashMap<>(); // guarded by this
  private final Set<Connection> clients = new CopyOnWriteArraySet<>();
  private final Set<Connection> gpsClients = new CopyOnWriteArraySet<>();
  private LocationCallback locationCallback; // guarded by this
  private volatile Location lastLocation;

  SensorLinkServer(
      Context context, int port, Callback callback, RemoteSettings remoteSettings, ScreenName screenName) {
    super(port);
    this.context = context.getApplicationContext();
    this.callback = callback;
    this.remoteSettings = remoteSettings;
    this.screenName = screenName;
    remoteSettings.addProvider(this);
    this.sensorManager = (SensorManager) this.context.getSystemService(Context.SENSOR_SERVICE);
    this.locationClient = LocationServices.getFusedLocationProviderClient(this.context);
    sensorThread.start();
    sensorHandler = new Handler(sensorThread.getLooper());
  }

  /** Stops the server and every sensor it switched on. Safe to call more than once. */
  void shutdown() {
    remoteSettings.removeProvider(this);
    main.removeCallbacks(pushState);
    stop();
    synchronized (this) {
      if (sensorManager != null) {
        for (TypeListener l : listeners.values()) sensorManager.unregisterListener(l);
      }
      listeners.clear();
      stopGpsLocked();
    }
    clients.clear();
    gpsClients.clear();
    sensorThread.quitSafely();
  }

  @Override
  protected void onStarted() {
    Timber.i("Sensor link server listening");
    callback.onServerStarted();
  }

  @Override
  protected void onFailed(Exception e) {
    Timber.e(e, "Sensor link server failed");
    callback.onServerFailed(e.getMessage() == null ? e.toString() : e.getMessage());
  }

  // region HTTP: dashboard and sensor list

  @Override
  protected HttpResponse onHttpRequest(String path) {
    int q = path.indexOf('?');
    String p = q >= 0 ? path.substring(0, q) : path;
    switch (p) {
      case "/":
      case "/index.html":
        byte[] page = dashboard();
        return page == null
            ? HttpResponse.text(500, "text/plain", "Dashboard missing from the app\n")
            : new HttpResponse(200, "text/html; charset=utf-8", page);
      case "/sensors":
        return HttpResponse.text(200, "application/json", sensorListJson());
      case "/camera.mjpg":
        return HttpResponse.stream("multipart/x-mixed-replace; boundary=frame", this::streamCamera);
      case "/camera.jpg":
        return snapshot();
      default:
        return null;
    }
  }

  private synchronized byte[] dashboard() {
    if (dashboard == null) {
      try (InputStream in = context.getAssets().open(DASHBOARD_ASSET)) {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        byte[] buf = new byte[8192];
        int n;
        while ((n = in.read(buf)) > 0) out.write(buf, 0, n);
        dashboard = out.toByteArray();
      } catch (IOException e) {
        Timber.e(e, "Sensor link dashboard asset missing");
      }
    }
    return dashboard;
  }

  /** {@code [{"type":"android.sensor.accelerometer","name":"..."}]}, one entry per type. */
  private String sensorListJson() {
    StringBuilder b = new StringBuilder("[");
    if (sensorManager != null) {
      Set<String> seen = new java.util.HashSet<>();
      for (Sensor s : sensorManager.getSensorList(Sensor.TYPE_ALL)) {
        String type = s.getStringType();
        if (type == null || !type.startsWith("android.sensor.") || !seen.add(type)) continue;
        if (b.length() > 1) b.append(',');
        b.append("{\"type\":\"")
            .append(SensorLinkProtocol.escapeJson(type))
            .append("\",\"name\":\"")
            .append(SensorLinkProtocol.escapeJson(s.getName()))
            .append("\"}");
      }
    }
    return b.append(']').toString();
  }

  // endregion

  // region Camera over HTTP

  /** MJPEG: one JPEG part per frame until the browser goes away or the server stops. */
  private void streamCamera(java.io.OutputStream out) throws IOException {
    CameraHub hub = CameraHub.get();
    hub.addViewer();
    try {
      long[] seq = {0};
      while (isRunning()) {
        byte[] jpeg;
        try {
          jpeg = hub.awaitFrame(seq[0], 1000, seq);
        } catch (InterruptedException e) {
          return;
        }
        if (jpeg == null) continue;
        out.write(
            ("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + jpeg.length + "\r\n\r\n")
                .getBytes(java.nio.charset.StandardCharsets.US_ASCII));
        out.write(jpeg);
        out.write("\r\n".getBytes(java.nio.charset.StandardCharsets.US_ASCII));
        out.flush();
      }
    } finally {
      hub.removeViewer();
    }
  }

  private HttpResponse snapshot() {
    CameraHub hub = CameraHub.get();
    hub.addViewer();
    try {
      byte[] jpeg = hub.awaitFrame(0, 4000, new long[1]);
      if (jpeg != null) return new HttpResponse(200, "image/jpeg", jpeg);
      String why = hub.getUnavailableReason();
      return HttpResponse.text(503, "text/plain", (why == null ? "No camera picture yet" : why) + "\n");
    } catch (InterruptedException e) {
      return HttpResponse.text(503, "text/plain", "Interrupted\n");
    } finally {
      hub.removeViewer();
    }
  }

  // endregion

  // region Browser control: /control WebSocket

  private void openControl(Connection conn) {
    controlClients.add(conn);
    main.post(pushState);
  }

  /** Settings state for every control client; repeats every 2 s so phone-side changes show too. */
  private void pushStateNow() {
    main.removeCallbacks(pushState);
    if (controlClients.isEmpty() || !isRunning()) return;
    String state = stateJson();
    for (Connection c : controlClients) c.send(state);
    main.postDelayed(pushState, 2000);
  }

  private String stateJson() {
    String base = remoteSettings.stateJson(screenName.get());
    String why = CameraHub.get().getUnavailableReason();
    // Append the camera status: {"screen":..,"settings":[..],"camera":"<reason or empty>"}
    return base.substring(0, base.length() - 1)
        + ",\"camera\":\""
        + SensorLinkProtocol.escapeJson(why == null ? "" : why)
        + "\"}";
  }

  private void handleControl(String message) {
    main.post(
        () -> {
          remoteSettings.apply(message);
          pushStateNow();
        });
  }

  /** Sensor-link settings: how fast sensors report. */
  @Override
  public void addSettings(java.util.List<RemoteSettings.Setting> out) {
    out.add(
        RemoteSettings.Setting.choice(
            "sensors.rate", "Sensors", "Sensor speed (ui ≈ 15/s, game ≈ 50/s, fastest)", SENSOR_RATES,
            () -> sensorRate,
            v -> setSensorRate((String) v)));
  }

  private int sensorDelay() {
    switch (sensorRate) {
      case "ui":
        return SensorManager.SENSOR_DELAY_UI;
      case "fastest":
        return SensorManager.SENSOR_DELAY_FASTEST;
      default:
        return SensorManager.SENSOR_DELAY_GAME;
    }
  }

  private synchronized void setSensorRate(String rate) {
    if (rate.equals(sensorRate)) return;
    sensorRate = rate;
    for (TypeListener l : listeners.values()) {
      Sensor sensor = findSensor(l.type);
      sensorManager.unregisterListener(l);
      if (sensor != null) sensorManager.registerListener(l, sensor, sensorDelay(), sensorHandler);
    }
  }

  // endregion

  // region WebSocket: SensorServer API

  @Override
  protected void onOpen(Connection conn, String path) {
    if (path.equals("/control") || path.startsWith("/control?")) {
      openControl(conn);
      return;
    }
    SensorLinkProtocol.Request request = SensorLinkProtocol.parse(path);
    switch (request.kind) {
      case GPS:
        if (ContextCompat.checkSelfPermission(context, Manifest.permission.ACCESS_FINE_LOCATION)
            != PackageManager.PERMISSION_GRANTED) {
          conn.close(CLOSE_NO_PERMISSION, "Location permission not granted on the phone");
          return;
        }
        conn.tag = new Subscription(false, true);
        clients.add(conn);
        gpsClients.add(conn);
        synchronized (this) {
          startGpsLocked();
        }
        Location l = lastLocation;
        if (l != null) conn.send(SensorLinkProtocol.gpsJson(toFix(l)));
        break;
      case SENSOR:
      case SENSORS:
        for (String type : request.types) {
          if (findSensor(type) == null) {
            conn.close(CLOSE_NO_SENSOR, "Sensor not available: " + type);
            return;
          }
        }
        conn.tag = new Subscription(request.kind == SensorLinkProtocol.Kind.SENSORS, false);
        clients.add(conn);
        synchronized (this) {
          for (String type : request.types) subscribeLocked(type, conn);
        }
        break;
      default:
        conn.close(CLOSE_UNKNOWN_PATH, "Unknown path " + path);
        return;
    }
    callback.onClientCountChanged(clients.size());
  }

  @Override
  protected void onClose(Connection conn) {
    controlClients.remove(conn);
    if (!clients.remove(conn)) return;
    synchronized (this) {
      if (gpsClients.remove(conn) && gpsClients.isEmpty()) stopGpsLocked();
      for (Map.Entry<String, TypeListener> e : new HashMap<>(listeners).entrySet()) {
        TypeListener l = e.getValue();
        l.clients.remove(conn);
        if (l.clients.isEmpty()) {
          sensorManager.unregisterListener(l);
          listeners.remove(e.getKey());
        }
      }
    }
    callback.onClientCountChanged(clients.size());
  }

  @Override
  protected void onMessage(Connection conn, String message) {
    if (controlClients.contains(conn)) {
      handleControl(message);
      return;
    }
    if (gpsClients.contains(conn)
        && SensorLinkProtocol.GET_LAST_KNOWN_LOCATION.equals(message.trim())) {
      Location l = lastLocation;
      if (l != null) conn.send(SensorLinkProtocol.gpsJson(toFix(l)));
    }
  }

  // endregion

  // region Sensors

  private Sensor findSensor(String type) {
    if (sensorManager == null) return null;
    List<Sensor> all = sensorManager.getSensorList(Sensor.TYPE_ALL);
    for (Sensor s : all) {
      if (type.equals(s.getStringType())) return s;
    }
    return null;
  }

  private void subscribeLocked(String type, Connection conn) {
    TypeListener l = listeners.get(type);
    if (l == null) {
      Sensor sensor = findSensor(type);
      if (sensor == null) return;
      l = new TypeListener(type);
      listeners.put(type, l);
      sensorManager.registerListener(l, sensor, sensorDelay(), sensorHandler);
    }
    l.clients.add(conn);
  }

  /** One registered sensor, fanned out to every client that asked for it. */
  private final class TypeListener implements SensorEventListener {
    final String type;
    final Set<Connection> clients = new CopyOnWriteArraySet<>();

    TypeListener(String type) {
      this.type = type;
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
      String single = null;
      String multi = null;
      for (Connection c : clients) {
        if (!c.isOpen()) continue;
        Subscription sub = (Subscription) c.tag;
        if (sub != null && sub.multi) {
          if (multi == null)
            multi = SensorLinkProtocol.sensorJson(type, event.values, event.timestamp, event.accuracy);
          c.send(multi);
        } else {
          if (single == null)
            single = SensorLinkProtocol.sensorJson(null, event.values, event.timestamp, event.accuracy);
          c.send(single);
        }
      }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {}
  }

  // endregion

  // region GPS

  @SuppressWarnings("deprecation") // play-services-location 19.x API, as in GpsNavLocationProvider
  private void startGpsLocked() {
    if (locationCallback != null) return;
    locationCallback =
        new LocationCallback() {
          @Override
          public void onLocationResult(LocationResult result) {
            Location l = result.getLastLocation();
            if (l == null) return;
            lastLocation = l;
            String msg = SensorLinkProtocol.gpsJson(toFix(l));
            for (Connection c : gpsClients) c.send(msg);
          }
        };
    LocationRequest request = new LocationRequest();
    request.setInterval(1000L);
    request.setFastestInterval(500L);
    request.setPriority(LocationRequest.PRIORITY_HIGH_ACCURACY);
    try {
      locationClient.requestLocationUpdates(request, locationCallback, sensorThread.getLooper());
      locationClient
          .getLastLocation()
          .addOnSuccessListener(
              l -> {
                if (l != null && lastLocation == null) {
                  lastLocation = l;
                  String msg = SensorLinkProtocol.gpsJson(toFix(l));
                  for (Connection c : gpsClients) c.send(msg);
                }
              });
    } catch (SecurityException e) {
      Timber.w(e, "Sensor link: no location permission");
      locationCallback = null;
    }
  }

  private void stopGpsLocked() {
    if (locationCallback != null) {
      locationClient.removeLocationUpdates(locationCallback);
      locationCallback = null;
    }
  }

  static SensorLinkProtocol.GpsFix toFix(Location l) {
    SensorLinkProtocol.GpsFix f = new SensorLinkProtocol.GpsFix();
    f.latitude = l.getLatitude();
    f.longitude = l.getLongitude();
    f.altitude = l.getAltitude();
    f.bearing = l.getBearing();
    f.accuracy = l.getAccuracy();
    f.speed = l.getSpeed();
    f.timeMs = l.getTime();
    return f;
  }

  // endregion
}
