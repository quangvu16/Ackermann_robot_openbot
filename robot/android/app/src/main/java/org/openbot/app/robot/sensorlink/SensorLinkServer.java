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
import java.net.InetSocketAddress;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CopyOnWriteArraySet;
import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;
import timber.log.Timber;

/**
 * A SensorServer-compatible WebSocket server: any SensorServer client (Python scripts, the
 * SensorServer web page, Unity, ROS bridges...) can read this phone's sensors and GPS while the car
 * is driving. See {@link SensorLinkProtocol} for the API.
 *
 * <p>Sensors are only switched on while someone is connected to them, and off again when the last
 * client leaves, so an idle server costs no battery.
 */
final class SensorLinkServer extends WebSocketServer {

  interface Callback {
    void onServerStarted();

    void onServerFailed(String reason);

    void onClientCountChanged(int count);
  }

  // Close codes in the private-use range (4000-4999).
  private static final int CLOSE_UNKNOWN_PATH = 4000;
  private static final int CLOSE_NO_SENSOR = 4001;
  private static final int CLOSE_NO_PERMISSION = 4002;

  private final Context context;
  private final Callback callback;
  private final SensorManager sensorManager;
  private final FusedLocationProviderClient locationClient;
  private final HandlerThread sensorThread = new HandlerThread("SensorLinkServer");
  private final Handler sensorHandler;

  /** Which sensors each connection asked for; {@code multi} adds the type to each message. */
  private static final class Subscription {
    final boolean multi;
    final boolean gps;

    Subscription(boolean multi, boolean gps) {
      this.multi = multi;
      this.gps = gps;
    }
  }

  private final Map<WebSocket, Subscription> subscriptions = new ConcurrentHashMap<>();
  private final Map<String, TypeListener> listeners = new HashMap<>(); // guarded by this
  private final Set<WebSocket> gpsClients = new CopyOnWriteArraySet<>();
  private LocationCallback locationCallback; // guarded by this
  private volatile Location lastLocation;

  SensorLinkServer(Context context, int port, Callback callback) {
    super(new InetSocketAddress(port));
    this.context = context.getApplicationContext();
    this.callback = callback;
    this.sensorManager = (SensorManager) this.context.getSystemService(Context.SENSOR_SERVICE);
    this.locationClient = LocationServices.getFusedLocationProviderClient(this.context);
    sensorThread.start();
    sensorHandler = new Handler(sensorThread.getLooper());
    setReuseAddr(true);
    setConnectionLostTimeout(15);
  }

  /** Stops the server and every sensor it switched on. Safe to call more than once. */
  void shutdown() {
    try {
      stop(500);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }
    synchronized (this) {
      for (TypeListener l : listeners.values()) sensorManager.unregisterListener(l);
      listeners.clear();
      stopGpsLocked();
    }
    subscriptions.clear();
    gpsClients.clear();
    sensorThread.quitSafely();
  }

  @Override
  public void onStart() {
    Timber.i("Sensor link server listening on %s", getAddress());
    callback.onServerStarted();
  }

  @Override
  public void onOpen(WebSocket conn, ClientHandshake handshake) {
    SensorLinkProtocol.Request request =
        SensorLinkProtocol.parse(handshake.getResourceDescriptor());
    switch (request.kind) {
      case GPS:
        if (ContextCompat.checkSelfPermission(context, Manifest.permission.ACCESS_FINE_LOCATION)
            != PackageManager.PERMISSION_GRANTED) {
          conn.close(CLOSE_NO_PERMISSION, "Location permission not granted on the phone");
          return;
        }
        subscriptions.put(conn, new Subscription(false, true));
        gpsClients.add(conn);
        synchronized (this) {
          startGpsLocked();
        }
        break;
      case SENSOR:
      case SENSORS:
        for (String type : request.types) {
          if (findSensor(type) == null) {
            conn.close(CLOSE_NO_SENSOR, "Sensor not available: " + type);
            return;
          }
        }
        subscriptions.put(conn, new Subscription(request.kind == SensorLinkProtocol.Kind.SENSORS, false));
        synchronized (this) {
          for (String type : request.types) subscribeLocked(type, conn);
        }
        break;
      default:
        conn.close(CLOSE_UNKNOWN_PATH, "Unknown path " + handshake.getResourceDescriptor());
        return;
    }
    callback.onClientCountChanged(subscriptions.size());
  }

  @Override
  public void onClose(WebSocket conn, int code, String reason, boolean remote) {
    Subscription sub = subscriptions.remove(conn);
    if (sub == null) return;
    synchronized (this) {
      if (sub.gps) {
        gpsClients.remove(conn);
        if (gpsClients.isEmpty()) stopGpsLocked();
      }
      for (Map.Entry<String, TypeListener> e : new HashMap<>(listeners).entrySet()) {
        TypeListener l = e.getValue();
        l.clients.remove(conn);
        if (l.clients.isEmpty()) {
          sensorManager.unregisterListener(l);
          listeners.remove(e.getKey());
        }
      }
    }
    callback.onClientCountChanged(subscriptions.size());
  }

  @Override
  public void onMessage(WebSocket conn, String message) {
    Subscription sub = subscriptions.get(conn);
    if (sub != null && sub.gps && SensorLinkProtocol.GET_LAST_KNOWN_LOCATION.equals(message.trim())) {
      Location l = lastLocation;
      if (l != null) conn.send(SensorLinkProtocol.gpsJson(toFix(l)));
    }
  }

  @Override
  public void onError(WebSocket conn, Exception ex) {
    if (conn == null) {
      // The server itself failed, most often "address already in use".
      Timber.e(ex, "Sensor link server failed");
      callback.onServerFailed(ex.getMessage() == null ? ex.toString() : ex.getMessage());
    } else {
      Timber.w(ex, "Sensor link client error");
    }
  }

  // region Sensors

  private Sensor findSensor(String type) {
    if (sensorManager == null) return null;
    List<Sensor> all = sensorManager.getSensorList(Sensor.TYPE_ALL);
    for (Sensor s : all) {
      if (type.equals(s.getStringType())) return s;
    }
    return null;
  }

  private void subscribeLocked(String type, WebSocket conn) {
    TypeListener l = listeners.get(type);
    if (l == null) {
      Sensor sensor = findSensor(type);
      if (sensor == null) return;
      l = new TypeListener(type);
      listeners.put(type, l);
      sensorManager.registerListener(l, sensor, SensorManager.SENSOR_DELAY_GAME, sensorHandler);
    }
    l.clients.add(conn);
  }

  /** One registered sensor, fanned out to every client that asked for it. */
  private final class TypeListener implements SensorEventListener {
    final String type;
    final Set<WebSocket> clients = new CopyOnWriteArraySet<>();

    TypeListener(String type) {
      this.type = type;
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
      String single = null;
      String multi = null;
      for (WebSocket c : clients) {
        if (!c.isOpen()) continue;
        Subscription sub = subscriptions.get(c);
        boolean withType = sub != null && sub.multi;
        String msg;
        if (withType) {
          if (multi == null)
            multi = SensorLinkProtocol.sensorJson(type, event.values, event.timestamp, event.accuracy);
          msg = multi;
        } else {
          if (single == null)
            single = SensorLinkProtocol.sensorJson(null, event.values, event.timestamp, event.accuracy);
          msg = single;
        }
        try {
          c.send(msg);
        } catch (RuntimeException e) {
          Timber.d(e, "Sensor link send failed");
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
            for (WebSocket c : gpsClients) {
              if (c.isOpen()) c.send(msg);
            }
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
                if (l != null && lastLocation == null) lastLocation = l;
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
