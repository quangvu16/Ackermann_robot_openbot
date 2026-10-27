package org.openbot.app.robot.sensorlink;

import android.os.Handler;
import android.os.Looper;
import java.util.concurrent.TimeUnit;
import okhttp3.OkHttpClient;
import okhttp3.Request;
import okhttp3.Response;
import okhttp3.WebSocket;
import okhttp3.WebSocketListener;
import timber.log.Timber;

/**
 * Reads GPS (and, when asked, the rotation vector for a compass heading) from a SensorServer app on
 * another phone. Reconnects every few seconds until stopped, so the other phone can be switched on
 * after this one. Callbacks arrive on the main thread.
 */
final class SensorLinkClient {

  interface Callback {
    void onClientConnected(boolean connected);

    void onRemoteFix(SensorLinkProtocol.GpsFix fix);

    void onRemoteRotationVector(float[] values);
  }

  private static final long RETRY_MS = 3000L;

  private final String baseUrl;
  private final boolean wantHeading;
  private final Callback callback;
  private final Handler main = new Handler(Looper.getMainLooper());
  private final OkHttpClient http =
      new OkHttpClient.Builder()
          .connectTimeout(5, TimeUnit.SECONDS)
          .pingInterval(10, TimeUnit.SECONDS) // notices a phone that walked out of Wi-Fi range
          .build();

  private volatile boolean running;
  private WebSocket gpsSocket;
  private WebSocket headingSocket;
  private boolean gpsOpen;

  SensorLinkClient(String baseUrl, boolean wantHeading, Callback callback) {
    this.baseUrl = baseUrl;
    this.wantHeading = wantHeading;
    this.callback = callback;
  }

  void start() {
    running = true;
    connectGps();
    if (wantHeading) connectHeading();
  }

  void stop() {
    running = false;
    main.removeCallbacksAndMessages(null);
    if (gpsSocket != null) gpsSocket.close(1000, null);
    if (headingSocket != null) headingSocket.close(1000, null);
    gpsSocket = null;
    headingSocket = null;
    http.dispatcher().executorService().shutdown();
  }

  private void connectGps() {
    if (!running) return;
    gpsSocket =
        http.newWebSocket(
            new Request.Builder().url(httpUrl(baseUrl + "/gps")).build(),
            new WebSocketListener() {
              @Override
              public void onOpen(WebSocket ws, Response response) {
                ws.send(SensorLinkProtocol.GET_LAST_KNOWN_LOCATION);
                main.post(() -> setGpsOpen(true));
              }

              @Override
              public void onMessage(WebSocket ws, String text) {
                SensorLinkProtocol.GpsFix fix = SensorLinkProtocol.parseGps(text);
                if (fix != null) main.post(() -> {
                  if (running) callback.onRemoteFix(fix);
                });
              }

              @Override
              public void onClosed(WebSocket ws, int code, String reason) {
                retryGps();
              }

              @Override
              public void onFailure(WebSocket ws, Throwable t, Response response) {
                Timber.d("Sensor link GPS: %s", t.getMessage());
                retryGps();
              }
            });
  }

  private void retryGps() {
    main.post(() -> setGpsOpen(false));
    main.postDelayed(this::connectGps, RETRY_MS);
  }

  private void setGpsOpen(boolean open) {
    if (!running || gpsOpen == open) return;
    gpsOpen = open;
    callback.onClientConnected(open);
  }

  private void connectHeading() {
    if (!running) return;
    headingSocket =
        http.newWebSocket(
            new Request.Builder()
                .url(httpUrl(baseUrl + SensorLinkProtocol.sensorPath(SensorLinkProtocol.TYPE_ROTATION_VECTOR)))
                .build(),
            new WebSocketListener() {
              @Override
              public void onMessage(WebSocket ws, String text) {
                SensorLinkProtocol.SensorReading r = SensorLinkProtocol.parseSensor(text);
                if (r != null && r.values.length >= 3) main.post(() -> {
                  if (running) callback.onRemoteRotationVector(r.values);
                });
              }

              @Override
              public void onClosed(WebSocket ws, int code, String reason) {
                main.postDelayed(SensorLinkClient.this::connectHeading, RETRY_MS);
              }

              @Override
              public void onFailure(WebSocket ws, Throwable t, Response response) {
                main.postDelayed(SensorLinkClient.this::connectHeading, RETRY_MS);
              }
            });
  }

  /** OkHttp takes ws:// URLs but validates them as http://. */
  private static String httpUrl(String wsUrl) {
    return wsUrl.startsWith("ws://") ? "http://" + wsUrl.substring(5) : wsUrl;
  }
}
