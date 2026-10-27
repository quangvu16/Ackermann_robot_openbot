package org.openbot.app.robot.sensorlink;

import android.graphics.Bitmap;
import android.graphics.Matrix;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import java.io.ByteArrayOutputStream;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Carries the phone camera to the browser (MJPEG at {@code /camera.mjpg}) and the browser's camera
 * settings back to whichever screen owns the camera.
 *
 * <p>The screen that has the camera open (Object Tracking and Autopilot always, GPS Mission only
 * while a browser watches) registers as the {@link Owner} and hands every frame to {@link
 * #offerFrame}. Frames are only encoded while someone is watching, and no faster than the chosen
 * frame rate, so the stream costs nothing when nobody looks.
 */
public final class CameraHub implements RemoteSettings.Provider {

  /** The screen that has the camera open. Called on the main thread. */
  public interface Owner {
    boolean isFront();

    void applyFacing(boolean front);

    void applyTorch(boolean on);

    void applyZoom(float ratio);

    float maxZoom();

    boolean hasFlash();
  }

  public static final List<String> QUALITIES = Arrays.asList("low", "medium", "high");
  public static final List<String> FACINGS = Arrays.asList("back", "front");

  private static final CameraHub INSTANCE = new CameraHub();

  public static CameraHub get() {
    return INSTANCE;
  }

  private final Handler main = new Handler(Looper.getMainLooper());
  private final AtomicInteger viewers = new AtomicInteger();
  private final List<Runnable> viewerListeners = new CopyOnWriteArrayList<>();
  private final Object frameLock = new Object();

  private volatile Owner owner;
  private volatile String unavailableReason = "No camera on this screen";

  // Settings (main thread writes, camera thread reads).
  private volatile boolean torch;
  private volatile float zoom = 1f;
  private volatile String quality = "medium";
  private volatile int fps = 10;

  private byte[] latestJpeg; // guarded by frameLock
  private long latestSeq; // guarded by frameLock
  private volatile long lastEncodeMs;

  private CameraHub() {}

  // region Owner side

  public void setOwner(Owner o) {
    owner = o;
    if (o != null) {
      unavailableReason = null;
      if (torch) o.applyTorch(true);
      if (zoom > 1f) o.applyZoom(Math.min(zoom, o.maxZoom()));
    }
  }

  public void clearOwner(Owner o) {
    if (owner == o) {
      owner = null;
      unavailableReason = "No camera on this screen";
    }
  }

  /** Why there is no picture, shown on the page; null when the camera is running. */
  public void setUnavailableReason(String reason) {
    unavailableReason = reason;
  }

  public String getUnavailableReason() {
    return owner == null ? unavailableReason : null;
  }

  /** Re-applies torch and zoom after the owner rebinds its camera (e.g. after switching lens). */
  public void reapply(Owner o) {
    if (o == null) return;
    o.applyTorch(torch);
    o.applyZoom(Math.min(zoom, o.maxZoom()));
  }

  public boolean hasViewers() {
    return viewers.get() > 0;
  }

  /** Told when the first viewer arrives or the last one leaves (main thread). */
  public void addViewerListener(Runnable r) {
    viewerListeners.add(r);
  }

  public void removeViewerListener(Runnable r) {
    viewerListeners.remove(r);
  }

  /**
   * A camera frame, on the camera thread. {@code rgb} may be reused by the caller after this
   * returns; it is copied (scaled and rotated upright) before encoding.
   */
  public void offerFrame(Bitmap rgb, int rotationDegrees) {
    if (viewers.get() == 0 || rgb == null) return;
    long now = SystemClock.elapsedRealtime();
    if (now - lastEncodeMs < 1000L / Math.max(1, fps)) return;
    lastEncodeMs = now;

    int targetLong = "low".equals(quality) ? 320 : "high".equals(quality) ? 1280 : 640;
    int longSide = Math.max(rgb.getWidth(), rgb.getHeight());
    float scale = Math.min(1f, targetLong / (float) longSide);
    Matrix m = new Matrix();
    m.postScale(scale, scale);
    if (rotationDegrees != 0) m.postRotate(rotationDegrees);
    Bitmap upright = Bitmap.createBitmap(rgb, 0, 0, rgb.getWidth(), rgb.getHeight(), m, true);
    ByteArrayOutputStream out = new ByteArrayOutputStream(64 * 1024);
    upright.compress(Bitmap.CompressFormat.JPEG, "high".equals(quality) ? 80 : 70, out);
    if (upright != rgb) upright.recycle();
    synchronized (frameLock) {
      latestJpeg = out.toByteArray();
      latestSeq++;
      frameLock.notifyAll();
    }
  }

  // endregion

  // region Viewer side (server threads)

  public void addViewer() {
    if (viewers.getAndIncrement() == 0) notifyViewers();
  }

  public void removeViewer() {
    if (viewers.decrementAndGet() == 0) {
      notifyViewers();
      synchronized (frameLock) {
        latestJpeg = null; // never hand a stale picture to the next viewer
      }
    }
  }

  private void notifyViewers() {
    main.post(() -> {
      for (Runnable r : viewerListeners) r.run();
    });
  }

  /** A frame newer than {@code afterSeq}, or null after {@code timeoutMs}. seq[0] gets its number. */
  public byte[] awaitFrame(long afterSeq, long timeoutMs, long[] seq) throws InterruptedException {
    long deadline = SystemClock.elapsedRealtime() + timeoutMs;
    synchronized (frameLock) {
      while (latestJpeg == null || latestSeq <= afterSeq) {
        long left = deadline - SystemClock.elapsedRealtime();
        if (left <= 0) return null;
        frameLock.wait(left);
      }
      seq[0] = latestSeq;
      return latestJpeg;
    }
  }

  // endregion

  // region Settings (main thread)

  @Override
  public void addSettings(List<RemoteSettings.Setting> out) {
    Owner o = owner;
    if (o != null) {
      out.add(
          RemoteSettings.Setting.choice(
              "camera.facing", "Camera", "Lens", FACINGS,
              () -> o.isFront() ? "front" : "back",
              v -> o.applyFacing("front".equals(v))));
      if (o.hasFlash()) {
        out.add(
            RemoteSettings.Setting.bool(
                "camera.torch", "Camera", "Flashlight",
                () -> torch,
                v -> {
                  torch = (Boolean) v;
                  o.applyTorch(torch);
                }));
      }
      float max = o.maxZoom();
      if (max > 1.01f) {
        out.add(
            RemoteSettings.Setting.range(
                "camera.zoom", "Camera", "Zoom", 1, max, 0.1,
                () -> zoom,
                v -> {
                  zoom = ((Double) v).floatValue();
                  o.applyZoom(zoom);
                }));
      }
    }
    out.add(
        RemoteSettings.Setting.choice(
            "camera.quality", "Camera", "Stream quality", QUALITIES,
            () -> quality,
            v -> quality = (String) v));
    out.add(
        RemoteSettings.Setting.range(
            "camera.fps", "Camera", "Stream frames per second", 1, 30, 1,
            () -> fps,
            v -> fps = (int) Math.round((Double) v)));
  }

  // endregion
}
