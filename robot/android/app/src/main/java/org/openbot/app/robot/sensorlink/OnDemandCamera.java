package org.openbot.app.robot.sensorlink;

import android.Manifest;
import android.annotation.SuppressLint;
import android.content.Context;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.util.Size;
import androidx.camera.core.Camera;
import androidx.camera.core.CameraSelector;
import androidx.camera.core.ImageAnalysis;
import androidx.camera.core.ImageProxy;
import androidx.camera.lifecycle.ProcessCameraProvider;
import androidx.core.content.ContextCompat;
import androidx.lifecycle.LifecycleOwner;
import com.google.common.util.concurrent.ListenableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import org.openbot.app.robot.utils.YuvToRgbConverter;
import timber.log.Timber;

/**
 * A camera for screens that do not show one (GPS Mission): it opens only while a browser is
 * watching {@code /camera.mjpg} and closes again when the last viewer leaves. No preview on the
 * phone, just frames for {@link CameraHub}.
 */
public final class OnDemandCamera implements CameraHub.Owner {

  private final Context context;
  private final LifecycleOwner lifecycleOwner;
  private final Runnable onViewersChanged = this::update;
  private ExecutorService executor;
  private ProcessCameraProvider provider;
  private Camera camera;
  private YuvToRgbConverter converter;
  private Bitmap buffer;
  private boolean front;
  private boolean active;
  private boolean started;

  public OnDemandCamera(Context context, LifecycleOwner lifecycleOwner) {
    this.context = context.getApplicationContext();
    this.lifecycleOwner = lifecycleOwner;
  }

  /** Call from onResume. */
  public void start() {
    if (started) return;
    started = true;
    CameraHub.get().setOwner(this);
    CameraHub.get().addViewerListener(onViewersChanged);
    update();
  }

  /** Call from onPause. */
  public void stop() {
    if (!started) return;
    started = false;
    CameraHub.get().removeViewerListener(onViewersChanged);
    CameraHub.get().clearOwner(this);
    close();
  }

  private void update() {
    if (!started) return;
    if (CameraHub.get().hasViewers()) open();
    else close();
  }

  private void open() {
    if (active) return;
    if (ContextCompat.checkSelfPermission(context, Manifest.permission.CAMERA)
        != PackageManager.PERMISSION_GRANTED) {
      CameraHub.get().clearOwner(this);
      CameraHub.get()
          .setUnavailableReason("Camera permission not granted: open Object Tracking once and allow the camera");
      return;
    }
    active = true;
    executor = Executors.newSingleThreadExecutor();
    converter = new YuvToRgbConverter(context);
    ListenableFuture<ProcessCameraProvider> future = ProcessCameraProvider.getInstance(context);
    future.addListener(
        () -> {
          try {
            provider = future.get();
            bind();
          } catch (Exception e) {
            Timber.e(e, "On-demand camera failed");
            CameraHub.get().setUnavailableReason("Camera failed: " + e.getMessage());
          }
        },
        ContextCompat.getMainExecutor(context));
  }

  @SuppressLint({"UnsafeExperimentalUsageError", "UnsafeOptInUsageError"})
  private void bind() {
    if (!active || provider == null) return;
    provider.unbindAll();
    ImageAnalysis analysis =
        new ImageAnalysis.Builder()
            .setTargetResolution(new Size(1280, 720))
            .setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST)
            .build();
    analysis.setAnalyzer(executor, this::analyze);
    CameraSelector selector =
        new CameraSelector.Builder()
            .requireLensFacing(front ? CameraSelector.LENS_FACING_FRONT : CameraSelector.LENS_FACING_BACK)
            .build();
    try {
      camera = provider.bindToLifecycle(lifecycleOwner, selector, analysis);
      CameraHub.get().setOwner(this);
      CameraHub.get().reapply(this);
    } catch (Exception e) {
      Timber.e(e, "On-demand camera bind failed");
      camera = null;
      CameraHub.get().setUnavailableReason("Camera busy or missing: " + e.getMessage());
    }
  }

  @SuppressLint({"UnsafeExperimentalUsageError", "UnsafeOptInUsageError"})
  private void analyze(ImageProxy image) {
    try {
      if (buffer == null || buffer.getWidth() != image.getWidth() || buffer.getHeight() != image.getHeight()) {
        buffer = Bitmap.createBitmap(image.getWidth(), image.getHeight(), Bitmap.Config.ARGB_8888);
      }
      if (image.getImage() != null) {
        converter.yuvToRgb(image.getImage(), buffer);
        CameraHub.get().offerFrame(buffer, image.getImageInfo().getRotationDegrees());
      }
    } finally {
      image.close();
    }
  }

  private void close() {
    if (!active) return;
    active = false;
    if (provider != null) provider.unbindAll();
    camera = null;
    if (executor != null) executor.shutdown();
    executor = null;
  }

  // region CameraHub.Owner

  @Override
  public boolean isFront() {
    return front;
  }

  @Override
  public void applyFacing(boolean front) {
    if (this.front == front) return;
    this.front = front;
    if (active) bind();
  }

  @Override
  public void applyTorch(boolean on) {
    if (camera != null && camera.getCameraInfo().hasFlashUnit()) {
      camera.getCameraControl().enableTorch(on);
    }
  }

  @Override
  public void applyZoom(float ratio) {
    if (camera != null) camera.getCameraControl().setZoomRatio(ratio);
  }

  @Override
  public float maxZoom() {
    if (camera == null || camera.getCameraInfo().getZoomState().getValue() == null) return 1f;
    return camera.getCameraInfo().getZoomState().getValue().getMaxZoomRatio();
  }

  @Override
  public boolean hasFlash() {
    return camera == null || camera.getCameraInfo().hasFlashUnit();
  }

  // endregion
}
