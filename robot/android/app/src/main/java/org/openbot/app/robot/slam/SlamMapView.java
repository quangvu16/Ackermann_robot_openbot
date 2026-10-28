package org.openbot.app.robot.slam;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.Path;
import android.os.SystemClock;
import android.util.AttributeSet;
import android.view.GestureDetector;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;
import android.view.View;
import java.util.List;

/**
 * An RViz-style view of the SLAM map: the occupancy grid (white free, black walls, grey unknown, as
 * RViz and map_server draw it), 1 m grid lines, the robot as an arrow, the live LiDAR scan, the
 * trail driven, the planned path and the goal. Each layer can be switched off, like RViz displays.
 *
 * <p>Drag to pan, pinch to zoom, double-tap to centre on the car (it then follows the car again),
 * tap to set a goal, long-press to tell the SLAM where the car is.
 */
public class SlamMapView extends View {

  public interface Listener {
    void onMapTap(double x, double y);

    void onMapLongPress(double x, double y);
  }

  // Layers
  public boolean showMap = true;
  public boolean showScan = true;
  public boolean showPath = true;
  public boolean showTrail = true;

  private OccupancyGrid grid;
  private int drawnVersion = -1;
  private long lastBitmapMs;
  private Bitmap bitmap;
  private int[] pixels;
  private Pose pose = Pose.ZERO;
  private double[] scanX = new double[0]; // world coordinates
  private double[] scanY = new double[0];
  private List<double[]> trail;
  private List<double[]> path;
  private double[] goal;
  private Listener listener;

  // View transform: screen = centre + (world - focus) * scale, y flipped.
  private float scale = 60f; // px per metre
  private double focusX;
  private double focusY;
  private boolean follow = true;

  private final Paint gridPaint = new Paint();
  private final Paint linePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
  private final Paint fillPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
  private final Paint bitmapPaint = new Paint();
  private final Matrix matrix = new Matrix();
  private final Path shape = new Path();
  private final ScaleGestureDetector scaler;
  private final GestureDetector gestures;

  public SlamMapView(Context c) {
    this(c, null);
  }

  public SlamMapView(Context c, AttributeSet a) {
    super(c, a);
    gridPaint.setColor(Color.argb(40, 0, 0, 0));
    gridPaint.setStrokeWidth(1f);
    linePaint.setStyle(Paint.Style.STROKE);
    linePaint.setStrokeCap(Paint.Cap.ROUND);
    linePaint.setStrokeJoin(Paint.Join.ROUND);
    bitmapPaint.setFilterBitmap(false); // crisp cells, like RViz
    setContentDescription("SLAM map");
    scaler =
        new ScaleGestureDetector(
            c,
            new ScaleGestureDetector.SimpleOnScaleGestureListener() {
              @Override
              public boolean onScale(ScaleGestureDetector d) {
                scale = Math.max(8f, Math.min(400f, scale * d.getScaleFactor()));
                invalidate();
                return true;
              }
            });
    gestures =
        new GestureDetector(
            c,
            new GestureDetector.SimpleOnGestureListener() {
              @Override
              public boolean onDown(MotionEvent e) {
                return true;
              }

              @Override
              public boolean onScroll(MotionEvent e1, MotionEvent e2, float dx, float dy) {
                follow = false;
                focusX += dx / scale;
                focusY -= dy / scale;
                invalidate();
                return true;
              }

              @Override
              public boolean onSingleTapConfirmed(MotionEvent e) {
                if (listener != null) listener.onMapTap(toWorldX(e.getX()), toWorldY(e.getY()));
                return true;
              }

              @Override
              public void onLongPress(MotionEvent e) {
                if (listener != null) listener.onMapLongPress(toWorldX(e.getX()), toWorldY(e.getY()));
              }

              @Override
              public boolean onDoubleTap(MotionEvent e) {
                follow = true;
                invalidate();
                return true;
              }
            });
  }

  public void setListener(Listener l) {
    listener = l;
  }

  public void setGrid(OccupancyGrid g) {
    if (g != grid) {
      grid = g;
      drawnVersion = -1;
      bitmap = null;
    }
    invalidate();
  }

  /** The newest state; call at the screen's update rate. */
  public void setState(Pose pose, double[] scanAngles, double[] scanRanges, List<double[]> trail,
      List<double[]> path, double[] goal) {
    this.pose = pose;
    this.trail = trail;
    this.path = path;
    this.goal = goal;
    if (scanAngles != null) {
      int n = scanAngles.length;
      if (scanX.length != n) {
        scanX = new double[n];
        scanY = new double[n];
      }
      for (int i = 0; i < n; i++) {
        double a = pose.theta + scanAngles[i];
        scanX[i] = pose.x + scanRanges[i] * Math.cos(a);
        scanY[i] = pose.y + scanRanges[i] * Math.sin(a);
      }
    } else {
      scanX = new double[0];
      scanY = new double[0];
    }
    invalidate();
  }

  public void recentre() {
    follow = true;
    invalidate();
  }

  /** For subclasses that draw on the map (Rooms & Zones): consume a touch to edit instead of pan. */
  protected boolean onEditTouch(MotionEvent e) {
    return false;
  }

  /** Drawn over the map and grid lines, under the trail, path, scan and car. */
  protected void drawOverlay(Canvas c, float dp) {}

  protected float getScale() {
    return scale;
  }

  protected OccupancyGrid getGrid() {
    return grid;
  }

  @Override
  public boolean onTouchEvent(MotionEvent e) {
    if (e.getPointerCount() == 1 && onEditTouch(e)) return true;
    scaler.onTouchEvent(e);
    if (!scaler.isInProgress()) gestures.onTouchEvent(e);
    return true;
  }

  protected float sx(double x) {
    return (float) (getWidth() / 2.0 + (x - focusX) * scale);
  }

  protected float sy(double y) {
    return (float) (getHeight() / 2.0 - (y - focusY) * scale);
  }

  protected double toWorldX(float px) {
    return focusX + (px - getWidth() / 2.0) / scale;
  }

  protected double toWorldY(float py) {
    return focusY - (py - getHeight() / 2.0) / scale;
  }

  /** Rebuild the map picture at most twice a second (600 x 600 cells is cheap, not free). */
  private void refreshBitmap() {
    if (grid == null) return;
    long now = SystemClock.uptimeMillis();
    if (bitmap != null && (grid.getVersion() == drawnVersion || now - lastBitmapMs < 500)) return;
    lastBitmapMs = now;
    drawnVersion = grid.getVersion();
    int w = grid.width;
    int h = grid.height;
    if (bitmap == null || bitmap.getWidth() != w || bitmap.getHeight() != h) {
      bitmap = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888);
      pixels = new int[w * h];
    }
    for (int y = 0; y < h; y++) {
      int row = (h - 1 - y) * w; // bitmap rows run top-down, the grid bottom-up
      for (int x = 0; x < w; x++) {
        float v = grid.logOdds[y * w + x];
        int c;
        if (v > OccupancyGrid.OCC_THRESHOLD) c = 0xFF101010;
        else if (v < OccupancyGrid.FREE_THRESHOLD) c = 0xFFFFFFFF;
        else if (v == 0f) c = 0xFFB4B4B4;
        else { // being learnt: shade between free and wall
          int g = (int) (180 - v / OccupancyGrid.L_MAX * 150);
          g = Math.max(20, Math.min(250, g));
          c = 0xFF000000 | (g << 16) | (g << 8) | g;
        }
        pixels[row + x] = c;
      }
    }
    bitmap.setPixels(pixels, 0, w, 0, 0, w, h);
  }

  @Override
  protected void onDraw(Canvas c) {
    super.onDraw(c);
    if (follow) {
      focusX = pose.x;
      focusY = pose.y;
    }
    c.drawColor(0xFFB4B4B4);

    if (showMap && grid != null) {
      refreshBitmap();
      if (bitmap != null) {
        float cell = (float) (grid.resolution * scale);
        matrix.reset();
        matrix.postScale(cell, cell);
        matrix.postTranslate(sx(grid.originX), sy(grid.originY + grid.height * grid.resolution));
        c.drawBitmap(bitmap, matrix, bitmapPaint);
      }
    }

    // 1 m grid lines across the visible area.
    double x0 = Math.floor(toWorldX(0));
    double x1 = Math.ceil(toWorldX(getWidth()));
    double y0 = Math.floor(toWorldY(getHeight()));
    double y1 = Math.ceil(toWorldY(0));
    if (scale > 15) {
      for (double x = x0; x <= x1; x++) c.drawLine(sx(x), 0, sx(x), getHeight(), gridPaint);
      for (double y = y0; y <= y1; y++) c.drawLine(0, sy(y), getWidth(), sy(y), gridPaint);
    }

    float dp = getResources().getDisplayMetrics().density;
    drawOverlay(c, dp);
    if (showTrail && trail != null && trail.size() > 1) {
      linePaint.setColor(0xFFFF9800);
      linePaint.setStrokeWidth(2 * dp);
      shape.reset();
      shape.moveTo(sx(trail.get(0)[0]), sy(trail.get(0)[1]));
      for (int i = 1; i < trail.size(); i++) shape.lineTo(sx(trail.get(i)[0]), sy(trail.get(i)[1]));
      c.drawPath(shape, linePaint);
    }
    if (showPath && path != null && path.size() > 1) {
      linePaint.setColor(0xFF2E7D32);
      linePaint.setStrokeWidth(3 * dp);
      shape.reset();
      shape.moveTo(sx(path.get(0)[0]), sy(path.get(0)[1]));
      for (int i = 1; i < path.size(); i++) shape.lineTo(sx(path.get(i)[0]), sy(path.get(i)[1]));
      c.drawPath(shape, linePaint);
    }
    if (goal != null) {
      fillPaint.setColor(0xFFD81B60);
      c.drawCircle(sx(goal[0]), sy(goal[1]), 7 * dp, fillPaint);
      linePaint.setColor(Color.WHITE);
      linePaint.setStrokeWidth(2 * dp);
      c.drawCircle(sx(goal[0]), sy(goal[1]), 7 * dp, linePaint);
    }
    if (showScan) {
      fillPaint.setColor(0xFFE53935);
      float r = Math.max(1.5f, 2 * dp);
      for (int i = 0; i < scanX.length; i++) c.drawCircle(sx(scanX[i]), sy(scanY[i]), r, fillPaint);
    }
    // The car: an arrow pointing along its heading.
    float px = sx(pose.x);
    float py = sy(pose.y);
    float len = Math.max(14 * dp, (float) (0.35 * scale));
    double th = pose.theta;
    shape.reset();
    shape.moveTo(px + (float) (Math.cos(th) * len), py - (float) (Math.sin(th) * len));
    shape.lineTo(px + (float) (Math.cos(th + 2.5) * len * 0.6), py - (float) (Math.sin(th + 2.5) * len * 0.6));
    shape.lineTo(px + (float) (Math.cos(th - 2.5) * len * 0.6), py - (float) (Math.sin(th - 2.5) * len * 0.6));
    shape.close();
    fillPaint.setColor(0xFF1E88E5);
    c.drawPath(shape, fillPaint);
    linePaint.setColor(Color.WHITE);
    linePaint.setStrokeWidth(1.5f * dp);
    c.drawPath(shape, linePaint);
  }
}
