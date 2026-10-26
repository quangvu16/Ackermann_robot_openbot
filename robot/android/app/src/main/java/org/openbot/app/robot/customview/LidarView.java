package org.openbot.app.robot.customview;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.util.AttributeSet;
import android.view.View;
import org.openbot.app.robot.vehicle.LidarScan;

/**
 * A north-up polar plot of one LiDAR revolution, the way every ground station draws a 2D scanner.
 *
 * <p>The car sits at the centre pointing up, so "forward" is up on screen no matter which way the
 * phone is held. Returns are drawn as a connected outline at their measured distance - the room's
 * walls read as the shape they actually are. Bins with no return are simply not drawn, which is
 * what "nothing measured here" means; the {@code +inf} that a ROS {@code /scan} publisher would
 * emit for those bins has no place on screen.
 *
 * <p>The mounting offset is applied here at draw time ({@link #setForwardOffsetDeg}) rather than by
 * reordering bins, matching the firmware's {@code LIDAR_FORWARD_DEG} rule: the ranges array is
 * never reordered on either side, so a single correction lands in one place instead of two
 * fighting.
 *
 * <p>Range rings and the {@code AVOID_MARGIN} ring are in metres; anything past {@link
 * #getRangeMaxMeters()} is clamped to the outer circle so a far return still shows its direction.
 */
public class LidarView extends View {

  private LidarScan scan;
  private float forwardOffsetDeg = 0f;
  private float rangeMaxMeters = 8.0f;
  private float marginMeters = 1.0f;
  private boolean showMargin = true;
  private boolean stale = false;
  private int scanColor = Color.parseColor("#26C6DA");
  private int staleColor = Color.parseColor("#9E9E9E");

  private final Paint ringPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
  private final Paint marginPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
  private final Paint scanPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
  private final Paint robotPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
  private final Paint labelPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
  private final Path scanPath = new Path();

  public LidarView(Context context) {
    super(context);
    init();
  }

  public LidarView(Context context, AttributeSet attrs) {
    super(context, attrs);
    init();
  }

  public LidarView(Context context, AttributeSet attrs, int defStyleAttr) {
    super(context, attrs, defStyleAttr);
    init();
  }

  private void init() {
    ringPaint.setStyle(Paint.Style.STROKE);
    ringPaint.setStrokeWidth(1.5f);
    ringPaint.setColor(Color.parseColor("#B0BEC5"));

    marginPaint.setStyle(Paint.Style.STROKE);
    marginPaint.setStrokeWidth(3f);
    marginPaint.setColor(Color.parseColor("#FFB300"));

    scanPaint.setStyle(Paint.Style.STROKE);
    scanPaint.setStrokeWidth(3f);
    scanPaint.setStrokeJoin(Paint.Join.ROUND);
    scanPaint.setStrokeCap(Paint.Cap.ROUND);
    scanPaint.setColor(Color.parseColor("#26C6DA"));

    robotPaint.setStyle(Paint.Style.FILL);
    robotPaint.setColor(Color.parseColor("#4B7BFF"));

    labelPaint.setColor(Color.parseColor("#78909C"));
    labelPaint.setTextSize(24f);
  }

  /** The revolution to draw. Null clears the view to rings only. */
  public void setScan(LidarScan scan) {
    this.scan = scan;
    invalidate();
  }

  /**
   * Where the car's nose sits in the LiDAR's frame, in degrees, matching the firmware's {@code
   * LIDAR_FORWARD_DEG}. Positive rotates the plot clockwise, same as the firmware's sector folding.
   */
  public void setForwardOffsetDeg(float forwardOffsetDeg) {
    this.forwardOffsetDeg = forwardOffsetDeg;
    invalidate();
  }

  /** Outer range ring in metres. Returns further out are clamped to it and still show direction. */
  public void setRangeMaxMeters(float rangeMaxMeters) {
    this.rangeMaxMeters = Math.max(0.1f, rangeMaxMeters);
    invalidate();
  }

  /** Draw the object-avoidance standoff ring at this distance, in metres. */
  public void setMarginMeters(float marginMeters) {
    this.marginMeters = marginMeters;
    invalidate();
  }

  /** Show or hide the {@code AVOID_MARGIN} ring. */
  public void setShowMargin(boolean showMargin) {
    this.showMargin = showMargin;
    invalidate();
  }

  /** Colour of a live scan. */
  public void setScanColor(int color) {
    this.scanColor = color;
    invalidate();
  }

  /** Used when the scan has gone stale, so a frozen picture reads as frozen. */
  public void setStaleColor(int color) {
    this.staleColor = color;
    invalidate();
  }

  /**
   * Mark the picture as frozen. Call this from the scanner's staleness check - a stale scan is
   * drawn in {@link #setStaleColor} rather than left looking live, because a room that stopped
   * moving on screen is the one thing a LiDAR view must not imply is still true.
   */
  public void setStale(boolean stale) {
    this.stale = stale;
    invalidate();
  }

  public LidarScan getScan() {
    return scan;
  }

  @Override
  protected void onDraw(Canvas canvas) {
    super.onDraw(canvas);
    float cx = getWidth() / 2f;
    float cy = getHeight() / 2f;
    float radius = Math.min(cx, cy) * 0.92f;
    if (radius <= 1f) {
      return;
    }

    // Range rings at half and full scale, plus the standoff ring when it is in view.
    canvas.drawCircle(cx, cy, radius / 2f, ringPaint);
    canvas.drawCircle(cx, cy, radius, ringPaint);
    canvas.drawCircle(cx, cy, radius / 4f, ringPaint);

    if (showMargin && marginMeters > 0f && marginMeters < rangeMaxMeters) {
      canvas.drawCircle(cx, cy, radius * (marginMeters / rangeMaxMeters), marginPaint);
    }

    // Nose and tail marks, so the plot reads as a car rather than a compass.
    canvas.drawLine(cx, cy - radius, cx, cy - radius * 0.88f, robotPaint);
    canvas.drawLine(cx, cy + radius, cx, cy + radius * 0.88f, ringPaint);

    drawScan(canvas, cx, cy, radius);
    drawRobot(canvas, cx, cy, radius);
    canvas.drawText(
        String.format(java.util.Locale.US, "%.0f m", rangeMaxMeters),
        cx + 6f,
        cy - radius + 30f,
        labelPaint);
  }

  private void drawScan(Canvas canvas, float cx, float cy, float radius) {
    if (scan == null) {
      return;
    }
    scanPaint.setColor(stale ? staleColor : scanColor);

    scanPath.reset();
    boolean penDown = false;
    int bins = scan.getBinCount();
    // Walk bins in wire order and lift the pen across runs of no-return, so an unmeasured arc is a
    // gap rather than a line dragged through the middle of the plot.
    for (int bin = 0; bin < bins; bin++) {
      int mm = scan.getRangeMm(bin);
      if (mm == 0) {
        penDown = false;
        continue;
      }
      float dist = mm / 1000f;
      float r = radius * Math.min(1f, dist / rangeMaxMeters);
      float angDeg = scan.getVehicleAngleDeg(bin, forwardOffsetDeg) - 90f;
      double angRad = Math.toRadians(angDeg);
      float x = cx + (float) (r * Math.cos(angRad));
      float y = cy + (float) (r * Math.sin(angRad));
      if (!penDown) {
        scanPath.moveTo(x, y);
        penDown = true;
      } else {
        scanPath.lineTo(x, y);
      }
    }
    canvas.drawPath(scanPath, scanPaint);
  }

  private void drawRobot(Canvas canvas, float cx, float cy, float radius) {
    float size = radius * 0.12f;
    Path p = new Path();
    p.moveTo(cx, cy - size);
    p.lineTo(cx + size * 0.7f, cy + size * 0.8f);
    p.lineTo(cx - size * 0.7f, cy + size * 0.8f);
    p.close();
    canvas.drawPath(p, robotPaint);
  }
}
