package org.openbot.app.robot.lidarbot;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.RectF;
import android.os.SystemClock;
import android.util.AttributeSet;
import android.view.View;

/**
 * A robot face for the LiDAR screen: two glowing eyes and a mouth on a dark screen, like a desk
 * robot. The mood comes from the running behaviour ({@link LidarBehaviors.Mood}) and the eyes look
 * towards whatever the behaviour is looking at ({@code gazeDeg}, + left), so the face shows what
 * the LiDAR sees: a smile when the way is clear, hearts while following, wide red eyes at the
 * warning distance. Drawn with plain Canvas calls, so it needs no images and scales to any size.
 *
 * <p>Mirrors RobotFaceView in the iOS app (LidarBehaviorsController.swift).
 */
public class RobotFaceView extends View {

  private LidarBehaviors.Mood mood = LidarBehaviors.Mood.IDLE;
  private float gazeDeg;
  private float shownGaze; // eased towards gazeDeg
  private String caption = "";
  private long nextBlinkMs = SystemClock.uptimeMillis() + 2500;

  private final Paint eye = new Paint(Paint.ANTI_ALIAS_FLAG);
  private final Paint line = new Paint(Paint.ANTI_ALIAS_FLAG);
  private final Paint text = new Paint(Paint.ANTI_ALIAS_FLAG);
  private final RectF r = new RectF();
  private final Path path = new Path();

  public RobotFaceView(Context c) {
    this(c, null);
  }

  public RobotFaceView(Context c, AttributeSet a) {
    super(c, a);
    eye.setStyle(Paint.Style.FILL);
    line.setStyle(Paint.Style.STROKE);
    line.setStrokeCap(Paint.Cap.ROUND);
    line.setStrokeJoin(Paint.Join.ROUND);
    text.setColor(Color.parseColor("#B0BEC5"));
    text.setTextAlign(Paint.Align.CENTER);
    setContentDescription("Robot face");
  }

  public void setMood(LidarBehaviors.Mood mood, float gazeDeg, String caption) {
    this.mood = mood == null ? LidarBehaviors.Mood.IDLE : mood;
    this.gazeDeg = Math.max(-90f, Math.min(90f, gazeDeg));
    this.caption = caption == null ? "" : caption;
    setContentDescription(this.caption);
    invalidate();
  }

  /** The eye colour for each mood; also used to tint the screen's status line. */
  public static int moodColor(LidarBehaviors.Mood mood) {
    switch (mood) {
      case HAPPY:
        return Color.parseColor("#4DD0E1");
      case LOVE:
        return Color.parseColor("#F06292");
      case FOCUSED:
        return Color.parseColor("#64B5F6");
      case SEARCHING:
        return Color.parseColor("#9575CD");
      case WORRIED:
        return Color.parseColor("#FFB74D");
      case SCARED:
        return Color.parseColor("#EF5350");
      case BLIND:
        return Color.parseColor("#90A4AE");
      case IDLE:
      default:
        return Color.parseColor("#78909C");
    }
  }

  @Override
  protected void onDraw(Canvas c) {
    super.onDraw(c);
    float w = getWidth();
    float h = getHeight();
    long now = SystemClock.uptimeMillis();
    c.drawColor(Color.parseColor("#0E151D"));

    int color = moodColor(mood);
    eye.setColor(color);
    line.setColor(color);
    float unit = Math.min(w, h * 1.4f);
    line.setStrokeWidth(unit * 0.035f);

    // Searching sweeps the eyes side to side; scared trembles.
    float target = gazeDeg;
    if (mood == LidarBehaviors.Mood.SEARCHING) target = (float) (60 * Math.sin(now / 600.0));
    shownGaze += (target - shownGaze) * 0.25f;
    float shake = mood == LidarBehaviors.Mood.SCARED ? (float) (unit * 0.012 * Math.sin(now / 25.0)) : 0f;
    // + gaze (left) moves the eyes to the left of the screen.
    float dx = -shownGaze / 90f * w * 0.16f + shake;

    float cx = w / 2f + dx;
    float cy = h * 0.44f;
    float eyeW = unit * 0.2f;
    float eyeH = unit * 0.26f;
    float gap = unit * 0.17f;

    // Blink: eyes close for 140 ms every few seconds (not when scared or blind).
    float open = 1f;
    if (now >= nextBlinkMs) {
      float t = (now - nextBlinkMs) / 140f;
      if (t >= 1f) nextBlinkMs = now + 2500 + (long) (Math.random() * 2500);
      else open = Math.abs(1f - 2f * t);
    }
    if (mood == LidarBehaviors.Mood.SCARED || mood == LidarBehaviors.Mood.BLIND) open = 1f;

    for (int side = -1; side <= 1; side += 2) {
      float ex = cx + side * gap;
      drawEye(c, ex, cy, eyeW, eyeH, open, side, unit);
    }
    drawMouth(c, cx, h * 0.76f, unit);

    if (!caption.isEmpty()) {
      text.setTextSize(Math.max(28f, unit * 0.075f));
      c.drawText(caption, w / 2f, h - text.getTextSize() * 0.5f, text);
    }
    postInvalidateOnAnimation();
  }

  private void drawEye(Canvas c, float ex, float ey, float ew, float eh, float open, int side, float unit) {
    switch (mood) {
      case HAPPY:
        // ^ ^ : upward arcs
        path.reset();
        path.moveTo(ex - ew / 2, ey + eh * 0.15f);
        path.quadTo(ex, ey - eh * 0.55f, ex + ew / 2, ey + eh * 0.15f);
        c.drawPath(path, line);
        return;
      case LOVE:
        drawHeart(c, ex, ey, ew * 1.05f);
        return;
      case BLIND:
        float s = ew * 0.38f;
        c.drawLine(ex - s, ey - s, ex + s, ey + s, line);
        c.drawLine(ex - s, ey + s, ex + s, ey - s, line);
        return;
      case SCARED:
        c.drawCircle(ex, ey, ew * 0.62f, eye);
        eye.setColor(Color.parseColor("#0E151D"));
        c.drawCircle(ex, ey, ew * 0.2f, eye);
        eye.setColor(moodColor(mood));
        return;
      default:
        break;
    }
    float hh = eh * open;
    if (mood == LidarBehaviors.Mood.IDLE) hh *= 0.35f; // sleepy
    if (mood == LidarBehaviors.Mood.FOCUSED) hh *= 0.75f; // narrowed
    r.set(ex - ew / 2, ey - hh / 2, ex + ew / 2, ey + hh / 2);
    c.drawRoundRect(r, ew * 0.35f, ew * 0.35f, eye);
    if (mood == LidarBehaviors.Mood.WORRIED) {
      // Brows high in the middle.
      float by = ey - eh * 0.62f;
      c.drawLine(ex - side * ew * 0.55f, by + eh * 0.12f, ex + side * ew * 0.45f, by - eh * 0.1f, line);
    } else if (mood == LidarBehaviors.Mood.FOCUSED) {
      // A flat lid line: concentrating.
      float by = ey - eh * 0.5f;
      c.drawLine(ex - ew * 0.6f, by + side * eh * 0.04f, ex + ew * 0.6f, by - side * eh * 0.04f, line);
    }
  }

  private void drawHeart(Canvas c, float x, float y, float size) {
    float s = size / 2f;
    path.reset();
    path.moveTo(x, y + s * 0.9f);
    path.cubicTo(x - s * 1.4f, y - s * 0.1f, x - s * 0.6f, y - s * 1.1f, x, y - s * 0.35f);
    path.cubicTo(x + s * 0.6f, y - s * 1.1f, x + s * 1.4f, y - s * 0.1f, x, y + s * 0.9f);
    path.close();
    c.drawPath(path, eye);
  }

  private void drawMouth(Canvas c, float x, float y, float unit) {
    float mw = unit * 0.16f;
    path.reset();
    switch (mood) {
      case HAPPY:
      case LOVE:
        path.moveTo(x - mw, y - unit * 0.02f);
        path.quadTo(x, y + unit * 0.09f, x + mw, y - unit * 0.02f);
        c.drawPath(path, line);
        break;
      case SCARED:
        r.set(x - mw * 0.35f, y - unit * 0.05f, x + mw * 0.35f, y + unit * 0.06f);
        c.drawOval(r, line);
        break;
      case SEARCHING:
        c.drawCircle(x, y, unit * 0.025f, line);
        break;
      case WORRIED:
        path.moveTo(x - mw, y);
        path.cubicTo(x - mw / 2, y - unit * 0.04f, x, y + unit * 0.04f, x + mw / 2, y - unit * 0.01f);
        path.lineTo(x + mw, y + unit * 0.02f);
        c.drawPath(path, line);
        break;
      case BLIND:
        c.drawLine(x - mw * 0.6f, y, x + mw * 0.6f, y, line);
        break;
      case FOCUSED:
      case IDLE:
      default:
        c.drawLine(x - mw * 0.45f, y, x + mw * 0.45f, y, line);
        break;
    }
  }
}
