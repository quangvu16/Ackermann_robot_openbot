package org.openbot.app.robot.main;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.RectF;
import android.view.View;
import org.openbot.app.robot.R;
import org.openbot.app.robot.vehicle.RcMode;

/**
 * Full-screen face shown over the whole app while the car's RC switch is in MANUAL (smiling face)
 * or HOLD / LOST (crying face). Hidden in AUTO. Tapping hides it until the mode changes, so the
 * app stays reachable (for example to pause a GPS mission).
 *
 * <p>The face is drawn with plain Canvas calls, so it needs no image assets and scales to any
 * screen.
 */
public class RcModeFaceView extends View {
  private static final int BG_SMILE = Color.rgb(0x2E, 0x7D, 0x32); // green: you are driving
  private static final int BG_CRY = Color.rgb(0x37, 0x47, 0x4F); // blue-grey: stopped
  private static final int FACE = Color.rgb(0xFF, 0xD5, 0x4F);
  private static final int OUTLINE = Color.rgb(0x5D, 0x40, 0x37);
  private static final int MOUTH = Color.rgb(0x8D, 0x2B, 0x2B);
  private static final int TEAR = Color.rgb(0x42, 0xA5, 0xF5);
  private static final int CHEEK = Color.argb(0x66, 0xFF, 0x70, 0x70);

  private final Paint fill = new Paint(Paint.ANTI_ALIAS_FLAG);
  private final Paint stroke = new Paint(Paint.ANTI_ALIAS_FLAG);
  private final Paint text = new Paint(Paint.ANTI_ALIAS_FLAG);
  private final RectF oval = new RectF();

  private RcMode mode = null;
  private RcMode dismissedMode = null;

  public RcModeFaceView(Context context) {
    super(context);
    stroke.setStyle(Paint.Style.STROKE);
    stroke.setStrokeCap(Paint.Cap.ROUND);
    stroke.setColor(OUTLINE);
    fill.setStyle(Paint.Style.FILL);
    text.setColor(Color.WHITE);
    text.setTextAlign(Paint.Align.CENTER);
    text.setFakeBoldText(true);
    setVisibility(GONE);
    setClickable(true); // swallow touches so the screen underneath is not driven by accident
    setOnClickListener(
        v -> {
          dismissedMode = mode;
          setVisibility(GONE);
        });
  }

  /** Shows the face for this mode; null (no reports) or AUTO hides it. */
  public void setMode(RcMode newMode) {
    if (newMode != mode) {
      dismissedMode = null; // a new mode shows again even after a tap
    }
    mode = newMode;
    boolean show =
        newMode != null && newMode.face() != RcMode.Face.NONE && newMode != dismissedMode;
    setVisibility(show ? VISIBLE : GONE);
    if (show) {
      bringToFront();
      invalidate();
    }
  }

  @Override
  protected void onDraw(Canvas canvas) {
    super.onDraw(canvas);
    if (mode == null) return;
    boolean smile = mode.face() == RcMode.Face.SMILE;
    float w = getWidth();
    float h = getHeight();
    float r = Math.min(w, h) * 0.32f;
    float cx = w / 2f;
    float cy = h * 0.42f;

    canvas.drawColor(smile ? BG_SMILE : BG_CRY);

    // head
    fill.setColor(FACE);
    canvas.drawCircle(cx, cy, r, fill);
    stroke.setStrokeWidth(r * 0.05f);
    canvas.drawCircle(cx, cy, r, stroke);

    float eyeDx = r * 0.36f;
    float eyeY = cy - r * 0.18f;
    float eyeW = r * 0.32f;
    stroke.setStrokeWidth(r * 0.07f);

    if (smile) {
      // happy closed eyes: upward arcs
      for (int side = -1; side <= 1; side += 2) {
        float ex = cx + side * eyeDx;
        oval.set(ex - eyeW / 2, eyeY - eyeW / 3, ex + eyeW / 2, eyeY + eyeW / 3);
        canvas.drawArc(oval, 180, 180, false, stroke);
      }
      // rosy cheeks
      fill.setColor(CHEEK);
      canvas.drawCircle(cx - r * 0.55f, cy + r * 0.15f, r * 0.13f, fill);
      canvas.drawCircle(cx + r * 0.55f, cy + r * 0.15f, r * 0.13f, fill);
      // big open smile
      oval.set(cx - r * 0.52f, cy - r * 0.10f, cx + r * 0.52f, cy + r * 0.62f);
      fill.setColor(MOUTH);
      canvas.drawArc(oval, 0, 180, true, fill);
      stroke.setStrokeWidth(r * 0.05f);
      canvas.drawArc(oval, 0, 180, true, stroke);
    } else {
      // sad eyebrows, high in the middle
      stroke.setStrokeWidth(r * 0.06f);
      for (int side = -1; side <= 1; side += 2) {
        float ex = cx + side * eyeDx;
        float outer = ex + side * eyeW / 2;
        float inner = ex - side * eyeW / 2;
        canvas.drawLine(outer, eyeY - r * 0.20f, inner, eyeY - r * 0.32f, stroke);
      }
      // squeezed eyes: downward arcs
      stroke.setStrokeWidth(r * 0.07f);
      for (int side = -1; side <= 1; side += 2) {
        float ex = cx + side * eyeDx;
        oval.set(ex - eyeW / 2, eyeY - eyeW / 3, ex + eyeW / 2, eyeY + eyeW / 3);
        canvas.drawArc(oval, 0, 180, false, stroke);
      }
      // streams of tears down both cheeks
      fill.setColor(TEAR);
      float tearW = r * 0.15f;
      for (int side = -1; side <= 1; side += 2) {
        float tx = cx + side * eyeDx;
        oval.set(tx - tearW / 2, eyeY + eyeW / 4, tx + tearW / 2, cy + r * 0.80f);
        canvas.drawRoundRect(oval, tearW / 2, tearW / 2, fill);
      }
      // open crying mouth, turned down
      oval.set(cx - r * 0.34f, cy + r * 0.30f, cx + r * 0.34f, cy + r * 0.80f);
      fill.setColor(MOUTH);
      canvas.drawArc(oval, 180, 180, true, fill);
      stroke.setStrokeWidth(r * 0.05f);
      canvas.drawArc(oval, 180, 180, true, stroke);
    }

    // caption under the face
    int label;
    switch (mode) {
      case MANUAL:
        label = R.string.rc_mode_manual;
        break;
      case LOST:
        label = R.string.rc_mode_lost;
        break;
      default:
        label = R.string.rc_mode_hold;
        break;
    }
    text.setTextSize(r * 0.20f);
    text.setAlpha(255);
    float textY = cy + r + r * 0.45f;
    canvas.drawText(getContext().getString(label), cx, textY, text);
    text.setTextSize(r * 0.12f);
    text.setAlpha(180);
    String hint = getContext().getString(R.string.rc_mode_tap_hint);
    canvas.drawText(hint, cx, textY + r * 0.30f, text);
  }
}
