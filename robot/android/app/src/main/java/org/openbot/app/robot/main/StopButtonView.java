package org.openbot.app.robot.main;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.view.HapticFeedbackConstants;
import android.view.View;
import org.openbot.app.robot.R;

/**
 * The big round STOP button that floats over every screen while the car is connected.
 *
 * <p>Red "STOP" stops the car and keeps it stopped (see EmergencyStop). It then turns green
 * "RESUME"; pressing that lets the app drive again. One large target in the same place on every
 * screen, so it can be hit without looking for it.
 */
public class StopButtonView extends View {
  private static final int RED = Color.rgb(0xD3, 0x2F, 0x2F);
  private static final int GREEN = Color.rgb(0x2E, 0x7D, 0x32);

  private final Paint fill = new Paint(Paint.ANTI_ALIAS_FLAG);
  private final Paint ring = new Paint(Paint.ANTI_ALIAS_FLAG);
  private final Paint text = new Paint(Paint.ANTI_ALIAS_FLAG);
  private boolean latched = false;

  public StopButtonView(Context context) {
    super(context);
    fill.setStyle(Paint.Style.FILL);
    ring.setStyle(Paint.Style.STROKE);
    ring.setColor(Color.WHITE);
    text.setColor(Color.WHITE);
    text.setTextAlign(Paint.Align.CENTER);
    text.setFakeBoldText(true);
    setClickable(true);
    setElevation(12f * getResources().getDisplayMetrics().density);
    updateDescription();
  }

  /** Shows STOP (false) or RESUME (true). */
  public void setLatched(boolean value) {
    if (latched == value) return;
    latched = value;
    updateDescription();
    invalidate();
  }

  @Override
  public boolean performClick() {
    performHapticFeedback(HapticFeedbackConstants.LONG_PRESS);
    return super.performClick();
  }

  private void updateDescription() {
    setContentDescription(
        getContext().getString(latched ? R.string.stop_button_resume : R.string.stop_button_stop));
  }

  @Override
  protected void onDraw(Canvas canvas) {
    super.onDraw(canvas);
    float w = getWidth();
    float h = getHeight();
    float r = Math.min(w, h) / 2f;
    fill.setColor(latched ? GREEN : RED);
    canvas.drawCircle(w / 2f, h / 2f, r, fill);
    ring.setStrokeWidth(r * 0.08f);
    canvas.drawCircle(w / 2f, h / 2f, r * 0.86f, ring);
    String label =
        getContext().getString(latched ? R.string.stop_button_resume : R.string.stop_button_stop);
    text.setTextSize(r * (label.length() > 5 ? 0.30f : 0.42f));
    float y = h / 2f - (text.descent() + text.ascent()) / 2f;
    canvas.drawText(label, w / 2f, y, text);
  }
}
