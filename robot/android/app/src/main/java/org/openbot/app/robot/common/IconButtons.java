package org.openbot.app.robot.common;

import androidx.annotation.DrawableRes;
import androidx.annotation.StringRes;
import androidx.appcompat.widget.TooltipCompat;
import com.google.android.material.button.MaterialButton;

/**
 * Buttons show an icon instead of words. The words are kept as the content description (read
 * aloud by TalkBack) and as the tooltip shown on a long press, so nothing is lost.
 *
 * <p>Android guidance: "Icon buttons ... must have a content description"
 * (developer.android.com/guide/topics/ui/accessibility/apps) and Material 3 icon buttons
 * (m3.material.io/components/icon-buttons) use a tooltip to name the action.
 */
public final class IconButtons {

  private IconButtons() {}

  /** Change the icon and the label of an icon button (for buttons that switch, like Start/Stop). */
  public static void set(MaterialButton b, @DrawableRes int icon, @StringRes int label) {
    set(b, icon, b.getContext().getString(label));
  }

  public static void set(MaterialButton b, @DrawableRes int icon, CharSequence label) {
    b.setIconResource(icon);
    b.setText(null);
    label(b, label);
  }

  /** Name the button for TalkBack and long press, without showing words. */
  public static void label(MaterialButton b, CharSequence label) {
    b.setContentDescription(label);
    // android:tooltipText needs Android 8; TooltipCompat also covers Android 7.
    TooltipCompat.setTooltipText(b, label);
  }
}
