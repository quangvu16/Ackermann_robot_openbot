package org.openbot.app.robot.vehicle;

import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;

/**
 * The app-wide STOP latch behind the big red STOP button.
 *
 * <p>While latched, {@link Vehicle#sendControl()} sends 0,0 for every drive command, whichever
 * screen or mode produced it (joystick, GPS mission, autopilot, game controller). It stays latched
 * until the user presses RESUME, so a mission or an AI mode cannot drive the car again on its own.
 *
 * <p>It only governs commands from the app. With the RC switch in MANUAL the transmitter drives
 * the car directly, and the RC switch's HOLD position is the stop there.
 */
public final class EmergencyStop {

  /** Told on the thread that changed the latch. */
  public interface Listener {
    void onStopChanged(boolean latched);
  }

  private volatile boolean latched = false;
  private final List<Listener> listeners = new CopyOnWriteArrayList<>();

  public boolean isLatched() {
    return latched;
  }

  /** Stops the car and keeps it stopped. */
  public void latch() {
    set(true);
  }

  /** Lets the app drive again. */
  public void release() {
    set(false);
  }

  public void toggle() {
    set(!latched);
  }

  public void addListener(Listener listener) {
    listeners.add(listener);
  }

  public void removeListener(Listener listener) {
    listeners.remove(listener);
  }

  /**
   * The command to actually send for a requested left/right pair: the request itself, or 0,0 while
   * latched.
   */
  public float[] filter(float left, float right) {
    return latched ? new float[] {0f, 0f} : new float[] {left, right};
  }

  private void set(boolean value) {
    if (latched == value) return;
    latched = value;
    for (Listener l : listeners) l.onStopChanged(value);
  }
}
