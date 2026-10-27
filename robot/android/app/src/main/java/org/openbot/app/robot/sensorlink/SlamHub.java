package org.openbot.app.robot.sensorlink;

import org.openbot.app.robot.slam.SlamWire;

/**
 * Hands the Indoor SLAM screen to the Sensor link server's {@code /slam} WebSocket while the screen
 * is open: the server asks it for the state (see {@link SlamWire}) and passes the page's commands
 * (set a goal, explore, find me...) back to it.
 */
public final class SlamHub {

  /** What the SLAM screen offers. Both methods may be called from any thread. */
  public interface Source {
    /** The state, with the map when {@code withMap}. */
    String json(boolean withMap);

    /** Bumped whenever the map changes, so the server only resends it then. */
    int mapVersion();

    void command(String text);
  }

  private static final SlamHub INSTANCE = new SlamHub();

  public static SlamHub get() {
    return INSTANCE;
  }

  private volatile Source source;

  private SlamHub() {}

  public void setSource(Source s) {
    source = s;
  }

  public void clearSource(Source s) {
    if (source == s) source = null;
  }

  public String json(boolean withMap) {
    Source s = source;
    return s == null ? SlamWire.INACTIVE : s.json(withMap);
  }

  public int mapVersion() {
    Source s = source;
    return s == null ? -1 : s.mapVersion();
  }

  public void command(String text) {
    Source s = source;
    if (s != null) s.command(text);
  }
}
