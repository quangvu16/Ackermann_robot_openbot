package org.openbot.app.robot.sensorlink;

import android.content.Context;
import android.content.SharedPreferences;
import androidx.preference.PreferenceManager;

/** What the Sensor link dialog saves. Read fresh each time; the values are tiny. */
public final class SensorLinkSettings {

  public enum Mode {
    OFF,
    SERVER,
    CLIENT
  }

  private static final String KEY_MODE = "sensor_link_mode";
  private static final String KEY_PORT = "sensor_link_port";
  private static final String KEY_ADDRESS = "sensor_link_address";
  private static final String KEY_USE_GPS = "sensor_link_use_gps";

  public Mode mode = Mode.OFF;
  public int port = SensorLinkProtocol.DEFAULT_PORT;
  public String address = "";
  /** Client mode: GPS Mission drives on the other phone's GPS and compass. */
  public boolean useRemoteGps = true;

  public static SensorLinkSettings load(Context context) {
    SharedPreferences p = PreferenceManager.getDefaultSharedPreferences(context);
    SensorLinkSettings s = new SensorLinkSettings();
    try {
      s.mode = Mode.valueOf(p.getString(KEY_MODE, Mode.OFF.name()));
    } catch (IllegalArgumentException e) {
      s.mode = Mode.OFF;
    }
    s.port = p.getInt(KEY_PORT, SensorLinkProtocol.DEFAULT_PORT);
    s.address = p.getString(KEY_ADDRESS, "");
    s.useRemoteGps = p.getBoolean(KEY_USE_GPS, true);
    return s;
  }

  public void save(Context context) {
    PreferenceManager.getDefaultSharedPreferences(context)
        .edit()
        .putString(KEY_MODE, mode.name())
        .putInt(KEY_PORT, port)
        .putString(KEY_ADDRESS, address == null ? "" : address.trim())
        .putBoolean(KEY_USE_GPS, useRemoteGps)
        .apply();
  }

  public static boolean isValidPort(int port) {
    return port >= 1024 && port <= 65535;
  }
}
