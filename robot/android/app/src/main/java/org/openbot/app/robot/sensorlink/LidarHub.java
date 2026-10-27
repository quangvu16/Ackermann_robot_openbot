package org.openbot.app.robot.sensorlink;

import org.openbot.app.robot.vehicle.LidarScan;
import org.openbot.app.robot.vehicle.LidarScanner;
import org.openbot.app.robot.vehicle.Proximity;

/**
 * Hands the open screen's LiDAR (full scans from {@link LidarScanner}, the car's 8 proximity
 * sectors from {@link Proximity}) to the Sensor link server's {@code /lidar} WebSocket.
 *
 * <p>Every driving screen already runs a LidarScanner (UDP from the car's WiFi, or {@code x}
 * lines on USB) and holds the vehicle's Proximity, so this only points at them while the screen
 * is resumed.
 */
public final class LidarHub {

  private static final LidarHub INSTANCE = new LidarHub();

  public static LidarHub get() {
    return INSTANCE;
  }

  private volatile LidarScanner scanner;
  private volatile Proximity proximity;

  private LidarHub() {}

  public void setSource(LidarScanner scanner, Proximity proximity) {
    this.scanner = scanner;
    this.proximity = proximity;
  }

  public void clearSource(LidarScanner scanner) {
    if (this.scanner == scanner) {
      this.scanner = null;
      this.proximity = null;
    }
  }

  /** Id of the newest full scan, or -1 when there is none. */
  public long latestScanId() {
    LidarScanner s = scanner;
    LidarScan scan = s == null ? null : s.getLatestScan();
    return scan == null ? -1 : scan.getScanId();
  }

  /** The current picture as {@link SensorLinkProtocol#lidarJson}, or null with no LiDAR at all. */
  public String latestJson() {
    LidarScanner s = scanner;
    Proximity p = proximity;
    LidarScan scan = s == null ? null : s.getLatestScan();
    int[] sectors = null;
    long sectorsAge = 0;
    if (p != null && p.hasEverReported()) {
      float[] m = p.getSectorMeters();
      sectors = new int[m.length];
      for (int i = 0; i < m.length; i++) {
        sectors[i] = Float.isInfinite(m[i]) || Float.isNaN(m[i]) ? 0 : Math.round(m[i] * 1000f);
      }
      sectorsAge = p.getAge();
    }
    if (scan == null && sectors == null) return null;
    return SensorLinkProtocol.lidarJson(
        scan == null ? -1 : scan.getScanId(),
        scan == null ? new int[0] : scan.getRangesMm(),
        0f,
        scan == null ? 0 : s.getAge(),
        sectors,
        sectorsAge);
  }
}
