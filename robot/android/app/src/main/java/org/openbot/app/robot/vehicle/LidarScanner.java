package org.openbot.app.robot.vehicle;

import android.os.SystemClock;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Receives "LDS1" scan datagrams over UDP and keeps the latest revolution on hand for display.
 *
 * <p>The firmware treats WiFi as an optional side channel that carries scans somewhere else (ROS
 * 2); the car itself is driven over BLE. The app joins that same stream so the operator can see
 * what the LiDAR sees, which means this runs entirely off the control path: nothing in {@code
 * Vehicle.sendControl()} ever waits on a socket, and a phone that is not on the robot's WiFi simply
 * shows "no scan" while the car drives exactly as before.
 *
 * <p>Staleness works like {@link Proximity} but is stricter, because this feed is for display
 * rather than for a safety law: a scan goes stale after {@link #STALE_AFTER_MS} and the view is
 * expected to say so rather than keep drawing a frozen picture of the room.
 */
public class LidarScanner {

  /** Default port, matching the firmware's {@code LIDAR_UDP_PORT}. */
  public static final int DEFAULT_PORT = 9900;

  /**
   * How old the newest scan may be before the picture is considered frozen. The firmware sends one
   * datagram per revolution: about 6 Hz at the LD14's 115200 baud and about 11 Hz at the LD14P's
   * 230400, so 500 ms is several missed revolutions rather than a slow link.
   */
  public static final long STALE_AFTER_MS = 500;

  /** Age reported before any scan has arrived. */
  private static final long NEVER_FED_AGE_MS = 1000;

  private final AtomicReference<LidarScan> latest = new AtomicReference<>();
  private final AtomicReference<Long> timestamp = new AtomicReference<>(0L);
  private final AtomicBoolean running = new AtomicBoolean(false);

  private Thread thread;
  private DatagramSocket socket;

  /**
   * Start listening. Returns false if the port could not be bound - another copy of the app, or a
   * ROS receiver already holding it. The caller is free to carry on without a scan view.
   */
  public synchronized boolean start(int port) {
    if (running.get()) {
      return true;
    }
    try {
      socket = new DatagramSocket(null);
      socket.setReuseAddress(true);
      socket.bind(new InetSocketAddress(port));
      socket.setSoTimeout(500);
    } catch (Exception e) {
      closeSocket();
      return false;
    }
    running.set(true);
    thread = new Thread(this::receiveLoop, "lidar-udp");
    thread.setDaemon(true);
    thread.start();
    return true;
  }

  /** Stop listening and drop the socket. Safe to call twice. */
  public synchronized void stop() {
    running.set(false);
    if (thread != null) {
      thread.interrupt();
      thread = null;
    }
    closeSocket();
    latest.set(null);
    timestamp.set(0L);
  }

  private void closeSocket() {
    if (socket != null && !socket.isClosed()) {
      socket.close();
    }
    socket = null;
  }

  private void receiveLoop() {
    byte[] buf = new byte[LidarScan.HEADER_LEN + LidarScan.MAX_BINS * 2];
    while (running.get()) {
      DatagramSocket s = socket;
      if (s == null || s.isClosed()) {
        break;
      }
      try {
        DatagramPacket packet = new DatagramPacket(buf, buf.length);
        s.receive(packet);
        // parse() takes an explicit length: the buffer is oversized for a 360-bin scan and the
        // trailing bytes are stale ranges from the previous datagram.
        LidarScan scan = LidarScan.parse(packet.getData(), packet.getLength());
        if (scan != null) {
          latest.set(scan);
          timestamp.set(SystemClock.elapsedRealtime());
        }
      } catch (Exception ignored) {
        // A timeout just means no scan this interval - the socket is set to 500 ms so the loop
        // re-checks running() regularly. Anything else (socket closed under us) ends the loop.
        if (!running.get()) {
          break;
        }
      }
    }
  }

  /** The newest accepted scan, or null when nothing has ever arrived. */
  /**
   * Accept a scan that arrived some other way than this scanner's UDP socket - the {@code x} line
   * on a USB serial link. It becomes the newest scan exactly as a datagram would, so the plan view
   * and its staleness check do not care which path it came by.
   */
  public void offer(LidarScan scan) {
    if (scan == null) {
      return;
    }
    latest.set(scan);
    timestamp.set(SystemClock.elapsedRealtime());
  }

  public LidarScan getLatestScan() {
    return latest.get();
  }

  /**
   * Milliseconds since the last accepted scan, or {@link #NEVER_FED_AGE_MS} before the first one.
   * Keeps growing when scans stop - that is the whole point of it.
   */
  public long getAge() {
    Long ts = timestamp.get();
    if (ts == null || ts == 0L) {
      return NEVER_FED_AGE_MS;
    }
    return SystemClock.elapsedRealtime() - ts;
  }

  /** True once at least one scan has been accepted. */
  public boolean hasEverReported() {
    Long ts = timestamp.get();
    return ts != null && ts > 0L;
  }

  /** True while scans are still arriving and the picture on screen can be trusted. */
  public boolean isFresh() {
    return hasEverReported() && getAge() <= STALE_AFTER_MS;
  }

  /** True when scans stopped after having arrived - a frozen picture rather than no sensor. */
  public boolean isStale() {
    return hasEverReported() && getAge() > STALE_AFTER_MS;
  }

  /** True when the listener is running, whether or not anything has arrived. */
  public boolean isRunning() {
    return running.get();
  }

  /**
   * The newest scan folded into {@code sectorCount} octants, in the same millimetres-per-sector,
   * 0-means-clear shape the {@code p} channel carries. Returns null when there is no scan, so a
   * missing reading can never be mistaken for eight clear sectors.
   */
  public int[] getLatestSectorsMm(int sectorCount, float forwardOffsetDeg) {
    LidarScan scan = latest.get();
    if (scan == null) {
      return null;
    }
    return scan.computeSectorsMm(sectorCount, forwardOffsetDeg);
  }
}
