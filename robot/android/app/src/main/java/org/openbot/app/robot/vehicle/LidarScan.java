package org.openbot.app.robot.vehicle;

/**
 * One finished LiDAR revolution, decoded from the firmware's "LDS1" UDP datagram.
 *
 * <p>Wire format (see {@code firmware/openbot-esp32s3/lidar_udp.h}):
 *
 * <pre>
 *   offset  size          field
 *        0     4          "LDS1"
 *        4     2          bin count           uint16 little-endian
 *        6     4          scan id             uint32 little-endian
 *       10     4          reserved, zero
 *       14  2*bins        ranges, millimetres uint16 little-endian each
 * </pre>
 *
 * <p>0 mm means "no return" in that bin - the same convention the firmware uses on the {@code p}
 * proximity channel. A real return at 0 mm is outside the LD14's 20 mm minimum range, so the two
 * can never be confused.
 *
 * <p><b>Angles.</b> Bin {@code i} is the measurement {@code i * (360 / bins)} degrees clockwise
 * from the LiDAR's own zero mark. The bins go out and come back in that order and are never
 * reordered: the mounting offset is applied here with {@code forwardOffsetDeg} at read time,
 * exactly as the firmware applies {@code LIDAR_FORWARD_DEG} when folding octants. Reordering the
 * bins and then also applying the offset would make the two corrections fight.
 *
 * <p>Deliberately pure Java with no Android imports, so the parser can be unit tested on the JVM.
 */
public final class LidarScan {

  /** The four bytes every datagram starts with. */
  public static final int MAGIC = 0x3153444c; // "LDS1" little-endian

  public static final int HEADER_LEN = 14;

  /**
   * Largest bin count we will accept. Matches the firmware's {@code UDP_BINS_MAX}: 14 + 720 * 2 is
   * still one datagram, and anything claiming more than that is not a scan this firmware sends.
   */
  public static final int MAX_BINS = 720;

  private final int binCount;
  private final long scanId;
  private final int[] rangeMm;

  private LidarScan(int binCount, long scanId, int[] rangeMm) {
    this.binCount = binCount;
    this.scanId = scanId;
    this.rangeMm = rangeMm;
  }

  /**
   * Decode one datagram. Returns null for anything malformed - wrong magic, truncated, a bin count
   * of zero, or a bin count past {@link #MAX_BINS} - so a stray packet on the port can never be
   * mistaken for a scan of open road.
   */
  public static LidarScan parse(byte[] datagram) {
    return parse(datagram, datagram == null ? 0 : datagram.length);
  }

  /**
   * Decode the first {@code length} bytes of {@code datagram}. Separate from {@link #parse(byte[])}
   * because a UDP receive hands back a buffer longer than the packet; the length is what bounds the
   * payload, and reading past it would pick up stale bytes as ranges.
   */
  public static LidarScan parse(byte[] datagram, int length) {
    if (datagram == null || length < HEADER_LEN) {
      return null;
    }
    if (readU32(datagram, 0) != MAGIC) {
      return null;
    }
    int bins = readU16(datagram, 4);
    if (bins <= 0 || bins > MAX_BINS) {
      return null;
    }
    long expected = HEADER_LEN + (long) bins * 2;
    if (length < expected) {
      return null;
    }
    long id = readU32(datagram, 6);
    int[] ranges = new int[bins];
    for (int i = 0; i < bins; i++) {
      ranges[i] = readU16(datagram, HEADER_LEN + i * 2);
    }
    return new LidarScan(bins, id, ranges);
  }

  /** Number of bins in this revolution. */
  public int getBinCount() {
    return binCount;
  }

  /** Firmware-side scan counter, incremented once per sent revolution. */
  public long getScanId() {
    return scanId;
  }

  /**
   * Range of one bin in millimetres, or 0 for "no return". Bin {@code i} sits at {@code i * (360 /
   * getBinCount())} degrees clockwise from the LiDAR's zero mark.
   */
  public int getRangeMm(int bin) {
    if (bin < 0 || bin >= binCount) {
      return 0;
    }
    return rangeMm[bin];
  }

  /**
   * All ranges in bin order, never reordered. Millimetres, 0 for "no return". The returned array is
   * a copy, safe to hold onto across the next scan.
   */
  public int[] getRangesMm() {
    return rangeMm.clone();
  }

  /**
   * Angle of one bin in degrees, clockwise from the LiDAR's zero mark, in {@code [0, 360)}.
   *
   * <p>Uses the bin centre ({@code i + 0.5}) so a 360-bin scan reports bin 0 at 0.5 degrees rather
   * than snapping it onto the zero mark - the same centre convention the firmware uses when folding
   * octants.
   */
  public float getBinAngleDeg(int bin) {
    return ((bin + 0.5f) * 360.0f) / (float) binCount;
  }

  /**
   * Angle of one bin in degrees relative to the car's nose, in {@code [0, 360)}.
   *
   * @param forwardOffsetDeg where the car's nose sits in the LiDAR's frame, matching the firmware's
   *     {@code LIDAR_FORWARD_DEG}; 0 when the LiDAR's zero mark already points forward
   */
  public float getVehicleAngleDeg(int bin, float forwardOffsetDeg) {
    return wrapDeg(getBinAngleDeg(bin) - forwardOffsetDeg);
  }

  /**
   * Range of a bin in metres, or {@link Float#POSITIVE_INFINITY} when there is no return. {@code 0}
   * mm on the wire means "nothing measured here", not "an obstacle at zero" - the same distinction
   * {@link Proximity} draws on the {@code p} channel.
   */
  public float getRangeMeters(int bin) {
    int mm = getRangeMm(bin);
    return mm == 0 ? Float.POSITIVE_INFINITY : mm / 1000.0f;
  }

  /** True when this bin returned a measurement. */
  public boolean hasReturn(int bin) {
    return getRangeMm(bin) > 0;
  }

  /**
   * Number of bins that returned a measurement, which is what the firmware logs as valid points.
   */
  public int getValidPoints() {
    int n = 0;
    for (int mm : rangeMm) {
      if (mm > 0) {
        n++;
      }
    }
    return n;
  }

  /**
   * Fold this revolution into {@code sectorCount} octants, keeping the nearest return in each.
   * Sector {@code i} is centred on {@code i * (360 / sectorCount)} degrees from the nose and spans
   * half a sector either side.
   *
   * <p>This mirrors the firmware's {@code computeSectors()} exactly, so an app-side octant folded
   * from a scan and one reported on the {@code p} channel describe the same geometry.
   *
   * @param forwardOffsetDeg where the car's nose sits in the LiDAR's frame
   * @return nearest return per sector in millimetres, 0 where the sector is clear
   */
  public int[] computeSectorsMm(int sectorCount, float forwardOffsetDeg) {
    int[] out = new int[Math.max(1, sectorCount)];
    if (sectorCount < 1) {
      return out;
    }
    float step = 360.0f / (float) sectorCount;
    for (int bin = 0; bin < binCount; bin++) {
      int mm = rangeMm[bin];
      if (mm == 0) {
        continue;
      }
      float a = getVehicleAngleDeg(bin, forwardOffsetDeg);
      int sector = ((int) ((a + step * 0.5f) / step)) % out.length;
      if (out[sector] == 0 || mm < out[sector]) {
        out[sector] = mm;
      }
    }
    return out;
  }

  /** Degrees, wrapped to {@code [0, 360)}. */
  public static float wrapDeg(float deg) {
    float a = deg % 360.0f;
    return a < 0f ? a + 360.0f : a;
  }

  private static int readU16(byte[] b, int off) {
    return (b[off] & 0xFF) | ((b[off + 1] & 0xFF) << 8);
  }

  private static long readU32(byte[] b, int off) {
    return (b[off] & 0xFFL)
        | ((b[off + 1] & 0xFFL) << 8)
        | ((b[off + 2] & 0xFFL) << 16)
        | ((b[off + 3] & 0xFFL) << 24);
  }

  /**
   * Decode the body of an {@code x} line from the serial link: one "LDS1" datagram, base64
   * encoded (the firmware's lidar_serial.h). Returns null for anything malformed, exactly like
   * {@link #parse(byte[])} - the same checks apply because the same parser runs.
   */
  public static LidarScan parseSerialLine(String body) {
    byte[] datagram = decodeBase64(body);
    return datagram == null ? null : parse(datagram);
  }

  /**
   * Plain RFC 4648 base64 decoder. Written out because {@code java.util.Base64} needs API 26
   * (minSdk is 24) and {@code android.util.Base64} does not run in JVM unit tests. Returns null on
   * any character outside the alphabet or a length that is not a multiple of four.
   */
  static byte[] decodeBase64(String text) {
    if (text == null) {
      return null;
    }
    String s = text.trim();
    if (s.isEmpty() || s.length() % 4 != 0) {
      return null;
    }
    int pad = 0;
    if (s.endsWith("==")) {
      pad = 2;
    } else if (s.endsWith("=")) {
      pad = 1;
    }
    byte[] out = new byte[s.length() / 4 * 3 - pad];
    int o = 0;
    for (int i = 0; i < s.length(); i += 4) {
      int v = 0;
      for (int k = 0; k < 4; k++) {
        char c = s.charAt(i + k);
        int d;
        if (c >= 'A' && c <= 'Z') {
          d = c - 'A';
        } else if (c >= 'a' && c <= 'z') {
          d = c - 'a' + 26;
        } else if (c >= '0' && c <= '9') {
          d = c - '0' + 52;
        } else if (c == '+') {
          d = 62;
        } else if (c == '/') {
          d = 63;
        } else if (c == '=' && i + 4 == s.length() && k >= 4 - pad) {
          d = 0;
        } else {
          return null;
        }
        v = (v << 6) | d;
      }
      for (int k = 2; k >= 0; k--) {
        int b = (v >> (8 * k)) & 0xFF;
        if (o < out.length) {
          out[o++] = (byte) b;
        }
      }
    }
    return out;
  }

  /** Encode a datagram back out. Used by the tests to round-trip the parser. */
  public static byte[] encode(int scanId, int[] rangesMm) {
    if (rangesMm == null) {
      return null;
    }
    byte[] out = new byte[HEADER_LEN + rangesMm.length * 2];
    out[0] = 'L';
    out[1] = 'D';
    out[2] = 'S';
    out[3] = '1';
    out[4] = (byte) (rangesMm.length & 0xFF);
    out[5] = (byte) ((rangesMm.length >> 8) & 0xFF);
    out[6] = (byte) (scanId & 0xFF);
    out[7] = (byte) ((scanId >> 8) & 0xFF);
    out[8] = (byte) ((scanId >> 16) & 0xFF);
    out[9] = (byte) ((scanId >> 24) & 0xFF);
    for (int i = 0; i < rangesMm.length; i++) {
      int mm = rangesMm[i] & 0xFFFF;
      out[HEADER_LEN + i * 2] = (byte) (mm & 0xFF);
      out[HEADER_LEN + i * 2 + 1] = (byte) ((mm >> 8) & 0xFF);
    }
    return out;
  }
}
