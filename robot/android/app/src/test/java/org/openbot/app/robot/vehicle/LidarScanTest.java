package org.openbot.app.robot.vehicle;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** JVM tests for the "LDS1" datagram decoder and its angle/sector geometry. */
public class LidarScanTest {

  // --- parsing ---

  @Test
  public void parseRoundTripsKnownScan() {
    int[] ranges = new int[360];
    ranges[0] = 1000;
    ranges[180] = 2500;
    ranges[359] = 800;
    byte[] datagram = LidarScan.encode(42, ranges);

    LidarScan scan = LidarScan.parse(datagram);

    assertNotNull(scan);
    assertEquals(360, scan.getBinCount());
    assertEquals(42L, scan.getScanId());
    assertEquals(1000, scan.getRangeMm(0));
    assertEquals(2500, scan.getRangeMm(180));
    assertEquals(800, scan.getRangeMm(359));
    assertEquals(0, scan.getRangeMm(1));
    assertEquals(3, scan.getValidPoints());
  }

  @Test
  public void parseRejectsNullAndShortDatagrams() {
    assertNull(LidarScan.parse(null));
    assertNull(LidarScan.parse(new byte[0]));
    assertNull(LidarScan.parse(new byte[13]));
  }

  @Test
  public void parseRejectsWrongMagic() {
    byte[] datagram = LidarScan.encode(1, new int[] {100});
    datagram[0] = 'X';
    assertNull(LidarScan.parse(datagram));
  }

  @Test
  public void parseRejectsZeroBins() {
    byte[] datagram = LidarScan.encode(1, new int[0]);
    assertNull(LidarScan.parse(datagram));
  }

  @Test
  public void parseRejectsBinCountPastMax() {
    // 721 bins claims more than the firmware will ever send (UDP_BINS_MAX is 720).
    byte[] datagram = new byte[LidarScan.HEADER_LEN + 721 * 2];
    datagram[0] = 'L';
    datagram[1] = 'D';
    datagram[2] = 'S';
    datagram[3] = '1';
    datagram[4] = (byte) (721 & 0xFF);
    datagram[5] = (byte) ((721 >> 8) & 0xFF);
    assertNull(LidarScan.parse(datagram));
  }

  @Test
  public void parseRejectsTruncatedPayload() {
    byte[] datagram = LidarScan.encode(1, new int[] {100, 200, 300});
    // Claim three bins but deliver only two of them.
    byte[] truncated = new byte[LidarScan.HEADER_LEN + 2 * 2];
    System.arraycopy(datagram, 0, truncated, 0, truncated.length);
    assertNull(LidarScan.parse(truncated));
  }

  @Test
  public void parseUsesLengthNotBufferCapacity() {
    // A UDP receive hands back an oversized buffer; the length is what bounds the payload. Reading
    // past it would pick up stale ranges from the previous datagram as if they were new.
    int[] ranges = new int[] {100, 200};
    byte[] datagram = LidarScan.encode(7, ranges);
    byte[] oversized = new byte[4096];
    System.arraycopy(datagram, 0, oversized, 0, datagram.length);
    // Fill the tail with what would be a false close obstacle.
    for (int i = datagram.length; i < oversized.length; i += 2) {
      oversized[i] = 0x10;
      oversized[i + 1] = 0x27; // 10000 mm
    }

    LidarScan scan = LidarScan.parse(oversized, datagram.length);

    assertNotNull(scan);
    assertEquals(2, scan.getBinCount());
    assertEquals(100, scan.getRangeMm(0));
    assertEquals(200, scan.getRangeMm(1));
  }

  @Test
  public void parseAcceptsLargestBinCount() {
    int[] ranges = new int[720];
    ranges[0] = 1;
    byte[] datagram = LidarScan.encode(1, ranges);
    LidarScan scan = LidarScan.parse(datagram);
    assertNotNull(scan);
    assertEquals(720, scan.getBinCount());
  }

  @Test
  public void parseReadsFullSixteenBitRanges() {
    int[] ranges = new int[] {0xFFFF, 0x0102};
    LidarScan scan = LidarScan.parse(LidarScan.encode(1, ranges));
    assertNotNull(scan);
    assertEquals(65535, scan.getRangeMm(0));
    assertEquals(258, scan.getRangeMm(1));
  }

  // --- units and the zero convention ---

  @Test
  public void zeroRangeReadsAsClearNotAsObstacleAtZero() {
    LidarScan scan = LidarScan.parse(LidarScan.encode(1, new int[] {0, 1500}));
    assertNotNull(scan);
    assertTrue(scan.hasReturn(1));
    assertFalse(scan.hasReturn(0));
    assertEquals(1.5f, scan.getRangeMeters(1), 1e-6);
    // A 0 mm bin is "nothing measured here" and must not read as an obstacle sitting on the car -
    // the same distinction Proximity draws on the p channel.
    assertEquals(Float.POSITIVE_INFINITY, scan.getRangeMeters(0), 0f);
  }

  @Test
  public void outOfRangeBinsAreSafe() {
    LidarScan scan = LidarScan.parse(LidarScan.encode(1, new int[] {500}));
    assertNotNull(scan);
    assertEquals(0, scan.getRangeMm(-1));
    assertEquals(0, scan.getRangeMm(1));
    assertEquals(Float.POSITIVE_INFINITY, scan.getRangeMeters(99), 0f);
  }

  // --- angles ---

  @Test
  public void binAnglesAreCentredAndOneDegreeApart() {
    LidarScan scan = LidarScan.parse(LidarScan.encode(1, new int[360]));
    assertNotNull(scan);
    // Bin centres, not bin edges: bin 0 at 0.5 deg for a 360-bin scan.
    assertEquals(0.5f, scan.getBinAngleDeg(0), 1e-4);
    assertEquals(90.5f, scan.getBinAngleDeg(90), 1e-4);
    assertEquals(359.5f, scan.getBinAngleDeg(359), 1e-4);
  }

  @Test
  public void vehicleAngleAppliesForwardOffset() {
    LidarScan scan = LidarScan.parse(LidarScan.encode(1, new int[360]));
    assertNotNull(scan);
    // Nose 90 deg clockwise from the LiDAR zero mark: bin 0 (LiDAR 0.5 deg) is now at vehicle
    // 270.5 deg - behind and to the left.
    assertEquals(270.5f, scan.getVehicleAngleDeg(0, 90f), 1e-4);
    // Bin 90 (LiDAR 90.5 deg) lines up with the nose.
    assertEquals(0.5f, scan.getVehicleAngleDeg(90, 90f), 1e-4);
  }

  @Test
  public void wrapDegKeepsResultInZeroTo360() {
    assertEquals(10f, LidarScan.wrapDeg(370f), 1e-4);
    assertEquals(350f, LidarScan.wrapDeg(-10f), 1e-4);
    assertEquals(0f, LidarScan.wrapDeg(360f), 1e-4);
    assertEquals(0f, LidarScan.wrapDeg(0f), 1e-4);
  }

  // --- sector folding, which has to match the firmware's computeSectors() ---

  @Test
  public void sectorFoldingPicksNearestReturn() {
    int[] ranges = new int[360];
    ranges[1] = 3000; // vehicle 1.5 deg -> front
    ranges[2] = 1200; // vehicle 2.5 deg -> front, nearer
    ranges[180] = 4000; // vehicle 180.5 deg -> back
    LidarScan scan = LidarScan.parse(LidarScan.encode(1, ranges));
    assertNotNull(scan);

    int[] sectors = scan.computeSectorsMm(8, 0f);

    assertEquals(8, sectors.length);
    assertEquals(1200, sectors[0]); // nearest of the two front returns
    assertEquals(4000, sectors[4]); // back
    assertEquals(0, sectors[2]); // right is clear, not zero-distance
  }

  @Test
  public void sectorFoldingHonoursForwardOffset() {
    int[] ranges = new int[360];
    ranges[0] = 700; // LiDAR 0.5 deg
    LidarScan scan = LidarScan.parse(LidarScan.encode(1, ranges));
    assertNotNull(scan);

    // With the nose on the LiDAR zero mark this is the front sector.
    assertEquals(700, scan.computeSectorsMm(8, 0f)[0]);
    // Rotate the car's nose 180 deg and the same bin is now behind it.
    assertEquals(700, scan.computeSectorsMm(8, 180f)[4]);
  }

  @Test
  public void emptySectorStaysZeroNotInfinite() {
    LidarScan scan = LidarScan.parse(LidarScan.encode(1, new int[360]));
    assertNotNull(scan);
    int[] sectors = scan.computeSectorsMm(8, 0f);
    // The wire spells "nothing measured" as 0; conversion to metres is the reader's job.
    for (int mm : sectors) {
      assertEquals(0, mm);
    }
    assertEquals(0, scan.getValidPoints());
  }

  @Test
  public void sectorFoldingAcceptsFewerSectors() {
    int[] ranges = new int[360];
    ranges[0] = 1000;
    ranges[180] = 2000;
    LidarScan scan = LidarScan.parse(LidarScan.encode(1, ranges));
    assertNotNull(scan);

    int[] halves = scan.computeSectorsMm(2, 0f);

    assertEquals(2, halves.length);
    assertEquals(1000, halves[0]); // front half
    assertEquals(2000, halves[1]); // rear half
  }

  @Test
  public void sectorFoldingAlwaysReturnsAnArray() {
    LidarScan scan = LidarScan.parse(LidarScan.encode(1, new int[] {100}));
    assertNotNull(scan);
    assertNotNull(scan.computeSectorsMm(8, 0f));
    assertNotNull(scan.computeSectorsMm(1, 0f));
    assertEquals(1, scan.computeSectorsMm(1, 0f).length);
    // Zero sectors cannot exist, but the reader must not get null for asking.
    assertNotNull(scan.computeSectorsMm(0, 0f));
    assertEquals(1, scan.computeSectorsMm(0, 0f).length);
  }

  // region serial "x" line

  @Test
  public void serialLineDecodesTheSameDatagram() {
    // Generated independently (Python base64 + struct) for id 7, ranges 100, 0, 300, 65535.
    LidarScan scan = LidarScan.parseSerialLine("TERTMQQABwAAAAAAAABkAAAALAH//w==");
    assertNotNull(scan);
    assertEquals(4, scan.getBinCount());
    assertEquals(7L, scan.getScanId());
    assertEquals(100, scan.getRangeMm(0));
    assertEquals(0, scan.getRangeMm(1));
    assertEquals(300, scan.getRangeMm(2));
    assertEquals(65535, scan.getRangeMm(3));
  }

  @Test
  public void serialLineRoundTripsEncode() {
    int[] ranges = new int[180];
    for (int i = 0; i < ranges.length; i++) {
      ranges[i] = (i % 7 == 0) ? 0 : 200 + i * 10;
    }
    byte[] datagram = LidarScan.encode(42, ranges);
    StringBuilder b64 = new StringBuilder();
    String tbl = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    for (int i = 0; i < datagram.length; i += 3) {
      int n = Math.min(3, datagram.length - i);
      int v = (datagram[i] & 0xFF) << 16;
      if (n > 1) v |= (datagram[i + 1] & 0xFF) << 8;
      if (n > 2) v |= datagram[i + 2] & 0xFF;
      b64.append(tbl.charAt((v >> 18) & 63)).append(tbl.charAt((v >> 12) & 63));
      b64.append(n > 1 ? tbl.charAt((v >> 6) & 63) : '=').append(n > 2 ? tbl.charAt(v & 63) : '=');
    }
    LidarScan scan = LidarScan.parseSerialLine(b64.toString());
    assertNotNull(scan);
    assertEquals(180, scan.getBinCount());
    for (int i = 0; i < ranges.length; i++) {
      assertEquals(ranges[i], scan.getRangeMm(i));
    }
  }

  @Test
  public void serialLineRejectsGarbage() {
    assertNull(LidarScan.parseSerialLine(null));
    assertNull(LidarScan.parseSerialLine(""));
    assertNull(LidarScan.parseSerialLine("abc")); // not a multiple of 4
    assertNull(LidarScan.parseSerialLine("TERT!QQA")); // bad character
    assertNull(LidarScan.parseSerialLine("AAAAAAAA")); // decodes, but no LDS1 magic
    assertNull(LidarScan.parseSerialLine("TERTMQQABwAA")); // header cut short
  }

  // endregion
}
