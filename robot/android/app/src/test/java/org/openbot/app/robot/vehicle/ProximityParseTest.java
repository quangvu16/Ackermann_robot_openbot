package org.openbot.app.robot.vehicle;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;

import org.junit.Test;

/**
 * Pure-JVM tests for the {@code p} message parser.
 *
 * <p>The parse is deliberately strict: a truncated or garbled BLE notification must never be
 * accepted as a scan of eight clear sectors, which would let the car drive straight at whatever it
 * just failed to see.
 */
public class ProximityParseTest {

  @Test
  public void parsesEightCommaSeparatedValues() {
    float[] s = Proximity.parseSectorsCm("100,200,300,400,500,600,700,800");
    assertNotNull(s);
    assertEquals(Proximity.SECTORS, s.length);
    assertEquals(100f, s[0], 0);
    assertEquals(800f, s[7], 0);
  }

  @Test
  public void parsesTheFirmwareZeroAsNoReturn() {
    // 0 on the wire means "nothing measured here". It must survive the parse intact so
    // Proximity can convert it to infinity rather than to "obstacle at zero".
    float[] s = Proximity.parseSectorsCm("0,0,0,0,0,0,0,0");
    assertNotNull(s);
    assertArrayEquals(new float[] {0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f}, s, 0);
  }

  @Test
  public void rejectsTooFewSectors() {
    assertNull(Proximity.parseSectorsCm("100,200,300"));
  }

  @Test
  public void rejectsTooManySectors() {
    assertNull(Proximity.parseSectorsCm("1,2,3,4,5,6,7,8,9"));
  }

  @Test
  public void rejectsEmptyBody() {
    assertNull(Proximity.parseSectorsCm(""));
  }

  @Test
  public void rejectsNullBody() {
    assertNull(Proximity.parseSectorsCm(null));
  }

  @Test
  public void rejectsNonNumericField() {
    assertNull(Proximity.parseSectorsCm("100,200,300,400,x,600,700,800"));
  }

  @Test
  public void rejectsTruncatedNotification() {
    // A BLE notification cut off mid-message must not read as a clear road.
    assertNull(Proximity.parseSectorsCm("100,200,300,400,500,600,700,"));
  }

  @Test
  public void toleratesWhitespaceAroundFields() {
    float[] s = Proximity.parseSectorsCm("100, 200,300,400,500,600,700, 800");
    assertNotNull(s);
    assertEquals(200f, s[1], 0);
    assertEquals(800f, s[7], 0);
  }

  @Test
  public void acceptsFloatValues() {
    // The firmware sends integers, but the parse should not be brittle about it.
    float[] s = Proximity.parseSectorsCm("12.5,0,0,0,0,0,0,0");
    assertNotNull(s);
    assertEquals(12.5f, s[0], 1e-6);
  }

  @Test
  public void sectorCountMatchesObjectAvoidance() {
    // The two classes are a contract with the firmware's PROX_SECTORS; if they disagree the
    // message length check and the octant indices silently stop lining up.
    assertEquals(ObjectAvoidance.SECTORS, Proximity.SECTORS);
  }
}
