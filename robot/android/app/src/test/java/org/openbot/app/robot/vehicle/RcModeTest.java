package org.openbot.app.robot.vehicle;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

import org.junit.Test;

public class RcModeTest {

  @Test
  public void parsesEveryModeLetter() {
    assertEquals(RcMode.MANUAL, RcMode.parse("M"));
    assertEquals(RcMode.AUTO, RcMode.parse("A"));
    assertEquals(RcMode.HOLD, RcMode.parse("H"));
    assertEquals(RcMode.LOST, RcMode.parse("L"));
  }

  @Test
  public void toleratesLineEndings() {
    // The serial link delivers "mM\r\n"; the header is stripped before parsing.
    assertEquals(RcMode.MANUAL, RcMode.parse("M\r\n"));
    assertEquals(RcMode.HOLD, RcMode.parse(" H "));
  }

  @Test
  public void rejectsGarbage() {
    assertNull(RcMode.parse(null));
    assertNull(RcMode.parse(""));
    assertNull(RcMode.parse("X"));
    assertNull(RcMode.parse("MA"));
    assertNull(RcMode.parse("m"));
  }

  @Test
  public void smileForManualCryWhenStopped() {
    assertEquals(RcMode.Face.SMILE, RcMode.MANUAL.face());
    assertEquals(RcMode.Face.CRY, RcMode.HOLD.face());
    assertEquals(RcMode.Face.CRY, RcMode.LOST.face());
    assertEquals(RcMode.Face.NONE, RcMode.AUTO.face());
  }
}
