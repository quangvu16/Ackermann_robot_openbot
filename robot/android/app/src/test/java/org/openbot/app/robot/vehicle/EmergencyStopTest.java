package org.openbot.app.robot.vehicle;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import org.junit.Test;

public class EmergencyStopTest {

  @Test
  public void startsReleasedAndPassesCommandsThrough() {
    EmergencyStop stop = new EmergencyStop();
    assertFalse(stop.isLatched());
    float[] out = stop.filter(0.5f, -0.25f);
    assertEquals(0.5f, out[0], 0f);
    assertEquals(-0.25f, out[1], 0f);
  }

  @Test
  public void latchedSendsZeroWhateverIsRequested() {
    EmergencyStop stop = new EmergencyStop();
    stop.latch();
    float[] out = stop.filter(1f, 1f);
    assertEquals(0f, out[0], 0f);
    assertEquals(0f, out[1], 0f);
    out = stop.filter(-1f, 0.3f); // reversing is held too: STOP means stop
    assertEquals(0f, out[0], 0f);
    assertEquals(0f, out[1], 0f);
  }

  @Test
  public void staysLatchedUntilReleased() {
    EmergencyStop stop = new EmergencyStop();
    stop.latch();
    stop.latch();
    assertTrue(stop.isLatched());
    stop.release();
    assertFalse(stop.isLatched());
    stop.toggle();
    assertTrue(stop.isLatched());
  }

  @Test
  public void listenersHearOnlyRealChanges() {
    EmergencyStop stop = new EmergencyStop();
    List<Boolean> heard = new ArrayList<>();
    stop.addListener(heard::add);
    stop.latch();
    stop.latch(); // no change, no event
    stop.release();
    assertEquals(2, heard.size());
    assertTrue(heard.get(0));
    assertFalse(heard.get(1));
  }
}
