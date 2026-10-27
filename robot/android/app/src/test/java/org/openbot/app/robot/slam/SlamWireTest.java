package org.openbot.app.robot.slam;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.Arrays;
import java.util.Base64;
import org.junit.Test;

public class SlamWireTest {

  @Test
  public void stateHasEveryFieldAndTheCroppedMap() {
    OccupancyGrid g = new OccupancyGrid(40, 40, 0.05);
    g.set(10, 12, -2f); // free
    g.set(11, 12, 3f); // wall
    g.set(12, 14, -2f);
    String j = SlamWire.json("Cartographer", "AMCL", "Theta*", "MPPI", true, false, new Pose(1, 2, 0.5), 0.8, 12, 3,
        1, new double[] {3, 4}, Arrays.asList(new double[] {0, 0}, new double[] {1, 1}), "Driving \"on\"", g);
    assertTrue(j, j.startsWith("{\"type\":\"slam\",\"active\":true,\"engine\":\"Cartographer\""));
    assertTrue(j.contains("\"pose\":[1.000,2.000,0.500]"));
    assertTrue(j.contains("\"goal\":[3.000,4.000]"));
    assertTrue(j.contains("\"path\":[[0.000,0.000],[1.000,1.000]]"));
    assertTrue(j.contains("\"status\":\"Driving \\\"on\\\"\""));
    assertTrue(j.contains("\"w\":3,\"h\":3"));
    assertTrue(j.contains("\"x0\":-0.500,\"y0\":-0.400"));
    String cells = j.substring(j.indexOf("\"cells\":\"") + 9, j.lastIndexOf("\"}"));
    byte[] b = Base64.getDecoder().decode(cells);
    assertEquals(9, b.length);
    assertEquals(1, b[0]); // (10,12)
    assertEquals(2, b[1]); // (11,12)
    assertEquals(1, b[8]); // (12,14)
    assertEquals(0, b[4]);
  }

  @Test
  public void base64MatchesTheJdk() {
    for (int n = 0; n < 7; n++) {
      byte[] d = new byte[n];
      for (int i = 0; i < n; i++) d[i] = (byte) (i * 77 + 3);
      assertEquals(Base64.getEncoder().encodeToString(d), SlamWire.base64(d));
    }
  }

  @Test
  public void parsesCommandsAndRejectsJunk() {
    SlamWire.Command c = SlamWire.parse("{\"goal\": [1.5, -2]}");
    assertEquals("goal", c.kind);
    assertEquals(1.5, c.x, 1e-9);
    assertEquals(-2, c.y, 1e-9);
    c = SlamWire.parse("{\"place\":[0,1,3.14]}");
    assertEquals("place", c.kind);
    assertEquals(3.14, c.theta, 1e-9);
    assertEquals("explore", SlamWire.parse("{\"explore\":true}").kind);
    assertTrue(SlamWire.parse("{\"explore\":true}").flag);
    assertEquals("cancel", SlamWire.parse("{\"cancel\":true}").kind);
    assertEquals("findMe", SlamWire.parse("{\"findMe\":true}").kind);
    assertNull(SlamWire.parse("{\"goal\":[1]}").kind);
    assertNull(SlamWire.parse("{\"goal\":[NaN,1]}").kind);
    assertNull(SlamWire.parse("hello").kind);
    assertNull(SlamWire.parse(null).kind);
  }

  @Test
  public void emptyMapIsZeroSized() {
    String m = SlamWire.mapJson(new OccupancyGrid(10, 10, 0.05));
    assertTrue(m, m.contains("\"w\":0,\"h\":0,\"cells\":\"\""));
  }
}
