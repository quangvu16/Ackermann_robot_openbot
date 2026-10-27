package org.openbot.app.robot.sensorlink;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Arrays;
import org.junit.Test;

public class RemoteSettingsTest {
  private boolean torch;
  private String speed = "normal";
  private double zoom = 1;
  private boolean stopped;

  private RemoteSettings settings() {
    RemoteSettings r = new RemoteSettings();
    r.addProvider(
        out -> {
          out.add(RemoteSettings.Setting.bool("camera.torch", "Camera", "Flashlight", () -> torch, v -> torch = (Boolean) v));
          out.add(RemoteSettings.Setting.choice("drive.speed", "Driving", "Speed mode", Arrays.asList("slow", "normal", "fast"), () -> speed, v -> speed = (String) v));
          out.add(RemoteSettings.Setting.range("camera.zoom", "Camera", "Zoom", 1, 5, 0.1, () -> zoom, v -> zoom = (Double) v));
          out.add(RemoteSettings.Setting.action("drive.stop", "Driving", "Emergency stop", "STOP", "RESUME", () -> stopped, v -> stopped = (Boolean) v));
        });
    return r;
  }

  @Test
  public void appliesEachTypeFromPageMessages() {
    RemoteSettings r = settings();
    assertTrue(r.apply("{\"set\":\"camera.torch\",\"value\":true}"));
    assertTrue(torch);
    assertTrue(r.apply("{\"set\":\"drive.speed\",\"value\":\"fast\"}"));
    assertEquals("fast", speed);
    assertTrue(r.apply("{\"set\":\"camera.zoom\",\"value\":2.5}"));
    assertEquals(2.5, zoom, 1e-9);
    assertTrue(r.apply("{\"set\":\"drive.stop\",\"value\":true}"));
    assertTrue(stopped);
  }

  @Test
  public void rejectsUnknownKeysAndBadValues() {
    RemoteSettings r = settings();
    assertFalse(r.apply("{\"set\":\"no.such\",\"value\":true}"));
    assertFalse(r.apply("{\"set\":\"drive.speed\",\"value\":\"ludicrous\"}"));
    assertEquals("normal", speed);
    assertFalse(r.apply("{\"set\":\"camera.torch\",\"value\":\"maybe\"}"));
    assertFalse(r.apply("not json"));
    // Out-of-range numbers are clamped, not rejected.
    assertTrue(r.apply("{\"set\":\"camera.zoom\",\"value\":99}"));
    assertEquals(5, zoom, 1e-9);
  }

  @Test
  public void stateDescribesEverySetting() {
    String json = settings().stateJson("GPS Mission");
    assertTrue(json, json.startsWith("{\"screen\":\"GPS Mission\",\"settings\":["));
    assertTrue(json.contains("{\"key\":\"camera.torch\",\"group\":\"Camera\",\"label\":\"Flashlight\",\"type\":\"bool\",\"value\":false}"));
    assertTrue(json.contains("\"options\":[\"slow\",\"normal\",\"fast\"]"));
    assertTrue(json.contains("\"min\":1,\"max\":5,\"step\":0.1"));
    assertTrue(json.contains("\"options\":[\"STOP\",\"RESUME\"]"));
  }
}
