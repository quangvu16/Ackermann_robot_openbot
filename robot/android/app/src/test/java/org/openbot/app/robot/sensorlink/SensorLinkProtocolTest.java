package org.openbot.app.robot.sensorlink;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.Arrays;
import org.junit.Test;

public class SensorLinkProtocolTest {

  @Test
  public void parsesSingleSensorPath() {
    SensorLinkProtocol.Request r =
        SensorLinkProtocol.parse("/sensor/connect?type=android.sensor.accelerometer");
    assertEquals(SensorLinkProtocol.Kind.SENSOR, r.kind);
    assertEquals(Arrays.asList("android.sensor.accelerometer"), r.types);
  }

  @Test
  public void parsesMultiSensorPathEncodedOrNot() {
    SensorLinkProtocol.Request plain =
        SensorLinkProtocol.parse(
            "/sensors/connect?types=[\"android.sensor.accelerometer\",\"android.sensor.gyroscope\"]");
    assertEquals(SensorLinkProtocol.Kind.SENSORS, plain.kind);
    assertEquals(
        Arrays.asList("android.sensor.accelerometer", "android.sensor.gyroscope"), plain.types);

    SensorLinkProtocol.Request encoded =
        SensorLinkProtocol.parse(
            "/sensors/connect?types=%5B%22android.sensor.accelerometer%22%2C%20%22android.sensor.gyroscope%22%5D");
    assertEquals(plain.types, encoded.types);
  }

  @Test
  public void parsesGpsAndRejectsUnknown() {
    assertEquals(SensorLinkProtocol.Kind.GPS, SensorLinkProtocol.parse("/gps").kind);
    assertEquals(SensorLinkProtocol.Kind.GPS, SensorLinkProtocol.parse("/gps/").kind);
    assertEquals(SensorLinkProtocol.Kind.UNKNOWN, SensorLinkProtocol.parse("/").kind);
    assertEquals(SensorLinkProtocol.Kind.UNKNOWN, SensorLinkProtocol.parse("/sensor/connect").kind);
    assertEquals(SensorLinkProtocol.Kind.UNKNOWN, SensorLinkProtocol.parse(null).kind);
  }

  @Test
  public void sensorMessageRoundTrips() {
    String json =
        SensorLinkProtocol.sensorJson(
            "android.sensor.gyroscope", new float[] {0.5f, -1.25f, 9.75f}, 123456789L, 3);
    assertEquals(
        "{\"type\":\"android.sensor.gyroscope\",\"values\":[0.5,-1.25,9.75],\"timestamp\":123456789,\"accuracy\":3}",
        json);
    SensorLinkProtocol.SensorReading r = SensorLinkProtocol.parseSensor(json);
    assertEquals("android.sensor.gyroscope", r.type);
    assertEquals(3, r.values.length);
    assertEquals(-1.25f, r.values[1], 0f);
    assertEquals(123456789L, r.timestampNs);
    assertEquals(3, r.accuracy);

    // SensorServer's own single-sensor message (no type).
    SensorLinkProtocol.SensorReading s =
        SensorLinkProtocol.parseSensor(
            "{\"accuracy\":2,\"timestamp\":3925657519043709,\"values\":[0.31892395,-0.97802734,10.049896]}");
    assertNull(s.type);
    assertEquals(10.049896f, s.values[2], 1e-6f);
    assertEquals(3925657519043709L, s.timestampNs);
  }

  @Test
  public void gpsMessageRoundTrips() {
    SensorLinkProtocol.GpsFix f = new SensorLinkProtocol.GpsFix();
    f.latitude = 10.7769;
    f.longitude = 106.7009;
    f.altitude = 12.5;
    f.bearing = 90f;
    f.accuracy = 3.5f;
    f.speed = 1.2f;
    f.timeMs = 1761530000000L;
    SensorLinkProtocol.GpsFix g = SensorLinkProtocol.parseGps(SensorLinkProtocol.gpsJson(f));
    assertEquals(10.7769, g.latitude, 1e-9);
    assertEquals(106.7009, g.longitude, 1e-9);
    assertEquals(3.5f, g.accuracy, 0f);
    assertEquals(1761530000000L, g.timeMs);
    assertNull(SensorLinkProtocol.parseGps("{\"values\":[1,2,3]}"));
    assertNull(SensorLinkProtocol.parseGps("{\"latitude\":95,\"longitude\":0}"));
  }

  @Test
  public void normalisesAddresses() {
    assertEquals("ws://192.168.43.1:8080", SensorLinkProtocol.baseUrl("192.168.43.1"));
    assertEquals("ws://192.168.43.1:8081", SensorLinkProtocol.baseUrl(" ws://192.168.43.1:8081/ "));
    assertEquals("ws://phone.local:9000", SensorLinkProtocol.baseUrl("http://phone.local:9000/gps"));
    assertNull(SensorLinkProtocol.baseUrl(""));
    assertNull(SensorLinkProtocol.baseUrl(":8080"));
    assertTrue(SensorLinkProtocol.sensorPath("android.sensor.rotation_vector").startsWith("/sensor/connect?type="));
  }
}
