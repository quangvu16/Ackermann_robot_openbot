package org.openbot.app.robot.sensorlink;

import java.io.UnsupportedEncodingException;
import java.net.URLDecoder;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * The WebSocket API of the SensorServer app (https://github.com/UmerCodez/SensorServer), written
 * from its documentation so OpenBot can talk to SensorServer clients and servers. No SensorServer
 * code is used (that project is GPL-3.0; this one is MIT).
 *
 * <pre>
 *   ws://host:port/sensor/connect?type=android.sensor.accelerometer      one sensor
 *   ws://host:port/sensors/connect?types=["android.sensor.gyroscope",...] several sensors
 *   ws://host:port/gps                                                    location
 *
 *   sensor message  {"type":"android.sensor.gyroscope","values":[x,y,z],"timestamp":ns,"accuracy":3}
 *   gps message     {"longitude":..,"latitude":..,"altitude":..,"bearing":..,"accuracy":..,
 *                    "speed":..,"time":ms}
 * </pre>
 *
 * Pure Java (no Android classes), so it runs in plain unit tests.
 */
public final class SensorLinkProtocol {

  private SensorLinkProtocol() {}

  public static final int DEFAULT_PORT = 8080;

  /** Text a client may send on /gps to get the last fix right away. */
  public static final String GET_LAST_KNOWN_LOCATION = "getLastKnownLocation";

  public static final String TYPE_ROTATION_VECTOR = "android.sensor.rotation_vector";

  public enum Kind {
    SENSOR,
    SENSORS,
    GPS,
    UNKNOWN
  }

  /** A parsed connection request: which endpoint, and which sensor types it asked for. */
  public static final class Request {
    public final Kind kind;
    public final List<String> types;

    Request(Kind kind, List<String> types) {
      this.kind = kind;
      this.types = Collections.unmodifiableList(types);
    }
  }

  /** Parses the request path, e.g. {@code /sensors/connect?types=%5B%22a%22%5D}. */
  public static Request parse(String resourceDescriptor) {
    if (resourceDescriptor == null) {
      return new Request(Kind.UNKNOWN, new ArrayList<>());
    }
    String path = resourceDescriptor;
    String query = "";
    int q = path.indexOf('?');
    if (q >= 0) {
      query = path.substring(q + 1);
      path = path.substring(0, q);
    }
    while (path.length() > 1 && path.endsWith("/")) {
      path = path.substring(0, path.length() - 1);
    }
    switch (path) {
      case "/gps":
        return new Request(Kind.GPS, new ArrayList<>());
      case "/sensor/connect":
        {
          String type = queryParam(query, "type");
          List<String> types = new ArrayList<>();
          if (type != null && !type.trim().isEmpty()) types.add(type.trim());
          return new Request(types.isEmpty() ? Kind.UNKNOWN : Kind.SENSOR, types);
        }
      case "/sensors/connect":
        {
          List<String> types = parseTypeArray(queryParam(query, "types"));
          return new Request(types.isEmpty() ? Kind.UNKNOWN : Kind.SENSORS, types);
        }
      default:
        return new Request(Kind.UNKNOWN, new ArrayList<>());
    }
  }

  static String queryParam(String query, String name) {
    for (String pair : query.split("&")) {
      int eq = pair.indexOf('=');
      String key = eq >= 0 ? pair.substring(0, eq) : pair;
      if (decode(key).equals(name)) {
        return eq >= 0 ? decode(pair.substring(eq + 1)) : "";
      }
    }
    return null;
  }

  private static String decode(String s) {
    try {
      // URLDecoder turns '+' into a space; the JSON array never contains either on purpose.
      return URLDecoder.decode(s, "UTF-8");
    } catch (UnsupportedEncodingException | IllegalArgumentException e) {
      return s;
    }
  }

  /** {@code ["a","b"]} (or a bare {@code a,b}) to a list of type names. */
  static List<String> parseTypeArray(String raw) {
    List<String> out = new ArrayList<>();
    if (raw == null) return out;
    String s = raw.trim();
    if (s.startsWith("[")) s = s.substring(1);
    if (s.endsWith("]")) s = s.substring(0, s.length() - 1);
    for (String part : s.split(",")) {
      String t = part.trim();
      if (t.length() >= 2 && t.startsWith("\"") && t.endsWith("\"")) {
        t = t.substring(1, t.length() - 1).trim();
      }
      if (!t.isEmpty() && !out.contains(t)) out.add(t);
    }
    return out;
  }

  // region Building messages (server side)

  /** One sensor event. {@code type} is included when not null (always, on /sensors/connect). */
  public static String sensorJson(String type, float[] values, long timestampNs, int accuracy) {
    StringBuilder b = new StringBuilder(96);
    b.append('{');
    if (type != null) {
      b.append("\"type\":\"").append(escape(type)).append("\",");
    }
    b.append("\"values\":[");
    for (int i = 0; i < values.length; i++) {
      if (i > 0) b.append(',');
      b.append(number(values[i]));
    }
    b.append("],\"timestamp\":").append(timestampNs);
    b.append(",\"accuracy\":").append(accuracy).append('}');
    return b.toString();
  }

  /** A location fix, with the keys SensorServer uses on /gps. */
  public static String gpsJson(GpsFix f) {
    StringBuilder b = new StringBuilder(160);
    b.append("{\"longitude\":").append(f.longitude);
    b.append(",\"latitude\":").append(f.latitude);
    b.append(",\"altitude\":").append(f.altitude);
    b.append(",\"bearing\":").append(number(f.bearing));
    b.append(",\"accuracy\":").append(number(f.accuracy));
    b.append(",\"speed\":").append(number(f.speed));
    b.append(",\"time\":").append(f.timeMs);
    b.append('}');
    return b.toString();
  }

  private static String number(float v) {
    if (Float.isNaN(v) || Float.isInfinite(v)) return "0";
    return Float.toString(v);
  }

  private static String escape(String s) {
    return escapeJson(s);
  }

  /** Escapes a string for use inside JSON double quotes. */
  public static String escapeJson(String s) {
    if (s == null) return "";
    StringBuilder b = new StringBuilder(s.length() + 8);
    for (int i = 0; i < s.length(); i++) {
      char c = s.charAt(i);
      if (c == '"' || c == '\\') b.append('\\').append(c);
      else if (c < 0x20) b.append(String.format("\\u%04x", (int) c));
      else b.append(c);
    }
    return b.toString();
  }

  // endregion

  /** Text a client may connect to for LiDAR scans (an OpenBot addition to the SensorServer API). */
  public static final String LIDAR_PATH = "/lidar";

  /** Indoor SLAM state and commands (see SlamWire). */
  public static final String SLAM_PATH = "/slam";

  /**
   * One LiDAR revolution for the browser:
   *
   * <pre>
   *   {"scanId":42,"bins":360,"forwardOffset":0,"age":35,"ranges":[mm,...],
   *    "sectors":[mm x8],"sectorsAge":20}
   * </pre>
   *
   * Bin {@code i} is {@code (i + 0.5) * 360 / bins} degrees clockwise from the LiDAR's zero mark;
   * subtract {@code forwardOffset} for degrees from the car's nose. Ranges and sectors are
   * millimetres with 0 meaning no return / clear, as on the firmware's wire. {@code ranges} is
   * empty when only the car's 8 proximity sectors are known; {@code sectors} is null when the car
   * has not reported any. Ages are milliseconds since the data arrived.
   */
  public static String lidarJson(
      long scanId, int[] rangesMm, float forwardOffsetDeg, long ageMs, int[] sectorsMm, long sectorsAgeMs) {
    StringBuilder b = new StringBuilder(32 + (rangesMm == null ? 0 : rangesMm.length * 5));
    b.append("{\"scanId\":").append(scanId);
    b.append(",\"bins\":").append(rangesMm == null ? 0 : rangesMm.length);
    b.append(",\"forwardOffset\":").append(number(forwardOffsetDeg));
    b.append(",\"age\":").append(ageMs);
    b.append(",\"ranges\":[");
    if (rangesMm != null) {
      for (int i = 0; i < rangesMm.length; i++) {
        if (i > 0) b.append(',');
        b.append(rangesMm[i]);
      }
    }
    b.append("],\"sectors\":");
    if (sectorsMm == null) {
      b.append("null");
    } else {
      b.append('[');
      for (int i = 0; i < sectorsMm.length; i++) {
        if (i > 0) b.append(',');
        b.append(sectorsMm[i]);
      }
      b.append(']');
    }
    b.append(",\"sectorsAge\":").append(sectorsAgeMs).append('}');
    return b.toString();
  }

  // region Reading messages (client side)

  /** A location fix as carried on /gps. Unknown values are NaN (accuracy, bearing, speed). */
  public static final class GpsFix {
    public double latitude;
    public double longitude;
    public double altitude;
    public float bearing = Float.NaN;
    public float accuracy = Float.NaN;
    public float speed = Float.NaN;
    public long timeMs;
  }

  /** One reading from /sensor/connect or /sensors/connect. */
  public static final class SensorReading {
    public String type; // null when the server did not say (single-sensor endpoint)
    public float[] values;
    public long timestampNs;
    public int accuracy;
  }

  private static final Pattern VALUES = Pattern.compile("\"values\"\\s*:\\s*\\[([^\\]]*)\\]");
  private static final Pattern TYPE = Pattern.compile("\"type\"\\s*:\\s*\"([^\"]*)\"");

  /** Null if the text is not a GPS message (it must at least carry latitude and longitude). */
  public static GpsFix parseGps(String json) {
    if (json == null) return null;
    Double lat = numberField(json, "latitude");
    Double lon = numberField(json, "longitude");
    if (lat == null || lon == null || lat < -90 || lat > 90 || lon < -180 || lon > 180) {
      return null;
    }
    GpsFix f = new GpsFix();
    f.latitude = lat;
    f.longitude = lon;
    Double v;
    if ((v = numberField(json, "altitude")) != null) f.altitude = v;
    if ((v = numberField(json, "bearing")) != null) f.bearing = v.floatValue();
    if ((v = numberField(json, "accuracy")) != null) f.accuracy = v.floatValue();
    if ((v = numberField(json, "speed")) != null) f.speed = v.floatValue();
    if ((v = numberField(json, "time")) != null) f.timeMs = v.longValue();
    return f;
  }

  /** Null if the text is not a sensor message. */
  public static SensorReading parseSensor(String json) {
    if (json == null) return null;
    Matcher m = VALUES.matcher(json);
    if (!m.find()) return null;
    String inner = m.group(1).trim();
    String[] parts = inner.isEmpty() ? new String[0] : inner.split(",");
    float[] values = new float[parts.length];
    try {
      for (int i = 0; i < parts.length; i++) values[i] = Float.parseFloat(parts[i].trim());
    } catch (NumberFormatException e) {
      return null;
    }
    SensorReading r = new SensorReading();
    r.values = values;
    Matcher t = TYPE.matcher(json);
    r.type = t.find() ? t.group(1) : null;
    Double v;
    if ((v = numberField(json, "timestamp")) != null) r.timestampNs = v.longValue();
    if ((v = numberField(json, "accuracy")) != null) r.accuracy = v.intValue();
    return r;
  }

  static Double numberField(String json, String key) {
    Matcher m =
        Pattern.compile("\"" + Pattern.quote(key) + "\"\\s*:\\s*(-?[0-9][0-9.eE+-]*)").matcher(json);
    if (!m.find()) return null;
    try {
      return Double.parseDouble(m.group(1));
    } catch (NumberFormatException e) {
      return null;
    }
  }

  // endregion

  /**
   * Turns what a person types ({@code 192.168.43.1}, {@code 192.168.43.1:8080}, {@code
   * ws://host:port/}) into {@code ws://host:port}, or null if there is no host.
   */
  public static String baseUrl(String address) {
    if (address == null) return null;
    String a = address.trim();
    String lower = a.toLowerCase(Locale.ROOT);
    if (lower.startsWith("ws://")) a = a.substring(5);
    else if (lower.startsWith("http://")) a = a.substring(7);
    int slash = a.indexOf('/');
    if (slash >= 0) a = a.substring(0, slash);
    if (a.isEmpty() || a.startsWith(":")) return null;
    // An IPv6 literal must be in brackets; a bare host gets the default port.
    boolean hasPort = a.startsWith("[") ? a.contains("]:") : a.indexOf(':') >= 0;
    if (!hasPort) a = a + ":" + DEFAULT_PORT;
    return "ws://" + a;
  }

  /** {@code /sensor/connect?type=...} with the type URL-encoded. */
  public static String sensorPath(String type) {
    return "/sensor/connect?type=" + type.replace(" ", "%20");
  }
}
