package org.openbot.app.robot.valetudo;

import java.util.Locale;
import org.openbot.app.robot.slam.CleaningMission;
import org.openbot.app.robot.slam.Json;
import org.openbot.app.robot.slam.Pose;
import org.openbot.app.robot.slam.RoomMap;

/**
 * Valetudo's MQTT topics for this car (github.com/Hypfer/Valetudo, backend/lib/mqtt; Homie layout
 * {@code <prefix>/<identifier>/<node>/<property>}, commands on {@code .../set}), plus Home
 * Assistant MQTT discovery for a vacuum entity and a "current room" sensor.
 *
 * <pre>
 * valetudo/openbot/$state                                   ready | lost (last will)
 * valetudo/openbot/StatusStateAttribute/status              idle, cleaning, moving, returning, paused, docked, error
 * valetudo/openbot/StatusStateAttribute/flag                none, segment, zone, target, resumable
 * valetudo/openbot/StatusStateAttribute/error_description
 * valetudo/openbot/MapData/segments                         {"1":"Kitchen",...}
 * valetudo/openbot/OpenBot/room                             current room name (OpenBot extra)
 * valetudo/openbot/OpenBot/position                         {"x":cm,"y":cm,"angle":deg} (OpenBot extra)
 * commands:
 * valetudo/openbot/BasicControlCapability/operation/set     START | STOP | PAUSE | HOME (| LOCATE)
 * valetudo/openbot/GoToLocationCapability/go/set            {"coordinates":{"x":cm,"y":cm}}
 * valetudo/openbot/MapSegmentationCapability/clean/set      {"segment_ids":["1","2"]}
 * valetudo/openbot/ZoneCleaningCapability/start/set         {"zones":[{"points":{...}}]}
 * valetudo/openbot/LocateCapability/locate/set              PERFORM
 * </pre>
 */
public final class ValetudoMqtt implements MqttLite.Listener {

  public static final class Settings {
    public boolean enabled;
    public String host = "";
    public int port = 1883;
    public String username = "";
    public String password = "";
    public String prefix = "valetudo";
    public String identifier = "openbot";
    public boolean homeAssistant = true;
  }

  public interface StatusListener {
    void onMqttStatus(String text);
  }

  private final ValetudoApi.Robot robot;
  private final ValetudoApi api;
  private final Settings settings;
  private final StatusListener statusListener;
  private final String base;
  private MqttLite client;
  private volatile String lastStatus = "";
  // Last values sent, so only changes are published.
  private String sentStatus, sentFlag, sentError, sentSegments, sentRoom, sentPosition, sentHass;

  public ValetudoMqtt(ValetudoApi.Robot robot, ValetudoApi api, Settings s, StatusListener l) {
    this.robot = robot;
    this.api = api;
    this.settings = s;
    this.statusListener = l;
    this.base = topicSafe(s.prefix) + "/" + topicSafe(s.identifier);
  }

  static String topicSafe(String s) {
    String t = s == null ? "" : s.trim().replaceAll("[#+\\s/]+", "_");
    return t.isEmpty() ? "openbot" : t;
  }

  public String getBase() {
    return base;
  }

  public String getStatus() {
    return lastStatus;
  }

  public void start() {
    if (!settings.enabled || settings.host.trim().isEmpty()) return;
    MqttLite.Config c = new MqttLite.Config();
    c.host = settings.host.trim();
    c.port = settings.port;
    c.username = settings.username;
    c.password = settings.password;
    c.clientId = "openbot-" + topicSafe(settings.identifier);
    c.willTopic = base + "/$state";
    c.willPayload = "lost";
    c.subscriptions.add(base + "/+/+/set");
    client = new MqttLite(c, this);
    client.start();
  }

  public void stop() {
    MqttLite c = client;
    if (c != null) {
      c.publish(base + "/$state", "disconnected", true);
      c.stop();
    }
    client = null;
  }

  public boolean isConnected() {
    MqttLite c = client;
    return c != null && c.isConnected();
  }

  // region Publishing

  @Override
  public void onConnected() {
    sentStatus = sentFlag = sentError = sentSegments = sentRoom = sentPosition = sentHass = null;
    MqttLite c = client;
    if (c == null) return;
    c.publish(base + "/$homie", "4.0.0", true);
    c.publish(base + "/$name", "OpenBot " + settings.identifier, true);
    c.publish(base + "/$state", "ready", true);
    if (settings.homeAssistant) publishDiscovery(c);
    publishState();
  }

  /** Call about once a second; sends only what changed. */
  public void publishState() {
    MqttLite c = client;
    if (c == null || !c.isConnected()) return;
    CleaningMission.Status st = robot.status();
    sentStatus = send(c, "/StatusStateAttribute/status", st.valetudo, sentStatus);
    sentFlag = send(c, "/StatusStateAttribute/flag", robot.flag().valetudo, sentFlag);
    sentError = send(c, "/StatusStateAttribute/error_description",
        st == CleaningMission.Status.ERROR ? robot.error() : "No error", sentError);
    RoomMap rooms = robot.rooms();
    StringBuilder seg = new StringBuilder("{");
    boolean first = true;
    for (java.util.Map.Entry<Integer, String> e : rooms.names.entrySet()) {
      if (!first) seg.append(',');
      first = false;
      seg.append('"').append(e.getKey()).append("\":").append(Json.quote(e.getValue()));
    }
    sentSegments = send(c, "/MapData/segments", seg.append('}').toString(), sentSegments);
    Pose p = robot.pose();
    int room = p == null ? 0 : rooms.roomAt(p.x, p.y);
    sentRoom = send(c, "/OpenBot/room", room > 0 ? rooms.name(room) : "", sentRoom);
    if (p != null && robot.grid() != null) {
      String pos = String.format(Locale.ROOT, "{\"x\":%d,\"y\":%d,\"angle\":%.0f}",
          ValetudoApi.vx(robot.grid(), p.x), ValetudoApi.vy(robot.grid(), p.y),
          ((90 - Math.toDegrees(p.theta)) % 360 + 360) % 360);
      sentPosition = send(c, "/OpenBot/position", pos, sentPosition);
    }
    if (settings.homeAssistant) {
      sentHass = send(c, "/hass/" + hassId() + "_vacuum/state", "{\"state\":\"" + hassState(st) + "\"}", sentHass);
    }
  }

  private String send(MqttLite c, String sub, String value, String last) {
    if (value.equals(last)) return last;
    c.publish(base + sub, value, true);
    return value;
  }

  private String hassId() {
    return topicSafe(settings.identifier);
  }

  /** Home Assistant vacuum states: cleaning, docked, paused, idle, returning, error. */
  static String hassState(CleaningMission.Status s) {
    switch (s) {
      case CLEANING:
      case MOVING:
        return "cleaning";
      case DOCKED:
        return "docked";
      case PAUSED:
        return "paused";
      case RETURNING:
        return "returning";
      case ERROR:
        return "error";
      default:
        return "idle";
    }
  }

  private void publishDiscovery(MqttLite c) {
    String id = hassId();
    String device = "{\"identifiers\":[\"openbot_" + id + "\"],\"name\":\"OpenBot " + id
        + "\",\"manufacturer\":\"OpenBot\",\"model\":\"Ackermann RC_S3\"}";
    String availability = "\"availability_topic\":\"" + base + "/$state\",\"payload_available\":\"ready\","
        + "\"payload_not_available\":\"lost\"";
    c.publish("homeassistant/vacuum/" + id + "/" + id + "_vacuum/config",
        "{\"name\":null,\"unique_id\":\"" + id + "_vacuum\",\"device\":" + device + "," + availability
            + ",\"command_topic\":\"" + base + "/BasicControlCapability/operation/set\""
            + ",\"payload_start\":\"START\",\"payload_stop\":\"STOP\",\"payload_pause\":\"PAUSE\""
            + ",\"payload_return_to_base\":\"HOME\",\"payload_locate\":\"LOCATE\""
            + ",\"state_topic\":\"" + base + "/hass/" + id + "_vacuum/state\""
            + ",\"send_command_topic\":\"" + base + "/MapSegmentationCapability/clean/set\""
            + ",\"supported_features\":[\"start\",\"stop\",\"pause\",\"return_home\",\"locate\",\"send_command\"]}",
        true);
    c.publish("homeassistant/sensor/" + id + "/" + id + "_room/config",
        "{\"name\":\"Current room\",\"unique_id\":\"" + id + "_room\",\"device\":" + device + "," + availability
            + ",\"state_topic\":\"" + base + "/OpenBot/room\",\"icon\":\"mdi:floor-plan\"}",
        true);
    c.publish("homeassistant/sensor/" + id + "/" + id + "_flag/config",
        "{\"name\":\"Status flag\",\"unique_id\":\"" + id + "_flag\",\"device\":" + device + "," + availability
            + ",\"state_topic\":\"" + base + "/StatusStateAttribute/flag\",\"icon\":\"mdi:flag\"}",
        true);
  }

  // endregion

  // region Commands

  @Override
  public void onMessage(String topic, String payload) {
    if (!topic.startsWith(base + "/") || !topic.endsWith("/set")) return;
    String what = topic.substring(base.length() + 1, topic.length() - "/set".length());
    String p = payload == null ? "" : payload.trim();
    String result = handle(what, p);
    if (result != null) onStatus("MQTT " + what + ": " + result);
  }

  /** Returns null when done, or why not. Package-private for tests. */
  String handle(String what, String p) {
    switch (what) {
      case "BasicControlCapability/operation": {
        String v = p.toUpperCase(Locale.ROOT);
        if (v.equals("LOCATE")) return robot.locate();
        if (!v.equals("START") && !v.equals("STOP") && !v.equals("PAUSE") && !v.equals("HOME")) return "unknown " + p;
        return robot.basicControl(v.toLowerCase(Locale.ROOT));
      }
      case "LocateCapability/locate":
        return "PERFORM".equalsIgnoreCase(p) ? robot.locate() : "expected PERFORM";
      case "GoToLocationCapability/go":
        return viaApi("GoToLocationCapability", "goto", p);
      case "MapSegmentationCapability/clean":
        return viaApi("MapSegmentationCapability", "start_segment_action", p);
      case "ZoneCleaningCapability/start":
        return viaApi("ZoneCleaningCapability", "clean", p);
      default:
        return "not supported";
    }
  }

  /** Same JSON as the REST interface, with the action added. */
  private String viaApi(String cap, String action, String json) {
    if (!json.startsWith("{")) return "payload must be JSON";
    String body = "{\"action\":\"" + action + "\"" + (json.trim().equals("{}") ? "}" : "," + json.substring(1));
    ValetudoApi.Response r = api.handle("PUT", "/api/v2/robot/capabilities/" + cap, body);
    return r.status == 200 ? null : r.body;
  }

  @Override
  public void onStatus(String text) {
    lastStatus = text;
    if (statusListener != null) statusListener.onMqttStatus(text);
  }

  // endregion
}
