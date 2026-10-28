package org.openbot.app.robot.valetudo;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import org.junit.Test;
import org.openbot.app.robot.slam.CleaningMission;
import org.openbot.app.robot.slam.Json;
import org.openbot.app.robot.slam.OccupancyGrid;
import org.openbot.app.robot.slam.Pose;
import org.openbot.app.robot.slam.RoomMap;
import org.openbot.app.robot.slam.RoomSegmenter;

/** The Valetudo REST API and MQTT topics, against a fake robot on a two-room map. */
public class ValetudoApiTest {

  static final class FakeRobot implements ValetudoApi.Robot {
    final OccupancyGrid grid = twoRooms();
    final RoomMap rooms = new RoomMap();
    Pose pose = new Pose(2, 2, 0);
    String lastAction;
    double[] lastGoTo;
    List<Integer> lastSegments;
    List<double[]> lastZones;

    FakeRobot() {
      rooms.setRooms(RoomSegmenter.segment(grid, new RoomSegmenter.Params()), RoomMap.stampOf(grid));
    }

    @Override public OccupancyGrid grid() { return grid; }
    @Override public RoomMap rooms() { return rooms; }
    @Override public Pose pose() { return pose; }
    @Override public CleaningMission.Status status() { return CleaningMission.Status.IDLE; }
    @Override public CleaningMission.Flag flag() { return CleaningMission.Flag.NONE; }
    @Override public String error() { return ""; }
    @Override public List<double[]> plannedPath() { return new ArrayList<>(); }
    @Override public double[] goToTarget() { return null; }
    @Override public List<double[]> activeZones() { return new ArrayList<>(); }
    @Override public String basicControl(String action) { lastAction = action; return null; }
    @Override public String goTo(double x, double y) { lastGoTo = new double[] {x, y}; return null; }
    @Override public String cleanSegments(List<Integer> ids) { lastSegments = ids; return null; }
    @Override public String cleanZones(List<double[]> zones) { lastZones = zones; return null; }
    @Override public String renameSegment(int id, String name) { rooms.names.put(id, name); return null; }
    @Override public String setRestrictions(List<double[]> walls, List<double[]> noGo) {
      rooms.walls.clear();
      rooms.walls.addAll(walls);
      rooms.noGo.clear();
      rooms.noGo.addAll(noGo);
      return null;
    }
    @Override public String locate() { lastAction = "locate"; return null; }
  }

  static OccupancyGrid twoRooms() {
    OccupancyGrid g = OccupancyGrid.indoorDefault();
    for (int cy = g.cellY(0); cy <= g.cellY(4); cy++)
      for (int cx = g.cellX(0); cx <= g.cellX(7); cx++) g.set(cx, cy, -2f);
    double[][] walls = {{0, 0, 7, 0}, {7, 0, 7, 4}, {7, 4, 0, 4}, {0, 4, 0, 0}, {4, 0, 4, 1.5}, {4, 2.5, 4, 4}};
    for (double[] s : walls) {
      int n = (int) (Math.hypot(s[2] - s[0], s[3] - s[1]) / 0.02) + 1;
      for (int k = 0; k <= n; k++) {
        double t = (double) k / n;
        g.set(g.cellX(s[0] + (s[2] - s[0]) * t), g.cellY(s[1] + (s[3] - s[1]) * t), 3f);
      }
    }
    return g;
  }

  @Test
  public void robotStateAndCapabilities() {
    ValetudoApi api = new ValetudoApi(new FakeRobot());
    ValetudoApi.Response r = api.handle("GET", "/api/v2/robot", null);
    assertEquals(200, r.status);
    assertEquals("OpenBot", Json.str(Json.get(Json.parse(r.body), "manufacturer")));
    List<Object> attrs = Json.arr(Json.parse(api.handle("GET", "/api/v2/robot/state/attributes", null).body));
    assertEquals("StatusStateAttribute", Json.str(Json.get(attrs.get(0), "__class")));
    assertEquals("idle", Json.str(Json.get(attrs.get(0), "value")));
    List<Object> caps = Json.arr(Json.parse(api.handle("GET", "/api/v2/robot/capabilities/", null).body));
    assertTrue(caps.contains("MapSegmentationCapability"));
    assertEquals(404, api.handle("GET", "/api/v2/robot/capabilities/MopDockCleanManualTriggerCapability", null).status);
    assertEquals(405, api.handle("DELETE", "/api/v2/robot", null).status);
  }

  @Test
  public void mapHasSegmentsAndTheRobotInValetudoCoordinates() {
    FakeRobot robot = new FakeRobot();
    robot.rooms.names.put(2, "Kitchen");
    ValetudoApi api = new ValetudoApi(robot);
    Map<String, Object> map = Json.obj(Json.parse(api.handle("GET", "/api/v2/robot/state/map", null).body));
    assertEquals("ValetudoMap", map.get("__class"));
    assertEquals(5.0, Json.num(map.get("pixelSize"), 0), 1e-9);
    List<Object> layers = Json.arr(map.get("layers"));
    int segments = 0;
    boolean kitchen = false, wall = false;
    for (Object l : layers) {
      String type = Json.str(Json.get(l, "type"));
      if ("wall".equals(type)) wall = true;
      if ("segment".equals(type)) {
        segments++;
        if ("Kitchen".equals(Json.str(Json.get(l, "metaData", "name")))) kitchen = true;
        List<Object> runs = Json.arr(Json.get(l, "compressedPixels"));
        assertEquals(0, runs.size() % 3);
        double count = 0;
        for (int i = 2; i < runs.size(); i += 3) count += Json.num(runs.get(i), 0);
        assertEquals(count, Json.num(Json.get(l, "dimensions", "pixelCount"), 0), 1e-9);
      }
    }
    assertTrue(wall);
    assertEquals(2, segments);
    assertTrue(kitchen);
    // Robot at world (2, 2): grid origin (-15, -15), 30 m tall, so (1700, 1300) cm.
    for (Object e : Json.arr(map.get("entities"))) {
      if ("robot_position".equals(Json.str(Json.get(e, "type")))) {
        List<Object> pts = Json.arr(Json.get(e, "points"));
        assertEquals(1700, Json.num(pts.get(0), 0), 1e-9);
        assertEquals(1300, Json.num(pts.get(1), 0), 1e-9);
        assertEquals(90, Json.num(Json.get(e, "metaData", "angle"), 0), 1e-9); // facing +x = right
        return;
      }
    }
    throw new AssertionError("no robot_position");
  }

  @Test
  public void capabilitiesCallTheRobot() {
    FakeRobot robot = new FakeRobot();
    ValetudoApi api = new ValetudoApi(robot);
    assertEquals(200, api.handle("PUT", "/api/v2/robot/capabilities/BasicControlCapability", "{\"action\":\"home\"}").status);
    assertEquals("home", robot.lastAction);
    assertEquals(400, api.handle("PUT", "/api/v2/robot/capabilities/BasicControlCapability", "{\"action\":\"fly\"}").status);
    api.handle("PUT", "/api/v2/robot/capabilities/GoToLocationCapability",
        "{\"action\":\"goto\",\"coordinates\":{\"x\":2050,\"y\":1300}}");
    assertArrayEquals(new double[] {5.5, 2.0}, robot.lastGoTo, 1e-9);
    api.handle("PUT", "/api/v2/robot/capabilities/MapSegmentationCapability",
        "{\"action\":\"start_segment_action\",\"segment_ids\":[\"2\",\"1\"],\"iterations\":1,\"customOrder\":true}");
    assertEquals(2, (int) robot.lastSegments.get(0));
    api.handle("PUT", "/api/v2/robot/capabilities/MapSegmentRenameCapability",
        "{\"action\":\"rename_segment\",\"segment_id\":\"1\",\"name\":\"Living room\"}");
    List<Object> segs = Json.arr(Json.parse(api.handle("GET", "/api/v2/robot/capabilities/MapSegmentationCapability", null).body));
    assertEquals("Living room", Json.str(Json.get(segs.get(0), "name")));
    assertEquals("1", Json.str(Json.get(segs.get(0), "id")));
    api.handle("PUT", "/api/v2/robot/capabilities/ZoneCleaningCapability",
        "{\"action\":\"clean\",\"zones\":[{\"points\":{\"pA\":{\"x\":1550,\"y\":1450},\"pB\":{\"x\":1650,\"y\":1450},"
            + "\"pC\":{\"x\":1650,\"y\":1350},\"pD\":{\"x\":1550,\"y\":1350}}}],\"iterations\":1}");
    assertEquals(1, robot.lastZones.size());
    assertEquals(0.5, robot.lastZones.get(0)[0], 1e-9);
    assertEquals(0.5, robot.lastZones.get(0)[1], 1e-9);
    assertEquals(400, api.handle("PUT", "/api/v2/robot/capabilities/ZoneCleaningCapability",
        "{\"action\":\"clean\",\"zones\":[{\"nope\":1}]}").status);
    assertEquals(200, api.handle("PUT", "/api/v2/robot/capabilities/LocateCapability", "{\"action\":\"locate\"}").status);
    assertEquals("locate", robot.lastAction);
  }

  @Test
  public void virtualRestrictionsRoundTrip() {
    FakeRobot robot = new FakeRobot();
    ValetudoApi api = new ValetudoApi(robot);
    String body = "{\"virtualWalls\":[{\"points\":{\"pA\":{\"x\":1900,\"y\":1350},\"pB\":{\"x\":1900,\"y\":1250}}}],"
        + "\"restrictedZones\":[{\"points\":{\"pA\":{\"x\":2000,\"y\":1200},\"pB\":{\"x\":2100,\"y\":1200},"
        + "\"pC\":{\"x\":2100,\"y\":1150},\"pD\":{\"x\":2000,\"y\":1150}},\"type\":\"regular\"}]}";
    assertEquals(200, api.handle("PUT", "/api/v2/robot/capabilities/CombinedVirtualRestrictionsCapability", body).status);
    assertEquals(1, robot.rooms.walls.size());
    assertEquals(4.0, robot.rooms.walls.get(0)[0], 1e-9);
    assertEquals(1.5, robot.rooms.walls.get(0)[1], 1e-9);
    assertTrue(robot.rooms.blocked(5.5, 3.2));
    Object got = Json.parse(api.handle("GET", "/api/v2/robot/capabilities/CombinedVirtualRestrictionsCapability", null).body);
    Object wall = Json.arr(Json.get(got, "virtualWalls")).get(0);
    assertEquals(1900, Json.num(Json.get(wall, "points", "pA", "x"), 0), 1e-9);
    assertEquals(1250, Json.num(Json.get(wall, "points", "pB", "y"), 0), 1e-9);
    assertEquals("regular", Json.str(Json.get(Json.arr(Json.get(got, "restrictedZones")).get(0), "type")));
  }

  @Test
  public void mqttPacketsAndTopics() {
    assertArrayEquals(new byte[] {0x00}, MqttLite.remainingLength(0));
    assertArrayEquals(new byte[] {(byte) 0xC1, 0x02}, MqttLite.remainingLength(321));
    MqttLite.Config c = new MqttLite.Config();
    c.clientId = "ob";
    c.keepAliveSeconds = 30;
    byte[] p = MqttLite.connectPacket(c);
    assertArrayEquals(new byte[] {0x10, 14, 0, 4, 'M', 'Q', 'T', 'T', 4, 2, 0, 30, 0, 2, 'o', 'b'}, p);
    byte[] pub = MqttLite.publishPacket("a/b", "hi", true);
    assertArrayEquals(new byte[] {0x31, 7, 0, 3, 'a', '/', 'b', 'h', 'i'}, pub);
    byte[] sub = MqttLite.subscribePacket(1, "x/#");
    assertArrayEquals(new byte[] {(byte) 0x82, 8, 0, 1, 0, 3, 'x', '/', '#', 0}, sub);

    FakeRobot robot = new FakeRobot();
    ValetudoApi api = new ValetudoApi(robot);
    ValetudoMqtt.Settings s = new ValetudoMqtt.Settings();
    ValetudoMqtt m = new ValetudoMqtt(robot, api, s, null);
    assertEquals("valetudo/openbot", m.getBase());
    assertNull(m.handle("BasicControlCapability/operation", "START"));
    assertEquals("start", robot.lastAction);
    assertNull(m.handle("LocateCapability/locate", "PERFORM"));
    assertEquals("locate", robot.lastAction);
    assertNull(m.handle("GoToLocationCapability/go", "{\"coordinates\":{\"x\":1700,\"y\":1300}}"));
    assertArrayEquals(new double[] {2, 2}, robot.lastGoTo, 1e-9);
    assertNull(m.handle("MapSegmentationCapability/clean", "{\"segment_ids\":[\"2\"]}"));
    assertEquals(2, (int) robot.lastSegments.get(0));
    assertNotNull(m.handle("BasicControlCapability/operation", "FLY"));
    assertEquals("cleaning", ValetudoMqtt.hassState(CleaningMission.Status.MOVING));
    assertEquals("a_b", ValetudoMqtt.topicSafe("a/b"));
  }
}
