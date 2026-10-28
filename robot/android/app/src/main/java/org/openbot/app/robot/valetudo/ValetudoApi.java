package org.openbot.app.robot.valetudo;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import org.openbot.app.robot.slam.CleaningMission;
import org.openbot.app.robot.slam.Json;
import org.openbot.app.robot.slam.OccupancyGrid;
import org.openbot.app.robot.slam.Pose;
import org.openbot.app.robot.slam.RoomMap;

/**
 * The part of Valetudo's REST API (v2) that fits this car, served by the Sensor link's web server
 * at {@code http://<phone>:8080/api/v2/...} while Rooms &amp; Zones is open. Same paths, bodies and
 * answers as Valetudo (github.com/Hypfer/Valetudo, backend/lib/webserver), so scripts and Home
 * Assistant setups written for Valetudo work:
 *
 * <pre>
 * GET  /api/v2/robot                       manufacturer, model
 * GET  /api/v2/robot/state                 attributes + map
 * GET  /api/v2/robot/state/attributes      StatusStateAttribute (value, flag)
 * GET  /api/v2/robot/state/map             ValetudoMap: floor, wall, segment layers; entities
 * GET  /api/v2/robot/capabilities          the list below
 * PUT  .../BasicControlCapability          {"action":"start"|"stop"|"pause"|"home"}
 * PUT  .../GoToLocationCapability          {"action":"goto","coordinates":{"x":cm,"y":cm}}
 * GET  .../MapSegmentationCapability       segments (rooms) with names
 * PUT  .../MapSegmentationCapability       {"action":"start_segment_action","segment_ids":["1"]}
 * PUT  .../MapSegmentRenameCapability      {"action":"rename_segment","segment_id":"1","name":"Kitchen"}
 * PUT  .../ZoneCleaningCapability          {"action":"clean","zones":[{"points":{"pA":..,"pD":..}}]}
 * GET/PUT .../CombinedVirtualRestrictionsCapability  {"virtualWalls":[...],"restrictedZones":[...]}
 * PUT  .../LocateCapability                {"action":"locate"}
 * </pre>
 *
 * <b>Coordinates</b> are Valetudo's: centimetres from the map's top-left corner, y pointing down.
 * Map pixels are SLAM grid cells ({@code pixelSize} = cell size in cm). Plain Java, unit tested.
 */
public final class ValetudoApi {

  public static final String[] CAPABILITIES = {
    "BasicControlCapability",
    "CombinedVirtualRestrictionsCapability",
    "GoToLocationCapability",
    "LocateCapability",
    "MapSegmentRenameCapability",
    "MapSegmentationCapability",
    "ZoneCleaningCapability",
  };

  /** What the Rooms & Zones screen offers. Getters may be called from any thread. */
  public interface Robot {
    OccupancyGrid grid(); // null when no map is loaded

    RoomMap rooms();

    Pose pose();

    CleaningMission.Status status();

    CleaningMission.Flag flag();

    String error();

    List<double[]> plannedPath();

    double[] goToTarget();

    List<double[]> activeZones();

    /** Actions. Each returns null on success or a short reason it cannot be done. */
    String basicControl(String action);

    String goTo(double x, double y);

    String cleanSegments(List<Integer> ids);

    String cleanZones(List<double[]> zones);

    String renameSegment(int id, String name);

    String setRestrictions(List<double[]> walls, List<double[]> noGo);

    String locate();
  }

  public static final class Response {
    public final int status;
    public final String contentType;
    public final String body;

    Response(int status, String contentType, String body) {
      this.status = status;
      this.contentType = contentType;
      this.body = body;
    }

    static Response json(String body) {
      return new Response(200, "application/json", body);
    }

    static Response ok() {
      return new Response(200, "text/plain", "OK");
    }

    static Response error(int status, String text) {
      return new Response(status, "text/plain", text);
    }
  }

  private final Robot robot;

  public ValetudoApi(Robot robot) {
    this.robot = robot;
  }

  /** Answer a request; {@code path} may carry a query string. Never throws. */
  public Response handle(String method, String path, String body) {
    try {
      int q = path.indexOf('?');
      String p = q >= 0 ? path.substring(0, q) : path;
      while (p.length() > 1 && p.endsWith("/")) p = p.substring(0, p.length() - 1);
      boolean get = "GET".equals(method);
      boolean put = "PUT".equals(method) || "POST".equals(method);
      switch (p) {
        case "/api/v2/robot":
          return get ? Response.json("{\"manufacturer\":\"OpenBot\",\"modelName\":\"Ackermann RC_S3\","
              + "\"modelDetails\":{\"supportedAttachments\":[]},\"implementation\":\"OpenBotRoomsRobot\"}") : notAllowed();
        case "/api/v2/robot/properties":
          return get ? Response.json("{\"firmwareVersion\":\"openbot\"}") : notAllowed();
        case "/api/v2/robot/state":
          return get ? Response.json("{\"__class\":\"RobotState\",\"metaData\":{},\"attributes\":" + attributes()
              + ",\"map\":" + mapJson() + "}") : notAllowed();
        case "/api/v2/robot/state/attributes":
          return get ? Response.json(attributes()) : notAllowed();
        case "/api/v2/robot/state/map":
          return get ? Response.json(mapJson()) : notAllowed();
        case "/api/v2/robot/capabilities":
          return get ? Response.json(capabilityList()) : notAllowed();
        case "/api/v2/valetudo":
          return get ? Response.json("{\"embedded\":false,\"systemId\":\"openbot\",\"welcomeDialogDismissed\":true}") : notAllowed();
        case "/api/v2/valetudo/version":
          return get ? Response.json("{\"release\":\"openbot-rooms\",\"commit\":\"\"}") : notAllowed();
        default:
          break;
      }
      String prefix = "/api/v2/robot/capabilities/";
      if (!p.startsWith(prefix)) return Response.error(404, "Not found");
      String rest = p.substring(prefix.length());
      boolean props = rest.endsWith("/properties");
      String cap = props ? rest.substring(0, rest.length() - "/properties".length()) : rest;
      if (!java.util.Arrays.asList(CAPABILITIES).contains(cap)) return Response.error(404, "Capability not supported");
      if (props) return get ? Response.json(properties(cap)) : notAllowed();
      if (get) return capabilityGet(cap);
      if (put) return capabilityPut(cap, Json.obj(Json.tryParse(body)));
      return notAllowed();
    } catch (RuntimeException e) {
      return Response.error(500, String.valueOf(e.getMessage()));
    }
  }

  private static Response notAllowed() {
    return Response.error(405, "Method not allowed");
  }

  // region State

  String attributes() {
    return "[{\"__class\":\"StatusStateAttribute\",\"metaData\":{},\"value\":" + Json.quote(robot.status().valetudo)
        + ",\"flag\":" + Json.quote(robot.flag().valetudo)
        + (robot.status() == CleaningMission.Status.ERROR
            ? ",\"error\":{\"severity\":{\"kind\":\"unknown\",\"level\":\"error\"},\"subsystem\":\"navigation\",\"message\":"
                + Json.quote(robot.error()) + "}"
            : "")
        + "}]";
  }

  private static String capabilityList() {
    StringBuilder b = new StringBuilder("[");
    for (int i = 0; i < CAPABILITIES.length; i++) {
      if (i > 0) b.append(',');
      b.append(Json.quote(CAPABILITIES[i]));
    }
    return b.append(']').toString();
  }

  private static String properties(String cap) {
    switch (cap) {
      case "MapSegmentationCapability":
        return "{\"iterationCount\":{\"min\":1,\"max\":1},\"customOrderSupport\":true}";
      case "CombinedVirtualRestrictionsCapability":
        return "{\"supportedRestrictedZoneTypes\":[\"regular\"]}";
      case "ZoneCleaningCapability":
        return "{\"zoneCount\":{\"min\":1,\"max\":10},\"iterationCount\":{\"min\":1,\"max\":1}}";
      default:
        return "{}";
    }
  }

  // endregion

  // region Capabilities

  private Response capabilityGet(String cap) {
    switch (cap) {
      case "MapSegmentationCapability":
        return Response.json(segmentsJson());
      case "CombinedVirtualRestrictionsCapability":
        return Response.json(restrictionsJson());
      default:
        return notAllowed();
    }
  }

  private Response capabilityPut(String cap, Map<String, Object> b) {
    if (b == null) return Response.error(400, "Body must be JSON");
    String action = Json.str(b.get("action"));
    String err;
    switch (cap) {
      case "BasicControlCapability":
        if (!"start".equals(action) && !"stop".equals(action) && !"pause".equals(action) && !"home".equals(action)) {
          return Response.error(400, "Bad action");
        }
        err = robot.basicControl(action);
        break;
      case "GoToLocationCapability": {
        Object cx = Json.get(b, "coordinates", "x"), cy = Json.get(b, "coordinates", "y");
        if (!"goto".equals(action) || !(cx instanceof Number) || !(cy instanceof Number)) return Response.error(400, "Bad request");
        OccupancyGrid g = robot.grid();
        if (g == null) return Response.error(500, "No map");
        err = robot.goTo(worldX(g, Json.num(cx, 0)), worldY(g, Json.num(cy, 0)));
        break;
      }
      case "MapSegmentationCapability": {
        List<Object> ids = Json.arr(b.get("segment_ids"));
        if (!"start_segment_action".equals(action) || ids == null) return Response.error(400, "Bad request");
        List<Integer> out = new ArrayList<>();
        for (Object o : ids) {
          try {
            out.add(o instanceof Number ? ((Number) o).intValue() : Integer.parseInt(Json.str(o)));
          } catch (NumberFormatException e) {
            return Response.error(400, "Bad segment id " + o);
          }
        }
        err = robot.cleanSegments(out);
        break;
      }
      case "MapSegmentRenameCapability": {
        String id = Json.str(b.get("segment_id")), name = Json.str(b.get("name"));
        if (!"rename_segment".equals(action) || id == null || name == null) return Response.error(400, "Bad request");
        try {
          err = robot.renameSegment(Integer.parseInt(id), name);
        } catch (NumberFormatException e) {
          return Response.error(400, "Bad segment id");
        }
        break;
      }
      case "ZoneCleaningCapability": {
        List<Object> zones = Json.arr(b.get("zones"));
        if (!"clean".equals(action) || zones == null) return Response.error(400, "Bad request");
        OccupancyGrid g = robot.grid();
        if (g == null) return Response.error(500, "No map");
        List<double[]> out = new ArrayList<>();
        for (Object z : zones) {
          double[] poly = polygon(g, Json.get(z, "points"), 4);
          if (poly == null) return Response.error(400, "Invalid Zone");
          out.add(poly);
        }
        err = robot.cleanZones(out);
        break;
      }
      case "CombinedVirtualRestrictionsCapability": {
        List<Object> walls = Json.arr(b.get("virtualWalls"));
        List<Object> zones = Json.arr(b.get("restrictedZones"));
        if (walls == null || zones == null) return Response.error(400, "Bad request");
        OccupancyGrid g = robot.grid();
        if (g == null) return Response.error(500, "No map");
        List<double[]> ws = new ArrayList<>(), zs = new ArrayList<>();
        for (Object w : walls) {
          double[] s = polygon(g, Json.get(w, "points"), 2);
          if (s == null) return Response.error(400, "Invalid wall");
          ws.add(s);
        }
        for (Object z : zones) {
          double[] poly = polygon(g, Json.get(z, "points"), 4);
          if (poly == null) return Response.error(400, "Invalid zone");
          zs.add(poly);
        }
        err = robot.setRestrictions(ws, zs);
        break;
      }
      case "LocateCapability":
        if (!"locate".equals(action)) return Response.error(400, "Bad action");
        err = robot.locate();
        break;
      default:
        return notAllowed();
    }
    return err == null ? Response.ok() : Response.error(500, err);
  }

  /** {"pA":{x,y},"pB":..} in cm (Valetudo frame) to world x0,y0,x1,y1,... */
  static double[] polygon(OccupancyGrid g, Object points, int n) {
    if (points == null) return null;
    double[] out = new double[2 * n];
    for (int i = 0; i < n; i++) {
      Object x = Json.get(points, "p" + (char) ('A' + i), "x"), y = Json.get(points, "p" + (char) ('A' + i), "y");
      if (!(x instanceof Number) || !(y instanceof Number)) return null;
      out[2 * i] = worldX(g, ((Number) x).doubleValue());
      out[2 * i + 1] = worldY(g, ((Number) y).doubleValue());
    }
    return out;
  }

  private String pointsJson(OccupancyGrid g, double[] poly) {
    StringBuilder b = new StringBuilder("{");
    for (int i = 0; i < poly.length / 2; i++) {
      if (i > 0) b.append(',');
      b.append(String.format(Locale.ROOT, "\"p%c\":{\"x\":%d,\"y\":%d}", (char) ('A' + i),
          vx(g, poly[2 * i]), vy(g, poly[2 * i + 1])));
    }
    return b.append('}').toString();
  }

  String segmentsJson() {
    RoomMap r = robot.rooms();
    StringBuilder b = new StringBuilder("[");
    boolean first = true;
    for (Map.Entry<Integer, String> e : r.names.entrySet()) {
      if (!first) b.append(',');
      first = false;
      b.append("{\"__class\":\"ValetudoMapSegment\",\"metaData\":{},\"id\":").append(Json.quote(String.valueOf(e.getKey())))
          .append(",\"name\":").append(Json.quote(e.getValue())).append('}');
    }
    return b.append(']').toString();
  }

  String restrictionsJson() {
    OccupancyGrid g = robot.grid();
    RoomMap r = robot.rooms();
    StringBuilder b = new StringBuilder("{\"__class\":\"ValetudoVirtualRestrictions\",\"metaData\":{},\"virtualWalls\":[");
    if (g != null) {
      for (int i = 0; i < r.walls.size(); i++) {
        if (i > 0) b.append(',');
        b.append("{\"__class\":\"ValetudoVirtualWall\",\"metaData\":{},\"points\":").append(pointsJson(g, r.walls.get(i))).append('}');
      }
    }
    b.append("],\"restrictedZones\":[");
    if (g != null) {
      for (int i = 0; i < r.noGo.size(); i++) {
        if (i > 0) b.append(',');
        b.append("{\"__class\":\"ValetudoRestrictedZone\",\"metaData\":{},\"points\":").append(pointsJson(g, r.noGo.get(i)))
            .append(",\"type\":\"regular\"}");
      }
    }
    return b.append("]}").toString();
  }

  // endregion

  // region Map

  /** Centimetres in Valetudo's frame (top-left origin, y down) from world metres, and back. */
  static int vx(OccupancyGrid g, double x) {
    return (int) Math.round((x - g.originX) * 100);
  }

  static int vy(OccupancyGrid g, double y) {
    return (int) Math.round((g.originY + g.height * g.resolution - y) * 100);
  }

  static double worldX(OccupancyGrid g, double cm) {
    return g.originX + cm / 100.0;
  }

  static double worldY(OccupancyGrid g, double cm) {
    return g.originY + g.height * g.resolution - cm / 100.0;
  }

  /** The map as Valetudo serialises it (ValetudoMap with MapLayer compressedPixels). */
  public String mapJson() {
    OccupancyGrid g = robot.grid();
    if (g == null) return "null";
    RoomMap rooms = robot.rooms();
    int w = g.width, h = g.height;
    int pixelSize = (int) Math.round(g.resolution * 100);
    // Classify cells: 0 unknown, 1 floor, 2 wall, >= 10 room id + 10.
    int[] kind = new int[w * h];
    int maxRoom = 0;
    for (int cy = 0; cy < h; cy++) {
      for (int cx = 0; cx < w; cx++) {
        float v = g.get(cx, cy);
        int k = v > OccupancyGrid.OCC_THRESHOLD ? 2 : v < OccupancyGrid.FREE_THRESHOLD ? 1 : 0;
        if (k == 1 && rooms.labels.length > 0) {
          int id = rooms.roomAt(g.worldX(cx), g.worldY(cy));
          if (id > 0) {
            k = 10 + id;
            maxRoom = Math.max(maxRoom, id);
          }
        }
        kind[cy * w + cx] = k;
      }
    }
    StringBuilder b = new StringBuilder(64 * 1024);
    b.append("{\"__class\":\"ValetudoMap\",\"metaData\":{\"version\":2,\"nonce\":\"openbot\"},");
    b.append(String.format(Locale.ROOT, "\"size\":{\"x\":%d,\"y\":%d},\"pixelSize\":%d,\"layers\":[", w * pixelSize, h * pixelSize, pixelSize));
    boolean first = true;
    first = layer(b, kind, w, h, 1, "floor", "{}", pixelSize, first);
    first = layer(b, kind, w, h, 2, "wall", "{}", pixelSize, first);
    int robotRoom = 0;
    Pose pose = robot.pose();
    if (pose != null) robotRoom = rooms.roomAt(pose.x, pose.y);
    for (int id = 1; id <= maxRoom; id++) {
      String meta = String.format(Locale.ROOT, "{\"segmentId\":\"%d\",\"active\":%b,\"name\":%s", id, id == robotRoom, Json.quote(rooms.name(id)));
      first = layer(b, kind, w, h, 10 + id, "segment", meta, pixelSize, first);
    }
    b.append("],\"entities\":[");
    List<String> ents = new ArrayList<>();
    if (pose != null) {
      double angle = ((90 - Math.toDegrees(pose.theta)) % 360 + 360) % 360; // 0 = up, clockwise
      ents.add(String.format(Locale.ROOT,
          "{\"__class\":\"PointMapEntity\",\"metaData\":{\"angle\":%.0f},\"points\":[%d,%d],\"type\":\"robot_position\"}",
          angle, vx(g, pose.x), vy(g, pose.y)));
    }
    if (rooms.home != null) {
      ents.add(String.format(Locale.ROOT, "{\"__class\":\"PointMapEntity\",\"metaData\":{},\"points\":[%d,%d],\"type\":\"charger_location\"}",
          vx(g, rooms.home[0]), vy(g, rooms.home[1])));
    }
    double[] target = robot.goToTarget();
    if (target != null) {
      ents.add(String.format(Locale.ROOT, "{\"__class\":\"PointMapEntity\",\"metaData\":{},\"points\":[%d,%d],\"type\":\"go_to_target\"}",
          vx(g, target[0]), vy(g, target[1])));
    }
    List<double[]> path = robot.plannedPath();
    if (path != null && path.size() > 1) ents.add(pathEntity(g, path, "predicted_path"));
    for (double[] z : robot.activeZones()) ents.add(polygonEntity(g, z, "active_zone"));
    for (double[] z : rooms.noGo) ents.add(polygonEntity(g, z, "no_go_area"));
    for (double[] s : rooms.walls) {
      ents.add(String.format(Locale.ROOT, "{\"__class\":\"LineMapEntity\",\"metaData\":{},\"points\":[%d,%d,%d,%d],\"type\":\"virtual_wall\"}",
          vx(g, s[0]), vy(g, s[1]), vx(g, s[2]), vy(g, s[3])));
    }
    b.append(String.join(",", ents));
    return b.append("]}").toString();
  }

  private static String polygonEntity(OccupancyGrid g, double[] poly, String type) {
    StringBuilder b = new StringBuilder("{\"__class\":\"PolygonMapEntity\",\"metaData\":{},\"points\":[");
    for (int i = 0; i < poly.length / 2; i++) {
      if (i > 0) b.append(',');
      b.append(vx(g, poly[2 * i])).append(',').append(vy(g, poly[2 * i + 1]));
    }
    return b.append("],\"type\":\"").append(type).append("\"}").toString();
  }

  private static String pathEntity(OccupancyGrid g, List<double[]> path, String type) {
    StringBuilder b = new StringBuilder("{\"__class\":\"PathMapEntity\",\"metaData\":{},\"points\":[");
    for (int i = 0; i < path.size(); i++) {
      if (i > 0) b.append(',');
      b.append(vx(g, path.get(i)[0])).append(',').append(vy(g, path.get(i)[1]));
    }
    return b.append("],\"type\":\"").append(type).append("\"}").toString();
  }

  /**
   * One MapLayer: pixels as Valetudo's run-length triples [xStart, y, count] sorted by row; plus its
   * dimensions and area (cm²). Returns the new "first" flag.
   */
  private static boolean layer(StringBuilder b, int[] kind, int w, int h, int want, String type, String meta, int pixelSize,
      boolean first) {
    StringBuilder runs = new StringBuilder();
    int count = 0, minX = Integer.MAX_VALUE, maxX = -1, minY = Integer.MAX_VALUE, maxY = -1;
    long sumX = 0, sumY = 0;
    boolean any = false;
    for (int py = 0; py < h; py++) {
      int cy = h - 1 - py; // grid rows run bottom-up, Valetudo's top-down
      int x = 0;
      while (x < w) {
        if (kind[cy * w + x] != want) {
          x++;
          continue;
        }
        int s = x;
        while (x < w && kind[cy * w + x] == want) x++;
        int n = x - s;
        if (any) runs.append(',');
        any = true;
        runs.append(s).append(',').append(py).append(',').append(n);
        count += n;
        minX = Math.min(minX, s);
        maxX = Math.max(maxX, x - 1);
        minY = Math.min(minY, py);
        maxY = Math.max(maxY, py);
        sumX += (long) (s + x - 1) * n / 2;
        sumY += (long) py * n;
      }
    }
    if (count == 0) return first;
    if (!first) b.append(',');
    String metaWithArea = meta.equals("{}") ? String.format(Locale.ROOT, "{\"area\":%d}", (long) count * pixelSize * pixelSize)
        : meta + String.format(Locale.ROOT, ",\"area\":%d}", (long) count * pixelSize * pixelSize);
    b.append("{\"__class\":\"MapLayer\",\"metaData\":").append(metaWithArea).append(",\"type\":\"").append(type)
        .append("\",\"pixels\":[],\"dimensions\":");
    b.append(String.format(Locale.ROOT,
        "{\"x\":{\"min\":%d,\"max\":%d,\"mid\":%d,\"avg\":%d},\"y\":{\"min\":%d,\"max\":%d,\"mid\":%d,\"avg\":%d},\"pixelCount\":%d}",
        minX, maxX, (minX + maxX) / 2, sumX / count, minY, maxY, (minY + maxY) / 2, sumY / count, count));
    b.append(",\"compressedPixels\":[").append(runs).append("]}");
    return false;
  }

  // endregion
}
