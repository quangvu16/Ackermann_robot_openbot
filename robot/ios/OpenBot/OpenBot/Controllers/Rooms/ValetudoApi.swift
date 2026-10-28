//
// Valetudo REST API (v2) and MQTT topics for the Rooms & Zones screen. Same paths, bodies and
// topics as Valetudo (github.com/Hypfer/Valetudo); same behaviour as the Android ValetudoApi,
// ValetudoMqtt and MqttLite classes. Coordinates are Valetudo's: cm from the map's top-left, y down.
//

import Foundation
import Network

/// What Rooms & Zones offers to the API. Getters may be called from any queue.
protocol ValetudoRobot: AnyObject {
    var vGrid: OccupancyGrid? { get }
    var vRooms: RoomMap { get }
    var vPose: SlamPose { get }
    var vStatus: CleaningMission.Status { get }
    var vFlag: CleaningMission.Flag { get }
    var vError: String { get }
    var vPlannedPath: [(Double, Double)] { get }
    var vGoToTarget: (Double, Double)? { get }
    var vActiveZones: [[Double]] { get }
    /// Actions return nil when done, or why not.
    func basicControl(_ action: String) -> String?
    func goTo(_ x: Double, _ y: Double) -> String?
    func cleanSegments(_ ids: [Int]) -> String?
    func cleanZones(_ zones: [[Double]]) -> String?
    func renameSegment(_ id: Int, _ name: String) -> String?
    func setRestrictions(walls: [[Double]], noGo: [[Double]]) -> String?
    func locate() -> String?
}

struct ValetudoResponse {
    let status: Int
    let contentType: String
    let body: String
    static func json(_ s: String) -> ValetudoResponse { ValetudoResponse(status: 200, contentType: "application/json", body: s) }
    static func error(_ status: Int, _ s: String) -> ValetudoResponse { ValetudoResponse(status: status, contentType: "text/plain", body: s) }
    static let ok = ValetudoResponse(status: 200, contentType: "text/plain", body: "OK")
}

final class ValetudoApi {
    static let capabilities = ["BasicControlCapability", "CombinedVirtualRestrictionsCapability", "GoToLocationCapability",
                               "LocateCapability", "MapSegmentRenameCapability", "MapSegmentationCapability", "ZoneCleaningCapability"]

    weak var robot: ValetudoRobot?

    init(robot: ValetudoRobot) { self.robot = robot }

    static func quote(_ s: String) -> String {
        let d = (try? JSONSerialization.data(withJSONObject: [s], options: [])) ?? Data("[\"\"]".utf8)
        let t = String(decoding: d, as: UTF8.self)
        return String(t.dropFirst().dropLast())
    }

    func handle(method: String, path: String, body: String?) -> ValetudoResponse {
        guard let robot = robot else { return .error(503, "Not ready") }
        var p = path.split(separator: "?", maxSplits: 1).first.map(String.init) ?? path
        while p.count > 1 && p.hasSuffix("/") { p.removeLast() }
        let get = method == "GET", put = method == "PUT" || method == "POST"
        let notAllowed = ValetudoResponse.error(405, "Method not allowed")
        switch p {
        case "/api/v2/robot":
            return get ? .json("{\"manufacturer\":\"OpenBot\",\"modelName\":\"Ackermann RC_S3\",\"modelDetails\":{\"supportedAttachments\":[]},\"implementation\":\"OpenBotRoomsRobot\"}") : notAllowed
        case "/api/v2/robot/properties":
            return get ? .json("{\"firmwareVersion\":\"openbot\"}") : notAllowed
        case "/api/v2/robot/state":
            return get ? .json("{\"__class\":\"RobotState\",\"metaData\":{},\"attributes\":\(attributes(robot)),\"map\":\(mapJson())}") : notAllowed
        case "/api/v2/robot/state/attributes":
            return get ? .json(attributes(robot)) : notAllowed
        case "/api/v2/robot/state/map":
            return get ? .json(mapJson()) : notAllowed
        case "/api/v2/robot/capabilities":
            return get ? .json("[" + ValetudoApi.capabilities.map { ValetudoApi.quote($0) }.joined(separator: ",") + "]") : notAllowed
        case "/api/v2/valetudo":
            return get ? .json("{\"embedded\":false,\"systemId\":\"openbot\",\"welcomeDialogDismissed\":true}") : notAllowed
        case "/api/v2/valetudo/version":
            return get ? .json("{\"release\":\"openbot-rooms\",\"commit\":\"\"}") : notAllowed
        default:
            break
        }
        let prefix = "/api/v2/robot/capabilities/"
        guard p.hasPrefix(prefix) else { return .error(404, "Not found") }
        var cap = String(p.dropFirst(prefix.count))
        let props = cap.hasSuffix("/properties")
        if props { cap = String(cap.dropLast("/properties".count)) }
        guard ValetudoApi.capabilities.contains(cap) else { return .error(404, "Capability not supported") }
        if props {
            guard get else { return notAllowed }
            switch cap {
            case "MapSegmentationCapability": return .json("{\"iterationCount\":{\"min\":1,\"max\":1},\"customOrderSupport\":true}")
            case "CombinedVirtualRestrictionsCapability": return .json("{\"supportedRestrictedZoneTypes\":[\"regular\"]}")
            case "ZoneCleaningCapability": return .json("{\"zoneCount\":{\"min\":1,\"max\":10},\"iterationCount\":{\"min\":1,\"max\":1}}")
            default: return .json("{}")
            }
        }
        if get {
            switch cap {
            case "MapSegmentationCapability": return .json(segmentsJson(robot))
            case "CombinedVirtualRestrictionsCapability": return .json(restrictionsJson(robot))
            default: return notAllowed
            }
        }
        guard put else { return notAllowed }
        guard let data = body?.data(using: .utf8), let b = (try? JSONSerialization.jsonObject(with: data)) as? [String: Any] else {
            return .error(400, "Body must be JSON")
        }
        let action = b["action"] as? String
        var err: String?
        switch cap {
        case "BasicControlCapability":
            guard let a = action, ["start", "stop", "pause", "home"].contains(a) else { return .error(400, "Bad action") }
            err = robot.basicControl(a)
        case "GoToLocationCapability":
            guard action == "goto", let c = b["coordinates"] as? [String: Any], let x = (c["x"] as? NSNumber)?.doubleValue,
                  let y = (c["y"] as? NSNumber)?.doubleValue else { return .error(400, "Bad request") }
            guard let g = robot.vGrid else { return .error(500, "No map") }
            err = robot.goTo(ValetudoApi.worldX(g, x), ValetudoApi.worldY(g, y))
        case "MapSegmentationCapability":
            guard action == "start_segment_action", let ids = b["segment_ids"] as? [Any] else { return .error(400, "Bad request") }
            var out: [Int] = []
            for o in ids {
                if let n = o as? NSNumber { out.append(n.intValue) } else if let s = o as? String, let n = Int(s) { out.append(n) } else {
                    return .error(400, "Bad segment id")
                }
            }
            err = robot.cleanSegments(out)
        case "MapSegmentRenameCapability":
            guard action == "rename_segment", let id = b["segment_id"] as? String, let n = Int(id), let name = b["name"] as? String else {
                return .error(400, "Bad request")
            }
            err = robot.renameSegment(n, name)
        case "ZoneCleaningCapability":
            guard action == "clean", let zones = b["zones"] as? [Any] else { return .error(400, "Bad request") }
            guard let g = robot.vGrid else { return .error(500, "No map") }
            var out: [[Double]] = []
            for z in zones {
                guard let pts = (z as? [String: Any])?["points"], let poly = ValetudoApi.polygon(g, pts, 4) else { return .error(400, "Invalid Zone") }
                out.append(poly)
            }
            err = robot.cleanZones(out)
        case "CombinedVirtualRestrictionsCapability":
            guard let walls = b["virtualWalls"] as? [Any], let zones = b["restrictedZones"] as? [Any] else { return .error(400, "Bad request") }
            guard let g = robot.vGrid else { return .error(500, "No map") }
            var ws: [[Double]] = [], zs: [[Double]] = []
            for w in walls {
                guard let pts = (w as? [String: Any])?["points"], let s = ValetudoApi.polygon(g, pts, 2) else { return .error(400, "Invalid wall") }
                ws.append(s)
            }
            for z in zones {
                guard let pts = (z as? [String: Any])?["points"], let poly = ValetudoApi.polygon(g, pts, 4) else { return .error(400, "Invalid zone") }
                zs.append(poly)
            }
            err = robot.setRestrictions(walls: ws, noGo: zs)
        case "LocateCapability":
            guard action == "locate" else { return .error(400, "Bad action") }
            err = robot.locate()
        default:
            return notAllowed
        }
        return err == nil ? .ok : .error(500, err!)
    }

    // MARK: Coordinates

    static func vx(_ g: OccupancyGrid, _ x: Double) -> Int { Int(((x - g.originX) * 100).rounded()) }
    static func vy(_ g: OccupancyGrid, _ y: Double) -> Int { Int(((g.originY + Double(g.height) * g.resolution - y) * 100).rounded()) }
    static func worldX(_ g: OccupancyGrid, _ cm: Double) -> Double { g.originX + cm / 100 }
    static func worldY(_ g: OccupancyGrid, _ cm: Double) -> Double { g.originY + Double(g.height) * g.resolution - cm / 100 }
    static func angle(_ theta: Double) -> Double { ((90 - theta * 180 / .pi).truncatingRemainder(dividingBy: 360) + 360).truncatingRemainder(dividingBy: 360) }

    static func polygon(_ g: OccupancyGrid, _ points: Any, _ n: Int) -> [Double]? {
        guard let pts = points as? [String: Any] else { return nil }
        var out: [Double] = []
        for i in 0..<n {
            let key = "p" + String(Character(UnicodeScalar(UInt8(65 + i))))
            guard let pt = pts[key] as? [String: Any], let x = (pt["x"] as? NSNumber)?.doubleValue,
                  let y = (pt["y"] as? NSNumber)?.doubleValue else { return nil }
            out.append(worldX(g, x)); out.append(worldY(g, y))
        }
        return out
    }

    private func pointsJson(_ g: OccupancyGrid, _ poly: [Double]) -> String {
        "{" + (0..<(poly.count / 2)).map { i in
            "\"p\(String(Character(UnicodeScalar(UInt8(65 + i)))))\":{\"x\":\(ValetudoApi.vx(g, poly[2 * i])),\"y\":\(ValetudoApi.vy(g, poly[2 * i + 1]))}"
        }.joined(separator: ",") + "}"
    }

    // MARK: State

    func attributes(_ r: ValetudoRobot) -> String {
        var s = "[{\"__class\":\"StatusStateAttribute\",\"metaData\":{},\"value\":\"\(r.vStatus.rawValue)\",\"flag\":\"\(r.vFlag.rawValue)\""
        if r.vStatus == .error {
            s += ",\"error\":{\"severity\":{\"kind\":\"unknown\",\"level\":\"error\"},\"subsystem\":\"navigation\",\"message\":\(ValetudoApi.quote(r.vError))}"
        }
        return s + "}]"
    }

    func segmentsJson(_ r: ValetudoRobot) -> String {
        let rooms = r.vRooms
        return "[" + rooms.roomIds.map {
            "{\"__class\":\"ValetudoMapSegment\",\"metaData\":{},\"id\":\"\($0)\",\"name\":\(ValetudoApi.quote(rooms.name($0)))}"
        }.joined(separator: ",") + "]"
    }

    func restrictionsJson(_ r: ValetudoRobot) -> String {
        let rooms = r.vRooms
        var walls: [String] = [], zones: [String] = []
        if let g = r.vGrid {
            walls = rooms.walls.map { "{\"__class\":\"ValetudoVirtualWall\",\"metaData\":{},\"points\":\(pointsJson(g, $0))}" }
            zones = rooms.noGo.map { "{\"__class\":\"ValetudoRestrictedZone\",\"metaData\":{},\"points\":\(pointsJson(g, $0)),\"type\":\"regular\"}" }
        }
        return "{\"__class\":\"ValetudoVirtualRestrictions\",\"metaData\":{},\"virtualWalls\":[\(walls.joined(separator: ","))],\"restrictedZones\":[\(zones.joined(separator: ","))]}"
    }

    // MARK: Map

    /// The map as Valetudo serialises it: floor, wall and segment layers with compressedPixels.
    func mapJson() -> String {
        guard let r = robot, let g = r.vGrid else { return "null" }
        let rooms = r.vRooms
        let w = g.width, h = g.height
        let pixelSize = Int((g.resolution * 100).rounded())
        var kind = [Int](repeating: 0, count: w * h)
        var maxRoom = 0
        for cy in 0..<h {
            for cx in 0..<w {
                let v = g.logOdds[cy * w + cx]
                var k = v > OccupancyGrid.occThreshold ? 2 : v < OccupancyGrid.freeThreshold ? 1 : 0
                if k == 1 && !rooms.labels.isEmpty {
                    let id = rooms.roomAt(g.worldX(cx), g.originY + (Double(cy) + 0.5) * g.resolution)
                    if id > 0 { k = 10 + id; maxRoom = max(maxRoom, id) }
                }
                kind[cy * w + cx] = k
            }
        }
        var layers: [String] = []
        if let l = layer(kind, w, h, 1, "floor", nil, pixelSize) { layers.append(l) }
        if let l = layer(kind, w, h, 2, "wall", nil, pixelSize) { layers.append(l) }
        let pose = r.vPose
        let here = rooms.roomAt(pose.x, pose.y)
        if maxRoom > 0 {
            for id in 1...maxRoom {
                let meta = "\"segmentId\":\"\(id)\",\"active\":\(id == here),\"name\":\(ValetudoApi.quote(rooms.name(id)))"
                if let l = layer(kind, w, h, 10 + id, "segment", meta, pixelSize) { layers.append(l) }
            }
        }
        var ents: [String] = []
        ents.append("{\"__class\":\"PointMapEntity\",\"metaData\":{\"angle\":\(Int(ValetudoApi.angle(pose.theta).rounded()))},\"points\":[\(ValetudoApi.vx(g, pose.x)),\(ValetudoApi.vy(g, pose.y))],\"type\":\"robot_position\"}")
        if let hm = rooms.home {
            ents.append("{\"__class\":\"PointMapEntity\",\"metaData\":{},\"points\":[\(ValetudoApi.vx(g, hm.0)),\(ValetudoApi.vy(g, hm.1))],\"type\":\"charger_location\"}")
        }
        if let t = r.vGoToTarget {
            ents.append("{\"__class\":\"PointMapEntity\",\"metaData\":{},\"points\":[\(ValetudoApi.vx(g, t.0)),\(ValetudoApi.vy(g, t.1))],\"type\":\"go_to_target\"}")
        }
        let path = r.vPlannedPath
        if path.count > 1 {
            ents.append("{\"__class\":\"PathMapEntity\",\"metaData\":{},\"points\":[" + path.map { "\(ValetudoApi.vx(g, $0.0)),\(ValetudoApi.vy(g, $0.1))" }.joined(separator: ",") + "],\"type\":\"predicted_path\"}")
        }
        func poly(_ p: [Double], _ type: String) -> String {
            "{\"__class\":\"PolygonMapEntity\",\"metaData\":{},\"points\":[" + (0..<(p.count / 2)).map { "\(ValetudoApi.vx(g, p[2 * $0])),\(ValetudoApi.vy(g, p[2 * $0 + 1]))" }.joined(separator: ",") + "],\"type\":\"\(type)\"}"
        }
        for z in r.vActiveZones { ents.append(poly(z, "active_zone")) }
        for z in rooms.noGo { ents.append(poly(z, "no_go_area")) }
        for s in rooms.walls {
            ents.append("{\"__class\":\"LineMapEntity\",\"metaData\":{},\"points\":[\(ValetudoApi.vx(g, s[0])),\(ValetudoApi.vy(g, s[1])),\(ValetudoApi.vx(g, s[2])),\(ValetudoApi.vy(g, s[3]))],\"type\":\"virtual_wall\"}")
        }
        return "{\"__class\":\"ValetudoMap\",\"metaData\":{\"version\":2,\"nonce\":\"openbot\"},\"size\":{\"x\":\(w * pixelSize),\"y\":\(h * pixelSize)},\"pixelSize\":\(pixelSize),\"layers\":[\(layers.joined(separator: ","))],\"entities\":[\(ents.joined(separator: ","))]}"
    }

    private func layer(_ kind: [Int], _ w: Int, _ h: Int, _ want: Int, _ type: String, _ meta: String?, _ pixelSize: Int) -> String? {
        var runs: [String] = []
        var count = 0, minX = Int.max, maxX = -1, minY = Int.max, maxY = -1
        var sumX = 0, sumY = 0
        for py in 0..<h {
            let cy = h - 1 - py
            var x = 0
            while x < w {
                if kind[cy * w + x] != want { x += 1; continue }
                let s = x
                while x < w && kind[cy * w + x] == want { x += 1 }
                let n = x - s
                runs.append("\(s),\(py),\(n)")
                count += n
                minX = min(minX, s); maxX = max(maxX, x - 1); minY = min(minY, py); maxY = max(maxY, py)
                sumX += (s + x - 1) * n / 2; sumY += py * n
            }
        }
        guard count > 0 else { return nil }
        let area = count * pixelSize * pixelSize
        let m = meta.map { "{\($0),\"area\":\(area)}" } ?? "{\"area\":\(area)}"
        return "{\"__class\":\"MapLayer\",\"metaData\":\(m),\"type\":\"\(type)\",\"pixels\":[],\"dimensions\":{\"x\":{\"min\":\(minX),\"max\":\(maxX),\"mid\":\((minX + maxX) / 2),\"avg\":\(sumX / count)},\"y\":{\"min\":\(minY),\"max\":\(maxY),\"mid\":\((minY + maxY) / 2),\"avg\":\(sumY / count)},\"pixelCount\":\(count)},\"compressedPixels\":[\(runs.joined(separator: ","))]}"
    }
}

/// Hands the Rooms & Zones screen to the Sensor link web server while it is open.
final class ValetudoHub {
    static let shared = ValetudoHub()
    private let lock = NSLock()
    private weak var api: ValetudoApi?

    func set(_ a: ValetudoApi) { lock.lock(); api = a; lock.unlock() }
    func clear(_ a: ValetudoApi) { lock.lock(); if api === a { api = nil }; lock.unlock() }

    func handle(method: String, path: String, body: String?) -> ValetudoResponse {
        lock.lock(); let a = api; lock.unlock()
        guard let a = a else { return .error(503, "Open Rooms & Zones on the phone to use the Valetudo API\n") }
        return a.handle(method: method, path: path, body: body)
    }
}

// MARK: - MQTT

/// A small MQTT 3.1.1 client: plain TCP, QoS 0, retained messages, last will, pings, reconnect.
final class MqttLite {
    struct Config {
        var host = "", port = 1883, clientId = "openbot", username = "", password = ""
        var keepAlive = 30
        var willTopic: String?, willPayload: String?
        var subscriptions: [String] = []
    }

    var onConnected: (() -> Void)?
    var onMessage: ((String, String) -> Void)?
    var onStatus: ((String) -> Void)?

    private let config: Config
    private let queue = DispatchQueue(label: "mqtt")
    private var connection: NWConnection?
    private var buffer = Data()
    private var running = false
    private(set) var connected = false
    private var pingTimer: DispatchSourceTimer?
    private var packetId: UInt16 = 1

    init(_ config: Config) { self.config = config }

    func start() { queue.async { guard !self.running else { return }; self.running = true; self.connect() } }

    func stop() {
        queue.sync {
            running = false
            if connected { connection?.send(content: Data([0xE0, 0]), completion: .idempotent) }
            close()
        }
    }

    func publish(_ topic: String, _ payload: String, retain: Bool) {
        queue.async {
            guard self.connected, let c = self.connection else { return }
            c.send(content: MqttLite.publishPacket(topic, payload, retain: retain), completion: .idempotent)
        }
    }

    private func connect() {
        guard running, let port = NWEndpoint.Port(rawValue: UInt16(clamping: config.port)) else { return }
        onStatus?("Connecting to \(config.host):\(config.port)")
        let c = NWConnection(host: NWEndpoint.Host(config.host), port: port, using: .tcp)
        connection = c
        buffer = Data()
        c.stateUpdateHandler = { [weak self] state in
            guard let self = self else { return }
            switch state {
            case .ready:
                c.send(content: MqttLite.connectPacket(self.config), completion: .idempotent)
                self.receive(c)
            case .failed(let e):
                self.onStatus?("MQTT: \(e.localizedDescription)")
                self.close(); self.retry()
            case .cancelled:
                break
            default:
                break
            }
        }
        c.start(queue: queue)
    }

    private func retry() {
        guard running else { return }
        queue.asyncAfter(deadline: .now() + 5) { [weak self] in self?.connect() }
    }

    private func close() {
        connected = false
        pingTimer?.cancel(); pingTimer = nil
        connection?.cancel(); connection = nil
    }

    private func receive(_ c: NWConnection) {
        c.receive(minimumIncompleteLength: 1, maximumLength: 65536) { [weak self] data, _, done, error in
            guard let self = self, c === self.connection else { return }
            if let d = data { self.buffer.append(d); self.parse() }
            if done || error != nil {
                self.onStatus?("MQTT: connection closed")
                self.close(); self.retry()
            } else {
                self.receive(c)
            }
        }
    }

    private func parse() {
        while true {
            let bytes = [UInt8](buffer.prefix(5))
            guard bytes.count >= 2 else { return }
            var len = 0, mult = 1, i = 1
            while true {
                guard i < bytes.count else { return }
                let b = Int(bytes[i]); len += (b & 0x7F) * mult; mult *= 128; i += 1
                if b & 0x80 == 0 { break }
                if i >= 5 { close(); retry(); return }
            }
            guard buffer.count >= i + len else { return }
            let all = [UInt8](buffer.prefix(i + len))
            buffer.removeFirst(i + len)
            let type = all[0] & 0xF0
            let body = Array(all[i...])
            switch type {
            case 0x20: // CONNACK
                if body.count >= 2 && body[1] == 0 {
                    connected = true
                    for t in config.subscriptions {
                        connection?.send(content: MqttLite.subscribePacket(packetId, t), completion: .idempotent)
                        packetId &+= 1
                    }
                    startPing()
                    onStatus?("Connected to \(config.host)")
                    onConnected?()
                } else {
                    onStatus?("Broker refused the connection (code \(body.count >= 2 ? Int(body[1]) : -1))")
                    close(); retry(); return
                }
            case 0x30: // PUBLISH
                guard body.count >= 2 else { break }
                let tl = Int(body[0]) << 8 | Int(body[1])
                guard body.count >= 2 + tl else { break }
                let topic = String(decoding: body[2..<(2 + tl)], as: UTF8.self)
                var at = 2 + tl
                if (all[0] >> 1) & 3 > 0 { at += 2 }
                let payload = at <= body.count ? String(decoding: body[at...], as: UTF8.self) : ""
                onMessage?(topic, payload)
            default:
                break // SUBACK, PINGRESP
            }
        }
    }

    private func startPing() {
        let t = DispatchSource.makeTimerSource(queue: queue)
        let every = max(5, config.keepAlive / 2)
        t.schedule(deadline: .now() + .seconds(every), repeating: .seconds(every))
        t.setEventHandler { [weak self] in self?.connection?.send(content: Data([0xC0, 0]), completion: .idempotent) }
        t.resume()
        pingTimer = t
    }

    // MARK: Packets

    static func remainingLength(_ n: Int) -> [UInt8] {
        var len = n, out: [UInt8] = []
        repeat {
            var d = UInt8(len % 128)
            len /= 128
            if len > 0 { d |= 0x80 }
            out.append(d)
        } while len > 0
        return out
    }

    private static func str(_ s: String) -> [UInt8] {
        let u = Array(s.utf8)
        return [UInt8(u.count >> 8), UInt8(u.count & 0xFF)] + u
    }

    private static func packet(_ type: UInt8, _ body: [UInt8]) -> Data { Data([type] + remainingLength(body.count) + body) }

    static func connectPacket(_ c: Config) -> Data {
        var b = str("MQTT") + [4]
        var flags: UInt8 = 0x02
        let will = c.willTopic != nil && c.willPayload != nil
        if will { flags |= 0x04 | 0x20 }
        let user = !c.username.isEmpty, pass = user && !c.password.isEmpty
        if user { flags |= 0x80 }
        if pass { flags |= 0x40 }
        b += [flags, UInt8(c.keepAlive >> 8), UInt8(c.keepAlive & 0xFF)]
        b += str(c.clientId)
        if will { b += str(c.willTopic!) + str(c.willPayload!) }
        if user { b += str(c.username) }
        if pass { b += str(c.password) }
        return packet(0x10, b)
    }

    static func publishPacket(_ topic: String, _ payload: String, retain: Bool) -> Data {
        packet(0x30 | (retain ? 1 : 0), str(topic) + Array(payload.utf8))
    }

    static func subscribePacket(_ id: UInt16, _ filter: String) -> Data {
        packet(0x82, [UInt8(id >> 8), UInt8(id & 0xFF)] + str(filter) + [0])
    }
}

/// Valetudo's MQTT topics (Homie layout) plus Home Assistant discovery; see the Android
/// ValetudoMqtt class for the topic list.
final class ValetudoMqtt {
    struct Settings {
        var enabled = false, host = "", port = 1883, username = "", password = ""
        var prefix = "valetudo", identifier = "openbot", homeAssistant = true
    }

    let base: String
    private let settings: Settings
    private weak var robot: ValetudoRobot?
    private let api: ValetudoApi
    private var client: MqttLite?
    private var sent: [String: String] = [:]
    private let lock = NSLock()
    var onStatus: ((String) -> Void)?

    init(robot: ValetudoRobot, api: ValetudoApi, settings: Settings) {
        self.robot = robot
        self.api = api
        self.settings = settings
        base = ValetudoMqtt.topicSafe(settings.prefix) + "/" + ValetudoMqtt.topicSafe(settings.identifier)
    }

    static func topicSafe(_ s: String) -> String {
        let t = s.trimmingCharacters(in: .whitespaces).replacingOccurrences(of: "[#+\\s/]+", with: "_", options: .regularExpression)
        return t.isEmpty ? "openbot" : t
    }

    func start() {
        guard settings.enabled, !settings.host.trimmingCharacters(in: .whitespaces).isEmpty else { return }
        var c = MqttLite.Config()
        c.host = settings.host.trimmingCharacters(in: .whitespaces)
        c.port = settings.port
        c.username = settings.username
        c.password = settings.password
        c.clientId = "openbot-" + ValetudoMqtt.topicSafe(settings.identifier)
        c.willTopic = base + "/$state"
        c.willPayload = "lost"
        c.subscriptions = [base + "/+/+/set"]
        let m = MqttLite(c)
        m.onStatus = { [weak self] s in self?.onStatus?(s) }
        m.onConnected = { [weak self] in self?.connected() }
        m.onMessage = { [weak self] t, p in
            guard let self = self, t.hasPrefix(self.base + "/"), t.hasSuffix("/set") else { return }
            let what = String(t.dropFirst(self.base.count + 1).dropLast(4))
            if let err = self.handle(what, p.trimmingCharacters(in: .whitespacesAndNewlines)) { self.onStatus?("MQTT \(what): \(err)") }
        }
        client = m
        m.start()
    }

    func stop() {
        client?.publish(base + "/$state", "disconnected", retain: true)
        client?.stop()
        client = nil
    }

    private func connected() {
        lock.lock(); sent = [:]; lock.unlock()
        guard let c = client else { return }
        c.publish(base + "/$homie", "4.0.0", retain: true)
        c.publish(base + "/$name", "OpenBot \(settings.identifier)", retain: true)
        c.publish(base + "/$state", "ready", retain: true)
        if settings.homeAssistant { publishDiscovery(c) }
        publishState()
    }

    /// About once a second; sends only what changed.
    func publishState() {
        guard let c = client, c.connected, let r = robot else { return }
        let st = r.vStatus
        send(c, "/StatusStateAttribute/status", st.rawValue)
        send(c, "/StatusStateAttribute/flag", r.vFlag.rawValue)
        send(c, "/StatusStateAttribute/error_description", st == .error ? r.vError : "No error")
        let rooms = r.vRooms
        send(c, "/MapData/segments", "{" + rooms.roomIds.map { "\"\($0)\":\(ValetudoApi.quote(rooms.name($0)))" }.joined(separator: ",") + "}")
        let p = r.vPose
        let room = rooms.roomAt(p.x, p.y)
        send(c, "/OpenBot/room", room > 0 ? rooms.name(room) : "")
        if let g = r.vGrid {
            send(c, "/OpenBot/position", "{\"x\":\(ValetudoApi.vx(g, p.x)),\"y\":\(ValetudoApi.vy(g, p.y)),\"angle\":\(Int(ValetudoApi.angle(p.theta).rounded()))}")
        }
        if settings.homeAssistant {
            send(c, "/hass/\(ValetudoMqtt.topicSafe(settings.identifier))_vacuum/state", "{\"state\":\"\(ValetudoMqtt.hassState(st))\"}")
        }
    }

    private func send(_ c: MqttLite, _ sub: String, _ value: String) {
        lock.lock()
        let changed = sent[sub] != value
        if changed { sent[sub] = value }
        lock.unlock()
        if changed { c.publish(base + sub, value, retain: true) }
    }

    static func hassState(_ s: CleaningMission.Status) -> String {
        switch s {
        case .cleaning, .moving: return "cleaning"
        case .docked: return "docked"
        case .paused: return "paused"
        case .returning: return "returning"
        case .error: return "error"
        case .idle: return "idle"
        }
    }

    private func publishDiscovery(_ c: MqttLite) {
        let id = ValetudoMqtt.topicSafe(settings.identifier)
        let device = "{\"identifiers\":[\"openbot_\(id)\"],\"name\":\"OpenBot \(id)\",\"manufacturer\":\"OpenBot\",\"model\":\"Ackermann RC_S3\"}"
        let avail = "\"availability_topic\":\"\(base)/$state\",\"payload_available\":\"ready\",\"payload_not_available\":\"lost\""
        c.publish("homeassistant/vacuum/\(id)/\(id)_vacuum/config",
                  "{\"name\":null,\"unique_id\":\"\(id)_vacuum\",\"device\":\(device),\(avail),\"command_topic\":\"\(base)/BasicControlCapability/operation/set\",\"payload_start\":\"START\",\"payload_stop\":\"STOP\",\"payload_pause\":\"PAUSE\",\"payload_return_to_base\":\"HOME\",\"payload_locate\":\"LOCATE\",\"state_topic\":\"\(base)/hass/\(id)_vacuum/state\",\"send_command_topic\":\"\(base)/MapSegmentationCapability/clean/set\",\"supported_features\":[\"start\",\"stop\",\"pause\",\"return_home\",\"locate\",\"send_command\"]}",
                  retain: true)
        c.publish("homeassistant/sensor/\(id)/\(id)_room/config",
                  "{\"name\":\"Current room\",\"unique_id\":\"\(id)_room\",\"device\":\(device),\(avail),\"state_topic\":\"\(base)/OpenBot/room\",\"icon\":\"mdi:floor-plan\"}", retain: true)
        c.publish("homeassistant/sensor/\(id)/\(id)_flag/config",
                  "{\"name\":\"Status flag\",\"unique_id\":\"\(id)_flag\",\"device\":\(device),\(avail),\"state_topic\":\"\(base)/StatusStateAttribute/flag\",\"icon\":\"mdi:flag\"}", retain: true)
    }

    /// nil when done, or why not.
    func handle(_ what: String, _ p: String) -> String? {
        guard let r = robot else { return "not ready" }
        switch what {
        case "BasicControlCapability/operation":
            let v = p.uppercased()
            if v == "LOCATE" { return r.locate() }
            guard ["START", "STOP", "PAUSE", "HOME"].contains(v) else { return "unknown \(p)" }
            return r.basicControl(v.lowercased())
        case "LocateCapability/locate":
            return p.uppercased() == "PERFORM" ? r.locate() : "expected PERFORM"
        case "GoToLocationCapability/go": return viaApi("GoToLocationCapability", "goto", p)
        case "MapSegmentationCapability/clean": return viaApi("MapSegmentationCapability", "start_segment_action", p)
        case "ZoneCleaningCapability/start": return viaApi("ZoneCleaningCapability", "clean", p)
        default: return "not supported"
        }
    }

    private func viaApi(_ cap: String, _ action: String, _ json: String) -> String? {
        guard json.hasPrefix("{") else { return "payload must be JSON" }
        let rest = json.dropFirst().trimmingCharacters(in: .whitespaces)
        let body = "{\"action\":\"\(action)\"" + (rest == "}" ? "}" : "," + rest)
        let r = api.handle(method: "PUT", path: "/api/v2/robot/capabilities/\(cap)", body: body)
        return r.status == 200 ? nil : r.body
    }
}
