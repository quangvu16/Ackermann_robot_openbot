//
//  SensorLink.swift
//  OpenBot
//
//  The phone's SensorServer link, shared by GPS Mission, Object Tracking and
//  Autopilot. Mirrors org.openbot.app.robot.sensorlink on Android.
//
//  - Server: this phone streams its sensors and GPS over the WebSocket API of
//    the SensorServer app (https://github.com/UmerCodez/SensorServer), so
//    SensorServer clients on a laptop can read them while the car drives.
//  - Client: this phone reads GPS and compass from a SensorServer app on
//    another phone, and GPS Mission can drive on them.
//
//  The protocol is written from SensorServer's documentation; no SensorServer
//  code is used (that project is GPL-3.0, this one is MIT).
//
//    ws://host:port/sensor/connect?type=android.sensor.accelerometer
//    ws://host:port/sensors/connect?types=["android.sensor.gyroscope",...]
//    ws://host:port/gps
//
//    sensor  {"type":"android.sensor.gyroscope","values":[x,y,z],"timestamp":ns,"accuracy":3}
//    gps     {"longitude":..,"latitude":..,"altitude":..,"bearing":..,"accuracy":..,"speed":..,"time":ms}
//

import Foundation

// MARK: - Protocol

enum SensorLinkProtocol {
    static let defaultPort = 8080
    /// Text a client may send on /gps to get the last fix right away.
    static let getLastKnownLocation = "getLastKnownLocation"
    static let rotationVector = "android.sensor.rotation_vector"

    enum Kind { case sensor, sensors, gps, unknown }

    struct Request {
        let kind: Kind
        let types: [String]
    }

    struct GpsFix {
        var latitude: Double
        var longitude: Double
        var altitude: Double = 0
        var bearing: Double?
        var accuracy: Double?
        var speed: Double?
        var timeMs: Int64 = 0
    }

    /// Parses a request path such as `/sensors/connect?types=%5B%22a%22%5D`.
    static func parse(_ resource: String) -> Request {
        var path = resource
        var query = ""
        if let q = resource.firstIndex(of: "?") {
            path = String(resource[..<q])
            query = String(resource[resource.index(after: q)...])
        }
        while path.count > 1 && path.hasSuffix("/") { path.removeLast() }
        switch path {
        case "/gps":
            return Request(kind: .gps, types: [])
        case "/sensor/connect":
            let type = (queryParam(query, "type") ?? "").trimmingCharacters(in: .whitespaces)
            return type.isEmpty ? Request(kind: .unknown, types: []) : Request(kind: .sensor, types: [type])
        case "/sensors/connect":
            let types = parseTypeArray(queryParam(query, "types"))
            return types.isEmpty ? Request(kind: .unknown, types: []) : Request(kind: .sensors, types: types)
        default:
            return Request(kind: .unknown, types: [])
        }
    }

    static func queryParam(_ query: String, _ name: String) -> String? {
        for pair in query.split(separator: "&", omittingEmptySubsequences: true) {
            let parts = pair.split(separator: "=", maxSplits: 1, omittingEmptySubsequences: false)
            let key = String(parts[0]).removingPercentEncoding ?? String(parts[0])
            if key == name {
                let value = parts.count > 1 ? String(parts[1]) : ""
                return value.removingPercentEncoding ?? value
            }
        }
        return nil
    }

    static func parseTypeArray(_ raw: String?) -> [String] {
        guard var s = raw?.trimmingCharacters(in: .whitespaces) else { return [] }
        if s.hasPrefix("[") { s.removeFirst() }
        if s.hasSuffix("]") { s.removeLast() }
        var out: [String] = []
        for part in s.split(separator: ",") {
            var t = part.trimmingCharacters(in: .whitespaces)
            if t.count >= 2 && t.hasPrefix("\"") && t.hasSuffix("\"") {
                t = String(t.dropFirst().dropLast()).trimmingCharacters(in: .whitespaces)
            }
            if !t.isEmpty && !out.contains(t) { out.append(t) }
        }
        return out
    }

    static func sensorJSON(type: String?, values: [Float], timestampNs: UInt64, accuracy: Int) -> String {
        var s = "{"
        if let type = type { s += "\"type\":\"\(type)\"," }
        s += "\"values\":[" + values.map { number($0) }.joined(separator: ",") + "]"
        s += ",\"timestamp\":\(timestampNs),\"accuracy\":\(accuracy)}"
        return s
    }

    /// OpenBot addition: LiDAR scans on `ws://host:port/lidar`.
    static let lidarPath = "/lidar"
    static let slamPath = "/slam"

    /// One LiDAR revolution, same keys as SensorLinkProtocol.lidarJson on Android:
    /// `{"scanId","bins","forwardOffset","age","ranges":[mm],"sectors":[mm x8]|null,"sectorsAge"}`.
    /// 0 mm means no return / clear sector; `ranges` is empty when only the car's
    /// 8 proximity sectors are known.
    static func lidarJSON(scanId: Int64, rangesMm: [Int], forwardOffset: Double, ageMs: Int,
                          sectorsMm: [Int]?, sectorsAgeMs: Int) -> String {
        let ranges = rangesMm.map(String.init).joined(separator: ",")
        let sectors = sectorsMm.map { "[" + $0.map(String.init).joined(separator: ",") + "]" } ?? "null"
        return "{\"scanId\":\(scanId),\"bins\":\(rangesMm.count),\"forwardOffset\":\(forwardOffset),\"age\":\(ageMs)"
            + ",\"ranges\":[\(ranges)],\"sectors\":\(sectors),\"sectorsAge\":\(sectorsAgeMs)}"
    }

    static func gpsJSON(_ f: GpsFix) -> String {
        "{\"longitude\":\(f.longitude),\"latitude\":\(f.latitude),\"altitude\":\(f.altitude)"
            + ",\"bearing\":\(f.bearing ?? 0),\"accuracy\":\(f.accuracy ?? 0)"
            + ",\"speed\":\(f.speed ?? 0),\"time\":\(f.timeMs)}"
    }

    private static func number(_ v: Float) -> String {
        v.isFinite ? "\(v)" : "0"
    }

    private static func object(_ text: String) -> [String: Any]? {
        guard let data = text.data(using: .utf8) else { return nil }
        return (try? JSONSerialization.jsonObject(with: data)) as? [String: Any]
    }

    private static func double(_ any: Any?) -> Double? {
        (any as? NSNumber)?.doubleValue
    }

    /// nil if the text is not a GPS message (latitude and longitude are required).
    static func parseGPS(_ text: String) -> GpsFix? {
        guard let o = object(text),
              let lat = double(o["latitude"]), let lon = double(o["longitude"]),
              (-90...90).contains(lat), (-180...180).contains(lon)
        else { return nil }
        var f = GpsFix(latitude: lat, longitude: lon)
        f.altitude = double(o["altitude"]) ?? 0
        f.bearing = double(o["bearing"])
        f.accuracy = double(o["accuracy"])
        f.speed = double(o["speed"])
        f.timeMs = Int64(double(o["time"]) ?? 0)
        return f
    }

    /// The values array of a sensor message, nil if there is none.
    static func parseSensorValues(_ text: String) -> [Double]? {
        guard let o = object(text), let values = o["values"] as? [Any] else { return nil }
        return values.compactMap { double($0) }
    }

    /// `192.168.43.1`, `192.168.43.1:8080` or `ws://host:port/` to `ws://host:port`.
    static func baseURL(_ address: String) -> String? {
        var a = address.trimmingCharacters(in: .whitespacesAndNewlines)
        for prefix in ["ws://", "http://"] where a.lowercased().hasPrefix(prefix) {
            a = String(a.dropFirst(prefix.count))
        }
        if let slash = a.firstIndex(of: "/") { a = String(a[..<slash]) }
        if a.isEmpty || a.hasPrefix(":") { return nil }
        let hasPort = a.hasPrefix("[") ? a.contains("]:") : a.contains(":")
        if !hasPort { a += ":\(defaultPort)" }
        return "ws://" + a
    }

    /// Android's SensorManager.getRotationMatrixFromVector: row-major device-to-world
    /// (X east, Y north, Z up).
    static func rotationMatrix(fromVector v: [Double]) -> [Double]? {
        guard v.count >= 3 else { return nil }
        let x = v[0], y = v[1], z = v[2]
        let w = v.count >= 4 ? v[3] : max(0, 1 - x * x - y * y - z * z).squareRoot()
        let sqX = 2 * x * x, sqY = 2 * y * y, sqZ = 2 * z * z
        let xy = 2 * x * y, zw = 2 * z * w, xz = 2 * x * z, yw = 2 * y * w, yz = 2 * y * z, xw = 2 * x * w
        return [1 - sqY - sqZ, xy - zw, xz + yw,
                xy + zw, 1 - sqX - sqZ, yz - xw,
                xz - yw, yz + xw, 1 - sqX - sqY]
    }

    /// Heading of the car's nose from magnetic north, as GpsNavLocationProvider.java does it:
    /// phone upright with the back camera forward (device -Z), or lying flat (top edge forward).
    static func heading(fromRotationMatrix r: [Double]) -> Double {
        let east: Double, north: Double
        if abs(r[8]) < 0.7 {
            east = -r[2]; north = -r[5]
        } else {
            east = r[1]; north = r[4]
        }
        var h = atan2(east, north) * 180 / .pi
        if h < 0 { h += 360 }
        return h
    }
}

// MARK: - Settings

struct SensorLinkSettings {
    enum Mode: String { case off, server, client }

    var mode: Mode = .off
    var port: Int = SensorLinkProtocol.defaultPort
    var address: String = ""
    /// Client mode: GPS Mission drives on the other phone's GPS and compass.
    var useRemoteGps = true

    private static let modeKey = "sensor_link_mode"
    private static let portKey = "sensor_link_port"
    private static let addressKey = "sensor_link_address"
    private static let useGpsKey = "sensor_link_use_gps"

    static func load() -> SensorLinkSettings {
        let d = UserDefaults.standard
        var s = SensorLinkSettings()
        s.mode = Mode(rawValue: d.string(forKey: modeKey) ?? "") ?? .off
        let port = d.integer(forKey: portKey)
        s.port = isValidPort(port) ? port : SensorLinkProtocol.defaultPort
        s.address = d.string(forKey: addressKey) ?? ""
        s.useRemoteGps = d.object(forKey: useGpsKey) as? Bool ?? true
        return s
    }

    func save() {
        let d = UserDefaults.standard
        d.set(mode.rawValue, forKey: Self.modeKey)
        d.set(port, forKey: Self.portKey)
        d.set(address.trimmingCharacters(in: .whitespaces), forKey: Self.addressKey)
        d.set(useRemoteGps, forKey: Self.useGpsKey)
    }

    static func isValidPort(_ port: Int) -> Bool { (1024...65535).contains(port) }
}

// MARK: - Manager

/// GPS Mission's hook for the other phone's data (client mode, "use its GPS" on).
protocol SensorLinkRemoteDelegate: AnyObject {
    func sensorLink(didReceiveRemoteFix fix: SensorLinkProtocol.GpsFix)
    func sensorLink(didReceiveRemoteRotationVector values: [Double])
}

extension Notification.Name {
    static let sensorLinkStatusChanged = Notification.Name("sensorLinkStatusChanged")
}

/// Runs the link while one of the three screens is on screen. Main thread only.
final class SensorLink {
    static let shared = SensorLink()
    /// A remote fix older than this no longer counts; the phone's own GPS takes over.
    static let remoteStale: TimeInterval = 3

    weak var remoteDelegate: SensorLinkRemoteDelegate?

    private(set) var settings = SensorLinkSettings()
    private var attachCount = 0
    private var server: SensorLinkServer?
    private var client: SensorLinkClient?

    private var serverError: String?
    private var serverClients = 0
    private var clientConnected = false
    private var remoteAccuracy: Double?
    private var remoteFixUptime: TimeInterval = 0

    // Browser settings: the camera everywhere, plus whatever the open screen adds.
    let remoteSettings = RemoteSettings()
    private(set) var screenName = "OpenBot"
    private var screenProvider: RemoteSettingsProvider?

    private init() {
        remoteSettings.add(CameraHub.shared)
    }

    /// The open screen's name and settings for the browser page. Main thread.
    func setScreen(_ name: String, _ provider: RemoteSettingsProvider) {
        if let old = screenProvider { remoteSettings.remove(old) }
        screenName = name
        screenProvider = provider
        remoteSettings.add(provider)
    }

    func clearScreen(_ provider: RemoteSettingsProvider) {
        guard screenProvider === provider else { return }
        remoteSettings.remove(provider)
        screenProvider = nil
        screenName = "OpenBot"
    }

    func attach() {
        attachCount += 1
        if attachCount == 1 { start() }
    }

    func detach() {
        guard attachCount > 0 else { return }
        attachCount -= 1
        if attachCount == 0 { stop() }
    }

    /// Apply saved settings now, if a screen has the link running.
    func restart() {
        settings = SensorLinkSettings.load()
        guard attachCount > 0 else { notify(); return }
        stop()
        start()
    }

    var isRemoteFixFresh: Bool {
        client != nil && ProcessInfo.processInfo.systemUptime - remoteFixUptime < Self.remoteStale
    }

    /// Short text for the navigation bar button.
    var shortStatus: String {
        switch settings.mode {
        case .off: return L10n.string("sensor_link_mode_off")
        case .server: return serverError == nil ? "\(serverClients)" : "!"
        case .client: return clientConnected ? "✓" : "…"
        }
    }

    var isActive: Bool { settings.mode != .off && attachCount > 0 }

    /// One line describing what the link is doing right now.
    var statusText: String {
        switch settings.mode {
        case .off:
            return L10n.string("sensor_link_off")
        case .server:
            if let error = serverError {
                return String(format: L10n.string("sensor_link_server_failed"), error)
            }
            guard let ip = SensorLink.localIPv4() else {
                return String(format: L10n.string("sensor_link_server_no_ip"), Int32(settings.port))
            }
            return String(format: L10n.string("sensor_link_server"), "http://\(ip):\(settings.port)", Int32(serverClients))
        case .client:
            guard let base = SensorLinkProtocol.baseURL(settings.address) else {
                return L10n.string("sensor_link_client_no_address")
            }
            let host = String(base.dropFirst("ws://".count))
            if !clientConnected {
                return String(format: L10n.string("sensor_link_client_connecting"), host)
            }
            if let accuracy = remoteAccuracy,
               ProcessInfo.processInfo.systemUptime - remoteFixUptime < Self.remoteStale {
                return String(format: L10n.string("sensor_link_client_gps"), host, accuracy)
            }
            return String(format: L10n.string("sensor_link_client_connected"), host)
        }
    }

    private func start() {
        settings = SensorLinkSettings.load()
        resetStatus()
        switch settings.mode {
        case .server:
            let server = SensorLinkServer(port: settings.port, remoteSettings: remoteSettings,
                                          screenName: { [weak self] in self?.screenName ?? "OpenBot" })
            server.onStarted = { [weak self] in
                self?.serverError = nil
                self?.notify()
            }
            server.onFailed = { [weak self] reason in
                self?.serverError = reason
                self?.notify()
            }
            server.onClientCount = { [weak self] count in
                self?.serverClients = count
                self?.notify()
            }
            self.server = server
            server.start()
        case .client:
            guard let base = SensorLinkProtocol.baseURL(settings.address) else { break }
            let client = SensorLinkClient(baseURL: base, wantHeading: settings.useRemoteGps)
            client.onConnected = { [weak self] connected in
                self?.clientConnected = connected
                self?.notify()
            }
            client.onFix = { [weak self] fix in
                guard let self = self else { return }
                self.remoteAccuracy = fix.accuracy
                self.remoteFixUptime = ProcessInfo.processInfo.systemUptime
                if self.settings.useRemoteGps {
                    self.remoteDelegate?.sensorLink(didReceiveRemoteFix: fix)
                }
                self.notify()
            }
            client.onRotationVector = { [weak self] values in
                guard let self = self, self.settings.useRemoteGps else { return }
                self.remoteDelegate?.sensorLink(didReceiveRemoteRotationVector: values)
            }
            self.client = client
            client.start()
        case .off:
            break
        }
        notify()
    }

    private func stop() {
        server?.stop()
        server = nil
        client?.stop()
        client = nil
        resetStatus()
        notify()
    }

    private func resetStatus() {
        serverError = nil
        serverClients = 0
        clientConnected = false
        remoteAccuracy = nil
        remoteFixUptime = 0
    }

    private func notify() {
        NotificationCenter.default.post(name: .sensorLinkStatusChanged, object: self)
    }

    /// The address other devices on the same Wi-Fi or hotspot reach this phone at.
    static func localIPv4() -> String? {
        var ifaddr: UnsafeMutablePointer<ifaddrs>?
        guard getifaddrs(&ifaddr) == 0, let first = ifaddr else { return nil }
        defer { freeifaddrs(ifaddr) }
        var fallback: String?
        for ptr in sequence(first: first, next: { $0.pointee.ifa_next }) {
            let flags = Int32(ptr.pointee.ifa_flags)
            guard (flags & IFF_UP) != 0, (flags & IFF_LOOPBACK) == 0,
                  let addr = ptr.pointee.ifa_addr, addr.pointee.sa_family == UInt8(AF_INET)
            else { continue }
            var host = [CChar](repeating: 0, count: Int(NI_MAXHOST))
            guard getnameinfo(addr, socklen_t(addr.pointee.sa_len), &host, socklen_t(host.count),
                              nil, 0, NI_NUMERICHOST) == 0 else { continue }
            let ip = String(cString: host)
            let name = String(cString: ptr.pointee.ifa_name)
            // en0 is Wi-Fi; bridge100 is the Personal Hotspot.
            if name == "en0" || name.hasPrefix("bridge") { return ip }
            if fallback == nil && !name.hasPrefix("pdp_ip") && !name.hasPrefix("utun") { fallback = ip }
        }
        return fallback
    }
}

// MARK: - Client

/// Reads GPS (and the rotation vector, for a compass heading) from a
/// SensorServer app on another phone. Reconnects every 3 s until stopped.
/// Callbacks arrive on the main queue.
final class SensorLinkClient: NSObject, URLSessionWebSocketDelegate {
    var onConnected: ((Bool) -> Void)?
    var onFix: ((SensorLinkProtocol.GpsFix) -> Void)?
    var onRotationVector: (([Double]) -> Void)?

    private let baseURL: String
    private let wantHeading: Bool
    private var session: URLSession!
    private var gpsTask: URLSessionWebSocketTask?
    private var headingTask: URLSessionWebSocketTask?
    private var running = false
    private var connected = false

    init(baseURL: String, wantHeading: Bool) {
        self.baseURL = baseURL
        self.wantHeading = wantHeading
        super.init()
        let config = URLSessionConfiguration.default
        config.timeoutIntervalForRequest = 10
        session = URLSession(configuration: config, delegate: self, delegateQueue: .main)
    }

    func start() {
        running = true
        connectGps()
        if wantHeading { connectHeading() }
    }

    func stop() {
        running = false
        gpsTask?.cancel(with: .normalClosure, reason: nil)
        headingTask?.cancel(with: .normalClosure, reason: nil)
        gpsTask = nil
        headingTask = nil
        session.invalidateAndCancel()
    }

    private func connectGps() {
        guard running, let url = URL(string: baseURL + "/gps") else { return }
        let task = session.webSocketTask(with: url)
        gpsTask = task
        task.resume()
        receive(task, isGps: true)
    }

    private func connectHeading() {
        let path = "/sensor/connect?type=" + SensorLinkProtocol.rotationVector
        guard running, let url = URL(string: baseURL + path) else { return }
        let task = session.webSocketTask(with: url)
        headingTask = task
        task.resume()
        receive(task, isGps: false)
    }

    private func receive(_ task: URLSessionWebSocketTask, isGps: Bool) {
        task.receive { [weak self] result in
            DispatchQueue.main.async {
                guard let self = self, self.running else { return }
                switch result {
                case .success(let message):
                    var text: String?
                    switch message {
                    case .string(let s): text = s
                    case .data(let d): text = String(data: d, encoding: .utf8)
                    @unknown default: text = nil
                    }
                    if let text = text {
                        if isGps, let fix = SensorLinkProtocol.parseGPS(text) {
                            self.setConnected(true)
                            self.onFix?(fix)
                        } else if !isGps, let values = SensorLinkProtocol.parseSensorValues(text), values.count >= 3 {
                            self.onRotationVector?(values)
                        }
                    }
                    self.receive(task, isGps: isGps)
                case .failure:
                    self.retry(isGps: isGps, task: task)
                }
            }
        }
    }

    private func retry(isGps: Bool, task: URLSessionWebSocketTask) {
        // Only the current task reconnects; an old one failing after a reconnect is ignored.
        if isGps {
            guard task === gpsTask else { return }
            setConnected(false)
            DispatchQueue.main.asyncAfter(deadline: .now() + 3) { [weak self] in self?.connectGps() }
        } else {
            guard task === headingTask else { return }
            DispatchQueue.main.asyncAfter(deadline: .now() + 3) { [weak self] in self?.connectHeading() }
        }
    }

    private func setConnected(_ value: Bool) {
        guard running, connected != value else { return }
        connected = value
        onConnected?(value)
    }

    func urlSession(_ session: URLSession, webSocketTask: URLSessionWebSocketTask,
                    didOpenWithProtocol protocol: String?) {
        guard webSocketTask === gpsTask else { return }
        setConnected(true)
        webSocketTask.send(.string(SensorLinkProtocol.getLastKnownLocation)) { _ in }
    }
}
