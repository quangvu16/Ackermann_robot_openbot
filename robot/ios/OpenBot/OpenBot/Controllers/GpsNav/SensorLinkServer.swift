//
//  SensorLinkServer.swift
//  OpenBot
//
//  A SensorServer-compatible WebSocket server (see SensorLink.swift for the
//  API). Built on Network.framework with the WebSocket handshake and framing
//  done here, because NWProtocolWebSocket does not tell a server which path
//  the client asked for - and the path is how SensorServer picks the sensor.
//
//  iOS sensors are published under the Android names SensorServer clients
//  expect, converted to Android units and signs:
//    android.sensor.accelerometer        m/s², gravity included
//    android.sensor.gyroscope            rad/s
//    android.sensor.magnetic_field       µT
//    android.sensor.gravity              m/s²
//    android.sensor.linear_acceleration  m/s²
//    android.sensor.rotation_vector      x, y, z, w (world: east, north, up)
//    android.sensor.pressure             hPa
//  Sensors run only while a client is connected to them.
//

import CoreLocation
import CoreMotion
import CryptoKit
import Foundation
import Network

final class SensorLinkServer: RemoteSettingsProvider {
    /// All three are called on the main queue.
    var onStarted: (() -> Void)?
    var onFailed: ((String) -> Void)?
    var onClientCount: ((Int) -> Void)?

    private let port: Int
    private let queue = DispatchQueue(label: "SensorLinkServer")
    private var listener: NWListener?
    private var connections: [ObjectIdentifier: WebSocketConnection] = [:]
    private lazy var sensors = SensorLinkSensors(queue: queue) { [weak self] type, values, ts, accuracy in
        self?.broadcast(type: type, values: values, timestampNs: ts, accuracy: accuracy)
    } gps: { [weak self] fix in
        self?.broadcastGps(fix)
    }

    // Browser page: settings over /control, camera over /camera.mjpg (see SensorLinkRemote.swift).
    private let remoteSettings: RemoteSettings
    private let screenName: () -> String
    private var pushTimer: Timer?            // main queue
    private var sensorRate = "game"           // main queue
    // LiDAR (/lidar): our own UDP listener for the car's scans while someone watches (on `queue`).
    private var lidarAcquired = false
    private var lidarTimer: DispatchSourceTimer?
    private var lidarSentId: Int64 = -2
    private var lidarSentUptime: TimeInterval = 0
    // Indoor SLAM (/slam): the screen's state every 0.5 s, the map when it changes (on `queue`).
    private var slamTimer: DispatchSourceTimer?
    private var slamForceMap = false
    private var slamSentVersion = Int.min
    private var slamMapUptime: TimeInterval = 0

    init(port: Int, remoteSettings: RemoteSettings, screenName: @escaping () -> String) {
        self.port = port
        self.remoteSettings = remoteSettings
        self.screenName = screenName
    }

    func start() {
        remoteSettings.add(self)
        CameraHub.shared.onFrame = { [weak self] jpeg in
            self?.queue.async { self?.sendFrame(jpeg) }
        }
        let status = CLLocationManager().authorizationStatus
        let authorized = status == .authorizedWhenInUse || status == .authorizedAlways
        queue.async { self.sensors.canUseLocation = authorized }
        guard let nwPort = NWEndpoint.Port(rawValue: UInt16(clamping: port)) else {
            report { self.onFailed?("bad port") }
            return
        }
        do {
            let params = NWParameters.tcp
            params.allowLocalEndpointReuse = true
            let listener = try NWListener(using: params, on: nwPort)
            listener.stateUpdateHandler = { [weak self] state in
                switch state {
                case .ready:
                    self?.report { self?.onStarted?() }
                case .failed(let error):
                    self?.report { self?.onFailed?(error.localizedDescription) }
                default:
                    break
                }
            }
            listener.newConnectionHandler = { [weak self] nw in
                guard let self = self else { return }
                let c = WebSocketConnection(nw, queue: self.queue)
                c.onOpen = { [weak self] c, path in self?.opened(c, path: path) }
                c.onText = { [weak self] c, text in self?.received(c, text: text) }
                c.onClose = { [weak self] c in self?.closed(c) }
                c.httpHandler = { [weak self] path in self?.httpResponse(path) ?? HTTPReply(status: 503, contentType: "text/plain", body: Data()) }
                c.onStream = { [weak self] c in self?.startedStream(c) }
                c.onSnapshot = { [weak self] c in self?.wantsSnapshot(c) }
                // Held here from the start: its own callbacks only hold it weakly.
                self.connections[ObjectIdentifier(c)] = c
                c.start()
            }
            self.listener = listener
            listener.start(queue: queue)
        } catch {
            report { self.onFailed?(error.localizedDescription) }
        }
    }

    func stop() {
        remoteSettings.remove(self)
        pushTimer?.invalidate()
        pushTimer = nil
        CameraHub.shared.onFrame = nil
        CameraHub.shared.setViewers(0)
        queue.async {
            self.listener?.cancel()
            self.listener = nil
            self.connections.values.forEach { $0.close(code: 1001, reason: "server stopped") }
            self.connections.removeAll()
            self.sensors.stopAll()
            self.stopLidar()
            self.stopSlam()
        }
    }

    private func report(_ block: @escaping () -> Void) {
        DispatchQueue.main.async(execute: block)
    }

    // MARK: Connections (all on `queue`)

    private func opened(_ c: WebSocketConnection, path: String) {
        if path == SensorLinkProtocol.lidarPath || path.hasPrefix(SensorLinkProtocol.lidarPath + "?") {
            c.isLidar = true
            c.registered = true
            startLidar()
            return
        }
        if path == SensorLinkProtocol.slamPath || path.hasPrefix(SensorLinkProtocol.slamPath + "?") {
            c.isSlam = true
            c.registered = true
            startSlam()
            return
        }
        if path == "/control" || path.hasPrefix("/control?") {
            c.isControl = true
            c.registered = true
            report { self.pushState() }
            return
        }
        let request = SensorLinkProtocol.parse(path)
        switch request.kind {
        case .gps:
            guard sensors.canUseLocation else {
                c.close(code: 4002, reason: "Location permission not granted on the phone")
                return
            }
            c.wantsGps = true
            sensors.retainGps()
        case .sensor, .sensors:
            if let missing = request.types.first(where: { !sensors.isAvailable($0) }) {
                c.close(code: 4001, reason: "Sensor not available: \(missing)")
                return
            }
            c.multi = request.kind == .sensors
            c.types = Set(request.types)
            request.types.forEach { sensors.retain($0) }
        case .unknown:
            c.close(code: 4000, reason: "Unknown path \(path)")
            return
        }
        c.registered = true
        reportCount()
    }

    private func received(_ c: WebSocketConnection, text: String) {
        if c.isSlam {
            SlamHub.shared.command(text)
            return
        }
        if c.isControl {
            report {
                self.remoteSettings.apply(text)
                self.pushState()
            }
            return
        }
        if c.wantsGps, text.trimmingCharacters(in: .whitespacesAndNewlines) == SensorLinkProtocol.getLastKnownLocation,
           let fix = sensors.lastFix {
            c.send(SensorLinkProtocol.gpsJSON(fix))
        }
    }

    private func closed(_ c: WebSocketConnection) {
        guard connections.removeValue(forKey: ObjectIdentifier(c)) != nil, c.registered else { return }
        if c.isStream || c.isSnapshot { updateViewers(); return }
        if c.isControl { return }
        if c.isSlam {
            if !connections.values.contains(where: { $0.registered && $0.isSlam }) { stopSlam() }
            return
        }
        if c.isLidar {
            if !connections.values.contains(where: { $0.registered && $0.isLidar }) { stopLidar() }
            return
        }
        if c.wantsGps { sensors.releaseGps() }
        c.types.forEach { sensors.release($0) }
        reportCount()
    }

    private func reportCount() {
        let count = connections.values.filter { $0.registered && !$0.isControl && !$0.isStream && !$0.isSnapshot && !$0.isLidar && !$0.isSlam }.count
        report { self.onClientCount?(count) }
    }

    private func broadcast(type: String, values: [Float], timestampNs: UInt64, accuracy: Int) {
        var single: String?
        var multi: String?
        for c in connections.values where c.registered && c.types.contains(type) {
            if c.multi {
                if multi == nil {
                    multi = SensorLinkProtocol.sensorJSON(type: type, values: values, timestampNs: timestampNs, accuracy: accuracy)
                }
                c.send(multi!)
            } else {
                if single == nil {
                    single = SensorLinkProtocol.sensorJSON(type: nil, values: values, timestampNs: timestampNs, accuracy: accuracy)
                }
                c.send(single!)
            }
        }
    }

    private func broadcastGps(_ fix: SensorLinkProtocol.GpsFix) {
        let text = SensorLinkProtocol.gpsJSON(fix)
        for c in connections.values where c.registered && c.wantsGps { c.send(text) }
    }

    // MARK: LiDAR (on `queue`)

    private func startLidar() {
        lidarSentId = -2 // the newcomer gets the current picture straight away
        guard lidarTimer == nil else { return }
        SharedLidar.acquire() // shared with the avoidance map: one UDP listener
        lidarAcquired = true
        let timer = DispatchSource.makeTimerSource(queue: queue)
        timer.schedule(deadline: .now(), repeating: .milliseconds(100))
        timer.setEventHandler { [weak self] in self?.pollLidar() }
        timer.resume()
        lidarTimer = timer
    }

    private func stopLidar() {
        lidarTimer?.cancel()
        lidarTimer = nil
        if lidarAcquired { SharedLidar.release() }
        lidarAcquired = false
    }

    /// Every 100 ms: a new revolution as soon as it lands, otherwise a refresh once a second.
    private func pollLidar() {
        let scanner = SharedLidar.scanner
        let scan = scanner.latestScan
        let id = scan.map { Int64($0.scanId) } ?? -1
        let now = ProcessInfo.processInfo.systemUptime
        guard id != lidarSentId || now - lidarSentUptime >= 1 else { return }
        lidarSentId = id
        lidarSentUptime = now
        let ranges = scan?.rangesMm.map { Int($0) } ?? []
        let age = scan == nil ? 0 : Int(scanner.ageMs())
        // The car's 8 proximity sectors are fed on the main queue by Bluetooth; read them there.
        DispatchQueue.main.async {
            let proximity = bluetoothDataController.shared.driveGate.proximity
            var sectors: [Int]?
            var sectorsAge = 0
            if proximity.hasEverReported() {
                sectors = proximity.sectors().map { $0.isFinite ? Int(($0 * 1000).rounded()) : 0 }
                sectorsAge = Int(proximity.ageMs())
            }
            let json = SensorLinkProtocol.lidarJSON(scanId: id, rangesMm: ranges, forwardOffset: 0, ageMs: age,
                                                    sectorsMm: sectors, sectorsAgeMs: sectorsAge)
            self.queue.async {
                for c in self.connections.values where c.registered && c.isLidar { c.send(json) }
            }
        }
    }

    // MARK: Indoor SLAM (on `queue`)

    private func startSlam() {
        slamForceMap = true // the newcomer needs the whole map
        guard slamTimer == nil else { return }
        let timer = DispatchSource.makeTimerSource(queue: queue)
        timer.schedule(deadline: .now(), repeating: .milliseconds(500))
        timer.setEventHandler { [weak self] in self?.pollSlam() }
        timer.resume()
        slamTimer = timer
    }

    private func stopSlam() {
        slamTimer?.cancel()
        slamTimer = nil
    }

    private func pollSlam() {
        let now = ProcessInfo.processInfo.systemUptime
        let version = SlamHub.shared.mapVersion
        let withMap = slamForceMap || (version != slamSentVersion && now - slamMapUptime >= 2)
        let json = SlamHub.shared.json(withMap: withMap)
        if withMap { slamForceMap = false; slamSentVersion = version; slamMapUptime = now }
        for c in connections.values where c.registered && c.isSlam { c.send(json) }
    }

    // MARK: Camera (on `queue`)

    private func updateViewers() {
        CameraHub.shared.setViewers(connections.values.filter { $0.registered && ($0.isStream || $0.isSnapshot) }.count)
    }

    private func startedStream(_ c: WebSocketConnection) {
        c.isStream = true
        c.registered = true
        updateViewers()
    }

    private func wantsSnapshot(_ c: WebSocketConnection) {
        let hub = CameraHub.shared
        if let jpeg = hub.latest, ProcessInfo.processInfo.systemUptime - hub.latestUptime < 1.5 {
            c.respond(status: 200, contentType: "image/jpeg", body: jpeg)
            return
        }
        c.isSnapshot = true
        c.registered = true
        updateViewers()
        queue.asyncAfter(deadline: .now() + 4) { [weak self, weak c] in
            guard let self = self, let c = c, c.isSnapshot else { return }
            c.isSnapshot = false
            let why = hub.reason ?? "No camera picture yet"
            c.respond(status: 503, contentType: "text/plain", body: Data((why + "\n").utf8))
            self.updateViewers()
        }
    }

    private func sendFrame(_ jpeg: Data) {
        let head = Data("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: \(jpeg.count)\r\n\r\n".utf8)
        let part = head + jpeg + Data("\r\n".utf8)
        for c in connections.values where c.registered {
            if c.isStream, c.canTakeMore {
                c.sendRaw(part) // a slow browser skips frames instead of piling them up
            } else if c.isSnapshot {
                c.isSnapshot = false
                c.respond(status: 200, contentType: "image/jpeg", body: jpeg)
            }
        }
    }

    // MARK: Settings (main queue)

    /// Sends the settings state to every /control client; repeats every 2 s so phone-side changes show too.
    private func pushState() {
        let state = remoteSettings.stateJSON(screen: screenName(), camera: CameraHub.shared.reason ?? "")
        queue.async {
            let clients = self.connections.values.filter { $0.registered && $0.isControl }
            clients.forEach { $0.send(state) }
            let any = !clients.isEmpty
            DispatchQueue.main.async {
                if any && self.pushTimer == nil {
                    self.pushTimer = Timer.scheduledTimer(withTimeInterval: 2, repeats: true) { [weak self] _ in self?.pushState() }
                } else if !any {
                    self.pushTimer?.invalidate()
                    self.pushTimer = nil
                }
            }
        }
    }

    func addSettings(_ out: inout [RemoteSetting]) {
        out.append(.choice("sensors.rate", "Sensors", "Sensor speed (ui ≈ 15/s, game ≈ 50/s, fastest)", ["ui", "game", "fastest"],
                           get: { [unowned self] in self.sensorRate },
                           set: { [unowned self] rate in
                               self.sensorRate = rate
                               let interval = rate == "ui" ? 1.0 / 15 : rate == "fastest" ? 1.0 / 100 : 1.0 / 50
                               self.queue.async { self.sensors.setInterval(interval) }
                           }))
    }

    /// Plain HTTP: the live dashboard at `/` and the sensor list it reads at `/sensors`.
    private func httpResponse(_ path: String) -> HTTPReply {
        let route = path.split(separator: "?", maxSplits: 1).first.map(String.init) ?? path
        switch route {
        case "/", "/index.html":
            if let url = Bundle.main.url(forResource: "SensorLinkDashboard", withExtension: "html"),
               let page = try? Data(contentsOf: url) {
                return HTTPReply(status: 200, contentType: "text/html; charset=utf-8", body: page)
            }
            return HTTPReply(status: 500, contentType: "text/plain", body: Data("Dashboard missing from the app\n".utf8))
        case "/camera.mjpg":
            return HTTPReply(status: 200, contentType: "multipart/x-mixed-replace; boundary=frame", body: Data(), mode: .stream)
        case "/camera.jpg":
            return HTTPReply(status: 200, contentType: "image/jpeg", body: Data(), mode: .snapshot)
        case "/sensors":
            let list = SensorLinkSensors.allTypes.filter { sensors.isAvailable($0) }.map {
                "{\"type\":\"\($0)\",\"name\":\"\(SensorLinkSensors.displayName($0))\"}"
            }
            return HTTPReply(status: 200, contentType: "application/json", body: Data(("[" + list.joined(separator: ",") + "]").utf8))
        default:
            return HTTPReply(status: 404, contentType: "text/plain", body: Data("Not found\n".utf8))
        }
    }
}

// MARK: - WebSocket connection

/// A plain HTTP answer.
private struct HTTPReply {
    enum Mode { case full, stream, snapshot }
    let status: Int
    let contentType: String
    let body: Data
    var mode: Mode = .full
}

/// One client: HTTP upgrade, then RFC 6455 frames. Server frames are never
/// masked; client frames always are.
private final class WebSocketConnection {
    var onOpen: ((WebSocketConnection, String) -> Void)?
    var onText: ((WebSocketConnection, String) -> Void)?
    var onClose: ((WebSocketConnection) -> Void)?
    var httpHandler: ((String) -> HTTPReply)?

    /// True once the client asked for something we serve.
    var registered = false
    var wantsGps = false
    var multi = false
    var isControl = false   // /control settings socket
    var isLidar = false     // /lidar scans
    var isSlam = false      // /slam Indoor SLAM
    var isStream = false    // /camera.mjpg
    var isSnapshot = false  // /camera.jpg waiting for a frame
    var onStream: ((WebSocketConnection) -> Void)?
    var onSnapshot: ((WebSocketConnection) -> Void)?
    var types: Set<String> = []

    private let connection: NWConnection
    private let queue: DispatchQueue
    private var buffer = Data()
    private var upgraded = false
    private var closed = false
    /// Frames waiting to be written; slow clients are dropped rather than buffered forever.
    private var pendingSends = 0
    private static let maxPendingSends = 200
    private static let maxFrame = 1 << 20

    init(_ connection: NWConnection, queue: DispatchQueue) {
        self.connection = connection
        self.queue = queue
    }

    func start() {
        connection.stateUpdateHandler = { [weak self] state in
            switch state {
            case .failed, .cancelled:
                self?.finish()
            default:
                break
            }
        }
        connection.start(queue: queue)
        receive()
    }

    private func receive() {
        connection.receive(minimumIncompleteLength: 1, maximumLength: 65536) { [weak self] data, _, isComplete, error in
            guard let self = self else { return }
            if let data = data, !data.isEmpty {
                self.buffer.append(data)
                if self.upgraded { self.readFrames() } else { self.readHandshake() }
            }
            if isComplete || error != nil {
                self.finish()
            } else if !self.closed {
                self.receive()
            }
        }
    }

    // MARK: Handshake

    private func readHandshake() {
        guard let end = buffer.range(of: Data("\r\n\r\n".utf8)) else {
            if buffer.count > 16384 { finish() }
            return
        }
        let head = String(decoding: buffer[buffer.startIndex..<end.lowerBound], as: UTF8.self)
        buffer.removeSubrange(buffer.startIndex..<end.upperBound)
        let lines = head.components(separatedBy: "\r\n")
        let requestLine = lines.first?.split(separator: " ") ?? []
        guard requestLine.count >= 2, requestLine[0] == "GET" else {
            writeRaw("HTTP/1.1 405 Method Not Allowed\r\nConnection: close\r\n\r\n", thenClose: true)
            return
        }
        let path = String(requestLine[1])
        var headers: [String: String] = [:]
        for line in lines.dropFirst() {
            guard let colon = line.firstIndex(of: ":") else { continue }
            let name = line[..<colon].trimmingCharacters(in: .whitespaces).lowercased()
            headers[name] = line[line.index(after: colon)...].trimmingCharacters(in: .whitespaces)
        }
        guard headers["upgrade"]?.lowercased() == "websocket", let key = headers["sec-websocket-key"] else {
            // A plain browser visit: the page, the sensor list, or the camera.
            let reply = httpHandler?(path) ?? HTTPReply(status: 404, contentType: "text/plain", body: Data())
            switch reply.mode {
            case .full:
                respond(status: reply.status, contentType: reply.contentType, body: reply.body)
            case .stream:
                let head = "HTTP/1.1 200 OK\r\nContent-Type: \(reply.contentType)\r\nCache-Control: no-store\r\n"
                    + "Access-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n"
                write(Data(head.utf8), thenClose: false)
                onStream?(self)
            case .snapshot:
                onSnapshot?(self)
            }
            return
        }
        let digest = Insecure.SHA1.hash(data: Data((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").utf8))
        let accept = Data(digest).base64EncodedString()
        writeRaw("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
            + "Sec-WebSocket-Accept: \(accept)\r\n\r\n", thenClose: false)
        upgraded = true
        onOpen?(self, path)
        if !buffer.isEmpty { readFrames() }
    }

    // MARK: Frames

    private func readFrames() {
        while !closed {
            let bytes = [UInt8](buffer.prefix(14))
            guard bytes.count >= 2 else { return }
            let opcode = bytes[0] & 0x0F
            let masked = bytes[1] & 0x80 != 0
            var length = Int(bytes[1] & 0x7F)
            var offset = 2
            if length == 126 {
                guard bytes.count >= 4 else { return }
                length = Int(bytes[2]) << 8 | Int(bytes[3])
                offset = 4
            } else if length == 127 {
                guard bytes.count >= 10 else { return }
                var l: UInt64 = 0
                for i in 2..<10 { l = l << 8 | UInt64(bytes[i]) }
                guard l <= UInt64(Self.maxFrame) else { finish(); return }
                length = Int(l)
                offset = 10
            }
            guard length <= Self.maxFrame else { finish(); return }
            let maskOffset = offset
            if masked { offset += 4 }
            guard buffer.count >= offset + length else { return }
            let all = [UInt8](buffer.prefix(offset + length))
            var payload = Array(all[offset..<(offset + length)])
            if masked {
                let mask = Array(all[maskOffset..<(maskOffset + 4)])
                for i in 0..<payload.count { payload[i] ^= mask[i % 4] }
            }
            buffer.removeFirst(offset + length)

            switch opcode {
            case 0x1: // text (fragmented messages are not used by SensorServer clients)
                onText?(self, String(decoding: payload, as: UTF8.self))
            case 0x8: // close: echo it, then hang up
                writeFrame(opcode: 0x8, payload: Array(payload.prefix(2)), thenClose: true)
                return
            case 0x9: // ping
                writeFrame(opcode: 0xA, payload: payload, thenClose: false)
            default:
                break
            }
        }
    }

    /// A complete HTTP answer, then hang up.
    func respond(status: Int, contentType: String, body: Data) {
        let reason = status == 200 ? "OK" : status == 404 ? "Not Found" : "Error"
        let head = "HTTP/1.1 \(status) \(reason)\r\nContent-Type: \(contentType)\r\n"
            + "Content-Length: \(body.count)\r\nCache-Control: no-store\r\n"
            + "Access-Control-Allow-Origin: *\r\nConnection: close\r\n\r\n"
        write(Data(head.utf8) + body, thenClose: true)
    }

    /// True while fewer than 3 writes are in flight (for skipping camera frames).
    var canTakeMore: Bool { !closed && pendingSends < 3 }

    func sendRaw(_ data: Data) {
        write(data, thenClose: false)
    }

    func send(_ text: String) {
        guard upgraded, !closed else { return }
        if pendingSends > Self.maxPendingSends {
            close(code: 1008, reason: "client too slow")
            return
        }
        writeFrame(opcode: 0x1, payload: Array(text.utf8), thenClose: false)
    }

    func close(code: UInt16, reason: String) {
        guard !closed else { return }
        if upgraded {
            writeFrame(opcode: 0x8, payload: [UInt8(code >> 8), UInt8(code & 0xFF)] + Array(reason.utf8.prefix(120)), thenClose: true)
        } else {
            finish()
        }
    }

    private func writeFrame(opcode: UInt8, payload: [UInt8], thenClose: Bool) {
        var frame: [UInt8] = [0x80 | opcode]
        let n = payload.count
        if n < 126 {
            frame.append(UInt8(n))
        } else if n <= 0xFFFF {
            frame += [126, UInt8(n >> 8), UInt8(n & 0xFF)]
        } else {
            frame.append(127)
            for shift in stride(from: 56, through: 0, by: -8) { frame.append(UInt8((UInt64(n) >> UInt64(shift)) & 0xFF)) }
        }
        frame += payload
        write(Data(frame), thenClose: thenClose)
    }

    private func writeRaw(_ text: String, thenClose: Bool) {
        write(Data(text.utf8), thenClose: thenClose)
    }

    private func write(_ data: Data, thenClose: Bool) {
        guard !closed else { return }
        pendingSends += 1
        connection.send(content: data, completion: .contentProcessed { [weak self] error in
            guard let self = self else { return }
            self.pendingSends -= 1
            if error != nil || thenClose { self.finish() }
        })
        if thenClose { closed = true }
    }

    private func finish() {
        closed = true
        upgraded = false
        connection.cancel()
        let callback = onClose
        onClose = nil
        callback?(self)
    }
}

// MARK: - Sensors

/// Reference-counted CoreMotion, altimeter and location sources, reporting in
/// Android units on `queue`.
private final class SensorLinkSensors: NSObject, CLLocationManagerDelegate {
    static let accelerometer = "android.sensor.accelerometer"
    static let gyroscope = "android.sensor.gyroscope"
    static let magneticField = "android.sensor.magnetic_field"
    static let gravity = "android.sensor.gravity"
    static let linearAcceleration = "android.sensor.linear_acceleration"
    static let rotationVector = SensorLinkProtocol.rotationVector
    static let pressure = "android.sensor.pressure"
    static let allTypes = [accelerometer, gyroscope, magneticField, gravity, linearAcceleration, rotationVector, pressure]

    static func displayName(_ type: String) -> String {
        switch type {
        case accelerometer: return "Accelerometer"
        case gyroscope: return "Gyroscope"
        case magneticField: return "Magnetometer"
        case gravity: return "Gravity"
        case linearAcceleration: return "Linear acceleration"
        case rotationVector: return "Rotation vector"
        case pressure: return "Barometer"
        default: return type
        }
    }
    private static let deviceMotionTypes: Set<String> = [gravity, linearAcceleration, rotationVector]
    private static let g = 9.80665

    typealias Emit = (String, [Float], UInt64, Int) -> Void

    private let queue: DispatchQueue
    private let emit: Emit
    private let emitGps: (SensorLinkProtocol.GpsFix) -> Void
    private let motion = CMMotionManager()
    private let altimeter = CMAltimeter()
    private let operationQueue = OperationQueue()
    private var counts: [String: Int] = [:]
    private var gpsCount = 0
    private var locationManager: CLLocationManager?
    private(set) var lastFix: SensorLinkProtocol.GpsFix?

    init(queue: DispatchQueue, emit: @escaping Emit, gps: @escaping (SensorLinkProtocol.GpsFix) -> Void) {
        self.queue = queue
        self.emit = emit
        self.emitGps = gps
        super.init()
        operationQueue.underlyingQueue = queue
        operationQueue.maxConcurrentOperationCount = 1
        let interval = 0.02 // 50 Hz, like Android's SENSOR_DELAY_GAME
        motion.accelerometerUpdateInterval = interval
        motion.gyroUpdateInterval = interval
        motion.magnetometerUpdateInterval = interval
        motion.deviceMotionUpdateInterval = interval
    }

    /// Set from the main thread when the server starts (CLLocationManager wants the main thread).
    var canUseLocation = false

    func isAvailable(_ type: String) -> Bool {
        switch type {
        case Self.accelerometer: return motion.isAccelerometerAvailable
        case Self.gyroscope: return motion.isGyroAvailable
        case Self.magneticField: return motion.isMagnetometerAvailable
        case Self.gravity, Self.linearAcceleration: return motion.isDeviceMotionAvailable
        case Self.rotationVector:
            return motion.isDeviceMotionAvailable
                && CMMotionManager.availableAttitudeReferenceFrames().contains(.xMagneticNorthZVertical)
        case Self.pressure: return CMAltimeter.isRelativeAltitudeAvailable()
        default: return false
        }
    }

    func retain(_ type: String) {
        counts[type, default: 0] += 1
        guard counts[type] == 1 else { return }
        if Self.deviceMotionTypes.contains(type) {
            if deviceMotionUsers == 1 { startDeviceMotion() }
            return
        }
        switch type {
        case Self.accelerometer:
            motion.startAccelerometerUpdates(to: operationQueue) { [weak self] data, _ in
                guard let d = data, let self = self else { return }
                let a = d.acceleration
                self.emit(Self.accelerometer, [Float(-a.x * Self.g), Float(-a.y * Self.g), Float(-a.z * Self.g)], Self.ns(d.timestamp), 3)
            }
        case Self.gyroscope:
            motion.startGyroUpdates(to: operationQueue) { [weak self] data, _ in
                guard let d = data, let self = self else { return }
                let r = d.rotationRate
                self.emit(Self.gyroscope, [Float(r.x), Float(r.y), Float(r.z)], Self.ns(d.timestamp), 3)
            }
        case Self.magneticField:
            motion.startMagnetometerUpdates(to: operationQueue) { [weak self] data, _ in
                guard let d = data, let self = self else { return }
                let m = d.magneticField
                self.emit(Self.magneticField, [Float(m.x), Float(m.y), Float(m.z)], Self.ns(d.timestamp), 3)
            }
        case Self.pressure:
            altimeter.startRelativeAltitudeUpdates(to: operationQueue) { [weak self] data, _ in
                guard let d = data, let self = self else { return }
                self.emit(Self.pressure, [Float(d.pressure.doubleValue * 10)], Self.ns(d.timestamp), 3) // kPa -> hPa
            }
        default:
            break
        }
    }

    func release(_ type: String) {
        guard let n = counts[type], n > 0 else { return }
        counts[type] = n - 1
        guard n - 1 == 0 else { return }
        counts[type] = nil
        if Self.deviceMotionTypes.contains(type) {
            if deviceMotionUsers == 0 { motion.stopDeviceMotionUpdates() }
            return
        }
        switch type {
        case Self.accelerometer: motion.stopAccelerometerUpdates()
        case Self.gyroscope: motion.stopGyroUpdates()
        case Self.magneticField: motion.stopMagnetometerUpdates()
        case Self.pressure: altimeter.stopRelativeAltitudeUpdates()
        default: break
        }
    }

    /// Seconds between readings for the CoreMotion sensors (the browser's "Sensor speed").
    func setInterval(_ interval: TimeInterval) {
        motion.accelerometerUpdateInterval = interval
        motion.gyroUpdateInterval = interval
        motion.magnetometerUpdateInterval = interval
        motion.deviceMotionUpdateInterval = interval
    }

    func stopAll() {
        motion.stopAccelerometerUpdates()
        motion.stopGyroUpdates()
        motion.stopMagnetometerUpdates()
        motion.stopDeviceMotionUpdates()
        altimeter.stopRelativeAltitudeUpdates()
        counts.removeAll()
        if gpsCount > 0 {
            gpsCount = 0
            DispatchQueue.main.async { self.locationManager?.stopUpdatingLocation() }
        }
    }

    private var deviceMotionUsers: Int {
        Self.deviceMotionTypes.reduce(0) { $0 + (counts[$1] ?? 0 > 0 ? 1 : 0) }
    }

    private func startDeviceMotion() {
        let frame: CMAttitudeReferenceFrame =
            CMMotionManager.availableAttitudeReferenceFrames().contains(.xMagneticNorthZVertical)
                ? .xMagneticNorthZVertical : .xArbitraryZVertical
        motion.startDeviceMotionUpdates(using: frame, to: operationQueue) { [weak self] data, _ in
            guard let d = data, let self = self else { return }
            let ts = Self.ns(d.timestamp)
            // Android's gravity points up (the reaction), iOS's points down.
            let gx = -d.gravity.x, gy = -d.gravity.y, gz = -d.gravity.z
            if (self.counts[Self.gravity] ?? 0) > 0 {
                self.emit(Self.gravity, [Float(gx * Self.g), Float(gy * Self.g), Float(gz * Self.g)], ts, 3)
            }
            if (self.counts[Self.linearAcceleration] ?? 0) > 0 {
                let u = d.userAcceleration
                self.emit(Self.linearAcceleration, [Float(-u.x * Self.g), Float(-u.y * Self.g), Float(-u.z * Self.g)], ts, 3)
            }
            if (self.counts[Self.rotationVector] ?? 0) > 0,
               d.magneticField.accuracy != .uncalibrated,
               let q = Self.androidRotationVector(gravity: (gx, gy, gz), field: d.magneticField.field) {
                let accuracy: Int
                switch d.magneticField.accuracy {
                case .high: accuracy = 3
                case .medium: accuracy = 2
                default: accuracy = 1
                }
                self.emit(Self.rotationVector, q.map { Float($0) }, ts, accuracy)
            }
        }
    }

    /// Android's getRotationMatrix(gravity, geomagnetic) turned into its rotation
    /// vector [x, y, z, w], so SensorServer clients see the Android convention.
    private static func androidRotationVector(gravity a: (Double, Double, Double), field e: CMMagneticField) -> [Double]? {
        let (ax0, ay0, az0) = a
        let ex = e.x, ey = e.y, ez = e.z
        var hx = ey * az0 - ez * ay0, hy = ez * ax0 - ex * az0, hz = ex * ay0 - ey * ax0
        let normH = (hx * hx + hy * hy + hz * hz).squareRoot()
        let normA = (ax0 * ax0 + ay0 * ay0 + az0 * az0).squareRoot()
        guard normH > 0.1, normA > 0.1 else { return nil } // free fall or next to a magnet
        hx /= normH; hy /= normH; hz /= normH
        let ax = ax0 / normA, ay = ay0 / normA, az = az0 / normA
        let mx = ay * hz - az * hy, my = az * hx - ax * hz, mz = ax * hy - ay * hx
        let r = [hx, hy, hz, mx, my, mz, ax, ay, az] // rows: east, north, up
        var x, y, z, w: Double
        let trace = r[0] + r[4] + r[8]
        if trace > 0 {
            let s = (trace + 1).squareRoot() * 2
            w = 0.25 * s; x = (r[7] - r[5]) / s; y = (r[2] - r[6]) / s; z = (r[3] - r[1]) / s
        } else if r[0] > r[4] && r[0] > r[8] {
            let s = (1 + r[0] - r[4] - r[8]).squareRoot() * 2
            w = (r[7] - r[5]) / s; x = 0.25 * s; y = (r[1] + r[3]) / s; z = (r[2] + r[6]) / s
        } else if r[4] > r[8] {
            let s = (1 + r[4] - r[0] - r[8]).squareRoot() * 2
            w = (r[2] - r[6]) / s; x = (r[1] + r[3]) / s; y = 0.25 * s; z = (r[5] + r[7]) / s
        } else {
            let s = (1 + r[8] - r[0] - r[4]).squareRoot() * 2
            w = (r[3] - r[1]) / s; x = (r[2] + r[6]) / s; y = (r[5] + r[7]) / s; z = 0.25 * s
        }
        if w < 0 { x = -x; y = -y; z = -z; w = -w }
        return [x, y, z, w]
    }

    private static func ns(_ seconds: TimeInterval) -> UInt64 {
        UInt64(max(0, seconds) * 1_000_000_000)
    }

    // MARK: GPS

    func retainGps() {
        gpsCount += 1
        guard gpsCount == 1 else { return }
        DispatchQueue.main.async {
            if self.locationManager == nil {
                let m = CLLocationManager()
                m.delegate = self
                m.desiredAccuracy = kCLLocationAccuracyBest
                m.pausesLocationUpdatesAutomatically = false
                self.locationManager = m
            }
            self.locationManager?.startUpdatingLocation()
        }
    }

    func releaseGps() {
        guard gpsCount > 0 else { return }
        gpsCount -= 1
        if gpsCount == 0 {
            DispatchQueue.main.async { self.locationManager?.stopUpdatingLocation() }
        }
    }

    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        guard let l = locations.last, l.horizontalAccuracy >= 0 else { return }
        var fix = SensorLinkProtocol.GpsFix(latitude: l.coordinate.latitude, longitude: l.coordinate.longitude)
        fix.altitude = l.altitude
        fix.bearing = l.course >= 0 ? l.course : 0
        fix.accuracy = l.horizontalAccuracy
        fix.speed = l.speed >= 0 ? l.speed : 0
        fix.timeMs = Int64(l.timestamp.timeIntervalSince1970 * 1000)
        queue.async {
            self.lastFix = fix
            self.emitGps(fix)
        }
    }
}
