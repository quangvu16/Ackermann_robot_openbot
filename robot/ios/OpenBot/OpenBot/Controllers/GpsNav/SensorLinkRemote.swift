//
//  SensorLinkRemote.swift
//  OpenBot
//
//  The browser page's camera view and settings panel (Sensor link, server
//  mode). Mirrors RemoteSettings.java, CameraHub.java and OnDemandCamera.java.
//
//    GET /camera.mjpg   live MJPEG stream, GET /camera.jpg one picture
//    WS  /control       phone -> page {"screen","settings":[...],"camera"}
//                       page -> phone {"set":"<key>","value":<value>}
//

import AVFoundation
import CoreImage
import ImageIO
import Foundation
import UIKit

// MARK: - Settings the page can change

struct RemoteSetting {
    enum Kind: String { case bool, choice, range, action }

    let key: String
    let group: String
    let label: String
    let kind: Kind
    var options: [String] = []
    var min = 0.0
    var max = 0.0
    var step = 0.0
    let get: () -> Any
    let set: (Any) -> Void

    static func bool(_ key: String, _ group: String, _ label: String,
                     get: @escaping () -> Bool, set: @escaping (Bool) -> Void) -> RemoteSetting {
        RemoteSetting(key: key, group: group, label: label, kind: .bool, get: { get() }, set: { set($0 as? Bool ?? false) })
    }

    static func choice(_ key: String, _ group: String, _ label: String, _ options: [String],
                       get: @escaping () -> String, set: @escaping (String) -> Void) -> RemoteSetting {
        RemoteSetting(key: key, group: group, label: label, kind: .choice, options: options,
                      get: { get() }, set: { set($0 as? String ?? "") })
    }

    static func range(_ key: String, _ group: String, _ label: String, _ min: Double, _ max: Double, _ step: Double,
                      get: @escaping () -> Double, set: @escaping (Double) -> Void) -> RemoteSetting {
        RemoteSetting(key: key, group: group, label: label, kind: .range, min: min, max: max, step: step,
                      get: { get() }, set: { set($0 as? Double ?? min) })
    }

    /// A button; `offLabel` shows while the value is false (e.g. STOP), `onLabel` while true (RESUME).
    static func action(_ key: String, _ group: String, _ label: String, _ offLabel: String, _ onLabel: String,
                       get: @escaping () -> Bool, set: @escaping (Bool) -> Void) -> RemoteSetting {
        RemoteSetting(key: key, group: group, label: label, kind: .action, options: [offLabel, onLabel],
                      get: { get() }, set: { set($0 as? Bool ?? false) })
    }

    var json: [String: Any] {
        var o: [String: Any] = ["key": key, "group": group, "label": label, "type": kind.rawValue, "value": get()]
        if !options.isEmpty { o["options"] = options }
        if kind == .range { o["min"] = min; o["max"] = max; o["step"] = step }
        return o
    }

    /// What the page sent, converted to this setting's type; nil when it does not fit.
    func coerce(_ raw: Any?) -> Any? {
        switch kind {
        case .bool, .action:
            if let n = raw as? NSNumber { return n.boolValue }
            return nil
        case .choice:
            guard let s = raw as? String, options.contains(s) else { return nil }
            return s
        case .range:
            guard let n = raw as? NSNumber else { return nil }
            let d = n.doubleValue
            guard d.isFinite else { return nil }
            return Swift.max(min, Swift.min(max, d))
        }
    }
}

protocol RemoteSettingsProvider: AnyObject {
    func addSettings(_ out: inout [RemoteSetting])
}

/// A provider made from a closure, for screens.
final class ClosureSettingsProvider: RemoteSettingsProvider {
    private let body: (inout [RemoteSetting]) -> Void
    init(_ body: @escaping (inout [RemoteSetting]) -> Void) { self.body = body }
    func addSettings(_ out: inout [RemoteSetting]) { body(&out) }
}

/// Collects settings from whoever is active. Main thread.
final class RemoteSettings {
    private var providers: [RemoteSettingsProvider] = []

    func add(_ p: RemoteSettingsProvider) {
        if !providers.contains(where: { $0 === p }) { providers.append(p) }
    }

    func remove(_ p: RemoteSettingsProvider) {
        providers.removeAll { $0 === p }
    }

    func list() -> [RemoteSetting] {
        var out: [RemoteSetting] = []
        providers.forEach { $0.addSettings(&out) }
        return out
    }

    func stateJSON(screen: String, camera: String) -> String {
        let o: [String: Any] = ["screen": screen, "settings": list().map { $0.json }, "camera": camera]
        guard let data = try? JSONSerialization.data(withJSONObject: o),
              let text = String(data: data, encoding: .utf8) else { return "{}" }
        return text
    }

    @discardableResult
    func apply(_ message: String) -> Bool {
        guard let data = message.data(using: .utf8),
              let o = (try? JSONSerialization.jsonObject(with: data)) as? [String: Any],
              let key = o["set"] as? String,
              let setting = list().first(where: { $0.key == key }),
              let value = setting.coerce(o["value"])
        else { return false }
        setting.set(value)
        return true
    }
}

// MARK: - Camera to the browser

/// The screen that has the camera open. Main thread.
protocol CameraHubOwner: AnyObject {
    var cameraIsFront: Bool { get }
    func cameraApplyFacing(_ front: Bool)
    func cameraApplyTorch(_ on: Bool)
    func cameraApplyZoom(_ ratio: Double)
    var cameraMaxZoom: Double { get }
    var cameraHasFlash: Bool { get }
}

extension Notification.Name {
    /// Posted on the main queue when the first browser starts watching or the last one stops.
    static let cameraHubViewersChanged = Notification.Name("cameraHubViewersChanged")
}

/// Encodes camera frames as JPEG for the browser, only while someone watches.
final class CameraHub: RemoteSettingsProvider {
    static let shared = CameraHub()
    static let qualities = ["low", "medium", "high"]

    weak var owner: CameraHubOwner? {
        didSet { if owner != nil { unavailableReason = nil } }
    }
    private var unavailableReason: String? = "No camera on this screen"

    /// Why there is no picture (shown on the page); nil while a camera runs.
    var reason: String? { owner == nil ? (unavailableReason ?? "No camera on this screen") : nil }

    func setUnavailable(_ reason: String) {
        unavailableReason = reason
    }

    // Settings: written on main, read on the capture queue.
    private(set) var torch = false
    private(set) var zoom = 1.0
    private(set) var quality = "medium"
    private(set) var fps = 10.0

    /// Called on the capture queue with each encoded frame (the server sends it on).
    var onFrame: ((Data) -> Void)?

    private let lock = NSLock()
    private var viewers = 0
    private var lastEncode: TimeInterval = 0
    private(set) var latest: Data?
    private(set) var latestUptime: TimeInterval = 0
    private let context = CIContext()

    var hasViewers: Bool {
        lock.lock(); defer { lock.unlock() }
        return viewers > 0
    }

    func setViewers(_ n: Int) {
        lock.lock()
        let changed = (viewers > 0) != (n > 0)
        viewers = n
        if n == 0 { latest = nil }
        lock.unlock()
        if changed {
            DispatchQueue.main.async { NotificationCenter.default.post(name: .cameraHubViewersChanged, object: nil) }
        }
    }

    func reapply(_ o: CameraHubOwner) {
        o.cameraApplyTorch(torch)
        o.cameraApplyZoom(Swift.min(zoom, o.cameraMaxZoom))
    }

    /// A camera frame, on the capture queue. Cheap when nobody is watching.
    func offer(_ sampleBuffer: CMSampleBuffer) {
        guard hasViewers, let pixels = CMSampleBufferGetImageBuffer(sampleBuffer) else { return }
        let now = ProcessInfo.processInfo.systemUptime
        guard now - lastEncode >= 1.0 / Swift.max(1, fps) else { return }
        lastEncode = now

        let target: CGFloat = quality == "low" ? 320 : quality == "high" ? 1280 : 640
        var image = CIImage(cvPixelBuffer: pixels)
        let longSide = Swift.max(image.extent.width, image.extent.height)
        let scale = Swift.min(1, target / longSide)
        if scale < 1 { image = image.transformed(by: CGAffineTransform(scaleX: scale, y: scale)) }
        let options = [CIImageRepresentationOption(rawValue: kCGImageDestinationLossyCompressionQuality as String): quality == "high" ? 0.8 : 0.7]
        guard let jpeg = context.jpegRepresentation(of: image, colorSpace: CGColorSpaceCreateDeviceRGB(), options: options)
        else { return }
        lock.lock()
        latest = jpeg
        latestUptime = now
        lock.unlock()
        onFrame?(jpeg)
    }

    func addSettings(_ out: inout [RemoteSetting]) {
        if let o = owner {
            out.append(.choice("camera.facing", "Camera", "Lens", ["back", "front"],
                               get: { o.cameraIsFront ? "front" : "back" },
                               set: { o.cameraApplyFacing($0 == "front") }))
            if o.cameraHasFlash {
                out.append(.bool("camera.torch", "Camera", "Flashlight",
                                 get: { [unowned self] in self.torch },
                                 set: { [unowned self] in self.torch = $0; o.cameraApplyTorch($0) }))
            }
            let max = o.cameraMaxZoom
            if max > 1.01 {
                out.append(.range("camera.zoom", "Camera", "Zoom", 1, max, 0.1,
                                  get: { [unowned self] in self.zoom },
                                  set: { [unowned self] in self.zoom = $0; o.cameraApplyZoom($0) }))
            }
        }
        out.append(.choice("camera.quality", "Camera", "Stream quality", Self.qualities,
                           get: { [unowned self] in self.quality },
                           set: { [unowned self] in self.quality = $0 }))
        out.append(.range("camera.fps", "Camera", "Stream frames per second", 1, 30, 1,
                          get: { [unowned self] in self.fps },
                          set: { [unowned self] in self.fps = $0.rounded() }))
    }
}

// MARK: - The camera screens (Object Tracking, Autopilot)

extension CameraController: CameraHubOwner {
    private var currentCameraDevice: AVCaptureDevice? {
        (captureSession?.inputs.first as? AVCaptureDeviceInput)?.device
    }

    var cameraIsFront: Bool { currentCameraDevice?.position == .front }

    func cameraApplyFacing(_ front: Bool) {
        guard front != cameraIsFront else { return }
        switchCameraView()
        CameraHub.shared.reapply(self)
    }

    func cameraApplyTorch(_ on: Bool) {
        guard let d = currentCameraDevice, d.hasTorch, (try? d.lockForConfiguration()) != nil else { return }
        d.torchMode = on ? .on : .off
        d.unlockForConfiguration()
    }

    func cameraApplyZoom(_ ratio: Double) {
        guard let d = currentCameraDevice, (try? d.lockForConfiguration()) != nil else { return }
        d.videoZoomFactor = CGFloat(Swift.max(1, Swift.min(ratio, cameraMaxZoom)))
        d.unlockForConfiguration()
    }

    var cameraMaxZoom: Double { Double(Swift.min(currentCameraDevice?.activeFormat.videoMaxZoomFactor ?? 1, 10)) }

    var cameraHasFlash: Bool { currentCameraDevice?.hasTorch ?? false }
}

// MARK: - A camera for GPS Mission

/// Opens the back camera only while a browser watches, with no preview on the
/// phone. GPS Mission starts it in viewDidAppear and stops it on disappear.
final class OnDemandCamera: NSObject, AVCaptureVideoDataOutputSampleBufferDelegate, CameraHubOwner {
    private let session = AVCaptureSession()
    private let queue = DispatchQueue(label: "OnDemandCamera")
    private let output = AVCaptureVideoDataOutput()
    private var device: AVCaptureDevice?
    private var started = false
    private var running = false
    private var front = false

    func start() {
        guard !started else { return }
        started = true
        CameraHub.shared.owner = self
        NotificationCenter.default.addObserver(self, selector: #selector(update), name: .cameraHubViewersChanged, object: nil)
        update()
    }

    func stop() {
        guard started else { return }
        started = false
        NotificationCenter.default.removeObserver(self, name: .cameraHubViewersChanged, object: nil)
        if CameraHub.shared.owner === self { CameraHub.shared.owner = nil }
        close()
    }

    @objc private func update() {
        guard started else { return }
        CameraHub.shared.hasViewers ? open() : close()
    }

    private func open() {
        guard !running else { return }
        guard AVCaptureDevice.authorizationStatus(for: .video) == .authorized else {
            if CameraHub.shared.owner === self { CameraHub.shared.owner = nil }
            CameraHub.shared.setUnavailable("Camera permission not granted: open Object Tracking once and allow the camera")
            return
        }
        running = true
        configure()
        queue.async { self.session.startRunning() }
    }

    private func configure() {
        session.beginConfiguration()
        session.inputs.forEach { session.removeInput($0) }
        if session.canSetSessionPreset(.hd1280x720) { session.sessionPreset = .hd1280x720 }
        let position: AVCaptureDevice.Position = front ? .front : .back
        if let d = AVCaptureDevice.default(.builtInWideAngleCamera, for: .video, position: position),
           let input = try? AVCaptureDeviceInput(device: d), session.canAddInput(input) {
            session.addInput(input)
            device = d
        } else {
            CameraHub.shared.setUnavailable("Camera busy or missing")
        }
        if session.outputs.isEmpty, session.canAddOutput(output) {
            output.alwaysDiscardsLateVideoFrames = true
            output.setSampleBufferDelegate(self, queue: queue)
            session.addOutput(output)
        }
        output.connection(with: .video)?.videoOrientation = .portrait
        session.commitConfiguration()
        CameraHub.shared.owner = self
        CameraHub.shared.reapply(self)
    }

    private func close() {
        guard running else { return }
        running = false
        queue.async { self.session.stopRunning() }
    }

    func captureOutput(_ output: AVCaptureOutput, didOutput sampleBuffer: CMSampleBuffer, from connection: AVCaptureConnection) {
        CameraHub.shared.offer(sampleBuffer)
    }

    // CameraHubOwner

    var cameraIsFront: Bool { front }

    func cameraApplyFacing(_ front: Bool) {
        guard front != self.front else { return }
        self.front = front
        if running { configure() }
    }

    func cameraApplyTorch(_ on: Bool) {
        guard let d = device, d.hasTorch, (try? d.lockForConfiguration()) != nil else { return }
        d.torchMode = on ? .on : .off
        d.unlockForConfiguration()
    }

    func cameraApplyZoom(_ ratio: Double) {
        guard let d = device, (try? d.lockForConfiguration()) != nil else { return }
        d.videoZoomFactor = CGFloat(Swift.max(1, Swift.min(ratio, cameraMaxZoom)))
        d.unlockForConfiguration()
    }

    var cameraMaxZoom: Double { Double(Swift.min(device?.activeFormat.videoMaxZoomFactor ?? 1, 10)) }

    var cameraHasFlash: Bool { device?.hasTorch ?? true }
}

// MARK: - Driving settings, shared by the three screens

enum SensorLinkDrivingSettings {
    static func add(_ out: inout [RemoteSetting]) {
        out.append(.action("drive.stop", "Driving", "Emergency stop", "STOP", "RESUME",
                           get: { EmergencyStop.shared.isLatched },
                           set: { $0 ? EmergencyStop.shared.latch() : EmergencyStop.shared.release() }))
        out.append(.choice("drive.speed", "Driving", "Speed mode", ["slow", "normal", "fast"],
                           get: {
                               switch GameController.shared.selectedSpeedMode {
                               case .SLOW: return "slow"
                               case .NORMAL: return "normal"
                               case .FAST: return "fast"
                               }
                           },
                           set: {
                               GameController.shared.selectedSpeedMode = $0 == "slow" ? .SLOW : $0 == "fast" ? .FAST : .NORMAL
                           }))
        let bluetooth = bluetoothDataController.shared
        out.append(.bool("drive.avoidance", "Driving", "Object avoidance",
                         get: { bluetooth.driveGate.settings.enabled },
                         set: { on in
                             var s = bluetooth.driveGate.settings
                             s.enabled = on
                             bluetooth.updateObjectAvoidance(s)
                         }))
        out.append(.range("drive.avoidanceMargin", "Driving", "Avoidance distance (m)",
                          AvoidanceSettings.marginRange.lowerBound, AvoidanceSettings.marginRange.upperBound, 0.1,
                          get: { bluetooth.driveGate.settings.marginMeters },
                          set: { m in
                              var s = bluetooth.driveGate.settings
                              s.marginMeters = m
                              bluetooth.updateObjectAvoidance(s)
                          }))
    }
}
