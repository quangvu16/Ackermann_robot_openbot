//
//  RoomsController.swift
//  OpenBot
//
//  Rooms & Zones: what Valetudo (github.com/Hypfer/Valetudo) gives a robot vacuum, for this car on
//  its own LiDAR map. Mirrors RoomsFragment on Android. The saved Indoor SLAM map is loaded in
//  localize-only mode; rooms, zones, no-go areas, virtual walls and home sit on top (RoomsCore).
//  SLAM, the navigator, the job and the rooms are owned by one serial queue; the screen draws copies.
//

import AudioToolbox
import CoreMotion
import UIKit

final class RoomsController: UIViewController, ValetudoRobot {
    private let mapView = RoomsMapView()
    private let mapStatus = PaddedLabel()
    private let statusLabel = UILabel()
    private let selectionLabel = UILabel()
    private let sourceLabel = UILabel()
    private let mqttLabel = UILabel()
    private let toolPicker = UISegmentedControl.icons([
        ("hand.tap", L10n.string("rooms_tool_select")), ("location.north.fill", L10n.string("rooms_tool_goto")),
        ("square.dashed", L10n.string("rooms_tool_zone")), ("nosign", L10n.string("rooms_tool_nogo")),
        ("line.diagonal", L10n.string("rooms_tool_wall")), ("house.circle", L10n.string("rooms_tool_home")),
    ])
    private let startButton = UIButton(type: .system)

    private static let queueKey = DispatchSpecificKey<Bool>()
    private let queue: DispatchQueue = {
        let q = DispatchQueue(label: "RoomsController")
        q.setSpecific(key: RoomsController.queueKey, value: true)
        return q
    }()
    // Owned by `queue`:
    private let slam = SlamSystem(.cartographer, .indoorDefault())
    private let navigator = SlamNavigator()
    private lazy var mission = CleaningMission(navigator: navigator)
    private var rooms = RoomMap()
    private var mapLoaded = false
    private var activeZones: [[Double]] = []
    private var driving = false
    private var lastImageTime = 0.0
    // Main thread:
    private var timer: Timer?
    private var lastScanId: UInt32?
    private var scanInFlight = false
    private var shownRelocalizeSeq = 0
    private var roomsVersion = 0
    private var selectedRooms: [Int] = []
    private var selectedItem: (kind: Int, index: Int)?
    private var uiRooms = RoomMap()
    private var uiBusy = false, uiStatus = CleaningMission.Status.idle
    private var lastMqtt = 0.0
    private var noMapText = ""

    private let motion = CMMotionManager()
    private var imuYaw = Double.nan
    private let bluetooth = bluetoothDataController.shared
    private lazy var api = ValetudoApi(robot: self)
    private var mqtt: ValetudoMqtt?
    private var mqttStatus = ""

    private lazy var sensorLinkItem = SensorLinkBarItem(owner: self, screen: "Rooms & Zones") { [unowned self] out in
        out.append(.action("rooms.start", "Rooms & Zones", "Patrol the selected rooms (or all)", "START", "PAUSE",
                           get: { [unowned self] in self.uiBusy },
                           set: { [unowned self] _ in _ = self.basicControl(self.uiBusy ? "pause" : "start") }))
        out.append(.action("rooms.home", "Rooms & Zones", "Return home", "HOME", "HOME",
                           get: { false }, set: { [unowned self] _ in _ = self.basicControl("home") }))
        out.append(.action("rooms.locate", "Rooms & Zones", "Locate", "LOCATE", "LOCATE",
                           get: { false }, set: { [unowned self] _ in _ = self.locate() }))
    }

    override func viewDidLoad() {
        super.viewDidLoad()
        view.backgroundColor = .systemBackground
        title = L10n.string("rooms_title")
        layout()
        mapView.showTrail = false
        mapView.onTap = { [weak self] x, y in self?.tapped(x, y) }
        mapView.onLongPress = { [weak self] x, y in
            guard let self = self else { return }
            self.queue.async { self.slam.resetPose(SlamPose(x, y, self.slam.pose.theta)) }
            self.toast(L10n.string("slam_pose_set"))
        }
        mapView.onShape = { [weak self] tool, a, b in self?.shapeDrawn(tool, a, b) }
        navigationItem.rightBarButtonItem = sensorLinkItem
        applySavedAlgorithms()
    }

    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(animated)
        SharedLidar.acquire()
        UIApplication.shared.isIdleTimerDisabled = true
        if motion.isDeviceMotionAvailable {
            motion.deviceMotionUpdateInterval = 0.02
            motion.startDeviceMotionUpdates(to: .main) { [weak self] m, _ in
                guard let self = self, let m = m else { return }
                let g = m.gravity, w = m.rotationRate
                let n = (g.x * g.x + g.y * g.y + g.z * g.z).squareRoot()
                guard n > 0.1 else { return }
                let rate = -(w.x * g.x + w.y * g.y + w.z * g.z) / n
                self.imuYaw = (self.imuYaw.isNaN ? 0 : self.imuYaw) + rate * 0.02
            }
        }
        loadMapAndRooms()
        ValetudoHub.shared.set(api)
        startMqtt()
        timer = Timer.scheduledTimer(withTimeInterval: 0.1, repeats: true) { [weak self] _ in self?.tick() }
        sensorLinkItem.appeared()
    }

    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        ValetudoHub.shared.clear(api)
        stopMqtt()
        sensorLinkItem.disappeared()
        timer?.invalidate()
        timer = nil
        motion.stopDeviceMotionUpdates()
        queue.async { self.mission.stop() }
        sendControl(0, 0)
        SharedLidar.release()
        UIApplication.shared.isIdleTimerDisabled = false
    }

    // MARK: Layout

    private func layout() {
        mapView.translatesAutoresizingMaskIntoConstraints = false
        view.addSubview(mapView)
        let centre = UIButton.icon("scope", label: L10n.string("slam_center"), style: .tinted())
        centre.addTarget(self, action: #selector(recentre), for: .touchUpInside)
        centre.translatesAutoresizingMaskIntoConstraints = false
        view.addSubview(centre)
        mapStatus.font = .monospacedDigitSystemFont(ofSize: 12, weight: .regular)
        mapStatus.textColor = .white
        mapStatus.backgroundColor = UIColor.black.withAlphaComponent(0.7)
        mapStatus.translatesAutoresizingMaskIntoConstraints = false
        view.addSubview(mapStatus)

        let scroll = UIScrollView()
        scroll.translatesAutoresizingMaskIntoConstraints = false
        view.addSubview(scroll)
        let stack = UIStackView()
        stack.axis = .vertical
        stack.spacing = 8
        stack.translatesAutoresizingMaskIntoConstraints = false
        scroll.addSubview(stack)
        NSLayoutConstraint.activate([
            mapView.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor),
            mapView.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            mapView.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            mapView.heightAnchor.constraint(equalTo: view.safeAreaLayoutGuide.heightAnchor, multiplier: 0.55),
            centre.topAnchor.constraint(equalTo: mapView.topAnchor, constant: 10),
            centre.leadingAnchor.constraint(equalTo: mapView.leadingAnchor, constant: 10),
            mapStatus.leadingAnchor.constraint(equalTo: mapView.leadingAnchor, constant: 8),
            mapStatus.bottomAnchor.constraint(equalTo: mapView.bottomAnchor, constant: -8),
            scroll.topAnchor.constraint(equalTo: mapView.bottomAnchor),
            scroll.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            scroll.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            scroll.bottomAnchor.constraint(equalTo: view.bottomAnchor),
            stack.topAnchor.constraint(equalTo: scroll.contentLayoutGuide.topAnchor, constant: 10),
            stack.leadingAnchor.constraint(equalTo: scroll.frameLayoutGuide.leadingAnchor, constant: 14),
            stack.trailingAnchor.constraint(equalTo: scroll.frameLayoutGuide.trailingAnchor, constant: -14),
            stack.bottomAnchor.constraint(equalTo: scroll.contentLayoutGuide.bottomAnchor, constant: -20),
        ])
        statusLabel.font = .systemFont(ofSize: 15, weight: .semibold)
        statusLabel.numberOfLines = 0
        stack.addArrangedSubview(statusLabel)
        selectionLabel.font = .systemFont(ofSize: 13)
        selectionLabel.numberOfLines = 0
        stack.addArrangedSubview(selectionLabel)
        toolPicker.selectedSegmentIndex = 0
        toolPicker.addTarget(self, action: #selector(toolChanged), for: .valueChanged)
        stack.addArrangedSubview(toolPicker)

        var go = UIButton.Configuration.filled()
        go.cornerStyle = .large
        go.baseBackgroundColor = UIColor(red: 0.29, green: 0.48, blue: 1, alpha: 1)
        startButton.configuration = go
        startButton.setIcon("play.fill", label: L10n.string("rooms_start"), pointSize: 26)
        startButton.addTarget(self, action: #selector(startTapped), for: .touchUpInside)
        let actions = UIStackView(arrangedSubviews: [
            startButton,
            button("stop.fill", "rooms_stop", #selector(stopTapped)),
            button("house.fill", "rooms_home", #selector(homeTapped)),
            button("speaker.wave.2.fill", "rooms_locate", #selector(locateTapped)),
        ])
        actions.distribution = .fillEqually
        actions.spacing = 6
        actions.heightAnchor.constraint(equalToConstant: 52).isActive = true
        stack.addArrangedSubview(actions)
        let edits = UIStackView(arrangedSubviews: [
            button("wand.and.stars", "rooms_detect", #selector(detectTapped)),
            button("pencil", "rooms_rename", #selector(renameTapped)),
            button("trash", "rooms_delete", #selector(deleteTapped)),
            button("gearshape", "rooms_settings", #selector(settingsTapped)),
        ])
        edits.distribution = .fillEqually
        edits.spacing = 6
        edits.heightAnchor.constraint(equalToConstant: 44).isActive = true
        stack.addArrangedSubview(edits)
        for l in [sourceLabel, mqttLabel] {
            l.font = .systemFont(ofSize: 12)
            l.textColor = .secondaryLabel
            l.numberOfLines = 0
            stack.addArrangedSubview(l)
        }
        for key in ["rooms_help", "rooms_api_note"] {
            let l = UILabel()
            l.text = L10n.string(key)
            l.font = .systemFont(ofSize: 13)
            l.textColor = .secondaryLabel
            l.numberOfLines = 0
            stack.addArrangedSubview(l)
        }
    }

    private func button(_ symbol: String, _ key: String, _ action: Selector) -> UIButton {
        let b = UIButton.icon(symbol, label: L10n.string(key))
        b.addTarget(self, action: action, for: .touchUpInside)
        return b
    }

    @objc private func recentre() { mapView.follow = true; mapView.setNeedsDisplay() }

    @objc private func toolChanged() {
        let tool = RoomsMapView.Tool(rawValue: toolPicker.selectedSegmentIndex) ?? .select
        mapView.tool = tool
        mapView.drawsWithPan = tool == .zone || tool == .noGo || tool == .wall
    }

    // MARK: Queue helper

    private func onQueue<T>(_ f: () -> T) -> T {
        DispatchQueue.getSpecific(key: RoomsController.queueKey) == true ? f() : queue.sync(execute: f)
    }

    // MARK: Files

    private var slamDir: URL {
        let dir = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0].appendingPathComponent("slam")
        try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        return dir
    }

    private func loadMapAndRooms() {
        let mapURL = slamDir.appendingPathComponent("map.obmap"), roomsURL = slamDir.appendingPathComponent("rooms.json")
        guard let d = try? Data(contentsOf: mapURL), let g = OccupancyGrid.read(d) else {
            noMapText = L10n.string("rooms_no_map")
            queue.async { self.mapLoaded = false }
            return
        }
        noMapText = ""
        let saved = (try? String(contentsOf: roomsURL, encoding: .utf8)).map { RoomMap.fromJson($0) } ?? RoomMap()
        queue.async {
            self.slam.setGrid(g)
            self.slam.mapping = false
            self.slam.requestGlobalLocalization()
            self.mapLoaded = true
            self.lastImageTime = 0
            saved.robotRadius = self.navigator.params.robotRadius
            let stale = saved.mapStamp != RoomMap.stamp(of: g)
            self.rooms = saved
            self.roomsChanged(save: false)
            if stale || !saved.hasRooms { self.detectRooms() }
        }
        toast(L10n.string("slam_finding"))
    }

    /// On `queue`: apply restrictions to the planner, save, and hand the screen a copy.
    private func roomsChanged(save: Bool = true) {
        let r = rooms
        if r.hasRestrictions { navigator.params.keepOut = { x, y in r.blocked(x, y) } } else { navigator.params.keepOut = nil }
        mission.home = r.home
        if let g = navigator.goal { navigator.setGoal(g.0, g.1) } // re-plan around restrictions
        if save { try? Data(r.toJson().utf8).write(to: slamDir.appendingPathComponent("rooms.json")) }
        let copy = RoomMap.fromJson(r.toJson())
        DispatchQueue.main.async {
            self.uiRooms = copy
            self.roomsVersion += 1
            self.selectedRooms = self.selectedRooms.filter { copy.names[$0] != nil }
        }
    }

    /// On `queue`.
    private func detectRooms() {
        guard mapLoaded else { DispatchQueue.main.async { self.toast(L10n.string("rooms_no_map")) }; return }
        let res = RoomSegmenter.segment(slam.grid)
        rooms.setRooms(res, stamp: RoomMap.stamp(of: slam.grid))
        DispatchQueue.main.async { self.selectedRooms = [] }
        roomsChanged()
        DispatchQueue.main.async { self.toast(String(format: L10n.string("rooms_detected"), Int32(res.count))) }
    }

    // MARK: Taps and drawing

    private func tapped(_ x: Double, _ y: Double) {
        switch mapView.tool {
        case .goto:
            if let e = goTo(x, y) { toast(e) }
        case .home:
            queue.async { self.rooms.home = (x, y); self.roomsChanged() }
            toast(L10n.string("rooms_home_set"))
        case .select:
            if let item = uiRooms.itemAt(x, y, within: 0.25) {
                if let s = selectedItem, s.kind == item.kind, s.index == item.index { selectedItem = nil } else { selectedItem = item }
                selectedRooms = []
            } else {
                selectedItem = nil
                let id = uiRooms.roomAt(x, y)
                if id > 0 {
                    if let i = selectedRooms.firstIndex(of: id) { selectedRooms.remove(at: i) } else { selectedRooms.append(id) }
                } else {
                    selectedRooms = []
                }
            }
        default:
            break
        }
    }

    private func shapeDrawn(_ tool: RoomsMapView.Tool, _ a: (Double, Double), _ b: (Double, Double)) {
        queue.async {
            switch tool {
            case .zone:
                self.rooms.zones.append(RoomMap.box(a.0, a.1, b.0, b.1))
                let i = self.rooms.zones.count - 1
                DispatchQueue.main.async { self.selectedItem = (0, i); self.selectedRooms = [] }
            case .noGo: self.rooms.noGo.append(RoomMap.box(a.0, a.1, b.0, b.1))
            case .wall: self.rooms.walls.append([a.0, a.1, b.0, b.1])
            default: return
            }
            self.roomsChanged()
        }
    }

    @objc private func deleteTapped() {
        guard let it = selectedItem else { toast(L10n.string("rooms_delete_none")); return }
        selectedItem = nil
        queue.async {
            switch it.kind {
            case 0: if it.index < self.rooms.zones.count { self.rooms.zones.remove(at: it.index) }
            case 1: if it.index < self.rooms.noGo.count { self.rooms.noGo.remove(at: it.index) }
            default: if it.index < self.rooms.walls.count { self.rooms.walls.remove(at: it.index) }
            }
            self.roomsChanged()
        }
    }

    @objc private func renameTapped() {
        guard selectedRooms.count == 1, let id = selectedRooms.first else { toast(L10n.string("rooms_pick_one")); return }
        let a = UIAlertController(title: L10n.string("rooms_rename"), message: nil, preferredStyle: .alert)
        a.addTextField { $0.text = self.uiRooms.name(id); $0.clearButtonMode = .whileEditing }
        a.addAction(UIAlertAction(title: "Cancel", style: .cancel))
        a.addAction(UIAlertAction(title: "OK", style: .default) { [weak self, weak a] _ in
            _ = self?.renameSegment(id, a?.textFields?.first?.text ?? "")
        })
        present(a, animated: true)
    }

    @objc private func detectTapped() { queue.async { self.detectRooms() } }
    @objc private func startTapped() { if let e = basicControl(uiBusy ? "pause" : "start") { toast(e) } }
    @objc private func stopTapped() { _ = basicControl("stop") }
    @objc private func homeTapped() { if let e = basicControl("home") { toast(e) } }
    @objc private func locateTapped() { _ = locate() }

    // MARK: Loop

    private func tick() {
        let scanner = SharedLidar.scanner
        if let scan = scanner.latestScan, scanner.isFresh {
            sourceLabel.text = String(format: L10n.string("lidarbot_source_scan"), Int32(scan.validPoints))
            if scan.scanId != lastScanId && !scanInFlight {
                lastScanId = scan.scanId
                let mm = scan.rangesMm.map { Int($0) }
                let yaw = imuYaw
                scanInFlight = true
                queue.async { self.onScan(mm, yaw) }
            }
        } else {
            sourceLabel.text = L10n.string("slam_needs_scan")
            queue.async { if self.driving { self.driving = false; DispatchQueue.main.async { self.sendControl(0, 0) } } }
        }
        mapView.setRooms(uiRooms, version: roomsVersion, selected: Set(selectedRooms), item: selectedItem)
        showSelection()
        mqttLabel.text = mqttStatus
        let now = ProcessInfo.processInfo.systemUptime
        if let m = mqtt, now - lastMqtt > 1 {
            lastMqtt = now
            DispatchQueue.global(qos: .utility).async { m.publishState() }
        }
    }

    private func showSelection() {
        var parts = selectedRooms.map { uiRooms.name($0) }
        if let it = selectedItem { parts.append(L10n.string(it.kind == 0 ? "rooms_tool_zone" : it.kind == 1 ? "rooms_tool_nogo" : "rooms_tool_wall")) }
        selectionLabel.text = parts.isEmpty ? (uiRooms.hasRooms ? "" : L10n.string("rooms_need_rooms")) : "▸ " + parts.joined(separator: ", ")
    }

    /// On `queue`: localize, drive the job, hand the screen a snapshot.
    private func onScan(_ mm: [Int], _ yaw: Double) {
        let n = mm.count
        var a = [Double](repeating: 0, count: n), r = [Double](repeating: 0, count: n)
        for i in 0..<n {
            a[i] = -((Double(i) + 0.5) * 360 / Double(n)) * .pi / 180
            r[i] = mm[i] == 0 ? 99 : Double(mm[i]) / 1000
        }
        let pose = slam.update(angles: a, ranges: r, imuYaw: yaw, time: ProcessInfo.processInfo.systemUptime)
        var frontMin = Double.infinity
        for i in 0..<n where r[i] >= SlamEngine.minRange && r[i] < SlamEngine.maxRange && abs(a[i]) < 25 * .pi / 180 {
            frontMin = min(frontMin, r[i])
        }
        let now = ProcessInfo.processInfo.systemUptime
        let wasBusy = mission.isBusy
        var command: (Double, Double)?
        if let c = mission.step(pose: pose, grid: slam.grid, frontMin: frontMin, now: now) {
            command = (c.throttle, c.steer)
        } else if wasBusy || driving {
            command = (0, 0)
        }
        if let c = command { driving = c.0 != 0 || c.1 != 0 }
        if !mission.isBusy { activeZones = [] }
        var image: CGImage?
        if now - lastImageTime > 0.5 {
            lastImageTime = now
            image = SlamMapView.makeImage(slam.grid)
        }
        let snap = SlamMapView.Snapshot(
            pose: pose, scan: zip(a, r).filter { $0.1 < SlamEngine.maxRange }.map { (pose.x + $0.1 * cos(pose.theta + $0.0), pose.y + $0.1 * sin(pose.theta + $0.0)) },
            trail: [], path: navigator.path, goal: navigator.goal)
        let grid = slam.grid
        let status = mapLoaded ? mission.status(at: pose) : .error
        let label = mission.label, error = mission.error, busy = mission.isBusy
        let lane = mission.total > 1 ? " · \(mission.index + 1) / \(mission.total)" : ""
        let plan = mission.remaining
        let quality = Int((slam.matchQuality * 100).rounded())
        let relocSeq = slam.relocalizeSeq, relocFound = slam.relocalizeFound
        let finalCommand = command
        DispatchQueue.main.async {
            self.scanInFlight = false
            if relocSeq != self.shownRelocalizeSeq {
                self.shownRelocalizeSeq = relocSeq
                self.toast(L10n.string(relocFound ? "slam_found" : "slam_not_found"))
            }
            if let c = finalCommand { self.sendControl(c.0, c.1) }
            if let img = image { self.mapView.setImage(img, grid: grid) }
            self.mapView.snapshot = snap
            self.mapView.plan = plan
            self.mapView.setNeedsDisplay()
            let here = self.uiRooms.roomAt(pose.x, pose.y)
            self.mapStatus.text = (here > 0 ? self.uiRooms.name(here) : "—") + lane + " · \(quality)%"
            var s = L10n.string("rooms_st_" + status.rawValue)
            if !label.isEmpty { s += " · " + label }
            if status == .error { s += ": " + error }
            self.statusLabel.text = self.noMapText.isEmpty ? s : self.noMapText
            self.uiBusy = busy
            self.uiStatus = status
            self.startButton.setIcon(busy ? "pause.fill" : "play.fill",
                                     label: L10n.string(busy ? "rooms_pause" : status == .paused ? "rooms_resume" : "rooms_start"), pointSize: 26)
        }
    }

    private func sendControl(_ throttle: Double, _ steer: Double) {
        let w = LidarBehaviors.toWheels(throttle: throttle, steer: steer)
        let speed = GameController.shared.selectedSpeedMode.rawValue
        bluetooth.sendData(payload: "c" + String(Int(Float(w.left) * speed)) + "," + String(Int(Float(w.right) * speed)) + "\n")
    }

    // MARK: Jobs (buttons, REST API and MQTT)

    func basicControl(_ action: String) -> String? {
        guard onQueue({ mapLoaded }) else { return L10n.string("rooms_no_map") }
        switch action {
        case "stop":
            queue.async { self.mission.stop() }
            DispatchQueue.main.async { self.sendControl(0, 0) }
            return nil
        case "pause":
            queue.async { self.mission.pause() }
            DispatchQueue.main.async { self.sendControl(0, 0) }
            return nil
        case "home":
            guard onQueue({ rooms.home != nil }) else { return "No home set: choose the home tool and tap the map" }
            queue.async { self.mission.returnHome() }
            return nil
        case "start":
            if onQueue({ mission.status(at: nil) == .paused }) { queue.async { self.mission.resume() }; return nil }
            var item: (kind: Int, index: Int)?
            var sel: [Int] = []
            let read = { item = self.selectedItem; sel = self.selectedRooms }
            if Thread.isMainThread { read() } else { DispatchQueue.main.sync(execute: read) }
            let zone: [Double]? = onQueue {
                guard let it = item, it.kind == 0, it.index < rooms.zones.count else { return nil }
                return rooms.zones[it.index]
            }
            if let z = zone { return cleanZones([z]) }
            let ids = sel.isEmpty ? onQueue({ rooms.roomIds }) : sel
            guard !ids.isEmpty else { return L10n.string("rooms_need_rooms") }
            return cleanSegments(ids)
        default:
            return "Unknown action"
        }
    }

    func goTo(_ x: Double, _ y: Double) -> String? {
        guard onQueue({ mapLoaded }) else { return L10n.string("rooms_no_map") }
        queue.async { self.mission.clearError(); self.mission.goTo(x, y) }
        return nil
    }

    private var laneSpacing: Double {
        let v = UserDefaults.standard.double(forKey: "rooms.laneSpacing")
        return v > 0 ? v : 0.5
    }

    func cleanSegments(_ ids: [Int]) -> String? {
        guard onQueue({ mapLoaded }) else { return L10n.string("rooms_no_map") }
        if let bad = onQueue({ ids.first { rooms.names[$0] == nil } }) { return "No room \(bad)" }
        let spacing = laneSpacing
        queue.async {
            var all: [(Double, Double)] = []
            var from: SlamPose? = self.slam.pose
            let r = self.rooms
            for id in ids {
                let wps = CoveragePlanner.plan(self.slam.grid, self.navigator.params, spacing: spacing, start: from) { x, y in r.roomAt(x, y) == id }
                all += wps
                if let last = wps.last { from = SlamPose(last.0, last.1, 0) }
            }
            self.mission.clearError()
            self.mission.start(all, flag: .segment, label: ids.map { r.name($0) }.joined(separator: ", "))
        }
        return nil
    }

    func cleanZones(_ zones: [[Double]]) -> String? {
        guard onQueue({ mapLoaded }) else { return L10n.string("rooms_no_map") }
        guard !zones.isEmpty else { return "No zones" }
        let spacing = laneSpacing
        queue.async {
            var all: [(Double, Double)] = []
            var from: SlamPose? = self.slam.pose
            for z in zones {
                let wps = CoveragePlanner.plan(self.slam.grid, self.navigator.params, spacing: spacing, start: from) { x, y in RoomMap.inside(z, x, y) }
                all += wps
                if let last = wps.last { from = SlamPose(last.0, last.1, 0) }
            }
            self.activeZones = zones
            self.mission.clearError()
            self.mission.start(all, flag: .zone, label: zones.count == 1 ? "Zone" : "\(zones.count) zones")
        }
        return nil
    }

    func renameSegment(_ id: Int, _ name: String) -> String? {
        guard onQueue({ rooms.names[id] != nil }) else { return "No room \(id)" }
        let n = name.trimmingCharacters(in: .whitespaces)
        queue.async { self.rooms.names[id] = n.isEmpty ? "Room \(id)" : n; self.roomsChanged() }
        return nil
    }

    func setRestrictions(walls: [[Double]], noGo: [[Double]]) -> String? {
        queue.async { self.rooms.walls = walls; self.rooms.noGo = noGo; self.roomsChanged() }
        DispatchQueue.main.async { self.selectedItem = nil }
        return nil
    }

    /// Valetudo's "locate": blink the indicators left / right and beep for about 3 s.
    func locate() -> String? {
        DispatchQueue.main.async {
            for k in 0..<6 {
                DispatchQueue.main.asyncAfter(deadline: .now() + Double(k) * 0.5) {
                    self.bluetooth.sendData(payload: k % 2 == 0 ? "i1,0\n" : "i0,1\n")
                    AudioServicesPlaySystemSound(1057)
                }
            }
            DispatchQueue.main.asyncAfter(deadline: .now() + 3.2) { self.bluetooth.sendData(payload: "i0,0\n") }
        }
        return nil
    }

    // MARK: ValetudoRobot getters (any queue but `queue` itself is fine too)

    var vGrid: OccupancyGrid? { onQueue { mapLoaded ? slam.grid : nil } }
    var vRooms: RoomMap { onQueue { RoomMap.fromJson(rooms.toJson()) } }
    var vPose: SlamPose { onQueue { slam.pose } }
    var vStatus: CleaningMission.Status { onQueue { mapLoaded ? mission.status(at: slam.pose) : .error } }
    var vFlag: CleaningMission.Flag { onQueue { mission.flag } }
    var vError: String { onQueue { mapLoaded ? mission.error : "No saved map: build one in Indoor SLAM" } }
    var vPlannedPath: [(Double, Double)] { onQueue { navigator.path } }
    var vGoToTarget: (Double, Double)? { onQueue { mission.flag == .target ? mission.target : nil } }
    var vActiveZones: [[Double]] { onQueue { activeZones } }

    // MARK: Settings and MQTT

    private func mqttSettings() -> ValetudoMqtt.Settings {
        let d = UserDefaults.standard
        var s = ValetudoMqtt.Settings()
        s.enabled = d.bool(forKey: "rooms.mqtt.enabled")
        s.host = d.string(forKey: "rooms.mqtt.host") ?? ""
        s.port = d.object(forKey: "rooms.mqtt.port") as? Int ?? 1883
        s.username = d.string(forKey: "rooms.mqtt.user") ?? ""
        s.password = d.string(forKey: "rooms.mqtt.pass") ?? ""
        s.prefix = d.string(forKey: "rooms.mqtt.prefix") ?? "valetudo"
        s.identifier = d.string(forKey: "rooms.mqtt.id") ?? "openbot"
        s.homeAssistant = d.object(forKey: "rooms.mqtt.hass") as? Bool ?? true
        return s
    }

    private func startMqtt() {
        let s = mqttSettings()
        guard s.enabled, !s.host.isEmpty else { mqttStatus = ""; return }
        let m = ValetudoMqtt(robot: self, api: api, settings: s)
        m.onStatus = { [weak self] text in DispatchQueue.main.async { self?.mqttStatus = text } }
        mqtt = m
        m.start()
    }

    private func stopMqtt() {
        let m = mqtt
        mqtt = nil
        DispatchQueue.global(qos: .utility).async { m?.stop() }
        mqttStatus = ""
    }

    @objc private func settingsTapped() {
        let vc = RoomsSettingsController(settings: mqttSettings(), laneSpacing: laneSpacing)
        vc.onSave = { [weak self] s, spacing in
            let d = UserDefaults.standard
            d.set(min(1.5, max(0.25, spacing)), forKey: "rooms.laneSpacing")
            d.set(s.enabled, forKey: "rooms.mqtt.enabled")
            d.set(s.host, forKey: "rooms.mqtt.host")
            d.set(s.port, forKey: "rooms.mqtt.port")
            d.set(s.username, forKey: "rooms.mqtt.user")
            d.set(s.password, forKey: "rooms.mqtt.pass")
            d.set(s.prefix, forKey: "rooms.mqtt.prefix")
            d.set(s.identifier, forKey: "rooms.mqtt.id")
            d.set(s.homeAssistant, forKey: "rooms.mqtt.hass")
            self?.stopMqtt()
            self?.startMqtt()
        }
        present(UINavigationController(rootViewController: vc), animated: true)
    }

    /// The SLAM, planner and controller chosen on Indoor SLAM apply here too.
    private func applySavedAlgorithms() {
        let d = UserDefaults.standard
        let s = SlamType(rawValue: d.integer(forKey: "slam.algo.slam")) ?? .cartographer
        let l = SlamLocalizer(rawValue: d.integer(forKey: "slam.algo.localizer")) ?? .scanMatching
        let p = NavPlanner(rawValue: d.integer(forKey: "slam.algo.planner")) ?? .navfn
        let c = NavController(rawValue: d.integer(forKey: "slam.algo.controller")) ?? .rpp
        queue.async {
            self.slam.setEngine(s)
            self.slam.setLocalizer(l)
            self.navigator.params.planner = p
            self.navigator.params.controller = c
        }
    }

    private func toast(_ text: String) {
        guard presentedViewController == nil else { return }
        let a = UIAlertController(title: nil, message: text, preferredStyle: .alert)
        present(a, animated: true)
        DispatchQueue.main.asyncAfter(deadline: .now() + 1.3) { a.dismiss(animated: true) }
    }
}

// MARK: - Settings sheet

final class RoomsSettingsController: UIViewController {
    var onSave: ((ValetudoMqtt.Settings, Double) -> Void)?
    private var settings: ValetudoMqtt.Settings
    private let spacing = UITextField(), host = UITextField(), port = UITextField(), user = UITextField()
    private let pass = UITextField(), prefix = UITextField(), ident = UITextField()
    private let enabled = UISwitch(), hass = UISwitch()

    init(settings: ValetudoMqtt.Settings, laneSpacing: Double) {
        self.settings = settings
        super.init(nibName: nil, bundle: nil)
        spacing.text = String(format: "%.2f", laneSpacing)
    }

    required init?(coder: NSCoder) { fatalError("init(coder:) is not used") }

    override func viewDidLoad() {
        super.viewDidLoad()
        view.backgroundColor = .systemBackground
        title = L10n.string("rooms_settings")
        navigationItem.leftBarButtonItem = UIBarButtonItem(barButtonSystemItem: .cancel, target: self, action: #selector(cancel))
        navigationItem.rightBarButtonItem = UIBarButtonItem(barButtonSystemItem: .save, target: self, action: #selector(save))
        let scroll = UIScrollView()
        scroll.translatesAutoresizingMaskIntoConstraints = false
        view.addSubview(scroll)
        let stack = UIStackView()
        stack.axis = .vertical
        stack.spacing = 6
        stack.translatesAutoresizingMaskIntoConstraints = false
        scroll.addSubview(stack)
        NSLayoutConstraint.activate([
            scroll.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor),
            scroll.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            scroll.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            scroll.bottomAnchor.constraint(equalTo: view.bottomAnchor),
            stack.topAnchor.constraint(equalTo: scroll.contentLayoutGuide.topAnchor, constant: 16),
            stack.leadingAnchor.constraint(equalTo: scroll.frameLayoutGuide.leadingAnchor, constant: 16),
            stack.trailingAnchor.constraint(equalTo: scroll.frameLayoutGuide.trailingAnchor, constant: -16),
            stack.bottomAnchor.constraint(equalTo: scroll.contentLayoutGuide.bottomAnchor, constant: -20),
        ])
        func field(_ key: String, _ f: UITextField, _ text: String?, _ kb: UIKeyboardType = .default) {
            let l = UILabel()
            l.text = L10n.string(key)
            l.font = .systemFont(ofSize: 13)
            stack.addArrangedSubview(l)
            f.text = text ?? f.text
            f.borderStyle = .roundedRect
            f.keyboardType = kb
            f.autocapitalizationType = .none
            f.autocorrectionType = .no
            stack.addArrangedSubview(f)
        }
        func toggle(_ key: String, _ s: UISwitch, _ on: Bool) {
            let l = UILabel()
            l.text = L10n.string(key)
            l.numberOfLines = 0
            s.isOn = on
            let row = UIStackView(arrangedSubviews: [l, s])
            row.spacing = 8
            stack.addArrangedSubview(row)
        }
        field("rooms_lane_spacing", spacing, nil, .decimalPad)
        let head = UILabel()
        head.text = L10n.string("rooms_mqtt")
        head.font = .systemFont(ofSize: 15, weight: .bold)
        head.numberOfLines = 0
        stack.setCustomSpacing(18, after: spacing)
        stack.addArrangedSubview(head)
        toggle("rooms_mqtt_enable", enabled, settings.enabled)
        field("rooms_mqtt_host", host, settings.host, .URL)
        field("rooms_mqtt_port", port, String(settings.port), .numberPad)
        field("rooms_mqtt_user", user, settings.username)
        pass.isSecureTextEntry = true
        field("rooms_mqtt_pass", pass, settings.password)
        field("rooms_mqtt_prefix", prefix, settings.prefix)
        field("rooms_mqtt_id", ident, settings.identifier)
        toggle("rooms_mqtt_hass", hass, settings.homeAssistant)
    }

    @objc private func cancel() { dismiss(animated: true) }

    @objc private func save() {
        settings.enabled = enabled.isOn
        settings.host = (host.text ?? "").trimmingCharacters(in: .whitespaces)
        settings.port = Int(port.text ?? "") ?? 1883
        settings.username = user.text ?? ""
        settings.password = pass.text ?? ""
        settings.prefix = (prefix.text ?? "").trimmingCharacters(in: .whitespaces)
        settings.identifier = (ident.text ?? "").trimmingCharacters(in: .whitespaces)
        settings.homeAssistant = hass.isOn
        let sp = Double((spacing.text ?? "").replacingOccurrences(of: ",", with: ".")) ?? 0.5
        onSave?(settings, sp)
        dismiss(animated: true)
    }
}

// MARK: - Map view with rooms, zones and restrictions

final class RoomsMapView: SlamMapView {
    enum Tool: Int { case select, goto, zone, noGo, wall, home }

    var tool: Tool = .select
    var onShape: ((Tool, (Double, Double), (Double, Double)) -> Void)?
    var plan: [(Double, Double)] = []
    private var rooms = RoomMap()
    private var version = -1, drawnVersion = -2
    private var selected: Set<Int> = []
    private var drawnSelected: Set<Int> = []
    private var item: (kind: Int, index: Int)?
    private var roomImage: CGImage?
    private var centroids: [Int: (Double, Double)] = [:]
    private var draft: [Double]?
    private static let palette: [(CGFloat, CGFloat, CGFloat)] = [
        (0.10, 0.63, 0.63), (0.48, 0.75, 0.22), (0.87, 0.34, 0.09), (0.97, 0.78, 0.25),
        (0.60, 0.40, 0.80), (0.29, 0.56, 0.89), (0.90, 0.45, 0.45), (0.55, 0.43, 0.39),
    ]

    override init(frame: CGRect) {
        super.init(frame: frame)
        accessibilityLabel = "Rooms and zones map"
        onDraw = { [weak self] state, a, b in
            guard let self = self else { return }
            switch state {
            case .began, .changed: self.draft = [a.0, a.1, b.0, b.1]
            case .ended:
                self.draft = nil
                if hypot(b.0 - a.0, b.1 - a.1) > 0.2 { self.onShape?(self.tool, a, b) }
            default: self.draft = nil
            }
            self.setNeedsDisplay()
        }
    }

    required init?(coder: NSCoder) { fatalError("init(coder:) is not used") }

    func setRooms(_ r: RoomMap, version v: Int, selected s: Set<Int>, item i: (kind: Int, index: Int)?) {
        rooms = r; version = v; selected = s; item = i
        setNeedsDisplay()
    }

    private func rebuildImage() {
        let r = rooms
        guard r.w > 0, r.labels.count == r.w * r.h else { roomImage = nil; centroids = [:]; return }
        var px = [UInt8](repeating: 0, count: r.w * r.h * 4)
        for y in 0..<r.h {
            let row = (r.h - 1 - y) * r.w
            for x in 0..<r.w {
                let id = Int(r.labels[y * r.w + x])
                guard id > 0 else { continue }
                let c = RoomsMapView.palette[(id - 1) % RoomsMapView.palette.count]
                let a: CGFloat = selected.isEmpty || selected.contains(id) ? 0.47 : 0.18
                let i = (row + x) * 4
                px[i] = UInt8(c.0 * a * 255); px[i + 1] = UInt8(c.1 * a * 255); px[i + 2] = UInt8(c.2 * a * 255); px[i + 3] = UInt8(a * 255)
            }
        }
        if let provider = CGDataProvider(data: Data(px) as CFData) {
            roomImage = CGImage(width: r.w, height: r.h, bitsPerComponent: 8, bitsPerPixel: 32, bytesPerRow: r.w * 4,
                                space: CGColorSpaceCreateDeviceRGB(), bitmapInfo: CGBitmapInfo(rawValue: CGImageAlphaInfo.premultipliedLast.rawValue),
                                provider: provider, decode: nil, shouldInterpolate: false, intent: .defaultIntent)
        }
        centroids = [:]
        for id in r.roomIds { if let c = r.centroid(id) { centroids[id] = c } }
    }

    override func drawOverlay(_ ctx: CGContext) {
        let r = rooms
        if drawnVersion != version || drawnSelected != selected {
            drawnVersion = version; drawnSelected = selected
            rebuildImage()
        }
        if let img = roomImage {
            ctx.saveGState()
            ctx.interpolationQuality = .none
            let rect = CGRect(x: sx(r.originX), y: sy(r.originY + Double(r.h) * r.res),
                              width: CGFloat(Double(r.w) * r.res) * scale, height: CGFloat(Double(r.h) * r.res) * scale)
            ctx.translateBy(x: 0, y: rect.midY * 2)
            ctx.scaleBy(x: 1, y: -1)
            ctx.draw(img, in: rect)
            ctx.restoreGState()
        }
        func polygon(_ p: [Double], fill: UIColor, stroke: UIColor, selected sel: Bool) {
            let path = UIBezierPath()
            path.move(to: CGPoint(x: sx(p[0]), y: sy(p[1])))
            for i in 1..<(p.count / 2) { path.addLine(to: CGPoint(x: sx(p[2 * i]), y: sy(p[2 * i + 1]))) }
            path.close()
            fill.setFill(); path.fill()
            path.lineWidth = sel ? 3 : 2
            if sel { path.setLineDash([8, 5], count: 2, phase: 0) }
            stroke.setStroke(); path.stroke()
        }
        let blue = UIColor(red: 0.10, green: 0.46, blue: 0.82, alpha: 1), red = UIColor(red: 0.83, green: 0.18, blue: 0.18, alpha: 1)
        for (i, z) in r.zones.enumerated() { polygon(z, fill: blue.withAlphaComponent(0.2), stroke: blue, selected: item?.kind == 0 && item?.index == i) }
        for (i, z) in r.noGo.enumerated() { polygon(z, fill: red.withAlphaComponent(0.27), stroke: red, selected: item?.kind == 1 && item?.index == i) }
        for (i, s) in r.walls.enumerated() {
            let p = UIBezierPath()
            p.move(to: CGPoint(x: sx(s[0]), y: sy(s[1])))
            p.addLine(to: CGPoint(x: sx(s[2]), y: sy(s[3])))
            p.lineWidth = item?.kind == 2 && item?.index == i ? 6 : 4
            p.lineCapStyle = .round
            red.setStroke(); p.stroke()
        }
        if plan.count > 1 {
            let p = UIBezierPath()
            p.move(to: CGPoint(x: sx(plan[0].0), y: sy(plan[0].1)))
            for q in plan.dropFirst() { p.addLine(to: CGPoint(x: sx(q.0), y: sy(q.1))) }
            p.lineWidth = 1.5
            p.setLineDash([6, 4], count: 2, phase: 0)
            UIColor(red: 0.42, green: 0.11, blue: 0.6, alpha: 0.8).setStroke()
            p.stroke()
        }
        if let h = r.home {
            let c = UIBezierPath(arcCenter: CGPoint(x: sx(h.0), y: sy(h.1)), radius: 9, startAngle: 0, endAngle: 2 * .pi, clockwise: true)
            UIColor(red: 0.18, green: 0.49, blue: 0.2, alpha: 1).setFill()
            c.fill()
            let a: [NSAttributedString.Key: Any] = [.font: UIFont.boldSystemFont(ofSize: 11), .foregroundColor: UIColor.white]
            let t = "H" as NSString, size = t.size(withAttributes: a)
            t.draw(at: CGPoint(x: sx(h.0) - size.width / 2, y: sy(h.1) - size.height / 2), withAttributes: a)
        }
        for (id, c) in centroids {
            let attrs: [NSAttributedString.Key: Any] = [.font: UIFont.boldSystemFont(ofSize: 13), .foregroundColor: UIColor(white: 0.15, alpha: 1),
                                                        .strokeColor: UIColor.white, .strokeWidth: -3]
            let t = r.name(id) as NSString, size = t.size(withAttributes: attrs)
            t.draw(at: CGPoint(x: sx(c.0) - size.width / 2, y: sy(c.1) - size.height / 2), withAttributes: attrs)
        }
        if let d = draft {
            if tool == .wall {
                let p = UIBezierPath()
                p.move(to: CGPoint(x: sx(d[0]), y: sy(d[1])))
                p.addLine(to: CGPoint(x: sx(d[2]), y: sy(d[3])))
                p.lineWidth = 4
                red.setStroke(); p.stroke()
            } else {
                let box = RoomMap.box(d[0], d[1], d[2], d[3])
                if tool == .zone { polygon(box, fill: blue.withAlphaComponent(0.2), stroke: blue, selected: true) } else {
                    polygon(box, fill: red.withAlphaComponent(0.27), stroke: red, selected: true)
                }
            }
        }
    }
}
