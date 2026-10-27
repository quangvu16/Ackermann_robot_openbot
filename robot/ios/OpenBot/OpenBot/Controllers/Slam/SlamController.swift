//
//  SlamController.swift
//  OpenBot
//
//  Indoor SLAM & navigation screen (SlamCore.swift), mirroring SlamFragment on
//  Android. SLAM and the navigator run on one serial queue; the screen only
//  draws snapshots they hand back, so nothing is read while it is written.
//

import CoreMotion
import UIKit

final class SlamController: UIViewController {
    private let mapView = SlamMapView()
    private let mapStatus = PaddedLabel()
    private let statusLabel = UILabel()
    private let sourceLabel = UILabel()
    private let modePicker = UISegmentedControl(items: [L10n.string("slam_mode_map"), L10n.string("slam_mode_localize")])
    private let exploreButton = UIButton(type: .system)
    private let cancelButton = UIButton(type: .system)

    private let queue = DispatchQueue(label: "SlamController")
    // Owned by `queue`:
    private let slam = SlamSystem(.cartographer, .indoorDefault())
    private let navigator = SlamNavigator()
    private let explorer = LidarBehaviors(mode: .avoid, params: .defaults(.avoid))
    private var exploring = false
    private var driving = false
    private var lastImageTime = 0.0
    // Main thread:
    private var timer: Timer?
    private var lastScanId: UInt32?
    private var uiExploring = false
    private var uiHasGoal = false
    private var scanInFlight = false // main thread: drop scans while the last one is being matched
    private var shownRelocalizeSeq = 0
    private let findButton = UIButton(type: .system)
    private var algorithmsText = ""

    // Gyro yaw: rotation rate about gravity, integrated (works however the phone is mounted).
    private let motion = CMMotionManager()
    private var imuYaw = Double.nan
    private let bluetooth = bluetoothDataController.shared

    override func viewDidLoad() {
        super.viewDidLoad()
        view.backgroundColor = .systemBackground
        title = L10n.string("slam_title")
        layout()
        mapView.onTap = { [weak self] x, y in self?.setGoal(x, y) }
        mapView.onLongPress = { [weak self] x, y in self?.placeCar(x, y) }
        applyAlgorithms(savedAlgorithms())
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
                // iOS gravity points down; counter-clockwise seen from above is +.
                let rate = -(w.x * g.x + w.y * g.y + w.z * g.z) / n
                self.imuYaw = (self.imuYaw.isNaN ? 0 : self.imuYaw) + rate * 0.02
            }
        }
        timer = Timer.scheduledTimer(withTimeInterval: 0.1, repeats: true) { [weak self] _ in self?.tick() }
    }

    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        timer?.invalidate()
        timer = nil
        motion.stopDeviceMotionUpdates()
        queue.async {
            self.exploring = false
            self.navigator.cancel()
        }
        sendControl(0, 0) // never leave the car driving itself off screen
        SharedLidar.release()
        UIApplication.shared.isIdleTimerDisabled = false
    }

    // MARK: Layout

    private func layout() {
        mapView.translatesAutoresizingMaskIntoConstraints = false
        view.addSubview(mapView)
        let centre = UIButton(type: .system)
        var cfg = UIButton.Configuration.tinted()
        cfg.title = L10n.string("slam_center")
        cfg.cornerStyle = .capsule
        centre.configuration = cfg
        centre.addTarget(self, action: #selector(recentre), for: .touchUpInside)
        centre.translatesAutoresizingMaskIntoConstraints = false
        view.addSubview(centre)
        mapStatus.font = .monospacedDigitSystemFont(ofSize: 12, weight: .regular)
        mapStatus.textColor = .white
        mapStatus.backgroundColor = UIColor.black.withAlphaComponent(0.7)
        mapStatus.numberOfLines = 0
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
            mapView.heightAnchor.constraint(equalTo: view.safeAreaLayoutGuide.heightAnchor, multiplier: 0.56),
            // Top-left: the top-right corner belongs to the STOP button.
            centre.topAnchor.constraint(equalTo: mapView.topAnchor, constant: 10),
            centre.leadingAnchor.constraint(equalTo: mapView.leadingAnchor, constant: 10),
            mapStatus.leadingAnchor.constraint(equalTo: mapView.leadingAnchor, constant: 8),
            mapStatus.bottomAnchor.constraint(equalTo: mapView.bottomAnchor, constant: -8),
            mapStatus.trailingAnchor.constraint(lessThanOrEqualTo: mapView.trailingAnchor, constant: -8),
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
        statusLabel.heightAnchor.constraint(greaterThanOrEqualToConstant: 30).isActive = true
        stack.addArrangedSubview(statusLabel)

        modePicker.selectedSegmentIndex = 0
        modePicker.addTarget(self, action: #selector(modeChanged), for: .valueChanged)
        stack.addArrangedSubview(modePicker)

        var ex = UIButton.Configuration.filled()
        ex.cornerStyle = .large
        exploreButton.configuration = ex
        exploreButton.addTarget(self, action: #selector(exploreTapped), for: .touchUpInside)
        var cc = UIButton.Configuration.bordered()
        cc.title = L10n.string("slam_cancel")
        cc.cornerStyle = .large
        cancelButton.configuration = cc
        cancelButton.addTarget(self, action: #selector(cancelTapped), for: .touchUpInside)
        var fc = UIButton.Configuration.bordered()
        fc.title = L10n.string("slam_find")
        findButton.configuration = fc
        findButton.addTarget(self, action: #selector(findTapped), for: .touchUpInside)
        let actions = UIStackView(arrangedSubviews: [exploreButton, cancelButton, findButton])
        actions.distribution = .fillEqually
        actions.spacing = 8
        actions.heightAnchor.constraint(equalToConstant: 50).isActive = true
        stack.addArrangedSubview(actions)

        let files = UIStackView()
        files.distribution = .fillEqually
        files.spacing = 6
        for (key, sel) in [("slam_algorithms", #selector(algorithmsTapped)), ("slam_save", #selector(saveTapped)), ("slam_load", #selector(loadTapped)),
                           ("slam_clear", #selector(clearTapped)), ("slam_export", #selector(exportTapped))] {
            var c = UIButton.Configuration.bordered()
            c.title = L10n.string(key)
            c.titleLineBreakMode = .byWordWrapping
            let b = UIButton(configuration: c)
            b.addTarget(self, action: sel, for: .touchUpInside)
            files.addArrangedSubview(b)
        }
        stack.addArrangedSubview(files)

        let layers = UIStackView()
        layers.distribution = .fillEqually
        layers.spacing = 6
        for (i, key) in ["slam_layer_map", "slam_layer_scan", "slam_layer_path", "slam_layer_trail"].enumerated() {
            var c = UIButton.Configuration.tinted()
            c.title = L10n.string(key)
            c.image = UIImage(systemName: "checkmark")
            c.imagePadding = 4
            let b = UIButton(configuration: c)
            b.tag = i
            b.addTarget(self, action: #selector(layerTapped(_:)), for: .touchUpInside)
            layers.addArrangedSubview(b)
        }
        stack.addArrangedSubview(layers)

        sourceLabel.font = .systemFont(ofSize: 12)
        sourceLabel.textColor = .secondaryLabel
        sourceLabel.numberOfLines = 0
        stack.addArrangedSubview(sourceLabel)
        let help = UILabel()
        help.text = L10n.string("slam_help")
        help.font = .systemFont(ofSize: 13)
        help.textColor = .secondaryLabel
        help.numberOfLines = 0
        stack.addArrangedSubview(help)
        showButtons()
    }

    // MARK: Actions

    @objc private func recentre() { mapView.follow = true; mapView.setNeedsDisplay() }

    @objc private func modeChanged() {
        let mapping = modePicker.selectedSegmentIndex == 0
        queue.async { self.slam.mapping = mapping }
    }

    @objc private func exploreTapped() {
        uiExploring.toggle()
        let on = uiExploring
        queue.async {
            self.exploring = on
            if on { self.navigator.cancel() }
        }
        if !on { sendControl(0, 0) }
        showButtons()
    }

    @objc private func cancelTapped() {
        uiExploring = false
        queue.async { self.exploring = false; self.navigator.cancel() }
        sendControl(0, 0)
        showButtons()
    }

    private func setGoal(_ x: Double, _ y: Double) {
        uiExploring = false
        queue.async { self.exploring = false; self.navigator.setGoal(x, y) }
        showButtons()
    }

    // MARK: Algorithm choices (saved on the phone)

    typealias Algorithms = (slam: SlamType, localizer: SlamLocalizer, planner: NavPlanner, controller: NavController)

    private func savedAlgorithms() -> Algorithms {
        let d = UserDefaults.standard
        return (SlamType(rawValue: d.integer(forKey: "slam.algo.slam")) ?? .cartographer,
                SlamLocalizer(rawValue: d.integer(forKey: "slam.algo.localizer")) ?? .scanMatching,
                NavPlanner(rawValue: d.integer(forKey: "slam.algo.planner")) ?? .navfn,
                NavController(rawValue: d.integer(forKey: "slam.algo.controller")) ?? .rpp)
    }

    private func applyAlgorithms(_ a: Algorithms) {
        let d = UserDefaults.standard
        d.set(a.slam.rawValue, forKey: "slam.algo.slam")
        d.set(a.localizer.rawValue, forKey: "slam.algo.localizer")
        d.set(a.planner.rawValue, forKey: "slam.algo.planner")
        d.set(a.controller.rawValue, forKey: "slam.algo.controller")
        algorithmsText = [a.slam.label, a.localizer.label, a.planner.label, a.controller.label].joined(separator: " · ")
        queue.async {
            self.slam.setEngine(a.slam)
            self.slam.setLocalizer(a.localizer)
            self.navigator.params.planner = a.planner
            self.navigator.params.controller = a.controller
            if let g = self.navigator.goal { self.navigator.setGoal(g.0, g.1) } // re-plan with the new planner
        }
    }

    /// Four pickers (SLAM, localizer, planner, controller), each a button with a menu.
    @objc private func algorithmsTapped() {
        let vc = UIViewController()
        vc.title = L10n.string("slam_algorithms")
        vc.view.backgroundColor = .systemBackground
        var current = savedAlgorithms()
        let stack = UIStackView()
        stack.axis = .vertical
        stack.spacing = 6
        stack.translatesAutoresizingMaskIntoConstraints = false
        func row(_ titleKey: String, _ labels: [String], _ selected: Int, _ pick: @escaping (Int) -> Void) {
            let t = UILabel()
            t.text = L10n.string(titleKey)
            t.font = .boldSystemFont(ofSize: 15)
            stack.addArrangedSubview(t)
            let b = UIButton(type: .system)
            var c = UIButton.Configuration.bordered()
            c.title = labels[selected]
            b.configuration = c
            b.showsMenuAsPrimaryAction = true
            b.menu = UIMenu(children: labels.enumerated().map { i, l in
                UIAction(title: l, state: i == selected ? .on : .off) { _ in
                    pick(i)
                    var cc = b.configuration
                    cc?.title = l
                    b.configuration = cc
                }
            })
            stack.addArrangedSubview(b)
        }
        row("slam_algo_slam", SlamType.allCases.map { $0.label }, current.slam.rawValue) { current.slam = SlamType(rawValue: $0)! }
        row("slam_algo_localizer", SlamLocalizer.allCases.map { $0.label }, current.localizer.rawValue) { current.localizer = SlamLocalizer(rawValue: $0)! }
        row("slam_algo_planner", NavPlanner.allCases.map { $0.label }, current.planner.rawValue) { current.planner = NavPlanner(rawValue: $0)! }
        row("slam_algo_controller", NavController.allCases.map { $0.label }, current.controller.rawValue) { current.controller = NavController(rawValue: $0)! }
        let note = UILabel()
        note.text = L10n.string("slam_algo_note")
        note.font = .systemFont(ofSize: 13)
        note.textColor = .secondaryLabel
        note.numberOfLines = 0
        stack.addArrangedSubview(note)
        var ok = UIButton.Configuration.filled()
        ok.title = "OK"
        let done = UIButton(configuration: ok, primaryAction: UIAction { [weak self, weak vc] _ in
            self?.applyAlgorithms(current)
            vc?.dismiss(animated: true)
        })
        stack.addArrangedSubview(done)
        vc.view.addSubview(stack)
        NSLayoutConstraint.activate([
            stack.leadingAnchor.constraint(equalTo: vc.view.layoutMarginsGuide.leadingAnchor),
            stack.trailingAnchor.constraint(equalTo: vc.view.layoutMarginsGuide.trailingAnchor),
            stack.topAnchor.constraint(equalTo: vc.view.safeAreaLayoutGuide.topAnchor, constant: 20),
        ])
        if let sheet = vc.sheetPresentationController { sheet.detents = [.medium(), .large()] }
        present(vc, animated: true)
    }

    /// Find the car on the map with no hint (branch and bound over the whole map).
    @objc private func findTapped() {
        queue.async { self.slam.requestGlobalLocalization() }
        toast(L10n.string("slam_finding"))
    }

    private func placeCar(_ x: Double, _ y: Double) {
        queue.async { self.slam.resetPose(SlamPose(x, y, self.slam.pose.theta)) }
        toast(L10n.string("slam_pose_set"))
    }

    @objc private func layerTapped(_ b: UIButton) {
        switch b.tag {
        case 0: mapView.showMap.toggle()
        case 1: mapView.showScan.toggle()
        case 2: mapView.showPath.toggle()
        default: mapView.showTrail.toggle()
        }
        let on = [mapView.showMap, mapView.showScan, mapView.showPath, mapView.showTrail][b.tag]
        var c = b.configuration
        c?.image = on ? UIImage(systemName: "checkmark") : nil
        b.configuration = c
        mapView.setNeedsDisplay()
    }

    private var mapURL: URL {
        let dir = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0].appendingPathComponent("slam")
        try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        return dir.appendingPathComponent("map.obmap")
    }

    @objc private func saveTapped() {
        let url = mapURL
        queue.async {
            let d = self.slam.grid.data()
            DispatchQueue.main.async {
                do { try d.write(to: url); self.toast(L10n.string("slam_saved")) } catch { self.toast(error.localizedDescription) }
            }
        }
    }

    @objc private func loadTapped() {
        guard let d = try? Data(contentsOf: mapURL), let g = OccupancyGrid.read(d) else {
            toast(L10n.string("slam_no_saved"))
            return
        }
        queue.async {
            self.navigator.cancel()
            self.slam.setGrid(g)
            self.slam.mapping = false // localize on the saved map (Cartographer pure localization)
            self.slam.requestGlobalLocalization() // and find the car on it
            self.lastImageTime = 0
        }
        modePicker.selectedSegmentIndex = 1
        toast(L10n.string("slam_finding"))
    }

    @objc private func clearTapped() {
        queue.async { self.navigator.cancel(); self.slam.reset(); self.lastImageTime = 0 }
    }

    /// openbot_map.pgm + .yaml (ROS map_server format) through the share sheet.
    @objc private func exportTapped() {
        queue.async {
            let pgm = self.slam.grid.pgm(), yaml = self.slam.grid.rosYaml("openbot_map.pgm")
            DispatchQueue.main.async {
                let dir = FileManager.default.temporaryDirectory
                let p = dir.appendingPathComponent("openbot_map.pgm"), y = dir.appendingPathComponent("openbot_map.yaml")
                do {
                    try pgm.write(to: p)
                    try Data(yaml.utf8).write(to: y)
                    let share = UIActivityViewController(activityItems: [p, y], applicationActivities: nil)
                    share.popoverPresentationController?.sourceView = self.view
                    self.present(share, animated: true)
                } catch { self.toast(error.localizedDescription) }
            }
        }
    }

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
            queue.async {
                if self.driving { self.driving = false; DispatchQueue.main.async { self.sendControl(0, 0) } }
            }
        }
    }

    /// On `queue`: SLAM, then the navigator or Explore, then a snapshot for the screen.
    private func onScan(_ mm: [Int], _ yaw: Double) {
        let n = mm.count
        var a = [Double](repeating: 0, count: n), r = [Double](repeating: 0, count: n)
        for i in 0..<n {
            a[i] = -((Double(i) + 0.5) * 360 / Double(n)) * .pi / 180 // bins clockwise; + left here
            r[i] = mm[i] == 0 ? 99 : Double(mm[i]) / 1000
        }
        let pose = slam.update(angles: a, ranges: r, imuYaw: yaw, time: ProcessInfo.processInfo.systemUptime)
        var frontMin = Double.infinity
        for i in 0..<n where r[i] >= SlamEngine.minRange && r[i] < SlamEngine.maxRange && abs(a[i]) < 25 * .pi / 180 {
            frontMin = min(frontMin, r[i])
        }
        let now = ProcessInfo.processInfo.systemUptime
        var status = ""
        var command: (Double, Double)?
        if navigator.goal != nil {
            let c = navigator.step(pose: pose, grid: slam.grid, frontMin: frontMin, now: now)
            status = c.message
            command = (c.throttle, c.steer)
            if navigator.goal == nil { command = (0, 0) }
        } else if exploring {
            let d = explorer.step(LidarBehaviors.fromScan(mm, minRange: 0.15), now: now)
            status = "Exploring: " + d.reason
            command = (d.throttle, d.steer)
        } else if navigator.state == .arrived {
            status = "Arrived at the goal"
        } else if navigator.state == .noPath {
            status = "No path to the goal on this map"
        }
        if let c = command { driving = c.0 != 0 || c.1 != 0 }
        let finalCommand = command

        // Snapshot for the screen; the map picture at most twice a second.
        var image: CGImage?
        if now - lastImageTime > 0.5 {
            lastImageTime = now
            image = SlamMapView.makeImage(slam.grid)
        }
        let snap = SlamMapView.Snapshot(
            pose: pose, scan: zip(a, r).filter { $0.1 < SlamEngine.maxRange }.map { (pose.x + $0.1 * cos(pose.theta + $0.0), pose.y + $0.1 * sin(pose.theta + $0.0)) },
            trail: slam.trail, path: navigator.path, goal: navigator.goal)
        let grid = slam.grid
        let quality = Int((slam.matchQuality * 100).rounded()), scans = slam.scans
        let submapCount = slam.submapCount, loops = slam.loopClosures
        let relocSeq = slam.relocalizeSeq, relocFound = slam.relocalizeFound
        let hasGoal = navigator.goal != nil, exploringNow = exploring
        DispatchQueue.main.async {
            self.scanInFlight = false
            if relocSeq != self.shownRelocalizeSeq {
                self.shownRelocalizeSeq = relocSeq
                self.toast(L10n.string(relocFound ? "slam_found" : "slam_not_found"))
            }
            if let c = finalCommand { self.sendControl(c.0, c.1) }
            if let img = image { self.mapView.setImage(img, grid: grid) }
            self.mapView.snapshot = snap
            self.mapView.setNeedsDisplay()
            self.mapStatus.text = String(format: L10n.string("slam_status"), pose.text, Int32(quality), Int32(scans))
                + " · " + String(format: L10n.string("slam_graph"), Int32(submapCount), Int32(loops)) + "\n" + self.algorithmsText
            self.statusLabel.text = status
            self.uiHasGoal = hasGoal
            self.uiExploring = exploringNow
            self.showButtons()
        }
    }

    private func sendControl(_ throttle: Double, _ steer: Double) {
        let w = LidarBehaviors.toWheels(throttle: throttle, steer: steer)
        let speed = GameController.shared.selectedSpeedMode.rawValue
        bluetooth.sendData(payload: "c" + String(Int(Float(w.left) * speed)) + "," + String(Int(Float(w.right) * speed)) + "\n")
    }

    private func showButtons() {
        var c = exploreButton.configuration
        c?.title = L10n.string(uiExploring ? "slam_stop" : "slam_explore")
        c?.baseBackgroundColor = uiExploring ? .systemRed : UIColor(red: 0.29, green: 0.48, blue: 1, alpha: 1)
        exploreButton.configuration = c
        cancelButton.isEnabled = uiHasGoal || uiExploring
    }

    private func toast(_ text: String) {
        let a = UIAlertController(title: nil, message: text, preferredStyle: .alert)
        present(a, animated: true)
        DispatchQueue.main.asyncAfter(deadline: .now() + 1.3) { a.dismiss(animated: true) }
    }
}

/// A label with a little padding, for the overlay on the map.
final class PaddedLabel: UILabel {
    override func drawText(in rect: CGRect) { super.drawText(in: rect.insetBy(dx: 6, dy: 4)) }
    override var intrinsicContentSize: CGSize {
        let s = super.intrinsicContentSize
        return CGSize(width: s.width + 12, height: s.height + 8)
    }
}

// MARK: - The map view

/// RViz-style map: grid (white free, black wall, grey unknown), 1 m lines,
/// trail, path, goal, scan and the car. Pan, pinch, double-tap to follow,
/// tap = goal, long-press = place the car.
final class SlamMapView: UIView {
    struct Snapshot {
        var pose = SlamPose.zero
        var scan: [(Double, Double)] = []
        var trail: [(Double, Double)] = []
        var path: [(Double, Double)] = []
        var goal: (Double, Double)?
    }

    var onTap: ((Double, Double) -> Void)?
    var onLongPress: ((Double, Double) -> Void)?
    var snapshot = Snapshot()
    var showMap = true, showScan = true, showPath = true, showTrail = true
    var follow = true
    private var image: CGImage?
    private var gridInfo: (originX: Double, originY: Double, res: Double, w: Int, h: Int)?
    private var scale: CGFloat = 60 // points per metre
    private var focus = (0.0, 0.0)

    override init(frame: CGRect) {
        super.init(frame: frame)
        backgroundColor = UIColor(white: 0.71, alpha: 1)
        isAccessibilityElement = true
        accessibilityLabel = "SLAM map"
        addGestureRecognizer(UIPanGestureRecognizer(target: self, action: #selector(pan(_:))))
        addGestureRecognizer(UIPinchGestureRecognizer(target: self, action: #selector(pinch(_:))))
        let double = UITapGestureRecognizer(target: self, action: #selector(doubleTap))
        double.numberOfTapsRequired = 2
        addGestureRecognizer(double)
        let tap = UITapGestureRecognizer(target: self, action: #selector(tap(_:)))
        tap.require(toFail: double)
        addGestureRecognizer(tap)
        addGestureRecognizer(UILongPressGestureRecognizer(target: self, action: #selector(longPress(_:))))
    }

    required init?(coder: NSCoder) { fatalError("init(coder:) is not used") }

    func setImage(_ img: CGImage, grid: OccupancyGrid) {
        image = img
        gridInfo = (grid.originX, grid.originY, grid.resolution, grid.width, grid.height)
    }

    /// The grid as an image, top row first (built on the SLAM queue).
    static func makeImage(_ g: OccupancyGrid) -> CGImage? {
        let w = g.width, h = g.height
        var px = [UInt8](repeating: 0, count: w * h * 4)
        for y in 0..<h {
            let row = (h - 1 - y) * w
            for x in 0..<w {
                let v = g.logOdds[y * w + x]
                var c: UInt8
                if v > OccupancyGrid.occThreshold { c = 16 }
                else if v < OccupancyGrid.freeThreshold { c = 255 }
                else if v == 0 { c = 180 }
                else { c = UInt8(max(20, min(250, 180 - Int(v / OccupancyGrid.lMax * 150)))) }
                let i = (row + x) * 4
                px[i] = c; px[i + 1] = c; px[i + 2] = c; px[i + 3] = 255
            }
        }
        guard let provider = CGDataProvider(data: Data(px) as CFData) else { return nil }
        return CGImage(width: w, height: h, bitsPerComponent: 8, bitsPerPixel: 32, bytesPerRow: w * 4,
                       space: CGColorSpaceCreateDeviceRGB(), bitmapInfo: CGBitmapInfo(rawValue: CGImageAlphaInfo.premultipliedLast.rawValue),
                       provider: provider, decode: nil, shouldInterpolate: false, intent: .defaultIntent)
    }

    private func sx(_ x: Double) -> CGFloat { bounds.midX + CGFloat(x - focus.0) * scale }
    private func sy(_ y: Double) -> CGFloat { bounds.midY - CGFloat(y - focus.1) * scale }
    private func world(_ p: CGPoint) -> (Double, Double) {
        (focus.0 + Double((p.x - bounds.midX) / scale), focus.1 - Double((p.y - bounds.midY) / scale))
    }

    @objc private func pan(_ g: UIPanGestureRecognizer) {
        let t = g.translation(in: self)
        follow = false
        focus.0 -= Double(t.x / scale)
        focus.1 += Double(t.y / scale)
        g.setTranslation(.zero, in: self)
        setNeedsDisplay()
    }

    @objc private func pinch(_ g: UIPinchGestureRecognizer) {
        scale = min(400, max(8, scale * g.scale))
        g.scale = 1
        setNeedsDisplay()
    }

    @objc private func doubleTap() { follow = true; setNeedsDisplay() }

    @objc private func tap(_ g: UITapGestureRecognizer) {
        let w = world(g.location(in: self))
        onTap?(w.0, w.1)
    }

    @objc private func longPress(_ g: UILongPressGestureRecognizer) {
        guard g.state == .began else { return }
        let w = world(g.location(in: self))
        onLongPress?(w.0, w.1)
    }

    override func draw(_ rect: CGRect) {
        guard let ctx = UIGraphicsGetCurrentContext() else { return }
        let s = snapshot
        if follow { focus = (s.pose.x, s.pose.y) }
        if showMap, let img = image, let gi = gridInfo {
            ctx.saveGState()
            ctx.interpolationQuality = .none
            let r = CGRect(x: sx(gi.originX), y: sy(gi.originY + Double(gi.h) * gi.res),
                           width: CGFloat(Double(gi.w) * gi.res) * scale, height: CGFloat(Double(gi.h) * gi.res) * scale)
            // CGContext draws images bottom-up in UIKit's flipped space: flip around the rect.
            ctx.translateBy(x: 0, y: r.midY * 2)
            ctx.scaleBy(x: 1, y: -1)
            ctx.draw(img, in: r)
            ctx.restoreGState()
        }
        if scale > 15 {
            UIColor.black.withAlphaComponent(0.15).setStroke()
            let p = UIBezierPath()
            let (wx0, wy1) = world(.zero), (wx1, wy0) = world(CGPoint(x: bounds.maxX, y: bounds.maxY))
            for x in stride(from: floor(wx0), through: ceil(wx1), by: 1) { p.move(to: CGPoint(x: sx(x), y: 0)); p.addLine(to: CGPoint(x: sx(x), y: bounds.maxY)) }
            for y in stride(from: floor(wy0), through: ceil(wy1), by: 1) { p.move(to: CGPoint(x: 0, y: sy(y))); p.addLine(to: CGPoint(x: bounds.maxX, y: sy(y))) }
            p.lineWidth = 1
            p.stroke()
        }
        func polyline(_ pts: [(Double, Double)], _ color: UIColor, _ width: CGFloat) {
            guard pts.count > 1 else { return }
            let p = UIBezierPath()
            p.move(to: CGPoint(x: sx(pts[0].0), y: sy(pts[0].1)))
            for q in pts.dropFirst() { p.addLine(to: CGPoint(x: sx(q.0), y: sy(q.1))) }
            p.lineWidth = width
            p.lineJoinStyle = .round
            p.lineCapStyle = .round
            color.setStroke()
            p.stroke()
        }
        if showTrail { polyline(s.trail, .systemOrange, 2) }
        if showPath { polyline(s.path, UIColor(red: 0.18, green: 0.49, blue: 0.2, alpha: 1), 3) }
        if let g = s.goal {
            let c = UIBezierPath(arcCenter: CGPoint(x: sx(g.0), y: sy(g.1)), radius: 7, startAngle: 0, endAngle: 2 * .pi, clockwise: true)
            UIColor(red: 0.85, green: 0.11, blue: 0.38, alpha: 1).setFill()
            c.fill()
            UIColor.white.setStroke()
            c.lineWidth = 2
            c.stroke()
        }
        if showScan {
            UIColor.systemRed.setFill()
            for q in s.scan { UIBezierPath(ovalIn: CGRect(x: sx(q.0) - 2, y: sy(q.1) - 2, width: 4, height: 4)).fill() }
        }
        let px = sx(s.pose.x), py = sy(s.pose.y), th = s.pose.theta
        let len = max(14, CGFloat(0.35) * scale)
        let arrow = UIBezierPath()
        arrow.move(to: CGPoint(x: px + CGFloat(cos(th)) * len, y: py - CGFloat(sin(th)) * len))
        arrow.addLine(to: CGPoint(x: px + CGFloat(cos(th + 2.5)) * len * 0.6, y: py - CGFloat(sin(th + 2.5)) * len * 0.6))
        arrow.addLine(to: CGPoint(x: px + CGFloat(cos(th - 2.5)) * len * 0.6, y: py - CGFloat(sin(th - 2.5)) * len * 0.6))
        arrow.close()
        UIColor(red: 0.12, green: 0.53, blue: 0.9, alpha: 1).setFill()
        arrow.fill()
        UIColor.white.setStroke()
        arrow.lineWidth = 1.5
        arrow.stroke()
    }
}
