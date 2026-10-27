//
//  LidarBotController.swift
//  OpenBot
//
//  LiDAR robot: Patrol, Follow, Avoid, Track and Warn (LidarBehaviors.swift),
//  with a face that shows each behaviour's mood beside the LiDAR map. Mirrors
//  LidarBotFragment on Android.
//
//  A 10 Hz timer reads the newest LiDAR data - full scans from the car's Wi-Fi
//  (SharedLidar), otherwise the car's 8 proximity sectors over Bluetooth - asks
//  the behaviour what to do and sends it like every other screen, so the STOP
//  latch and object avoidance still apply. Stale or missing LiDAR stops the car.
//

import AudioToolbox
import UIKit

final class LidarBotController: UIViewController {
    private let face = RobotFaceView()
    private let map = LidarView()
    private let sourceLabel = UILabel()
    private let modePicker = UISegmentedControl.icons(LidarBehaviors.Mode.allCases.map { mode in
        let symbol: String
        switch mode.rawValue {
        case "patrol": symbol = "repeat"
        case "follow": symbol = "figure.walk"
        case "avoid": symbol = "arrow.triangle.branch"
        case "track": symbol = "scope"
        default: symbol = "exclamationmark.triangle.fill"
        }
        return (symbol, L10n.string("lidarbot_" + mode.rawValue))
    })
    private let helpLabel = UILabel()
    private let startButton = UIButton(type: .system)
    private let statusLabel = UILabel()
    private let speedSlider = UISlider(), distanceSlider = UISlider(), angleSlider = UISlider()
    private let speedLabel = UILabel(), distanceLabel = UILabel(), angleLabel = UILabel()

    private var mode: LidarBehaviors.Mode = .follow
    private var params: [LidarBehaviors.Mode: LidarBehaviors.Params] = [:]
    private var behaviour: LidarBehaviors?
    private var timer: Timer?
    private var lastBeep: TimeInterval = 0
    private var vehicleControl = Control()
    private let bluetooth = bluetoothDataController.shared
    private let modeKey = "lidarbot_mode"
    /// Sensor link (WebSocket) button; in server mode the browser page can change the behaviour too.
    private lazy var sensorLinkItem = SensorLinkBarItem(owner: self, screen: "LiDAR robot") { [unowned self] out in
        self.addBrowserSettings(&out)
    }

    override func viewDidLoad() {
        super.viewDidLoad()
        view.backgroundColor = .systemBackground
        title = L10n.string("lidarbot_title")
        LidarBehaviors.Mode.allCases.forEach { params[$0] = .defaults($0) }
        mode = LidarBehaviors.Mode(rawValue: UserDefaults.standard.string(forKey: modeKey) ?? "") ?? .follow
        layout()
        showMode()
        navigationItem.rightBarButtonItem = sensorLinkItem
    }

    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(animated)
        SharedLidar.acquire()
        face.startAnimating()
        timer = Timer.scheduledTimer(withTimeInterval: 0.1, repeats: true) { [weak self] _ in self?.step() }
        sensorLinkItem.appeared()
    }

    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        stopBehaviour() // never leave the car driving itself off screen
        sensorLinkItem.disappeared()
        timer?.invalidate()
        timer = nil
        face.stopAnimating()
        SharedLidar.release()
    }

    // MARK: Layout

    private func layout() {
        let scroll = UIScrollView()
        scroll.translatesAutoresizingMaskIntoConstraints = false
        view.addSubview(scroll)
        let stack = UIStackView()
        stack.axis = .vertical
        stack.spacing = 10
        stack.translatesAutoresizingMaskIntoConstraints = false
        scroll.addSubview(stack)
        NSLayoutConstraint.activate([
            scroll.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor),
            scroll.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            scroll.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            scroll.bottomAnchor.constraint(equalTo: view.bottomAnchor),
            // 64 pt at the top keeps the face clear of the floating STOP button.
            stack.topAnchor.constraint(equalTo: scroll.contentLayoutGuide.topAnchor, constant: 64),
            stack.leadingAnchor.constraint(equalTo: scroll.frameLayoutGuide.leadingAnchor, constant: 16),
            stack.trailingAnchor.constraint(equalTo: scroll.frameLayoutGuide.trailingAnchor, constant: -16),
            stack.bottomAnchor.constraint(equalTo: scroll.contentLayoutGuide.bottomAnchor, constant: -24),
        ])

        map.rangeMaxMeters = 3
        map.layer.cornerRadius = 12
        map.clipsToBounds = true
        map.isAccessibilityElement = true
        map.accessibilityLabel = L10n.string("avoid_lidar_map")
        face.layer.cornerRadius = 12
        face.clipsToBounds = true
        let top = UIStackView(arrangedSubviews: [face, map])
        top.axis = .horizontal
        top.spacing = 8
        top.distribution = .fillEqually
        top.heightAnchor.constraint(equalToConstant: 220).isActive = true
        stack.addArrangedSubview(top)

        sourceLabel.font = .systemFont(ofSize: 12)
        sourceLabel.textColor = .secondaryLabel
        sourceLabel.numberOfLines = 0
        stack.addArrangedSubview(sourceLabel)

        modePicker.addTarget(self, action: #selector(modeChanged), for: .valueChanged)
        stack.addArrangedSubview(modePicker)
        helpLabel.numberOfLines = 0
        helpLabel.font = .systemFont(ofSize: 14)
        stack.addArrangedSubview(helpLabel)

        var config = UIButton.Configuration.filled()
        config.cornerStyle = .large
        startButton.configuration = config
        startButton.heightAnchor.constraint(equalToConstant: 56).isActive = true
        startButton.addTarget(self, action: #selector(startTapped), for: .touchUpInside)
        stack.addArrangedSubview(startButton)

        statusLabel.font = .systemFont(ofSize: 15, weight: .semibold)
        statusLabel.numberOfLines = 0
        statusLabel.heightAnchor.constraint(greaterThanOrEqualToConstant: 40).isActive = true
        stack.addArrangedSubview(statusLabel)

        for (label, slider, min, max) in [(speedLabel, speedSlider, 0.0, 1.0),
                                          (distanceLabel, distanceSlider, 0.2, 2.0),
                                          (angleLabel, angleSlider, 10.0, 120.0)] {
            label.font = .systemFont(ofSize: 14)
            slider.minimumValue = Float(min)
            slider.maximumValue = Float(max)
            slider.addTarget(self, action: #selector(sliderChanged(_:)), for: .valueChanged)
            stack.addArrangedSubview(label)
            stack.addArrangedSubview(slider)
        }
        speedSlider.accessibilityLabel = L10n.string("lidarbot_speed")
        distanceSlider.accessibilityLabel = L10n.string("lidarbot_distance")
        angleSlider.accessibilityLabel = L10n.string("lidarbot_angle")

        let safety = UILabel()
        safety.text = L10n.string("lidarbot_safety")
        safety.font = .systemFont(ofSize: 13)
        safety.textColor = .secondaryLabel
        safety.numberOfLines = 0
        stack.addArrangedSubview(safety)
    }

    // MARK: Browser settings (main queue)

    private func addBrowserSettings(_ out: inout [RemoteSetting]) {
        out.append(.choice("lidarbot.mode", "LiDAR robot", "Behaviour", LidarBehaviors.Mode.allCases.map { $0.rawValue },
                           get: { [unowned self] in self.mode.rawValue },
                           set: { [unowned self] v in
                               guard let m = LidarBehaviors.Mode(rawValue: v),
                                     let i = LidarBehaviors.Mode.allCases.firstIndex(of: m) else { return }
                               self.modePicker.selectedSegmentIndex = i
                               self.modeChanged()
                           }))
        out.append(.action("lidarbot.run", "LiDAR robot", "Run the behaviour", "START", "STOP",
                           get: { [unowned self] in self.behaviour != nil },
                           set: { [unowned self] on in
                               if on && self.behaviour == nil { self.startBehaviour() } else if !on && self.behaviour != nil { self.stopBehaviour() }
                           }))
        out.append(.range("lidarbot.speed", "LiDAR robot", "Speed (fraction)", 0, 1, 0.05,
                          get: { [unowned self] in self.current.speed },
                          set: { [unowned self] v in var p = self.current; p.speed = v; self.current = p; self.showMode() }))
        out.append(.range("lidarbot.distance", "LiDAR robot", "Distance (m)", 0.2, 2.0, 0.05,
                          get: { [unowned self] in self.current.responseDist },
                          set: { [unowned self] v in var p = self.current; p.responseDist = v; self.current = p; self.showMode() }))
        out.append(.range("lidarbot.angle", "LiDAR robot", "Angle (± degrees)", 10, 120, 5,
                          get: { [unowned self] in self.current.laserAngle },
                          set: { [unowned self] v in var p = self.current; p.laserAngle = v; self.current = p; self.showMode() }))
    }

    // MARK: UI

    private var current: LidarBehaviors.Params {
        get { params[mode] ?? .defaults(mode) }
        set { params[mode] = newValue; behaviour?.params = newValue }
    }

    @objc private func modeChanged() {
        let m = LidarBehaviors.Mode.allCases[modePicker.selectedSegmentIndex]
        guard m != mode else { return }
        stopBehaviour()
        mode = m
        UserDefaults.standard.set(m.rawValue, forKey: modeKey)
        showMode()
    }

    private func showMode() {
        modePicker.selectedSegmentIndex = LidarBehaviors.Mode.allCases.firstIndex(of: mode) ?? 1
        helpLabel.text = L10n.string("lidarbot_" + mode.rawValue + "_help")
        let p = current
        speedSlider.value = Float(p.speed)
        distanceSlider.value = Float(p.responseDist)
        angleSlider.value = Float(p.laserAngle)
        speedSlider.isEnabled = mode != .warn // Warn never drives
        showParams()
        showRunning()
    }

    @objc private func sliderChanged(_ slider: UISlider) {
        var p = current
        if slider === speedSlider { p.speed = Double((slider.value * 20).rounded() / 20) }
        if slider === distanceSlider { p.responseDist = Double((slider.value * 20).rounded() / 20) }
        if slider === angleSlider { p.laserAngle = Double((slider.value / 5).rounded() * 5) }
        current = p
        showParams()
    }

    private func showParams() {
        let p = current
        speedLabel.text = "\(L10n.string("lidarbot_speed"))  \(Int((p.speed * 100).rounded()))%"
        distanceLabel.text = String(format: "%@  %.2f", L10n.string("lidarbot_distance"), p.responseDist)
        angleLabel.text = String(format: "%@  ±%.0f", L10n.string("lidarbot_angle"), p.laserAngle)
    }

    private func showRunning() {
        let running = behaviour != nil
        var config = startButton.configuration
        config?.baseBackgroundColor = running ? .systemRed : UIColor(red: 0.29, green: 0.48, blue: 1, alpha: 1)
        startButton.configuration = config
        startButton.setIcon(running ? "stop.fill" : "play.fill", label: L10n.string(running ? "lidarbot_stop" : "lidarbot_start"), pointSize: 28)
        if !running { statusLabel.text = "" }
    }

    @objc private func startTapped() {
        if behaviour != nil { stopBehaviour() } else { startBehaviour() }
    }

    // MARK: Behaviour

    private func startBehaviour() {
        behaviour = LidarBehaviors(mode: mode, params: current)
        UIApplication.shared.isIdleTimerDisabled = true // the face is the display
        showRunning()
    }

    private func stopBehaviour() {
        let was = behaviour != nil
        behaviour = nil
        UIApplication.shared.isIdleTimerDisabled = false
        sendControl(left: 0, right: 0)
        if was { showRunning() }
    }

    /// A fresh full scan, else fresh proximity sectors, else nil.
    private func readLidar() -> LidarBehaviors.Points? {
        let scanner = SharedLidar.scanner
        if let scan = scanner.latestScan, scanner.isFresh {
            return LidarBehaviors.fromScan(scan.rangesMm.map { Int($0) }, minRange: current.minRange)
        }
        let proximity = bluetooth.driveGate.proximity
        if proximity.hasEverReported() && proximity.ageMs() < proximityStaleAfterMs {
            return LidarBehaviors.fromSectors(proximity.sectors())
        }
        return nil
    }

    private func step() {
        let points = readLidar()
        // Map: full scan (grey when stale) plus the car's sectors.
        let scanner = SharedLidar.scanner
        map.scan = scanner.latestScan
        map.isStale = scanner.isStale
        let proximity = bluetooth.driveGate.proximity
        map.sectorsMeters = proximity.hasEverReported() && proximity.ageMs() < 1000 ? proximity.sectors() : nil
        map.marginMeters = bluetooth.driveGate.settings.marginMeters
        if let p = points {
            sourceLabel.text = p.coarse ? L10n.string("lidarbot_source_sectors")
                : String(format: L10n.string("lidarbot_source_scan"), Int32(p.count))
        } else {
            sourceLabel.text = L10n.string("lidarbot_source_none")
        }

        guard let behaviour = behaviour else {
            let mood: LidarBehaviors.Mood = points == nil ? .blind : .idle
            face.set(mood: mood, gazeDeg: 0, caption: caption(mood))
            return
        }
        let d = behaviour.step(points, now: ProcessInfo.processInfo.systemUptime)
        let wheels = LidarBehaviors.toWheels(throttle: d.throttle, steer: d.steer)
        sendControl(left: Float(wheels.left), right: Float(wheels.right))
        face.set(mood: d.mood, gazeDeg: d.gazeDeg, caption: caption(d.mood))
        statusLabel.text = d.reason
        statusLabel.textColor = RobotFaceView.color(d.mood).darker()
        if d.alarm { alarm() }
    }

    /// Same path as the other driving screens: speed mode applied, then Bluetooth.
    private func sendControl(left: Float, right: Float) {
        let control = Control(left: left, right: right)
        let speed = GameController.shared.selectedSpeedMode.rawValue
        vehicleControl = control
        bluetooth.sendData(payload: "c" + String(Int(control.getLeft() * speed)) + "," + String(Int(control.getRight() * speed)) + "\n")
    }

    /// Warn's alarm: a short sound and buzz, at most every 0.4 s.
    private func alarm() {
        let now = ProcessInfo.processInfo.systemUptime
        guard now - lastBeep >= 0.4 else { return }
        lastBeep = now
        AudioServicesPlaySystemSound(1005)
        AudioServicesPlaySystemSound(kSystemSoundID_Vibrate)
    }

    private func caption(_ mood: LidarBehaviors.Mood) -> String {
        switch mood {
        case .idle: return L10n.string("lidarbot_mood_idle")
        case .happy: return L10n.string("lidarbot_mood_happy")
        case .love: return L10n.string("lidarbot_mood_love")
        case .focused: return L10n.string("lidarbot_mood_focused")
        case .searching: return L10n.string("lidarbot_mood_searching")
        case .worried: return L10n.string("lidarbot_mood_worried")
        case .scared: return L10n.string("lidarbot_mood_scared")
        case .blind: return L10n.string("lidarbot_mood_blind")
        }
    }
}

private extension UIColor {
    func darker() -> UIColor {
        var h: CGFloat = 0, s: CGFloat = 0, b: CGFloat = 0, a: CGFloat = 0
        guard getHue(&h, saturation: &s, brightness: &b, alpha: &a) else { return self }
        return UIColor(hue: h, saturation: s, brightness: b * 0.65, alpha: a)
    }
}

// MARK: - The face

/// A desk-robot face: two glowing eyes and a mouth on a dark screen. Mood and
/// gaze come from the running behaviour. Mirrors RobotFaceView.java.
final class RobotFaceView: UIView {
    private var mood: LidarBehaviors.Mood = .idle
    private var gazeDeg = 0.0
    private var shownGaze = 0.0
    private var caption = ""
    private var nextBlink = ProcessInfo.processInfo.systemUptime + 2.5
    private var link: CADisplayLink?

    override init(frame: CGRect) {
        super.init(frame: frame)
        isOpaque = true
        contentMode = .redraw
        isAccessibilityElement = true
    }

    required init?(coder: NSCoder) {
        fatalError("init(coder:) is not used")
    }

    func set(mood: LidarBehaviors.Mood, gazeDeg: Double, caption: String) {
        self.mood = mood
        self.gazeDeg = max(-90, min(90, gazeDeg))
        self.caption = caption
        accessibilityLabel = caption
    }

    func startAnimating() {
        guard link == nil else { return }
        let l = CADisplayLink(target: self, selector: #selector(animationTick(_:)))
        l.preferredFramesPerSecond = 30
        l.add(to: .main, forMode: .common)
        link = l
    }

    func stopAnimating() {
        link?.invalidate()
        link = nil
    }

    @objc private func animationTick(_ link: CADisplayLink) { setNeedsDisplay() }

    static func color(_ mood: LidarBehaviors.Mood) -> UIColor {
        func hex(_ v: UInt32) -> UIColor {
            UIColor(red: CGFloat((v >> 16) & 0xFF) / 255, green: CGFloat((v >> 8) & 0xFF) / 255, blue: CGFloat(v & 0xFF) / 255, alpha: 1)
        }
        switch mood {
        case .happy: return hex(0x4DD0E1)
        case .love: return hex(0xF06292)
        case .focused: return hex(0x64B5F6)
        case .searching: return hex(0x9575CD)
        case .worried: return hex(0xFFB74D)
        case .scared: return hex(0xEF5350)
        case .blind: return hex(0x90A4AE)
        case .idle: return hex(0x78909C)
        }
    }

    override func draw(_ rect: CGRect) {
        guard let g = UIGraphicsGetCurrentContext() else { return }
        let w = bounds.width, h = bounds.height
        let now = ProcessInfo.processInfo.systemUptime
        UIColor(red: 0x0E / 255, green: 0x15 / 255, blue: 0x1D / 255, alpha: 1).setFill()
        g.fill(bounds)
        let color = Self.color(mood)
        color.setFill()
        color.setStroke()
        let unit = min(w, h * 1.4)
        let lw = unit * 0.035

        var target = gazeDeg
        if mood == .searching { target = 60 * sin(now / 0.6) }
        shownGaze += (target - shownGaze) * 0.25
        let shake = mood == .scared ? unit * 0.012 * CGFloat(sin(now / 0.025)) : 0
        let dx = CGFloat(-shownGaze / 90) * w * 0.16 + shake // + gaze (left) moves the eyes left
        let cx = w / 2 + dx, cy = h * 0.44
        let eyeW = unit * 0.2, eyeH = unit * 0.26, gap = unit * 0.17

        var open: CGFloat = 1
        if now >= nextBlink {
            let t = CGFloat((now - nextBlink) / 0.14)
            if t >= 1 { nextBlink = now + 2.5 + Double.random(in: 0...2.5) } else { open = abs(1 - 2 * t) }
        }
        if mood == .scared || mood == .blind { open = 1 }

        for side: CGFloat in [-1, 1] {
            drawEye(g, CGPoint(x: cx + side * gap, y: cy), eyeW, eyeH, open, side, lw, color)
        }
        drawMouth(CGPoint(x: cx, y: h * 0.76), unit, lw)

        if !caption.isEmpty {
            let style = NSMutableParagraphStyle()
            style.alignment = .center
            let size = max(12, unit * 0.075)
            (caption as NSString).draw(in: CGRect(x: 0, y: h - size * 1.6, width: w, height: size * 1.4), withAttributes: [
                .font: UIFont.systemFont(ofSize: size, weight: .medium),
                .foregroundColor: UIColor(white: 0.72, alpha: 1),
                .paragraphStyle: style,
            ])
        }
    }

    private func stroke(_ path: UIBezierPath, _ lw: CGFloat) {
        path.lineWidth = lw
        path.lineCapStyle = .round
        path.lineJoinStyle = .round
        path.stroke()
    }

    private func drawEye(_ g: CGContext, _ c: CGPoint, _ ew: CGFloat, _ eh: CGFloat, _ open: CGFloat,
                         _ side: CGFloat, _ lw: CGFloat, _ color: UIColor) {
        switch mood {
        case .happy:
            let p = UIBezierPath()
            p.move(to: CGPoint(x: c.x - ew / 2, y: c.y + eh * 0.15))
            p.addQuadCurve(to: CGPoint(x: c.x + ew / 2, y: c.y + eh * 0.15), controlPoint: CGPoint(x: c.x, y: c.y - eh * 0.55))
            stroke(p, lw)
            return
        case .love:
            let s = ew * 1.05 / 2
            let p = UIBezierPath()
            p.move(to: CGPoint(x: c.x, y: c.y + s * 0.9))
            p.addCurve(to: CGPoint(x: c.x, y: c.y - s * 0.35), controlPoint1: CGPoint(x: c.x - s * 1.4, y: c.y - s * 0.1),
                       controlPoint2: CGPoint(x: c.x - s * 0.6, y: c.y - s * 1.1))
            p.addCurve(to: CGPoint(x: c.x, y: c.y + s * 0.9), controlPoint1: CGPoint(x: c.x + s * 0.6, y: c.y - s * 1.1),
                       controlPoint2: CGPoint(x: c.x + s * 1.4, y: c.y - s * 0.1))
            p.close()
            p.fill()
            return
        case .blind:
            let s = ew * 0.38
            let p = UIBezierPath()
            p.move(to: CGPoint(x: c.x - s, y: c.y - s)); p.addLine(to: CGPoint(x: c.x + s, y: c.y + s))
            p.move(to: CGPoint(x: c.x - s, y: c.y + s)); p.addLine(to: CGPoint(x: c.x + s, y: c.y - s))
            stroke(p, lw)
            return
        case .scared:
            UIBezierPath(arcCenter: c, radius: ew * 0.62, startAngle: 0, endAngle: 2 * .pi, clockwise: true).fill()
            UIColor(red: 0x0E / 255, green: 0x15 / 255, blue: 0x1D / 255, alpha: 1).setFill()
            UIBezierPath(arcCenter: c, radius: ew * 0.2, startAngle: 0, endAngle: 2 * .pi, clockwise: true).fill()
            color.setFill()
            return
        default:
            break
        }
        var hh = eh * open
        if mood == .idle { hh *= 0.35 }
        if mood == .focused { hh *= 0.75 }
        UIBezierPath(roundedRect: CGRect(x: c.x - ew / 2, y: c.y - hh / 2, width: ew, height: hh), cornerRadius: ew * 0.35).fill()
        if mood == .worried {
            let by = c.y - eh * 0.62
            let p = UIBezierPath()
            // Raised in the middle: the inner end (nearer the centre line) is higher.
            p.move(to: CGPoint(x: c.x - side * ew * 0.55, y: by - eh * 0.12))
            p.addLine(to: CGPoint(x: c.x + side * ew * 0.45, y: by + eh * 0.1))
            stroke(p, lw)
        } else if mood == .focused {
            let by = c.y - eh * 0.5
            let p = UIBezierPath()
            p.move(to: CGPoint(x: c.x - ew * 0.6, y: by + side * eh * 0.04))
            p.addLine(to: CGPoint(x: c.x + ew * 0.6, y: by - side * eh * 0.04))
            stroke(p, lw)
        }
    }

    private func drawMouth(_ c: CGPoint, _ unit: CGFloat, _ lw: CGFloat) {
        let mw = unit * 0.16
        let p = UIBezierPath()
        switch mood {
        case .happy, .love:
            p.move(to: CGPoint(x: c.x - mw, y: c.y - unit * 0.02))
            p.addQuadCurve(to: CGPoint(x: c.x + mw, y: c.y - unit * 0.02), controlPoint: CGPoint(x: c.x, y: c.y + unit * 0.09))
            stroke(p, lw)
        case .scared:
            stroke(UIBezierPath(ovalIn: CGRect(x: c.x - mw * 0.35, y: c.y - unit * 0.05, width: mw * 0.7, height: unit * 0.11)), lw)
        case .searching:
            stroke(UIBezierPath(arcCenter: c, radius: unit * 0.025, startAngle: 0, endAngle: 2 * .pi, clockwise: true), lw)
        case .worried:
            p.move(to: CGPoint(x: c.x - mw, y: c.y))
            p.addCurve(to: CGPoint(x: c.x + mw / 2, y: c.y - unit * 0.01), controlPoint1: CGPoint(x: c.x - mw / 2, y: c.y - unit * 0.04),
                       controlPoint2: CGPoint(x: c.x, y: c.y + unit * 0.04))
            p.addLine(to: CGPoint(x: c.x + mw, y: c.y + unit * 0.02))
            stroke(p, lw)
        case .blind:
            p.move(to: CGPoint(x: c.x - mw * 0.6, y: c.y)); p.addLine(to: CGPoint(x: c.x + mw * 0.6, y: c.y))
            stroke(p, lw)
        case .focused, .idle:
            p.move(to: CGPoint(x: c.x - mw * 0.45, y: c.y)); p.addLine(to: CGPoint(x: c.x + mw * 0.45, y: c.y))
            stroke(p, lw)
        }
    }
}
