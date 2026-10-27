//
//  LidarView.swift
//  OpenBot
//
//  Polar plot of one LiDAR revolution. The Python counterpart is
//  `lidar.plan_view`; the Android one is `robot.customview.LidarView`.
//
//  The robot sits at the centre pointing up, so "forward" is up on screen
//  whichever way the phone is held. Returns are drawn as a connected outline at
//  their measured distance - the room's walls read as the shape they actually
//  are. Bins with no return are *gaps* rather than zeros, which is what
//  "nothing measured here" means; the `+inf` a ROS `/scan` publisher would emit
//  has no place on screen.
//
//  The mounting offset is applied at draw time (`forwardOffsetDeg`) rather than
//  by reordering bins, matching the firmware's `LIDAR_FORWARD_DEG` rule.
//

import UIKit

/// Polar plot of one LiDAR revolution.
final class LidarView: UIView {
    /// Newest scan to draw; nil draws the empty grid.
    var scan: LidarScan? {
        didSet { setNeedsDisplay() }
    }

    /// LiDAR mounting offset in degrees clockwise from the car's nose.
    var forwardOffsetDeg: Double = 0 {
        didSet { setNeedsDisplay() }
    }

    /// Outer ring distance in metres. Returns further out are clamped to the
    /// circle and still show their direction.
    var rangeMaxMeters: Double = 8.0 {
        didSet { setNeedsDisplay() }
    }

    /// Standoff distance, drawn as the AVOID_MARGIN ring. Hides itself at 0.
    var marginMeters: Double = 1.0 {
        didSet { setNeedsDisplay() }
    }

    var showMargin = true {
        didSet { setNeedsDisplay() }
    }

    /// Frozen scans are drawn grey and must read as frozen, not as live data.
    var isStale = false {
        didSet { setNeedsDisplay() }
    }

    var scanColor = UIColor(red: 1.0, green: 0.70, blue: 0.0, alpha: 1.0) {
        didSet { setNeedsDisplay() }
    }

    var staleColor = UIColor(white: 0.62, alpha: 1.0) {
        didSet { setNeedsDisplay() }
    }

    /// The car's 8 proximity sectors in metres, clockwise from the nose (`.infinity`
    /// = clear), drawn as wedges: red under 0.5 m, amber under 1 m, green beyond.
    /// Over Bluetooth these are the only LiDAR data. nil hides them.
    var sectorsMeters: [Double]? {
        didSet { setNeedsDisplay() }
    }

    private let ringColor = UIColor(white: 0.72, alpha: 1.0)
    private let marginColor = UIColor(red: 0.15, green: 0.78, blue: 1.0, alpha: 1.0)
    private let robotColor = UIColor(red: 0.29, green: 0.48, blue: 1.0, alpha: 1.0)

    override init(frame: CGRect) {
        super.init(frame: frame)
        commonInit()
    }

    required init?(coder: NSCoder) {
        super.init(coder: coder)
        commonInit()
    }

    private func commonInit() {
        backgroundColor = UIColor(red: 0.06, green: 0.09, blue: 0.13, alpha: 1.0)
        isOpaque = true
        contentMode = .redraw
    }

    override func draw(_ rect: CGRect) {
        guard let context = UIGraphicsGetCurrentContext() else { return }
        let side = min(bounds.width, bounds.height)
        let center = CGPoint(x: bounds.midX, y: bounds.midY)
        let radius = side * 0.46

        context.setFillColor(backgroundColor?.cgColor ?? UIColor.black.cgColor)
        context.fill(bounds)

        // Range rings at 1/4, 1/2 and full radius, so distance is readable at a glance.
        context.setStrokeColor(ringColor.cgColor)
        context.setLineWidth(1)
        for fraction in [0.25, 0.5, 1.0] {
            context.addArc(
                center: center, radius: radius * fraction,
                startAngle: 0, endAngle: .pi * 2, clockwise: false
            )
            context.strokePath()
        }

        if showMargin, marginMeters > 0, marginMeters < rangeMaxMeters {
            let marginRadius = radius * CGFloat(marginMeters / rangeMaxMeters)
            context.setStrokeColor(marginColor.cgColor)
            context.setLineWidth(2)
            context.addArc(
                center: center, radius: marginRadius,
                startAngle: 0, endAngle: .pi * 2, clockwise: false
            )
            context.strokePath()
        }

        // Nose and tail marks so the plot reads as a robot rather than a compass.
        context.setStrokeColor(robotColor.cgColor)
        context.setLineWidth(2)
        context.move(to: CGPoint(x: center.x, y: center.y - radius))
        context.addLine(to: CGPoint(x: center.x, y: center.y - radius * 0.88))
        context.strokePath()
        context.setStrokeColor(ringColor.cgColor)
        context.setLineWidth(1)
        context.move(to: CGPoint(x: center.x, y: center.y + radius))
        context.addLine(to: CGPoint(x: center.x, y: center.y + radius * 0.88))
        context.strokePath()

        drawSectors(context, center: center, radius: radius)

        guard let scan else {
            drawRobot(context, center: center, radius: radius)
            return
        }

        let points = LidarView.scanToPoints(
            scan: scan,
            rangeMaxMeters: rangeMaxMeters,
            forwardOffsetDeg: forwardOffsetDeg
        )
        let strokes = LidarView.splitStrokes(points)
        let color = isStale ? staleColor : scanColor
        context.setStrokeColor(color.cgColor)
        context.setFillColor(color.cgColor)
        context.setLineWidth(2)
        for stroke in strokes {
            guard let first = stroke.first else { continue }
            if stroke.count == 1 {
                let point = LidarView.toPixels(
                    center: center, radius: radius, x: first.x, y: first.y
                )
                context.fillEllipse(in: CGRect(x: point.x - 2, y: point.y - 2, width: 4, height: 4))
                continue
            }
            context.move(to: LidarView.toPixels(center: center, radius: radius, x: first.x, y: first.y))
            for point in stroke.dropFirst() {
                context.addLine(
                    to: LidarView.toPixels(center: center, radius: radius, x: point.x, y: point.y)
                )
            }
            context.strokePath()
        }

        drawRobot(context, center: center, radius: radius)
    }

    private func drawSectors(_ context: CGContext, center: CGPoint, radius: CGFloat) {
        guard let sectors = sectorsMeters, !sectors.isEmpty, rangeMaxMeters > 0 else { return }
        let span = 2 * Double.pi / Double(sectors.count)
        for (i, m) in sectors.enumerated() where m.isFinite && m > 0 {
            let color: UIColor = m < 0.5 ? .systemRed : m < 1.0 ? .systemOrange : .systemGreen
            let r = radius * CGFloat(min(1.0, m / rangeMaxMeters))
            // Sector i is centred i * span clockwise from the nose (up). With UIKit's
            // flipped y, increasing angles run clockwise on screen.
            let start = CGFloat(Double(i) * span - span / 2 - Double.pi / 2)
            let end = start + CGFloat(span)
            context.setFillColor(color.withAlphaComponent(isStale ? 0.10 : 0.22).cgColor)
            context.move(to: center)
            context.addArc(center: center, radius: r, startAngle: start, endAngle: end, clockwise: false)
            context.closePath()
            context.fillPath()
            context.setStrokeColor(color.withAlphaComponent(isStale ? 0.4 : 0.95).cgColor)
            context.setLineWidth(3)
            context.addArc(center: center, radius: r, startAngle: start + 0.03, endAngle: end - 0.03, clockwise: false)
            context.strokePath()
        }
    }

    private func drawRobot(_ context: CGContext, center: CGPoint, radius: CGFloat) {
        let size = radius * 0.12
        context.setFillColor(robotColor.cgColor)
        context.move(to: CGPoint(x: center.x, y: center.y - size))
        context.addLine(to: CGPoint(x: center.x + size * 0.7, y: center.y + size * 0.8))
        context.addLine(to: CGPoint(x: center.x - size * 0.7, y: center.y + size * 0.8))
        context.closePath()
        context.fillPath()
    }

    // MARK: - Geometry (pure, so it can be tested without a display)

    struct PlotPoint {
        /// Normalised image coordinates: origin at the centre, +x right, +y down.
        let x: Double
        let y: Double
        let bin: Int
    }

    /// Project a scan into normalised plot points.
    ///
    /// Bins with no return are omitted, so an unmeasured arc is a gap rather
    /// than a line dragged through the middle of the plot. Returns further than
    /// `rangeMaxMeters` are clamped to the outer circle and still show their
    /// direction.
    static func scanToPoints(
        scan: LidarScan, rangeMaxMeters: Double, forwardOffsetDeg: Double
    ) -> [PlotPoint] {
        guard rangeMaxMeters > 0 else { return [] }
        var out = [PlotPoint]()
        out.reserveCapacity(scan.binCount)
        for bin in 0..<scan.binCount {
            let mm = scan.rangeMm(bin)
            if mm == 0 { continue }
            let dist = Double(mm) / 1000.0
            let r = 0.5 * min(1.0, dist / rangeMaxMeters)
            // Vehicle angle is clockwise from the nose; screen y grows downward,
            // so subtracting 90 maps nose-up to -y and keeps clockwise correct.
            let angle = (scan.vehicleAngleDeg(bin, forwardOffsetDeg: forwardOffsetDeg) - 90.0)
                * .pi / 180.0
            out.append(PlotPoint(x: r * cos(angle), y: r * sin(angle), bin: bin))
        }
        return out
    }

    /// Group consecutive points into polylines.
    ///
    /// `points` must already be in bin order. A gap of more than one bin index
    /// means the scanner saw nothing there, so the pen lifts - that is how an
    /// unmeasured arc becomes a visible hole rather than a straight line across it.
    static func splitStrokes(_ points: [PlotPoint]) -> [[PlotPoint]] {
        var strokes = [[PlotPoint]]()
        var current = [PlotPoint]()
        var previousBin: Int?
        for point in points {
            if let previousBin, point.bin != previousBin + 1 {
                if !current.isEmpty { strokes.append(current) }
                current = []
            }
            current.append(point)
            previousBin = point.bin
        }
        if !current.isEmpty { strokes.append(current) }
        return strokes
    }

    private static func toPixels(center: CGPoint, radius: CGFloat, x: Double, y: Double) -> CGPoint {
        CGPoint(
            x: center.x + CGFloat(x) * 2 * radius,
            y: center.y + CGFloat(y) * 2 * radius
        )
    }

    /// One-line status for the UI, matching the Android app's wording.
    static func formatStatus(scan: LidarScan?, isFresh: Bool) -> String {
        guard let scan else { return "Waiting for scans" }
        let state = isFresh ? "live" : "stale"
        return "\(scan.binCount) bins · \(scan.validPoints) returns · \(state)"
    }
}

// MARK: - Object avoidance controls

/// The quick-disable switch as a navigation bar button: one tap flips it.
///
/// ArduPilot's "SAFETY FIRST!" point is that the switch must be in reach while
/// the vehicle is moving, not buried in settings, so every screen that drives
/// the car puts this in its bar. The title always says the current state.
final class AvoidanceToggleItem: UIBarButtonItem {
    private let bluetooth = bluetoothDataController.shared

    override init() {
        super.init()
        target = self
        action = #selector(toggle)
        refresh()
    }

    required init?(coder: NSCoder) {
        super.init(coder: coder)
        target = self
        action = #selector(toggle)
        refresh()
    }

    func refresh() {
        let on = bluetooth.driveGate.enabled
        title = L10n.string(on ? "avoid_on" : "avoid_off")
        tintColor = on ? .systemGreen : .systemRed
        accessibilityLabel = L10n.string("avoid_title")
    }

    @objc private func toggle() {
        var settings = bluetooth.driveGate.settings
        settings.enabled.toggle()
        bluetooth.updateObjectAvoidance(settings)
        refresh()
    }
}

/// Settings sheet for the three ArduPilot knobs: the quick-disable switch,
/// `AVOID_MARGIN` and `AVOID_BACKUP_SPD`. Changes apply and persist at once.
final class AvoidanceSettingsController: UIViewController {
    private let bluetooth = bluetoothDataController.shared
    private let enabledSwitch = UISwitch()
    private let marginSlider = UISlider()
    private let marginValue = UILabel()
    private let backupSlider = UISlider()
    private let backupValue = UILabel()

    /// Called after every change, so the presenting screen can refresh its own
    /// toggle and margin ring.
    var onChange: (() -> Void)?

    override func viewDidLoad() {
        super.viewDidLoad()
        view.backgroundColor = .systemBackground

        let title = UILabel()
        title.text = L10n.string("avoid_title")
        title.font = .systemFont(ofSize: 20, weight: .semibold)

        let note = UILabel()
        note.text = L10n.string("avoid_note")
        note.font = .systemFont(ofSize: 13)
        note.textColor = .secondaryLabel
        note.numberOfLines = 0

        let settings = bluetooth.driveGate.settings
        enabledSwitch.isOn = settings.enabled
        enabledSwitch.addTarget(self, action: #selector(changed), for: .valueChanged)

        marginSlider.minimumValue = Float(AvoidanceSettings.marginRange.lowerBound)
        marginSlider.maximumValue = Float(AvoidanceSettings.marginRange.upperBound)
        marginSlider.value = Float(settings.marginMeters)
        marginSlider.addTarget(self, action: #selector(changed), for: .valueChanged)

        backupSlider.minimumValue = Float(AvoidanceSettings.backupRange.lowerBound)
        backupSlider.maximumValue = Float(AvoidanceSettings.backupRange.upperBound)
        backupSlider.value = Float(settings.backupSpeed)
        backupSlider.addTarget(self, action: #selector(changed), for: .valueChanged)

        let stack = UIStackView(arrangedSubviews: [
            title,
            row(L10n.string("avoid_enabled"), enabledSwitch),
            row(L10n.string("avoid_margin"), marginValue),
            marginSlider,
            row(L10n.string("avoid_backup"), backupValue),
            backupSlider,
            note,
        ])
        stack.axis = .vertical
        stack.spacing = 14
        stack.translatesAutoresizingMaskIntoConstraints = false
        view.addSubview(stack)
        NSLayoutConstraint.activate([
            stack.leadingAnchor.constraint(equalTo: view.safeAreaLayoutGuide.leadingAnchor, constant: 20),
            stack.trailingAnchor.constraint(equalTo: view.safeAreaLayoutGuide.trailingAnchor, constant: -20),
            stack.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 24),
        ])
        refreshLabels()

        if let sheet = sheetPresentationController {
            sheet.detents = [.medium(), .large()]
            sheet.prefersGrabberVisible = true
        }
    }

    private func row(_ text: String, _ trailing: UIView) -> UIStackView {
        let label = UILabel()
        label.text = text
        label.font = .systemFont(ofSize: 16)
        let row = UIStackView(arrangedSubviews: [label, trailing])
        row.axis = .horizontal
        row.distribution = .equalSpacing
        row.alignment = .center
        return row
    }

    @objc private func changed() {
        // Snap to the steps the Android sliders use: 0.1 m and 0.05.
        let margin = (Double(marginSlider.value) * 10).rounded() / 10
        let backup = (Double(backupSlider.value) * 20).rounded() / 20
        bluetooth.updateObjectAvoidance(AvoidanceSettings(
            enabled: enabledSwitch.isOn,
            marginMeters: margin,
            backupSpeed: backup
        ))
        refreshLabels()
        onChange?()
    }

    private func refreshLabels() {
        let settings = bluetooth.driveGate.settings
        marginValue.text = String(format: "%.1f m", settings.marginMeters)
        backupValue.text = settings.backupSpeed <= 0
            ? L10n.string("avoid_backup_off")
            : String(format: "%.0f%%", settings.backupSpeed * 100)
    }
}


// MARK: - One LiDAR listener for everyone

/// The car's LiDAR datagrams arrive on one UDP port, so the avoidance map and the
/// Sensor link server share one listener, started by the first user and stopped
/// by the last.
enum SharedLidar {
    static let scanner = LidarScanner()
    private static var users = 0
    private static let lock = NSLock()

    static func acquire() {
        lock.lock(); defer { lock.unlock() }
        users += 1
        if users == 1 { scanner.start() }
    }

    static func release() {
        lock.lock(); defer { lock.unlock() }
        guard users > 0 else { return }
        users -= 1
        if users == 0 { scanner.stop() }
    }
}

// MARK: - Object avoidance with its LiDAR map

/// The object-avoidance switch for a screen's information panel. While avoidance
/// is on, a LiDAR map shows underneath: the car in the middle facing up, the scan
/// outline, the car's 8 proximity sectors as coloured wedges and the standoff
/// ring. Mirrors object_avoidance_control.xml on Android.
///
/// The screen calls `start()` when it appears and `stop()` when it goes away.
final class AvoidanceLidarCard: UIView {
    /// Called after the user flips the switch (e.g. to refresh a toolbar button).
    var onChange: (() -> Void)?

    private let bluetooth = bluetoothDataController.shared
    private let title = UILabel()
    private let toggle = UISwitch()
    private let statusLabel = UILabel()
    private let map = LidarView()
    private let info = UILabel()
    private var timer: Timer?
    private var started = false

    /// `mapHeight`: 180 for a panel, smaller for an overlay card.
    init(mapHeight: CGFloat = 180) {
        super.init(frame: .zero)
        title.text = L10n.string("avoid_title")
        title.font = .systemFont(ofSize: 15, weight: .semibold)
        title.textColor = .label
        toggle.addTarget(self, action: #selector(toggled), for: .valueChanged)
        toggle.accessibilityLabel = L10n.string("avoid_title")
        statusLabel.font = .systemFont(ofSize: 12)
        statusLabel.textColor = .secondaryLabel
        statusLabel.numberOfLines = 0
        map.rangeMaxMeters = 4
        map.layer.cornerRadius = 10
        map.clipsToBounds = true
        map.isAccessibilityElement = true
        map.accessibilityLabel = L10n.string("avoid_lidar_map")
        map.heightAnchor.constraint(equalToConstant: mapHeight).isActive = true
        info.font = .systemFont(ofSize: 12)
        info.textColor = .secondaryLabel
        info.textAlignment = .center
        info.numberOfLines = 0

        let header = UIStackView(arrangedSubviews: [title, toggle])
        header.axis = .horizontal
        header.alignment = .center
        header.spacing = 8
        let stack = UIStackView(arrangedSubviews: [header, statusLabel, map, info])
        stack.axis = .vertical
        stack.spacing = 6
        stack.translatesAutoresizingMaskIntoConstraints = false
        addSubview(stack)
        NSLayoutConstraint.activate([
            stack.topAnchor.constraint(equalTo: topAnchor),
            stack.leadingAnchor.constraint(equalTo: leadingAnchor),
            stack.trailingAnchor.constraint(equalTo: trailingAnchor),
            stack.bottomAnchor.constraint(equalTo: bottomAnchor),
        ])
        refresh()
    }

    required init?(coder: NSCoder) {
        fatalError("init(coder:) is not used")
    }

    /// A floating card for camera screens (Object Tracking, Autopilot): top-left,
    /// under the navigation bar, clear of the STOP button on the right.
    @discardableResult
    static func addOverlay(to view: UIView) -> AvoidanceLidarCard {
        let card = AvoidanceLidarCard(mapHeight: 150)
        let box = UIView()
        box.backgroundColor = UIColor.systemBackground.withAlphaComponent(0.9)
        box.layer.cornerRadius = 12
        box.translatesAutoresizingMaskIntoConstraints = false
        card.translatesAutoresizingMaskIntoConstraints = false
        box.addSubview(card)
        view.addSubview(box)
        NSLayoutConstraint.activate([
            card.topAnchor.constraint(equalTo: box.topAnchor, constant: 8),
            card.leadingAnchor.constraint(equalTo: box.leadingAnchor, constant: 10),
            card.trailingAnchor.constraint(equalTo: box.trailingAnchor, constant: -10),
            card.bottomAnchor.constraint(equalTo: box.bottomAnchor, constant: -8),
            box.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 8),
            box.leadingAnchor.constraint(equalTo: view.safeAreaLayoutGuide.leadingAnchor, constant: 8),
            box.widthAnchor.constraint(equalToConstant: 210),
        ])
        return card
    }

    func start() {
        guard !started else { return }
        started = true
        SharedLidar.acquire()
        timer = Timer.scheduledTimer(withTimeInterval: 0.1, repeats: true) { [weak self] _ in self?.refresh() }
        refresh()
    }

    func stop() {
        guard started else { return }
        started = false
        timer?.invalidate()
        timer = nil
        SharedLidar.release()
    }

    deinit {
        timer?.invalidate()
        if started { SharedLidar.release() }
    }

    @objc private func toggled() {
        var settings = bluetooth.driveGate.settings
        settings.enabled = toggle.isOn
        bluetooth.updateObjectAvoidance(settings)
        refresh()
        onChange?()
    }

    private func refresh() {
        let settings = bluetooth.driveGate.settings
        if !toggle.isTracking { toggle.isOn = settings.enabled }
        statusLabel.text = String(format: "%@ · %.1f m", L10n.string(settings.enabled ? "avoid_on" : "avoid_off"),
                                  settings.marginMeters)
        let on = settings.enabled
        if map.isHidden == on { map.isHidden = !on; info.isHidden = !on }
        guard on else { return }

        let scanner = SharedLidar.scanner
        let scan = scanner.latestScan
        map.scan = scan
        map.isStale = scanner.isStale
        map.marginMeters = settings.marginMeters
        map.showMargin = true
        let proximity = bluetooth.driveGate.proximity
        let sectorsFresh = proximity.hasEverReported() && proximity.ageMs() < 1000
        map.sectorsMeters = sectorsFresh ? proximity.sectors() : nil
        if let scan {
            let key = scanner.isStale ? "lidar_scan_stale" : "lidar_scan_live"
            info.text = String(format: L10n.string(key), Int32(scan.binCount), Int32(scan.validPoints))
        } else {
            info.text = L10n.string(sectorsFresh ? "lidar_sectors_only" : "lidar_waiting")
        }
    }
}
