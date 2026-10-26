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
