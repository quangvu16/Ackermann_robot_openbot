//
//  CarDashboard.swift
//  OpenBot
//
//  Task-first home for the ESP32-S3 car, mirroring the Android redesign:
//
//  - EmergencyStop: the app-wide STOP latch. While latched every drive
//    command ("c...") is written as "c0,0", whatever screen or mode sent it.
//  - StopButtonOverlay: a big round STOP / RESUME button on every screen
//    while the car is connected.
//  - CarDashboardView: connection card, status chips (LiDAR, RC mode,
//    avoidance), three task tiles (Drive, GPS Mission, Car setup) and
//    "More features" for the full OpenBot grid.
//

import UIKit

extension Notification.Name {
    static let emergencyStopChanged = Notification.Name("emergencyStopChanged")
}

// MARK: - STOP latch

/// The STOP latch behind the big red button. Checked in
/// `bluetoothDataController.writeWire`, the one place bytes leave for the car.
///
/// It only governs commands from the app: with the RC switch in MANUAL the
/// transmitter drives the car directly, and the switch's HOLD is the stop there.
final class EmergencyStop {
    static let shared = EmergencyStop()

    private(set) var isLatched = false

    func latch() { set(true) }
    func release() { set(false) }
    func toggle() { set(!isLatched) }

    /// What to actually send: drive commands become "c0,0" while latched;
    /// everything else (lights, heartbeat, queries) passes through.
    func filter(_ payload: String) -> String {
        (isLatched && payload.hasPrefix("c")) ? "c0,0\n" : payload
    }

    private func set(_ value: Bool) {
        guard value != isLatched else { return }
        isLatched = value
        NotificationCenter.default.post(name: .emergencyStopChanged, object: nil)
    }
}

private func appKeyWindow() -> UIWindow? {
    let windows = UIApplication.shared.connectedScenes
        .compactMap { $0 as? UIWindowScene }
        .flatMap { $0.windows }
    return windows.first(where: { $0.isKeyWindow }) ?? windows.first
}

// MARK: - Floating STOP button

/// The big round STOP / RESUME button, in the same place on every screen.
final class StopButtonOverlay {
    static let shared = StopButtonOverlay()

    private static let red = UIColor(red: 0xD3 / 255, green: 0x2F / 255, blue: 0x2F / 255, alpha: 1)
    private static let green = UIColor(red: 0x2E / 255, green: 0x7D / 255, blue: 0x32 / 255, alpha: 1)
    private static let size: CGFloat = 88

    private var button: UIButton?
    private var timer: Timer?

    /// Starts watching the connection; safe to call more than once.
    func start() {
        guard timer == nil else { return }
        timer = Timer.scheduledTimer(withTimeInterval: 0.5, repeats: true) { [weak self] _ in self?.refresh() }
        NotificationCenter.default.addObserver(self, selector: #selector(refreshSoon), name: .emergencyStopChanged, object: nil)
        refresh()
    }

    @objc private func refreshSoon() {
        DispatchQueue.main.async { self.refresh() }
    }

    /// STOP while connected, RESUME while stopped, nothing otherwise.
    func refresh() {
        let latched = EmergencyStop.shared.isLatched
        guard latched || isBluetoothConnected, let window = appKeyWindow() else {
            button?.removeFromSuperview()
            button = nil
            return
        }
        let stopButton = button ?? makeButton()
        stopButton.backgroundColor = latched ? Self.green : Self.red
        let title = L10n.string(latched ? "stop_button_resume" : "stop_button_stop")
        stopButton.setTitle(title, for: .normal)
        stopButton.accessibilityLabel = title
        // Home tabs: bottom-right, above the tab bar (49 pt) and the home indicator.
        // Driving screens (pushed from Home): top-right under the navigation bar, clear of
        // their bottom control panels.
        let x = window.bounds.width - Self.size - 16 - window.safeAreaInsets.right
        let y = Self.onDrivingScreen(window)
            ? window.safeAreaInsets.top + 44 + 12
            : window.bounds.height - Self.size - window.safeAreaInsets.bottom - 49 - 16
        stopButton.frame = CGRect(x: x, y: y, width: Self.size, height: Self.size)
        if stopButton.superview !== window { window.addSubview(stopButton) }
        // Below the RC face: in MANUAL the transmitter drives and this STOP does not apply.
        if let face = RcModeFaceOverlay.shared.faceView, face.superview === window {
            window.insertSubview(stopButton, belowSubview: face)
        } else {
            window.bringSubviewToFront(stopButton)
        }
        button = stopButton
    }

    /// True when a screen has been pushed on top of a home tab (Free Roam, GPS Mission, ...).
    private static func onDrivingScreen(_ window: UIWindow) -> Bool {
        var top = window.rootViewController
        while let presented = top?.presentedViewController { top = presented }
        if let tabs = top as? UITabBarController { top = tabs.selectedViewController }
        if let nav = top as? UINavigationController { return nav.viewControllers.count > 1 }
        return false
    }

    private func makeButton() -> UIButton {
        let stopButton = UIButton(type: .custom)
        stopButton.layer.cornerRadius = Self.size / 2
        stopButton.layer.borderColor = UIColor.white.cgColor
        stopButton.layer.borderWidth = 4
        stopButton.layer.shadowColor = UIColor.black.cgColor
        stopButton.layer.shadowOpacity = 0.35
        stopButton.layer.shadowRadius = 6
        stopButton.layer.shadowOffset = CGSize(width: 0, height: 3)
        stopButton.titleLabel?.font = UIFont.systemFont(ofSize: 20, weight: .heavy)
        stopButton.titleLabel?.adjustsFontSizeToFitWidth = true
        stopButton.titleLabel?.minimumScaleFactor = 0.5
        stopButton.setTitleColor(.white, for: .normal)
        stopButton.addTarget(self, action: #selector(tapped), for: .touchUpInside)
        return stopButton
    }

    @objc private func tapped() {
        UIImpactFeedbackGenerator(style: .heavy).impactOccurred()
        EmergencyStop.shared.toggle()
    }
}

// MARK: - Car dashboard

/// The top of the home screen: is the car ready, and what do you want to do?
final class CarDashboardView: UIView {
    var onConnect: (() -> Void)?
    var onDrive: (() -> Void)?
    var onMission: (() -> Void)?
    var onSetup: (() -> Void)?
    var onToggleMore: (() -> Void)?

    private static let green = UIColor(red: 0x2E / 255, green: 0x7D / 255, blue: 0x32 / 255, alpha: 1)
    private static let amber = UIColor(red: 0xF9 / 255, green: 0xA8 / 255, blue: 0x25 / 255, alpha: 1)
    private static let red = UIColor(red: 0xC6 / 255, green: 0x28 / 255, blue: 0x28 / 255, alpha: 1)
    private static let grey = UIColor.systemGray
    private static let blue = UIColor(red: 0x4B / 255, green: 0x7B / 255, blue: 1, alpha: 1)

    private let dot = UIView()
    private let titleLabel = UILabel()
    private let detailLabel = UILabel()
    private let connectButton = UIButton(type: .system)
    private let lidarChip = UILabel()
    private let rcChip = UILabel()
    private let avoidChip = UILabel()
    private let moreButton = UIButton(type: .system)

    override init(frame: CGRect) {
        super.init(frame: frame)
        build()
        refresh()
    }

    required init?(coder: NSCoder) {
        fatalError("init(coder:) is not used")
    }

    // MARK: layout

    private func build() {
        let stack = UIStackView()
        stack.axis = .vertical
        stack.spacing = 10
        stack.translatesAutoresizingMaskIntoConstraints = false
        addSubview(stack)
        NSLayoutConstraint.activate([
            stack.topAnchor.constraint(equalTo: topAnchor, constant: 12),
            stack.leadingAnchor.constraint(equalTo: leadingAnchor, constant: 16),
            stack.trailingAnchor.constraint(equalTo: trailingAnchor, constant: -16),
            stack.bottomAnchor.constraint(equalTo: bottomAnchor, constant: -4),
        ])

        // 1. Is the car connected?
        let card = UIView()
        card.backgroundColor = .secondarySystemBackground
        card.layer.cornerRadius = 16
        let cardStack = UIStackView()
        cardStack.axis = .vertical
        cardStack.spacing = 8
        cardStack.translatesAutoresizingMaskIntoConstraints = false
        card.addSubview(cardStack)
        NSLayoutConstraint.activate([
            cardStack.topAnchor.constraint(equalTo: card.topAnchor, constant: 16),
            cardStack.leadingAnchor.constraint(equalTo: card.leadingAnchor, constant: 16),
            cardStack.trailingAnchor.constraint(equalTo: card.trailingAnchor, constant: -16),
            cardStack.bottomAnchor.constraint(equalTo: card.bottomAnchor, constant: -16),
        ])

        dot.layer.cornerRadius = 7
        dot.widthAnchor.constraint(equalToConstant: 14).isActive = true
        dot.heightAnchor.constraint(equalToConstant: 14).isActive = true
        titleLabel.font = UIFont.systemFont(ofSize: 20, weight: .bold)
        titleLabel.textColor = .label
        titleLabel.numberOfLines = 0
        let titleRow = UIStackView(arrangedSubviews: [dot, titleLabel])
        titleRow.axis = .horizontal
        titleRow.spacing = 10
        titleRow.alignment = .center
        cardStack.addArrangedSubview(titleRow)

        detailLabel.font = UIFont.systemFont(ofSize: 15)
        detailLabel.textColor = .secondaryLabel
        detailLabel.numberOfLines = 0
        cardStack.addArrangedSubview(detailLabel)

        connectButton.setTitle(L10n.string("dash_connect"), for: .normal)
        connectButton.titleLabel?.font = UIFont.systemFont(ofSize: 17, weight: .semibold)
        connectButton.setTitleColor(.white, for: .normal)
        connectButton.backgroundColor = Self.blue
        connectButton.layer.cornerRadius = 12
        connectButton.heightAnchor.constraint(equalToConstant: 52).isActive = true
        connectButton.addTarget(self, action: #selector(connectTapped), for: .touchUpInside)
        cardStack.addArrangedSubview(connectButton)

        // 2. Is the car ready? One glance at the sensors that matter.
        for chip in [lidarChip, rcChip, avoidChip] {
            chip.font = UIFont.systemFont(ofSize: 13, weight: .semibold)
            chip.textAlignment = .center
            chip.layer.cornerRadius = 14
            chip.layer.masksToBounds = true
            chip.adjustsFontSizeToFitWidth = true
            chip.minimumScaleFactor = 0.7
            chip.heightAnchor.constraint(equalToConstant: 28).isActive = true
        }
        avoidChip.isUserInteractionEnabled = true
        avoidChip.addGestureRecognizer(UITapGestureRecognizer(target: self, action: #selector(avoidTapped)))
        avoidChip.accessibilityTraits = .button
        let chips = UIStackView(arrangedSubviews: [lidarChip, rcChip, avoidChip])
        chips.axis = .horizontal
        chips.spacing = 6
        chips.distribution = .fillEqually
        cardStack.addArrangedSubview(chips)
        stack.addArrangedSubview(card)

        // 3. What do you want to do?
        let header = UILabel()
        header.text = L10n.string("dash_tasks_header")
        header.font = UIFont.systemFont(ofSize: 17, weight: .bold)
        header.textColor = .label
        stack.setCustomSpacing(16, after: card)
        stack.addArrangedSubview(header)
        stack.addArrangedSubview(makeTile(icon: Images.freeRoam, color: Colors.freeRoamColor,
                                          title: "dash_drive_title", detail: "dash_drive_desc", action: #selector(driveTapped)))
        stack.addArrangedSubview(makeTile(icon: Images.gpsMissionIcon, color: Colors.gpsMissionColor,
                                          title: "dash_mission_title", detail: "dash_mission_desc", action: #selector(missionTapped)))
        stack.addArrangedSubview(makeTile(icon: Images.robotInfoIcon, color: Self.blue,
                                          title: "dash_setup_title", detail: "dash_setup_desc", action: #selector(setupTapped)))

        // 4. Everything else, out of the way until asked for.
        moreButton.titleLabel?.font = UIFont.systemFont(ofSize: 17, weight: .semibold)
        moreButton.setTitleColor(Self.blue, for: .normal)
        moreButton.heightAnchor.constraint(equalToConstant: 48).isActive = true
        moreButton.addTarget(self, action: #selector(moreTapped), for: .touchUpInside)
        stack.addArrangedSubview(moreButton)
        setMoreOpen(false)
    }

    private func makeTile(icon: UIImage?, color: UIColor, title: String, detail: String, action: Selector) -> UIView {
        let tile = UIControl()
        tile.backgroundColor = .secondarySystemBackground
        tile.layer.cornerRadius = 16
        tile.heightAnchor.constraint(greaterThanOrEqualToConstant: 84).isActive = true
        tile.addTarget(self, action: action, for: .touchUpInside)
        tile.accessibilityLabel = L10n.string(title)
        tile.accessibilityHint = L10n.string(detail)
        tile.accessibilityTraits = .button
        tile.isAccessibilityElement = true

        let iconBox = UIView()
        iconBox.backgroundColor = color
        iconBox.layer.cornerRadius = 14
        iconBox.isUserInteractionEnabled = false
        let iconView = UIImageView(image: icon?.withRenderingMode(.alwaysTemplate))
        iconView.tintColor = .white
        iconView.contentMode = .scaleAspectFit
        iconView.translatesAutoresizingMaskIntoConstraints = false
        iconBox.addSubview(iconView)
        iconBox.translatesAutoresizingMaskIntoConstraints = false

        let titleText = UILabel()
        titleText.text = L10n.string(title)
        titleText.font = UIFont.systemFont(ofSize: 19, weight: .bold)
        titleText.textColor = .label
        let detailText = UILabel()
        detailText.text = L10n.string(detail)
        detailText.font = UIFont.systemFont(ofSize: 14)
        detailText.textColor = .secondaryLabel
        detailText.numberOfLines = 0
        let texts = UIStackView(arrangedSubviews: [titleText, detailText])
        texts.axis = .vertical
        texts.spacing = 2
        texts.isUserInteractionEnabled = false
        texts.translatesAutoresizingMaskIntoConstraints = false

        tile.addSubview(iconBox)
        tile.addSubview(texts)
        NSLayoutConstraint.activate([
            iconBox.leadingAnchor.constraint(equalTo: tile.leadingAnchor, constant: 16),
            iconBox.centerYAnchor.constraint(equalTo: tile.centerYAnchor),
            iconBox.widthAnchor.constraint(equalToConstant: 52),
            iconBox.heightAnchor.constraint(equalToConstant: 52),
            iconView.centerXAnchor.constraint(equalTo: iconBox.centerXAnchor),
            iconView.centerYAnchor.constraint(equalTo: iconBox.centerYAnchor),
            iconView.widthAnchor.constraint(equalToConstant: 28),
            iconView.heightAnchor.constraint(equalToConstant: 28),
            texts.leadingAnchor.constraint(equalTo: iconBox.trailingAnchor, constant: 16),
            texts.trailingAnchor.constraint(equalTo: tile.trailingAnchor, constant: -16),
            texts.topAnchor.constraint(greaterThanOrEqualTo: tile.topAnchor, constant: 14),
            texts.bottomAnchor.constraint(lessThanOrEqualTo: tile.bottomAnchor, constant: -14),
            texts.centerYAnchor.constraint(equalTo: tile.centerYAnchor),
        ])
        return tile
    }

    // MARK: state

    func setMoreOpen(_ open: Bool) {
        moreButton.setTitle(L10n.string(open ? "dash_more_hide" : "dash_more_show"), for: .normal)
    }

    /// Connection card and chips, from the live state.
    func refresh() {
        let connected = isBluetoothConnected
        let stopped = EmergencyStop.shared.isLatched
        dot.backgroundColor = stopped ? Self.red : connected ? Self.green : Self.grey
        titleLabel.text = L10n.string(connected ? "dash_title_connected_ble" : "dash_title_disconnected")
        detailLabel.text = L10n.string(connected ? (stopped ? "dash_detail_stopped" : "dash_detail_connected") : "dash_detail_ble")
        connectButton.isHidden = connected

        switch bluetoothDataController.shared.driveGate.proximity.dataState() {
        case "fresh": setChip(lidarChip, "dash_chip_lidar_live", Self.green)
        case "stale": setChip(lidarChip, "dash_chip_lidar_stale", Self.red)
        default: setChip(lidarChip, "dash_chip_lidar_none", Self.grey)
        }

        switch RcModeFaceOverlay.shared.currentMode {
        case .none: setChip(rcChip, "dash_rc_none", Self.grey)
        case .some(.manual): setChip(rcChip, "dash_rc_manual", Self.amber)
        case .some(.auto): setChip(rcChip, "dash_rc_auto", Self.green)
        case .some(.hold): setChip(rcChip, "dash_rc_hold", Self.amber)
        case .some(.lost): setChip(rcChip, "dash_rc_lost", Self.red)
        }

        let avoid = AvoidanceSettings.load().enabled
        setChip(avoidChip, avoid ? "dash_chip_avoid_on" : "dash_chip_avoid_off", avoid ? Self.green : Self.amber)
    }

    private func setChip(_ chip: UILabel, _ key: String, _ color: UIColor) {
        chip.text = L10n.string(key)
        chip.backgroundColor = color
        chip.textColor = color == Self.amber ? UIColor(white: 0.16, alpha: 1) : .white
    }

    // MARK: actions

    @objc private func connectTapped() { onConnect?() }
    @objc private func driveTapped() { onDrive?() }
    @objc private func missionTapped() { onMission?() }
    @objc private func setupTapped() { onSetup?() }
    @objc private func moreTapped() { onToggleMore?() }

    @objc private func avoidTapped() {
        var settings = AvoidanceSettings.load()
        settings.enabled.toggle()
        bluetoothDataController.shared.updateObjectAvoidance(settings)
        refresh()
    }
}
