//
// Created by Nitish Yadav on 22/12/22.
// Redesigned for the ESP32-S3 Ackermann car (firmware/openbot-esp32s3): it shows only what that car
// has - ESC throttle, steering servo, LD14 LiDAR (obstacle distance, 8 proximity sectors, scans),
// turn indicators, front/rear lights, status LEDs and an RC receiver. There is no battery sensor,
// sonar, bumper or wheel encoder on this car, so none are shown. Mirrors the Android
// RobotInfoFragment / fragment_robot_info.xml.
//

import Foundation
import UIKit

class RobotInfoFrame: UIViewController {

    let bluetooth = bluetoothDataController.shared
    var robotInfo: String = ""
    @IBOutlet weak var openBotIcon: UIImageView!

    private let scroll = UIScrollView()
    private let stack = UIStackView()
    private let robotName = UILabel()
    private let driveInfo = UILabel()
    private let iconView = UIImageView()
    private var bluetoothIconButton = UIButton(type: .system)
    private var features: [String: (Checkbox, UILabel)] = [:]
    private let obstacleValue = UILabel()
    private let rcModeValue = UILabel()
    private let linkValue = UILabel()
    private var indicatorsRow = UIStackView()
    private var lightsRow = UIStackView()
    private let lightSlider = UISlider()
    /// Full plan view of the LiDAR revolution, with a status line under it.
    var lidarView = LidarView()
    var lidarStatusLabel = UILabel()
    var lidarScanner = LidarScanner()
    var lidarTimer: Timer?
    private var rcTimer: Timer?

    override func viewDidLoad() {
        super.viewDidLoad()
        view.backgroundColor = .systemBackground
        openBotIcon?.isHidden = true // the storyboard icon is replaced by iconView in the stack
        setupNavigationBarItem()
        createUI()
        NotificationCenter.default.addObserver(self, selector: #selector(updateConnect), name: .bluetoothConnected, object: nil)
        NotificationCenter.default.addObserver(self, selector: #selector(updateConnect), name: .bluetoothDisconnected, object: nil)
        NotificationCenter.default.addObserver(self, selector: #selector(updateScreen), name: .updateLabel, object: nil)
        bluetooth.sendData(payload: "f\n")
        updateRobotInfo()
    }

    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        updateRobotInfo()
        if !isBluetoothConnected { updateScreenOnBluetoothDisconnection() }
        startLidar()
        rcTimer?.invalidate()
        rcTimer = Timer.scheduledTimer(withTimeInterval: 0.5, repeats: true) { [weak self] _ in self?.showRcMode() }
        showRcMode()
    }

    override func viewDidDisappear(_ animated: Bool) {
        super.viewDidDisappear(animated)
        stopLidar()
        rcTimer?.invalidate()
        rcTimer = nil
    }

    // MARK: Layout (a scrolling stack: works in portrait and landscape)

    private func createUI() {
        scroll.translatesAutoresizingMaskIntoConstraints = false
        view.addSubview(scroll)
        stack.axis = .vertical
        stack.spacing = 6
        stack.translatesAutoresizingMaskIntoConstraints = false
        scroll.addSubview(stack)
        NSLayoutConstraint.activate([
            scroll.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor),
            scroll.bottomAnchor.constraint(equalTo: view.bottomAnchor),
            scroll.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            scroll.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            stack.topAnchor.constraint(equalTo: scroll.contentLayoutGuide.topAnchor, constant: 12),
            stack.bottomAnchor.constraint(equalTo: scroll.contentLayoutGuide.bottomAnchor, constant: -24),
            stack.leadingAnchor.constraint(equalTo: scroll.frameLayoutGuide.leadingAnchor, constant: 16),
            stack.trailingAnchor.constraint(equalTo: scroll.frameLayoutGuide.trailingAnchor, constant: -16),
        ])

        // Header: robot type, refresh, Bluetooth.
        let typeLabel = UILabel()
        typeLabel.text = Strings.robotTypeHeading
        typeLabel.font = .boldSystemFont(ofSize: 14)
        robotName.text = "N/A"
        robotName.font = .systemFont(ofSize: 14)
        let refresh = UIButton(type: .system)
        refresh.setImage(UIImage(named: "refresh") ?? UIImage(systemName: "arrow.clockwise"), for: .normal)
        refresh.addTarget(self, action: #selector(refreshTapped), for: .touchUpInside)
        bluetoothIconButton.setImage(isBluetoothConnected ? Images.bluetoothConnected_v2 : Images.bluetoothDisconnected_v2, for: .normal)
        bluetoothIconButton.addTarget(self, action: #selector(openBluetoothSettings(_:)), for: .touchUpInside)
        let header = UIStackView(arrangedSubviews: [typeLabel, robotName, UIView(), refresh, bluetoothIconButton])
        header.spacing = 8
        header.alignment = .center
        stack.addArrangedSubview(header)

        iconView.contentMode = .scaleAspectFit
        iconView.image = UIImage(named: "openBotLogo")
        iconView.heightAnchor.constraint(equalToConstant: 120).isActive = true
        stack.addArrangedSubview(iconView)
        driveInfo.text = L10n.string("setup_drive_unknown")
        driveInfo.font = .systemFont(ofSize: 13)
        driveInfo.textAlignment = .center
        driveInfo.numberOfLines = 0
        stack.addArrangedSubview(driveInfo)

        // What the car reports in its 'f' reply (plus the RC receiver once 'm' arrives).
        stack.addArrangedSubview(heading("setup_features"))
        let keys: [(String, String)] = [
            ("s", "setup_feat_obstacle"), ("p", "setup_feat_proximity"),
            ("x", "setup_feat_scans"), ("rc", "setup_feat_rc"),
            ("i", "indicatorText"), ("ls", "setup_feat_status_leds"),
            ("lf", "setup_feat_front_lights"), ("lb", "setup_feat_rear_lights"),
        ]
        for pair in stride(from: 0, to: keys.count, by: 2) {
            let row = UIStackView()
            row.distribution = .fillEqually
            row.spacing = 8
            for k in keys[pair..<min(pair + 2, keys.count)] {
                let box = Checkbox(frame: CGRect(x: 0, y: 0, width: 22, height: 22))
                let dark = traitCollection.userInterfaceStyle == .dark
                box.checkedBorderColor = dark ? .white : .black
                box.uncheckedBorderColor = dark ? .white : .black
                box.checkmarkColor = dark ? .white : .black
                box.checkmarkStyle = .tick
                box.isEnabled = false
                box.widthAnchor.constraint(equalToConstant: 22).isActive = true
                box.heightAnchor.constraint(equalToConstant: 22).isActive = true
                let label = UILabel()
                label.text = L10n.string(k.1)
                label.font = .systemFont(ofSize: 13)
                label.numberOfLines = 0
                let cell = UIStackView(arrangedSubviews: [box, label])
                cell.spacing = 6
                cell.alignment = .center
                row.addArrangedSubview(cell)
                features[k.0] = (box, label)
            }
            stack.addArrangedSubview(row)
        }

        // Live readings.
        stack.addArrangedSubview(heading("readings"))
        stack.addArrangedSubview(reading("setup_obstacle_ahead", obstacleValue))
        stack.addArrangedSubview(reading("setup_rc_mode", rcModeValue))
        stack.addArrangedSubview(reading("setup_link", linkValue))

        // Tests.
        stack.addArrangedSubview(heading("sendCommand"))
        stack.addArrangedSubview(buttonRow(Strings.motors, [
            (Strings.forward, #selector(forwardButton(_:))), (Strings.backward, #selector(backwardButton(_:))),
            (Strings.stopLabel, #selector(stopButton(_:))),
        ]))
        stack.addArrangedSubview(buttonRow(L10n.string("setup_steering"), [
            (L10n.string("setup_left"), #selector(steerLeft)), (L10n.string("setup_centre"), #selector(steerCentre)),
            (L10n.string("setup_right"), #selector(steerRight)),
        ]))
        indicatorsRow = buttonRow(L10n.string("indicatorText"), [
            (L10n.string("setup_left"), #selector(indicatorLeft)), (L10n.string("setup_off"), #selector(indicatorOff)),
            (L10n.string("setup_right"), #selector(indicatorRight)),
        ])
        stack.addArrangedSubview(indicatorsRow)
        let lightLabel = UILabel()
        lightLabel.text = Strings.lightLabel
        lightLabel.font = .systemFont(ofSize: 13)
        lightLabel.widthAnchor.constraint(equalToConstant: 84).isActive = true
        lightSlider.thumbTintColor = Colors.title
        lightSlider.minimumTrackTintColor = Colors.title
        lightSlider.addTarget(self, action: #selector(lightValueChange(_:)), for: .valueChanged)
        lightsRow = UIStackView(arrangedSubviews: [lightLabel, lightSlider])
        lightsRow.spacing = 8
        stack.addArrangedSubview(lightsRow)
        let note = UILabel()
        note.text = L10n.string("setup_test_note")
        note.font = .systemFont(ofSize: 12)
        note.textColor = .secondaryLabel
        note.numberOfLines = 0
        stack.addArrangedSubview(note)

        // LiDAR plan view.
        stack.addArrangedSubview(heading("setup_lidar"))
        lidarView.layer.cornerRadius = 8
        lidarView.clipsToBounds = true
        lidarView.marginMeters = bluetooth.driveGate.marginMeters
        lidarView.heightAnchor.constraint(equalTo: lidarView.widthAnchor).isActive = true
        let lidarBox = UIStackView(arrangedSubviews: [lidarView])
        lidarBox.alignment = .center
        lidarView.widthAnchor.constraint(equalToConstant: 280).isActive = true
        stack.addArrangedSubview(lidarBox)
        lidarStatusLabel.font = .systemFont(ofSize: 12)
        lidarStatusLabel.textAlignment = .center
        lidarStatusLabel.numberOfLines = 0
        stack.addArrangedSubview(lidarStatusLabel)
        obstacleValue.text = "N/A"
        rcModeValue.text = "N/A"
        showLink()
        showFeatures()
    }

    private func heading(_ key: String) -> UILabel {
        let l = UILabel()
        l.text = L10n.string(key)
        l.font = .systemFont(ofSize: 15, weight: .heavy)
        l.textColor = Colors.border
        l.heightAnchor.constraint(greaterThanOrEqualToConstant: 32).isActive = true
        return l
    }

    private func reading(_ key: String, _ value: UILabel) -> UIStackView {
        let name = UILabel()
        name.text = L10n.string(key)
        name.font = .systemFont(ofSize: 13)
        value.font = .boldSystemFont(ofSize: 13)
        let row = UIStackView(arrangedSubviews: [name, value])
        row.distribution = .fillEqually
        return row
    }

    private func buttonRow(_ title: String, _ buttons: [(String, Selector)]) -> UIStackView {
        let label = UILabel()
        label.text = title
        label.font = .systemFont(ofSize: 13)
        label.widthAnchor.constraint(equalToConstant: 84).isActive = true
        let row = UIStackView(arrangedSubviews: [label])
        row.spacing = 6
        row.alignment = .center
        let group = UIStackView()
        group.distribution = .fillEqually
        group.spacing = 6
        for (text, sel) in buttons {
            var c = UIButton.Configuration.filled()
            c.title = text
            c.baseBackgroundColor = Colors.title
            c.titleLineBreakMode = .byTruncatingTail
            let b = UIButton(configuration: c)
            b.addTarget(self, action: sel, for: .touchUpInside)
            b.heightAnchor.constraint(greaterThanOrEqualToConstant: 44).isActive = true
            group.addArrangedSubview(b)
        }
        row.addArrangedSubview(group)
        return row
    }

    // MARK: LiDAR

    private func startLidar() {
        lidarScanner.onScan = { [weak self] _ in DispatchQueue.main.async { self?.refreshLidar() } }
        lidarScanner.start()
        lidarTimer?.invalidate()
        lidarTimer = Timer.scheduledTimer(withTimeInterval: 0.1, repeats: true) { [weak self] _ in self?.refreshLidar() }
        refreshLidar()
    }

    private func stopLidar() {
        lidarTimer?.invalidate()
        lidarTimer = nil
        lidarScanner.onScan = nil
        lidarScanner.stop()
    }

    private func refreshLidar() {
        lidarView.scan = lidarScanner.latestScan
        lidarView.isStale = lidarScanner.isStale
        lidarView.marginMeters = bluetooth.driveGate.marginMeters
        lidarView.showMargin = bluetooth.driveGate.enabled
        lidarStatusLabel.text = LidarView.formatStatus(scan: lidarScanner.latestScan, isFresh: lidarScanner.isFresh)
    }

    // MARK: State

    func updateRobotInfo() {
        if isBluetoothConnected { bluetooth.sendData(payload: "f\n") }
        robotInfo = bluetooth.robotInfo
        if robotInfo.isEmpty { return }
        let name = robotName(from: robotInfo)
        robotName.text = name
        let ackermann = name.hasPrefix("RC_")
        driveInfo.text = L10n.string(ackermann ? "setup_drive_ackermann" : "setup_drive_unknown")
        updateLogo(ackermann: ackermann)
        showFeatures()
        showLink()
    }

    /// Feature flags as the firmware sends them: "f<type>:i:lf:lb:ls:s:p:x:".
    private func has(_ flag: String) -> Bool { robotInfo.contains(":" + flag + ":") }

    private func showFeatures() {
        for key in ["s", "p", "x", "i", "ls", "lf", "lb"] { features[key]?.0.isChecked = has(key) }
        indicatorsRow.isHidden = !has("i")
        lightsRow.isHidden = !(has("lf") || has("lb"))
        showRcMode()
    }

    private func showRcMode() {
        let mode = RcModeFaceOverlay.shared.currentMode
        features["rc"]?.0.isChecked = mode != nil
        switch mode {
        case .manual?: rcModeValue.text = L10n.string("setup_rc_manual")
        case .auto?: rcModeValue.text = L10n.string("setup_rc_auto")
        case .hold?: rcModeValue.text = L10n.string("setup_rc_hold")
        case .lost?: rcModeValue.text = L10n.string("setup_rc_lost")
        case nil: rcModeValue.text = "N/A"
        }
    }

    private func showLink() {
        linkValue.text = L10n.string(isBluetoothConnected ? "setup_link_ble" : "setup_link_none")
    }

    private func robotName(from info: String) -> String {
        guard let f = info.firstIndex(of: "f"), let colon = info.firstIndex(of: ":"), f < colon else { return "N/A" }
        return String(info[info.index(after: f)..<colon])
    }

    private func updateLogo(ackermann: Bool) {
        if ackermann, let data = NSDataAsset(name: "rc_car")?.data {
            iconView.image = UIImage.gifImageWithData(data)
        } else {
            iconView.image = UIImage(named: "openBotLogo")
        }
    }

    @objc func updateConnect(_ notification: Notification) {
        bluetoothIconButton.setImage(isBluetoothConnected ? Images.bluetoothConnected_v2 : Images.bluetoothDisconnected_v2, for: .normal)
        if isBluetoothConnected { updateRobotInfo() } else { updateScreenOnBluetoothDisconnection() }
    }

    /// New telemetry: the LiDAR's nearest return ahead arrives as 's<cm>'.
    @objc func updateScreen() {
        let sonarData = bluetooth.sonarData
        if sonarData.count > 1 {
            let cm = Int(String(sonarData.dropFirst()).trimmingCharacters(in: .whitespacesAndNewlines)) ?? 0
            // 0 = no fresh scan (the car holds still), 300 = nothing within 3 m.
            if cm <= 0 { obstacleValue.text = L10n.string("setup_obstacle_stale") } else if cm >= 300 {
                obstacleValue.text = L10n.string("setup_obstacle_clear")
            } else {
                obstacleValue.text = "\(cm) cm"
            }
        }
        if robotInfo.isEmpty { updateRobotInfo() }
    }

    func updateScreenOnBluetoothDisconnection() {
        robotInfo = ""
        robotName.text = "N/A"
        driveInfo.text = L10n.string("setup_drive_unknown")
        obstacleValue.text = "N/A"
        updateLogo(ackermann: false)
        showFeatures()
        showLink()
    }

    // MARK: Tests (each lasts about a second: the car's heartbeat then stops it)

    private func send(throttle: Double, steer: Double) {
        let w = LidarBehaviors.toWheels(throttle: throttle, steer: steer)
        bluetooth.sendData(payload: "c" + String(Int(w.left * 255)) + "," + String(Int(w.right * 255)) + "\n")
    }

    @objc func forwardButton(_ sender: UIButton) { bluetooth.sendData(payload: "c192,192\n") }
    @objc func backwardButton(_ sender: UIButton) { bluetooth.sendData(payload: "c-192,-192\n") }
    @objc func stopButton(_ sender: UIButton) { bluetooth.sendData(payload: "c0,0\n") }
    @objc func steerLeft() { send(throttle: 0, steer: 1) }
    @objc func steerCentre() { send(throttle: 0, steer: 0) }
    @objc func steerRight() { send(throttle: 0, steer: -1) }
    @objc func indicatorLeft() { bluetooth.sendData(payload: "i1,0\n") }
    @objc func indicatorOff() { bluetooth.sendData(payload: "i0,0\n") }
    @objc func indicatorRight() { bluetooth.sendData(payload: "i0,1\n") }

    @objc func lightValueChange(_ sender: UISlider) {
        let v = Int((sender.value * 255).rounded())
        bluetooth.sendData(payload: "l\(v),\(v)\n")
    }

    @objc func openBluetoothSettings(_ sender: UIButton) {
        let next = storyboard?.instantiateViewController(withIdentifier: Strings.bluetoothScreen)
        if let next { navigationController?.pushViewController(next, animated: true) }
    }

    @objc func refreshTapped() {
        if isBluetoothConnected { updateRobotInfo() } else { updateScreenOnBluetoothDisconnection() }
    }

    func setupNavigationBarItem() {
        if UIImage(named: "back") != nil {
            let backNavigationIcon = (UIImage(named: "back")?.withRenderingMode(.alwaysOriginal))!
            let newBackButton = UIBarButtonItem(image: backNavigationIcon, title: Strings.robotInfo, target: self, action: #selector(RobotInfoFrame.back(sender:)), titleColor: Colors.navigationColor ?? .white)
            navigationItem.leftBarButtonItem = newBackButton
        }
        // Full avoidance settings: quick-disable, AVOID_MARGIN, AVOID_BACKUP_SPD.
        navigationItem.rightBarButtonItem = UIBarButtonItem(
            title: L10n.string("avoid_settings"),
            style: .plain,
            target: self,
            action: #selector(showAvoidanceSettings)
        )
    }

    @objc func showAvoidanceSettings() {
        let settings = AvoidanceSettingsController()
        settings.onChange = { [weak self] in self?.refreshLidar() }
        settings.modalPresentationStyle = .pageSheet
        present(settings, animated: true)
    }

    @objc func back(sender: UIBarButtonItem) {
        _ = navigationController?.popViewController(animated: true)
    }
}

extension String {
    func index(from: Int) -> Index {
        self.index(startIndex, offsetBy: from)
    }

    func substring(from: Int) -> String {
        let fromIndex = index(from: from)
        return String(self[fromIndex...])
    }

    func substring(to: Int) -> String {
        let toIndex = index(from: to)
        return String(self[..<toIndex])
    }

    func substring(with r: Range<Int>) -> String {
        let startIndex = index(from: r.lowerBound)
        let endIndex = index(from: r.upperBound)
        return String(self[startIndex..<endIndex])
    }
}
