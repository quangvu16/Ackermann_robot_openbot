//
//  GpsMissionController.swift
//  OpenBot
//
//  GPS Mission Navigation - Mission Planner's flight modes on the phone.
//
//  The phone is the robot's brain and its GPS source. This screen plans and
//  drives waypoint missions with the same feature set as Mission Planner:
//  tap to add waypoints, MISSION to fly the plan, GUIDED "go here", RTL back
//  to launch, `.waypoints` interchange and KML export of the flown track.
//

import MapKit
import UIKit
import UniformTypeIdentifiers

final class GpsMissionController: UIViewController {
    // MARK: - Outlets

    private let mapView = MKMapView()
    private let hudLabel = UILabel()        // readouts (data panel)
    private let statusLabel = UILabel()     // warnings and messages (data panel)
    private let buttonRow = UIStackView()   // Start / Stop / Return home (header)

    // Bottom panel in the Object Tracking design: header always shown, data panel hideable.
    private let panel = UIView()
    private let dataPanel = UIStackView()
    private let modeLabel = UILabel()
    private let connectionIcon = UIImageView()
    private let togglePanelButton = UIButton(type: .system)
    private let panelOpenKey = "gps_mission_panel_open"

    // MARK: - State

    private let navigator = Navigator()
    private let locationProvider = GpsNavLocationProvider()
    private var vehicleControl = Control()
    private var latestFix: GpsNavFix?
    /// Uptime of the newest fix. Steering on a fix older than `fixTimeout` would
    /// drive on a position the robot is no longer at, so the tick holds instead.
    private var latestFixUptime: TimeInterval = 0
    private let fixTimeout: TimeInterval = 3
    private let avoidanceToggle = AvoidanceToggleItem()
    private lazy var sensorLinkItem = SensorLinkBarItem(owner: self)
    private var navigationTimer: Timer?

    private let bluetooth = bluetoothDataController.shared
    private let missionFileName = "mission.waypoints"

    /// Guidance loop rate, matching the Android 5 Hz navigation tick.
    private let navPeriod: TimeInterval = 0.2

    // MARK: - Lifecycle

    override func viewDidLoad() {
        super.viewDidLoad()
        view.backgroundColor = .systemBackground
        FragmentType.shared.currentFragment = "GpsMission"

        setupNavigationBarItem()
        setupMap()
        setupPanel()

        locationProvider.delegate = self
        loadPersistedMission()
    }

    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(animated)
        locationProvider.start()
        startNavigationTimer()
        SensorLink.shared.remoteDelegate = self
        sensorLinkItem.appeared()
    }

    override func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        // Never leave the motors running after the screen goes away.
        navigator.stop()
        sendControl(control: Control())
        stopNavigationTimer()
        locationProvider.stop()
        saveMission()
        if SensorLink.shared.remoteDelegate === self { SensorLink.shared.remoteDelegate = nil }
        sensorLinkItem.disappeared()
    }

    // MARK: - UI

    private func setupNavigationBarItem() {
        let backIcon = UIImage(named: "back")?.withRenderingMode(.alwaysOriginal) ?? UIImage()
        let back = UIBarButtonItem(
            image: backIcon,
            title: Strings.gpsMission,
            target: self,
            action: #selector(back(sender:)),
            titleColor: Colors.gpsMissionColor
        )
        navigationItem.leftBarButtonItem = back
        // The object-avoidance quick-disable, in reach while a mission drives.
        // and the Sensor link (SensorServer WebSocket) button.
        navigationItem.rightBarButtonItems = [avoidanceToggle, sensorLinkItem]
    }

    @objc private func back(sender: UIBarButtonItem) {
        _ = navigationController?.popViewController(animated: true)
    }

    private func setupMap() {
        mapView.translatesAutoresizingMaskIntoConstraints = false
        mapView.delegate = self
        mapView.showsUserLocation = true
        mapView.mapType = .hybrid
        view.addSubview(mapView)

        let tap = UITapGestureRecognizer(target: self, action: #selector(mapTapped(_:)))
        mapView.addGestureRecognizer(tap)

        NSLayoutConstraint.activate([
            mapView.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor),
            mapView.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            mapView.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            mapView.bottomAnchor.constraint(equalTo: view.bottomAnchor),
        ])
    }

    private enum Action: Int {
        case startPause, stop, returnHome, addWaypoint, clear, files
    }

    private func setupPanel() {
        panel.translatesAutoresizingMaskIntoConstraints = false
        panel.backgroundColor = Colors.freeRoamButtonsColor ?? .secondarySystemBackground
        panel.layer.cornerRadius = 16
        panel.layer.maskedCorners = [.layerMinXMinYCorner, .layerMaxXMinYCorner]
        view.addSubview(panel)

        let stack = UIStackView()
        stack.axis = .vertical
        stack.spacing = 8
        stack.translatesAutoresizingMaskIntoConstraints = false
        panel.addSubview(stack)
        NSLayoutConstraint.activate([
            panel.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            panel.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            panel.bottomAnchor.constraint(equalTo: view.bottomAnchor),
            stack.topAnchor.constraint(equalTo: panel.topAnchor, constant: 4),
            stack.leadingAnchor.constraint(equalTo: panel.leadingAnchor, constant: 16),
            stack.trailingAnchor.constraint(equalTo: panel.trailingAnchor, constant: -16),
            stack.bottomAnchor.constraint(equalTo: panel.safeAreaLayoutGuide.bottomAnchor, constant: -12),
        ])

        // Handle: the same bar as Object Tracking, in a strip that is easy to hit.
        let handleArea = UIView()
        handleArea.heightAnchor.constraint(equalToConstant: 24).isActive = true
        let bar = UIView()
        bar.backgroundColor = Colors.title ?? .systemGray
        bar.layer.cornerRadius = 2.5
        bar.translatesAutoresizingMaskIntoConstraints = false
        handleArea.addSubview(bar)
        NSLayoutConstraint.activate([
            bar.centerXAnchor.constraint(equalTo: handleArea.centerXAnchor),
            bar.centerYAnchor.constraint(equalTo: handleArea.centerYAnchor),
            bar.widthAnchor.constraint(equalToConstant: 50),
            bar.heightAnchor.constraint(equalToConstant: 5),
        ])
        handleArea.isAccessibilityElement = true
        handleArea.accessibilityLabel = L10n.string("gps_panel_toggle")
        handleArea.accessibilityTraits = .button
        handleArea.addGestureRecognizer(UITapGestureRecognizer(target: self, action: #selector(togglePanel)))
        stack.addArrangedSubview(handleArea)

        // Header row: status, connection, Hide / Show data.
        modeLabel.font = .systemFont(ofSize: 16, weight: .bold)
        modeLabel.textColor = .label
        modeLabel.numberOfLines = 2
        connectionIcon.image = UIImage(systemName: "antenna.radiowaves.left.and.right")
        connectionIcon.contentMode = .scaleAspectFit
        connectionIcon.accessibilityLabel = L10n.string("gps_panel_connection")
        connectionIcon.widthAnchor.constraint(equalToConstant: 24).isActive = true
        connectionIcon.heightAnchor.constraint(equalToConstant: 24).isActive = true
        togglePanelButton.titleLabel?.font = .systemFont(ofSize: 15, weight: .semibold)
        togglePanelButton.setTitleColor(Colors.gpsMissionColor, for: .normal)
        togglePanelButton.addTarget(self, action: #selector(togglePanel), for: .touchUpInside)
        togglePanelButton.setContentHuggingPriority(.required, for: .horizontal)
        togglePanelButton.heightAnchor.constraint(greaterThanOrEqualToConstant: 44).isActive = true
        let header = UIStackView(arrangedSubviews: [modeLabel, connectionIcon, togglePanelButton])
        header.axis = .horizontal
        header.spacing = 10
        header.alignment = .center
        stack.addArrangedSubview(header)

        // The three actions a running mission needs, always in reach.
        buttonRow.axis = .horizontal
        buttonRow.spacing = 8
        buttonRow.distribution = .fillEqually
        buttonRow.heightAnchor.constraint(equalToConstant: 48).isActive = true
        buttonRow.addArrangedSubview(makeButton(L10n.string("gps_nav_start"), .startPause, filled: true))
        buttonRow.addArrangedSubview(makeButton(L10n.string("gps_nav_stop"), .stop, filled: false))
        buttonRow.addArrangedSubview(makeButton(L10n.string("gps_nav_rtl"), .returnHome, filled: false))
        stack.addArrangedSubview(buttonRow)

        // Data panel: readouts, warnings, planning tools. Hideable.
        dataPanel.axis = .vertical
        dataPanel.spacing = 8
        hudLabel.font = .monospacedDigitSystemFont(ofSize: 14, weight: .semibold)
        hudLabel.textColor = .label
        hudLabel.numberOfLines = 0
        statusLabel.font = .systemFont(ofSize: 13)
        statusLabel.textColor = .secondaryLabel
        statusLabel.numberOfLines = 0
        let tools = UIStackView(arrangedSubviews: [
            makeButton(L10n.string("gps_nav_add_wp"), .addWaypoint, filled: false),
            makeButton(L10n.string("gps_nav_clear"), .clear, filled: false),
            makeButton(L10n.string("gps_nav_files"), .files, filled: false),
        ])
        tools.axis = .horizontal
        tools.spacing = 8
        tools.distribution = .fillEqually
        tools.heightAnchor.constraint(equalToConstant: 44).isActive = true
        let divider = UIView()
        divider.backgroundColor = .separator
        divider.heightAnchor.constraint(equalToConstant: 1).isActive = true
        [divider, hudLabel, statusLabel, tools].forEach { dataPanel.addArrangedSubview($0) }
        stack.addArrangedSubview(dataPanel)

        // Swipe down to hide the data, up to show it, as on Object Tracking.
        let down = UISwipeGestureRecognizer(target: self, action: #selector(swiped(_:)))
        down.direction = .down
        let up = UISwipeGestureRecognizer(target: self, action: #selector(swiped(_:)))
        up.direction = .up
        panel.addGestureRecognizer(down)
        panel.addGestureRecognizer(up)

        let open = UserDefaults.standard.object(forKey: panelOpenKey) as? Bool ?? true
        setPanelOpen(open, animated: false)
    }

    private func makeButton(_ title: String, _ action: Action, filled: Bool) -> UIButton {
        let button = UIButton(type: .system)
        button.setTitle(title, for: .normal)
        button.tag = action.rawValue
        button.titleLabel?.font = .systemFont(ofSize: 15, weight: .semibold)
        button.titleLabel?.adjustsFontSizeToFitWidth = true
        button.titleLabel?.minimumScaleFactor = 0.7
        button.layer.cornerRadius = 12
        if filled {
            button.backgroundColor = Colors.gpsMissionColor
            button.setTitleColor(.white, for: .normal)
        } else {
            button.backgroundColor = .clear
            button.layer.borderWidth = 1.5
            button.layer.borderColor = Colors.gpsMissionColor.cgColor
            button.setTitleColor(Colors.gpsMissionColor, for: .normal)
        }
        button.addTarget(self, action: #selector(buttonTapped(_:)), for: .touchUpInside)
        return button
    }

    @objc private func togglePanel() {
        setPanelOpen(dataPanel.isHidden, animated: true)
    }

    @objc private func swiped(_ gesture: UISwipeGestureRecognizer) {
        setPanelOpen(gesture.direction == .up, animated: true)
    }

    private func setPanelOpen(_ open: Bool, animated: Bool) {
        UserDefaults.standard.set(open, forKey: panelOpenKey)
        togglePanelButton.setTitle(L10n.string(open ? "gps_panel_hide" : "gps_panel_show"), for: .normal)
        guard dataPanel.isHidden == open else { return }
        let change = {
            self.dataPanel.isHidden = !open
            self.dataPanel.alpha = open ? 1 : 0
            self.view.layoutIfNeeded()
        }
        if animated {
            UIView.animate(withDuration: 0.25, animations: change)
        } else {
            change()
        }
    }

    @objc private func buttonTapped(_ sender: UIButton) {
        switch Action(rawValue: sender.tag) {
        case .startPause: toggleStartPause()
        case .stop: stopNavigation()
        case .returnHome: startRTL()
        case .addWaypoint: addWaypointAtRobot()
        case .clear: clearMission()
        case .files: showFileActions(from: sender)
        case .none: break
        }
    }

    // MARK: - Mission actions

    private func toggleStartPause() {
        if navigator.isNavigating && !navigator.isPaused {
            navigator.pause()
            sendControl(control: Control())
        } else if navigator.isNavigating {
            navigator.resume()
        } else {
            guard hasUsableFix() else {
                showAlert(L10n.string("gps_nav_no_fix"))
                return
            }
            if navigator.mission.homeLatitude == nil, let fix = latestFix {
                navigator.markHome(fix.latitude, fix.longitude)
            }
            if navigator.mission.isEmpty {
                showAlert(L10n.string("gps_nav_no_waypoints"))
                return
            }
            navigator.startMission()
        }
        refreshHud()
    }

    private func stopNavigation() {
        navigator.stop()
        sendControl(control: Control())
        refreshHud()
    }

    private func startRTL() {
        guard hasUsableFix() else {
            showAlert(L10n.string("gps_nav_no_fix"))
            return
        }
        if navigator.mission.homeLatitude == nil, let fix = latestFix {
            navigator.markHome(fix.latitude, fix.longitude)
        }
        if navigator.startRTL() {
            refreshHud()
        } else {
            showAlert(L10n.string("gps_nav_no_home"))
        }
    }

    private func addWaypointAtRobot() {
        guard let fix = latestFix else {
            showAlert(L10n.string("gps_nav_no_fix"))
            return
        }
        addWaypoint(latitude: fix.latitude, longitude: fix.longitude)
    }

    private func addWaypoint(latitude: Double, longitude: Double) {
        let index = navigator.mission.count + 1
        navigator.mission.addWaypoint(
            Waypoint(latitude: latitude, longitude: longitude, name: "WP\(index)")
        )
        if navigator.mission.homeLatitude == nil {
            navigator.markHome(latitude, longitude)
        }
        saveMission()
        refreshMap()
        refreshHud()
    }

    private func clearMission() {
        navigator.stop()
        sendControl(control: Control())
        navigator.mission.clearWaypoints()
        navigator.clearTrack()
        saveMission()
        refreshMap()
        refreshHud()
    }

    /// Mission Planner interchange and KML export, in one menu so the button
    /// row stays readable on a phone.
    private func showFileActions(from sender: UIView) {
        let sheet = UIAlertController(title: nil, message: nil, preferredStyle: .actionSheet)
        sheet.addAction(UIAlertAction(title: L10n.string("gps_nav_export_kml"), style: .default) { [weak self] _ in
            self?.exportKml()
        })
        sheet.addAction(UIAlertAction(title: L10n.string("gps_nav_share_waypoints"), style: .default) { [weak self] _ in
            self?.shareWaypoints()
        })
        sheet.addAction(UIAlertAction(title: L10n.string("gps_nav_open_waypoints"), style: .default) { [weak self] _ in
            self?.openWaypoints()
        })
        sheet.addAction(UIAlertAction(title: L10n.string("gps_nav_cancel"), style: .cancel))
        // An action sheet on iPad must be anchored or UIKit raises.
        sheet.popoverPresentationController?.sourceView = sender
        sheet.popoverPresentationController?.sourceRect = sender.bounds
        present(sheet, animated: true)
    }

    /// Write the plan as a Mission Planner `.waypoints` file and hand it to the
    /// share sheet - AirDrop to a PC, Files, Drive, Mail.
    private func shareWaypoints() {
        let url = FileManager.default.temporaryDirectory.appendingPathComponent(missionFileName)
        do {
            try navigator.saveMissionFile().write(to: url, atomically: true, encoding: .utf8)
            share(url)
        } catch {
            showAlert(error.localizedDescription)
        }
    }

    /// Pick a `.waypoints` file planned in Mission Planner or QGroundControl.
    private func openWaypoints() {
        let picker = UIDocumentPickerViewController(
            forOpeningContentTypes: [.plainText, .text, .data, .item],
            asCopy: true
        )
        picker.delegate = self
        picker.allowsMultipleSelection = false
        present(picker, animated: true)
    }

    fileprivate func importWaypoints(from url: URL) {
        do {
            let contents = try String(contentsOf: url, encoding: .utf8)
            let mission = try Mission.parseWaypointsFile(contents)
            // Swapping the plan under a running mission would retarget the car
            // mid-drive, so stop first.
            stopNavigation()
            navigator.loadMission(mission)
            saveMission()
            refreshMap()
            refreshHud()
            showAlert(String(format: L10n.string("gps_nav_imported"), mission.count))
        } catch {
            showAlert(error.localizedDescription)
        }
    }

    private func share(_ url: URL) {
        let activity = UIActivityViewController(activityItems: [url], applicationActivities: nil)
        activity.popoverPresentationController?.sourceView = buttonRow
        activity.popoverPresentationController?.sourceRect = buttonRow.bounds
        present(activity, animated: true)
    }

    private func exportKml() {
        let kml = navigator.exportKml(documentName: Strings.gpsMission)
        guard let directory = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first else {
            return
        }
        let formatter = DateFormatter()
        formatter.dateFormat = "yyyyMMdd-HHmmss"
        let path = directory.appendingPathComponent("mission-\(formatter.string(from: Date())).kml")
        do {
            try kml.write(to: path, atomically: true, encoding: .utf8)
            // Kept in Documents (visible in the Files app) and offered to the
            // share sheet so it can go straight to Google Earth or a PC.
            share(path)
        } catch {
            showAlert(error.localizedDescription)
        }
    }

    @objc private func mapTapped(_ recognizer: UITapGestureRecognizer) {
        let point = recognizer.location(in: mapView)
        let coordinate = mapView.convert(point, toCoordinateFrom: mapView)

        let sheet = UIAlertController(title: nil, message: nil, preferredStyle: .actionSheet)
        sheet.addAction(UIAlertAction(title: L10n.string("gps_nav_add_here"), style: .default) { [weak self] _ in
            self?.addWaypoint(latitude: coordinate.latitude, longitude: coordinate.longitude)
        })
        sheet.addAction(UIAlertAction(title: L10n.string("gps_nav_go_here"), style: .default) { [weak self] _ in
            self?.startGuided(to: coordinate)
        })
        sheet.addAction(UIAlertAction(title: L10n.string("gps_nav_cancel"), style: .cancel))
        sheet.popoverPresentationController?.sourceView = mapView
        sheet.popoverPresentationController?.sourceRect = CGRect(origin: point, size: .zero)
        present(sheet, animated: true)
    }

    private func startGuided(to coordinate: CLLocationCoordinate2D) {
        guard hasUsableFix() else {
            showAlert(L10n.string("gps_nav_no_fix"))
            return
        }
        if navigator.mission.homeLatitude == nil, let fix = latestFix {
            navigator.markHome(fix.latitude, fix.longitude)
        }
        navigator.startGuided(latitude: coordinate.latitude, longitude: coordinate.longitude)
        refreshHud()
    }

    // MARK: - Control loop

    private func startNavigationTimer() {
        stopNavigationTimer()
        navigationTimer = Timer.scheduledTimer(withTimeInterval: navPeriod, repeats: true) { [weak self] _ in
            self?.tick()
        }
    }

    private func stopNavigationTimer() {
        navigationTimer?.invalidate()
        navigationTimer = nil
    }

    private func tick() {
        avoidanceToggle.refresh()
        guard let fix = latestFix else { return }
        if navigator.isNavigating,
           ProcessInfo.processInfo.systemUptime - latestFixUptime > fixTimeout {
            // The provider stopped reporting: hold rather than steer on a
            // position that is seconds old. Navigation resumes with the next fix.
            sendControl(control: Control())
            statusLabel.text = L10n.string("gps_nav_fix_lost")
            modeLabel.text = L10n.string("gps_nav_fix_lost")
            return
        }

        let status = navigator.step(
            latitude: fix.latitude,
            longitude: fix.longitude,
            accuracyMeters: fix.accuracyMeters
        )
        sendControl(control: Control(left: Float(status.left), right: Float(status.right)))
        refreshHud()
    }

    /// PointGoalFragment's sendControl pattern: only write to the robot on change.
    private func sendControl(control: Control) {
        guard control.getRight() != vehicleControl.getRight()
            || control.getLeft() != vehicleControl.getLeft() else { return }

        let left = control.getLeft() * gameController.selectedSpeedMode.rawValue
        let right = control.getRight() * gameController.selectedSpeedMode.rawValue
        vehicleControl = control
        bluetooth.sendData(payload: "c" + String(left) + "," + String(right) + "\n")
        NotificationCenter.default.post(
            name: .updateSpeedLabel,
            object: String(Int(left)) + "," + String(Int(right))
        )
    }

    // MARK: - HUD and map

    private func refreshHud() {
        let status = navigator.lastStatus
        modeLabel.text = navigator.isPaused ? L10n.string("gps_nav_paused") : L10n.string("gps_mode_\(navigator.mode.rawValue)")
        connectionIcon.tintColor = isBluetoothConnected ? .systemGreen : .systemGray
        var parts: [String] = []

        if let index = status?.activeWaypointIndex, navigator.mode == .mission {
            parts.append("WP \(index + 1)/\(navigator.mission.count)")
        }
        if let distance = status?.distanceMeters {
            parts.append(String(format: "%.1f m", distance))
        }
        if let bearing = status?.bearingDegrees {
            parts.append(String(format: "brg %03.0f", bearing))
        }
        if let error = status?.turnErrorDegrees {
            parts.append(String(format: "err %+04.0f", error))
        }
        if let fix = latestFix {
            parts.append(String(format: "acc %.1f m", fix.accuracyMeters ?? -1))
        }
        hudLabel.text = parts.isEmpty ? "–" : parts.joined(separator: "   ")

        var detail: [String] = []
        if navigator.isPaused { detail.append(L10n.string("gps_nav_paused")) }
        if let message = status?.message, !message.isEmpty { detail.append(message) }
        if navigator.isNavigating,
           !(latestFix?.hasCourse ?? false),
           !locationProvider.compassReliable {
            // Steering is on the compass right now, and the compass is off.
            detail.append(L10n.string("gps_nav_compass_unreliable"))
        }
        statusLabel.text = detail.joined(separator: " · ")
        // Warnings also go in the header, which stays visible with the data hidden.
        if let warning = detail.last(where: { $0 != L10n.string("gps_nav_paused") }) {
            modeLabel.text = (modeLabel.text ?? "") + "\n" + warning
        }

        refreshMap()
    }

    private func refreshMap() {
        mapView.removeOverlays(mapView.overlays)
        mapView.removeAnnotations(mapView.annotations.filter { !($0 is MKUserLocation) })

        let mission = navigator.mission
        if let homeLat = mission.homeLatitude, let homeLon = mission.homeLongitude {
            let home = MKPointAnnotation()
            home.title = "Home"
            home.coordinate = CLLocationCoordinate2D(latitude: homeLat, longitude: homeLon)
            mapView.addAnnotation(home)
        }

        for (offset, waypoint) in mission.waypoints.enumerated() {
            let annotation = MKPointAnnotation()
            annotation.title = "WP\(offset + 1)"
            annotation.coordinate = CLLocationCoordinate2D(
                latitude: waypoint.latitude, longitude: waypoint.longitude
            )
            mapView.addAnnotation(annotation)
        }

        var path: [CLLocationCoordinate2D] = []
        if let homeLat = mission.homeLatitude, let homeLon = mission.homeLongitude {
            path.append(CLLocationCoordinate2D(latitude: homeLat, longitude: homeLon))
        }
        path += mission.waypoints.map {
            CLLocationCoordinate2D(latitude: $0.latitude, longitude: $0.longitude)
        }
        if path.count > 1 {
            mapView.addOverlay(MKPolyline(coordinates: path, count: path.count))
        }

        let track = navigator.track.map {
            CLLocationCoordinate2D(latitude: $0.latitude, longitude: $0.longitude)
        }
        if track.count > 1 {
            mapView.addOverlay(MKPolyline(coordinates: track, count: track.count))
        }
    }

    // MARK: - Persistence

    private var missionFileURL: URL? {
        FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first?
            .appendingPathComponent(missionFileName)
    }

    private func saveMission() {
        guard let url = missionFileURL else { return }
        try? navigator.saveMissionFile().write(to: url, atomically: true, encoding: .utf8)
    }

    private func loadPersistedMission() {
        guard let url = missionFileURL,
              let contents = try? String(contentsOf: url, encoding: .utf8),
              let mission = try? Mission.parseWaypointsFile(contents)
        else { return }
        navigator.loadMission(mission)
        refreshMap()
    }

    // MARK: - Helpers

    private func hasUsableFix() -> Bool {
        guard let fix = latestFix, let accuracy = fix.accuracyMeters else { return false }
        return accuracy <= Navigator.maxPositionAccuracyMeters
    }

    private func showAlert(_ message: String) {
        let alert = UIAlertController(title: nil, message: message, preferredStyle: .alert)
        alert.addAction(UIAlertAction(title: L10n.string("gps_nav_ok"), style: .default))
        present(alert, animated: true)
    }
}

// MARK: - GpsNavLocationProviderDelegate

extension GpsMissionController: GpsNavLocationProviderDelegate {
    func locationProvider(_ provider: GpsNavLocationProvider, didUpdate fix: GpsNavFix) {
        latestFix = fix
        latestFixUptime = ProcessInfo.processInfo.systemUptime
        // Course-over-ground is the steering heading while moving.
        navigator.updateHeading(fix.courseDegrees, speedMetersPerSecond: fix.speedMetersPerSecond)
    }

    func locationProvider(_ provider: GpsNavLocationProvider, didUpdateHeading headingDegrees: Double) {
        // Magnetometer heading keeps steering sane while the robot is stationary.
        // Keep following it until GPS course takes over: taking only the first
        // reading would lock in a bad one from an uncalibrated compass.
        if !(latestFix?.hasCourse ?? false) {
            navigator.setHeading(headingDegrees)
        }
    }

    func locationProvider(_ provider: GpsNavLocationProvider, didFailWith message: String) {
        statusLabel.text = message
        modeLabel.text = message
    }
}

// MARK: - SensorLinkRemoteDelegate

extension GpsMissionController: SensorLinkRemoteDelegate {
    func sensorLink(didReceiveRemoteFix fix: SensorLinkProtocol.GpsFix) {
        locationProvider.remoteFix(fix)
    }

    func sensorLink(didReceiveRemoteRotationVector values: [Double]) {
        locationProvider.remoteRotationVector(values)
    }
}

// MARK: - MKMapViewDelegate

extension GpsMissionController: MKMapViewDelegate {
    func mapView(_ mapView: MKMapView, rendererFor overlay: MKOverlay) -> MKOverlayRenderer {
        guard let polyline = overlay as? MKPolyline else {
            return MKOverlayRenderer(overlay: overlay)
        }
        let renderer = MKPolylineRenderer(polyline: polyline)
        // Dashed cyan for the plan, solid green for the track actually flown.
        if polyline.pointCount > 0 {
            let first = polyline.points()[0]
            let isTrack = navigator.track.contains {
                abs($0.latitude - first.coordinate.latitude) < 1e-9
                    && abs($0.longitude - first.coordinate.longitude) < 1e-9
            }
            renderer.strokeColor = isTrack ? .systemGreen : .systemCyan
        } else {
            renderer.strokeColor = .systemCyan
        }
        renderer.lineWidth = 3
        return renderer
    }
}

// MARK: - UIDocumentPickerDelegate

extension GpsMissionController: UIDocumentPickerDelegate {
    func documentPicker(_ controller: UIDocumentPickerViewController, didPickDocumentsAt urls: [URL]) {
        guard let url = urls.first else { return }
        importWaypoints(from: url)
    }
}
