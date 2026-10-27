//
//  SensorLinkView.swift
//  OpenBot
//
//  The Sensor link button for the navigation bar of GPS Mission, Object
//  Tracking and Autopilot, and the settings sheet it opens.
//

import UIKit

/// Shows the link state at a glance (antenna + "Off", client count, ✓) and
/// opens the settings. The owning screen calls `appeared()` / `disappeared()`
/// so the link runs only while one of the three screens is visible.
final class SensorLinkBarItem: UIBarButtonItem {
    private weak var owner: UIViewController?
    private let button = UIButton(type: .system)
    private var attached = false
    private let screenName: String
    private let screenSettings: ClosureSettingsProvider
    private weak var cameraOwner: CameraHubOwner?

    /// `screen` names the screen on the browser page; `extra` adds its own settings
    /// after the driving ones; `camera` is the screen's camera, if it has one.
    init(owner: UIViewController, screen: String, camera: CameraHubOwner? = nil,
         extra: ((inout [RemoteSetting]) -> Void)? = nil) {
        self.owner = owner
        self.screenName = screen
        self.cameraOwner = camera
        self.screenSettings = ClosureSettingsProvider { out in
            SensorLinkDrivingSettings.add(&out)
            extra?(&out)
        }
        super.init()
        button.setImage(UIImage(systemName: "antenna.radiowaves.left.and.right"), for: .normal)
        button.titleLabel?.font = .systemFont(ofSize: 13, weight: .semibold)
        let color = Colors.navigationColor ?? .label
        button.tintColor = color
        button.setTitleColor(color, for: .normal)
        button.accessibilityLabel = L10n.string("sensor_link_button")
        button.addTarget(self, action: #selector(tapped), for: .touchUpInside)
        customView = button
        NotificationCenter.default.addObserver(self, selector: #selector(refresh), name: .sensorLinkStatusChanged, object: nil)
        refresh()
    }

    required init?(coder: NSCoder) {
        fatalError("init(coder:) is not used")
    }

    deinit {
        NotificationCenter.default.removeObserver(self)
        if attached {
            SensorLink.shared.clearScreen(screenSettings)
            SensorLink.shared.detach()
        }
    }

    func appeared() {
        guard !attached else { return }
        attached = true
        SensorLink.shared.setScreen(screenName, screenSettings)
        if let camera = cameraOwner {
            CameraHub.shared.owner = camera
            CameraHub.shared.reapply(camera)
        }
        SensorLink.shared.attach()
        refresh()
    }

    func disappeared() {
        guard attached else { return }
        attached = false
        SensorLink.shared.clearScreen(screenSettings)
        if let camera = cameraOwner, CameraHub.shared.owner === camera { CameraHub.shared.owner = nil }
        SensorLink.shared.detach()
    }

    @objc private func refresh() {
        let link = SensorLink.shared
        button.setTitle(" " + link.shortStatus, for: .normal)
        button.alpha = link.settings.mode == .off ? 0.6 : 1
        button.accessibilityValue = link.statusText
        button.sizeToFit()
    }

    @objc private func tapped() {
        guard let owner = owner else { return }
        let nav = UINavigationController(rootViewController: SensorLinkSettingsController())
        if let sheet = nav.sheetPresentationController {
            sheet.detents = [.medium(), .large()]
            sheet.prefersGrabberVisible = true
        }
        owner.present(nav, animated: true)
    }
}

/// Off / server / client, the port to serve on, and the other phone's address.
final class SensorLinkSettingsController: UIViewController {
    private var settings = SensorLinkSettings.load()
    private let statusLabel = UILabel()
    private let errorLabel = UILabel()
    private var modeButtons: [SensorLinkSettings.Mode: UIButton] = [:]
    private let portField = UITextField()
    private let addressField = UITextField()
    private let useGpsSwitch = UISwitch()
    private let useGpsRow = UIStackView()

    override func viewDidLoad() {
        super.viewDidLoad()
        view.backgroundColor = .systemBackground
        title = L10n.string("sensor_link_title")
        navigationItem.leftBarButtonItem = UIBarButtonItem(
            title: L10n.string("sensor_link_cancel"), style: .plain, target: self, action: #selector(cancel))
        navigationItem.rightBarButtonItem = UIBarButtonItem(
            title: L10n.string("sensor_link_save"), style: .done, target: self, action: #selector(save))

        let stack = UIStackView()
        stack.axis = .vertical
        stack.spacing = 12
        stack.translatesAutoresizingMaskIntoConstraints = false
        let scroll = UIScrollView()
        scroll.translatesAutoresizingMaskIntoConstraints = false
        scroll.keyboardDismissMode = .interactive
        view.addSubview(scroll)
        scroll.addSubview(stack)
        NSLayoutConstraint.activate([
            scroll.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor),
            scroll.leadingAnchor.constraint(equalTo: view.leadingAnchor),
            scroll.trailingAnchor.constraint(equalTo: view.trailingAnchor),
            scroll.bottomAnchor.constraint(equalTo: view.bottomAnchor),
            stack.topAnchor.constraint(equalTo: scroll.contentLayoutGuide.topAnchor, constant: 16),
            stack.leadingAnchor.constraint(equalTo: scroll.frameLayoutGuide.leadingAnchor, constant: 20),
            stack.trailingAnchor.constraint(equalTo: scroll.frameLayoutGuide.trailingAnchor, constant: -20),
            stack.bottomAnchor.constraint(equalTo: scroll.contentLayoutGuide.bottomAnchor, constant: -24),
        ])

        statusLabel.numberOfLines = 0
        statusLabel.font = .systemFont(ofSize: 15, weight: .semibold)
        stack.addArrangedSubview(statusLabel)

        for (mode, key) in [(SensorLinkSettings.Mode.off, "sensor_link_mode_off"),
                            (.server, "sensor_link_mode_server"),
                            (.client, "sensor_link_mode_client")] {
            var config = UIButton.Configuration.plain()
            config.title = L10n.string(key)
            config.imagePadding = 10
            config.contentInsets = NSDirectionalEdgeInsets(top: 10, leading: 0, bottom: 10, trailing: 0)
            let b = UIButton(configuration: config)
            b.contentHorizontalAlignment = .leading
            b.tag = ["off": 0, "server": 1, "client": 2][mode.rawValue] ?? 0
            b.addTarget(self, action: #selector(modeTapped(_:)), for: .touchUpInside)
            modeButtons[mode] = b
            stack.addArrangedSubview(b)
        }

        portField.placeholder = L10n.string("sensor_link_port")
        portField.keyboardType = .numberPad
        portField.borderStyle = .roundedRect
        portField.text = String(settings.port)
        addressField.placeholder = L10n.string("sensor_link_address")
        addressField.keyboardType = .URL
        addressField.autocapitalizationType = .none
        addressField.autocorrectionType = .no
        addressField.borderStyle = .roundedRect
        addressField.text = settings.address
        [portField, addressField].forEach {
            $0.heightAnchor.constraint(greaterThanOrEqualToConstant: 44).isActive = true
            stack.addArrangedSubview($0)
        }

        let useGpsLabel = UILabel()
        useGpsLabel.text = L10n.string("sensor_link_use_gps")
        useGpsLabel.numberOfLines = 0
        useGpsSwitch.isOn = settings.useRemoteGps
        useGpsRow.axis = .horizontal
        useGpsRow.spacing = 12
        useGpsRow.alignment = .center
        useGpsRow.addArrangedSubview(useGpsLabel)
        useGpsRow.addArrangedSubview(useGpsSwitch)
        stack.addArrangedSubview(useGpsRow)

        errorLabel.textColor = .systemRed
        errorLabel.numberOfLines = 0
        errorLabel.font = .systemFont(ofSize: 13)
        errorLabel.isHidden = true
        stack.addArrangedSubview(errorLabel)

        let help = UILabel()
        help.text = L10n.string("sensor_link_help")
        help.numberOfLines = 0
        help.font = .systemFont(ofSize: 13)
        help.textColor = .secondaryLabel
        stack.addArrangedSubview(help)

        NotificationCenter.default.addObserver(self, selector: #selector(refreshStatus), name: .sensorLinkStatusChanged, object: nil)
        refreshStatus()
        showMode()
    }

    deinit {
        NotificationCenter.default.removeObserver(self)
    }

    @objc private func refreshStatus() {
        statusLabel.text = SensorLink.shared.statusText
    }

    @objc private func modeTapped(_ sender: UIButton) {
        settings.mode = [SensorLinkSettings.Mode.off, .server, .client][sender.tag]
        errorLabel.isHidden = true
        showMode()
    }

    /// Checkmark on the chosen mode; only the fields that mode uses.
    private func showMode() {
        for (mode, button) in modeButtons {
            var config = button.configuration
            config?.image = UIImage(systemName: mode == settings.mode ? "checkmark.circle.fill" : "circle")
            button.configuration = config
        }
        portField.isHidden = settings.mode != .server
        addressField.isHidden = settings.mode != .client
        useGpsRow.isHidden = settings.mode != .client
    }

    @objc private func cancel() {
        dismiss(animated: true)
    }

    @objc private func save() {
        let port = Int(portField.text?.trimmingCharacters(in: .whitespaces) ?? "") ?? -1
        let address = addressField.text?.trimmingCharacters(in: .whitespaces) ?? ""
        if settings.mode == .server && !SensorLinkSettings.isValidPort(port) {
            showError(L10n.string("sensor_link_bad_port"))
            return
        }
        if settings.mode == .client && SensorLinkProtocol.baseURL(address) == nil {
            showError(L10n.string("sensor_link_client_no_address"))
            return
        }
        if SensorLinkSettings.isValidPort(port) { settings.port = port }
        settings.address = address
        settings.useRemoteGps = useGpsSwitch.isOn
        settings.save()
        SensorLink.shared.restart()
        dismiss(animated: true)
    }

    private func showError(_ text: String) {
        errorLabel.text = text
        errorLabel.isHidden = false
    }
}
