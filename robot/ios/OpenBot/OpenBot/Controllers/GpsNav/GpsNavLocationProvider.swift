//
//  GpsNavLocationProvider.swift
//  OpenBot
//
//  Continuous GPS + heading for mission navigation.
//
//  `sensorDataRetrieve` only samples significant location changes (~500 m),
//  which is far too coarse to steer by, so this class owns its own
//  CLLocationManager with continuous updates and magnetometer heading. It
//  plays the same role as `GpsNavLocationProvider` on Android: publish fused
//  position and heading, and say so when there is no fix to trust.
//

import CoreLocation
import Foundation
import UIKit

/// One fused navigation sample.
struct GpsNavFix {
    let latitude: Double
    let longitude: Double
    /// Horizontal accuracy in metres; `nil` when CoreLocation has no estimate.
    let accuracyMeters: Double?
    /// Speed in m/s; negative when CoreLocation has no estimate.
    let speedMetersPerSecond: Double
    /// Course-over-ground in degrees clockwise from true north, if valid.
    let courseDegrees: Double?
    let timestamp: Date

    /// True when the course is trustworthy enough to steer by.
    ///
    /// Mirrors the Android fusion rule: course-over-ground only while moving.
    var hasCourse: Bool {
        guard let course = courseDegrees, course >= 0 else { return false }
        return speedMetersPerSecond >= GpsNavLocationProvider.courseSpeedThresholdMetersPerSecond
    }
}

protocol GpsNavLocationProviderDelegate: AnyObject {
    func locationProvider(_ provider: GpsNavLocationProvider, didUpdate fix: GpsNavFix)
    /// Called with the magnetometer heading, used when the robot is stationary.
    func locationProvider(_ provider: GpsNavLocationProvider, didUpdateHeading headingDegrees: Double)
    func locationProvider(_ provider: GpsNavLocationProvider, didFailWith message: String)
}

/// Publishes GPS fixes and heading for the mission navigation screen.
final class GpsNavLocationProvider: NSObject, CLLocationManagerDelegate {
    /// Below this speed course-over-ground is noise, so it is not used as heading.
    static let courseSpeedThresholdMetersPerSecond = 0.5

    private let manager = CLLocationManager()
    private(set) var isRunning = false

    /// False when the compass should not be steered by: Core Location rates
    /// the heading worse than `maxHeadingErrorDegrees` (it wants a figure-8
    /// calibration), or the field strength is not Earth-like (motors, the ESC
    /// or steel near the phone). Only matters while standing still - once the
    /// car moves, GPS course is the heading.
    private(set) var compassReliable = true

    /// Earth's field is about 25-65 microtesla everywhere.
    static let fieldRangeMicroTesla: ClosedRange<Double> = 25...70
    /// Worse than this and the compass is not worth steering by.
    static let maxHeadingErrorDegrees = 25.0

    /// The reliability rule on its own, so it can be unit tested.
    /// `headingAccuracy` is CLHeading's: degrees of possible error, negative
    /// when invalid. `fieldMicroTesla` is nil when no raw reading is available.
    static func isCompassReliable(headingAccuracy: Double, fieldMicroTesla: Double?) -> Bool {
        guard headingAccuracy >= 0, headingAccuracy <= maxHeadingErrorDegrees else { return false }
        guard let field = fieldMicroTesla else { return true }
        return fieldRangeMicroTesla.contains(field)
    }
    weak var delegate: GpsNavLocationProviderDelegate?

    override init() {
        super.init()
        manager.delegate = self
        // The phone is the robot's only GPS: ask for the navigation-grade fix
        // (GPS fused with the motion sensors) and never let iOS pause updates
        // because the car sat still for a moment - a paused provider would
        // read as a lost fix and hold the mission.
        manager.desiredAccuracy = kCLLocationAccuracyBestForNavigation
        manager.activityType = .otherNavigation
        manager.pausesLocationUpdatesAutomatically = false
        manager.distanceFilter = kCLDistanceFilterNone
        NotificationCenter.default.addObserver(
            self,
            selector: #selector(deviceOrientationChanged),
            name: UIDevice.orientationDidChangeNotification,
            object: nil
        )
    }

    deinit {
        NotificationCenter.default.removeObserver(self)
    }

    /// Core Location reports heading relative to the device's top edge in
    /// `headingOrientation`. With the phone in the mount in landscape that
    /// would be 90 degrees off, so keep it matched to how the phone is held.
    /// (Face-up/face-down and unknown leave the last good value alone.)
    @objc private func deviceOrientationChanged() {
        let orientation = UIDevice.current.orientation
        switch orientation {
        case .portrait, .portraitUpsideDown, .landscapeLeft, .landscapeRight:
            if let value = CLDeviceOrientation(rawValue: Int32(orientation.rawValue)) {
                manager.headingOrientation = value
            }
        default:
            break
        }
    }

    /// Requests permission and starts continuous location + heading updates.
    func start() {
        let status = manager.authorizationStatus
        switch status {
        case .notDetermined:
            manager.requestWhenInUseAuthorization()
        case .restricted, .denied:
            delegate?.locationProvider(
                self, didFailWith: "Location permission denied. Enable it in Settings."
            )
            return
        case .authorizedWhenInUse, .authorizedAlways:
            break
        @unknown default:
            break
        }

        UIDevice.current.beginGeneratingDeviceOrientationNotifications()
        deviceOrientationChanged()
        manager.startUpdatingLocation()
        if CLLocationManager.headingAvailable() {
            manager.startUpdatingHeading()
        }
        isRunning = true
    }

    func stop() {
        manager.stopUpdatingLocation()
        manager.stopUpdatingHeading()
        if isRunning {
            UIDevice.current.endGeneratingDeviceOrientationNotifications()
        }
        isRunning = false
    }

    // MARK: CLLocationManagerDelegate

    func locationManagerDidChangeAuthorization(_ manager: CLLocationManager) {
        switch manager.authorizationStatus {
        case .authorizedWhenInUse, .authorizedAlways:
            if isRunning {
                manager.startUpdatingLocation()
                if CLLocationManager.headingAvailable() {
                    manager.startUpdatingHeading()
                }
            }
        case .denied, .restricted:
            stop()
            delegate?.locationProvider(
                self, didFailWith: "Location permission denied. Enable it in Settings."
            )
        default:
            break
        }
    }

    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        guard let location = locations.last else { return }

        // A negative course is CoreLocation's "unknown".
        let course = location.course >= 0 ? location.course : nil
        let accuracy = location.horizontalAccuracy >= 0 ? location.horizontalAccuracy : nil

        let fix = GpsNavFix(
            latitude: location.coordinate.latitude,
            longitude: location.coordinate.longitude,
            accuracyMeters: accuracy,
            speedMetersPerSecond: location.speed >= 0 ? location.speed : 0.0,
            courseDegrees: course,
            timestamp: location.timestamp
        )
        delegate?.locationProvider(self, didUpdate: fix)
    }

    func locationManager(_ manager: CLLocationManager, didUpdateHeading newHeading: CLHeading) {
        // True heading is what the bearing math wants; fall back to magnetic.
        let heading = newHeading.trueHeading >= 0 ? newHeading.trueHeading : newHeading.magneticHeading
        let x = newHeading.x, y = newHeading.y, z = newHeading.z
        let field = (x == 0 && y == 0 && z == 0) ? nil : (x * x + y * y + z * z).squareRoot()
        compassReliable = GpsNavLocationProvider.isCompassReliable(
            headingAccuracy: newHeading.headingAccuracy, fieldMicroTesla: field
        )
        guard heading >= 0 else { return }
        delegate?.locationProvider(self, didUpdateHeading: heading)
    }

    /// Let iOS show its figure-8 calibration prompt when the compass needs it.
    func locationManagerShouldDisplayHeadingCalibration(_ manager: CLLocationManager) -> Bool {
        true
    }

    func locationManager(_ manager: CLLocationManager, didFailWithError error: Error) {
        guard let clError = error as? CLError, clError.code != .locationUnknown else {
            // .locationUnknown is transient; CoreLocation keeps trying.
            return
        }
        delegate?.locationProvider(self, didFailWith: error.localizedDescription)
    }
}
