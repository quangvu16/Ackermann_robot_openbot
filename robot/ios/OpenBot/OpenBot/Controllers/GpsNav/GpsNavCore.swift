//
//  GpsNavCore.swift
//  OpenBot
//
//  Pure GPS mission-navigation core: geo math, waypoints, Mission Planner
//  interchange, the differential-drive guidance law and the MISSION / GUIDED /
//  RTL state machine.
//
//  Mirrors the Android `gpsNav` package and the Linux `gpsnav` module so a
//  mission planned on one platform behaves identically on the others. Nothing
//  here touches UIKit or CoreLocation, so it is unit testable in isolation.
//

import Foundation

// MARK: - Geo math

/// Geographic helpers. All angles in degrees, all distances in metres.
enum GeoMath {
    /// Mean earth radius used by the haversine formula.
    static let earthRadiusMeters = 6371008.8

    private static let degToRad = Double.pi / 180.0
    private static let radToDeg = 180.0 / Double.pi

    /// Great-circle distance between two WGS-84 points, in metres.
    static func distanceMeters(_ lat1: Double, _ lon1: Double, _ lat2: Double, _ lon2: Double) -> Double {
        let dLat = (lat2 - lat1) * degToRad
        let dLon = (lon2 - lon1) * degToRad
        let a = sin(dLat / 2) * sin(dLat / 2)
            + cos(lat1 * degToRad) * cos(lat2 * degToRad) * sin(dLon / 2) * sin(dLon / 2)
        return 2 * earthRadiusMeters * asin(min(1.0, sqrt(a)))
    }

    /// Initial great-circle bearing from the first point to the second.
    ///
    /// Result is in degrees clockwise from true north, wrapped to `[-180, 180)`.
    static func bearingDegrees(_ lat1: Double, _ lon1: Double, _ lat2: Double, _ lon2: Double) -> Double {
        let phi1 = lat1 * degToRad
        let phi2 = lat2 * degToRad
        let dLon = (lon2 - lon1) * degToRad
        let y = sin(dLon) * cos(phi2)
        let x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(dLon)
        return wrapDegrees(atan2(y, x) * radToDeg)
    }

    /// Wraps an angle to the range `[-180, 180)`.
    static func wrapDegrees(_ degrees: Double) -> Double {
        let shifted = (degrees + 180.0).truncatingRemainder(dividingBy: 360.0)
        return (shifted + 360.0).truncatingRemainder(dividingBy: 360.0) - 180.0
    }

    /// Smallest signed turn from a heading onto a target bearing.
    ///
    /// Positive means the target is to the right (clockwise), negative to the left.
    static func turnErrorDegrees(_ headingDegrees: Double, _ targetBearingDegrees: Double) -> Double {
        wrapDegrees(targetBearingDegrees - headingDegrees)
    }

    /// Moves a point along a great circle by the given distance and initial bearing.
    static func destinationPoint(_ latitude: Double, _ longitude: Double, _ bearingDegrees: Double, _ distanceMeters: Double) -> (latitude: Double, longitude: Double) {
        let angular = distanceMeters / earthRadiusMeters
        let theta = bearingDegrees * degToRad
        let phi1 = latitude * degToRad
        let lambda1 = longitude * degToRad

        let sinPhi2 = sin(phi1) * cos(angular) + cos(phi1) * sin(angular) * cos(theta)
        let phi2 = asin(max(-1.0, min(1.0, sinPhi2)))
        let lambda2 = lambda1 + atan2(
            sin(theta) * sin(angular) * cos(phi1),
            cos(angular) - sin(phi1) * sinPhi2
        )
        return (phi2 * radToDeg, wrapDegrees(lambda2 * radToDeg))
    }
}

// MARK: - Waypoint

/// A single GPS waypoint, the ground-robot equivalent of a Mission Planner entry.
///
/// Waypoints only carry the horizontal target plus an acceptance radius; unlike
/// an aircraft mission there is no altitude leg.
struct Waypoint: Equatable {
    let latitude: Double
    let longitude: Double
    var name: String
    /// Radius in metres at which this waypoint counts as reached. Floor of 0.5 m.
    var acceptanceRadiusMeters: Double

    init(latitude: Double, longitude: Double, name: String = "", acceptanceRadiusMeters: Double = defaultAcceptanceRadiusMeters) {
        self.latitude = latitude
        self.longitude = longitude
        self.name = name
        self.acceptanceRadiusMeters = max(0.5, acceptanceRadiusMeters)
    }

    /// Distance in metres from the given position to this waypoint.
    func distanceFrom(_ latitude: Double, _ longitude: Double) -> Double {
        GeoMath.distanceMeters(latitude, longitude, self.latitude, self.longitude)
    }

    /// Initial bearing in degrees from the given position to this waypoint.
    func bearingFrom(_ latitude: Double, _ longitude: Double) -> Double {
        GeoMath.bearingDegrees(latitude, longitude, self.latitude, self.longitude)
    }
}

// MARK: - Mission

/// Default waypoint acceptance radius. The phone's own GPS is good to about
/// 3-5 m under open sky, so a tighter radius leaves the car circling a waypoint
/// it can never quite "reach". (The Linux robot, which reads an external
/// receiver that may be RTK, keeps 2.5 m.)
let defaultAcceptanceRadiusMeters = 4.0

enum MissionError: Error, Equatable {
    case emptyFile
    case unsupportedHeader(String)
    case malformedRow(String)
}

/// An ordered GPS mission: a home position plus a list of waypoints to visit.
///
/// Serialises to the QGroundControl / Mission Planner `.waypoints` format
/// (`QGC WPL 110`) so missions can be planned in Mission Planner and driven
/// here, or the other way round. Only the horizontal component is meaningful
/// for a ground robot, so the altitude column is written as zero and ignored
/// on read.
class Mission {
    static let wplHeader = "QGC WPL 110"
    static let cmdNavWaypoint = 16

    private(set) var waypoints: [Waypoint] = []
    var homeLatitude: Double?
    var homeLongitude: Double?

    var isEmpty: Bool { waypoints.isEmpty }
    var count: Int { waypoints.count }
    var hasHome: Bool { homeLatitude != nil && homeLongitude != nil }

    // MARK: bookkeeping

    func clearWaypoints() {
        waypoints.removeAll()
    }

    func addWaypoint(_ waypoint: Waypoint, at index: Int? = nil) {
        if let index = index {
            waypoints.insert(waypoint, at: max(0, min(index, waypoints.count)))
        } else {
            waypoints.append(waypoint)
        }
    }

    func removeWaypoint(at index: Int) {
        guard waypoints.indices.contains(index) else { return }
        waypoints.remove(at: index)
    }

    func moveWaypoint(from fromIndex: Int, to toIndex: Int) {
        guard waypoints.indices.contains(fromIndex), waypoints.indices.contains(toIndex) else { return }
        waypoints.insert(waypoints.remove(at: fromIndex), at: toIndex)
    }

    func setWaypoints(_ replacement: [Waypoint]) {
        waypoints = replacement
    }

    func waypoint(at index: Int) -> Waypoint? {
        waypoints.indices.contains(index) ? waypoints[index] : nil
    }

    func setHome(_ latitude: Double, _ longitude: Double) {
        homeLatitude = latitude
        homeLongitude = longitude
    }

    /// Total along-track length in metres, home to last waypoint.
    func totalDistanceMeters() -> Double {
        guard !waypoints.isEmpty else { return 0.0 }

        var total = 0.0
        var previous: (latitude: Double, longitude: Double)
        var hasPrevious: Bool

        if let homeLat = homeLatitude, let homeLon = homeLongitude {
            previous = (homeLat, homeLon)
            hasPrevious = true
        } else {
            previous = (0, 0)
            hasPrevious = false
        }

        for waypoint in waypoints {
            if hasPrevious {
                total += GeoMath.distanceMeters(
                    previous.latitude, previous.longitude,
                    waypoint.latitude, waypoint.longitude
                )
            }
            previous = (waypoint.latitude, waypoint.longitude)
            hasPrevious = true
        }
        return total
    }

    // MARK: Mission Planner .waypoints codec

    /// Serialises this mission to the Mission Planner / QGC waypoint format.
    ///
    /// Index 0 is the home position (`current` flag set), matching what Mission
    /// Planner writes. When no home has been recorded yet the first waypoint is
    /// used so the file stays valid.
    func toWaypointsFile() -> String {
        var lines = [Mission.wplHeader]
        if hasHome || !waypoints.isEmpty {
            let lat: Double
            let lon: Double
            if let homeLat = homeLatitude, let homeLon = homeLongitude {
                lat = homeLat
                lon = homeLon
            } else {
                lat = waypoints[0].latitude
                lon = waypoints[0].longitude
            }
            lines.append(Mission.formatWplLine(index: 0, current: 1, latitude: lat, longitude: lon))
        }
        for (offset, waypoint) in waypoints.enumerated() {
            lines.append(Mission.formatWplLine(
                index: offset + 1, current: 0,
                latitude: waypoint.latitude, longitude: waypoint.longitude
            ))
        }
        return lines.joined(separator: "\n") + "\n"
    }

    /// Parses a Mission Planner / QGroundControl waypoint file.
    ///
    /// Row 0 is treated as the home position; every subsequent `NAV_WAYPOINT`
    /// row becomes a mission waypoint. Non-navigation commands are skipped
    /// rather than rejected so files planned for a rover with extra `DO_`
    /// commands still load their waypoints.
    static func parseWaypointsFile(_ contents: String) throws -> Mission {
        let mission = Mission()
        let lines = contents.split(separator: "\n", omittingEmptySubsequences: false).map(String.init)
        guard let first = lines.first, !first.trimmingCharacters(in: .whitespaces).isEmpty else {
            throw MissionError.emptyFile
        }

        let header = first.trimmingCharacters(in: .whitespaces)
        guard header.hasPrefix("QGC WPL") else {
            throw MissionError.unsupportedHeader(header)
        }

        for (rowIndex, rawLine) in lines.dropFirst().enumerated() {
            let line = rawLine.trimmingCharacters(in: .whitespaces)
            if line.isEmpty { continue }

            var fields = line.components(separatedBy: "\t")
            if fields.count < 11 {
                fields = line.components(separatedBy: .whitespaces).filter { !$0.isEmpty }
            }
            guard fields.count >= 11,
                  let command = Int(fields[3]),
                  let latitude = Double(fields[8]),
                  let longitude = Double(fields[9])
            else {
                throw MissionError.malformedRow(line)
            }

            if rowIndex == 0 {
                mission.setHome(latitude, longitude)
            } else if command == Mission.cmdNavWaypoint {
                mission.addWaypoint(Waypoint(latitude: latitude, longitude: longitude))
            }
        }
        return mission
    }

    private static func formatWplLine(index: Int, current: Int, latitude: Double, longitude: Double) -> String {
        // index, current, frame, command, p1, p2, p3, p4, x, y, z, autocontinue
        String(
            format: "%d\t%d\t3\t%d\t0\t0\t0\t0\t%.8f\t%.8f\t0\t1",
            index, current, Mission.cmdNavWaypoint, latitude, longitude
        )
    }
}

// MARK: - Guidance

/// Outcome of one guidance step.
struct GuidanceResult {
    /// Normalised left wheel command in `[-1, 1]`.
    let left: Double
    /// Normalised right wheel command in `[-1, 1]`.
    let right: Double
    let distanceMeters: Double
    let bearingDegrees: Double
    /// Signed turn needed to point at the target; positive is to the right.
    let turnErrorDegrees: Double
    let arrived: Bool
}

/// Steers a differential-drive robot toward a GPS waypoint.
///
/// This is the ground-robot equivalent of Mission Planner's "go to waypoint" /
/// guided mode. The defaults match the Android implementation exactly so the
/// three platforms drive the same way.
struct GuidanceController {
    /// Distance at which a waypoint counts as reached.
    var acceptanceRadiusMeters: Double = defaultAcceptanceRadiusMeters
    /// Distance at which the approach speed ramp begins.
    var slowdownRadiusMeters: Double = 8.0
    /// Heading error that saturates steering.
    var fullLockBearingErrorDegrees: Double = 70.0
    /// Heading error above which we pivot on the spot rather than arc.
    var pivotBearingErrorDegrees: Double = 55.0
    /// Throttle used far from the waypoint, in `[0, 1]`.
    var cruiseThrottle: Double = 0.70
    /// Throttle floor so the motors keep turning near the waypoint.
    var minThrottle: Double = 0.28

    func compute(latitude: Double, longitude: Double, headingDegrees: Double, target: Waypoint) -> GuidanceResult {
        let distance = target.distanceFrom(latitude, longitude)
        let bearing = target.bearingFrom(latitude, longitude)
        let turnError = GeoMath.turnErrorDegrees(headingDegrees, bearing)

        let radius = max(acceptanceRadiusMeters, target.acceptanceRadiusMeters)
        if distance <= radius {
            return GuidanceResult(
                left: 0, right: 0,
                distanceMeters: distance, bearingDegrees: bearing,
                turnErrorDegrees: turnError, arrived: true
            )
        }

        // Positive turn error means the target lies to the right of the nose.
        let turn = GuidanceController.clamp(turnError / fullLockBearingErrorDegrees, -1.0, 1.0)

        // Ramp up from minThrottle at the acceptance radius to cruiseThrottle
        // at the slowdown radius.
        let ramp = GuidanceController.clamp(
            (distance - radius) / (slowdownRadiusMeters - radius), 0.0, 1.0
        )
        var throttle = minThrottle + (cruiseThrottle - minThrottle) * ramp

        // Bleed a little more speed while the nose is well off the bearing so
        // we do not overshoot.
        throttle *= 1.0 - 0.35 * abs(turn)

        let left: Double
        let right: Double
        if abs(turnError) > pivotBearingErrorDegrees {
            // Pivot on the spot: right turn means left wheel forward, right wheel back.
            let pivot = max(throttle, minThrottle)
            let sign = turnError >= 0 ? 1.0 : -1.0
            left = GuidanceController.clamp(sign * pivot, -1.0, 1.0)
            right = GuidanceController.clamp(-sign * pivot, -1.0, 1.0)
        } else {
            let turnComponent = throttle * turn
            left = GuidanceController.clamp(throttle + turnComponent, -1.0, 1.0)
            right = GuidanceController.clamp(throttle - turnComponent, -1.0, 1.0)
        }

        return GuidanceResult(
            left: left, right: right,
            distanceMeters: distance, bearingDegrees: bearing,
            turnErrorDegrees: turnError, arrived: false
        )
    }

    static func clamp(_ value: Double, _ low: Double, _ high: Double) -> Double {
        max(low, min(high, value))
    }
}

// MARK: - Navigator

/// GPS mission navigation modes, the ground-robot equivalent of Mission Planner.
enum NavMode: String {
    case idle
    case mission
    case guided
    case rtl
}

/// Outcome of one navigation tick.
struct NavStatus {
    let mode: NavMode
    let left: Double
    let right: Double
    let distanceMeters: Double?
    let bearingDegrees: Double?
    let turnErrorDegrees: Double?
    let activeWaypointIndex: Int?
    let arrived: Bool
    let message: String

    var isMoving: Bool { left != 0.0 || right != 0.0 }
}

/// Runs waypoint missions against a live GPS fix.
class Navigator {
    /// Hold position rather than drive on a fix worse than this.
    static let maxPositionAccuracyMeters = 15.0

    let guidance: GuidanceController
    var mission: Mission
    private(set) var mode: NavMode = .idle
    private(set) var isPaused = false
    private(set) var activeIndex = 0
    private(set) var guidedTarget: Waypoint?
    private(set) var track: [(latitude: Double, longitude: Double)] = []
    private(set) var headingDegrees: Double?
    private(set) var lastStatus: NavStatus?

    init(mission: Mission = Mission(), guidance: GuidanceController = GuidanceController()) {
        self.mission = mission
        self.guidance = guidance
    }

    // MARK: mission bookkeeping

    var isNavigating: Bool { mode != .idle }

    /// Forgets the track driven so far (the track stays read-only from outside).
    func clearTrack() {
        track.removeAll()
    }

    /// Records the launch position RTL returns to.
    func markHome(_ latitude: Double, _ longitude: Double) {
        mission.setHome(latitude, longitude)
    }

    func loadMission(_ mission: Mission) {
        self.mission = mission
        activeIndex = 0
    }

    /// Loads a Mission Planner `.waypoints` document.
    func loadMissionFile(_ contents: String) throws -> Mission {
        let loaded = try Mission.parseWaypointsFile(contents)
        loadMission(loaded)
        return loaded
    }

    /// Serialises the current mission to the `.waypoints` format.
    func saveMissionFile() -> String {
        mission.toWaypointsFile()
    }

    /// The waypoint being sought right now, or `nil` when idle.
    func currentTarget() -> Waypoint? {
        if mode == .mission {
            return mission.waypoint(at: activeIndex)
        }
        return guidedTarget
    }

    // MARK: mode control

    /// AUTO: drive every waypoint in order. Returns false if there are none.
    @discardableResult
    func startMission() -> Bool {
        guard !mission.isEmpty else { return false }
        activeIndex = 0
        guidedTarget = nil
        mode = .mission
        isPaused = false
        return true
    }

    /// GUIDED: drive to a single target ("go to here").
    func startGuided(latitude: Double, longitude: Double, acceptanceRadiusMeters: Double = defaultAcceptanceRadiusMeters) {
        guidedTarget = Waypoint(
            latitude: latitude, longitude: longitude,
            name: "", acceptanceRadiusMeters: acceptanceRadiusMeters
        )
        mode = .guided
        isPaused = false
    }

    /// RTL: return to the recorded home position.
    @discardableResult
    func startRTL(acceptanceRadiusMeters: Double = defaultAcceptanceRadiusMeters) -> Bool {
        guard let homeLat = mission.homeLatitude, let homeLon = mission.homeLongitude else {
            return false
        }
        guidedTarget = Waypoint(
            latitude: homeLat, longitude: homeLon,
            name: "Home", acceptanceRadiusMeters: acceptanceRadiusMeters
        )
        mode = .rtl
        isPaused = false
        return true
    }

    func pause() {
        if isNavigating { isPaused = true }
    }

    func resume() {
        isPaused = false
    }

    /// Disarms navigation and returns the robot to manual/idle control.
    func stop() {
        mode = .idle
        isPaused = false
        guidedTarget = nil
        activeIndex = 0
    }

    // MARK: guidance

    /// Fuses a heading sample into the steering heading.
    ///
    /// `CLLocation.course` is only trustworthy while the device is moving, so
    /// the sample is ignored below the speed threshold and the last known
    /// heading is held instead.
    func updateHeading(_ headingDegrees: Double?, speedMetersPerSecond: Double = 0.0) {
        guard let headingDegrees = headingDegrees, speedMetersPerSecond >= 0.5 else { return }
        self.headingDegrees = GeoMath.wrapDegrees(headingDegrees)
    }

    /// Direct heading override, used by the magnetometer path and by tests.
    func setHeading(_ headingDegrees: Double?) {
        self.headingDegrees = headingDegrees.map(GeoMath.wrapDegrees)
    }

    /// Computes one control tick from the current fix.
    func step(latitude: Double, longitude: Double, accuracyMeters: Double? = nil) -> NavStatus {
        track.append((latitude, longitude))

        if !isNavigating || isPaused {
            return record(NavStatus(
                mode: mode, left: 0, right: 0,
                distanceMeters: nil, bearingDegrees: nil, turnErrorDegrees: nil,
                activeWaypointIndex: nil, arrived: false,
                message: isPaused ? "paused" : "idle"
            ))
        }

        if let accuracy = accuracyMeters, accuracy > Navigator.maxPositionAccuracyMeters {
            // Driving on a bad fix is worse than waiting: hold and say so.
            return record(NavStatus(
                mode: mode, left: 0, right: 0,
                distanceMeters: nil, bearingDegrees: nil, turnErrorDegrees: nil,
                activeWaypointIndex: activeIndex, arrived: false,
                message: String(format: "GPS accuracy %.1f m, holding", accuracy)
            ))
        }

        guard let target = currentTarget() else {
            stop()
            return record(NavStatus(
                mode: .idle, left: 0, right: 0,
                distanceMeters: nil, bearingDegrees: nil, turnErrorDegrees: nil,
                activeWaypointIndex: nil, arrived: false, message: "no target"
            ))
        }

        guard let heading = headingDegrees else {
            // No usable heading yet: hold rather than steer blind.
            return record(NavStatus(
                mode: mode, left: 0, right: 0,
                distanceMeters: target.distanceFrom(latitude, longitude),
                bearingDegrees: target.bearingFrom(latitude, longitude),
                turnErrorDegrees: nil,
                activeWaypointIndex: activeIndex, arrived: false,
                message: "acquiring heading"
            ))
        }

        let result = guidance.compute(
            latitude: latitude, longitude: longitude,
            headingDegrees: heading, target: target
        )
        let reachedIndex = activeIndex

        if result.arrived {
            onArrived()
            return record(NavStatus(
                mode: mode, left: 0, right: 0,
                distanceMeters: result.distanceMeters,
                bearingDegrees: result.bearingDegrees,
                turnErrorDegrees: result.turnErrorDegrees,
                activeWaypointIndex: reachedIndex,
                arrived: true, message: "waypoint reached"
            ))
        }

        return record(NavStatus(
            mode: mode, left: result.left, right: result.right,
            distanceMeters: result.distanceMeters,
            bearingDegrees: result.bearingDegrees,
            turnErrorDegrees: result.turnErrorDegrees,
            activeWaypointIndex: reachedIndex,
            arrived: false, message: ""
        ))
    }

    private func onArrived() {
        if mode == .mission {
            activeIndex += 1
            if activeIndex >= mission.count {
                // Last waypoint: mission complete, back to idle.
                stop()
            }
        } else {
            // GUIDED and RTL are one-shot: arriving disarms them.
            stop()
        }
    }

    @discardableResult
    private func record(_ status: NavStatus) -> NavStatus {
        lastStatus = status
        return status
    }

    // MARK: export

    /// Builds a KML document of the plan plus the track driven so far.
    func exportKml(documentName: String = "OpenBot Mission") -> String {
        KmlWriter.build(documentName: documentName, mission: mission, track: track)
    }
}

// MARK: - KML export

/// KML export for a planned mission and the driven track.
///
/// Mirrors `KmlWriter` on Android so an export from any platform drops straight
/// into Google Earth the same way.
enum KmlWriter {
    static func build(documentName: String, mission: Mission, track: [(latitude: Double, longitude: Double)]) -> String {
        var parts: [String] = [
            "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n",
            "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n",
            "  <Document>\n",
            "    <name>\(xmlEscape(documentName))</name>\n",
        ]

        if let homeLat = mission.homeLatitude, let homeLon = mission.homeLongitude {
            appendPlacemark(&parts, name: "Home", latitude: homeLat, longitude: homeLon)
        }

        for (offset, waypoint) in mission.waypoints.enumerated() {
            var label = "WP\(offset + 1)"
            if !waypoint.name.isEmpty { label += " " + waypoint.name }
            appendPlacemark(&parts, name: label, latitude: waypoint.latitude, longitude: waypoint.longitude)
        }

        if mission.waypoints.count > 1 || mission.hasHome {
            parts.append("    <Placemark>\n")
            parts.append("      <name>Mission Path</name>\n")
            parts.append("      <Style><LineStyle><color>ff00aaff</color><width>3</width></LineStyle></Style>\n")
            parts.append("      <LineString><tessellate>1</tessellate><coordinates>")
            if let homeLat = mission.homeLatitude, let homeLon = mission.homeLongitude {
                parts.append(coord(homeLat, homeLon))
            }
            for waypoint in mission.waypoints {
                parts.append(coord(waypoint.latitude, waypoint.longitude))
            }
            parts.append("</coordinates></LineString>\n")
            parts.append("    </Placemark>\n")
        }

        if !track.isEmpty {
            parts.append("    <Placemark>\n")
            parts.append("      <name>Track</name>\n")
            parts.append("      <Style><LineStyle><color>ff00ff00</color><width>3</width></LineStyle></Style>\n")
            parts.append("      <LineString><tessellate>1</tessellate><coordinates>")
            for position in track {
                parts.append(coord(position.latitude, position.longitude))
            }
            parts.append("</coordinates></LineString>\n")
            parts.append("    </Placemark>\n")
        }

        parts.append("  </Document>\n")
        parts.append("</kml>\n")
        return parts.joined()
    }

    private static func appendPlacemark(_ parts: inout [String], name: String, latitude: Double, longitude: Double) {
        parts.append("    <Placemark>\n")
        parts.append("      <name>\(xmlEscape(name))</name>\n")
        parts.append("      <Point><coordinates>\(coord(latitude, longitude))</coordinates></Point>\n")
        parts.append("    </Placemark>\n")
    }

    private static func coord(_ latitude: Double, _ longitude: Double) -> String {
        String(format: "%.8f,%.8f,0 ", longitude, latitude)
    }

    private static func xmlEscape(_ text: String) -> String {
        text
            .replacingOccurrences(of: "&", with: "&amp;")
            .replacingOccurrences(of: "<", with: "&lt;")
            .replacingOccurrences(of: ">", with: "&gt;")
            .replacingOccurrences(of: "\"", with: "&quot;")
    }
}
