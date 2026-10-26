//
//  LidarCore.swift
//  OpenBot
//
//  Pure LiDAR core: the "LDS1" scan codec, the eight proximity octants and the
//  ArduPilot Rover object-avoidance law.
//
//  Mirrors the Android `vehicle.LidarScan` / `vehicle.Proximity` /
//  `vehicle.ObjectAvoidance` classes and the Linux `lidar` package so an
//  obstacle stops the car the same way on every platform. Nothing here touches
//  UIKit or Network, so it is unit testable in isolation.
//

import Foundation

// MARK: - LidarScan

/// The four bytes every scan datagram starts with.
let lidarScanMagic = Data([0x4C, 0x44, 0x53, 0x31]) // "LDS1"

/// Bytes before the range array: magic(4) + bins(2) + scanId(4) + reserved(4).
let lidarScanHeaderLength = 14

/// Largest bin count accepted, matching the firmware's `UDP_BINS_MAX`.
let lidarScanMaxBins = 720

/// One finished LiDAR revolution.
///
/// The wire format is the firmware's (see `firmware/openbot-esp32s3/lidar_udp.h`)::
///
///     offset  size          field
///          0     4          "LDS1"
///          4     2          bin count           uint16 little-endian
///          6     4          scan id             uint32 little-endian
///         10     4          reserved, zero
///         14  2*bins        ranges, millimetres uint16 little-endian each
///
/// `0 mm` means "no return" in that bin - never "an obstacle at zero". A real
/// return at 0 mm is outside the LD14's 20 mm minimum range, so the two can
/// never be confused.
///
/// Bin `i` sits at `i * (360 / bins)` degrees clockwise from the LiDAR's own
/// zero mark. Bins are never reordered: the mounting offset is applied at read
/// time with `forwardOffsetDeg`, exactly as the firmware applies
/// `LIDAR_FORWARD_DEG` when folding octants. Reordering the bins *and* applying
/// the offset would make the two corrections fight.
struct LidarScan {
    let binCount: Int
    let scanId: UInt32
    /// All ranges in bin order, never reordered. Millimetres, 0 for "no return".
    let rangesMm: [UInt16]

    init(binCount: Int, scanId: UInt32, rangesMm: [UInt16]) {
        self.binCount = binCount
        self.scanId = scanId
        self.rangesMm = rangesMm
    }

    /// Range of one bin in millimetres, or 0 for "no return".
    func rangeMm(_ bin: Int) -> Int {
        guard bin >= 0, bin < binCount, bin < rangesMm.count else { return 0 }
        return Int(rangesMm[bin])
    }

    /// Range in metres, or `.infinity` when there is no return.
    func rangeMeters(_ bin: Int) -> Double {
        let mm = rangeMm(bin)
        return mm == 0 ? .infinity : Double(mm) / 1000.0
    }

    /// True when this bin returned a measurement.
    func hasReturn(_ bin: Int) -> Bool {
        rangeMm(bin) > 0
    }

    /// Number of bins that returned a measurement.
    var validPoints: Int {
        rangesMm.reduce(0) { $0 + ($1 > 0 ? 1 : 0) }
    }

    /// Angle of a bin, clockwise from the LiDAR's zero mark, in `[0, 360)`.
    ///
    /// Uses the bin centre (`i + 0.5`) so a 360-bin scan reports bin 0 at 0.5
    /// degrees rather than snapping it onto the zero mark - the same centre
    /// convention the firmware uses when folding octants.
    func binAngleDeg(_ bin: Int) -> Double {
        guard binCount > 0 else { return 0 }
        return (Double(bin) + 0.5) * 360.0 / Double(binCount)
    }

    /// Angle of a bin relative to the car's nose, in `[0, 360)`.
    func vehicleAngleDeg(_ bin: Int, forwardOffsetDeg: Double = 0) -> Double {
        wrapDegrees(binAngleDeg(bin) - forwardOffsetDeg)
    }

    /// Fold this revolution into `sectorCount` octants, keeping the nearest return.
    ///
    /// Sector `i` is centred on `i * (360 / sectorCount)` degrees from the nose
    /// and spans half a sector either side. This mirrors the firmware's
    /// `computeSectors()` exactly, so an octant folded here and one reported on
    /// the `p` channel describe the same geometry.
    ///
    /// Returns millimetres, 0 where the sector is clear.
    func computeSectorsMm(_ sectorCount: Int, forwardOffsetDeg: Double = 0) -> [Int] {
        let count = max(1, sectorCount)
        var out = [Int](repeating: 0, count: count)
        let step = 360.0 / Double(count)
        for bin in 0..<binCount {
            let mm = rangeMm(bin)
            if mm == 0 { continue }
            let angle = vehicleAngleDeg(bin, forwardOffsetDeg: forwardOffsetDeg)
            let sector = Int((angle + step * 0.5) / step) % count
            if out[sector] == 0 || mm < out[sector] {
                out[sector] = mm
            }
        }
        return out
    }

    /// Decode one datagram.
    ///
    /// Returns nil for anything malformed - wrong magic, truncated, a bin count
    /// of zero, or a bin count past `lidarScanMaxBins` - so a stray packet on the
    /// port can never be mistaken for a scan of open road.
    ///
    /// `length` bounds the payload when the buffer is oversized (a UDP receive
    /// hands one back); reading past it would pick up stale bytes from the
    /// previous datagram as ranges.
    static func parse(_ datagram: Data, length: Int? = nil) -> LidarScan? {
        let count = length ?? datagram.count
        guard count >= lidarScanHeaderLength else { return nil }
        guard Data(datagram.prefix(4)) == lidarScanMagic else { return nil }
        let binCount = Int(readUInt16(datagram, 4))
        guard binCount > 0, binCount <= lidarScanMaxBins else { return nil }
        guard count >= lidarScanHeaderLength + binCount * 2 else { return nil }
        let scanId = readUInt32(datagram, 6)
        var ranges = [UInt16]()
        ranges.reserveCapacity(binCount)
        for i in 0..<binCount {
            ranges.append(readUInt16(datagram, lidarScanHeaderLength + i * 2))
        }
        return LidarScan(binCount: binCount, scanId: scanId, rangesMm: ranges)
    }

    /// Encode a datagram back out. Used by the tests to round-trip the parser.
    static func encode(scanId: UInt32, rangesMm: [UInt16]) -> Data {
        var out = lidarScanMagic
        out.append(writeUInt16(UInt16(rangesMm.count)))
        out.append(writeUInt32(scanId))
        out.append(Data([0, 0, 0, 0]))
        for mm in rangesMm {
            out.append(writeUInt16(mm))
        }
        return out
    }
}

/// Degrees, wrapped to `[0, 360)`.
func wrapDegrees(_ degrees: Double) -> Double {
    let wrapped = degrees.truncatingRemainder(dividingBy: 360.0)
    return wrapped < 0 ? wrapped + 360.0 : wrapped
}

private func readUInt16(_ data: Data, _ offset: Int) -> UInt16 {
    UInt16(data[offset]) | (UInt16(data[offset + 1]) << 8)
}

private func readUInt32(_ data: Data, _ offset: Int) -> UInt32 {
    UInt32(data[offset])
        | (UInt32(data[offset + 1]) << 8)
        | (UInt32(data[offset + 2]) << 16)
        | (UInt32(data[offset + 3]) << 24)
}

private func writeUInt16(_ value: UInt16) -> Data {
    Data([UInt8(value & 0xFF), UInt8((value >> 8) & 0xFF)])
}

private func writeUInt32(_ value: UInt32) -> Data {
    Data([
        UInt8(value & 0xFF),
        UInt8((value >> 8) & 0xFF),
        UInt8((value >> 16) & 0xFF),
        UInt8((value >> 24) & 0xFF),
    ])
}

// MARK: - Proximity

/// Number of proximity octants the car reports on the OpenBot `p` channel.
let proximitySectors = 8

/// How old the last report may be before it is considered stale.
let proximityStaleAfterMs = 500.0

/// Age reported before any reading has arrived.
let proximityNeverFedAgeMs = 1000.0

/// The eight proximity octants the car reports on the OpenBot `p` channel.
///
/// The layout is the MAVLink `DISTANCE_SENSOR` octant arrangement that
/// ArduPilot's proximity library uses, and the same one the firmware's
/// `PROX_SECTORS` folding produces: sector 0 is straight ahead and the numbering
/// goes clockwise, one sector every 45 degrees::
///
///            7   0   1        0 = front        4 = back
///          6   +   2          1 = front-right  5 = back-left
///            5   4   3        2 = right        6 = left
///                              3 = back-right   7 = front-left
///
/// **Units and the zero convention.** The wire carries centimetres to match the
/// `s` sonar channel, and the firmware spells "nothing measured in this sector"
/// as `0`. This type converts on ingest: metres internally, and a 0 becomes
/// `.infinity` so an empty sector can never be mistaken for an obstacle sitting
/// on top of the robot. That is the *opposite* of the `s` channel, where 0 means
/// stale and the car stops - on `p`, staleness is signalled by the report simply
/// not arriving, which is what `dataState` watches for.
final class Proximity {
    private var sectorsM = [Double](repeating: .infinity, count: proximitySectors)
    private var timestampMs: Double = 0
    private let clock: () -> Double

    /// Injectable clock so the tests can age the reading without sleeping.
    init(clock: @escaping () -> Double = monotonicMs) {
        self.clock = clock
    }

    /// Ingest one `p` report. `sectorCm` is eight octant distances in centimetres;
    /// `0` means "no return" and is stored as infinite.
    ///
    /// Returns true if the report was accepted. A nil, short or otherwise
    /// unusable list is rejected and the previous reading is left alone rather
    /// than being replaced with zeros that look like clear road.
    @discardableResult
    func setSectorsCm(_ sectorCm: [Double]?) -> Bool {
        guard let sectorCm, sectorCm.count == proximitySectors else { return false }
        for cm in sectorCm {
            guard cm.isFinite, cm >= 0 else { return false }
        }
        for (i, cm) in sectorCm.enumerated() {
            // 0 cm on the wire means "nothing measured here", not "an obstacle at
            // zero". A real return at 0 is outside the LD14's 20 mm minimum
            // anyway, so the two cannot be confused.
            sectorsM[i] = cm == 0 ? .infinity : cm / 100.0
        }
        timestampMs = clock()
        return true
    }

    /// The eight octant distances in metres, clear sectors as `.infinity`.
    func sectors() -> [Double] {
        sectorsM
    }

    /// Distance of one sector in metres, or `.infinity` when there is no return.
    func sector(_ index: Int) -> Double {
        guard index >= 0, index < proximitySectors else { return .infinity }
        return sectorsM[index]
    }

    /// Milliseconds since the last accepted report, or `neverFedAgeMs` before it.
    func ageMs() -> Double {
        timestampMs == 0 ? proximityNeverFedAgeMs : clock() - timestampMs
    }

    /// True once at least one report has been ingested.
    func hasEverReported() -> Bool {
        timestampMs > 0
    }

    /// How fresh the picture is: "never", "stale" or "fresh".
    ///
    /// "never" never blocks driving - a robot with no LiDAR must stay drivable.
    /// "stale" blocks forward motion - a LiDAR that died mid-drive must not
    /// leave the robot driving blind. "fresh" can be acted on.
    func dataState() -> String {
        if timestampMs == 0 { return "never" }
        return ageMs() > proximityStaleAfterMs ? "stale" : "fresh"
    }

    /// Reset to "nothing has ever been measured".
    func reset() {
        sectorsM = [Double](repeating: .infinity, count: proximitySectors)
        timestampMs = 0
    }

    /// Parse the body of a `p` message: eight comma-separated distances in cm.
    ///
    /// Returns nil for anything malformed, so a truncated BLE notification
    /// cannot be mistaken for a scan of eight clear sectors.
    static func parseSectorsCm(_ body: String) -> [Double]? {
        let parts = body.split(separator: ",", omittingEmptySubsequences: false)
        guard parts.count == proximitySectors else { return nil }
        var out = [Double]()
        out.reserveCapacity(proximitySectors)
        for part in parts {
            guard let value = Double(part.trimmingCharacters(in: .whitespaces)) else {
                return nil
            }
            out.append(value)
        }
        return out
    }
}

// MARK: - Object avoidance

/// Sector indices, clockwise from the front. See `Proximity`.
enum ProximitySector: Int, CaseIterable {
    case front = 0
    case frontRight = 1
    case right = 2
    case backRight = 3
    case back = 4
    case backLeft = 5
    case left = 6
    case frontLeft = 7
}

/// The front arc checked against forward motion: front-left, front, front-right.
///
/// A 135 degree sweep - a finer-grained version of the forward quadrant
/// ArduPilot checks, and it covers the corners an Ackermann car swings through
/// as it steers.
let frontArc: [ProximitySector] = [.frontLeft, .front, .frontRight]

/// The rear arc checked against reverse motion, the mirror of `frontArc`.
let rearArc: [ProximitySector] = [.backLeft, .back, .backRight]

/// What the object-avoidance law decided.
///
/// `left` and `right` are the commands to actually drive, in `[-1, 1]`.
struct AvoidanceDecision {
    let left: Float
    let right: Float
    /// True when an obstacle vetoed motion the driver or autopilot actually asked for.
    let blocked: Bool
    /// True when the robot is actively reversing to restore the margin.
    let backing: Bool
    /// Sector index of the nearest obstacle in the checked arc, or -1 when clear.
    let nearestSector: Int
    /// Distance to that obstacle in metres, or `.infinity` when clear.
    let nearestMeters: Double

    /// True when this decision is holding the robot back, stopped or backing.
    var isHolding: Bool {
        blocked || backing
    }

    static let passthrough = AvoidanceDecision(
        left: 0, right: 0, blocked: false, backing: false,
        nearestSector: -1, nearestMeters: .infinity
    )
}

/// ArduPilot Rover-style Simple Object Avoidance.
///
/// The model follows ArduPilot's *Rover* behaviour rather than Copter's:
///
/// - **Rover always stops.** Copter's `AVOID_BEHAVE` (Stop vs Slide) has no Rover
///   equivalent - there is no sliding sideways past an obstacle on a car.
/// - **Applies in every mode except manual.** ArduPilot applies this in the
///   firmware's navigation stack so no flight mode escapes it. Here the
///   equivalent chokepoint is the caller's drive path.
/// - **`AVOID_MARGIN`** is the standoff at which the vehicle stops, in metres.
/// - **`AVOID_BACKUP_SPD`** actively backs away to restore the margin. `0`
///   disables it, matching ArduPilot's default of 0.
/// - **"SAFETY FIRST!"** ArduPilot insists on a quick-disable switch
///   (`RCx_OPTION = 40`) before first use. That is the `enabled` argument here.
///
/// Two deliberate deviations from ArduPilot, both toward stopping rather than
/// driving:
///
/// - A **stale** sensor (reports arrived, then stopped) blocks forward motion.
///   ArduPilot simply fails to avoid when it has no data; for a robot with no
///   other ranging that means driving blind, which is the one outcome the
///   feature exists to prevent. Reverse stays available so the driver can back
///   out. A sensor that has **never** reported is treated as "not fitted" and
///   never blocks - otherwise a robot without a LiDAR would be undrivable.
/// - Blocking zeroes the throttle but **keeps the steering**, so the wheels stay
///   pointed where the driver put them while the robot is held.
enum ObjectAvoidance {
    static func evaluate(
        left: Float,
        right: Float,
        sectorMeters: [Double],
        state: String,
        marginMeters: Double,
        backupSpeed: Double,
        enabled: Bool
    ) -> AvoidanceDecision {
        let l = clampControl(left)
        let r = clampControl(right)

        if !enabled || marginMeters <= 0 || state == "never" {
            return AvoidanceDecision(
                left: l, right: r, blocked: false, backing: false,
                nearestSector: -1, nearestMeters: .infinity
            )
        }

        if state == "stale" {
            // The last picture we had is too old to trust. A frozen "clear road"
            // is exactly the reading that drives a robot into a wall, so block
            // the direction we can no longer see. Reverse stays open - backing
            // out of trouble is what the driver wants.
            if throttle(l, r) > 0 {
                return stopKeepingSteering(l, r, nearestSector: -1, nearestMeters: .infinity)
            }
            return AvoidanceDecision(
                left: l, right: r, blocked: false, backing: false,
                nearestSector: -1, nearestMeters: .infinity
            )
        }

        let t = throttle(l, r)
        if t > 0 {
            let obstacle = nearest(sectorMeters, arc: frontArc)
            if obstacle.meters < marginMeters {
                if backupSpeed > 0 {
                    let rear = nearest(sectorMeters, arc: rearArc)
                    if rear.meters >= marginMeters {
                        // ArduPilot's AVOID_BACKUP_SPD: actively back away to
                        // restore the margin rather than sitting against the
                        // barrier. Only when the way behind is clear - backing
                        // into a wall to get away from a wall is not an
                        // improvement.
                        let b = clampControl(Float(backupSpeed))
                        return AvoidanceDecision(
                            left: -b, right: -b, blocked: true, backing: true,
                            nearestSector: obstacle.sector, nearestMeters: obstacle.meters
                        )
                    }
                }
                return stopKeepingSteering(
                    l, r, nearestSector: obstacle.sector, nearestMeters: obstacle.meters
                )
            }
        } else if t < 0 {
            let obstacle = nearest(sectorMeters, arc: rearArc)
            if obstacle.meters < marginMeters {
                return stopKeepingSteering(
                    l, r, nearestSector: obstacle.sector, nearestMeters: obstacle.meters
                )
            }
        }

        return AvoidanceDecision(
            left: l, right: r, blocked: false, backing: false,
            nearestSector: -1, nearestMeters: .infinity
        )
    }

    /// Throttle of a tank-style pair: the same mix the firmware uses to drive the ESC.
    private static func throttle(_ left: Float, _ right: Float) -> Float {
        left + right
    }

    /// Zero the throttle but keep the steering angle the driver asked for.
    ///
    /// With throttle 0 and `steering = left - right` preserved, the two tracks
    /// become `+/- steering/2` - the wheels stay pointed, the robot does not move.
    private static func stopKeepingSteering(
        _ left: Float, _ right: Float, nearestSector: Int, nearestMeters: Double
    ) -> AvoidanceDecision {
        let steering = left - right
        let half = clampControl(steering / 2.0)
        return AvoidanceDecision(
            left: half, right: -half, blocked: true, backing: false,
            nearestSector: nearestSector, nearestMeters: nearestMeters
        )
    }

    /// Nearest return across the given sectors.
    ///
    /// A sector with no return is infinite here, so a clear arc never looks like
    /// an obstacle sitting at zero.
    private static func nearest(
        _ sectorMeters: [Double], arc: [ProximitySector]
    ) -> (sector: Int, meters: Double) {
        var bestSector = -1
        var best = Double.infinity
        guard sectorMeters.count >= proximitySectors else { return (bestSector, best) }
        for sector in arc {
            let index = sector.rawValue
            guard index >= 0, index < sectorMeters.count else { continue }
            let d = sectorMeters[index]
            guard d > 0, d.isFinite else { continue }
            if d < best {
                best = d
                bestSector = index
            }
        }
        return (bestSector, best)
    }

    static func clampControl(_ value: Float) -> Float {
        guard value.isFinite else { return 0 }
        return max(-1, min(1, value))
    }
}

// MARK: - DriveGate

/// The single chokepoint every drive command passes through.
///
/// ArduPilot applies Simple Object Avoidance inside the navigation stack so no
/// flight mode escapes it; on this robot the equivalent is the one place every
/// caller already uses to reach the motors.
///
/// `control` is what the driver or autopilot asked for; `lastDecision` is what
/// actually went to the wheels after the law had its say. Keeping both is what
/// lets a cleared blockage release without the driver re-moving the stick:
/// every new proximity report re-evaluates the stored request.
final class DriveGate {
    let proximity: Proximity
    var marginMeters: Double
    var backupSpeed: Double
    /// The quick-disable switch, ArduPilot's `RCx_OPTION = 40`.
    var enabled: Bool

    /// The last request the driver or autopilot made, before avoidance.
    private(set) var control: (left: Float, right: Float) = (0, 0)
    private(set) var lastDecision = AvoidanceDecision.passthrough

    init(
        proximity: Proximity = Proximity(),
        marginMeters: Double = 1.0,
        backupSpeed: Double = 0,
        enabled: Bool = true
    ) {
        self.proximity = proximity
        self.marginMeters = marginMeters
        self.backupSpeed = backupSpeed
        self.enabled = enabled
    }

    /// Run one control request through the law and return what to actually drive.
    @discardableResult
    func apply(left: Float, right: Float) -> AvoidanceDecision {
        control = (left, right)
        lastDecision = ObjectAvoidance.evaluate(
            left: left,
            right: right,
            sectorMeters: proximity.sectors(),
            state: proximity.dataState(),
            marginMeters: marginMeters,
            backupSpeed: backupSpeed,
            enabled: enabled
        )
        return lastDecision
    }

    /// Toggle the quick-disable switch and re-evaluate the stored request.
    ///
    /// Returns the new decision so a caller can re-send immediately without
    /// waiting for the next stick move.
    @discardableResult
    func setEnabled(_ enabled: Bool) -> AvoidanceDecision {
        self.enabled = enabled
        return apply(left: control.left, right: control.right)
    }

    /// Re-evaluate after a new `p` report.
    ///
    /// Called whenever proximity data changes so a cleared blockage releases the
    /// robot without the driver re-moving the stick.
    @discardableResult
    func onProximityUpdate() -> AvoidanceDecision {
        apply(left: control.left, right: control.right)
    }

    /// Ingest one revolution folded into octants and re-evaluate.
    @discardableResult
    func update(from scan: LidarScan, forwardOffsetDeg: Double = 0) -> AvoidanceDecision? {
        let sectorsMm = scan.computeSectorsMm(proximitySectors, forwardOffsetDeg: forwardOffsetDeg)
        let sectorsCm = sectorsMm.map { Double($0) / 10.0 }
        guard proximity.setSectorsCm(sectorsCm) else { return nil }
        return onProximityUpdate()
    }

    /// One-line status for a HUD, matching the Android app's wording.
    func statusText() -> String {
        if !enabled { return "Off" }
        if !proximity.hasEverReported() { return "Idle" }
        if proximity.dataState() == "stale" { return "Blind" }
        if lastDecision.backing { return "Backing" }
        if lastDecision.blocked { return "Holding" }
        return "Clear"
    }

    /// Apply the law to an OpenBot wire command `c<left>,<right>`.
    ///
    /// Only `c` payloads are touched; indicators, lights and everything else
    /// pass through untouched. Values on the wire are integers in `[-255, 255]`
    /// (the firmware's PWM scale), so they are normalised to `[-1, 1]` for the
    /// law and scaled back out afterwards. Returns the rewritten payload, or
    /// the original when it is not a drive command.
    func applyWire(_ payload: String, scale: Double = 255) -> String {
        guard payload.first == "c", scale > 0 else { return payload }
        let body = payload.dropFirst()
        let trimmed = body.trimmingCharacters(in: .whitespacesAndNewlines)
        let parts = trimmed.split(separator: ",")
        guard parts.count == 2,
              let leftValue = Double(parts[0]),
              let rightValue = Double(parts[1]
                  .trimmingCharacters(in: CharacterSet(charactersIn: "\r\n")))
        else {
            return payload
        }
        let decision = apply(
            left: Float(leftValue / scale),
            right: Float(rightValue / scale)
        )
        let leftOut = Int((decision.left * Float(scale)).rounded())
        let rightOut = Int((decision.right * Float(scale)).rounded())
        return "c\(leftOut),\(rightOut)\n"
    }
}

// MARK: - Clock

/// Milliseconds on a clock that never jumps. Wall-clock time (`Date()`) can step
/// backwards or forwards when the phone syncs its time, which would make a stale
/// LiDAR look fresh - or a fresh one stale - for however long the step was.
func monotonicMs() -> Double {
    ProcessInfo.processInfo.systemUptime * 1000.0
}

// MARK: - Persisted settings

/// The three avoidance knobs, persisted so they survive a relaunch.
///
/// Same names and meaning as ArduPilot: `AVOID_MARGIN` is the standoff in
/// metres, `AVOID_BACKUP_SPD` the reverse command used to back away (0 turns it
/// off) and `enabled` the quick-disable switch (`RCx_OPTION = 40`).
struct AvoidanceSettings: Equatable {
    static let marginRange: ClosedRange<Double> = 0.3...3.0
    static let backupRange: ClosedRange<Double> = 0.0...0.5

    var enabled: Bool
    var marginMeters: Double
    var backupSpeed: Double

    static let defaults = AvoidanceSettings(enabled: true, marginMeters: 1.0, backupSpeed: 0)

    private static let enabledKey = "objectAvoidanceEnabled"
    private static let marginKey = "objectAvoidanceMargin"
    private static let backupKey = "objectAvoidanceBackup"

    static func load(from defaults: UserDefaults = .standard) -> AvoidanceSettings {
        var settings = AvoidanceSettings.defaults
        if defaults.object(forKey: enabledKey) != nil {
            settings.enabled = defaults.bool(forKey: enabledKey)
        }
        if defaults.object(forKey: marginKey) != nil {
            settings.marginMeters = defaults.double(forKey: marginKey)
        }
        if defaults.object(forKey: backupKey) != nil {
            settings.backupSpeed = defaults.double(forKey: backupKey)
        }
        return settings.clamped()
    }

    func save(to defaults: UserDefaults = .standard) {
        let value = clamped()
        defaults.set(value.enabled, forKey: AvoidanceSettings.enabledKey)
        defaults.set(value.marginMeters, forKey: AvoidanceSettings.marginKey)
        defaults.set(value.backupSpeed, forKey: AvoidanceSettings.backupKey)
    }

    /// Values pulled back into the ranges the settings screen offers, so a
    /// hand-edited or corrupt preference can never disable the law by accident.
    func clamped() -> AvoidanceSettings {
        AvoidanceSettings(
            enabled: enabled,
            marginMeters: min(max(marginMeters.isFinite ? marginMeters : 1.0,
                                  AvoidanceSettings.marginRange.lowerBound),
                              AvoidanceSettings.marginRange.upperBound),
            backupSpeed: min(max(backupSpeed.isFinite ? backupSpeed : 0,
                                 AvoidanceSettings.backupRange.lowerBound),
                             AvoidanceSettings.backupRange.upperBound)
        )
    }
}

extension DriveGate {
    /// The gate's current knobs as a settings value.
    var settings: AvoidanceSettings {
        AvoidanceSettings(enabled: enabled, marginMeters: marginMeters, backupSpeed: backupSpeed)
    }

    /// Apply new knobs and re-evaluate the stored request.
    ///
    /// Returns the new decision so the caller can re-send immediately - a car
    /// held at a barrier resumes the moment the switch goes off, and stops the
    /// moment it goes on, without waiting for the next stick move.
    @discardableResult
    func apply(settings: AvoidanceSettings) -> AvoidanceDecision {
        let value = settings.clamped()
        enabled = value.enabled
        marginMeters = value.marginMeters
        backupSpeed = value.backupSpeed
        return onProximityUpdate()
    }
}
