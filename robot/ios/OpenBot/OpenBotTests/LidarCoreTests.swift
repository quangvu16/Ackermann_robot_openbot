//
//  LidarCoreTests.swift
//  OpenBotTests
//
//  Unit tests for the LiDAR scan codec, proximity octants and object avoidance.
//  Pure logic, no hardware and no display needed. Mirrors the Android
//  `LidarScanTest` and the Linux `tests/test_lidar.py` suite so the geometry and
//  the stop law behave identically on every platform.
//
//  Staleness is tested with an injected clock rather than `sleep`, so the suite
//  stays fast and cannot flake on a loaded machine.
//

import XCTest
@testable import OpenBot

final class LidarCoreTests: XCTestCase {
    /// A clock the tests can advance without sleeping.
    private final class FakeClock {
        var now: Double
        init(start: Double = 1000) {
            now = start
        }
        func call() -> Double { now }
        func advance(_ ms: Double) { now += ms }
    }

    private func makeScan(_ ranges: [UInt16], scanId: UInt32 = 1) -> LidarScan {
        let datagram = LidarScan.encode(scanId: scanId, rangesMm: ranges)
        return LidarScan.parse(datagram)!
    }

    // MARK: - Scan codec

    func testRoundTripPreservesBinsAndId() {
        let ranges: [UInt16] = [100, 200, 300, 0, 500]
        let datagram = LidarScan.encode(scanId: 42, rangesMm: ranges)
        let scan = LidarScan.parse(datagram)
        XCTAssertNotNil(scan)
        XCTAssertEqual(scan!.binCount, 5)
        XCTAssertEqual(scan!.scanId, 42)
        XCTAssertEqual(scan!.rangesMm, ranges)
    }

    func testRejectsEmptyDatagram() {
        XCTAssertNil(LidarScan.parse(Data()))
    }

    func testRejectsShortDatagram() {
        XCTAssertNil(LidarScan.parse(Data("LDS1".utf8)))
    }

    func testRejectsWrongMagic() {
        var datagram = LidarScan.encode(scanId: 1, rangesMm: [100, 200])
        datagram[0] = 0x58 // "X"
        XCTAssertNil(LidarScan.parse(datagram))
    }

    func testRejectsZeroBins() {
        var datagram = Data("LDS1".utf8)
        datagram.append(contentsOf: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        XCTAssertNil(LidarScan.parse(datagram))
    }

    func testRejectsBinCountPastMax() {
        var datagram = LidarScan.encode(scanId: 1, rangesMm: [1, 1, 1, 1])
        let over = UInt16(lidarScanMaxBins + 1)
        datagram[4] = UInt8(over & 0xFF)
        datagram[5] = UInt8((over >> 8) & 0xFF)
        XCTAssertNil(LidarScan.parse(datagram))
    }

    func testRejectsTruncatedRanges() {
        let full = LidarScan.encode(scanId: 1, rangesMm: [100, 200, 300])
        XCTAssertNil(LidarScan.parse(full.prefix(lidarScanHeaderLength + 2)))
    }

    func testUsesLengthNotBufferCapacity() {
        // An oversized receive buffer must not turn stale bytes into ranges.
        let full = LidarScan.encode(scanId: 1, rangesMm: [100, 200, 300, 400])
        var padded = full
        padded.append(Data(repeating: 0xFF, count: 64))
        let scan = LidarScan.parse(padded, length: full.count)
        XCTAssertNotNil(scan)
        XCTAssertEqual(scan!.rangesMm, [100, 200, 300, 400])
    }

    func testAcceptsMaxBins() {
        let ranges = [UInt16](repeating: 1, count: lidarScanMaxBins)
        let scan = LidarScan.parse(LidarScan.encode(scanId: 1, rangesMm: ranges))
        XCTAssertNotNil(scan)
        XCTAssertEqual(scan!.binCount, lidarScanMaxBins)
    }

    func testFull16BitRange() {
        let scan = makeScan([0xFFFF, 1])
        XCTAssertEqual(scan.rangeMm(0), 0xFFFF)
    }

    // MARK: - Scan geometry

    func testZeroMeansNoReturnNotObstacleAtZero() {
        let scan = makeScan([0, 500])
        XCTAssertFalse(scan.hasReturn(0))
        XCTAssertEqual(scan.rangeMeters(0), .infinity)
        XCTAssertEqual(scan.rangeMeters(1), 0.5, accuracy: 1e-9)
        XCTAssertEqual(scan.validPoints, 1)
    }

    func testOutOfRangeBinIsSafe() {
        let scan = makeScan([100])
        XCTAssertEqual(scan.rangeMm(-1), 0)
        XCTAssertEqual(scan.rangeMm(1), 0)
        XCTAssertEqual(scan.rangeMeters(5), .infinity)
    }

    func testBinAnglesAreCentred() {
        let scan = makeScan([UInt16](repeating: 0, count: 360))
        XCTAssertEqual(scan.binAngleDeg(0), 0.5, accuracy: 1e-9)
        XCTAssertEqual(scan.binAngleDeg(180), 180.5, accuracy: 1e-9)
        XCTAssertEqual(scan.binAngleDeg(359), 359.5, accuracy: 1e-9)
    }

    func testForwardOffsetRotatesVehicleAngle() {
        let scan = makeScan([UInt16](repeating: 0, count: 360))
        XCTAssertEqual(scan.vehicleAngleDeg(0, forwardOffsetDeg: 0), 0.5, accuracy: 1e-9)
        XCTAssertEqual(scan.vehicleAngleDeg(0, forwardOffsetDeg: 10), 350.5, accuracy: 1e-9)
        XCTAssertEqual(scan.vehicleAngleDeg(10, forwardOffsetDeg: 10), 0.5, accuracy: 1e-9)
    }

    func testWrapDegrees() {
        XCTAssertEqual(wrapDegrees(0), 0, accuracy: 1e-9)
        XCTAssertEqual(wrapDegrees(360), 0, accuracy: 1e-9)
        XCTAssertEqual(wrapDegrees(-10), 350, accuracy: 1e-9)
        XCTAssertEqual(wrapDegrees(370), 10, accuracy: 1e-9)
    }

    func testSectorFoldingPicksNearest() {
        // Bin 0 at 0.5 deg and bin 1 at 1.5 deg are both in the front octant;
        // bin 90 at 90.5 deg lands in the right octant (centred on 90 deg).
        var ranges = [UInt16](repeating: 0, count: 360)
        ranges[0] = 2000
        ranges[1] = 1500
        ranges[90] = 3000
        let sectors = makeScan(ranges).computeSectorsMm(8)
        XCTAssertEqual(sectors[ProximitySector.front.rawValue], 1500)
        XCTAssertEqual(sectors[ProximitySector.right.rawValue], 3000)
        XCTAssertEqual(sectors[ProximitySector.back.rawValue], 0)
    }

    func testSectorFoldingHonoursOffset() {
        var ranges = [UInt16](repeating: 0, count: 360)
        ranges[0] = 1000 // 0.5 deg on the LiDAR, 90.5 deg on the car after offset
        let sectors = makeScan(ranges).computeSectorsMm(8, forwardOffsetDeg: -90)
        XCTAssertEqual(sectors[ProximitySector.right.rawValue], 1000)
        XCTAssertEqual(sectors[ProximitySector.front.rawValue], 0)
    }

    func testSectorFoldingKeepsEmptySectorsAtZero() {
        let scan = makeScan([UInt16](repeating: 0, count: 360))
        XCTAssertEqual(scan.computeSectorsMm(8), [Int](repeating: 0, count: 8))
    }

    func testSectorFoldingAcceptsFewerSectors() {
        var ranges = [UInt16](repeating: 0, count: 360)
        ranges[0] = 800
        XCTAssertEqual(makeScan(ranges).computeSectorsMm(4), [800, 0, 0, 0])
    }

    func testSectorFoldingNeverReturnsNull() {
        let scan = makeScan([UInt16](repeating: 0, count: 4))
        XCTAssertEqual(scan.computeSectorsMm(8), [Int](repeating: 0, count: 8))
        XCTAssertEqual(scan.computeSectorsMm(0), [0]) // floored to 1
    }

    // MARK: - Proximity

    func testParseSectorsCm() {
        XCTAssertEqual(
            Proximity.parseSectorsCm("10,20,30,40,50,60,70,80"),
            [10, 20, 30, 40, 50, 60, 70, 80]
        )
        XCTAssertNil(Proximity.parseSectorsCm("1,2,3"))
        XCTAssertNil(Proximity.parseSectorsCm("1,2,3,4,5,6,7,8,9"))
        XCTAssertNil(Proximity.parseSectorsCm("1,2,3,4,5,6,7,x"))
        XCTAssertNil(Proximity.parseSectorsCm(""))
    }

    func testZeroCmIsClearNotObstacleAtZero() {
        let proximity = Proximity()
        XCTAssertTrue(proximity.setSectorsCm([Double](repeating: 0, count: 8)))
        XCTAssertEqual(proximity.sectors(), [Double](repeating: .infinity, count: 8))
        XCTAssertEqual(proximity.sector(ProximitySector.front.rawValue), .infinity)
    }

    func testConvertsCmToM() {
        let proximity = Proximity()
        proximity.setSectorsCm([100, 0, 250, 0, 0, 0, 0, 0])
        XCTAssertEqual(proximity.sector(ProximitySector.front.rawValue), 1.0, accuracy: 1e-9)
        XCTAssertEqual(proximity.sector(1), .infinity)
        XCTAssertEqual(proximity.sector(2), 2.5, accuracy: 1e-9)
    }

    func testRejectsWrongArityWithoutClobbering() {
        let proximity = Proximity()
        proximity.setSectorsCm([Double](repeating: 100, count: 8))
        XCTAssertFalse(proximity.setSectorsCm([1, 2, 3]))
        XCTAssertEqual(proximity.sector(ProximitySector.front.rawValue), 1.0, accuracy: 1e-9)
    }

    func testRejectsNegative() {
        let proximity = Proximity()
        XCTAssertFalse(proximity.setSectorsCm([-1, 0, 0, 0, 0, 0, 0, 0]))
    }

    func testStateIsNeverBeforeFirstReport() {
        let proximity = Proximity()
        XCTAssertFalse(proximity.hasEverReported())
        XCTAssertEqual(proximity.dataState(), "never")
        XCTAssertEqual(proximity.ageMs(), proximityNeverFedAgeMs, accuracy: 1e-9)
    }

    func testStateGoesStaleWhenReportsStop() {
        let clock = FakeClock()
        let proximity = Proximity { clock.call() }
        proximity.setSectorsCm([Double](repeating: 100, count: 8))
        XCTAssertEqual(proximity.dataState(), "fresh")
        clock.advance(499)
        XCTAssertEqual(proximity.dataState(), "fresh")
        clock.advance(2)
        XCTAssertEqual(proximity.dataState(), "stale")
        XCTAssertTrue(proximity.hasEverReported())
    }

    func testAgeIsTimeSinceLastReportNotGap() {
        // A dead sensor must not look permanently fresh.
        let clock = FakeClock()
        let proximity = Proximity { clock.call() }
        proximity.setSectorsCm([Double](repeating: 100, count: 8))
        clock.advance(100)
        proximity.setSectorsCm([Double](repeating: 100, count: 8))
        clock.advance(600)
        XCTAssertEqual(proximity.ageMs(), 600, accuracy: 1e-9)
        XCTAssertEqual(proximity.dataState(), "stale")
    }

    func testResetReturnsToNever() {
        let proximity = Proximity()
        proximity.setSectorsCm([Double](repeating: 100, count: 8))
        proximity.reset()
        XCTAssertEqual(proximity.dataState(), "never")
        XCTAssertEqual(proximity.sectors(), [Double](repeating: .infinity, count: 8))
    }

    func testOutOfRangeSectorIsClear() {
        let proximity = Proximity()
        XCTAssertEqual(proximity.sector(-1), .infinity)
        XCTAssertEqual(proximity.sector(8), .infinity)
    }

    // MARK: - Object avoidance

    private func evaluate(
        _ left: Float, _ right: Float,
        sectors: [Double], state: String,
        margin: Double = 1.0, backup: Double = 0, enabled: Bool = true
    ) -> AvoidanceDecision {
        ObjectAvoidance.evaluate(
            left: left, right: right, sectorMeters: sectors, state: state,
            marginMeters: margin, backupSpeed: backup, enabled: enabled
        )
    }

    func testClearRoadPassesThrough() {
        let d = evaluate(0.5, 0.5, sectors: [Double](repeating: .infinity, count: 8), state: "fresh")
        XCTAssertEqual(d.left, 0.5, accuracy: 1e-6)
        XCTAssertEqual(d.right, 0.5, accuracy: 1e-6)
        XCTAssertFalse(d.blocked)
        XCTAssertFalse(d.backing)
        XCTAssertFalse(d.isHolding)
    }

    func testDisabledPassesThrough() {
        var sectors = [Double](repeating: .infinity, count: 8)
        sectors[ProximitySector.front.rawValue] = 0.2
        let d = evaluate(0.5, 0.5, sectors: sectors, state: "fresh", enabled: false)
        XCTAssertEqual(d.left, 0.5, accuracy: 1e-6)
        XCTAssertFalse(d.blocked)
    }

    func testZeroMarginPassesThrough() {
        var sectors = [Double](repeating: .infinity, count: 8)
        sectors[ProximitySector.front.rawValue] = 0.2
        let d = evaluate(0.5, 0.5, sectors: sectors, state: "fresh", margin: 0)
        XCTAssertFalse(d.blocked)
    }

    func testNeverFittedNeverBlocks() {
        // A robot with no LiDAR must stay drivable.
        let d = evaluate(
            0.5, 0.5, sectors: [Double](repeating: .infinity, count: 8), state: "never"
        )
        XCTAssertEqual(d.left, 0.5, accuracy: 1e-6)
        XCTAssertFalse(d.blocked)
    }

    func testStaleBlocksForwardKeepsReverse() {
        let clear = [Double](repeating: .infinity, count: 8)
        let forward = evaluate(0.5, 0.5, sectors: clear, state: "stale")
        XCTAssertTrue(forward.blocked)
        XCTAssertFalse(forward.backing)
        XCTAssertTrue(forward.isHolding)

        let reverse = evaluate(-0.5, -0.5, sectors: clear, state: "stale")
        XCTAssertFalse(reverse.blocked)
        XCTAssertEqual(reverse.left, -0.5, accuracy: 1e-6)
    }

    func testFrontObstacleBlocksForward() {
        var sectors = [Double](repeating: .infinity, count: 8)
        sectors[ProximitySector.front.rawValue] = 0.4
        let d = evaluate(0.5, 0.5, sectors: sectors, state: "fresh")
        XCTAssertTrue(d.blocked)
        XCTAssertFalse(d.backing)
        XCTAssertEqual(d.nearestSector, ProximitySector.front.rawValue)
        XCTAssertEqual(d.nearestMeters, 0.4, accuracy: 1e-9)
    }

    func testFrontArcCoversCorners() {
        for sector in [ProximitySector.frontLeft, ProximitySector.frontRight] {
            var sectors = [Double](repeating: .infinity, count: 8)
            sectors[sector.rawValue] = 0.3
            let d = evaluate(0.5, 0.5, sectors: sectors, state: "fresh")
            XCTAssertTrue(d.blocked, "sector \(sector) should block forward motion")
        }
    }

    func testRearObstacleBlocksReverse() {
        var sectors = [Double](repeating: .infinity, count: 8)
        sectors[ProximitySector.back.rawValue] = 0.3
        let d = evaluate(-0.5, -0.5, sectors: sectors, state: "fresh")
        XCTAssertTrue(d.blocked)
        XCTAssertEqual(d.nearestSector, ProximitySector.back.rawValue)
    }

    func testRearObstacleDoesNotBlockForward() {
        var sectors = [Double](repeating: .infinity, count: 8)
        sectors[ProximitySector.back.rawValue] = 0.3
        let d = evaluate(0.5, 0.5, sectors: sectors, state: "fresh")
        XCTAssertFalse(d.blocked)
    }

    func testBlockingKeepsSteering() {
        var sectors = [Double](repeating: .infinity, count: 8)
        sectors[ProximitySector.front.rawValue] = 0.2
        let d = evaluate(0.8, 0.2, sectors: sectors, state: "fresh")
        XCTAssertTrue(d.blocked)
        // Throttle zeroed, steering (left-right) preserved as +/- half.
        XCTAssertEqual(d.left, 0.3, accuracy: 1e-6)
        XCTAssertEqual(d.right, -0.3, accuracy: 1e-6)
    }

    func testBacksAwayWhenEnabledAndRearClear() {
        var sectors = [Double](repeating: .infinity, count: 8)
        sectors[ProximitySector.front.rawValue] = 0.3
        let d = evaluate(0.5, 0.5, sectors: sectors, state: "fresh", backup: 0.4)
        XCTAssertTrue(d.blocked)
        XCTAssertTrue(d.backing)
        XCTAssertEqual(d.left, -0.4, accuracy: 1e-6)
        XCTAssertEqual(d.right, -0.4, accuracy: 1e-6)
    }

    func testDoesNotBackIntoAWall() {
        var sectors = [Double](repeating: .infinity, count: 8)
        sectors[ProximitySector.front.rawValue] = 0.3
        sectors[ProximitySector.back.rawValue] = 0.3
        let d = evaluate(0.5, 0.5, sectors: sectors, state: "fresh", backup: 0.4)
        XCTAssertTrue(d.blocked)
        XCTAssertFalse(d.backing)
        XCTAssertEqual(d.left, 0, accuracy: 1e-6)
        XCTAssertEqual(d.right, 0, accuracy: 1e-6)
    }

    func testNoReturnIsNotAnObstacle() {
        // 0 on the wire means clear; it must never read as an obstacle at zero.
        let d = evaluate(
            0.5, 0.5, sectors: [Double](repeating: 0, count: 8), state: "fresh"
        )
        XCTAssertFalse(d.blocked)
    }

    func testCommandsAreClamped() {
        let d = evaluate(
            5, -5, sectors: [Double](repeating: .infinity, count: 8), state: "fresh"
        )
        XCTAssertEqual(d.left, 1, accuracy: 1e-6)
        XCTAssertEqual(d.right, -1, accuracy: 1e-6)
    }

    // MARK: - Drive gate

    func testNoProximityPassesThrough() {
        let gate = DriveGate()
        let d = gate.apply(left: 0.6, right: 0.6)
        XCTAssertEqual(d.left, 0.6, accuracy: 1e-6)
        XCTAssertFalse(d.blocked)
    }

    func testStoredRequestReEvaluatedOnProximityUpdate() {
        // A cleared blockage releases without the driver re-moving the stick.
        let proximity = Proximity()
        proximity.setSectorsCm([30, 0, 0, 0, 0, 0, 0, 0])
        let gate = DriveGate(proximity: proximity, marginMeters: 1.0)
        let held = gate.apply(left: 0.5, right: 0.5)
        XCTAssertTrue(held.blocked)

        proximity.setSectorsCm([Double](repeating: 0, count: 8))
        let released = gate.onProximityUpdate()
        XCTAssertFalse(released.blocked)
        XCTAssertEqual(released.left, 0.5, accuracy: 1e-6)
    }

    func testQuickDisableReleasesImmediately() {
        let proximity = Proximity()
        proximity.setSectorsCm([30, 0, 0, 0, 0, 0, 0, 0])
        let gate = DriveGate(proximity: proximity, marginMeters: 1.0, enabled: true)
        gate.apply(left: 0.5, right: 0.5)
        let d = gate.setEnabled(false)
        XCTAssertFalse(d.blocked)
        XCTAssertEqual(d.left, 0.5, accuracy: 1e-6)
    }

    func testUpdateFromScanFeedsProximity() {
        var ranges = [UInt16](repeating: 0, count: 360)
        ranges[0] = 400 // 40 cm dead ahead
        let scan = makeScan(ranges)
        let gate = DriveGate(proximity: Proximity(), marginMeters: 1.0)
        gate.apply(left: 0.5, right: 0.5) // request forward first; (0, 0) never blocks
        let d = gate.update(from: scan)
        XCTAssertNotNil(d)
        XCTAssertTrue(d!.blocked)
        XCTAssertEqual(gate.statusText(), "Holding")
    }

    func testStatusText() {
        let clock = FakeClock()
        let proximity = Proximity { clock.call() }
        let gate = DriveGate(proximity: proximity, marginMeters: 1.0)
        XCTAssertEqual(gate.statusText(), "Idle")
        gate.setEnabled(false)
        XCTAssertEqual(gate.statusText(), "Off")
        gate.setEnabled(true)
        proximity.setSectorsCm([Double](repeating: 0, count: 8))
        XCTAssertEqual(gate.statusText(), "Clear")
        clock.advance(600)
        XCTAssertEqual(gate.statusText(), "Blind")
    }

    func testApplyWireOnlyTouchesDriveCommands() {
        let gate = DriveGate()
        XCTAssertEqual(gate.applyWire("i1,0\n"), "i1,0\n")
        XCTAssertEqual(gate.applyWire("l10,20\n"), "l10,20\n")
        XCTAssertEqual(gate.applyWire("f\n"), "f\n")
        XCTAssertEqual(gate.applyWire("c255,255\n"), "c255,255\n")
    }

    func testApplyWireBlocksForwardIntoStop() {
        let proximity = Proximity()
        proximity.setSectorsCm([30, 0, 0, 0, 0, 0, 0, 0])
        let gate = DriveGate(proximity: proximity, marginMeters: 1.0)
        XCTAssertEqual(gate.applyWire("c255,255\n"), "c0,0\n")
    }

    func testApplyWireKeepsSteeringWhenHeld() {
        let proximity = Proximity()
        proximity.setSectorsCm([30, 0, 0, 0, 0, 0, 0, 0])
        let gate = DriveGate(proximity: proximity, marginMeters: 1.0)
        // left=204, right=51 -> steering 153, half 76.5 -> 77
        XCTAssertEqual(gate.applyWire("c204,51\n"), "c77,-77\n")
    }

    // MARK: - Plan view geometry

    func testZeroRangeMaxIsSafe() {
        let scan = makeScan([100])
        XCTAssertTrue(
            LidarView.scanToPoints(scan: scan, rangeMaxMeters: 0, forwardOffsetDeg: 0).isEmpty
        )
    }

    func testOmitsBinsWithoutReturns() {
        let scan = makeScan([0, 1000, 0])
        let points = LidarView.scanToPoints(scan: scan, rangeMaxMeters: 8, forwardOffsetDeg: 0)
        XCTAssertEqual(points.count, 1)
        XCTAssertEqual(points[0].bin, 1)
    }

    func testNoseIsUp() {
        // Bin 0 sits at 0.5 deg from the LiDAR zero, i.e. essentially straight up.
        var ranges = [UInt16](repeating: 0, count: 360)
        ranges[0] = 1000
        let points = LidarView.scanToPoints(
            scan: makeScan(ranges), rangeMaxMeters: 8, forwardOffsetDeg: 0
        )
        XCTAssertEqual(points.count, 1)
        XCTAssertLessThan(points[0].y, 0)
        XCTAssertLessThan(abs(points[0].x), 0.05)
    }

    func testRangeIsClampedToOuterCircle() {
        var ranges = [UInt16](repeating: 0, count: 360)
        ranges[0] = 100_000 // 100 m, way past the 8 m max
        let points = LidarView.scanToPoints(
            scan: makeScan(ranges), rangeMaxMeters: 8, forwardOffsetDeg: 0
        )
        let r = (points[0].x * points[0].x + points[0].y * points[0].y).squareRoot()
        XCTAssertEqual(r, 0.5, accuracy: 1e-6)
    }

    func testForwardOffsetRotatesThePicture() {
        var ranges = [UInt16](repeating: 0, count: 360)
        ranges[0] = 1000
        let scan = makeScan(ranges)
        let before = LidarView.scanToPoints(scan: scan, rangeMaxMeters: 8, forwardOffsetDeg: 0)[0]
        let after = LidarView.scanToPoints(scan: scan, rangeMaxMeters: 8, forwardOffsetDeg: 90)[0]
        XCTAssertNotEqual(before.x, after.x)
        // A positive offset means the LiDAR's zero sits 90 deg clockwise of the
        // nose, so that return is 90 deg counter-clockwise of the nose: left.
        XCTAssertLessThan(after.x, 0)
        XCTAssertLessThan(abs(after.y), 0.05)
    }

    func testSplitStrokesLiftsThePenOnGaps() {
        let points = [
            LidarView.PlotPoint(x: 0, y: 0, bin: 0),
            LidarView.PlotPoint(x: 0.1, y: 0, bin: 1),
            LidarView.PlotPoint(x: 0.5, y: 0, bin: 3), // gap: bin 2 had no return
            LidarView.PlotPoint(x: 0.6, y: 0, bin: 4),
        ]
        let strokes = LidarView.splitStrokes(points)
        XCTAssertEqual(strokes.count, 2)
        XCTAssertEqual(strokes[0].map(\.bin), [0, 1])
        XCTAssertEqual(strokes[1].map(\.bin), [3, 4])
    }

    func testSplitStrokesEmpty() {
        XCTAssertTrue(LidarView.splitStrokes([]).isEmpty)
    }

    func testFormatStatus() {
        XCTAssertEqual(LidarView.formatStatus(scan: nil, isFresh: true), "Waiting for scans")
        let scan = makeScan([100, 0, 300])
        XCTAssertEqual(
            LidarView.formatStatus(scan: scan, isFresh: true),
            "3 bins · 2 returns · live"
        )
        XCTAssertEqual(
            LidarView.formatStatus(scan: scan, isFresh: false),
            "3 bins · 2 returns · stale"
        )
    }
}

// MARK: - Persisted avoidance settings

extension LidarCoreTests {
    private func freshDefaults() -> UserDefaults {
        let name = "LidarCoreTests.\(UUID().uuidString)"
        let defaults = UserDefaults(suiteName: name)!
        defaults.removePersistentDomain(forName: name)
        return defaults
    }

    func testSettingsDefaultWhenNothingStored() {
        XCTAssertEqual(AvoidanceSettings.load(from: freshDefaults()), AvoidanceSettings.defaults)
    }

    func testSettingsRoundTrip() {
        let defaults = freshDefaults()
        let settings = AvoidanceSettings(enabled: false, marginMeters: 0.8, backupSpeed: 0.25)
        settings.save(to: defaults)
        XCTAssertEqual(AvoidanceSettings.load(from: defaults), settings)
    }

    func testSettingsClampOutOfRangeValues() {
        let clamped = AvoidanceSettings(enabled: true, marginMeters: -5, backupSpeed: 9).clamped()
        XCTAssertEqual(clamped.marginMeters, AvoidanceSettings.marginRange.lowerBound)
        XCTAssertEqual(clamped.backupSpeed, AvoidanceSettings.backupRange.upperBound)
        let nan = AvoidanceSettings(enabled: true, marginMeters: .nan, backupSpeed: .infinity).clamped()
        XCTAssertEqual(nan.marginMeters, 1.0)
        XCTAssertEqual(nan.backupSpeed, 0)
    }

    func testQuickDisableReleasesAHeldCommand() {
        let gate = DriveGate(proximity: Proximity(clock: { 1000 }))
        gate.proximity.setSectorsCm([40, 0, 0, 0, 0, 0, 0, 0]) // wall 0.4 m ahead
        XCTAssertTrue(gate.apply(left: 1, right: 1).blocked)

        var settings = gate.settings
        settings.enabled = false
        let released = gate.apply(settings: settings)
        XCTAssertFalse(released.blocked)
        XCTAssertEqual(released.left, 1)
        XCTAssertEqual(released.right, 1)

        settings.enabled = true
        XCTAssertTrue(gate.apply(settings: settings).blocked)
    }

    // MARK: - RC mode face

    func testRcModeParsesEveryLetter() {
        XCTAssertEqual(RcMode.parse("mM"), .manual)
        XCTAssertEqual(RcMode.parse("mA"), .auto)
        XCTAssertEqual(RcMode.parse("mH"), .hold)
        XCTAssertEqual(RcMode.parse("mL"), .lost)
        XCTAssertEqual(RcMode.parse("mH\n"), .hold)  // a trailing newline is fine
    }

    func testRcModeRejectsGarbage() {
        XCTAssertNil(RcMode.parse(""))
        XCTAssertNil(RcMode.parse("m"))
        XCTAssertNil(RcMode.parse("mX"))
        XCTAssertNil(RcMode.parse("mMA"))
        XCTAssertNil(RcMode.parse("s42"))
    }

    func testRcModeFaces() {
        XCTAssertEqual(RcMode.manual.face, .smile)
        XCTAssertEqual(RcMode.hold.face, .cry)
        XCTAssertEqual(RcMode.lost.face, .cry)
        XCTAssertEqual(RcMode.auto.face, RcMode.Face.none)
    }
}
