//
//  GpsNavCoreTests.swift
//  OpenBotTests
//
//  Unit tests for the GPS mission navigation core. Pure logic, no hardware and
//  no simulator location needed. Mirrors the Android `gpsNav` and Linux
//  `gpsnav` test suites so the guidance law, mission codec and geo math are
//  proven to behave identically on every platform.
//

import XCTest
@testable import OpenBot

final class GpsNavCoreTests: XCTestCase {
    /// Keep tolerances in metres / degrees identical to the other platforms.
    private let toleranceMeters = 1.0
    private let toleranceDegrees = 0.01

    // MARK: - Geo math

    func testDistanceIsZeroForSamePoint() {
        XCTAssertEqual(
            GeoMath.distanceMeters(47.397742, 8.545594, 47.397742, 8.545594),
            0.0, accuracy: 1e-6
        )
    }

    func testOneDegreeOfLatitudeIsAbout111km() {
        XCTAssertEqual(GeoMath.distanceMeters(0, 0, 1, 0), 111194.9, accuracy: 50.0)
    }

    func testDistanceIsSymmetric() {
        let forward = GeoMath.distanceMeters(47.39, 8.54, 47.40, 8.55)
        let backward = GeoMath.distanceMeters(47.40, 8.55, 47.39, 8.54)
        XCTAssertEqual(forward, backward, accuracy: 1e-6)
    }

    func testCardinalBearings() {
        XCTAssertEqual(GeoMath.bearingDegrees(0, 0, 1, 0), 0.0, accuracy: toleranceDegrees)
        XCTAssertEqual(GeoMath.bearingDegrees(0, 0, 0, 1), 90.0, accuracy: toleranceDegrees)
        XCTAssertEqual(GeoMath.bearingDegrees(0, 0, 0, -1), -90.0, accuracy: toleranceDegrees)
        // Due south is antipodal in longitude terms; atan2 gives -180.
        XCTAssertEqual(GeoMath.bearingDegrees(0, 0, -1, 0), -180.0, accuracy: toleranceDegrees)
    }

    func testWrapDegreesStaysInRange() {
        XCTAssertEqual(GeoMath.wrapDegrees(0), 0.0, accuracy: 1e-9)
        XCTAssertEqual(GeoMath.wrapDegrees(370), 10.0, accuracy: 1e-9)
        XCTAssertEqual(GeoMath.wrapDegrees(-370), -10.0, accuracy: 1e-9)
        XCTAssertEqual(GeoMath.wrapDegrees(180), -180.0, accuracy: 1e-9)
        XCTAssertEqual(GeoMath.wrapDegrees(170), 170.0, accuracy: 1e-9)

        for angle in [-720.0, -360, -181, -180, -90, -1, 0, 1, 90, 179, 180, 359, 720] {
            let wrapped = GeoMath.wrapDegrees(angle)
            XCTAssertTrue((-180.0..<180.0).contains(wrapped), "wrapDegrees(\(angle)) = \(wrapped)")
        }
    }

    func testTurnErrorIsPositiveWhenTargetIsToTheRight() {
        // Facing north (0 deg), target to the east (90 deg) -> turn right by 90.
        XCTAssertEqual(GeoMath.turnErrorDegrees(0, 90), 90.0, accuracy: toleranceDegrees)
        // Facing north, target to the west -> turn left.
        XCTAssertEqual(GeoMath.turnErrorDegrees(0, -90), -90.0, accuracy: toleranceDegrees)
    }

    func testTurnErrorPicksShortWayAround() {
        // Facing 170 deg, target at -170 deg: 20 deg to the right, not 340 left.
        XCTAssertEqual(GeoMath.turnErrorDegrees(170, -170), 20.0, accuracy: toleranceDegrees)
        // Facing -170 deg, target at 170 deg: 20 deg to the left.
        XCTAssertEqual(GeoMath.turnErrorDegrees(-170, 170), -20.0, accuracy: toleranceDegrees)
    }

    func testDestinationRoundTripsWithBearingAndDistance() {
        let bearing = 37.0
        let distance = 250.0
        let end = GeoMath.destinationPoint(47.397742, 8.545594, bearing, distance)

        XCTAssertEqual(
            GeoMath.distanceMeters(47.397742, 8.545594, end.latitude, end.longitude),
            distance, accuracy: toleranceMeters
        )
        XCTAssertEqual(
            GeoMath.bearingDegrees(47.397742, 8.545594, end.latitude, end.longitude),
            bearing, accuracy: toleranceDegrees
        )
    }

    // MARK: - Guidance

    private let robotLatitude = 0.0
    private let robotLongitude = 0.0

    private func targetAt(_ bearing: Double, _ distance: Double) -> Waypoint {
        let end = GeoMath.destinationPoint(robotLatitude, robotLongitude, bearing, distance)
        return Waypoint(latitude: end.latitude, longitude: end.longitude)
    }

    private var guidance: GuidanceController { GuidanceController() }

    func testTargetStraightAheadDrivesBothWheelsEqually() {
        let result = guidance.compute(
            latitude: robotLatitude, longitude: robotLongitude,
            headingDegrees: 0, target: targetAt(0, 20)
        )

        XCTAssertFalse(result.arrived)
        XCTAssertEqual(result.turnErrorDegrees, 0.0, accuracy: 0.5)
        XCTAssertEqual(result.left, result.right, accuracy: 1e-4)
        XCTAssertGreaterThan(result.left, 0.1)
    }

    func testTargetToTheRightLeadsWithLeftWheel() {
        let result = guidance.compute(
            latitude: robotLatitude, longitude: robotLongitude,
            headingDegrees: 0, target: targetAt(30, 20)
        )

        XCTAssertGreaterThan(result.turnErrorDegrees, 0)
        XCTAssertGreaterThan(result.left, result.right)
    }

    func testTargetToTheLeftLeadsWithRightWheel() {
        let result = guidance.compute(
            latitude: robotLatitude, longitude: robotLongitude,
            headingDegrees: 0, target: targetAt(-30, 20)
        )

        XCTAssertLessThan(result.turnErrorDegrees, 0)
        XCTAssertGreaterThan(result.right, result.left)
    }

    func testLargeHeadingErrorPivotsInPlace() {
        let result = guidance.compute(
            latitude: robotLatitude, longitude: robotLongitude,
            headingDegrees: 0, target: targetAt(90, 20)
        )

        // Hard right: left wheel forward, right wheel backward.
        XCTAssertGreaterThan(result.left, 0)
        XCTAssertLessThan(result.right, 0)
        XCTAssertEqual(result.left + result.right, 0.0, accuracy: 1e-3)
    }

    func testLargeNegativeHeadingErrorPivotsTheOtherWay() {
        let result = guidance.compute(
            latitude: robotLatitude, longitude: robotLongitude,
            headingDegrees: 0, target: targetAt(-90, 20)
        )

        XCTAssertLessThan(result.left, 0)
        XCTAssertGreaterThan(result.right, 0)
        XCTAssertEqual(result.left + result.right, 0.0, accuracy: 1e-3)
    }

    func testInsideAcceptanceRadiusArrivesAndStops() {
        let result = guidance.compute(
            latitude: robotLatitude, longitude: robotLongitude,
            headingDegrees: 0, target: targetAt(0, 1)
        )

        XCTAssertTrue(result.arrived)
        XCTAssertEqual(result.left, 0)
        XCTAssertEqual(result.right, 0)
    }

    func testWaypointAcceptanceRadiusOverridesDefault() {
        let loose = Waypoint(latitude: 0, longitude: 0, acceptanceRadiusMeters: 20)
        let result = guidance.compute(
            latitude: robotLatitude, longitude: robotLongitude,
            headingDegrees: 0, target: loose
        )

        XCTAssertTrue(result.arrived)
    }

    func testSlowsDownWhenCloseToTheWaypoint() {
        let far = guidance.compute(
            latitude: robotLatitude, longitude: robotLongitude,
            headingDegrees: 0, target: targetAt(0, 20)
        ).left
        let near = guidance.compute(
            latitude: robotLatitude, longitude: robotLongitude,
            headingDegrees: 0, target: targetAt(0, 5)
        ).left

        XCTAssertLessThan(near, far)
    }

    func testDefaultRadiusSuitsPhoneGps() {
        // 3.5 m out is inside the phone-GPS default of 4 m: arrived, not circling.
        XCTAssertTrue(guidance.compute(
            latitude: robotLatitude, longitude: robotLongitude,
            headingDegrees: 0, target: targetAt(0, 3.5)
        ).arrived)
        XCTAssertFalse(guidance.compute(
            latitude: robotLatitude, longitude: robotLongitude,
            headingDegrees: 0, target: targetAt(0, 4.5)
        ).arrived)
    }

    func testCommandsStayInsideNormalisedRange() {
        let bearings = [0.0, 15, 45, 55, 70, 90, 135, 170, -170, -90, -45, -15]
        let distances = [1.5, 2.6, 3, 5, 8, 12, 30, 100]

        for bearing in bearings {
            for distance in distances {
                let result = guidance.compute(
                    latitude: robotLatitude, longitude: robotLongitude,
                    headingDegrees: 0, target: targetAt(bearing, distance)
                )
                XCTAssertTrue((-1.0...1.0).contains(result.left))
                XCTAssertTrue((-1.0...1.0).contains(result.right))
            }
        }
    }

    func testReportsDistanceAndBearingToTarget() {
        let result = guidance.compute(
            latitude: robotLatitude, longitude: robotLongitude,
            headingDegrees: 0, target: targetAt(45, 50)
        )

        XCTAssertEqual(result.distanceMeters, 50.0, accuracy: 1.0)
        XCTAssertEqual(result.bearingDegrees, 45.0, accuracy: 0.5)
        XCTAssertEqual(result.turnErrorDegrees, 45.0, accuracy: 0.5)
    }

    // MARK: - Mission codec

    func testWaypointsRoundTripPreservesHomeAndOrder() throws {
        let original = Mission()
        original.setHome(47.3977420, 8.5455940)
        original.addWaypoint(Waypoint(latitude: 47.3981000, longitude: 8.5460000))
        original.addWaypoint(Waypoint(latitude: 47.3985000, longitude: 8.5470000, acceptanceRadiusMeters: 5))
        original.addWaypoint(Waypoint(latitude: 47.3990000, longitude: 8.5480000))

        let parsed = try Mission.parseWaypointsFile(original.toWaypointsFile())

        XCTAssertTrue(parsed.hasHome)
        XCTAssertEqual(parsed.homeLatitude!, original.homeLatitude!, accuracy: 1e-7)
        XCTAssertEqual(parsed.homeLongitude!, original.homeLongitude!, accuracy: 1e-7)
        XCTAssertEqual(parsed.count, original.count)

        for (expected, actual) in zip(original.waypoints, parsed.waypoints) {
            XCTAssertEqual(actual.latitude, expected.latitude, accuracy: 1e-7)
            XCTAssertEqual(actual.longitude, expected.longitude, accuracy: 1e-7)
        }
    }

    func testFileHeaderMatchesMissionPlanner() {
        let mission = Mission()
        mission.setHome(47, 8)
        mission.addWaypoint(Waypoint(latitude: 47.1, longitude: 8.1))

        let text = mission.toWaypointsFile()
        XCTAssertTrue(text.hasPrefix("QGC WPL 110\n"))
        let lines = text.split(separator: "\n", omittingEmptySubsequences: false).map(String.init)
        XCTAssertEqual(lines.count, 4) // header, home, wp, trailing empty
        XCTAssertTrue(lines[1].hasPrefix("0\t1\t"))
        XCTAssertTrue(lines[2].hasPrefix("1\t0\t"))
    }

    func testNonNavigationCommandsAreSkippedOnRead() throws {
        let text = """
        QGC WPL 110
        0\t1\t3\t16\t0\t0\t0\t0\t47.3977420\t8.5455940\t0\t1
        1\t0\t3\t177\t0\t0\t0\t0\t47.3980000\t8.5460000\t0\t1
        2\t0\t3\t16\t0\t0\t0\t0\t47.3985000\t8.5470000\t0\t1
        """

        let mission = try Mission.parseWaypointsFile(text)

        XCTAssertEqual(mission.count, 1)
        XCTAssertEqual(mission.waypoints[0].latitude, 47.3985000, accuracy: 1e-7)
    }

    func testRejectsForeignHeader() {
        XCTAssertThrowsError(try Mission.parseWaypointsFile("not a waypoint file\n")) { error in
            guard case MissionError.unsupportedHeader = error else {
                return XCTFail("expected unsupportedHeader, got \(error)")
            }
        }
    }

    func testRejectsMalformedRow() {
        XCTAssertThrowsError(try Mission.parseWaypointsFile("QGC WPL 110\n0\t1\t3\n")) { error in
            guard case MissionError.malformedRow = error else {
                return XCTFail("expected malformedRow, got \(error)")
            }
        }
    }

    func testEditsMaintainOrder() {
        let mission = Mission()
        mission.addWaypoint(Waypoint(latitude: 1, longitude: 1))
        mission.addWaypoint(Waypoint(latitude: 2, longitude: 2))
        mission.addWaypoint(Waypoint(latitude: 3, longitude: 3))

        mission.moveWaypoint(from: 2, to: 0)
        XCTAssertEqual(mission.waypoints.map(\.latitude), [3, 1, 2])

        mission.removeWaypoint(at: 1)
        XCTAssertEqual(mission.waypoints.map(\.latitude), [3, 2])

        mission.clearWaypoints()
        XCTAssertTrue(mission.isEmpty)
    }

    func testTotalDistanceRunsHomeThroughWaypoints() {
        let mission = Mission()
        // 1 degree of latitude is about 111.19 km.
        mission.setHome(0, 0)
        mission.addWaypoint(Waypoint(latitude: 1, longitude: 0))
        mission.addWaypoint(Waypoint(latitude: 2, longitude: 0))

        XCTAssertEqual(mission.totalDistanceMeters(), 222389.9, accuracy: 100.0)
    }

    func testTotalDistanceWithoutHomeStartsAtFirstWaypoint() {
        let mission = Mission()
        mission.addWaypoint(Waypoint(latitude: 0, longitude: 0))
        mission.addWaypoint(Waypoint(latitude: 1, longitude: 0))

        XCTAssertEqual(mission.totalDistanceMeters(), 111194.9, accuracy: 50.0)
    }

    // MARK: - Navigator

    private func makeNavigator(_ waypoints: Waypoint..., home: (Double, Double)? = (0, 0)) -> Navigator {
        let mission = Mission()
        if let home = home {
            mission.setHome(home.0, home.1)
        }
        waypoints.forEach { mission.addWaypoint($0) }
        return Navigator(mission: mission)
    }

    func testIdleReportsZeroCommand() {
        let nav = Navigator()
        let status = nav.step(latitude: 0, longitude: 0)

        XCTAssertEqual(status.mode, .idle)
        XCTAssertEqual(status.left, 0)
        XCTAssertEqual(status.right, 0)
        XCTAssertFalse(nav.isNavigating)
    }

    func testStartMissionRefusesAnEmptyPlan() {
        XCTAssertFalse(Navigator().startMission())
    }

    func testMissionNeedsAHeadingBeforeItWillSteer() {
        let nav = makeNavigator(targetAt(0, 20))
        XCTAssertTrue(nav.startMission())

        let status = nav.step(latitude: robotLatitude, longitude: robotLongitude)

        XCTAssertEqual(status.mode, .mission)
        XCTAssertEqual(status.message, "acquiring heading")
        XCTAssertEqual(status.left, 0)
        XCTAssertEqual(status.right, 0)
    }

    func testMissionDrivesTowardTheFirstWaypoint() {
        let nav = makeNavigator(targetAt(0, 20))
        nav.startMission()
        nav.setHeading(0)

        let status = nav.step(latitude: robotLatitude, longitude: robotLongitude)

        XCTAssertEqual(status.mode, .mission)
        XCTAssertGreaterThan(status.left, 0)
        XCTAssertEqual(status.left, status.right, accuracy: 1e-3)
        XCTAssertEqual(status.distanceMeters ?? -1, 20.0, accuracy: 1.0)
    }

    func testMissionAdvancesToTheNextWaypointOnArrival() {
        let nav = makeNavigator(targetAt(0, 1), targetAt(90, 20))
        nav.startMission()
        nav.setHeading(0)

        let status = nav.step(latitude: robotLatitude, longitude: robotLongitude)

        XCTAssertTrue(status.arrived)
        XCTAssertEqual(status.activeWaypointIndex, 0)
        XCTAssertEqual(nav.activeIndex, 1)
        XCTAssertTrue(nav.isNavigating)
    }

    func testMissionCompletesAfterTheLastWaypoint() {
        let nav = makeNavigator(targetAt(0, 1))
        nav.startMission()
        nav.setHeading(0)

        let status = nav.step(latitude: robotLatitude, longitude: robotLongitude)

        XCTAssertTrue(status.arrived)
        XCTAssertEqual(status.activeWaypointIndex, 0)
        XCTAssertFalse(nav.isNavigating)
        XCTAssertEqual(nav.mode, .idle)
    }

    func testGuidedDrivesToOnePointThenDisarms() {
        let nav = makeNavigator(home: nil)
        nav.markHome(0, 0)
        let end = GeoMath.destinationPoint(0, 0, 0, 1)
        nav.startGuided(latitude: end.latitude, longitude: end.longitude)
        nav.setHeading(0)

        let status = nav.step(latitude: robotLatitude, longitude: robotLongitude)

        XCTAssertEqual(nav.mode, .idle) // arrival already disarmed it
        XCTAssertTrue(status.arrived)
        XCTAssertFalse(nav.isNavigating)
    }

    func testRTLNeedsAHomePosition() {
        XCTAssertFalse(Navigator().startRTL())
    }

    func testRTLDrivesBackToHome() {
        let far = targetAt(0, 30)
        let nav = makeNavigator(far)
        XCTAssertTrue(nav.startRTL())
        nav.setHeading(180)

        // Standing at the far waypoint, facing away: still 30 m from home.
        let status = nav.step(latitude: far.latitude, longitude: far.longitude)

        XCTAssertEqual(status.mode, .rtl)
        XCTAssertEqual(status.distanceMeters ?? -1, 30.0, accuracy: 1.0)
        XCTAssertFalse(status.arrived)
    }

    func testRTLArrivesAndDisarms() {
        let nav = makeNavigator(targetAt(0, 30))
        XCTAssertTrue(nav.startRTL())
        nav.setHeading(0)

        let status = nav.step(latitude: robotLatitude, longitude: robotLongitude)

        XCTAssertTrue(status.arrived)
        XCTAssertEqual(nav.mode, .idle)
    }

    func testStopDisarmsNavigation() {
        let nav = makeNavigator(targetAt(0, 20))
        nav.startMission()
        nav.stop()

        XCTAssertFalse(nav.isNavigating)
        XCTAssertEqual(nav.step(latitude: 0, longitude: 0).left, 0)
    }

    func testPauseHoldsTheRobot() {
        let nav = makeNavigator(targetAt(0, 20))
        nav.startMission()
        nav.setHeading(0)
        nav.pause()

        let status = nav.step(latitude: 0, longitude: 0)

        XCTAssertEqual(status.mode, .mission)
        XCTAssertEqual(status.message, "paused")
        XCTAssertEqual(status.left, 0)

        nav.resume()
        XCTAssertTrue(nav.step(latitude: 0, longitude: 0).isMoving)
    }

    func testHoldsWhenTheFixIsUntrustworthy() {
        let nav = makeNavigator(targetAt(0, 20))
        nav.startMission()
        nav.setHeading(0)

        let status = nav.step(latitude: 0, longitude: 0, accuracyMeters: 25)

        XCTAssertEqual(status.left, 0)
        XCTAssertEqual(status.right, 0)
        XCTAssertTrue(status.message.contains("holding"))
    }

    func testDrivesOnAFixInsideTheAccuracyBudget() {
        let nav = makeNavigator(targetAt(0, 20))
        nav.startMission()
        nav.setHeading(0)

        let status = nav.step(latitude: 0, longitude: 0, accuracyMeters: 4)

        XCTAssertTrue(status.isMoving)
    }

    func testRecordsATrackOfEveryFix() {
        let nav = makeNavigator(targetAt(0, 20))
        nav.startMission()
        nav.setHeading(0)

        nav.step(latitude: 0, longitude: 0)
        nav.step(latitude: 0, longitude: 0.001)

        XCTAssertEqual(nav.track.count, 2)
        XCTAssertEqual(nav.track[0].latitude, 0)
        XCTAssertEqual(nav.track[1].longitude, 0.001, accuracy: 1e-9)
    }

    func testKmlExportContainsThePlanAndTheTrack() {
        let nav = makeNavigator(targetAt(0, 20), targetAt(90, 20))
        nav.startMission()
        nav.setHeading(0)
        nav.step(latitude: 0, longitude: 0)

        let kml = nav.exportKml(documentName: "Test Mission")

        XCTAssertTrue(kml.contains("<name>Test Mission</name>"))
        XCTAssertTrue(kml.contains("<name>Home</name>"))
        XCTAssertTrue(kml.contains("<name>WP1</name>"))
        XCTAssertTrue(kml.contains("<name>WP2</name>"))
        XCTAssertTrue(kml.contains("<name>Track</name>"))
    }

    func testSaveAndLoadRoundTripThroughTheNavigator() throws {
        let nav = makeNavigator(targetAt(0, 20), targetAt(90, 20))
        let text = nav.saveMissionFile()

        let reloaded = Navigator()
        try reloaded.loadMissionFile(text)

        XCTAssertEqual(reloaded.mission.count, 2)
        XCTAssertTrue(reloaded.mission.hasHome)
    }

    func testRejectsAForeignFile() {
        XCTAssertThrowsError(try Navigator().loadMissionFile("garbage\n"))
    }

    // MARK: - Compass reliability

    func testCompassTrustedWhenCalibratedInEarthField() {
        XCTAssertTrue(GpsNavLocationProvider.isCompassReliable(headingAccuracy: 10, fieldMicroTesla: 42))
        XCTAssertTrue(GpsNavLocationProvider.isCompassReliable(headingAccuracy: 10, fieldMicroTesla: nil))
    }

    func testCompassDistrustedWhenUncalibrated() {
        XCTAssertFalse(GpsNavLocationProvider.isCompassReliable(headingAccuracy: -1, fieldMicroTesla: 42))
        XCTAssertFalse(GpsNavLocationProvider.isCompassReliable(headingAccuracy: 45, fieldMicroTesla: 42))
    }

    func testCompassDistrustedNearMotors() {
        XCTAssertFalse(GpsNavLocationProvider.isCompassReliable(headingAccuracy: 10, fieldMicroTesla: 140))
        XCTAssertFalse(GpsNavLocationProvider.isCompassReliable(headingAccuracy: 10, fieldMicroTesla: 8))
    }
}
