//
//  LidarBehaviors.swift
//  OpenBot
//
//  LiDAR behaviours for the Ackermann car, after the hoverboard robot's ROS 2
//  package hoverboard_laser (github.com/quangvu16/hoverboard-robot,
//  src/hoverboard_laser): laser_Avoidance, laser_Tracker and laser_Warning, plus
//  Patrol and Tracking built on the same helpers. Same logic as
//  LidarBehaviors.java on Android - keep the two in step.
//
//  The hoverboard turns on the spot; this car cannot, so "turn on the spot"
//  becomes a forward turn or backing out with the wheels turned.
//
//  Angles follow the hoverboard (ROS): degrees from the nose, positive LEFT.
//  Output: throttle -1...1 and steer -1...1 with positive = the nose turns left,
//  also in reverse (the firmware flips the servo when backing).
//

import Foundation

final class LidarBehaviors {
    enum Mode: String, CaseIterable { case patrol, follow, avoid, track, warn }

    enum Mood { case idle, happy, love, focused, searching, worried, scared, blind }

    struct Params {
        var speed = 0.35          // fraction of the speed mode
        var responseDist = 0.55   // metres (ResponseDist)
        var laserAngle = 45.0     // degrees (LaserAngle)
        var minRange = 0.15       // metres: closer is the car itself

        static func defaults(_ mode: Mode) -> Params {
            var p = Params()
            switch mode {
            case .follow: p.responseDist = 0.55; p.laserAngle = 30; p.speed = 0.4
            case .track: p.responseDist = 0.4; p.laserAngle = 90; p.speed = 0.25
            case .warn: p.responseDist = 0.5; p.laserAngle = 20; p.speed = 0
            case .patrol: p.responseDist = 0.6; p.laserAngle = 45; p.speed = 0.35
            case .avoid: p.responseDist = 0.55; p.laserAngle = 45; p.speed = 0.35
            }
            return p
        }
    }

    struct Decision {
        var throttle = 0.0
        var steer = 0.0
        var mood: Mood = .idle
        var gazeDeg = 0.0
        var targetDist = Double.nan
        var alarm = false
        var reason = ""

        mutating func set(_ throttle: Double, _ steer: Double, _ mood: Mood, _ reason: String) {
            self.throttle = LidarBehaviors.clamp(throttle, -1, 1)
            self.steer = LidarBehaviors.clamp(steer, -1, 1)
            self.mood = mood
            self.reason = reason
        }
    }

    struct Points {
        var angleDeg: [Double]
        var rangeM: [Double]
        var coarse: Bool
        var count: Int { angleDeg.count }
    }

    /// Bin i is (i + 0.5) * 360 / bins degrees CLOCKWISE from the LiDAR's zero mark; 0 mm = no return.
    static func fromScan(_ rangesMm: [Int], forwardOffsetDeg: Double = 0, minRange: Double) -> Points {
        var a: [Double] = [], r: [Double] = []
        let n = Double(rangesMm.count)
        for (i, mm) in rangesMm.enumerated() where mm > 0 && Double(mm) / 1000 >= minRange {
            let cw = (Double(i) + 0.5) * 360 / n - forwardOffsetDeg
            a.append(wrap(-cw))
            r.append(Double(mm) / 1000)
        }
        return Points(angleDeg: a, rangeM: r, coarse: false)
    }

    /// The car's 8 proximity sectors (metres, clockwise from the nose, .infinity = clear).
    static func fromSectors(_ sectorsM: [Double]) -> Points {
        var a: [Double] = [], r: [Double] = []
        let step = 360 / Double(sectorsM.count)
        for (i, m) in sectorsM.enumerated() where m.isFinite && m > 0 {
            a.append(wrap(-Double(i) * step))
            r.append(m)
        }
        return Points(angleDeg: a, rangeM: r, coarse: true)
    }

    let mode: Mode
    var params: Params
    private var held: Decision?
    private var holdUntil = 0.0
    private(set) var patrolTurns = 0
    private var patrolTurnStep = 0
    private var turnSide = 1.0
    // Avoid: escape side chosen when first blocked and kept for a while (no rocking in corners).
    private var escapeSide = 0.0
    private var escapeUntil = 0.0

    init(mode: Mode, params: Params) {
        self.mode = mode
        self.params = params
    }

    /// One step (about 10 Hz). nil points = no LiDAR: stop.
    func step(_ points: Points?, now: Double) -> Decision {
        var d = Decision()
        guard let p = points else {
            held = nil
            patrolTurnStep = 0
            d.set(0, 0, .blind, "No LiDAR data: stopped")
            return d
        }
        if let h = held, now < holdUntil { return h }
        held = nil
        switch mode {
        case .avoid: return avoid(p, now, d)
        case .follow: return follow(p, d)
        case .track: return track(p, d)
        case .warn: return warn(p, d)
        case .patrol: return patrol(p, now, d)
        }
    }

    // MARK: Avoidance (laser_Avoidance)

    private func avoid(_ p: Points, _ now: Double, _ d0: Decision) -> Decision {
        var d = d0
        let near = params.responseDist * 1.5
        let minPoints = p.coarse ? 0 : 5
        let a = params.laserAngle
        var front = 0, left = 0, right = 0
        var frontMin = Double.infinity, nearest = Double.infinity, nearestAng = 0.0
        for i in 0..<p.count {
            let ang = p.angleDeg[i], r = p.rangeM[i]
            if abs(ang) <= a && r < nearest { nearest = r; nearestAng = ang }
            if r >= near { continue }
            if abs(ang) <= 20 { front += 1; frontMin = min(frontMin, r) }
            else if ang > 20 && ang < a { left += 1 }
            else if ang < -20 && ang > -a { right += 1 }
        }
        let f = front > minPoints, l = left > minPoints, r = right > minPoints
        let v = params.speed
        d.gazeDeg = nearest < near ? nearestAng : 0
        d.targetDist = nearest
        if f && (escapeSide == 0 || now > escapeUntil) { escapeSide = left <= right ? 1 : -1 }
        if f { escapeUntil = now + 4 }
        let side = escapeSide > 0 ? "left" : "right"
        if f && l && r {
            d.set(-v * 0.6, escapeSide, .scared, "Boxed in: backing out, nose turning " + side)
            return hold(d, now, 1.2)
        } else if f && frontMin < params.responseDist {
            d.set(-v * 0.6, escapeSide, .worried, "Too close ahead: backing out, nose turning " + side)
            return hold(d, now, 0.8)
        } else if f && (r || l) {
            let what = r && !l ? "Front and right" : (l && !r ? "Front and left" : "Front")
            d.set(v * 0.5, escapeSide, .worried, what + " blocked: turning " + side)
            return hold(d, now, 0.4)
        } else if f {
            d.set(v * 0.5, escapeSide, .worried, "Front blocked: turning " + side)
            return hold(d, now, 0.4)
        } else if l && r {
            d.set(v * 0.5, 0, .focused, "Narrow gap: slowly straight through")
        } else if l {
            d.set(v, -0.6, .focused, "Left blocked: veering right")
        } else if r {
            d.set(v, 0.6, .focused, "Right blocked: veering left")
        } else {
            d.set(v, 0, .happy, "All clear: forward")
        }
        return d
    }

    // MARK: Follower (laser_Tracker)

    private func follow(_ p: Points, _ d0: Decision) -> Decision {
        var d = d0
        let dist = params.responseDist
        var best = -1
        for i in 0..<p.count where abs(p.angleDeg[i]) < 10 && p.rangeM[i] < dist + 0.5 {
            if best < 0 || p.rangeM[i] < p.rangeM[best] { best = i }
        }
        if best < 0 {
            for i in 0..<p.count where abs(p.angleDeg[i]) < params.laserAngle {
                if best < 0 || p.rangeM[i] < p.rangeM[best] { best = i }
            }
        }
        if best < 0 || p.rangeM[best] > 3 {
            d.set(0, 0, .searching, "Nothing to follow in front")
            return d
        }
        let r = p.rangeM[best], ang = p.angleDeg[best]
        d.gazeDeg = ang
        d.targetDist = r
        var err = r - dist
        if abs(err) < 0.1 { err = 0 }
        let throttle = Self.clamp(err * 2, -1, 1) * params.speed
        let steer = Self.clamp(ang / 45, -1, 1)
        if throttle == 0 {
            d.set(0, steer, .love, String(format: "Holding %.2f m from the target", r))
        } else if throttle > 0 {
            d.set(throttle, steer, .love, String(format: "Following: %.2f m ahead", r))
        } else {
            d.set(throttle, steer, .worried, String(format: "Too close (%.2f m): backing off", r))
        }
        return d
    }

    // MARK: Tracking

    private func track(_ p: Points, _ d0: Decision) -> Decision {
        var d = d0
        let best = Self.nearestIn(p, params.laserAngle, 3)
        guard best >= 0 else {
            d.set(0, 0, .searching, "Looking for something to track")
            return d
        }
        let r = p.rangeM[best], ang = p.angleDeg[best]
        d.gazeDeg = ang
        d.targetDist = r
        let steer = Self.clamp(ang / 30, -1, 1)
        if abs(ang) <= 10 {
            d.set(0, 0, .focused, String(format: "Facing the target, %.2f m", r))
        } else if r <= params.responseDist {
            d.set(0, steer, .worried, "Target too close to turn towards: pointing the wheels only")
        } else {
            d.set(params.speed, steer, .focused,
                  String(format: "Turning towards the target (%.0f° %@)", abs(ang), ang > 0 ? "left" : "right"))
        }
        return d
    }

    // MARK: Warning (laser_Warning)

    private func warn(_ p: Points, _ d0: Decision) -> Decision {
        var d = d0
        let best = Self.nearestIn(p, params.laserAngle, .infinity)
        guard best >= 0 else {
            d.set(0, 0, .happy, "Watching: nothing in front")
            return d
        }
        let r = p.rangeM[best], ang = p.angleDeg[best]
        d.gazeDeg = ang
        d.targetDist = r
        if r <= params.responseDist {
            d.alarm = true
            d.set(0, Self.clamp(ang / 30, -1, 1), .scared, String(format: "WARNING: object %.2f m away", r))
        } else {
            d.set(0, 0, r <= params.responseDist * 2 ? .worried : .focused, String(format: "Watching: nearest %.2f m", r))
        }
        return d
    }

    // MARK: Patrol

    private func patrol(_ p: Points, _ now: Double, _ d0: Decision) -> Decision {
        var d = d0
        let front = Self.minIn(p, -20, 20)
        let left = Self.minIn(p, 60, 120)
        let right = Self.minIn(p, -120, -60)
        let v = params.speed
        d.targetDist = front
        if patrolTurnStep > 0 {
            if front > params.responseDist * 2.5 {
                patrolTurnStep = 0
                patrolTurns += 1
            } else {
                let back = patrolTurnStep % 2 == 1
                patrolTurnStep += 1
                d.gazeDeg = turnSide * 60
                if back {
                    d.set(-v * 0.7, turnSide, .focused, "Patrol: turning round (backing)")
                } else {
                    d.set(v * 0.6, turnSide, .focused, "Patrol: turning round (forward)")
                }
                return hold(d, now, 0.9)
            }
        }
        if front < params.responseDist * 1.5 {
            turnSide = left >= right ? 1 : -1
            patrolTurnStep = 1
            d.set(0, 0, .worried, "Patrol: wall ahead, turning round " + (turnSide > 0 ? "left" : "right"))
            return hold(d, now, 0.3)
        }
        let l = min(left, 3), r = min(right, 3)
        let steer = Self.clamp((l - r) * 0.6, -0.6, 0.6)
        d.gazeDeg = steer * 30
        d.set(v, steer, .focused, String(format: "Patrolling: %.1f m clear ahead", min(front, 9.9)))
        return d
    }

    // MARK: Helpers

    private func hold(_ d: Decision, _ now: Double, _ seconds: Double) -> Decision {
        held = d
        holdUntil = now + seconds
        return d
    }

    private static func nearestIn(_ p: Points, _ halfAngle: Double, _ maxRange: Double) -> Int {
        var best = -1
        for i in 0..<p.count where abs(p.angleDeg[i]) <= halfAngle && p.rangeM[i] <= maxRange {
            if best < 0 || p.rangeM[i] < p.rangeM[best] { best = i }
        }
        return best
    }

    static func minIn(_ p: Points, _ from: Double, _ to: Double) -> Double {
        var m = Double.infinity
        for i in 0..<p.count where p.angleDeg[i] >= from && p.angleDeg[i] <= to { m = min(m, p.rangeM[i]) }
        return m
    }

    /// Throttle and steer (+ nose left) to the app's left/right pair (l + r = throttle, l - r = -steer).
    static func toWheels(throttle: Double, steer: Double) -> (left: Double, right: Double) {
        (clamp((throttle - steer) / 2, -1, 1), clamp((throttle + steer) / 2, -1, 1))
    }

    static func wrap(_ deg: Double) -> Double {
        let a = ((deg + 180).truncatingRemainder(dividingBy: 360) + 360).truncatingRemainder(dividingBy: 360) - 180
        return a == -180 ? 180 : a
    }

    static func clamp(_ v: Double, _ lo: Double, _ hi: Double) -> Double { max(lo, min(hi, v)) }
}
