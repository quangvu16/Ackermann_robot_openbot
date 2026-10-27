//
//  SlamCore.swift
//  OpenBot
//
//  Indoor SLAM and navigation for the car. The same algorithms as the Android
//  classes in org.openbot.app.robot.slam (OccupancyGrid, SlamEngine,
//  Navigator) - keep them in step. Written for this project after BreezySLAM /
//  CoreSLAM, SmartphoneGraphSLAM, slam_toolbox and Nav2; no code from them is
//  used (several are (L)GPL, this project is MIT).
//

import Foundation

struct SlamPose {
    var x: Double
    var y: Double
    var theta: Double // radians, counter-clockwise from +x

    init(_ x: Double, _ y: Double, _ theta: Double) {
        self.x = x
        self.y = y
        self.theta = SlamPose.wrap(theta)
    }

    static let zero = SlamPose(0, 0, 0)

    func distance(to x: Double, _ y: Double) -> Double { hypot(x - self.x, y - self.y) }

    static func wrap(_ a: Double) -> Double {
        let r = remainder(a, 2 * .pi)
        return r <= -.pi ? r + 2 * .pi : r
    }

    var text: String { String(format: "x %.2f m, y %.2f m, %.0f°", x, y, theta * 180 / .pi) }
}

// MARK: - Occupancy grid

/// Log-odds grid: 0 unknown, + wall, - free. Cell (0, 0) is the bottom-left corner.
final class OccupancyGrid {
    static let lOcc: Float = 0.9, lFree: Float = -0.35, lMin: Float = -3.5, lMax: Float = 3.5
    static let occThreshold: Float = 0.6, freeThreshold: Float = -0.6

    let width: Int, height: Int
    let resolution: Double
    let originX: Double, originY: Double
    var logOdds: [Float]
    private(set) var version = 0

    init(width: Int, height: Int, resolution: Double, originX: Double? = nil, originY: Double? = nil) {
        self.width = width
        self.height = height
        self.resolution = resolution
        self.originX = originX ?? -Double(width) * resolution / 2
        self.originY = originY ?? -Double(height) * resolution / 2
        logOdds = [Float](repeating: 0, count: width * height)
    }

    /// 30 m x 30 m at 5 cm.
    static func indoorDefault() -> OccupancyGrid { OccupancyGrid(width: 600, height: 600, resolution: 0.05) }

    func cellX(_ x: Double) -> Int { Int(floor((x - originX) / resolution)) }
    func cellY(_ y: Double) -> Int { Int(floor((y - originY) / resolution)) }
    func inBounds(_ cx: Int, _ cy: Int) -> Bool { cx >= 0 && cy >= 0 && cx < width && cy < height }
    func get(_ cx: Int, _ cy: Int) -> Float { inBounds(cx, cy) ? logOdds[cy * width + cx] : 0 }
    func set(_ cx: Int, _ cy: Int, _ v: Float) { if inBounds(cx, cy) { logOdds[cy * width + cx] = v } }
    func isOccupied(_ cx: Int, _ cy: Int) -> Bool { get(cx, cy) > Self.occThreshold }

    func clear() {
        for i in logOdds.indices { logOdds[i] = 0 }
        version += 1
    }

    private func add(_ cx: Int, _ cy: Int, _ d: Float) {
        guard inBounds(cx, cy) else { return }
        let i = cy * width + cx
        logOdds[i] = min(Self.lMax, max(Self.lMin, logOdds[i] + d))
    }

    /// Beams pass through free cells and end in a wall (unless at max range).
    func integrate(_ pose: SlamPose, angles: [Double], ranges: [Double], maxRange: Double) {
        let x0 = cellX(pose.x), y0 = cellY(pose.y)
        for i in 0..<angles.count {
            let hit = ranges[i] < maxRange
            let d = min(ranges[i], maxRange)
            let a = pose.theta + angles[i]
            let x1 = cellX(pose.x + d * cos(a)), y1 = cellY(pose.y + d * sin(a))
            traceFree(x0, y0, x1, y1)
            if hit { add(x1, y1, Self.lOcc - Self.lFree) }
        }
        version += 1
    }

    private func traceFree(_ x0: Int, _ y0: Int, _ x1: Int, _ y1: Int) {
        let dx = abs(x1 - x0), dy = -abs(y1 - y0)
        let sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1
        var err = dx + dy, x = x0, y = y0
        for _ in 0..<4096 {
            add(x, y, Self.lFree)
            if x == x1 && y == y1 { return }
            let e2 = 2 * err
            if e2 >= dy { err += dy; x += sx }
            if e2 <= dx { err += dx; y += sy }
        }
    }

    func hitScore(_ cx: Int, _ cy: Int) -> Double {
        guard inBounds(cx, cy) else { return 0 }
        var s = Double(max(0, get(cx, cy)))
        s += 0.35 * Double(max(0, get(cx + 1, cy)) + max(0, get(cx - 1, cy)) + max(0, get(cx, cy + 1)) + max(0, get(cx, cy - 1)))
        return s
    }

    // MARK: Files ("OBM1", the same layout as the Android app, big-endian)

    func data() -> Data {
        var d = Data()
        func i32(_ v: Int32) { var b = v.bigEndian; d.append(Data(bytes: &b, count: 4)) }
        func f64(_ v: Double) { var b = v.bitPattern.bigEndian; d.append(Data(bytes: &b, count: 8)) }
        i32(0x4F424D31); i32(Int32(width)); i32(Int32(height))
        f64(resolution); f64(originX); f64(originY)
        d.append(contentsOf: logOdds.map { UInt8(bitPattern: Int8(clamping: Int(($0 * 32).rounded()))) })
        return d
    }

    static func read(_ d: Data) -> OccupancyGrid? {
        guard d.count >= 36 else { return nil }
        let b = [UInt8](d)
        func i32(_ o: Int) -> Int { Int(Int32(bitPattern: UInt32(b[o]) << 24 | UInt32(b[o + 1]) << 16 | UInt32(b[o + 2]) << 8 | UInt32(b[o + 3]))) }
        func f64(_ o: Int) -> Double {
            var v: UInt64 = 0
            for k in 0..<8 { v = v << 8 | UInt64(b[o + k]) }
            return Double(bitPattern: v)
        }
        guard i32(0) == 0x4F424D31 else { return nil }
        let w = i32(4), h = i32(8)
        guard w > 0, h > 0, w * h <= 4_000_000, b.count >= 36 + w * h else { return nil }
        let g = OccupancyGrid(width: w, height: h, resolution: f64(12), originX: f64(20), originY: f64(28))
        for i in 0..<(w * h) { g.logOdds[i] = Float(Int8(bitPattern: b[36 + i])) / 32 }
        g.version += 1
        return g
    }

    /// ROS map_server PGM: 254 free, 0 wall, 205 unknown, top row first.
    func pgm() -> Data {
        var d = Data("P5\n# OpenBot SLAM map\n\(width) \(height)\n255\n".utf8)
        for y in stride(from: height - 1, through: 0, by: -1) {
            for x in 0..<width {
                let v = logOdds[y * width + x]
                d.append(v > Self.occThreshold ? 0 : v < Self.freeThreshold ? 254 : 205)
            }
        }
        return d
    }

    func rosYaml(_ pgmName: String) -> String {
        "image: \(pgmName)\nresolution: \(resolution)\norigin: [\(originX), \(originY), 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
    }
}

// MARK: - SLAM

/// CoreSLAM-style scan matching (RMHC) with the phone gyro as heading prior,
/// and slam_toolbox-like Mapping / Localization modes. No loop closure.
final class SlamEngine {
    static let maxRange = 8.0, minRange = 0.15

    private(set) var grid: OccupancyGrid
    private(set) var pose = SlamPose.zero
    private var lastMapPose: SlamPose?
    private var lastStep = (0.0, 0.0)
    private var lastImuYaw = Double.nan
    var mapping = true
    private(set) var scans = 0
    private(set) var matchQuality = 0.0
    private(set) var trail: [(Double, Double)] = []
    private var rng = SeededRandom(seed: 42)
    var iterations = 250
    var sigmaXY = 0.08
    var sigmaTheta = 4.0 * .pi / 180

    init(grid: OccupancyGrid) { self.grid = grid }

    func reset() {
        grid.clear()
        resetPose(.zero)
        scans = 0
    }

    func setGrid(_ g: OccupancyGrid) {
        grid = g
        resetPose(.zero)
        scans = 1
    }

    func resetPose(_ p: SlamPose) {
        pose = p
        lastMapPose = nil
        lastStep = (0, 0)
        lastImuYaw = .nan
        trail.removeAll()
    }

    /// angles: radians, + left of the nose; ranges: metres; imuYaw: radians CCW or NaN.
    @discardableResult
    func update(angles: [Double], ranges: [Double], imuYaw: Double) -> SlamPose {
        var a: [Double] = [], r: [Double] = []
        for i in 0..<ranges.count where ranges[i] >= Self.minRange && ranges[i] < Self.maxRange {
            a.append(angles[i]); r.append(ranges[i])
        }
        var dTheta = 0.0
        if !imuYaw.isNaN {
            if !lastImuYaw.isNaN { dTheta = SlamPose.wrap(imuYaw - lastImuYaw) }
            lastImuYaw = imuYaw
        }
        let prior = SlamPose(pose.x + lastStep.0 * 0.6, pose.y + lastStep.1 * 0.6, pose.theta + dTheta)
        var best = prior
        if scans > 0 && a.count >= 20 { best = match(prior, a, r) }
        lastStep = (best.x - pose.x, best.y - pose.y)
        pose = best
        scans += 1
        if mapping && a.count >= 20 {
            let moved = lastMapPose.map { hypot($0.x - pose.x, $0.y - pose.y) > 0.04 || abs(SlamPose.wrap(pose.theta - $0.theta)) > 2 * .pi / 180 } ?? true
            if moved {
                grid.integrate(pose, angles: a, ranges: r, maxRange: Self.maxRange)
                lastMapPose = pose
            }
        }
        if let last = trail.last, hypot(last.0 - pose.x, last.1 - pose.y) <= 0.05 {} else {
            trail.append((pose.x, pose.y))
            if trail.count > 5000 { trail.removeFirst() }
        }
        return pose
    }

    func match(_ prior: SlamPose, _ a: [Double], _ r: [Double]) -> SlamPose {
        var bestScore = score(prior, a, r), best = prior
        var sxy = sigmaXY, sth = sigmaTheta, since = 0
        for _ in 0..<iterations {
            let cand = SlamPose(best.x + rng.gaussian() * sxy, best.y + rng.gaussian() * sxy, best.theta + rng.gaussian() * sth)
            let s = score(cand, a, r)
            if s > bestScore {
                bestScore = s; best = cand; since = 0
            } else {
                since += 1
                if since > 25 {
                    sxy *= 0.5; sth *= 0.5; since = 0
                    if sxy < grid.resolution * 0.1 { break }
                }
            }
        }
        matchQuality = hitFraction(best, a, r)
        return best
    }

    func score(_ p: SlamPose, _ a: [Double], _ r: [Double]) -> Double {
        let c = cos(p.theta), s = sin(p.theta)
        var total = 0.0
        for i in 0..<a.count {
            let lx = r[i] * cos(a[i]), ly = r[i] * sin(a[i])
            total += grid.hitScore(grid.cellX(p.x + c * lx - s * ly), grid.cellY(p.y + s * lx + c * ly))
        }
        return total
    }

    private func hitFraction(_ p: SlamPose, _ a: [Double], _ r: [Double]) -> Double {
        guard !a.isEmpty else { return 0 }
        let c = cos(p.theta), s = sin(p.theta)
        var hits = 0
        for i in 0..<a.count {
            let lx = r[i] * cos(a[i]), ly = r[i] * sin(a[i])
            let cx = grid.cellX(p.x + c * lx - s * ly), cy = grid.cellY(p.y + s * lx + c * ly)
            if grid.isOccupied(cx, cy) || grid.isOccupied(cx + 1, cy) || grid.isOccupied(cx - 1, cy)
                || grid.isOccupied(cx, cy + 1) || grid.isOccupied(cx, cy - 1) { hits += 1 }
        }
        return Double(hits) / Double(a.count)
    }
}

/// Deterministic random numbers (tests reproduce exactly), Box-Muller Gaussians.
struct SeededRandom {
    private var state: UInt64
    init(seed: UInt64) { state = seed &* 6364136223846793005 &+ 1442695040888963407 }
    mutating func next() -> Double {
        state = state &* 6364136223846793005 &+ 1442695040888963407
        return Double(state >> 11) / Double(1 << 53)
    }
    mutating func gaussian() -> Double {
        let u = max(next(), 1e-12), v = next()
        return sqrt(-2 * log(u)) * cos(2 * .pi * v)
    }
}

// MARK: - Navigation (Nav2-like: inflated costmap, A*, regulated pure pursuit, backup)

final class SlamNavigator {
    enum State { case idle, driving, blocked, backingUp, arrived, noPath }

    struct Params {
        var speed = 0.35, wheelbase = 0.26, maxSteerDeg = 28.0, lookahead = 0.55
        var robotRadius = 0.22, inflation = 0.45, goalTolerance = 0.25, stopDistance = 0.35
    }

    struct Command {
        var throttle = 0.0, steer = 0.0
        var state: State = .idle
        var message = ""
    }

    var params = Params()
    private(set) var goal: (Double, Double)?
    private(set) var path: [(Double, Double)] = []
    private(set) var state: State = .idle
    private var lastPlan = -1e9, blockedSince = -1.0, backUntil = -1.0, backSteer = 0.0
    // Multi-point turn when the path is behind (side chosen once).
    private var turnSide = 0.0, turnPhaseStart = 0.0, turnBacking = true

    func setGoal(_ x: Double, _ y: Double) {
        goal = (x, y); path = []; lastPlan = -1e9; blockedSince = -1; backUntil = -1; state = .driving; turnSide = 0
    }

    func cancel() { goal = nil; path = []; state = .idle }

    func step(pose: SlamPose, grid: OccupancyGrid, frontMin: Double, now: Double) -> Command {
        var c = Command()
        guard let g = goal else { state = .idle; return c }
        let toGoal = pose.distance(to: g.0, g.1)
        if toGoal < params.goalTolerance {
            state = .arrived; goal = nil; c.state = .arrived; c.message = "Arrived at the goal"; return c
        }
        if now < backUntil {
            state = .backingUp
            c.state = state; c.throttle = -params.speed * 0.7; c.steer = backSteer
            c.message = "Blocked: backing up to try again"
            return c
        }
        if frontMin < params.stopDistance {
            if blockedSince < 0 { blockedSince = now }
            if now - blockedSince > 2.5 {
                backUntil = now + 1; blockedSince = -1; lastPlan = -1e9
                if let la = lookahead(pose) { backSteer = -(angle(pose, la) > 0 ? 1 : -1) } else { backSteer = 0 }
            }
            state = .blocked; c.state = state
            c.message = String(format: "Obstacle %.2f m ahead: waiting", frontMin)
            return c
        }
        blockedSince = -1
        if path.isEmpty || now - lastPlan > 2 {
            lastPlan = now
            guard let p = Self.plan(grid, from: (pose.x, pose.y), to: g, params) else {
                state = .noPath; path = []; c.state = state; c.message = "No path to the goal on this map"; return c
            }
            path = p
        }
        let la = lookahead(pose) ?? g
        let alpha = angle(pose, la)
        let ld = max(0.15, pose.distance(to: la.0, la.1))
        state = .driving; c.state = state
        if turnSide == 0 && abs(alpha) > 100 * .pi / 180 {
            turnSide = alpha >= 0 ? 1 : -1; turnBacking = true; turnPhaseStart = now
        } else if turnSide != 0 && abs(alpha) < 60 * .pi / 180 {
            turnSide = 0
        }
        if turnSide != 0 {
            let roomAhead = frontMin > params.stopDistance * 1.6
            if now - turnPhaseStart > 1.6 || (!turnBacking && !roomAhead) {
                turnBacking = !turnBacking || !roomAhead
                turnPhaseStart = now
            }
            c.throttle = (turnBacking ? -0.6 : 0.5) * params.speed
            c.steer = turnSide
            c.message = "Turning round towards the path"
            return c
        }
        let delta = atan2(2 * params.wheelbase * sin(alpha), ld)
        let steer = min(1, max(-1, delta / (params.maxSteerDeg * .pi / 180)))
        var v = params.speed * min(1, max(0.4, 1 - 0.5 * abs(steer)))
        if toGoal < 0.8 { v *= max(0.45, toGoal / 0.8) }
        if frontMin < params.stopDistance * 2.5 { v *= 0.55 }
        c.throttle = v; c.steer = steer
        c.message = String(format: "Driving to the goal: %.1f m to go", toGoal)
        return c
    }

    private func lookahead(_ pose: SlamPose) -> (Double, Double)? {
        guard !path.isEmpty else { return nil }
        var nearest = 0, best = Double.greatestFiniteMagnitude
        for (i, p) in path.enumerated() {
            let d = pose.distance(to: p.0, p.1)
            if d < best { best = d; nearest = i }
        }
        for i in nearest..<path.count where pose.distance(to: path[i].0, path[i].1) >= params.lookahead { return path[i] }
        return path.last
    }

    private func angle(_ pose: SlamPose, _ p: (Double, Double)) -> Double {
        SlamPose.wrap(atan2(p.1 - pose.y, p.0 - pose.x) - pose.theta)
    }

    /// A* on a 10 cm inflated costmap with line-of-sight shortcuts; nil when unreachable.
    static func plan(_ grid: OccupancyGrid, from s: (Double, Double), to g: (Double, Double), _ p: Params) -> [(Double, Double)]? {
        let factor = max(1, Int((0.10 / grid.resolution).rounded()))
        let res = grid.resolution * Double(factor)
        let w = grid.width / factor, h = grid.height / factor
        var cost = [Float](repeating: 1, count: w * h)
        var occ = [Bool](repeating: false, count: w * h)
        for y in 0..<h {
            for x in 0..<w {
                var o = false, known = false
                outer: for dy in 0..<factor {
                    for dx in 0..<factor {
                        let v = grid.get(x * factor + dx, y * factor + dy)
                        if v > OccupancyGrid.occThreshold { o = true; break outer }
                        if v < OccupancyGrid.freeThreshold { known = true }
                    }
                }
                occ[y * w + x] = o
                if !o && !known { cost[y * w + x] = 3 }
            }
        }
        let rL = Int(ceil(p.robotRadius / res)), rS = Int(ceil((p.robotRadius + p.inflation) / res))
        for y in 0..<h {
            for x in 0..<w where occ[y * w + x] {
                for dy in -rS...rS {
                    for dx in -rS...rS {
                        let nx = x + dx, ny = y + dy
                        guard nx >= 0, ny >= 0, nx < w, ny < h else { continue }
                        let d = hypot(Double(dx), Double(dy)), i = ny * w + nx
                        if d <= Double(rL) { cost[i] = -1 }
                        else if d <= Double(rS) && cost[i] >= 0 {
                            cost[i] = max(cost[i], Float(1 + 8 * (1 - (d - Double(rL)) / (Double(rS - rL) + 1e-9))))
                        }
                    }
                }
            }
        }
        func cx(_ v: Double) -> Int { Int(floor((v - grid.originX) / res)) }
        func cy(_ v: Double) -> Int { Int(floor((v - grid.originY) / res)) }
        func free(_ x: Int, _ y: Int) -> Bool { x >= 0 && y >= 0 && x < w && y < h && cost[y * w + x] >= 0 }
        let x0 = cx(s.0), y0 = cy(s.1), x1 = cx(g.0), y1 = cy(g.1)
        guard free(x1, y1), x0 >= 0, y0 >= 0, x0 < w, y0 < h else { return nil }
        let n = w * h, start = y0 * w + x0, target = y1 * w + x1
        var gs = [Float](repeating: .greatestFiniteMagnitude, count: n)
        var from = [Int](repeating: -1, count: n)
        var closed = [Bool](repeating: false, count: n)
        gs[start] = 0
        func octile(_ ax: Int, _ ay: Int) -> Double {
            let dx = Double(abs(x1 - ax)), dy = Double(abs(y1 - ay))
            return max(dx, dy) + (sqrt(2) - 1) * min(dx, dy)
        }
        var open = MinHeap()
        open.push(octile(x0, y0), start)
        let dxs = [1, -1, 0, 0, 1, 1, -1, -1], dys = [0, 0, 1, -1, 1, -1, 1, -1]
        var expanded = 0
        while let (_, cur) = open.pop(), expanded < 400_000 {
            if closed[cur] { continue }
            closed[cur] = true
            expanded += 1
            if cur == target { break }
            let ux = cur % w, uy = cur / w
            for k in 0..<8 {
                let nx = ux + dxs[k], ny = uy + dys[k]
                guard free(nx, ny) else { continue }
                let ni = ny * w + nx
                if closed[ni] { continue }
                let ng = gs[cur] + Float((k < 4 ? 1 : sqrt(2)) * Double(cost[ni]))
                if ng < gs[ni] { gs[ni] = ng; from[ni] = cur; open.push(Double(ng) + octile(nx, ny), ni) }
            }
        }
        if from[target] < 0 && target != start { return nil }
        var cells: [(Int, Int)] = []
        var c = target
        while c != -1 {
            cells.insert((c % w, c / w), at: 0)
            if c == start { break }
            c = from[c]
        }
        func lineFree(_ a: (Int, Int), _ b: (Int, Int)) -> Bool {
            let steps = max(abs(b.0 - a.0), abs(b.1 - a.1))
            for st in 0...max(0, steps) {
                let t = steps == 0 ? 0 : Double(st) / Double(steps)
                let x = Int((Double(a.0) + Double(b.0 - a.0) * t).rounded()), y = Int((Double(a.1) + Double(b.1 - a.1) * t).rounded())
                if !free(x, y) || cost[y * w + x] > 4 { return false }
            }
            return true
        }
        var out: [(Double, Double)] = [s]
        var i = 0
        while i < cells.count - 1 {
            var j = cells.count - 1
            while j > i + 1 && !lineFree(cells[i], cells[j]) { j -= 1 }
            out.append((grid.originX + (Double(cells[j].0) + 0.5) * res, grid.originY + (Double(cells[j].1) + 0.5) * res))
            i = j
        }
        out[out.count - 1] = g
        var dense: [(Double, Double)] = []
        for k in 0..<(out.count - 1) {
            let a = out[k], b = out[k + 1]
            let m = max(1, Int(ceil(hypot(b.0 - a.0, b.1 - a.1) / 0.1)))
            for q in 0..<m {
                let t = Double(q) / Double(m)
                dense.append((a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t))
            }
        }
        dense.append(out[out.count - 1])
        return dense
    }
}

/// Binary min-heap of (priority, index) for A*.
struct MinHeap {
    private var items: [(Double, Int)] = []
    mutating func push(_ p: Double, _ v: Int) {
        items.append((p, v))
        var i = items.count - 1
        while i > 0 {
            let parent = (i - 1) / 2
            if items[parent].0 <= items[i].0 { break }
            items.swapAt(parent, i); i = parent
        }
    }
    mutating func pop() -> (Double, Int)? {
        guard !items.isEmpty else { return nil }
        let top = items[0]
        let last = items.removeLast()
        if !items.isEmpty {
            items[0] = last
            var i = 0
            while true {
                let l = 2 * i + 1, r = l + 1
                var m = i
                if l < items.count && items[l].0 < items[m].0 { m = l }
                if r < items.count && items[r].0 < items[m].0 { m = r }
                if m == i { break }
                items.swapAt(i, m); i = m
            }
        }
        return top
    }
}
