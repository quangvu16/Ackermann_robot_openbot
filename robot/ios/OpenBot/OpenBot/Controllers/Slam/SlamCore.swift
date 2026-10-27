//
//  SlamCore.swift
//  OpenBot
//
//  Indoor SLAM and navigation for the car. The same algorithms as the Android
//  classes in org.openbot.app.robot.slam (OccupancyGrid, ScanMatcher,
//  FastCorrelativeMatcher, PoseGraph, SlamEngine, Navigator) - keep them in
//  step. SLAM follows Google Cartographer (submaps, correlative + least-squares
//  scan matching, branch-and-bound loop closure, sparse pose adjustment), from
//  Hess et al. ICRA 2016 and Cartographer's defaults (Apache-2.0); navigation
//  follows Nav2. Written for this project; no code from those projects is used.
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
    func distance(to p: SlamPose) -> Double { hypot(p.x - x, p.y - y) }

    /// self followed by o (o in self's frame).
    func compose(_ o: SlamPose) -> SlamPose {
        let c = cos(theta), s = sin(theta)
        return SlamPose(x + c * o.x - s * o.y, y + s * o.x + c * o.y, theta + o.theta)
    }

    var inverse: SlamPose {
        let c = cos(theta), s = sin(theta)
        return SlamPose(-c * x - s * y, s * x - c * y, -theta)
    }

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

    // MARK: Probabilities for the scan matchers (Cartographer style)

    /// Cartographer's kMinProbability / kMaxProbability; never-seen cells count as 0.1.
    static let pMin: Float = 0.1, pMax: Float = 0.9
    private static let probLUT: [Float] = (0..<449).map { i in
        let l = Double(OccupancyGrid.lMin) + Double(i) / 64
        return Float(min(Double(OccupancyGrid.pMax), max(Double(OccupancyGrid.pMin), 1 / (1 + exp(-l)))))
    }
    private var probCache: [Float] = []
    private var probVersion = -1

    static func probability(_ l: Float) -> Float {
        if l == 0 { return pMin }
        let i = Int(((l - lMin) * 64).rounded())
        return probLUT[min(probLUT.count - 1, max(0, i))]
    }

    /// All cells as probabilities, cached until the map changes.
    func probabilities() -> [Float] {
        if probVersion != version || probCache.count != logOdds.count {
            probCache = logOdds.map { Self.probability($0) }
            probVersion = version
        }
        return probCache
    }

    func copy() -> OccupancyGrid {
        let g = OccupancyGrid(width: width, height: height, resolution: resolution, originX: originX, originY: originY)
        g.logOdds = logOdds
        g.version = version + 1
        return g
    }

    func worldX(_ cx: Int) -> Double { originX + (Double(cx) + 0.5) * resolution }
    func worldY(_ cy: Int) -> Double { originY + (Double(cy) + 0.5) * resolution }

    /// Paints a submap into this grid at srcToThis (submap coordinates = its grid's minus its origin).
    func addSubmap(_ src: OccupancyGrid, originX sox: Double, originY soy: Double, at t: SlamPose, scale: Float) {
        let c = cos(t.theta), s = sin(t.theta)
        let corners = [(src.originX, src.originY), (src.originX + Double(src.width) * src.resolution, src.originY),
                       (src.originX, src.originY + Double(src.height) * src.resolution),
                       (src.originX + Double(src.width) * src.resolution, src.originY + Double(src.height) * src.resolution)]
        var minX = Double.greatestFiniteMagnitude, minY = minX, maxX = -minX, maxY = -minX
        for k in corners {
            let lx = k.0 - sox, ly = k.1 - soy
            let gx = t.x + c * lx - s * ly, gy = t.y + s * lx + c * ly
            minX = min(minX, gx); maxX = max(maxX, gx); minY = min(minY, gy); maxY = max(maxY, gy)
        }
        let x0 = max(0, cellX(minX)), x1 = min(width - 1, cellX(maxX))
        let y0 = max(0, cellY(minY)), y1 = min(height - 1, cellY(maxY))
        guard x0 <= x1, y0 <= y1 else { return }
        for cy in y0...y1 {
            let wy = worldY(cy) - t.y
            for cx in x0...x1 {
                let wx = worldX(cx) - t.x
                let sx = src.cellX(c * wx + s * wy + sox), sy = src.cellY(-s * wx + c * wy + soy)
                guard src.inBounds(sx, sy) else { continue }
                let v = src.logOdds[sy * src.width + sx]
                if v == 0 { continue }
                let i = cy * width + cx
                logOdds[i] = min(Self.lMax, max(Self.lMin, logOdds[i] + scale * v))
            }
        }
        version += 1
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

// MARK: - Local scan matching (Cartographer: real-time correlative + "Ceres")

/// Brute-force search of a small window, then least squares on a bicubic map.
final class ScanMatcher {
    var linearWindow = 0.15                 // Cartographer 0.1 m; no wheel odometry here
    var angularWindow = 20.0 * .pi / 180
    var translationDeltaCostWeight = 0.1, rotationDeltaCostWeight = 0.1
    // Cartographer uses 1 / 10 / 40 towards an odometry prediction; ours pull towards
    // the correlative answer (good to one cell), so the springs are 10x softer.
    var occupiedSpaceWeight = 1.0, translationWeight = 1.0, rotationWeight = 4.0
    var maxIterations = 20
    private(set) var lastScore = 0.0
    /// Weakest/strongest eigenvalue of the 2x2 position information (0 = could slide).
    private(set) var lastConstraint = 0.0

    func match(_ g: OccupancyGrid, _ initial: SlamPose, _ px: [Double], _ py: [Double]) -> SlamPose {
        refine(g, correlative(g, initial, px, py), px, py)
    }

    static func angularStep(_ res: Double, _ px: [Double], _ py: [Double]) -> Double {
        var maxR = 0.0
        for i in 0..<px.count { maxR = max(maxR, hypot(px[i], py[i])) }
        if maxR < res { return 5 * .pi / 180 }
        return acos(min(1, max(-1, 1 - res * res / (2 * maxR * maxR))))
    }

    func correlative(_ g: OccupancyGrid, _ initial: SlamPose, _ px: [Double], _ py: [Double]) -> SlamPose {
        let n = px.count
        guard n > 0 else { return initial }
        let prob = g.probabilities(), res = g.resolution
        let step = Self.angularStep(res, px, py)
        let nAng = min(80, Int(ceil(angularWindow / step))), nLin = Int(ceil(linearWindow / res))
        var cx = [Int](repeating: 0, count: n), cy = [Int](repeating: 0, count: n)
        var bestScore = -1.0, best = initial
        for k in -nAng...nAng {
            let th = initial.theta + Double(k) * step
            let c = cos(th), s = sin(th)
            for i in 0..<n {
                cx[i] = g.cellX(initial.x + c * px[i] - s * py[i])
                cy[i] = g.cellY(initial.y + s * px[i] + c * py[i])
            }
            for ox in -nLin...nLin {
                for oy in -nLin...nLin {
                    var sum = 0.0
                    for i in 0..<n {
                        let x = cx[i] + ox, y = cy[i] + oy
                        sum += (x >= 0 && y >= 0 && x < g.width && y < g.height) ? Double(prob[y * g.width + x]) : Double(OccupancyGrid.pMin)
                    }
                    let pen = translationDeltaCostWeight * hypot(Double(ox) * res, Double(oy) * res) + rotationDeltaCostWeight * abs(Double(k) * step)
                    let score = sum / Double(n) * exp(-pen * pen)
                    if score > bestScore {
                        bestScore = score
                        best = SlamPose(initial.x + Double(ox) * res, initial.y + Double(oy) * res, th)
                    }
                }
            }
        }
        lastScore = bestScore
        return best
    }

    func refine(_ g: OccupancyGrid, _ target: SlamPose, _ px: [Double], _ py: [Double]) -> SlamPose {
        refine(g, target, px, py, wOcc: occupiedSpaceWeight, wT: translationWeight, wR: rotationWeight)
    }

    func refine(_ g: OccupancyGrid, _ target: SlamPose, _ px: [Double], _ py: [Double], wOcc: Double, wT: Double, wR: Double) -> SlamPose {
        let n = px.count
        guard n > 0 else { return target }
        let prob = g.probabilities()
        var x = target.x, y = target.y, th = target.theta
        var lambda = 1e-3
        var cost = Self.cost(g, prob, px, py, x, y, th, target, wOcc, wT, wR)
        let scale = wOcc / Double(n).squareRoot()
        for _ in 0..<maxIterations {
            var h = [[Double]](repeating: [0, 0, 0], count: 3), b = [0.0, 0.0, 0.0]
            func acc(_ j: [Double], _ r: Double) {
                for a in 0..<3 { b[a] += j[a] * r; for k in 0..<3 { h[a][k] += j[a] * j[k] } }
            }
            let c = cos(th), s = sin(th)
            for i in 0..<n {
                let v = Self.bicubic(g, prob, x + c * px[i] - s * py[i], y + s * px[i] + c * py[i])
                let dwxdt = -s * px[i] - c * py[i], dwydt = c * px[i] - s * py[i]
                acc([-scale * v.1, -scale * v.2, -scale * (v.1 * dwxdt + v.2 * dwydt)], scale * (1 - v.0))
            }
            acc([wT, 0, 0], wT * (x - target.x))
            acc([0, wT, 0], wT * (y - target.y))
            acc([0, 0, wR], wR * SlamPose.wrap(th - target.theta))
            var accepted = false, converged = false
            for _ in 0..<6 where !accepted {
                var a = h
                for r in 0..<3 { a[r][r] += lambda * (h[r][r] + 1e-9) }
                guard let d = Self.solve3(a, [-b[0], -b[1], -b[2]]) else { break }
                let nc = Self.cost(g, prob, px, py, x + d[0], y + d[1], th + d[2], target, wOcc, wT, wR)
                if nc < cost {
                    converged = cost - nc < 1e-10
                    x += d[0]; y += d[1]; th += d[2]; cost = nc
                    lambda = max(1e-7, lambda * 0.3)
                    accepted = true
                } else {
                    lambda *= 10
                }
            }
            if !accepted || converged { break }
        }
        let result = SlamPose(x, y, th)
        lastScore = Self.meanProbability(g, result, px, py)
        lastConstraint = Self.translationConstraint(g, prob, result, px, py)
        return result
    }

    private static func cost(_ g: OccupancyGrid, _ prob: [Float], _ px: [Double], _ py: [Double], _ x: Double, _ y: Double,
                             _ th: Double, _ t: SlamPose, _ wOcc: Double, _ wT: Double, _ wR: Double) -> Double {
        let c = cos(th), s = sin(th)
        var sum = 0.0
        for i in 0..<px.count {
            let r = 1 - bicubic(g, prob, x + c * px[i] - s * py[i], y + s * px[i] + c * py[i]).0
            sum += r * r
        }
        sum *= wOcc * wOcc / Double(px.count)
        let dx = x - t.x, dy = y - t.y, dt = SlamPose.wrap(th - t.theta)
        return sum + wT * wT * (dx * dx + dy * dy) + wR * wR * dt * dt
    }

    static func meanProbability(_ g: OccupancyGrid, _ p: SlamPose, _ px: [Double], _ py: [Double]) -> Double {
        guard !px.isEmpty else { return 0 }
        let prob = g.probabilities(), c = cos(p.theta), s = sin(p.theta)
        var sum = 0.0
        for i in 0..<px.count {
            let x = g.cellX(p.x + c * px[i] - s * py[i]), y = g.cellY(p.y + s * px[i] + c * py[i])
            sum += g.inBounds(x, y) ? Double(prob[y * g.width + x]) : Double(OccupancyGrid.pMin)
        }
        return sum / Double(px.count)
    }

    static func translationConstraint(_ g: OccupancyGrid, _ prob: [Float], _ p: SlamPose, _ px: [Double], _ py: [Double]) -> Double {
        let c = cos(p.theta), s = sin(p.theta)
        var hxx = 0.0, hxy = 0.0, hyy = 0.0
        for i in 0..<px.count {
            let v = bicubic(g, prob, p.x + c * px[i] - s * py[i], p.y + s * px[i] + c * py[i])
            hxx += v.1 * v.1; hxy += v.1 * v.2; hyy += v.2 * v.2
        }
        let tr = hxx + hyy, det = hxx * hyy - hxy * hxy
        let disc = max(0, tr * tr / 4 - det).squareRoot()
        let hi = tr / 2 + disc, lo = tr / 2 - disc
        return hi <= 1e-12 ? 0 : max(0, lo) / hi
    }

    static func solve3(_ a: [[Double]], _ b: [Double]) -> [Double]? {
        func det(_ m: [[Double]]) -> Double {
            m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
        }
        let d = det(a)
        guard abs(d) > 1e-300 else { return nil }
        return (0..<3).map { col in
            var m = a
            for r in 0..<3 { m[r][col] = b[r] }
            return det(m) / d
        }
    }

    private static func cell(_ g: OccupancyGrid, _ prob: [Float], _ x: Int, _ y: Int) -> Double {
        (x >= 0 && y >= 0 && x < g.width && y < g.height) ? Double(prob[y * g.width + x]) : Double(OccupancyGrid.pMin)
    }

    /// Catmull-Rom bicubic interpolation (as Ceres' BiCubicInterpolator): value, d/dwx, d/dwy.
    static func bicubic(_ g: OccupancyGrid, _ prob: [Float], _ wx: Double, _ wy: Double) -> (Double, Double, Double) {
        let u = (wx - g.originX) / g.resolution - 0.5, v = (wy - g.originY) / g.resolution - 0.5
        let x0 = Int(floor(u)), y0 = Int(floor(v))
        let tx = u - Double(x0), ty = v - Double(y0)
        var f = [0.0, 0.0, 0.0, 0.0], fd = [0.0, 0.0, 0.0, 0.0]
        for j in 0..<4 {
            let yy = y0 - 1 + j
            let r = cubic(cell(g, prob, x0 - 1, yy), cell(g, prob, x0, yy), cell(g, prob, x0 + 1, yy), cell(g, prob, x0 + 2, yy), tx)
            f[j] = r.0; fd[j] = r.1
        }
        let val = cubic(f[0], f[1], f[2], f[3], ty), du = cubic(fd[0], fd[1], fd[2], fd[3], ty).0
        return (val.0, du / g.resolution, val.1 / g.resolution)
    }

    private static func cubic(_ p0: Double, _ p1: Double, _ p2: Double, _ p3: Double, _ t: Double) -> (Double, Double) {
        let a = (-p0 + 3 * p1 - 3 * p2 + p3) / 2, b = (2 * p0 - 5 * p1 + 4 * p2 - p3) / 2, c = (-p0 + p2) / 2
        return (((a * t + b) * t + c) * t + p1, (3 * a * t + 2 * b) * t + c)
    }
}

// MARK: - Branch and bound (Cartographer's fast correlative scan matcher)

/// Coarser copies of the map where each cell holds the best of the 2^h x 2^h cells it
/// covers; whole blocks of poses are skipped once a better answer is known.
final class FastCorrelativeMatcher {
    static let depth = 7 // Cartographer branch_and_bound_depth
    private let grid: OccupancyGrid
    private var levels: [[UInt8]] = []
    private var levelWidth: [Int] = [], levelHeight: [Int] = []

    init(_ grid: OccupancyGrid) {
        self.grid = grid
        let prob = grid.probabilities(), w = grid.width, h = grid.height
        let base = prob.map { Int(($0 * 255).rounded()) }
        for lev in 0..<Self.depth {
            let win = 1 << lev, lw = w + win - 1, lh = h + win - 1
            levelWidth.append(lw); levelHeight.append(lh)
            var rows = [Int](repeating: 0, count: lw * h)
            for y in 0..<h {
                let row = Self.slidingMax(Array(base[(y * w)..<(y * w + w)]), win)
                for x in 0..<lw { rows[y * lw + x] = row[x] }
            }
            var out = [UInt8](repeating: 0, count: lw * lh)
            var src = [Int](repeating: 0, count: h)
            for x in 0..<lw {
                for y in 0..<h { src[y] = rows[y * lw + x] }
                let col = Self.slidingMax(src, win)
                for y in 0..<lh { out[y * lw + x] = UInt8(col[y]) }
            }
            levels.append(out)
        }
    }

    /// out[k] = max of input over [k - win + 1, k] (outside = 0), k = 0 ..< n + win - 1.
    private static func slidingMax(_ input: [Int], _ win: Int) -> [Int] {
        let n = input.count, m = n + win - 1
        var out = [Int](repeating: 0, count: m)
        var dq = [Int](repeating: 0, count: n)
        var head = 0, tail = 0
        for k in 0..<m {
            if k < n {
                while tail > head && input[dq[tail - 1]] <= input[k] { tail -= 1 }
                dq[tail] = k; tail += 1
            }
            while tail > head && dq[head] < k - win + 1 { head += 1 }
            out[k] = tail > head ? input[dq[head]] : 0
        }
        return out
    }

    private func value(_ lev: Int, _ cx: Int, _ cy: Int) -> Int {
        let off = (1 << lev) - 1, x = cx + off, y = cy + off
        guard x >= 0, y >= 0, x < levelWidth[lev], y < levelHeight[lev] else { return 0 }
        return Int(levels[lev][y * levelWidth[lev] + x])
    }

    private struct Candidate { let angle: Int, ox: Int, oy: Int; var score = 0.0 }

    /// Best pose within the windows of `initial`, or nil if nothing beats minScore (0..1).
    func match(_ initial: SlamPose, _ px: [Double], _ py: [Double], linearWindow: Double, angularWindow: Double,
               minScore: Double) -> (pose: SlamPose, score: Double)? {
        let n = px.count
        guard n > 0 else { return nil }
        let res = grid.resolution, step = ScanMatcher.angularStep(res, px, py)
        var nAng = Int(ceil(angularWindow / step)), angles = 2 * nAng + 1
        if angularWindow >= .pi - 1e-9 { angles = Int(ceil(2 * .pi / step)); nAng = angles / 2 }
        var sx = [[Int]](), sy = [[Int]](), th = [Double]()
        for k in 0..<angles {
            let t = initial.theta + Double(k - nAng) * step
            th.append(t)
            let c = cos(t), s = sin(t)
            sx.append((0..<n).map { grid.cellX(initial.x + c * px[$0] - s * py[$0]) })
            sy.append((0..<n).map { grid.cellY(initial.y + s * px[$0] + c * py[$0]) })
        }
        let w = Int(ceil(linearWindow / res)), top = Self.depth - 1, stepTop = 1 << top
        var cands: [Candidate] = []
        for k in 0..<angles {
            for ox in stride(from: -w, through: w, by: stepTop) {
                for oy in stride(from: -w, through: w, by: stepTop) { cands.append(Candidate(angle: k, ox: ox, oy: oy)) }
            }
        }
        score(&cands, top, sx, sy, n)
        cands.sort { $0.score > $1.score }
        var best = minScore * 255
        var found: Candidate?
        search(cands, top, sx, sy, n, w, &best, &found)
        guard let c = found else { return nil }
        return (SlamPose(initial.x + Double(c.ox) * res, initial.y + Double(c.oy) * res, th[c.angle]), c.score / 255)
    }

    private func score(_ cands: inout [Candidate], _ lev: Int, _ sx: [[Int]], _ sy: [[Int]], _ n: Int) {
        for i in cands.indices {
            let xs = sx[cands[i].angle], ys = sy[cands[i].angle], ox = cands[i].ox, oy = cands[i].oy
            var sum = 0
            for k in 0..<n { sum += value(lev, xs[k] + ox, ys[k] + oy) }
            cands[i].score = Double(sum) / Double(n)
        }
    }

    private func search(_ cands: [Candidate], _ lev: Int, _ sx: [[Int]], _ sy: [[Int]], _ n: Int, _ w: Int,
                        _ best: inout Double, _ found: inout Candidate?) {
        for c in cands {
            if c.score <= best { return }
            if lev == 0 { best = c.score; found = c; return }
            let half = 1 << (lev - 1)
            var kids: [Candidate] = []
            for dx in [0, half] where c.ox + dx <= w {
                for dy in [0, half] where c.oy + dy <= w { kids.append(Candidate(angle: c.angle, ox: c.ox + dx, oy: c.oy + dy)) }
            }
            score(&kids, lev - 1, sx, sy, n)
            kids.sort { $0.score > $1.score }
            search(kids, lev - 1, sx, sy, n, w, &best, &found)
        }
    }
}

// MARK: - Pose graph (sparse pose adjustment over submaps)

final class PoseGraph {
    static let seqT = 5e2, seqR = 1.6e3       // matcher_translation/rotation_weight
    static let loopT = 1.1e4, loopR = 1e5     // loop_closure_translation/rotation_weight
    static let huber = 10.0                   // optimization_problem.huber_scale
    static let priorT = 1e1, priorR = 1e2     // fixed_frame_pose weights

    struct Constraint {
        let i: Int, j: Int // j == -1: a prior on i
        let relative: SlamPose
        let wT: Double, wR: Double
        let robust: Bool
    }

    private(set) var poses: [SlamPose] = []
    private var fixed: [Bool] = []
    private(set) var constraints: [Constraint] = []
    var maxIterations = 50

    @discardableResult
    func add(_ p: SlamPose, fixed f: Bool) -> Int { poses.append(p); fixed.append(f); return poses.count - 1 }
    func addConstraint(_ c: Constraint) { constraints.append(c) }
    func clear() { poses = []; fixed = []; constraints = [] }
    var size: Int { poses.count }

    private static func error(_ c: Constraint, _ p: [SlamPose]) -> (Double, Double, Double) {
        let a = p[c.i]
        if c.j < 0 { return (a.x - c.relative.x, a.y - c.relative.y, SlamPose.wrap(a.theta - c.relative.theta)) }
        let b = p[c.j], co = cos(a.theta), si = sin(a.theta), dx = b.x - a.x, dy = b.y - a.y
        return (co * dx + si * dy - c.relative.x, -si * dx + co * dy - c.relative.y, SlamPose.wrap(b.theta - a.theta - c.relative.theta))
    }

    func cost() -> Double { cost(poses) }

    private func cost(_ p: [SlamPose]) -> Double {
        var sum = 0.0
        let h2 = Self.huber * Self.huber
        for c in constraints {
            let e = Self.error(c, p)
            let s = pow(c.wT * e.0, 2) + pow(c.wT * e.1, 2) + pow(c.wR * e.2, 2)
            sum += c.robust && s > h2 ? 2 * Self.huber * s.squareRoot() - h2 : s
        }
        return sum
    }

    /// Drops loop closures the poses still miss by more than maxT / maxR (look-alike places).
    @discardableResult
    func removeOutliers(maxT: Double, maxR: Double) -> Int {
        let before = constraints.count
        constraints.removeAll { c in
            guard c.robust else { return false }
            let e = Self.error(c, poses)
            return hypot(e.0, e.1) > maxT || abs(e.2) > maxR
        }
        return before - constraints.count
    }

    /// Levenberg-Marquardt with a dense Cholesky solve.
    @discardableResult
    func optimize() -> Int {
        let n = poses.count
        var index = [Int](repeating: -1, count: n), m = 0
        for k in 0..<n where !fixed[k] { index[k] = m; m += 1 }
        guard m > 0 else { return 0 }
        let dim = 3 * m
        var lambda = 1e-4, current = cost(poses), it = 0
        while it < maxIterations {
            var h = [Double](repeating: 0, count: dim * dim), g = [Double](repeating: 0, count: dim)
            for c in constraints {
                let e = Self.error(c, poses)
                var ji = [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]], jj = [[0.0, 0, 0], [0, 0, 0], [0, 0, 0]]
                if c.j >= 0 {
                    let a = poses[c.i], b = poses[c.j], co = cos(a.theta), si = sin(a.theta), dx = b.x - a.x, dy = b.y - a.y
                    ji = [[-co, -si, -si * dx + co * dy], [si, -co, -co * dx - si * dy], [0, 0, -1]]
                    jj = [[co, si, 0], [-si, co, 0], [0, 0, 1]]
                }
                let w = [c.wT, c.wT, c.wR]
                let r = [w[0] * e.0, w[1] * e.1, w[2] * e.2]
                var rho = 1.0
                if c.robust {
                    let s = r[0] * r[0] + r[1] * r[1] + r[2] * r[2]
                    if s > Self.huber * Self.huber { rho = Self.huber / s.squareRoot() }
                }
                for row in 0..<3 { for col in 0..<3 { ji[row][col] *= w[row]; jj[row][col] *= w[row] } }
                let bi = index[c.i], bj = c.j < 0 ? -1 : index[c.j]
                func block(_ p: Int, _ ja: [[Double]], _ q: Int, _ jb: [[Double]], gradient: Bool) {
                    for a in 0..<3 {
                        if gradient { g[3 * p + a] += rho * (ja[0][a] * r[0] + ja[1][a] * r[1] + ja[2][a] * r[2]) }
                        for b in 0..<3 {
                            h[(3 * p + a) * dim + 3 * q + b] += rho * (ja[0][a] * jb[0][b] + ja[1][a] * jb[1][b] + ja[2][a] * jb[2][b])
                        }
                    }
                }
                if bi >= 0 { block(bi, ji, bi, ji, gradient: true) }
                if bj >= 0 {
                    block(bj, jj, bj, jj, gradient: true)
                    if bi >= 0 { block(bi, ji, bj, jj, gradient: false); block(bj, jj, bi, ji, gradient: false) }
                }
            }
            var accepted = false, converged = false
            for _ in 0..<8 where !accepted {
                var a = h
                for k in 0..<dim { a[k * dim + k] += lambda * h[k * dim + k] + 1e-9 }
                guard let d = Self.cholSolve(a, g.map { -$0 }, dim) else { lambda *= 10; continue }
                var trial = poses
                for k in 0..<n where index[k] >= 0 {
                    let b = index[k], p = poses[k]
                    trial[k] = SlamPose(p.x + d[3 * b], p.y + d[3 * b + 1], p.theta + d[3 * b + 2])
                }
                let nc = cost(trial)
                if nc < current {
                    converged = current - nc < 1e-9 * max(1, current)
                    poses = trial; current = nc
                    lambda = max(1e-9, lambda * 0.3)
                    accepted = true
                } else {
                    lambda *= 10
                }
            }
            it += 1
            if !accepted || converged { break }
        }
        return it
    }

    /// Solves A x = b (A symmetric positive definite, row-major n x n); nil if it isn't.
    static func cholSolve(_ a: [Double], _ b: [Double], _ n: Int) -> [Double]? {
        var l = [Double](repeating: 0, count: n * n)
        for i in 0..<n {
            for j in 0...i {
                var s = a[i * n + j]
                for k in 0..<j { s -= l[i * n + k] * l[j * n + k] }
                if i == j {
                    if s <= 0 { return nil }
                    l[i * n + i] = s.squareRoot()
                } else {
                    l[i * n + j] = s / l[j * n + j]
                }
            }
        }
        var y = [Double](repeating: 0, count: n)
        for i in 0..<n {
            var s = b[i]
            for k in 0..<i { s -= l[i * n + k] * y[k] }
            y[i] = s / l[i * n + i]
        }
        var x = [Double](repeating: 0, count: n)
        for i in stride(from: n - 1, through: 0, by: -1) {
            var s = y[i]
            for k in (i + 1)..<max(i + 1, n) { s -= l[k * n + i] * x[k] }
            x[i] = s / l[i * n + i]
        }
        return x
    }
}

// MARK: - SLAM (Cartographer-style: submaps, loop closure, global localization)

final class SlamEngine {
    static let maxRange = 8.0, minRange = 0.15
    // Cartographer trajectory_builder_2d.lua
    static let numRangeData = 90
    static let motionMaxDistance = 0.2, motionMaxAngle = 1.0 * .pi / 180, motionMaxTime = 5.0
    static let submapCells = 280, newSubmapDistance = 2.0
    // Cartographer pose_graph.lua
    static let samplingRatio = 0.3, maxConstraintDistance = 15.0, minScore = 0.55, globalLocalizationMinScore = 0.6
    static let loopLinearWindow = 7.0, loopAngularWindow = 30.0 * .pi / 180
    // Not in Cartographer (indoors places repeat): window grows with distance driven; outlier and slide checks.
    static func loopWindow(_ drivenSince: Double) -> Double { min(loopLinearWindow, 0.5 + 0.1 * abs(drivenSince)) }
    static let outlierT = 0.3, outlierR = 5.0 * .pi / 180, minConstraint = 0.3
    var optimizeEveryNNodes = 30
    var loopSearchesPerScan = 2

    final class Submap {
        let grid: OccupancyGrid
        let ox: Double, oy: Double, session: Int, frozen: Bool
        var driven = 0.0, inserts = 0, finished = false, graphIndex = 0
        var matcher: FastCorrelativeMatcher?
        init(grid: OccupancyGrid, ox: Double, oy: Double, session: Int, frozen: Bool) {
            self.grid = grid; self.ox = ox; self.oy = oy; self.session = session; self.frozen = frozen
        }
    }

    struct Node {
        let local: SlamPose
        let submap: Int
        let inSubmaps: [Int]
        let lx: [Double], ly: [Double]
        let driven: Double
    }

    private(set) var grid: OccupancyGrid
    private(set) var pose = SlamPose.zero
    private var localPose = SlamPose.zero
    private var lastInsertPose: SlamPose?
    private var lastInsertTime = -1e9
    private var lastStep = (0.0, 0.0)
    private var driven = 0.0
    private var lastImuYaw = Double.nan
    private var isMapping = true
    private(set) var scans = 0
    private(set) var matchQuality = 0.0
    private var lowQualityScans = 0
    private(set) var trail: [(Double, Double)] = []
    private var rng = SeededRandom(seed: 42)
    let matcher = ScanMatcher()
    private let graph = PoseGraph()
    private var submaps: [Submap] = []
    private var active: [Submap] = []
    private var nodes: [Node] = []
    private var loopQueue: [(Int, Int)] = []
    private var cachedMatchers: [Int] = [] // submap indices whose tables are kept (newest last)
    private var base: Submap?
    private var session = 0
    private var nodesSinceOptimize = 0, newLoops = 0
    private(set) var loopClosures = 0
    private var relocalizeRequested = false
    private(set) var relocalizeSeq = 0
    private(set) var relocalizeFound = false
    private var globalMatcher: FastCorrelativeMatcher?
    private var globalMatcherKey: (ObjectIdentifier, Int)?

    init(grid: OccupancyGrid) { self.grid = grid }

    var submapCount: Int { submaps.count }

    /// Mapping builds submaps; localization only matches against the map.
    var mapping: Bool {
        get { isMapping }
        set {
            guard newValue != isMapping else { return }
            isMapping = newValue
            if newValue { startSession() } else { finishActive() }
        }
    }

    func requestGlobalLocalization() { relocalizeRequested = true }

    func reset() {
        let g = grid
        clearGraph()
        grid = OccupancyGrid(width: g.width, height: g.height, resolution: g.resolution, originX: g.originX, originY: g.originY)
        resetPose(.zero)
        scans = 0
        trail.removeAll()
    }

    func setGrid(_ loaded: OccupancyGrid) {
        clearGraph()
        let b = Submap(grid: loaded, ox: 0, oy: 0, session: -1, frozen: true)
        b.finished = true
        b.graphIndex = graph.add(.zero, fixed: true)
        base = b
        grid = loaded.copy()
        resetPose(.zero)
        scans = 1
        trail.removeAll()
    }

    func resetPose(_ p: SlamPose) {
        pose = p
        localPose = p
        lastStep = (0, 0)
        lastImuYaw = .nan
        lowQualityScans = 0
        if isMapping { startSession() }
    }

    private func clearGraph() {
        graph.clear(); submaps = []; active = []; nodes = []; loopQueue = []; cachedMatchers = []
        base = nil; nodesSinceOptimize = 0; newLoops = 0; loopClosures = 0; globalMatcher = nil; globalMatcherKey = nil
    }

    private func startSession() {
        finishActive()
        session += 1
        localPose = pose
        lastInsertPose = nil
    }

    private func finishActive() {
        for s in active { finish(s) }
        active = []
    }

    /// angles: radians, + left of the nose; ranges: metres; imuYaw: radians CCW or NaN; time: seconds.
    @discardableResult
    func update(angles: [Double], ranges: [Double], imuYaw: Double, time: Double? = nil) -> SlamPose {
        let t = time ?? Double(scans) * 0.1
        var a: [Double] = [], r: [Double] = [], px: [Double] = [], py: [Double] = []
        for i in 0..<ranges.count where ranges[i] >= Self.minRange && ranges[i] < Self.maxRange {
            a.append(angles[i]); r.append(ranges[i])
            px.append(ranges[i] * cos(angles[i])); py.append(ranges[i] * sin(angles[i]))
        }
        let n = a.count
        let mp = Self.adaptiveVoxel(px, py, maxLength: 0.5, minPoints: 200)
        var dTheta = 0.0
        if !imuYaw.isNaN {
            if !lastImuYaw.isNaN { dTheta = SlamPose.wrap(imuYaw - lastImuYaw) }
            lastImuYaw = imuYaw
        }
        if relocalizeRequested && n >= 20 {
            relocalizeRequested = false
            globalLocalize(mp.0, mp.1)
        }
        if isMapping {
            let prior = SlamPose(localPose.x + lastStep.0 * 0.6, localPose.y + lastStep.1 * 0.6, localPose.theta + dTheta)
            var best = prior
            if let ms = active.first, n >= 20 {
                best = matcher.match(ms.grid, prior, mp.0, mp.1)
                matchQuality = Self.hitFraction(ms.grid, best, px, py)
            }
            lastStep = (best.x - localPose.x, best.y - localPose.y)
            driven += hypot(lastStep.0, lastStep.1)
            localPose = best
            pose = localToGlobal().compose(localPose)
            if n >= 20 && motionFilterPasses(t) { insert(a, r, mp.0, mp.1) }
            var k = 0
            while k < loopSearchesPerScan && !loopQueue.isEmpty {
                let job = loopQueue.removeFirst()
                tryLoopClosure(job.0, job.1)
                k += 1
            }
            if newLoops > 0 && nodesSinceOptimize >= optimizeEveryNNodes { optimize() }
        } else {
            let prior = SlamPose(pose.x + lastStep.0 * 0.6, pose.y + lastStep.1 * 0.6, pose.theta + dTheta)
            var best = prior
            if scans > 0 && n >= 20 {
                best = matcher.match(grid, prior, mp.0, mp.1)
                matchQuality = Self.hitFraction(grid, best, px, py)
                lowQualityScans = matchQuality < 0.3 ? lowQualityScans + 1 : 0
                if lowQualityScans >= 30 { lowQualityScans = 0; relocalizeRequested = true } // lost: search again
            }
            lastStep = (best.x - pose.x, best.y - pose.y)
            pose = best
            localPose = best
        }
        scans += 1
        if let last = trail.last, hypot(last.0 - pose.x, last.1 - pose.y) <= 0.05 {} else {
            trail.append((pose.x, pose.y))
            if trail.count > 5000 { trail.removeFirst() }
        }
        return pose
    }

    private func motionFilterPasses(_ t: Double) -> Bool {
        if let last = lastInsertPose, last.distance(to: localPose) <= Self.motionMaxDistance,
           abs(SlamPose.wrap(localPose.theta - last.theta)) <= Self.motionMaxAngle, t - lastInsertTime <= Self.motionMaxTime {
            return false
        }
        lastInsertPose = localPose
        lastInsertTime = t
        return true
    }

    private func localToGlobal() -> SlamPose {
        guard let last = submaps.last(where: { $0.session == session && !$0.frozen }) else { return .zero }
        return graph.poses[last.graphIndex].compose(SlamPose(last.ox, last.oy, 0).inverse)
    }

    private func insert(_ a: [Double], _ r: [Double], _ lx: [Double], _ ly: [Double]) {
        if let newest = active.last {
            if newest.inserts >= Self.numRangeData || hypot(localPose.x - newest.ox, localPose.y - newest.oy) > Self.newSubmapDistance {
                addSubmap(previous: newest)
            }
        } else {
            addSubmap(previous: nil)
        }
        var inSubs: [Int] = []
        for s in active {
            s.grid.integrate(localPose, angles: a, ranges: r, maxRange: Self.maxRange)
            s.inserts += 1
            inSubs.append(submaps.firstIndex { $0 === s } ?? 0)
        }
        grid.integrate(pose, angles: a, ranges: r, maxRange: Self.maxRange)
        let node = Node(local: localPose, submap: inSubs.last ?? 0, inSubmaps: inSubs, lx: lx, ly: ly, driven: driven)
        nodes.append(node)
        nodesSinceOptimize += 1
        for (si, s) in submaps.enumerated() where s.finished && isNear(node, s) && rng.next() < Self.samplingRatio {
            enqueue(nodes.count - 1, si)
        }
        if base != nil && rng.next() < Self.samplingRatio { enqueue(nodes.count - 1, -1) } // the loaded map
    }

    private func addSubmap(previous: Submap?) {
        let res = grid.resolution, half = Double(Self.submapCells) * res / 2
        let g = OccupancyGrid(width: Self.submapCells, height: Self.submapCells, resolution: res,
                              originX: localPose.x - half, originY: localPose.y - half)
        let s = Submap(grid: g, ox: localPose.x, oy: localPose.y, session: session, frozen: false)
        s.driven = driven
        let global = localToGlobal().compose(SlamPose(s.ox, s.oy, 0))
        let first = previous == nil || previous!.session != session
        let fix = first && base == nil && submaps.isEmpty
        s.graphIndex = graph.add(global, fixed: fix)
        if let p = previous, !first {
            graph.addConstraint(.init(i: p.graphIndex, j: s.graphIndex, relative: SlamPose(s.ox - p.ox, s.oy - p.oy, 0),
                                      wT: PoseGraph.seqT, wR: PoseGraph.seqR, robust: false))
        } else if !fix {
            graph.addConstraint(.init(i: s.graphIndex, j: -1, relative: global, wT: PoseGraph.priorT, wR: PoseGraph.priorR, robust: false))
        }
        submaps.append(s)
        active.append(s)
        if active.count > 2 { finish(active.removeFirst()) }
    }

    private func finish(_ s: Submap) {
        guard !s.finished else { return }
        s.finished = true
        guard let si = submaps.firstIndex(where: { $0 === s }) else { return }
        for (ni, nd) in nodes.enumerated() where !nd.inSubmaps.contains(si) && isNear(nd, s) && rng.next() < Self.samplingRatio {
            enqueue(ni, si)
        }
    }

    private func enqueue(_ node: Int, _ submap: Int) {
        loopQueue.append((node, submap))
        if loopQueue.count > 400 { loopQueue.removeFirst(loopQueue.count - 400) }
    }

    private func nodeGlobal(_ nd: Node) -> SlamPose {
        let a = submaps[nd.submap]
        return graph.poses[a.graphIndex].compose(SlamPose(nd.local.x - a.ox, nd.local.y - a.oy, nd.local.theta))
    }

    private func isNear(_ nd: Node, _ s: Submap) -> Bool {
        if s.frozen { return true }
        let g = nodeGlobal(nd), sg = graph.poses[s.graphIndex]
        return hypot(g.x - sg.x, g.y - sg.y) <= Self.maxConstraintDistance
    }

    /// Keeps branch-and-bound tables for the 6 most recently used submaps.
    private func matcherFor(_ si: Int) -> FastCorrelativeMatcher {
        let s = si >= 0 ? submaps[si] : base!
        if let m = s.matcher {
            if let k = cachedMatchers.firstIndex(of: si) { cachedMatchers.remove(at: k) }
            cachedMatchers.append(si)
            return m
        }
        let m = FastCorrelativeMatcher(s.grid)
        s.matcher = m
        cachedMatchers.append(si)
        if cachedMatchers.count > 6 {
            let old = cachedMatchers.removeFirst()
            if old >= 0 { submaps[old].matcher = nil } else { base?.matcher = nil }
        }
        return m
    }

    @discardableResult
    private func tryLoopClosure(_ ni: Int, _ si: Int) -> Bool {
        let nd = nodes[ni], s = si >= 0 ? submaps[si] : base!
        guard nd.lx.count >= 20 else { return false }
        let inSub = graph.poses[s.graphIndex].inverse.compose(nodeGlobal(nd))
        let initial = SlamPose(inSub.x + s.ox, inSub.y + s.oy, inSub.theta)
        let window = s.frozen ? Self.loopLinearWindow : Self.loopWindow(nd.driven - s.driven)
        guard let res = matcherFor(si).match(initial, nd.lx, nd.ly, linearWindow: window, angularWindow: Self.loopAngularWindow,
                                             minScore: Self.minScore) else { return false }
        // Cartographer constraint_builder ceres_scan_matcher: occupied 20, translation 10, rotation 1.
        let refined = matcher.refine(s.grid, res.pose, nd.lx, nd.ly, wOcc: 20, wT: 10, wR: 1)
        guard matcher.lastConstraint >= Self.minConstraint else { return false } // could slide
        let measured = SlamPose(refined.x - s.ox, refined.y - s.oy, refined.theta)
        let a = submaps[nd.submap]
        let relAN = SlamPose(nd.local.x - a.ox, nd.local.y - a.oy, nd.local.theta)
        graph.addConstraint(.init(i: s.graphIndex, j: a.graphIndex, relative: measured.compose(relAN.inverse),
                                  wT: PoseGraph.loopT, wR: PoseGraph.loopR, robust: true))
        loopClosures += 1
        newLoops += 1
        return true
    }

    func optimize() {
        graph.optimize()
        var round = 0
        while round < 2 && graph.removeOutliers(maxT: Self.outlierT, maxR: Self.outlierR) > 0 { graph.optimize(); round += 1 }
        loopClosures = graph.constraints.filter { $0.robust }.count
        newLoops = 0
        nodesSinceOptimize = 0
        pose = localToGlobal().compose(localPose)
        let old = grid
        let g = base?.grid.copy()
            ?? OccupancyGrid(width: old.width, height: old.height, resolution: old.resolution, originX: old.originX, originY: old.originY)
        for s in submaps where !s.frozen {
            g.addSubmap(s.grid, originX: s.ox, originY: s.oy, at: graph.poses[s.graphIndex], scale: 0.5)
        }
        grid = g
        trail = nodes.map { nd -> (Double, Double) in
            let p = self.nodeGlobal(nd)
            return (p.x, p.y)
        }
        if trail.count > 5000 { trail.removeFirst(trail.count - 5000) }
    }

    @discardableResult
    func globalLocalize(_ px: [Double], _ py: [Double]) -> Bool {
        let g = base?.grid ?? grid
        let key = (ObjectIdentifier(g), g.version)
        if globalMatcher == nil || globalMatcherKey?.0 != key.0 || globalMatcherKey?.1 != key.1 {
            globalMatcher = FastCorrelativeMatcher(g)
            globalMatcherKey = key
        }
        let cx = g.originX + Double(g.width) * g.resolution / 2, cy = g.originY + Double(g.height) * g.resolution / 2
        let window = Double(max(g.width, g.height)) * g.resolution / 2
        let res = globalMatcher!.match(SlamPose(cx, cy, 0), px, py, linearWindow: window, angularWindow: .pi,
                                       minScore: Self.globalLocalizationMinScore)
        relocalizeFound = res != nil
        if let res = res {
            let refined = matcher.refine(g, res.pose, px, py, wOcc: 20, wT: 10, wR: 1)
            pose = refined
            localPose = refined
            lastStep = (0, 0)
            lowQualityScans = 0
            if isMapping { startSession() }
        }
        relocalizeSeq += 1
        return relocalizeFound
    }

    static func adaptiveVoxel(_ px: [Double], _ py: [Double], maxLength: Double, minPoints: Int) -> ([Double], [Double]) {
        if px.count <= minPoints { return (px, py) }
        var f = voxel(px, py, maxLength)
        if f.0.count >= minPoints { return f }
        var high = maxLength, low = maxLength / 2
        f = voxel(px, py, low)
        while f.0.count < minPoints && low > 0.01 { high = low; low /= 2; f = voxel(px, py, low) }
        var best = f
        for _ in 0..<5 {
            let mid = (low + high) / 2, m = voxel(px, py, mid)
            if m.0.count >= minPoints { low = mid; best = m } else { high = mid }
        }
        return best
    }

    static func voxel(_ px: [Double], _ py: [Double], _ size: Double) -> ([Double], [Double]) {
        var seen = Set<Int64>(), ox: [Double] = [], oy: [Double] = []
        for i in 0..<px.count {
            let kx = Int64(floor(px[i] / size)), ky = Int64(floor(py[i] / size))
            if seen.insert((kx << 32) ^ (ky & 0xFFFF_FFFF)).inserted { ox.append(px[i]); oy.append(py[i]) }
        }
        return (ox, oy)
    }

    static func hitFraction(_ g: OccupancyGrid, _ p: SlamPose, _ px: [Double], _ py: [Double]) -> Double {
        guard !px.isEmpty else { return 0 }
        let c = cos(p.theta), s = sin(p.theta)
        var hits = 0
        for i in 0..<px.count {
            let cx = g.cellX(p.x + c * px[i] - s * py[i]), cy = g.cellY(p.y + s * px[i] + c * py[i])
            if g.isOccupied(cx, cy) || g.isOccupied(cx + 1, cy) || g.isOccupied(cx - 1, cy)
                || g.isOccupied(cx, cy + 1) || g.isOccupied(cx, cy - 1) { hits += 1 }
        }
        return Double(hits) / Double(px.count)
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
