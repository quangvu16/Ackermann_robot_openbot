//
//  SlamAlgorithms.swift
//  OpenBot
//
//  The SLAM engines and localizers the user can choose on the Indoor SLAM
//  screen; the same algorithms as the Android classes in
//  org.openbot.app.robot.slam (Slam, BaseSlam, KartoSlam, HectorSlam,
//  GMappingSlam, CoreSlam, Amcl, ScanOdometry, SlamSystem) - keep them in step.
//  Each is rewritten for this project from its paper and the project's
//  documented defaults; no code from those projects is used.
//

import Foundation

enum SlamType: Int, CaseIterable {
    case cartographer, slamToolbox, hector, gmapping, coreslam

    var label: String {
        switch self {
        case .cartographer: return "Cartographer"
        case .slamToolbox: return "slam_toolbox (Karto)"
        case .hector: return "Hector SLAM"
        case .gmapping: return "GMapping"
        case .coreslam: return "CoreSLAM / BreezySLAM"
        }
    }
}

enum SlamLocalizer: Int, CaseIterable {
    case scanMatching, amcl
    var label: String { self == .amcl ? "AMCL" : "Scan matching" }
}

/// One SLAM algorithm; every engine builds an OccupancyGrid in the same frame.
protocol SlamAlgorithm: AnyObject {
    var type: SlamType { get }
    var grid: OccupancyGrid { get }
    var pose: SlamPose { get }
    var scans: Int { get }
    var matchQuality: Double { get }
    var trail: [(Double, Double)] { get }
    var mapping: Bool { get set }
    var submapCount: Int { get }
    var loopClosures: Int { get }
    var relocalizeSeq: Int { get }
    var relocalizeFound: Bool { get }
    func reset()
    func setGrid(_ g: OccupancyGrid)
    func resetPose(_ p: SlamPose)
    func requestGlobalLocalization()
    @discardableResult
    func update(angles: [Double], ranges: [Double], imuYaw: Double, time: Double?) -> SlamPose
}

extension SlamEngine: SlamAlgorithm {
    var type: SlamType { .cartographer }
}

func makeSlam(_ t: SlamType, _ grid: OccupancyGrid) -> SlamAlgorithm {
    switch t {
    case .cartographer: return SlamEngine(grid: grid)
    case .slamToolbox: return KartoSlam(grid)
    case .hector: return HectorSlam(grid)
    case .gmapping: return GMappingSlam(grid)
    case .coreslam: return CoreSlam(grid)
    }
}

/// One revolution, filtered: valid beams, their points, and a thinned copy.
struct SlamScan {
    var a: [Double] = [], r: [Double] = [], px: [Double] = [], py: [Double] = []
    var mx: [Double] = [], my: [Double] = []
    var n: Int { a.count }

    init(_ angles: [Double], _ ranges: [Double]) {
        for i in 0..<ranges.count where ranges[i] >= SlamEngine.minRange && ranges[i] < SlamEngine.maxRange {
            a.append(angles[i]); r.append(ranges[i])
            px.append(ranges[i] * cos(angles[i])); py.append(ranges[i] * sin(angles[i]))
        }
        let m = SlamEngine.adaptiveVoxel(px, py, maxLength: 0.5, minPoints: 200)
        mx = m.0; my = m.1
    }
}

/// Branch and bound over the whole map and every heading.
final class GlobalLocalizer {
    private var fcm: FastCorrelativeMatcher?
    private var key: (ObjectIdentifier, Int)?

    func find(_ g: OccupancyGrid, _ px: [Double], _ py: [Double], _ refiner: ScanMatcher) -> SlamPose? {
        if fcm == nil || key?.0 != ObjectIdentifier(g) || key?.1 != g.version {
            fcm = FastCorrelativeMatcher(g)
            key = (ObjectIdentifier(g), g.version)
        }
        let cx = g.originX + Double(g.width) * g.resolution / 2, cy = g.originY + Double(g.height) * g.resolution / 2
        let window = Double(max(g.width, g.height)) * g.resolution / 2
        guard let res = fcm!.match(SlamPose(cx, cy, 0), px, py, linearWindow: window, angularWindow: .pi,
                                   minScore: SlamEngine.globalLocalizationMinScore) else { return nil }
        return refiner.refine(g, res.pose, px, py, wOcc: 20, wT: 10, wR: 1)
    }
}

/// Shared parts of the non-Cartographer engines; subclasses implement step().
class BaseSlam: SlamAlgorithm {
    var type: SlamType { .coreslam }
    var grid: OccupancyGrid
    var pose = SlamPose.zero
    var scans = 0
    var matchQuality = 0.0
    var trail: [(Double, Double)] = []
    var mapping = true
    var submapCount: Int { 0 }
    var loopClosures: Int { 0 }
    private(set) var relocalizeSeq = 0
    private(set) var relocalizeFound = false
    let matcher = ScanMatcher()
    private var lastImuYaw = Double.nan
    private var lastStep = (0.0, 0.0)
    private var relocalizeRequested = false
    private let globalLocalizer = GlobalLocalizer()

    init(_ grid: OccupancyGrid) { self.grid = grid }

    func reset() {
        let g = grid
        grid = OccupancyGrid(width: g.width, height: g.height, resolution: g.resolution, originX: g.originX, originY: g.originY)
        scans = 0
        onMapReplaced()
        resetPose(.zero)
        trail = []
    }

    func setGrid(_ loaded: OccupancyGrid) {
        grid = loaded.copy()
        scans = 1
        onMapReplaced()
        resetPose(.zero)
        trail = []
    }

    func resetPose(_ p: SlamPose) {
        pose = p
        lastStep = (0, 0)
        lastImuYaw = .nan
        onPoseReset(p)
    }

    func requestGlobalLocalization() { relocalizeRequested = true }

    func onMapReplaced() {}
    func onPoseReset(_ p: SlamPose) {}
    func step(_ s: SlamScan, _ prior: SlamPose, _ time: Double) -> SlamPose { prior }

    @discardableResult
    func update(angles: [Double], ranges: [Double], imuYaw: Double, time: Double?) -> SlamPose {
        let s = SlamScan(angles, ranges)
        let t = time ?? Double(scans) * 0.1
        var dTheta = 0.0
        if !imuYaw.isNaN {
            if !lastImuYaw.isNaN { dTheta = SlamPose.wrap(imuYaw - lastImuYaw) }
            lastImuYaw = imuYaw
        }
        if relocalizeRequested && s.n >= 20 {
            relocalizeRequested = false
            let found = globalLocalizer.find(grid, s.mx, s.my, matcher)
            relocalizeFound = found != nil
            if let f = found {
                let keep = lastImuYaw
                resetPose(f)
                lastImuYaw = keep
                dTheta = 0
            }
            relocalizeSeq += 1
        }
        let prior = SlamPose(pose.x + lastStep.0 * 0.6, pose.y + lastStep.1 * 0.6, pose.theta + dTheta)
        let best = s.n >= 20 ? step(s, prior, t) : prior
        lastStep = (best.x - pose.x, best.y - pose.y)
        pose = best
        scans += 1
        if let last = trail.last, hypot(last.0 - best.x, last.1 - best.y) <= 0.05 {} else {
            trail.append((best.x, best.y))
            if trail.count > 5000 { trail.removeFirst() }
        }
        return best
    }
}

// MARK: - CoreSLAM (BreezySLAM's RMHC: random-mutation hill climbing, one map, no loop closure)

final class CoreSlam: BaseSlam {
    override var type: SlamType { .coreslam }
    var iterations = 250, sigmaXY = 0.08, sigmaTheta = 4.0 * .pi / 180
    private var rng = SeededRandom(seed: 42)
    private var lastMapPose: SlamPose?

    override func onPoseReset(_ p: SlamPose) { lastMapPose = nil }

    override func step(_ s: SlamScan, _ prior: SlamPose, _ time: Double) -> SlamPose {
        var best = prior
        if scans > 0 {
            best = rmhc(prior, s.px, s.py)
            matchQuality = SlamEngine.hitFraction(grid, best, s.px, s.py)
        }
        if mapping {
            let moved = lastMapPose.map { $0.distance(to: best) > 0.04 || abs(SlamPose.wrap(best.theta - $0.theta)) > 2 * .pi / 180 } ?? true
            if moved { grid.integrate(best, angles: s.a, ranges: s.r, maxRange: SlamEngine.maxRange); lastMapPose = best }
        }
        return best
    }

    private func rmhc(_ prior: SlamPose, _ px: [Double], _ py: [Double]) -> SlamPose {
        var bestScore = score(prior, px, py), best = prior
        var sxy = sigmaXY, sth = sigmaTheta, since = 0
        for _ in 0..<iterations {
            let cand = SlamPose(best.x + rng.gaussian() * sxy, best.y + rng.gaussian() * sxy, best.theta + rng.gaussian() * sth)
            let sc = score(cand, px, py)
            if sc > bestScore { bestScore = sc; best = cand; since = 0 } else {
                since += 1
                if since > 25 { sxy *= 0.5; sth *= 0.5; since = 0; if sxy < grid.resolution * 0.1 { break } }
            }
        }
        return best
    }

    private func score(_ p: SlamPose, _ px: [Double], _ py: [Double]) -> Double {
        let c = cos(p.theta), sn = sin(p.theta)
        var sum = 0.0
        for i in 0..<px.count { sum += grid.hitScore(grid.cellX(p.x + c * px[i] - sn * py[i]), grid.cellY(p.y + sn * px[i] + c * py[i])) }
        return sum
    }
}

// MARK: - Hector SLAM (Gauss-Newton scan-to-map matching on 3 map resolutions, no odometry)

final class HectorSlam: BaseSlam {
    override var type: SlamType { .hector }
    static let levelsCount = 3, updateDistance = 0.4, updateAngle = 0.9 // hector_mapping defaults
    static let lOcc = Float(log(0.9 / 0.1)), lFree = Float(log(0.4 / 0.6)) // update factors
    private var levels: [OccupancyGrid] = []
    private var prob: [[Float]] = []
    private var probVersion: [Int] = []
    private var lastMapPose: SlamPose?

    override init(_ grid: OccupancyGrid) { super.init(grid); buildLevels() }
    override func onMapReplaced() { buildLevels() }
    override func onPoseReset(_ p: SlamPose) { lastMapPose = nil }

    private func buildLevels() {
        levels = [grid]
        for _ in 1..<Self.levelsCount {
            let f = levels.last!
            let c = OccupancyGrid(width: f.width / 2, height: f.height / 2, resolution: f.resolution * 2, originX: f.originX, originY: f.originY)
            for y in 0..<c.height {
                for x in 0..<c.width {
                    c.set(x, y, (f.get(2 * x, 2 * y) + f.get(2 * x + 1, 2 * y) + f.get(2 * x, 2 * y + 1) + f.get(2 * x + 1, 2 * y + 1)) / 4)
                }
            }
            levels.append(c)
        }
        prob = Array(repeating: [], count: Self.levelsCount)
        probVersion = Array(repeating: -1, count: Self.levelsCount)
    }

    private func probabilities(_ l: Int) -> [Float] {
        let g = levels[l]
        if probVersion[l] != g.version || prob[l].count != g.logOdds.count {
            prob[l] = g.logOdds.map { Float(1 / (1 + exp(-Double($0)))) }
            probVersion[l] = g.version
        }
        return prob[l]
    }

    override func step(_ s: SlamScan, _ prior: SlamPose, _ time: Double) -> SlamPose {
        var best = prior
        if scans > 0 {
            for l in stride(from: Self.levelsCount - 1, through: 0, by: -1) { best = gaussNewton(l, best, s.mx, s.my, l == 0 ? 5 : 3) }
            matchQuality = SlamEngine.hitFraction(grid, best, s.px, s.py)
        }
        if mapping, lastMapPose == nil || lastMapPose!.distance(to: best) > Self.updateDistance
            || abs(SlamPose.wrap(best.theta - lastMapPose!.theta)) > Self.updateAngle {
            for g in levels { g.integrate(best, angles: s.a, ranges: s.r, maxRange: SlamEngine.maxRange, lOcc: Self.lOcc, lFree: Self.lFree) }
            lastMapPose = best
        }
        return best
    }

    private func gaussNewton(_ l: Int, _ start: SlamPose, _ px: [Double], _ py: [Double], _ iterations: Int) -> SlamPose {
        let g = levels[l], p = probabilities(l)
        var x = start.x, y = start.y, th = start.theta
        var cost = Self.cost(g, p, px, py, x, y, th)
        for _ in 0..<iterations {
            var h = [[Double]](repeating: [0, 0, 0], count: 3), b = [0.0, 0.0, 0.0]
            let c = cos(th), sn = sin(th)
            for i in 0..<px.count {
                let m = Self.bilinear(g, p, x + c * px[i] - sn * py[i], y + sn * px[i] + c * py[i])
                let r = 1 - m.0
                let j = [m.1, m.2, m.1 * (-sn * px[i] - c * py[i]) + m.2 * (c * px[i] - sn * py[i])]
                for a in 0..<3 { b[a] += j[a] * r; for k in 0..<3 { h[a][k] += j[a] * j[k] } }
            }
            for a in 0..<3 { h[a][a] += 1e-6 }
            guard let d = ScanMatcher.solve3(h, b) else { break }
            let nc = Self.cost(g, p, px, py, x + d[0], y + d[1], th + d[2])
            if nc >= cost { break }
            x += d[0]; y += d[1]; th += d[2]; cost = nc
        }
        return SlamPose(x, y, th)
    }

    private static func cost(_ g: OccupancyGrid, _ p: [Float], _ px: [Double], _ py: [Double], _ x: Double, _ y: Double, _ th: Double) -> Double {
        let c = cos(th), sn = sin(th)
        var sum = 0.0
        for i in 0..<px.count {
            let m = bilinear(g, p, x + c * px[i] - sn * py[i], y + sn * px[i] + c * py[i])
            sum += (1 - m.0) * (1 - m.0)
        }
        return sum
    }

    private static func bilinear(_ g: OccupancyGrid, _ p: [Float], _ wx: Double, _ wy: Double) -> (Double, Double, Double) {
        let u = (wx - g.originX) / g.resolution - 0.5, v = (wy - g.originY) / g.resolution - 0.5
        let x0 = Int(floor(u)), y0 = Int(floor(v)), tx = u - Double(x0), ty = v - Double(y0)
        func at(_ x: Int, _ y: Int) -> Double { (x >= 0 && y >= 0 && x < g.width && y < g.height) ? Double(p[y * g.width + x]) : 0.5 }
        let p00 = at(x0, y0), p10 = at(x0 + 1, y0), p01 = at(x0, y0 + 1), p11 = at(x0 + 1, y0 + 1)
        return (ty * (tx * p11 + (1 - tx) * p01) + (1 - ty) * (tx * p10 + (1 - tx) * p00),
                (ty * (p11 - p01) + (1 - ty) * (p10 - p00)) / g.resolution,
                (tx * (p11 - p10) + (1 - tx) * (p01 - p00)) / g.resolution)
    }
}

// MARK: - GMapping (Rao-Blackwellized particle filter: each particle has its own map)

final class GMappingSlam: BaseSlam {
    override var type: SlamType { .gmapping }
    // gmapping defaults: srr 0.1, srt 0.2, str 0.1, stt 0.2, ogain 3, resampleThreshold 0.5.
    // Phone: 12 particles (gmapping 30), updates every 0.5 m / 0.25 rad (gmapping 1.0 / 0.5).
    static let particleCount = 12, linearUpdate = 0.5, angularUpdate = 0.25
    static let srr = 0.1, srt = 0.2, str = 0.1, stt = 0.2, ogain = 3.0, resampleThreshold = 0.5

    final class Particle {
        var pose: SlamPose
        var map: OccupancyGrid
        var logWeight = 0.0
        init(_ pose: SlamPose, _ map: OccupancyGrid) { self.pose = pose; self.map = map }
    }

    private var particles: [Particle] = []
    private var best = 0
    private var odomAtUpdate = SlamPose.zero
    private var started = false
    private var rng = SeededRandom(seed: 7)
    private var resamples = 0
    override var submapCount: Int { particles.count }
    override var loopClosures: Int { resamples }

    override func onMapReplaced() { started = false; particles = [] }

    override func onPoseReset(_ p: SlamPose) {
        guard !particles.isEmpty else { return }
        for q in particles { q.pose = p; q.logWeight = 0 }
        odomAtUpdate = p
    }

    override func step(_ s: SlamScan, _ prior: SlamPose, _ time: Double) -> SlamPose {
        if !started {
            particles = (0..<Self.particleCount).map { Particle(prior, $0 == 0 ? grid : grid.copy()) }
            best = 0; odomAtUpdate = prior; started = true
        }
        let bestMap = particles[best].map
        let odom = scans > 0 ? matcher.match(bestMap, prior, s.mx, s.my) : prior
        matchQuality = SlamEngine.hitFraction(bestMap, odom, s.px, s.py)
        if !mapping { return odom }
        let delta = odomAtUpdate.inverse.compose(odom)
        let first = scans <= 1 && odomAtUpdate.distance(to: odom) < 1e-9
        if !first && hypot(delta.x, delta.y) < Self.linearUpdate && abs(delta.theta) < Self.angularUpdate { return odom }
        let d = hypot(delta.x, delta.y), a = abs(delta.theta)
        let sT = Self.srr * d + Self.srt * a, sR = Self.str * d + Self.stt * a
        if !first {
            for q in particles {
                let noisy = SlamPose(delta.x + rng.gaussian() * sT, delta.y + rng.gaussian() * sT, delta.theta + rng.gaussian() * sR)
                q.pose = matcher.refine(q.map, q.pose.compose(noisy), s.mx, s.my) // improved proposal
                q.logWeight += Self.logLikelihood(q.map, q.pose, s.mx, s.my) / Self.ogain
            }
            resampleIfNeeded()
        }
        for q in particles { q.map.integrate(q.pose, angles: s.a, ranges: s.r, maxRange: SlamEngine.maxRange) }
        best = 0
        for i in 1..<particles.count where particles[i].logWeight > particles[best].logWeight { best = i }
        grid = particles[best].map
        odomAtUpdate = particles[best].pose
        return odomAtUpdate
    }

    private static func logLikelihood(_ g: OccupancyGrid, _ p: SlamPose, _ px: [Double], _ py: [Double]) -> Double {
        let prob = g.probabilities(), c = cos(p.theta), sn = sin(p.theta)
        var sum = 0.0
        for i in 0..<px.count {
            let x = g.cellX(p.x + c * px[i] - sn * py[i]), y = g.cellY(p.y + sn * px[i] + c * py[i])
            sum += log(Double(g.inBounds(x, y) ? prob[y * g.width + x] : OccupancyGrid.pMin))
        }
        return sum
    }

    private func resampleIfNeeded() {
        let n = particles.count
        let maxW = particles.map { $0.logWeight }.max() ?? 0
        var w = particles.map { exp($0.logWeight - maxW) }
        let sum = w.reduce(0, +)
        var sq = 0.0
        for i in 0..<n { w[i] /= sum; sq += w[i] * w[i]; particles[i].logWeight = log(max(w[i], 1e-300)) }
        guard 1 / sq < Self.resampleThreshold * Double(n) else { return }
        var next: [Particle] = []
        var used = [Bool](repeating: false, count: n)
        let r = rng.next() / Double(n)
        var c = w[0], i = 0
        for m in 0..<n {
            let u = r + Double(m) / Double(n)
            while u > c && i < n - 1 { i += 1; c += w[i] }
            let src = particles[i]
            next.append(Particle(src.pose, used[i] ? src.map.copy() : src.map))
            used[i] = true
        }
        particles = next
        resamples += 1
    }
}

// MARK: - slam_toolbox / Karto graph SLAM (scan nodes, running buffer, chain loop closure)

final class KartoSlam: BaseSlam {
    override var type: SlamType { .slamToolbox }
    // slam_toolbox mapper_params_online_async.yaml
    static let minTravelDistance = 0.5, minTravelHeading = 0.5, scanBufferSize = 10
    static let loopSearchMaxDistance = 3.0, loopMinChain = 10, loopSearchWindow = 4.0
    static let loopAngleWindow = 0.349, loopMinResponseFine = 0.45, bufferCells = 320

    private struct Node { let a: [Double], r: [Double], mx: [Double], my: [Double] }
    private let graph = PoseGraph()
    private var nodes: [Node] = []
    private var base: OccupancyGrid?
    private var buffer: OccupancyGrid?
    private var lastNodePose: SlamPose?
    private var loops = 0
    private var chainBreak = true
    override var submapCount: Int { nodes.count }
    override var loopClosures: Int { loops }

    override func onMapReplaced() {
        graph.clear(); nodes = []; buffer = nil; loops = 0
        base = scans > 0 ? grid.copy() : nil
    }

    override func onPoseReset(_ p: SlamPose) { lastNodePose = nil; buffer = nil; chainBreak = true }

    override func step(_ s: SlamScan, _ prior: SlamPose, _ time: Double) -> SlamPose {
        if !mapping {
            let best = scans > 0 ? matcher.match(grid, prior, s.mx, s.my) : prior
            matchQuality = SlamEngine.hitFraction(grid, best, s.px, s.py)
            return best
        }
        var best = prior
        if let against = buffer ?? (base != nil ? grid : nil) {
            best = matcher.match(against, prior, s.mx, s.my)
            matchQuality = SlamEngine.hitFraction(against, best, s.px, s.py)
        }
        if lastNodePose == nil || lastNodePose!.distance(to: best) >= Self.minTravelDistance
            || abs(SlamPose.wrap(best.theta - lastNodePose!.theta)) >= Self.minTravelHeading {
            best = addNode(s, best)
        }
        return best
    }

    private func addNode(_ s: SlamScan, _ pose: SlamPose) -> SlamPose {
        var p = pose
        let firstFixed = nodes.isEmpty && base == nil
        let gi = graph.add(p, fixed: firstFixed)
        if !chainBreak && !nodes.isEmpty {
            graph.addConstraint(.init(i: gi - 1, j: gi, relative: graph.poses[gi - 1].inverse.compose(p),
                                      wT: PoseGraph.seqT, wR: PoseGraph.seqR, robust: false))
        } else if !firstFixed {
            graph.addConstraint(.init(i: gi, j: -1, relative: p, wT: PoseGraph.priorT, wR: PoseGraph.priorR, robust: false))
        }
        chainBreak = false
        nodes.append(Node(a: s.a, r: s.r, mx: s.mx, my: s.my))
        grid.integrate(p, angles: s.a, ranges: s.r, maxRange: SlamEngine.maxRange)
        lastNodePose = p
        if tryLoopClosure(gi) {
            optimizeAndRedraw()
            p = graph.poses[gi]
            lastNodePose = p
        }
        rebuildBuffer()
        return p
    }

    private func rebuildBuffer() {
        let last = nodes.count - 1, c = graph.poses[last]
        let half = Double(Self.bufferCells) * grid.resolution / 2
        let b = OccupancyGrid(width: Self.bufferCells, height: Self.bufferCells, resolution: grid.resolution, originX: c.x - half, originY: c.y - half)
        for i in max(0, last - Self.scanBufferSize + 1)...last {
            b.integrate(graph.poses[i], angles: nodes[i].a, ranges: nodes[i].r, maxRange: SlamEngine.maxRange)
        }
        buffer = b
    }

    private func tryLoopClosure(_ ni: Int) -> Bool {
        let p = graph.poses[ni], limit = ni - Self.scanBufferSize
        guard limit > 0 else { return false }
        var nearest = -1, nearestD = Double.greatestFiniteMagnitude
        for i in 0..<limit {
            let d = graph.poses[i].distance(to: p)
            if d <= Self.loopSearchMaxDistance && d < nearestD { nearestD = d; nearest = i }
        }
        guard nearest >= 0 else { return false }
        var lo = nearest, hi = nearest
        while lo > 0 && graph.poses[lo - 1].distance(to: p) <= Self.loopSearchMaxDistance * 2 { lo -= 1 }
        while hi < limit - 1 && graph.poses[hi + 1].distance(to: p) <= Self.loopSearchMaxDistance * 2 { hi += 1 }
        guard hi - lo + 1 >= Self.loopMinChain else { return false }
        let xs = (lo...hi).map { graph.poses[$0].x }, ys = (lo...hi).map { graph.poses[$0].y }
        let pad = SlamEngine.maxRange, res = grid.resolution
        let minX = xs.min()!, minY = ys.min()!
        let w = Int(ceil((xs.max()! - minX + 2 * pad) / res)), h = Int(ceil((ys.max()! - minY + 2 * pad) / res))
        let chain = OccupancyGrid(width: w, height: h, resolution: res, originX: minX - pad, originY: minY - pad)
        for i in lo...hi { chain.integrate(graph.poses[i], angles: nodes[i].a, ranges: nodes[i].r, maxRange: SlamEngine.maxRange) }
        let nd = nodes[ni]
        guard let r = FastCorrelativeMatcher(chain).match(p, nd.mx, nd.my, linearWindow: Self.loopSearchWindow,
                                                          angularWindow: Self.loopAngleWindow, minScore: 0.1 + 0.8 * 0.35) else { return false }
        let refined = matcher.refine(chain, r.pose, nd.mx, nd.my, wOcc: 20, wT: 10, wR: 1)
        let response = (ScanMatcher.meanProbability(chain, refined, nd.mx, nd.my) - 0.1) / 0.8
        guard response >= Self.loopMinResponseFine, matcher.lastConstraint >= SlamEngine.minConstraint else { return false }
        graph.addConstraint(.init(i: nearest, j: ni, relative: graph.poses[nearest].inverse.compose(refined),
                                  wT: PoseGraph.loopT, wR: PoseGraph.loopR, robust: true))
        return true
    }

    private func optimizeAndRedraw() {
        graph.optimize()
        var round = 0
        while round < 2 && graph.removeOutliers(maxT: SlamEngine.outlierT, maxR: SlamEngine.outlierR) > 0 { graph.optimize(); round += 1 }
        loops = graph.constraints.filter { $0.robust }.count
        let old = grid
        let g = base?.copy() ?? OccupancyGrid(width: old.width, height: old.height, resolution: old.resolution, originX: old.originX, originY: old.originY)
        var t: [(Double, Double)] = []
        for i in 0..<nodes.count {
            let q = graph.poses[i]
            g.integrate(q, angles: nodes[i].a, ranges: nodes[i].r, maxRange: SlamEngine.maxRange)
            t.append((q.x, q.y))
        }
        grid = g
        trail = t
    }
}

// MARK: - AMCL (Nav2 defaults) with LiDAR odometry

final class ScanOdometry {
    private let matcher = ScanMatcher()
    private var local: OccupancyGrid?
    private(set) var pose = SlamPose.zero
    private var lastInsert = SlamPose.zero
    private var center = (0.0, 0.0)

    func reset(_ p: SlamPose) { pose = p; local = nil }

    func update(_ s: SlamScan, _ prior: SlamPose) -> SlamPose {
        if let l = local, s.n >= 20 { pose = matcher.match(l, prior, s.mx, s.my) } else { pose = prior }
        if local == nil || hypot(pose.x - center.0, pose.y - center.1) > 3 {
            center = (pose.x, pose.y)
            let l = OccupancyGrid(width: 280, height: 280, resolution: 0.05, originX: pose.x - 7, originY: pose.y - 7)
            l.integrate(pose, angles: s.a, ranges: s.r, maxRange: SlamEngine.maxRange)
            local = l
            lastInsert = pose
            return pose
        }
        if lastInsert.distance(to: pose) > 0.1 || abs(SlamPose.wrap(pose.theta - lastInsert.theta)) > 5 * .pi / 180 {
            local!.integrate(pose, angles: s.a, ranges: s.r, maxRange: SlamEngine.maxRange)
            lastInsert = pose
        }
        return pose
    }
}

final class Amcl {
    static let alpha1 = 0.2, alpha2 = 0.2, alpha3 = 0.2, alpha4 = 0.2
    static let updateMinD = 0.25, updateMinA = 0.2, maxBeams = 60
    static let sigmaHit = 0.2, zHit = 0.5, zRand = 0.5, likelihoodMaxDist = 2.0
    static let minParticles = 500, maxParticles = 2000

    private var px: [Double] = [], py: [Double] = [], pt: [Double] = [], w: [Double] = []
    private var rng = SeededRandom(seed: 11)
    private var field: [Float] = []
    private var fieldKey: (ObjectIdentifier, Int)?
    private var lastOdom: SlamPose?
    private(set) var estimate = SlamPose.zero
    private(set) var globalSearch = false
    private(set) var updates = 0
    var size: Int { px.count }

    func spread() -> Double {
        var s = 0.0
        for i in 0..<px.count { s += w[i] * (pow(px[i] - estimate.x, 2) + pow(py[i] - estimate.y, 2)) }
        return s.squareRoot()
    }

    func initAt(_ p: SlamPose) {
        let n = Self.minParticles
        px = (0..<n).map { _ in p.x + rng.gaussian() * 0.25 }
        py = (0..<n).map { _ in p.y + rng.gaussian() * 0.25 }
        pt = (0..<n).map { _ in p.theta + rng.gaussian() * 0.2 }
        w = [Double](repeating: 1 / Double(n), count: n)
        estimate = p; lastOdom = nil; globalSearch = false; updates = 0
    }

    func initGlobal(_ g: OccupancyGrid) {
        let n = Self.maxParticles
        px = []; py = []; pt = []
        var tries = 0
        while px.count < n {
            let cx = Int(rng.next() * Double(g.width)), cy = Int(rng.next() * Double(g.height))
            tries += 1
            if g.get(cx, cy) < OccupancyGrid.freeThreshold || tries > 2_000_000 {
                px.append(g.worldX(cx)); py.append(g.worldY(cy)); pt.append((rng.next() * 2 - 1) * .pi)
            }
        }
        w = [Double](repeating: 1 / Double(n), count: n)
        lastOdom = nil; globalSearch = true; updates = 0
    }

    @discardableResult
    func update(_ odom: SlamPose, _ angles: [Double], _ ranges: [Double], _ g: OccupancyGrid) -> Bool {
        if px.isEmpty { initAt(odom) }
        guard let last = lastOdom else {
            lastOdom = odom; weigh(angles, ranges, g); resample(); return true
        }
        let d = last.inverse.compose(odom)
        if hypot(d.x, d.y) < Self.updateMinD && abs(d.theta) < Self.updateMinA { return false }
        motion(last, odom)
        lastOdom = odom
        weigh(angles, ranges, g)
        resample()
        updates += 1
        return true
    }

    private func motion(_ o0: SlamPose, _ o1: SlamPose) {
        let dx = o1.x - o0.x, dy = o1.y - o0.y, trans = hypot(dx, dy)
        let rot1 = trans < 0.01 ? 0 : SlamPose.wrap(atan2(dy, dx) - o0.theta)
        let rot2 = SlamPose.wrap(o1.theta - o0.theta - rot1)
        let r1 = min(abs(rot1), abs(SlamPose.wrap(rot1 - .pi))), r2 = min(abs(rot2), abs(SlamPose.wrap(rot2 - .pi)))
        let sR1 = (Self.alpha1 * r1 * r1 + Self.alpha2 * trans * trans).squareRoot()
        let sT = (Self.alpha3 * trans * trans + Self.alpha4 * (r1 * r1 + r2 * r2)).squareRoot()
        let sR2 = (Self.alpha1 * r2 * r2 + Self.alpha2 * trans * trans).squareRoot()
        for i in 0..<px.count {
            let a1 = rot1 - rng.gaussian() * sR1, t = trans - rng.gaussian() * sT, a2 = rot2 - rng.gaussian() * sR2
            px[i] += t * cos(pt[i] + a1); py[i] += t * sin(pt[i] + a1)
            pt[i] = SlamPose.wrap(pt[i] + a1 + a2)
        }
    }

    /// Distance to the nearest wall, capped (two-pass chamfer transform).
    private func buildField(_ g: OccupancyGrid) {
        let w = g.width, h = g.height, big = Float(Self.likelihoodMaxDist)
        let res = Float(g.resolution), diag = Float(g.resolution * 2.0.squareRoot())
        var f = [Float](repeating: big, count: w * h)
        for y in 0..<h { for x in 0..<w where g.isOccupied(x, y) { f[y * w + x] = 0 } }
        for y in 0..<h {
            for x in 0..<w {
                let i = y * w + x
                var v = f[i]
                if x > 0 { v = min(v, f[i - 1] + res) }
                if y > 0 {
                    v = min(v, f[i - w] + res)
                    if x > 0 { v = min(v, f[i - w - 1] + diag) }
                    if x < w - 1 { v = min(v, f[i - w + 1] + diag) }
                }
                f[i] = v
            }
        }
        for y in stride(from: h - 1, through: 0, by: -1) {
            for x in stride(from: w - 1, through: 0, by: -1) {
                let i = y * w + x
                var v = f[i]
                if x < w - 1 { v = min(v, f[i + 1] + res) }
                if y < h - 1 {
                    v = min(v, f[i + w] + res)
                    if x < w - 1 { v = min(v, f[i + w + 1] + diag) }
                    if x > 0 { v = min(v, f[i + w - 1] + diag) }
                }
                f[i] = min(v, big)
            }
        }
        field = f
        fieldKey = (ObjectIdentifier(g), g.version)
    }

    private func weigh(_ angles: [Double], _ ranges: [Double], _ g: OccupancyGrid) {
        if fieldKey?.0 != ObjectIdentifier(g) || fieldKey?.1 != g.version || field.count != g.width * g.height { buildField(g) }
        let n = angles.count, step = max(1, n / Self.maxBeams)
        let denom = 2 * Self.sigmaHit * Self.sigmaHit, zRandMult = 1 / SlamEngine.maxRange
        var total = 0.0
        for i in 0..<px.count {
            let c = cos(pt[i]), s = sin(pt[i])
            var p = 1.0
            for b in stride(from: 0, to: n, by: step) {
                let r = ranges[b]
                if r < SlamEngine.minRange || r >= SlamEngine.maxRange { continue }
                let lx = r * cos(angles[b]), ly = r * sin(angles[b])
                let cx = g.cellX(px[i] + c * lx - s * ly), cy = g.cellY(py[i] + s * lx + c * ly)
                let z = g.inBounds(cx, cy) ? Double(field[cy * g.width + cx]) : Self.likelihoodMaxDist
                let pz = Self.zHit * exp(-(z * z) / denom) + Self.zRand * zRandMult
                p += pz * pz * pz
            }
            w[i] *= p
            total += w[i]
        }
        if total <= 0 { w = [Double](repeating: 1 / Double(w.count), count: w.count) } else { for i in 0..<w.count { w[i] /= total } }
        var mx = 0.0, my = 0.0, sc = 0.0, ss = 0.0
        for i in 0..<px.count { mx += w[i] * px[i]; my += w[i] * py[i]; sc += w[i] * cos(pt[i]); ss += w[i] * sin(pt[i]) }
        estimate = SlamPose(mx, my, atan2(ss, sc))
    }

    private func resample() {
        let sp = spread()
        let n = sp > 1.0 ? Self.maxParticles : Self.minParticles
        if globalSearch && sp < 0.5 { globalSearch = false }
        var nx = [Double](), ny = [Double](), nt = [Double]()
        nx.reserveCapacity(n); ny.reserveCapacity(n); nt.reserveCapacity(n)
        let r = rng.next() / Double(n)
        var c = w[0], i = 0
        for m in 0..<n {
            let u = r + Double(m) / Double(n)
            while u > c && i < w.count - 1 { i += 1; c += w[i] }
            nx.append(px[i]); ny.append(py[i]); nt.append(pt[i])
        }
        px = nx; py = ny; pt = nt
        w = [Double](repeating: 1 / Double(n), count: n)
    }
}

// MARK: - What the screen talks to: the chosen engine plus the chosen localizer

final class SlamSystem {
    private(set) var engine: SlamAlgorithm
    private(set) var localizer = SlamLocalizer.scanMatching
    private var isMapping = true
    private let amcl = Amcl()
    private let odometry = ScanOdometry()
    private var amclPose = SlamPose.zero
    private var lastImuYaw = Double.nan
    private var amclSearchPending = false
    private var amclSeq = 0, amclFound = false, lastFromAmcl = false
    private var amclQuality = 0.0, amclScans = 0
    private var amclTrail: [(Double, Double)] = []

    init(_ type: SlamType, _ grid: OccupancyGrid) { engine = makeSlam(type, grid) }

    private var amclActive: Bool { !isMapping && localizer == .amcl }

    var type: SlamType { engine.type }
    var grid: OccupancyGrid { engine.grid }
    var pose: SlamPose { amclActive ? amclPose : engine.pose }
    var scans: Int { engine.scans + amclScans }
    var matchQuality: Double { amclActive ? amclQuality : engine.matchQuality }
    var trail: [(Double, Double)] { amclActive ? amclTrail : engine.trail }
    var submapCount: Int { amclActive ? amcl.size : engine.submapCount }
    var loopClosures: Int { amclActive ? 0 : engine.loopClosures }
    var relocalizeSeq: Int { engine.relocalizeSeq + amclSeq }
    var relocalizeFound: Bool { lastFromAmcl ? amclFound : engine.relocalizeFound }

    var mapping: Bool {
        get { isMapping }
        set {
            guard newValue != isMapping else { return }
            let p = pose
            isMapping = newValue
            if newValue {
                engine.mapping = true
                engine.resetPose(p)
            } else {
                engine.mapping = false
                if localizer == .amcl { startAmcl(p) }
            }
        }
    }

    func setEngine(_ t: SlamType) {
        guard t != engine.type else { return }
        let p = pose, old = engine
        let next = makeSlam(t, old.grid)
        if old.scans > 0 { next.setGrid(old.grid) }
        next.mapping = isMapping
        next.resetPose(p)
        engine = next
    }

    func setLocalizer(_ l: SlamLocalizer) {
        guard l != localizer else { return }
        let p = pose
        localizer = l
        if amclActive { startAmcl(p) } else { engine.resetPose(p) }
    }

    private func startAmcl(_ p: SlamPose) {
        amcl.initAt(p); odometry.reset(p); amclPose = p; lastImuYaw = .nan
    }

    func reset() { engine.reset(); startAmcl(.zero); amclTrail = [] }
    func setGrid(_ g: OccupancyGrid) { engine.setGrid(g); startAmcl(.zero) }
    func resetPose(_ p: SlamPose) { engine.resetPose(p); if amclActive { startAmcl(p) } }
    func requestGlobalLocalization() { if amclActive { amclSearchPending = true } else { engine.requestGlobalLocalization() } }

    @discardableResult
    func update(angles: [Double], ranges: [Double], imuYaw: Double, time: Double? = nil) -> SlamPose {
        if !amclActive {
            let before = engine.relocalizeSeq
            let p = engine.update(angles: angles, ranges: ranges, imuYaw: imuYaw, time: time)
            if engine.relocalizeSeq != before { lastFromAmcl = false }
            return p
        }
        let s = SlamScan(angles, ranges), g = engine.grid
        if amclSearchPending && s.n >= 20 { amclSearchPending = false; amcl.initGlobal(g) }
        var dTheta = 0.0
        if !imuYaw.isNaN {
            if !lastImuYaw.isNaN { dTheta = SlamPose.wrap(imuYaw - lastImuYaw) }
            lastImuYaw = imuYaw
        }
        let odo = odometry.pose
        let odom = odometry.update(s, SlamPose(odo.x, odo.y, odo.theta + dTheta))
        let wasSearching = amcl.globalSearch
        if amcl.update(odom, angles, ranges, g) {
            amclPose = amcl.estimate
            if wasSearching && !amcl.globalSearch { finishSearch(true) } else if wasSearching && amcl.updates > 60 { finishSearch(false) }
        } else {
            amclPose = amclPose.compose(odo.inverse.compose(odom))
        }
        amclQuality = SlamEngine.hitFraction(g, amclPose, s.px, s.py)
        amclScans += 1
        if let last = amclTrail.last, hypot(last.0 - amclPose.x, last.1 - amclPose.y) <= 0.05 {} else {
            amclTrail.append((amclPose.x, amclPose.y))
            if amclTrail.count > 5000 { amclTrail.removeFirst() }
        }
        return amclPose
    }

    private func finishSearch(_ found: Bool) { amclFound = found; lastFromAmcl = true; amclSeq += 1 }
}

// MARK: - /slam WebSocket (same format as the Android SlamWire; see SensorLinkDashboard.html)

enum SlamWire {
    static let inactive = "{\"type\":\"slam\",\"active\":false}"

    struct Command {
        var kind: String?   // goal, cancel, explore, findMe, place, mapping
        var x = 0.0, y = 0.0, theta = 0.0
        var flag = false
    }

    private static func f(_ v: Double) -> String { String(format: "%.3f", v) }

    private static func esc(_ s: String) -> String {
        var out = ""
        for ch in s.unicodeScalars {
            if ch == "\"" || ch == "\\" { out += "\\" + String(ch) } else if ch.value < 0x20 { out += String(format: "\\u%04x", ch.value) } else { out.unicodeScalars.append(ch) }
        }
        return out
    }

    static func json(engine: String, localizer: String, planner: String, controller: String, mapping: Bool, exploring: Bool,
                     pose: SlamPose, quality: Double, scans: Int, submaps: Int, loops: Int, goal: (Double, Double)?,
                     path: [(Double, Double)], status: String, map: OccupancyGrid?) -> String {
        var b = "{\"type\":\"slam\",\"active\":true"
        b += ",\"engine\":\"\(esc(engine))\",\"localizer\":\"\(esc(localizer))\",\"planner\":\"\(esc(planner))\",\"controller\":\"\(esc(controller))\""
        b += ",\"mapping\":\(mapping),\"exploring\":\(exploring)"
        b += ",\"pose\":[\(f(pose.x)),\(f(pose.y)),\(f(pose.theta))]"
        b += ",\"quality\":" + String(format: "%.2f", quality)
        b += ",\"scans\":\(scans),\"submaps\":\(submaps),\"loops\":\(loops)"
        b += ",\"goal\":" + (goal.map { "[\(f($0.0)),\(f($0.1))]" } ?? "null")
        let step = max(1, path.count / 150)
        var pts: [String] = []
        var i = 0
        while i < path.count { pts.append("[\(f(path[i].0)),\(f(path[i].1))]"); i += step }
        if let last = path.last, (path.count - 1) % step != 0 { pts.append("[\(f(last.0)),\(f(last.1))]") }
        b += ",\"path\":[" + pts.joined(separator: ",") + "]"
        b += ",\"status\":\"\(esc(status))\""
        if let g = map { b += ",\"map\":" + mapJSON(g) }
        return b + "}"
    }

    /// The known part of the map: one byte per cell, 0 unknown, 1 free, 2 wall, row 0 at the bottom.
    static func mapJSON(_ g: OccupancyGrid) -> String {
        var x0 = g.width, y0 = g.height, x1 = -1, y1 = -1
        for y in 0..<g.height {
            for x in 0..<g.width {
                let v = g.logOdds[y * g.width + x]
                if v > OccupancyGrid.occThreshold || v < OccupancyGrid.freeThreshold {
                    x0 = min(x0, x); x1 = max(x1, x); y0 = min(y0, y); y1 = max(y1, y)
                }
            }
        }
        if x1 < 0 { x0 = 0; y0 = 0; x1 = -1; y1 = -1 }
        let w = x1 - x0 + 1, h = y1 - y0 + 1
        var cells = [UInt8](repeating: 0, count: max(0, w * h))
        if w > 0 && h > 0 {
            for y in 0..<h {
                for x in 0..<w {
                    let v = g.logOdds[(y + y0) * g.width + x + x0]
                    cells[y * w + x] = v > OccupancyGrid.occThreshold ? 2 : v < OccupancyGrid.freeThreshold ? 1 : 0
                }
            }
        }
        return "{\"version\":\(g.version),\"res\":\(g.resolution),\"x0\":\(f(g.originX + Double(x0) * g.resolution)),\"y0\":\(f(g.originY + Double(y0) * g.resolution)),\"w\":\(max(0, w)),\"h\":\(max(0, h)),\"cells\":\"\(Data(cells).base64EncodedString())\"}"
    }

    /// Parses a page command; bad input gives kind == nil.
    static func parse(_ text: String) -> Command {
        var c = Command()
        guard let data = text.data(using: .utf8),
              let obj = (try? JSONSerialization.jsonObject(with: data)) as? [String: Any] else { return c }
        func nums(_ key: String) -> [Double]? { (obj[key] as? [Any])?.compactMap { ($0 as? NSNumber)?.doubleValue } }
        if let v = nums("goal"), v.count >= 2 { c.kind = "goal"; c.x = v[0]; c.y = v[1] }
        else if let v = nums("place"), v.count >= 3 { c.kind = "place"; c.x = v[0]; c.y = v[1]; c.theta = v[2] }
        else if (obj["cancel"] as? Bool) == true { c.kind = "cancel" }
        else if (obj["findMe"] as? Bool) == true { c.kind = "findMe" }
        else if let b = obj["explore"] as? Bool { c.kind = "explore"; c.flag = b }
        else if let b = obj["mapping"] as? Bool { c.kind = "mapping"; c.flag = b }
        if c.kind != nil && !(c.x.isFinite && c.y.isFinite && c.theta.isFinite) { c.kind = nil }
        return c
    }
}

/// What the Indoor SLAM screen offers the Sensor link server; called on the server's queue.
protocol SlamHubSource: AnyObject {
    func slamJSON(withMap: Bool) -> String
    var slamMapVersion: Int { get }
    func slamCommand(_ text: String)
}

/// Hands the open SLAM screen to the server's /slam WebSocket.
final class SlamHub {
    static let shared = SlamHub()
    private let lock = NSLock()
    private weak var source: SlamHubSource?

    func setSource(_ s: SlamHubSource) { lock.lock(); source = s; lock.unlock() }
    func clearSource(_ s: SlamHubSource) { lock.lock(); if source === s { source = nil }; lock.unlock() }
    private var current: SlamHubSource? { lock.lock(); defer { lock.unlock() }; return source }

    func json(withMap: Bool) -> String { current?.slamJSON(withMap: withMap) ?? SlamWire.inactive }
    var mapVersion: Int { current?.slamMapVersion ?? -1 }
    func command(_ text: String) { current?.slamCommand(text) }
}
