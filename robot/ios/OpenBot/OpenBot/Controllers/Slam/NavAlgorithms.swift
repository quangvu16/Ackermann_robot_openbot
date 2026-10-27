//
//  NavAlgorithms.swift
//  OpenBot
//
//  The Nav2-style planners and controllers the user can choose; the same as
//  the Android classes Costmap, Planners and Controllers in
//  org.openbot.app.robot.slam - keep them in step. Written for this project
//  from the papers and Nav2's documentation; no Nav2 code is used.
//

import Foundation

enum NavPlanner: Int, CaseIterable {
    case navfn, thetaStar, hybridAStar
    var label: String { ["NavFn (A*)", "Theta*", "Smac Hybrid-A*"][rawValue] }
}

enum NavController: Int, CaseIterable {
    case rpp, dwb, mppi
    var label: String { ["Regulated Pure Pursuit", "DWB", "MPPI"][rawValue] }
}

/// A path point: x, y and the driving direction (+1 forward, -1 reverse).
typealias NavPoint = (Double, Double, Int)

/// Inflated costmap on a 10 cm grid: -1 lethal, 1 free .. 9 next to lethal, unknown 3.
final class NavCostmap {
    let w: Int, h: Int, res: Double
    let grid: OccupancyGrid
    var cost: [Float]

    init(_ grid: OccupancyGrid, _ p: SlamNavigator.Params) {
        self.grid = grid
        let factor = max(1, Int((0.10 / grid.resolution).rounded()))
        res = grid.resolution * Double(factor)
        w = grid.width / factor
        h = grid.height / factor
        cost = [Float](repeating: 1, count: w * h)
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
                        if d <= Double(rL) { cost[i] = -1 } else if d <= Double(rS) && cost[i] >= 0 {
                            cost[i] = max(cost[i], Float(1 + 8 * (1 - (d - Double(rL)) / (Double(rS - rL) + 1e-9))))
                        }
                    }
                }
            }
        }
    }

    func cx(_ v: Double) -> Int { Int(floor((v - grid.originX) / res)) }
    func cy(_ v: Double) -> Int { Int(floor((v - grid.originY) / res)) }
    func free(_ x: Int, _ y: Int) -> Bool { x >= 0 && y >= 0 && x < w && y < h && cost[y * w + x] >= 0 }
    func wx(_ x: Int) -> Double { grid.originX + (Double(x) + 0.5) * res }
    func wy(_ y: Int) -> Double { grid.originY + (Double(y) + 0.5) * res }
    func at(_ x: Double, _ y: Double) -> Float { let i = cx(x), j = cy(y); return free(i, j) ? cost[j * w + i] : -1 }
    func normalized(_ x: Double, _ y: Double) -> Double { let c = at(x, y); return c < 0 ? 1 : Double(c - 1) / 8 }
}

enum NavPlanners {
    static func plan(_ m: NavCostmap, _ start: SlamPose, _ goal: (Double, Double), _ p: SlamNavigator.Params) -> [NavPoint]? {
        switch p.planner {
        case .navfn: return navfn(m, (start.x, start.y), goal)?.map { ($0.0, $0.1, 1) }
        case .thetaStar: return thetaStar(m, (start.x, start.y), goal)?.map { ($0.0, $0.1, 1) }
        case .hybridAStar: return hybridAStar(m, start, goal, p)
        }
    }

    static let dxs = [1, -1, 0, 0, 1, 1, -1, -1], dys = [0, 0, 1, -1, 1, -1, 1, -1]

    static func densify(_ pts: [(Double, Double)], _ step: Double) -> [(Double, Double)] {
        var out: [(Double, Double)] = []
        for k in 0..<(pts.count - 1) {
            let a = pts[k], b = pts[k + 1]
            let n = max(1, Int(ceil(hypot(b.0 - a.0, b.1 - a.1) / step)))
            for q in 0..<n { let t = Double(q) / Double(n); out.append((a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t)) }
        }
        out.append(pts[pts.count - 1])
        return out
    }

    /// NavFn: A* on cost-weighted 8-connected cells, then cheap straight-line shortcuts.
    static func navfn(_ m: NavCostmap, _ s: (Double, Double), _ g: (Double, Double)) -> [(Double, Double)]? {
        let x0 = m.cx(s.0), y0 = m.cy(s.1), x1 = m.cx(g.0), y1 = m.cy(g.1)
        guard m.free(x1, y1), x0 >= 0, y0 >= 0, x0 < m.w, y0 < m.h else { return nil }
        let n = m.w * m.h, start = y0 * m.w + x0, target = y1 * m.w + x1
        var gs = [Float](repeating: .greatestFiniteMagnitude, count: n), from = [Int](repeating: -1, count: n)
        var closed = [Bool](repeating: false, count: n)
        gs[start] = 0
        func octile(_ ax: Int, _ ay: Int) -> Double {
            let dx = Double(abs(x1 - ax)), dy = Double(abs(y1 - ay))
            return max(dx, dy) + (2.0.squareRoot() - 1) * min(dx, dy)
        }
        var open = MinHeap()
        open.push(octile(x0, y0), start)
        var expanded = 0
        while let (_, cur) = open.pop(), expanded < 400_000 {
            if closed[cur] { continue }
            closed[cur] = true; expanded += 1
            if cur == target { break }
            let ux = cur % m.w, uy = cur / m.w
            for k in 0..<8 {
                let nx = ux + dxs[k], ny = uy + dys[k]
                guard m.free(nx, ny) else { continue }
                let ni = ny * m.w + nx
                if closed[ni] { continue }
                let ng = gs[cur] + Float((k < 4 ? 1 : 2.0.squareRoot()) * Double(m.cost[ni]))
                if ng < gs[ni] { gs[ni] = ng; from[ni] = cur; open.push(Double(ng) + octile(nx, ny), ni) }
            }
        }
        if from[target] < 0 && target != start { return nil }
        var cells: [(Int, Int)] = []
        var c = target
        while c != -1 { cells.insert((c % m.w, c / m.w), at: 0); if c == start { break }; c = from[c] }
        func lineCheap(_ a: (Int, Int), _ b: (Int, Int)) -> Bool {
            let steps = max(abs(b.0 - a.0), abs(b.1 - a.1))
            for st in 0...max(0, steps) {
                let t = steps == 0 ? 0 : Double(st) / Double(steps)
                let x = Int((Double(a.0) + Double(b.0 - a.0) * t).rounded()), y = Int((Double(a.1) + Double(b.1 - a.1) * t).rounded())
                if !m.free(x, y) || m.cost[y * m.w + x] > 4 { return false }
            }
            return true
        }
        var out: [(Double, Double)] = [s]
        var i = 0
        while i < cells.count - 1 {
            var j = cells.count - 1
            while j > i + 1 && !lineCheap(cells[i], cells[j]) { j -= 1 }
            out.append((m.wx(cells[j].0), m.wy(cells[j].1)))
            i = j
        }
        out[out.count - 1] = g
        return densify(out, 0.1)
    }

    // Theta*: w_euc_cost 1.0, w_traversal_cost 2.0 (Nav2 defaults).
    static let wEuc = 1.0, wTraversal = 2.0

    static func thetaStar(_ m: NavCostmap, _ s: (Double, Double), _ g: (Double, Double)) -> [(Double, Double)]? {
        let x0 = m.cx(s.0), y0 = m.cy(s.1), x1 = m.cx(g.0), y1 = m.cy(g.1)
        guard m.free(x1, y1), x0 >= 0, y0 >= 0, x0 < m.w, y0 < m.h else { return nil }
        let n = m.w * m.h, start = y0 * m.w + x0, target = y1 * m.w + x1
        var gs = [Double](repeating: .greatestFiniteMagnitude, count: n), parent = [Int](repeating: -1, count: n)
        var closed = [Bool](repeating: false, count: n)
        gs[start] = 0; parent[start] = start
        func segCost(_ ax: Int, _ ay: Int, _ bx: Int, _ by: Int) -> Double {
            let steps = max(abs(bx - ax), abs(by - ay)), len = hypot(Double(bx - ax), Double(by - ay))
            var sum = 0.0
            if steps > 0 {
                for st in 1...steps {
                    let t = Double(st) / Double(steps)
                    let x = Int((Double(ax) + Double(bx - ax) * t).rounded()), y = Int((Double(ay) + Double(by - ay) * t).rounded())
                    let c = Double(m.cost[y * m.w + x] - 1) / 8
                    sum += c * c
                }
            }
            return wEuc * len + wTraversal * len * (steps == 0 ? 0 : sum / Double(steps))
        }
        func sight(_ ax: Int, _ ay: Int, _ bx: Int, _ by: Int) -> Bool {
            let steps = max(abs(bx - ax), abs(by - ay))
            for st in 0...max(0, steps) {
                let t = steps == 0 ? 0 : Double(st) / Double(steps)
                if !m.free(Int((Double(ax) + Double(bx - ax) * t).rounded()), Int((Double(ay) + Double(by - ay) * t).rounded())) { return false }
            }
            return true
        }
        var open = MinHeap()
        open.push(hypot(Double(x1 - x0), Double(y1 - y0)) * wEuc, start)
        var expanded = 0
        while let (_, cur) = open.pop(), expanded < 400_000 {
            if closed[cur] { continue }
            closed[cur] = true; expanded += 1
            if cur == target { break }
            let ux = cur % m.w, uy = cur / m.w, par = parent[cur], px = par % m.w, py = par / m.w
            for k in 0..<8 {
                let nx = ux + dxs[k], ny = uy + dys[k]
                guard m.free(nx, ny) else { continue }
                let ni = ny * m.w + nx
                if closed[ni] { continue }
                var ng: Double, np: Int
                if par != cur && sight(px, py, nx, ny) { ng = gs[par] + segCost(px, py, nx, ny); np = par } else { ng = gs[cur] + segCost(ux, uy, nx, ny); np = cur }
                if ng < gs[ni] { gs[ni] = ng; parent[ni] = np; open.push(ng + hypot(Double(x1 - nx), Double(y1 - ny)) * wEuc, ni) }
            }
        }
        guard parent[target] >= 0 else { return nil }
        var out: [(Double, Double)] = []
        var c = target
        while true {
            out.insert((m.wx(c % m.w), m.wy(c / m.w)), at: 0)
            if c == start || parent[c] == c { break }
            c = parent[c]
        }
        out[0] = s
        if out.count == 1 { out.append(g) } else { out[out.count - 1] = g }
        return densify(out, 0.1)
    }

    // Smac Hybrid-A* (Reeds-Shepp): 72 headings; non_straight 1.2, reverse 2.0, change 0.0, cost 2.0.
    static let angleBins = 72, nonStraightPenalty = 1.2, reversePenalty = 2.0, changePenalty = 0.0, costPenalty = 2.0

    private final class HNode {
        let x: Double, y: Double, th: Double, dir: Int, g: Double
        let parent: HNode?
        init(_ x: Double, _ y: Double, _ th: Double, _ dir: Int, _ g: Double, _ parent: HNode?) {
            self.x = x; self.y = y; self.th = th; self.dir = dir; self.g = g; self.parent = parent
        }
    }

    static func hybridAStar(_ m: NavCostmap, _ start: SlamPose, _ goal: (Double, Double), _ p: SlamNavigator.Params) -> [NavPoint]? {
        let x1 = m.cx(goal.0), y1 = m.cy(goal.1)
        guard m.free(x1, y1) else { return nil }
        let sx = m.cx(start.x), sy = m.cy(start.y)
        guard sx >= 0, sy >= 0, sx < m.w, sy < m.h else { return nil }
        // Obstacle heuristic: Dijkstra distance (m) from the goal.
        var heur = [Float](repeating: .greatestFiniteMagnitude, count: m.w * m.h)
        var q = MinHeap()
        heur[y1 * m.w + x1] = 0
        q.push(0, y1 * m.w + x1)
        while let (d, cur) = q.pop() {
            if Float(d) > heur[cur] { continue }
            let ux = cur % m.w, uy = cur / m.w
            for k in 0..<8 {
                let nx = ux + dxs[k], ny = uy + dys[k]
                guard m.free(nx, ny) else { continue }
                let ni = ny * m.w + nx, nd = heur[cur] + Float((k < 4 ? 1 : 2.0.squareRoot()) * m.res)
                if nd < heur[ni] { heur[ni] = nd; q.push(Double(nd), ni) }
            }
        }
        func h(_ x: Double, _ y: Double) -> Double {
            let cx = m.cx(x), cy = m.cy(y)
            guard cx >= 0, cy >= 0, cx < m.w, cy < m.h, heur[cy * m.w + cx] < .greatestFiniteMagnitude else { return 1e18 }
            return Double(heur[cy * m.w + cx])
        }
        let radius = p.wheelbase / tan(p.maxSteerDeg * .pi / 180)
        let step = 2.0.squareRoot() * m.res, dTheta = step / radius, binSize = 2 * Double.pi / Double(angleBins)
        func bin(_ th: Double) -> Int { ((Int((th / binSize).rounded()) % angleBins) + angleBins) % angleBins }
        var closed = [Bool](repeating: false, count: m.w * m.h * angleBins)
        var nodes: [HNode] = [HNode(start.x, start.y, start.theta, 1, 0, nil)]
        var open = MinHeap()
        open.push(h(start.x, start.y), 0)
        var goalNode: HNode?
        var iterations = 0
        while let (_, idx) = open.pop(), iterations < 200_000 {
            iterations += 1
            let cur = nodes[idx]
            let key = (m.cy(cur.y) * m.w + m.cx(cur.x)) * angleBins + bin(cur.th)
            if key < 0 || key >= closed.count || closed[key] { continue }
            closed[key] = true
            if hypot(cur.x - goal.0, cur.y - goal.1) <= 1.5 * m.res { goalNode = cur; break }
            for dir in [1, -1] {
                for turn in -1...1 {
                    let dth = Double(turn) * dTheta, nth = cur.th + dth
                    var nx: Double, ny: Double
                    if turn == 0 {
                        nx = cur.x + Double(dir) * step * cos(cur.th); ny = cur.y + Double(dir) * step * sin(cur.th)
                    } else {
                        let r = Double(dir) * step / dth
                        nx = cur.x + r * (sin(nth) - sin(cur.th)); ny = cur.y - r * (cos(nth) - cos(cur.th))
                    }
                    let cEnd = m.at(nx, ny), cMid = m.at((cur.x + nx) / 2, (cur.y + ny) / 2)
                    if cEnd < 0 || cMid < 0 { continue }
                    let nkey = (m.cy(ny) * m.w + m.cx(nx)) * angleBins + bin(nth)
                    if closed[nkey] { continue }
                    var travel = step * (1 + costPenalty * Double(cEnd - 1) / 8)
                    if turn != 0 { travel *= nonStraightPenalty }
                    if dir < 0 { travel *= reversePenalty }
                    if dir != cur.dir { travel += changePenalty }
                    let next = HNode(nx, ny, nth, dir, cur.g + travel, cur)
                    nodes.append(next)
                    open.push(next.g + h(nx, ny), nodes.count - 1)
                }
            }
        }
        guard let gn = goalNode else { return nil }
        var out: [NavPoint] = []
        var k: HNode? = gn
        while let node = k {
            let dir = node.parent == nil ? (out.first?.2 ?? 1) : node.dir
            out.insert((node.x, node.y, dir), at: 0)
            k = node.parent
        }
        out.append((goal.0, goal.1, out.last!.2))
        return out
    }
}

enum NavControllers {
    static let vMax = 0.6 // m/s at throttle 1 (model)

    static func simulate(_ x: Double, _ y: Double, _ th: Double, _ throttle: Double, _ steer: Double,
                         _ p: SlamNavigator.Params, _ dt: Double) -> (Double, Double, Double) {
        let v = throttle * vMax, delta = steer * p.maxSteerDeg * .pi / 180
        let nth = th + abs(v) / p.wheelbase * tan(delta) * dt
        return (x + v * cos(nth) * dt, y + v * sin(nth) * dt, nth)
    }

    static func nearest(_ path: [NavPoint], _ x: Double, _ y: Double, _ lo: Int = 0, _ hi: Int = Int.max) -> Int {
        var best = max(0, lo), bd = Double.greatestFiniteMagnitude
        for i in max(0, lo)...min(path.count - 1, hi) {
            let d = hypot(path[i].0 - x, path[i].1 - y)
            if d < bd { bd = d; best = i }
        }
        return best
    }

    static func dist(_ path: [NavPoint], _ x: Double, _ y: Double, _ lo: Int = 0, _ hi: Int = Int.max) -> Double {
        let i = nearest(path, x, y, lo, hi)
        return hypot(path[i].0 - x, path[i].1 - y)
    }

    static func remaining(_ path: [NavPoint], _ i: Int) -> Double {
        var s = 0.0
        if i < path.count - 1 { for k in i..<(path.count - 1) { s += hypot(path[k + 1].0 - path[k].0, path[k + 1].1 - path[k].1) } }
        return s
    }

    /// DWB: (throttle, steer) grid driven for 1.7 s, scored by Nav2's critics.
    final class Dwb {
        static let simTime = 1.7, baseObstacle = 0.02, pathDist = 32.0, pathAlign = 32.0, goalDist = 24.0, goalAlign = 24.0
        static let oscillation = 5.0, forwardPoint = 0.1
        private var lastThrottle = 0.0

        func step(_ pose: SlamPose, _ path: [NavPoint], _ m: NavCostmap, _ p: SlamNavigator.Params) -> (Double, Double) {
            var best = Double.greatestFiniteMagnitude, cmd = (0.0, 0.0)
            for tf in [-0.5, -0.25, 0.25, 0.5, 0.75, 1.0] {
                let t = tf * p.speed
                for k in 0..<11 {
                    let s = -1 + 2 * Double(k) / 10
                    var x = pose.x, y = pose.y, th = pose.theta, obstacle = 0.0, hit = false
                    var tt = 0.0
                    while tt < Self.simTime {
                        (x, y, th) = simulate(x, y, th, t, s, p, 0.1)
                        let c = m.at(x, y)
                        if c < 0 { hit = true; break }
                        obstacle = max(obstacle, Double(c - 1) / 8 * 252)
                        tt += 0.1
                    }
                    if hit { continue }
                    let sg = t >= 0 ? 1.0 : -1.0
                    let fx = x + Self.forwardPoint * cos(th) * sg, fy = y + Self.forwardPoint * sin(th) * sg
                    var score = Self.baseObstacle * obstacle
                    score += Self.pathDist * dist(path, x, y) / m.res + Self.pathAlign * dist(path, fx, fy) / m.res
                    score += Self.goalDist * remaining(path, nearest(path, x, y)) / m.res
                    score += Self.goalAlign * remaining(path, nearest(path, fx, fy)) / m.res
                    if lastThrottle != 0 && (t >= 0) != (lastThrottle >= 0) { score += Self.oscillation * 10 }
                    if score < best { best = score; cmd = (t, s) }
                }
            }
            lastThrottle = cmd.0
            return cmd
        }
    }

    /// MPPI: 300 smoothed noisy versions of the plan, 56 steps of 0.1 s, Nav2 critic weights.
    final class Mppi {
        static let batch = 300, steps = 56, dt = 0.1, vStd = 0.4, steerStd = 0.3, temperature = 0.3
        static let wConstraint = 4.0, wCost = 3.81, wGoal = 5.0, wPathAlign = 14.0, wPathFollow = 10.0
        static let wPathAngle = 2.2, wPreferForward = 5.0, collisionCost = 1e6
        private var uv = [Double](repeating: 0, count: Mppi.steps), us = [Double](repeating: 0, count: Mppi.steps)
        private var rng = SeededRandom(seed: 3)

        func reset() { uv = [Double](repeating: 0, count: Self.steps); us = uv }

        func step(_ pose: SlamPose, _ path: [NavPoint], _ goal: (Double, Double), _ m: NavCostmap, _ p: SlamNavigator.Params) -> (Double, Double) {
            let n = Self.batch, T = Self.steps
            let vMaxF = p.speed, vMinF = -0.6 * p.speed
            var ev = [[Double]](repeating: [Double](repeating: 0, count: T), count: n), es = ev
            var cost = [Double](repeating: 0, count: n)
            let near = nearest(path, pose.x, pose.y)
            let ahead = Int(ceil(p.speed * vMax * Double(T) * Self.dt / 0.1))
            let fp = path[min(path.count - 1, near + max(6, ahead))]
            let goalNear = hypot(goal.0 - pose.x, goal.1 - pose.y) < 1.4
            for b in 0..<n {
                var x = pose.x, y = pose.y, th = pose.theta, c = 0.0, align = 0.0, nv = 0.0, ns = 0.0
                for t in 0..<T {
                    if b != 0 {
                        nv = 0.8 * nv + 0.6 * rng.gaussian() * Self.vStd * p.speed
                        ns = 0.8 * ns + 0.6 * rng.gaussian() * Self.steerStd
                    }
                    ev[b][t] = nv; es[b][t] = ns
                    var v = uv[t] + nv
                    let s = min(1, max(-1, us[t] + ns))
                    if v > vMaxF { c += Self.wConstraint * (v - vMaxF) * 10 }
                    if v < vMinF { c += Self.wConstraint * (vMinF - v) * 10 }
                    v = min(vMaxF, max(vMinF, v))
                    (x, y, th) = simulate(x, y, th, v, s, p, Self.dt)
                    if m.at(x, y) < 0 { c += Self.collisionCost } else { c += Self.wCost * m.normalized(x, y) / Double(T) }
                    align += dist(path, x, y, near - 5, near + 40)
                    if v < 0 { c += Self.wPreferForward * -v * vMax * Self.dt }
                }
                c += Self.wPathAlign * align / Double(T)
                if goalNear { c += Self.wGoal * hypot(goal.0 - x, goal.1 - y) } else { c += Self.wPathFollow * hypot(fp.0 - x, fp.1 - y) }
                let ang = abs(SlamPose.wrap(atan2(fp.1 - y, fp.0 - x) - th))
                if ang > .pi / 2 { c += Self.wPathAngle * ang }
                cost[b] = c
            }
            let minC = cost.min() ?? 0
            let w = cost.map { exp(-($0 - minC) / Self.temperature) }
            let sum = w.reduce(0, +)
            for t in 0..<T {
                var dv = 0.0, ds = 0.0
                for b in 0..<n { dv += w[b] * ev[b][t]; ds += w[b] * es[b][t] }
                uv[t] = min(vMaxF, max(vMinF, uv[t] + dv / sum))
                us[t] = min(1, max(-1, us[t] + ds / sum))
            }
            let sv = uv, ss = us
            for t in 1..<(T - 1) { uv[t] = (sv[t - 1] + sv[t] + sv[t + 1]) / 3; us[t] = (ss[t - 1] + ss[t] + ss[t + 1]) / 3 }
            let cmd = (uv[0], us[0])
            uv = Array(uv.dropFirst()) + [uv[T - 1]]
            us = Array(us.dropFirst()) + [us[T - 1]]
            return cmd
        }
    }
}
