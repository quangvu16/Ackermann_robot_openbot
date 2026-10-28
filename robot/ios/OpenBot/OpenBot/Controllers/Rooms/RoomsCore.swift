//
// Rooms & Zones (Valetudo-style) core: rooms, restrictions, lanes and the job runner.
// Same logic as the Android classes RoomMap, RoomSegmenter, CoveragePlanner and CleaningMission
// (robot/android/.../slam), which are unit tested on the JVM; tests here are in OpenBotTests.
//

import Foundation

// MARK: - RoomMap

/// What the user adds on top of a saved SLAM map, as Valetudo does on a vacuum's map: rooms
/// ("segments"), zones, no-go areas, virtual walls and home. World metres, x right, y up.
final class RoomMap {
    static let wallMargin = 0.05

    // Room labels (0 = none) on a 10 cm grid aligned with the SLAM grid.
    var w = 0, h = 0, factor = 1
    var res = 0.1, originX = 0.0, originY = 0.0
    var labels: [Int32] = []
    var names: [Int: String] = [:]

    var zones: [[Double]] = []   // 8 values: 4 corners
    var noGo: [[Double]] = []    // 8 values
    var walls: [[Double]] = []   // xA, yA, xB, yB
    var home: (Double, Double)?
    var mapStamp = ""
    var robotRadius = 0.22

    var hasRooms: Bool { !names.isEmpty }
    var hasRestrictions: Bool { !noGo.isEmpty || !walls.isEmpty }
    var roomIds: [Int] { names.keys.sorted() }

    func roomAt(_ x: Double, _ y: Double) -> Int {
        let l = labels
        guard !l.isEmpty else { return 0 }
        let cx = Int(floor((x - originX) / res)), cy = Int(floor((y - originY) / res))
        guard cx >= 0, cy >= 0, cx < w, cy < h else { return 0 }
        let i = cy * w + cx
        return i < l.count ? Int(l[i]) : 0
    }

    func name(_ id: Int) -> String { names[id] ?? "Room \(id)" }

    func area(_ id: Int) -> Double { Double(labels.filter { $0 == Int32(id) }.count) * res * res }

    func centroid(_ id: Int) -> (Double, Double)? {
        var sx = 0.0, sy = 0.0, n = 0
        for y in 0..<h { for x in 0..<w where labels[y * w + x] == Int32(id) { sx += Double(x); sy += Double(y); n += 1 } }
        guard n > 0 else { return nil }
        return (originX + (sx / Double(n) + 0.5) * res, originY + (sy / Double(n) + 0.5) * res)
    }

    func setRooms(_ r: RoomSegmenter.Result, stamp: String) {
        w = r.w; h = r.h; factor = r.factor; res = r.res; originX = r.originX; originY = r.originY
        labels = r.labels
        names = [:]
        if r.count > 0 { for id in 1...r.count { names[id] = "Room \(id)" } }
        mapStamp = stamp
    }

    static func stamp(of g: OccupancyGrid) -> String {
        var occ = 0, free = 0
        for v in g.logOdds { if v > OccupancyGrid.occThreshold { occ += 1 } else if v < OccupancyGrid.freeThreshold { free += 1 } }
        return "\(g.width)x\(g.height)@" + String(format: "%.3f:%.2f,%.2f", g.resolution, g.originX, g.originY) + ":\(occ)/\(free)"
    }

    /// No-go areas and virtual walls (the car's centre stays its radius away from a wall line).
    func blocked(_ x: Double, _ y: Double) -> Bool {
        for p in noGo where RoomMap.inside(p, x, y) { return true }
        let clear = robotRadius + RoomMap.wallMargin
        for s in walls where RoomMap.segmentDistance(s[0], s[1], s[2], s[3], x, y) < clear { return true }
        return false
    }

    static func box(_ ax: Double, _ ay: Double, _ bx: Double, _ by: Double) -> [Double] {
        let x0 = min(ax, bx), x1 = max(ax, bx), y0 = min(ay, by), y1 = max(ay, by)
        return [x0, y0, x1, y0, x1, y1, x0, y1]
    }

    static func inside(_ poly: [Double], _ x: Double, _ y: Double) -> Bool {
        var inside = false
        let n = poly.count / 2
        var j = n - 1
        for i in 0..<n {
            let xi = poly[2 * i], yi = poly[2 * i + 1], xj = poly[2 * j], yj = poly[2 * j + 1]
            if (yi > y) != (yj > y) && x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi { inside.toggle() }
            j = i
        }
        return inside
    }

    static func segmentDistance(_ ax: Double, _ ay: Double, _ bx: Double, _ by: Double, _ x: Double, _ y: Double) -> Double {
        let dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy
        let t = l2 < 1e-12 ? 0 : max(0, min(1, ((x - ax) * dx + (y - ay) * dy) / l2))
        return hypot(ax + t * dx - x, ay + t * dy - y)
    }

    static func edgeDistance(_ p: [Double], _ x: Double, _ y: Double) -> Double {
        var d = Double.greatestFiniteMagnitude
        let n = p.count / 2
        var j = n - 1
        for i in 0..<n {
            d = min(d, segmentDistance(p[2 * j], p[2 * j + 1], p[2 * i], p[2 * i + 1], x, y))
            j = i
        }
        return d
    }

    /// The zone (0), no-go area (1) or wall (2) nearest a tap, within `within` metres.
    func itemAt(_ x: Double, _ y: Double, within: Double) -> (kind: Int, index: Int)? {
        var best: (Int, Int)?
        var bd = within
        for (k, list) in [zones, noGo].enumerated() {
            for (i, p) in list.enumerated() {
                let d = RoomMap.inside(p, x, y) ? 0 : RoomMap.edgeDistance(p, x, y)
                if d <= bd { bd = d; best = (k, i) }
            }
        }
        for (i, s) in walls.enumerated() {
            let d = RoomMap.segmentDistance(s[0], s[1], s[2], s[3], x, y)
            if d <= bd { bd = d; best = (2, i) }
        }
        return best.map { (kind: $0.0, index: $0.1) }
    }

    // MARK: JSON (same file format as Android's rooms.json)

    func toJson() -> String {
        var rle: [Int] = []
        var i = 0
        while i < labels.count {
            let v = labels[i]
            var j = i
            while j < labels.count && labels[j] == v { j += 1 }
            rle.append(Int(v)); rle.append(j - i)
            i = j
        }
        var o: [String: Any] = [
            "version": 1, "mapStamp": mapStamp,
            "rooms": ["w": w, "h": h, "factor": factor, "res": res, "originX": originX, "originY": originY, "rle": rle,
                      "names": Dictionary(uniqueKeysWithValues: names.map { (String($0.key), $0.value) })] as [String: Any],
            "zones": zones, "noGo": noGo, "walls": walls,
        ]
        o["home"] = home.map { [$0.0, $0.1] } ?? NSNull()
        let d = (try? JSONSerialization.data(withJSONObject: o, options: [.sortedKeys])) ?? Data()
        return String(data: d, encoding: .utf8) ?? "{}"
    }

    static func fromJson(_ text: String) -> RoomMap {
        let m = RoomMap()
        guard let d = text.data(using: .utf8), let o = (try? JSONSerialization.jsonObject(with: d)) as? [String: Any] else { return m }
        m.mapStamp = o["mapStamp"] as? String ?? ""
        if let r = o["rooms"] as? [String: Any] {
            m.w = (r["w"] as? NSNumber)?.intValue ?? 0
            m.h = (r["h"] as? NSNumber)?.intValue ?? 0
            m.factor = (r["factor"] as? NSNumber)?.intValue ?? 1
            m.res = (r["res"] as? NSNumber)?.doubleValue ?? 0.1
            m.originX = (r["originX"] as? NSNumber)?.doubleValue ?? 0
            m.originY = (r["originY"] as? NSNumber)?.doubleValue ?? 0
            var labels = [Int32](repeating: 0, count: max(0, m.w * m.h))
            var at = 0
            let rle = (r["rle"] as? [NSNumber])?.map { $0.intValue } ?? []
            var k = 0
            while k + 1 < rle.count {
                for _ in 0..<rle[k + 1] where at < labels.count { labels[at] = Int32(rle[k]); at += 1 }
                k += 2
            }
            m.labels = at == labels.count ? labels : []
            if !m.labels.isEmpty, let names = r["names"] as? [String: Any] {
                for (key, v) in names { if let id = Int(key), let s = v as? String { m.names[id] = s } }
            }
        }
        func list(_ key: String, _ len: Int) -> [[Double]] {
            ((o[key] as? [[NSNumber]]) ?? []).filter { $0.count == len }.map { $0.map { $0.doubleValue } }
        }
        m.zones = list("zones", 8)
        m.noGo = list("noGo", 8)
        m.walls = list("walls", 4)
        if let h = o["home"] as? [NSNumber], h.count == 2 { m.home = (h[0].doubleValue, h[1].doubleValue) }
        return m
    }
}

// MARK: - RoomSegmenter

/// Distance-transform room segmentation (Bormann et al., ICRA 2016): room cores far from walls,
/// grown back over the free space they are nearest to, so rooms split at doorways.
enum RoomSegmenter {
    struct Params {
        var coreClearance = 0.55  // m from walls; doorways up to twice this split rooms
        var minCoreArea = 0.25    // m²
        var minRoomArea = 1.0     // m²; smaller rooms join a neighbour
    }

    struct Result {
        var w = 0, h = 0, factor = 1, res = 0.1, originX = 0.0, originY = 0.0
        var labels: [Int32] = []
        var count = 0
    }

    private static let dx = [1, -1, 0, 0], dy = [0, 0, 1, -1]

    static func segment(_ grid: OccupancyGrid, _ p: Params = Params()) -> Result {
        var r = Result()
        r.factor = max(1, Int((0.10 / grid.resolution).rounded()))
        r.res = grid.resolution * Double(r.factor)
        r.w = grid.width / r.factor; r.h = grid.height / r.factor
        r.originX = grid.originX; r.originY = grid.originY
        let w = r.w, h = r.h, n = w * h
        var free = [Bool](repeating: false, count: n)
        for y in 0..<h {
            for x in 0..<w {
                var occ = false, known = false
                outer: for ddy in 0..<r.factor {
                    for ddx in 0..<r.factor {
                        let v = grid.get(x * r.factor + ddx, y * r.factor + ddy)
                        if v > OccupancyGrid.occThreshold { occ = true; break outer }
                        if v < OccupancyGrid.freeThreshold { known = true }
                    }
                }
                free[y * w + x] = known && !occ
            }
        }
        let dist = chamfer(free, w, h)
        let coreCells = Int(ceil(p.coreClearance / r.res * 3))
        let minCore = Int(ceil(p.minCoreArea / (r.res * r.res)))
        var labels = [Int32](repeating: 0, count: n)
        var next: Int32 = 0
        var queue: [Int] = []
        for i in 0..<n where labels[i] == 0 && dist[i] >= coreCells {
            next += 1
            var comp: [Int] = []
            labels[i] = next
            queue = [i]
            var head = 0
            while head < queue.count {
                let c = queue[head]; head += 1
                comp.append(c)
                let cx = c % w, cy = c / w
                for k in 0..<4 {
                    let nx = cx + dx[k], ny = cy + dy[k]
                    guard nx >= 0, ny >= 0, nx < w, ny < h else { continue }
                    let j = ny * w + nx
                    if labels[j] == 0 && dist[j] >= coreCells { labels[j] = next; queue.append(j) }
                }
            }
            if comp.count < minCore { for c in comp { labels[c] = -1 }; next -= 1 }
        }
        for i in 0..<n where labels[i] < 0 { labels[i] = 0 }
        // Grow cores over the free space nearest to them.
        queue = (0..<n).filter { labels[$0] > 0 }
        var head = 0
        while head < queue.count {
            let c = queue[head]; head += 1
            let cx = c % w, cy = c / w
            for k in 0..<4 {
                let nx = cx + dx[k], ny = cy + dy[k]
                guard nx >= 0, ny >= 0, nx < w, ny < h else { continue }
                let j = ny * w + nx
                if free[j] && labels[j] == 0 { labels[j] = labels[c]; queue.append(j) }
            }
        }
        // Join small rooms to the neighbour with the longest shared border.
        let minRoom = Int(ceil(p.minRoomArea / (r.res * r.res)))
        var changed = true
        while changed && next > 0 {
            changed = false
            var area = [Int](repeating: 0, count: Int(next) + 1)
            for l in labels where l > 0 { area[Int(l)] += 1 }
            for id in 1...Int(next) where area[id] > 0 && area[id] < minRoom {
                var border = [Int](repeating: 0, count: Int(next) + 1)
                for i in 0..<n where labels[i] == Int32(id) {
                    let cx = i % w, cy = i / w
                    for k in 0..<4 {
                        let nx = cx + dx[k], ny = cy + dy[k]
                        guard nx >= 0, ny >= 0, nx < w, ny < h else { continue }
                        let l = Int(labels[ny * w + nx])
                        if l > 0 && l != id { border[l] += 1 }
                    }
                }
                var best = 0
                for l in 1...Int(next) where border[l] > border[best] { best = l }
                for i in 0..<n where labels[i] == Int32(id) { labels[i] = Int32(best) }
                changed = true
                break
            }
        }
        // Number by size, largest first.
        var area = [Int](repeating: 0, count: Int(next) + 1)
        for l in labels where l > 0 { area[Int(l)] += 1 }
        let order = next > 0 ? (1...Int(next)).sorted { area[$0] != area[$1] ? area[$0] > area[$1] : $0 < $1 } : []
        var map = [Int32](repeating: 0, count: Int(next) + 1)
        var count: Int32 = 0
        for id in order where area[id] > 0 { count += 1; map[id] = count }
        for i in 0..<n { labels[i] = labels[i] > 0 ? map[Int(labels[i])] : 0 }
        r.labels = labels
        r.count = Int(count)
        return r
    }

    /// Two-pass 3-4 chamfer distance to the nearest non-free cell (in thirds of a cell).
    static func chamfer(_ free: [Bool], _ w: Int, _ h: Int) -> [Int] {
        let inf = Int.max / 4
        var d = free.map { $0 ? inf : 0 }
        for y in 0..<h {
            for x in 0..<w {
                let i = y * w + x
                guard d[i] != 0 else { continue }
                var v = d[i]
                if x > 0 { v = min(v, d[i - 1] + 3) }
                if y > 0 {
                    v = min(v, d[i - w] + 3)
                    if x > 0 { v = min(v, d[i - w - 1] + 4) }
                    if x < w - 1 { v = min(v, d[i - w + 1] + 4) }
                }
                if x == 0 || y == 0 || x == w - 1 || y == h - 1 { v = min(v, 3) }
                d[i] = v
            }
        }
        for y in stride(from: h - 1, through: 0, by: -1) {
            for x in stride(from: w - 1, through: 0, by: -1) {
                let i = y * w + x
                guard d[i] != 0 else { continue }
                var v = d[i]
                if x < w - 1 { v = min(v, d[i + 1] + 3) }
                if y < h - 1 {
                    v = min(v, d[i + w] + 3)
                    if x < w - 1 { v = min(v, d[i + w + 1] + 4) }
                    if x > 0 { v = min(v, d[i + w - 1] + 4) }
                }
                d[i] = v
            }
        }
        return d
    }
}

// MARK: - CoveragePlanner

/// Boustrophedon lanes (Choset & Pignon, FSR 1997) over a room or zone, cut by walls and
/// restrictions, joined snake-wise from the corner nearest the car.
enum CoveragePlanner {
    static let maxLaneCost: Float = 6, minRun = 0.3, endInset = 0.1

    static func plan(_ grid: OccupancyGrid, _ p: SlamNavigator.Params, spacing: Double, start: SlamPose?,
                     region: (Double, Double) -> Bool) -> [(Double, Double)] {
        let cm = NavCostmap(grid, p)
        let w = cm.w, h = cm.h
        var inside = [Bool](repeating: false, count: w * h)
        var minX = w, minY = h, maxX = -1, maxY = -1
        for y in 0..<h {
            for x in 0..<w {
                let c = cm.cost[y * w + x]
                guard c >= 0, c <= maxLaneCost, region(cm.wx(x), cm.wy(y)) else { continue }
                inside[y * w + x] = true
                minX = min(minX, x); maxX = max(maxX, x); minY = min(minY, y); maxY = max(maxY, y)
            }
        }
        guard maxX >= 0 else { return [] }
        let alongX = (maxX - minX) >= (maxY - minY)
        let step = max(1, Int((spacing / cm.res).rounded()))
        let lo = alongX ? minY : minX, hi = alongX ? maxY : maxX
        let runLo = alongX ? minX : minY, runHi = alongX ? maxX : maxY
        let minRunCells = Int(ceil(minRun / cm.res)), inset = Int((endInset / cm.res).rounded())
        var lanes: [[[Double]]] = []
        var lane = lo + step / 2
        while lane <= hi {
            var pieces: [[Double]] = []
            var s = -1, gap = 0
            for t in runLo...(runHi + 3) {
                let ok = t <= runHi && (alongX ? inside[lane * w + t] : inside[t * w + lane])
                if ok { if s < 0 { s = t }; gap = 0 } else if s >= 0 {
                    gap += 1
                    if gap > 2 {
                        let e = t - gap
                        if e - s + 1 >= minRunCells {
                            let a = min(s + inset, e), b = max(e - inset, a)
                            pieces.append(alongX ? [cm.wx(a), cm.wy(lane), cm.wx(b), cm.wy(lane)]
                                                 : [cm.wx(lane), cm.wy(a), cm.wx(lane), cm.wy(b)])
                        }
                        s = -1; gap = 0
                    }
                }
            }
            if !pieces.isEmpty { lanes.append(pieces) }
            lane += step
        }
        guard !lanes.isEmpty else { return [] }
        var best = Double.greatestFiniteMagnitude
        var bestRoute: [(Double, Double)] = []
        for corner in 0..<4 {
            let route = snake(lanes, reverseLanes: corner & 1 != 0, firstBackwards: corner & 2 != 0)
            let d = start.map { hypot(route[0].0 - $0.x, route[0].1 - $0.y) } ?? 0
            if d < best { best = d; bestRoute = route }
        }
        var out: [(Double, Double)] = []
        for q in bestRoute {
            if let last = out.last, hypot(last.0 - q.0, last.1 - q.1) < 0.15 { continue }
            out.append(q)
        }
        return out
    }

    private static func snake(_ lanes: [[[Double]]], reverseLanes: Bool, firstBackwards: Bool) -> [(Double, Double)] {
        var route: [(Double, Double)] = []
        var backwards = firstBackwards
        for pieces in (reverseLanes ? Array(lanes.reversed()) : lanes) {
            for seg in (backwards ? Array(pieces.reversed()) : pieces) {
                if backwards { route.append((seg[2], seg[3])); route.append((seg[0], seg[1])) } else {
                    route.append((seg[0], seg[1])); route.append((seg[2], seg[3]))
                }
            }
            backwards.toggle()
        }
        return route
    }
}

// MARK: - CleaningMission

/// A Rooms & Zones job on top of SlamNavigator, with Valetudo's status and flag words.
final class CleaningMission {
    enum Status: String { case idle, docked, cleaning, moving, returning, paused, error }
    enum Flag: String { case none, segment, zone, target, resumable }

    static let noPathGrace = 1.0, dockedRadius = 0.35

    let navigator: SlamNavigator
    private(set) var waypoints: [(Double, Double)] = []
    private(set) var index = 0
    private(set) var skipped = 0
    private var current: Status = .idle
    private var before: Status = .idle
    private var beforeFlag: Flag = .none
    private(set) var flag: Flag = .none
    private(set) var label = ""
    private(set) var error = ""
    var home: (Double, Double)?
    private var noPathSince = -1.0

    init(navigator: SlamNavigator) { self.navigator = navigator }

    var isBusy: Bool { current == .cleaning || current == .moving || current == .returning }

    func start(_ wps: [(Double, Double)], flag f: Flag, label: String) {
        guard !wps.isEmpty else { fail("Nothing to drive: the area has no free space the car fits in"); return }
        waypoints = wps; index = 0; skipped = 0; flag = f; self.label = label; error = ""; current = .cleaning
        goToCurrent()
    }

    func goTo(_ x: Double, _ y: Double) {
        waypoints = [(x, y)]; index = 0; skipped = 0; flag = .target; label = ""; error = ""; current = .moving
        goToCurrent()
    }

    func returnHome() {
        guard let h = home else { fail("No home set: choose the home tool and tap the map"); return }
        waypoints = [h]; index = 0; skipped = 0; flag = .none; label = ""; error = ""; current = .returning
        goToCurrent()
    }

    func pause() {
        guard isBusy else { return }
        before = current; beforeFlag = flag; current = .paused; flag = .resumable
        navigator.cancel()
    }

    @discardableResult
    func resume() -> Bool {
        guard current == .paused else { return false }
        current = before; flag = beforeFlag
        goToCurrent()
        return true
    }

    func stop() {
        navigator.cancel()
        waypoints = []; index = 0; current = .idle; flag = .none; label = ""
    }

    private func fail(_ why: String) {
        navigator.cancel()
        waypoints = []; current = .error; flag = .none; error = why
    }

    func clearError() { if current == .error { current = .idle; error = "" } }

    private func goToCurrent() {
        noPathSince = -1
        let p = waypoints[index]
        navigator.setGoal(p.0, p.1)
    }

    /// One ~10 Hz step: the drive command, or nil when nothing should drive.
    func step(pose: SlamPose, grid: OccupancyGrid, frontMin: Double, now: Double) -> SlamNavigator.Command? {
        guard isBusy else { return nil }
        let c = navigator.step(pose: pose, grid: grid, frontMin: frontMin, now: now)
        if navigator.goal != nil {
            if c.state != .noPath { noPathSince = -1; return c }
            if noPathSince < 0 { noPathSince = now }
            if now - noPathSince < CleaningMission.noPathGrace { return c }
            navigator.cancel()
            skipped += 1
        }
        noPathSince = -1
        index += 1
        if index < waypoints.count { goToCurrent(); return c }
        let wasMoving = current == .moving
        let allSkipped = skipped == waypoints.count
        waypoints = []; index = 0; flag = .none; label = ""
        if allSkipped { fail(wasMoving ? "No path to that point on this map" : "No lane of the area could be reached") } else { current = .idle }
        return nil
    }

    func status(at pose: SlamPose?) -> Status {
        if current == .idle, let h = home, let p = pose, hypot(p.x - h.0, p.y - h.1) < CleaningMission.dockedRadius { return .docked }
        return current
    }

    var total: Int { waypoints.count }
    var remaining: [(Double, Double)] { index < waypoints.count ? Array(waypoints[index...]) : [] }
    var target: (Double, Double)? { index < waypoints.count ? waypoints[index] : nil }
}
