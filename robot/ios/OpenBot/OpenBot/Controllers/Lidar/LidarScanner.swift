//
//  LidarScanner.swift
//  OpenBot
//
//  Receives "LDS1" scan datagrams over UDP and keeps the newest revolution.
//
//  Mirrors Android's `vehicle.LidarScanner` and the Linux `lidar.scanner`
//  module. The firmware treats WiFi as an optional side channel that carries
//  scans somewhere else (ROS 2); the robot itself is driven over BLE. This
//  joins that same stream so the operator can see what the LiDAR sees. Nothing
//  on the control path waits on a socket, and a device that is not on the
//  robot's WiFi simply has no scans while the robot drives exactly as before.
//
//  BSD sockets rather than Network.framework: "bind a UDP port and recvfrom
//  anyone" is the exact shape this needs, and a short receive timeout keeps the
//  thread from hanging on a quiet network.
//

import Darwin
import Foundation

/// Default port, matching the firmware's `LIDAR_UDP_PORT`.
let lidarUdpPort: UInt16 = 9900

/// How old the newest scan may be before the picture is considered frozen. The
/// firmware sends one datagram per revolution - about 6 Hz at the LD14's 115200
/// baud and about 11 Hz at the LD14P's 230400 - so 500 ms is several missed
/// revolutions rather than a slow link.
let lidarStaleAfterMs = 500.0

/// Age reported before any scan has arrived.
let lidarNeverFedAgeMs = 1000.0

/// A UDP listener that holds the newest accepted scan.
///
/// `onScan` fires on the scanner queue whenever a revolution is accepted; hop
/// to the main queue before touching UIKit.
final class LidarScanner {
    private let queue = DispatchQueue(label: "org.openbot.lidar.scanner")
    private let stateLock = NSLock()

    private var socketFD: Int32 = -1
    private var running = false
    private var latest: LidarScan?
    private var timestampMs: Double = 0
    private let clock: () -> Double

    /// Called on the scanner queue with each accepted revolution.
    var onScan: ((LidarScan) -> Void)?

    init(clock: @escaping () -> Double = monotonicMs) {
        self.clock = clock
    }

    /// Start listening.
    ///
    /// Returns false if the port could not be bound - another process may
    /// already hold it. The caller is free to carry on without a scan view.
    @discardableResult
    func start(port: UInt16 = lidarUdpPort) -> Bool {
        stateLock.lock()
        if running {
            stateLock.unlock()
            return true
        }
        let fd = socket(AF_INET, SOCK_DGRAM, 0)
        guard fd >= 0 else {
            stateLock.unlock()
            return false
        }
        var reuse: Int32 = 1
        setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, socklen_t(MemoryLayout<Int32>.size))
        var timeout = timeval(tv_sec: 0, tv_usec: 500_000) // 0.5 s, like the Linux port
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, socklen_t(MemoryLayout<timeval>.size))

        var addr = sockaddr_in()
        addr.sin_family = sa_family_t(AF_INET)
        addr.sin_port = port.bigEndian
        addr.sin_addr.s_addr = INADDR_ANY
        let bound = withUnsafePointer(to: &addr) { pointer in
            pointer.withMemoryRebound(to: sockaddr.self, capacity: 1) {
                bind(fd, $0, socklen_t(MemoryLayout<sockaddr_in>.size))
            }
        }
        guard bound == 0 else {
            close(fd)
            stateLock.unlock()
            return false
        }
        socketFD = fd
        running = true
        stateLock.unlock()

        queue.async { [weak self] in
            self?.receiveLoop(fd)
        }
        return true
    }

    /// Stop listening and drop the socket. Safe to call twice.
    func stop() {
        stateLock.lock()
        running = false
        let fd = socketFD
        socketFD = -1
        latest = nil
        timestampMs = 0
        stateLock.unlock()
        if fd >= 0 {
            close(fd)
        }
    }

    /// The newest accepted scan, or nil when nothing has ever arrived.
    var latestScan: LidarScan? {
        stateLock.lock()
        defer { stateLock.unlock() }
        return latest
    }

    /// Milliseconds since the last accepted scan, or `neverFedAgeMs` before it.
    func ageMs() -> Double {
        stateLock.lock()
        let ts = timestampMs
        stateLock.unlock()
        if ts == 0 { return lidarNeverFedAgeMs }
        return clock() - ts
    }

    /// True once at least one scan has been accepted.
    func hasEverReported() -> Bool {
        stateLock.lock()
        defer { stateLock.unlock() }
        return timestampMs > 0
    }

    /// True while scans are still arriving and the picture can be trusted.
    var isFresh: Bool {
        hasEverReported() && ageMs() <= lidarStaleAfterMs
    }

    /// True when scans stopped after having arrived - a frozen picture, not no sensor.
    var isStale: Bool {
        hasEverReported() && ageMs() > lidarStaleAfterMs
    }

    /// True while the listener is running, whether or not anything has arrived.
    var isRunning: Bool {
        stateLock.lock()
        defer { stateLock.unlock() }
        return running
    }

    /// Newest scan folded into octants, 0 meaning clear.
    ///
    /// Returns nil when there is no scan, so a missing reading can never be
    /// mistaken for a set of clear sectors.
    func latestSectorsMm(_ sectorCount: Int, forwardOffsetDeg: Double = 0) -> [Int]? {
        guard let scan = latestScan else { return nil }
        return scan.computeSectorsMm(sectorCount, forwardOffsetDeg: forwardOffsetDeg)
    }

    private func receiveLoop(_ fd: Int32) {
        let bufferSize = lidarScanHeaderLength + lidarScanMaxBins * 2
        var buffer = [UInt8](repeating: 0, count: bufferSize)
        while true {
            stateLock.lock()
            let stillRunning = running && socketFD == fd
            stateLock.unlock()
            guard stillRunning else { break }

            let received = buffer.withUnsafeMutableBufferPointer { pointer -> Int in
                recv(fd, pointer.baseAddress, bufferSize, 0)
            }
            guard received > 0 else {
                // Timeout or an error. A closed socket shows up as an error;
                // the loop's guard above is what ends the thread on stop().
                continue
            }
            let datagram = Data(buffer.prefix(received))
            // parse() takes the datagram's real length; an oversized buffer must
            // not turn stale bytes into ranges.
            guard let scan = LidarScan.parse(datagram, length: received) else { continue }
            let now = clock()
            stateLock.lock()
            latest = scan
            timestampMs = now
            stateLock.unlock()
            onScan?(scan)
        }
    }
}
