"""Receive "LDS1" scan datagrams over UDP and keep the newest revolution.

Python port of ``org.openbot.app.robot.vehicle.LidarScanner`` on Android.

The firmware treats WiFi as an optional side channel that carries scans somewhere
else (ROS 2); the robot itself is driven over the serial link. This module joins
that same stream so the operator can see what the LiDAR sees. Nothing on the
control path waits on a socket, and a host that is not on the robot's WiFi simply
has no scans while the robot drives exactly as before.
"""

import socket
import threading
import time

from lidar.scan import HEADER_LEN, MAX_BINS, LidarScan

#: Default port, matching the firmware's ``LIDAR_UDP_PORT``.
DEFAULT_PORT = 9900

#: How old the newest scan may be before the picture is considered frozen. The
#: firmware sends one datagram per revolution - about 6 Hz at the LD14's 115200
#: baud and about 11 Hz at the LD14P's 230400 - so 500 ms is several missed
#: revolutions rather than a slow link.
STALE_AFTER_MS = 500

#: Age reported before any scan has arrived.
NEVER_FED_AGE_MS = 1000


class LidarScanner:
    """A UDP listener that holds the newest accepted scan."""

    def __init__(self, clock=None):
        self._latest = None
        self._timestamp_ms = 0
        self._running = False
        self._thread = None
        self._sock = None
        self._lock = threading.Lock()
        self._clock = clock if clock is not None else _monotonic_ms

    def start(self, port=DEFAULT_PORT):
        """Start listening.

        Returns False if the port could not be bound - another process may
        already hold it. The caller is free to carry on without a scan view.
        """
        with self._lock:
            if self._running:
                return True
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock.bind(("", port))
                sock.settimeout(0.5)
            except OSError:
                try:
                    sock.close()
                except Exception:
                    pass
                return False
            self._sock = sock
            self._running = True
            # The loop owns this socket. Handing it over (rather than reading
            # self._sock each pass) means a stop() followed by a quick start()
            # cannot leave the old thread reading the new socket alongside the
            # new thread.
            self._thread = threading.Thread(
                target=self._receive_loop, args=(sock,), daemon=True
            )
            self._thread.start()
            return True

    def stop(self):
        """Stop listening and drop the socket. Safe to call twice."""
        with self._lock:
            self._running = False
            sock = self._sock
            self._sock = None
            self._thread = None
            self._latest = None
            self._timestamp_ms = 0
        if sock is not None:
            try:
                sock.close()
            except Exception:
                pass

    def _receive_loop(self, sock):
        buf_size = HEADER_LEN + MAX_BINS * 2
        while self._running and self._sock is sock:
            try:
                data, _addr = sock.recvfrom(buf_size)
            except socket.timeout:
                continue
            except OSError:
                break
            scan = LidarScan.parse(data)
            if scan is not None:
                with self._lock:
                    if self._sock is not sock:
                        break
                    self._latest = scan
                    self._timestamp_ms = self._clock()

    @property
    def latest_scan(self):
        """The newest accepted scan, or None when nothing has ever arrived."""
        with self._lock:
            return self._latest

    def get_age_ms(self):
        """Milliseconds since the last accepted scan, or ``NEVER_FED_AGE_MS`` before it."""
        with self._lock:
            ts = self._timestamp_ms
        if ts == 0:
            return NEVER_FED_AGE_MS
        return self._clock() - ts

    def has_ever_reported(self):
        """True once at least one scan has been accepted."""
        with self._lock:
            return self._timestamp_ms > 0

    @property
    def is_fresh(self):
        """True while scans are still arriving and the picture can be trusted."""
        return self.has_ever_reported() and self.get_age_ms() <= STALE_AFTER_MS

    @property
    def is_stale(self):
        """True when scans stopped after having arrived - a frozen picture, not no sensor."""
        return self.has_ever_reported() and self.get_age_ms() > STALE_AFTER_MS

    @property
    def is_running(self):
        """True while the listener is running, whether or not anything has arrived."""
        with self._lock:
            return self._running

    def get_latest_sectors_mm(self, sector_count, forward_offset_deg=0.0):
        """Newest scan folded into octants, 0 meaning clear.

        Returns None when there is no scan, so a missing reading can never be
        mistaken for a set of clear sectors.
        """
        scan = self.latest_scan
        if scan is None:
            return None
        return scan.compute_sectors_mm(sector_count, forward_offset_deg)


def _monotonic_ms():
    return time.monotonic() * 1000.0
