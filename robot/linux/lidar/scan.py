"""Decode one "LDS1" LiDAR revolution.

Python port of ``org.openbot.app.robot.vehicle.LidarScan`` on Android. The wire
format is the firmware's (see ``firmware/openbot-esp32s3/lidar_udp.h``)::

    offset  size          field
         0     4          "LDS1"
         4     2          bin count           uint16 little-endian
         6     4          scan id             uint32 little-endian
        10     4          reserved, zero
        14  2*bins        ranges, millimetres uint16 little-endian each

``0 mm`` means "no return" in that bin - the same convention the firmware uses
on the ``p`` proximity channel. A real return at 0 mm is outside the LD14's
20 mm minimum range, so the two can never be confused.

Bin ``i`` is centred on ``(i + 0.5) * (360 / bins)`` degrees clockwise from the
LiDAR's own zero mark. Bins are never reordered: the mounting offset is applied at read time
with ``forward_offset_deg``, exactly as the firmware applies
``LIDAR_FORWARD_DEG`` when folding octants. Reordering the bins *and* applying
the offset would make the two corrections fight.
"""

import struct

#: The four bytes every datagram starts with.
MAGIC = b"LDS1"

HEADER_LEN = 14

#: Largest bin count accepted, matching the firmware's ``UDP_BINS_MAX``. Anything
#: claiming more is not a scan this firmware sends.
MAX_BINS = 720


class LidarScan:
    """One finished revolution. Ranges are in millimetres, 0 meaning "no return"."""

    def __init__(self, bin_count, scan_id, ranges_mm):
        self._bin_count = bin_count
        self._scan_id = scan_id
        self._ranges_mm = list(ranges_mm)

    @property
    def bin_count(self):
        """Number of bins in this revolution."""
        return self._bin_count

    @property
    def scan_id(self):
        """Firmware-side scan counter, incremented once per sent revolution."""
        return self._scan_id

    @property
    def ranges_mm(self):
        """All ranges in bin order, never reordered. Millimetres, 0 for "no return"."""
        return list(self._ranges_mm)

    def get_range_mm(self, bin_index):
        """Range of one bin in millimetres, or 0 for "no return"."""
        if bin_index < 0 or bin_index >= self._bin_count:
            return 0
        return self._ranges_mm[bin_index]

    def get_range_meters(self, bin_index):
        """Range in metres, or ``math.inf`` when there is no return.

        ``0`` mm on the wire means "nothing measured here", not "an obstacle at
        zero" - the same distinction :mod:`lidar.proximity` draws on the ``p``
        channel.
        """
        mm = self.get_range_mm(bin_index)
        return float("inf") if mm == 0 else mm / 1000.0

    def has_return(self, bin_index):
        """True when this bin returned a measurement."""
        return self.get_range_mm(bin_index) > 0

    @property
    def valid_points(self):
        """Number of bins that returned a measurement."""
        return sum(1 for mm in self._ranges_mm if mm > 0)

    def get_bin_angle_deg(self, bin_index):
        """Angle of a bin, clockwise from the LiDAR's zero mark, in ``[0, 360)``.

        Uses the bin centre (``i + 0.5``) so a 360-bin scan reports bin 0 at
        0.5 degrees rather than snapping it onto the zero mark - the same centre
        convention the firmware uses when folding octants.
        """
        return ((bin_index + 0.5) * 360.0) / float(self._bin_count)

    def get_vehicle_angle_deg(self, bin_index, forward_offset_deg=0.0):
        """Angle of a bin relative to the car's nose, in ``[0, 360)``."""
        return wrap_deg(self.get_bin_angle_deg(bin_index) - forward_offset_deg)

    def compute_sectors_mm(self, sector_count, forward_offset_deg=0.0):
        """Fold this revolution into ``sector_count`` octants, keeping the nearest return.

        Sector ``i`` is centred on ``i * (360 / sector_count)`` degrees from the
        nose and spans half a sector either side. This mirrors the firmware's
        ``computeSectors()`` exactly, so an octant folded here and one reported
        on the ``p`` channel describe the same geometry.

        Returns a list of millimetres, 0 where the sector is clear.
        """
        count = max(1, int(sector_count))
        out = [0] * count
        step = 360.0 / count
        for bin_index in range(self._bin_count):
            mm = self._ranges_mm[bin_index]
            if mm == 0:
                continue
            angle = self.get_vehicle_angle_deg(bin_index, forward_offset_deg)
            sector = int((angle + step * 0.5) / step) % count
            if out[sector] == 0 or mm < out[sector]:
                out[sector] = mm
        return out

    @staticmethod
    def parse(datagram, length=None):
        """Decode one datagram.

        Returns ``None`` for anything malformed - wrong magic, truncated, a bin
        count of zero, or a bin count past :data:`MAX_BINS` - so a stray packet
        on the port can never be mistaken for a scan of open road.

        ``length`` bounds the payload when the buffer is oversized (a UDP
        receive hands one back); reading past it would pick up stale bytes from
        the previous datagram as ranges.
        """
        if datagram is None:
            return None
        if length is None:
            length = len(datagram)
        if length < HEADER_LEN:
            return None
        if bytes(datagram[0:4]) != MAGIC:
            return None
        (bin_count,) = struct.unpack_from("<H", datagram, 4)
        if bin_count <= 0 or bin_count > MAX_BINS:
            return None
        if length < HEADER_LEN + bin_count * 2:
            return None
        (scan_id,) = struct.unpack_from("<I", datagram, 6)
        ranges = list(struct.unpack_from(f"<{bin_count}H", datagram, HEADER_LEN))
        return LidarScan(bin_count, scan_id, ranges)

    @staticmethod
    def encode(scan_id, ranges_mm):
        """Encode a datagram back out. Used by the tests to round-trip the parser."""
        if ranges_mm is None:
            return None
        count = len(ranges_mm)
        out = MAGIC + struct.pack("<HI", count, scan_id & 0xFFFFFFFF) + b"\x00\x00\x00\x00"
        out += struct.pack(f"<{count}H", *[int(mm) & 0xFFFF for mm in ranges_mm])
        return out


def wrap_deg(deg):
    """Degrees, wrapped to ``[0, 360)``."""
    return deg % 360.0
