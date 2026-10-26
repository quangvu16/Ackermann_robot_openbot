"""NMEA GPS source for mission navigation.

Mirrors the Android ``GpsNavLocationProvider`` role on the Linux robot: a
background thread that reads an NMEA receiver over serial and publishes fixes.
Unlike a phone there is no magnetometer here, so heading comes from GPS
course-over-ground while the robot is moving (:class:`HeadingFuser`).

Only the sentences we need are parsed (``GGA`` for position/fix quality and
``RMC`` for speed/course), so there is no third-party NMEA dependency.
"""

import threading

try:
    import serial
except ImportError:  # pragma: no cover - serial is a hard dep of the robot stack
    serial = None

from gpsnav.geo import wrap_degrees

#: Consumer GPS units put horizontal error at roughly this many metres per HDOP.
HDOP_TO_METERS = 4.0
KNOTS_TO_MPS = 0.514444
#: Below this speed course-over-ground is noise, so it is not used as heading.
COURSE_SPEED_THRESHOLD_MPS = 0.5


class GpsFix:
    """One GPS fix: position plus the quality signals guidance needs."""

    def __init__(
        self,
        latitude,
        longitude,
        speed_mps=0.0,
        course_degrees=None,
        accuracy_meters=None,
        timestamp=None,
    ):
        self.latitude = latitude
        self.longitude = longitude
        self.speed_mps = speed_mps
        #: Course-over-ground in degrees clockwise from true north, if known.
        self.course_degrees = course_degrees
        self.accuracy_meters = accuracy_meters
        self.timestamp = timestamp

    @property
    def has_position(self):
        return self.latitude is not None and self.longitude is not None

    @property
    def has_course(self):
        """True when the course is trustworthy enough to steer by."""
        return (
            self.course_degrees is not None
            and self.speed_mps >= COURSE_SPEED_THRESHOLD_MPS
        )

    def __repr__(self):
        return "GpsFix(lat=%.7f lon=%.7f spd=%.2f crs=%s acc=%s)" % (
            self.latitude,
            self.longitude,
            self.speed_mps,
            self.course_degrees,
            self.accuracy_meters,
        )


class HeadingFuser:
    """Turns raw GPS course-over-ground into a usable robot heading.

    A ground vehicle without a compass can only know which way it points while
    it is moving. Below the speed threshold the last known course is held; until
    the first moving fix there is no heading at all and the navigator should
    hold still or creep forward to establish one.
    """

    def __init__(self, speed_threshold_mps=COURSE_SPEED_THRESHOLD_MPS):
        self.speed_threshold_mps = speed_threshold_mps
        self._heading_degrees = None

    @property
    def is_ready(self):
        return self._heading_degrees is not None

    @property
    def heading_degrees(self):
        return self._heading_degrees

    def update(self, course_degrees, speed_mps=0.0):
        """Fuses one course sample; returns the current heading (possibly None)."""
        if course_degrees is None:
            return self._heading_degrees
        if speed_mps >= self.speed_threshold_mps:
            self._heading_degrees = wrap_degrees(course_degrees)
        return self._heading_degrees

    def update_from_fix(self, fix):
        if fix is None:
            return self._heading_degrees
        return self.update(fix.course_degrees, fix.speed_mps)

    def reset(self):
        self._heading_degrees = None


# ---------------------------------------------------------------------------
# NMEA
# ---------------------------------------------------------------------------


def _checksum_ok(body, checksum):
    if len(checksum) < 2:
        return False
    try:
        expected = int(checksum[:2], 16)
    except ValueError:
        return False
    actual = 0
    for char in body:
        actual ^= ord(char)
    return actual == expected


def _to_float(value):
    if value is None or value == "":
        return None
    try:
        return float(value)
    except ValueError:
        return None


def _parse_latlon(value, hemisphere):
    """Converts an NMEA ``ddmm.mmmm`` / ``dddmm.mmmm`` field to signed degrees."""
    if not value or not hemisphere:
        return None
    raw = _to_float(value)
    if raw is None:
        return None
    degrees = int(raw // 100)
    minutes = raw - degrees * 100
    decimal = degrees + minutes / 60.0
    if hemisphere in ("S", "W"):
        decimal = -decimal
    return decimal


def parse_nmea_line(line):
    """Parses one NMEA sentence into a partial fix update.

    Args:
        line: a single NMEA sentence, with or without trailing newline

    Returns:
        dict: fields present in that sentence (``latitude``, ``longitude``,
        ``speed_mps``, ``course_degrees``, ``accuracy_meters``), or ``None``
        when the sentence is unusable or not one we understand. A fix-quality
        sentence that says "no fix" returns ``{"valid": False}`` so the caller
        can drop a stale position.
    """
    if not line:
        return None
    line = line.strip()
    if not line.startswith("$") or "*" not in line:
        return None

    body, _, checksum = line[1:].partition("*")
    if not _checksum_ok(body, checksum):
        return None

    fields = body.split(",")
    sentence = fields[0].upper()
    # Accept any talker (GP/GN/GL...) as long as it is a GGA or RMC.
    if sentence.endswith("GGA"):
        return _parse_gga(fields)
    if sentence.endswith("RMC"):
        return _parse_rmc(fields)
    return None


def _parse_gga(fields):
    # fields[0] is the sentence id: $--GGA,time,lat,N,lon,E,fix,sats,hdop,...
    if len(fields) < 9:
        return None
    if fields[6] in ("", "0"):
        return {"valid": False}

    lat = _parse_latlon(fields[2], fields[3])
    lon = _parse_latlon(fields[4], fields[5])
    if lat is None or lon is None:
        return {"valid": False}

    update = {"latitude": lat, "longitude": lon, "valid": True}
    hdop = _to_float(fields[8])
    if hdop is not None:
        update["accuracy_meters"] = hdop * HDOP_TO_METERS
    return update


def _parse_rmc(fields):
    # fields[0] is the sentence id: $--RMC,time,status,lat,N,lon,E,speed,course,...
    if len(fields) < 9:
        return None
    if fields[2] != "A":
        return {"valid": False}

    lat = _parse_latlon(fields[3], fields[4])
    lon = _parse_latlon(fields[5], fields[6])
    if lat is None or lon is None:
        return {"valid": False}

    update = {"latitude": lat, "longitude": lon, "valid": True}
    speed_kn = _to_float(fields[7])
    if speed_kn is not None:
        update["speed_mps"] = speed_kn * KNOTS_TO_MPS
    course = _to_float(fields[8])
    if course is not None:
        update["course_degrees"] = wrap_degrees(course)
    return update


# ---------------------------------------------------------------------------
# Hardware thread
# ---------------------------------------------------------------------------


def get_gps_serial(device_address="/dev/ttyUSB1", baudrate=9600):
    """Opens the NMEA serial port, or returns ``None`` if it is unavailable.

    Mirrors ``infer.get_arduino_serial``: a missing receiver disables GPS
    rather than raising, so the robot can still run in other modes.
    """
    if serial is None:
        print("pyserial not installed. Not starting GPS.")
        return None
    try:
        return serial.Serial(device_address, baudrate=baudrate, timeout=1.0)
    except Exception as e:
        print(e)
        print(f"GPS not found at {device_address} with baudrate {baudrate}")
        return None


class Gps(threading.Thread):
    """Reads NMEA sentences and publishes :class:`GpsFix` updates.

    Mirrors ``realsense.Realsense``: a daemon thread that only starts when the
    device is present, and a single callback that receives the latest fix.
    """

    def __init__(self, callback, device_address="/dev/ttyUSB1", baudrate=9600):
        """Args:
        callback (function): called with each new :class:`GpsFix`
        device_address (str): serial device of the NMEA receiver
        baudrate (int): NMEA baudrate, typically 9600 or 38400
        """
        self.callback = callback
        self.device_address = device_address
        self.baudrate = baudrate
        self.latest = None
        self._pending = {}
        self._lock = threading.Lock()

        self.gps_serial = get_gps_serial(device_address, baudrate)

        super().__init__(name="gps-thread")
        self.daemon = True

        if self.gps_serial:
            self.start()
        else:
            print(f"GPS not found at {device_address}. Not starting GPS Thread.")

    @property
    def is_running(self):
        return self.gps_serial is not None

    def run(self):
        """Reads lines forever, merging partial NMEA updates into a fix."""
        while True:
            try:
                raw = self.gps_serial.readline()
            except Exception as exc:
                print(f"GPS read error: {exc}")
                break
            if not raw:
                continue
            self.feed(raw.decode("ascii", errors="ignore"))

    def feed(self, line):
        """Feeds one NMEA line; also used by tests to drive the parser."""
        update = parse_nmea_line(line)
        if update is None:
            return None
        return self.apply_update(update)

    def apply_update(self, update):
        """Merges a partial NMEA update and emits a fix when a position is known."""
        with self._lock:
            if not update.get("valid", False):
                # No fix: drop the stale position so guidance does not drive on it.
                self._pending.clear()
                self.latest = None
                return None

            self._pending.update({k: v for k, v in update.items() if k != "valid"})
            if "latitude" not in self._pending or "longitude" not in self._pending:
                return None

            fix = GpsFix(
                latitude=self._pending["latitude"],
                longitude=self._pending["longitude"],
                speed_mps=self._pending.get("speed_mps", 0.0),
                course_degrees=self._pending.get("course_degrees"),
                accuracy_meters=self._pending.get("accuracy_meters"),
            )
            self.latest = fix

        self.callback(fix)
        return fix
