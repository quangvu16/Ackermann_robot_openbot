"""Render a LiDAR revolution as a plan view, the way a ground station draws it.

Python counterpart of ``org.openbot.app.robot.customview.LidarView`` on Android.

The robot sits at the centre pointing up, so "forward" is up on screen whichever
way the host is oriented. Returns are drawn as a connected outline at their
measured distance - the room's walls read as the shape they actually are. Bins
with no return are *gaps* rather than zeros, which is what "nothing measured
here" means; the ``+inf`` a ROS ``/scan`` publisher would emit has no place on
screen.

The mounting offset is applied here at draw time (``forward_offset_deg``) rather
than by reordering bins, matching the firmware's ``LIDAR_FORWARD_DEG`` rule.

The geometry is pure (:func:`scan_to_points`) so it can be tested without a
display; :func:`draw_plan_view` is the thin OpenCV wrapper on top of it.
"""

import math

from lidar.scan import wrap_deg


def scan_to_points(scan, range_max_meters=8.0, forward_offset_deg=0.0):
    """Project a scan into image-space points.

    Returns a list of ``(x, y, bin_index)`` in a square image of side ``size``
    centred on the robot, with the nose pointing up (negative y). Bins with no
    return are omitted, so an unmeasured arc is a gap rather than a line dragged
    through the middle of the plot. Returns further than ``range_max_meters`` are
    clamped to the outer circle and still show their direction.

    Coordinates are normalised to ``[0, 1]`` on both axes so the caller can scale
    to any pixel size; the origin is the centre and +x is right, +y is down.
    """
    if scan is None or range_max_meters <= 0:
        return []
    out = []
    for bin_index in range(scan.bin_count):
        mm = scan.get_range_mm(bin_index)
        if mm == 0:
            continue
        dist = mm / 1000.0
        r = 0.5 * min(1.0, dist / range_max_meters)
        # Vehicle angle is clockwise from the nose; screen y grows downward, so
        # subtracting 90 maps nose-up to -y and keeps clockwise rotation correct.
        ang = math.radians(scan.get_vehicle_angle_deg(bin_index, forward_offset_deg) - 90.0)
        x = r * math.cos(ang)
        y = r * math.sin(ang)
        out.append((x, y, bin_index))
    return out


def split_strokes(points):
    """Group consecutive points into polylines.

    ``points`` must already be in bin order. A gap of more than one bin index
    means the scanner saw nothing there, so the pen lifts - that is how an
    unmeasured arc becomes a visible hole rather than a straight line across it.

    Returns a list of lists of ``(x, y)``.
    """
    strokes = []
    current = []
    prev_bin = None
    for x, y, bin_index in points:
        if prev_bin is not None and bin_index != prev_bin + 1:
            if current:
                strokes.append(current)
            current = []
        current.append((x, y))
        prev_bin = bin_index
    if current:
        strokes.append(current)
    return strokes


def close_seam(strokes, points, bin_count):
    """Join the outline across the 359 -> 0 seam when both ends returned.

    Bins are in a ring, so a wall that crosses the LiDAR's zero mark is one
    line, not two. :func:`split_strokes` works on a list and would otherwise
    leave a gap at the seam.
    """
    if len(strokes) < 2 or not points or bin_count <= 0:
        return strokes
    if points[0][2] != 0 or points[-1][2] != bin_count - 1:
        return strokes
    merged = strokes[-1] + strokes[0]
    return [merged] + strokes[1:-1]


def draw_plan_view(
    scan,
    size=280,
    range_max_meters=8.0,
    forward_offset_deg=0.0,
    margin_meters=1.0,
    show_margin=True,
    stale=False,
    background=(16, 24, 32),
    scan_color=(214, 198, 38),
    stale_color=(158, 158, 158),
    ring_color=(176, 190, 197),
    margin_color=(0, 179, 255),
    robot_color=(255, 123, 75),
):
    """Render a plan view into a BGR ``numpy`` array, OpenCV style.

    Returns ``(image, strokes)`` where ``strokes`` is what :func:`split_strokes`
    produced, so a caller that wants to draw with something other than OpenCV can
    still reuse the geometry.
    """
    import numpy as np

    image = np.zeros((size, size, 3), dtype=np.uint8)
    image[:, :] = background

    cx = cy = size // 2.0
    radius = size * 0.46

    _try_import_cv2()
    import cv2

    for frac in (0.25, 0.5, 1.0):
        cv2.circle(image, (int(cx), int(cy)), int(radius * frac), ring_color, 1)

    if show_margin and margin_meters > 0 and margin_meters < range_max_meters:
        r = int(radius * (margin_meters / range_max_meters))
        cv2.circle(image, (int(cx), int(cy)), r, margin_color, 2)

    # Nose and tail marks so the plot reads as a robot rather than a compass.
    cv2.line(
        image,
        (int(cx), int(cy - radius)),
        (int(cx), int(cy - radius * 0.88)),
        robot_color,
        2,
    )
    cv2.line(
        image,
        (int(cx), int(cy + radius)),
        (int(cx), int(cy + radius * 0.88)),
        ring_color,
        1,
    )

    points = scan_to_points(scan, range_max_meters, forward_offset_deg)
    strokes = split_strokes(points)
    if scan is not None:
        strokes = close_seam(strokes, points, scan.bin_count)
    colour = stale_color if stale else scan_color
    for stroke in strokes:
        if len(stroke) < 2:
            if stroke:
                x, y = stroke[0]
                cv2.circle(image, _to_px(cx, cy, radius, x, y), 2, colour, -1)
            continue
        px = [_to_px(cx, cy, radius, x, y) for x, y in stroke]
        for a, b in zip(px, px[1:]):
            cv2.line(image, a, b, colour, 2)

    _draw_robot(image, cx, cy, radius, robot_color)
    return image, strokes


def _to_px(cx, cy, radius, x, y):
    return int(cx + x * 2 * radius), int(cy + y * 2 * radius)


def _draw_robot(image, cx, cy, radius, color):
    import cv2

    import numpy as np

    size = radius * 0.12
    # fillPoly wants an int32 array of shape (N, 2); a list of tuples is rejected.
    pts = np.array(
        [
            (int(cx), int(cy - size)),
            (int(cx + size * 0.7), int(cy + size * 0.8)),
            (int(cx - size * 0.7), int(cy + size * 0.8)),
        ],
        dtype=np.int32,
    )
    cv2.fillPoly(image, [pts], color)


def _try_import_cv2():
    try:
        import cv2  # noqa: F401
    except ImportError as exc:  # pragma: no cover - depends on the host
        raise ImportError(
            "draw_plan_view needs opencv-python; use scan_to_points and "
            "split_strokes directly to draw with something else"
        ) from exc


def format_status(scan, fresh):
    """One-line status for the UI, matching the Android app's wording."""
    if scan is None:
        return "Waiting for scans"
    state = "live" if fresh else "stale"
    return f"{scan.bin_count} bins · {scan.valid_points} returns · {state}"


def main(argv=None):
    """Live plan view in an OpenCV window; run ``python lidar_view.py``.

    Listens on the "LDS1" UDP port and redraws about ten times a second. Press
    ``q`` or ``Esc`` to quit. Needs a display; on a headless robot pass
    ``--snapshot out.png`` to write one frame instead.
    """
    import argparse
    import time

    from lidar.scanner import DEFAULT_PORT, LidarScanner

    parser = argparse.ArgumentParser(description="Live LiDAR plan view.")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--range", type=float, default=8.0, help="Outer ring in metres.")
    parser.add_argument("--forward_deg", type=float, default=0.0,
                        help="LiDAR mounting offset, the firmware's LIDAR_FORWARD_DEG.")
    parser.add_argument("--margin", type=float, default=1.0,
                        help="Avoidance margin ring in metres (0 hides it).")
    parser.add_argument("--size", type=int, default=480)
    parser.add_argument("--snapshot", default=None,
                        help="Write one frame to this image file after --wait seconds and exit.")
    parser.add_argument("--wait", type=float, default=2.0)
    args = parser.parse_args(argv)

    _try_import_cv2()
    import cv2

    scanner = LidarScanner()
    if not scanner.start(port=args.port):
        print(f"Could not bind UDP port {args.port}")
        return 1
    try:
        if args.snapshot:
            time.sleep(args.wait)
            image, _ = draw_plan_view(scanner.latest_scan, args.size, args.range,
                                      args.forward_deg, args.margin, args.margin > 0,
                                      scanner.is_stale)
            cv2.imwrite(args.snapshot, image)
            print(format_status(scanner.latest_scan, scanner.is_fresh))
            return 0
        while True:
            image, _ = draw_plan_view(scanner.latest_scan, args.size, args.range,
                                      args.forward_deg, args.margin, args.margin > 0,
                                      scanner.is_stale)
            cv2.putText(image, format_status(scanner.latest_scan, scanner.is_fresh)
                        .replace("\u00b7", "-"), (8, args.size - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)
            cv2.imshow("OpenBot LiDAR", image)
            key = cv2.waitKey(100) & 0xFF
            if key in (ord("q"), 27):
                return 0
    except KeyboardInterrupt:
        return 0
    finally:
        scanner.stop()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
