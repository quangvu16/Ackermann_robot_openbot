"""Live LiDAR plan view for the OpenBot Linux robot.

Opens an OpenCV window with one revolution drawn as a polar plot, robot at the
centre pointing up, refreshed about ten times a second from the "LDS1" UDP
stream (port 9900 by default). Press ``q`` or ``Esc`` to quit.

    python lidar_view.py
    python lidar_view.py --forward_deg 180 --margin 0.8
    python lidar_view.py --snapshot scan.png   # headless: write one frame
"""

from lidar.plan_view import main

if __name__ == "__main__":
    raise SystemExit(main())
