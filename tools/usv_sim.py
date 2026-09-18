#!/usr/bin/env python3
"""가짜 USV — /usv/fix · /usv/heading · /usv/depth 를 5Hz 로 낸다. 지상 dry-run 전용.

    python3 tools/usv_sim.py 37.5 126.9                     # 제자리
    python3 tools/usv_sim.py 37.5 126.9 --hdg 90 --speed 1  # 동쪽으로 1 m/s

시작 좌표는 보통 우리 /rd/fix 근처로 준다 (ros2 topic echo /rd/fix --once).
"""
import argparse
import os
import sys
import time

import rclpy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mav  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("lat", type=float)
    ap.add_argument("lon", type=float)
    ap.add_argument("--hdg", type=float, default=0.0, help="° (0=북, 시계)")
    ap.add_argument("--speed", type=float, default=0.0, help="m/s, 기수 방향")
    ap.add_argument("--depth", type=float, default=5.0)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("usv_sim")
    pub_fix = node.create_publisher(NavSatFix, "usv/fix", 10)
    pub_hdg = node.create_publisher(Float64, "usv/heading", 10)
    pub_dep = node.create_publisher(Float64, "usv/depth", 10)
    t0 = time.time()
    try:
        while rclpy.ok():
            lat, lon = mav.offset_to_gps(args.lat, args.lon, args.hdg,
                                         args.speed * (time.time() - t0), 0.0)
            pub_fix.publish(NavSatFix(latitude=lat, longitude=lon))
            pub_hdg.publish(Float64(data=args.hdg))
            pub_dep.publish(Float64(data=args.depth))
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
