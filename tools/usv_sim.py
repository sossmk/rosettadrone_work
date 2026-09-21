#!/usr/bin/env python3
"""가짜 USV — /usv/fix · /usv/heading · /usv/depth 를 5Hz 로 낸다. 지상 시험 전용.

    python3 tools/usv_sim.py 37.5 126.9                     # 그 자리에 세워 둔다
    python3 tools/usv_sim.py 37.5 126.9 --hdg 90 --speed 1  # 동쪽으로 1 m/s
    python3 tools/usv_sim.py --selftest                     # ROS 없이 슬루 계산만

키로 민다 — w·s 위도 ±0.00002 (2.2m) · a·d 경도 ±0.00002 · r 처음 자리 · q 종료.
한 칸을 순간이동하지 않고 2 m/s 로 미끄러진다. 점프시키면 usv_follow_node 의 속도 추정이
vel_window(2초) 안에서 가짜 속도를 읽어 feedforward 가 2초간 튄다 — 이동 추종이 되는지
보려는 관찰을 그게 가린다.

**사람이 USV 자리에 서는 시험**이면 자기 위경도를 폰으로 재서 인자로 넣는다 (실험가이드.md 10).
기수는 안 움직이니 기본 0(북) 그대로 두면 대형 좌·우·후가 서·동·남으로 고정된다.
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mav  # noqa: E402

STEP = 0.00002      # 한 칸 — 위도로 2.22m
SLEW = 2.0          # m/s — 새 좌표까지 미끄러지는 속도


def slew(cur, base, dt):
    """cur 을 base 쪽으로 SLEW×dt 만큼 당긴다. 넘어가지 않는다."""
    d = mav.meters_between(cur[0], cur[1], base[0], base[1])
    if d < 0.01:
        return base
    b = mav.bearing_between(cur[0], cur[1], base[0], base[1])
    return mav.offset_to_gps(cur[0], cur[1], b, min(d, SLEW * dt), 0.0)


def selftest():
    assert abs(mav.meters_between(37.0, 127.0, 37.0 + STEP, 127.0) - 2.22) < 0.01
    # 5Hz 로 밀면 한 칸(2.22m)은 1.2초쯤에 닿고, 닿은 뒤에는 그 자리에 머문다
    cur, base = (37.0, 127.0), (37.0 + STEP, 127.0)
    for _ in range(20):
        cur = slew(cur, base, 0.2)
    assert cur == base, cur
    # 한 걸음은 SLEW×dt 를 넘지 않는다 (멀리 있어도)
    one = slew((37.0, 127.0), (37.01, 127.0), 0.2)
    assert abs(mav.meters_between(37.0, 127.0, *one) - 0.4) < 0.01
    print("selftest ok")


if __name__ == "__main__" and "--selftest" in sys.argv:
    sys.exit(selftest())

import rclpy  # noqa: E402
from sensor_msgs.msg import NavSatFix  # noqa: E402
from std_msgs.msg import Float64  # noqa: E402

from rd_cmd import key_loop  # noqa: E402  — 키 입력 + 한 줄 다시 그리기. tools/ 에 이미 있다


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("lat", type=float)
    ap.add_argument("lon", type=float)
    ap.add_argument("--hdg", type=float, default=0.0, help="° (0=북, 시계)")
    ap.add_argument("--speed", type=float, default=0.0, help="m/s, 기수 방향 — 키와 같이 쓴다")
    ap.add_argument("--depth", type=float, default=5.0)
    ap.add_argument("--selftest", action="store_true", help="ROS 없이 슬루 계산만")
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("usv_sim")
    # RELIABLE(10) 그대로 — rd_cmd.py 가 /usv/fix 를 RELIABLE 로 구독한다
    pub_fix = node.create_publisher(NavSatFix, "usv/fix", 10)
    pub_hdg = node.create_publisher(Float64, "usv/heading", 10)
    pub_dep = node.create_publisher(Float64, "usv/depth", 10)

    home = (args.lat, args.lon)
    st = {"base": home, "cur": home, "t": time.time(), "t0": time.time()}

    def on_key(k):
        d = {"w": (STEP, 0.0), "s": (-STEP, 0.0), "a": (0.0, -STEP), "d": (0.0, STEP)}.get(k)
        if d:
            st["base"] = (st["base"][0] + d[0], st["base"][1] + d[1])
        elif k == "r":
            st["base"] = home

    def tick():
        now = time.time()
        st["cur"] = slew(st["cur"], st["base"], now - st["t"])
        st["t"] = now
        lat, lon = mav.offset_to_gps(*st["cur"], args.hdg, args.speed * (now - st["t0"]), 0.0)
        pub_fix.publish(NavSatFix(latitude=lat, longitude=lon))
        pub_hdg.publish(Float64(data=args.hdg))
        pub_dep.publish(Float64(data=args.depth))
        left = mav.meters_between(*st["cur"], *st["base"])
        return (f"{lat:.7f},{lon:.7f} · 기수 {args.hdg:.0f}° · 처음 자리에서 "
                f"{mav.meters_between(*home, lat, lon):5.1f}m"
                + (f" · 미끄러지는 중 {left:4.1f}m" if left > 0.05 else ""))

    print(f"\n{args.lat:.7f}, {args.lon:.7f} — /usv/fix · heading · depth 를 5Hz 로")
    print(f"  w·s 위도 ±{STEP} ({STEP * 111195:.1f}m) · a·d 경도 ±{STEP} · r 처음 자리 · q 종료\n")
    try:
        key_loop(on_key, tick, hz=5)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
