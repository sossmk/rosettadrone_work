#!/usr/bin/env python3
"""토픽 수신 점검 — 브리지 토픽이 기대 주기와 값으로 차는지 본다.

    source /opt/ros/jazzy/setup.bash          # colcon 빌드는 필요 없다
    python3 tools/topic_check.py              # 10초 듣고 토픽별 판정표 (종료코드 0=전부 통과)
    python3 tools/topic_check.py --secs 30 --no-video
    python3 tools/topic_check.py list         # 지금 보이는 토픽 · 타입 · 발행자 수
    python3 tools/topic_check.py echo /rd/fix # 그 토픽의 최신 값을 1초마다 한 줄

먼저 브리지를 띄워 둔다:  ros2 launch rd_bridge bridge.launch.py nav:=false

판정 기준 — 기대 주기는 앱이 보내는 주기다 (자세·VFR_HUD 10Hz · 위치 5Hz · 배터리 1Hz ·
영상 약 32fps, 9/10 실측):
  · 실측 Hz ≥ 기대 × 0.8 (빠른 것은 문제 아님)
  · `/rd/fix` 최대 간격 < 0.6초 (= nav_node 의 stale_secs). 넘으면 비행 중 유도가 헛멈춘다.
    최대 간격에는 **마지막 수신 ~ 점검 끝**도 넣는다 — 도중에 죽은 토픽이 평균 뒤에 숨지 않게
  · 값: TEST 모드 좌표(-32.854,-71.215) · fix 없음 · 배터리 0V(patches/03 미적용 의심) 는 실패

⚠ **링크가 차는지**만 본다. 기체가 명령대로 움직이는지는 말해 주지 않는다.
⚠ 시각은 이 프로세스가 받은 시각이다 — 핫스팟 지연·손실이 섞인 값이고, nav_node 가 겪는 값이다.
"""
import argparse
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

TEST_LATLON = (-32.854074, -71.215363)   # 크래시 후 재시작하면 앱이 내는 가짜 좌표 (제약 #14)
MIN_RATIO = 0.8
FIX_MAX_GAP = 0.6

# 토픽 · 타입 · 기대 Hz
EXPECT = [
    ("/rd/fix",      "sensor_msgs/msg/NavSatFix",           5.0),
    ("/rd/rel_alt",  "std_msgs/msg/Float32",                5.0),
    ("/rd/velocity", "geometry_msgs/msg/TwistStamped",      5.0),
    ("/rd/attitude", "geometry_msgs/msg/Vector3Stamped",   10.0),
    ("/rd/heading",  "std_msgs/msg/Float64",               10.0),
    ("/rd/gspeed",   "std_msgs/msg/Float32",               10.0),
    ("/rd/battery",  "sensor_msgs/msg/BatteryState",        1.0),
    ("/image_raw",   "sensor_msgs/msg/Image",              32.0),
]


def judge_value(topic, m):
    """(실패 여부, 설명)."""
    if topic == "/rd/fix":
        if abs(m.latitude - TEST_LATLON[0]) < 1e-3 and abs(m.longitude - TEST_LATLON[1]) < 1e-3:
            return True, "TEST 모드 좌표 — 앱 강제 중지 후 START"
        if m.status.status < 0:
            return True, f"fix 없음 ({m.latitude:.6f},{m.longitude:.6f}) — 실내면 정상"
        return False, f"{m.latitude:.7f},{m.longitude:.7f} 상대고도 {m.altitude:.1f}m"
    if topic == "/rd/battery":
        if m.voltage <= 0.0:
            return True, "0V — patches/03 미적용 의심"
        return False, f"{m.voltage:.2f}V {m.current:.2f}A"
    if topic == "/rd/attitude":
        v = m.vector
        return False, f"roll {v.x:+.1f}° pitch {v.y:+.1f}° yaw {v.z:.0f}°"
    if topic == "/rd/velocity":
        v = m.twist.linear
        return False, f"동 {v.x:+.2f} 북 {v.y:+.2f} 위 {v.z:+.2f} m/s"
    if topic == "/image_raw":
        return False, f"{m.width}x{m.height} {m.encoding}"
    if hasattr(m, "data"):
        return False, f"{m.data:.2f}"
    return False, ""


def brief(m):
    """echo 용 한 줄. 영상은 픽셀을 찍지 않는다."""
    if hasattr(m, "width") and hasattr(m, "data") and hasattr(m, "encoding"):
        return f"{m.width}x{m.height} {m.encoding}"
    s = str(m)
    return s if len(s) < 300 else s[:300] + "…"


def cmd_check(node, args):
    topics = [e for e in EXPECT if not (args.no_video and e[0] == "/image_raw")]
    stat = {t: {"times": [], "last": None} for t, _, _ in topics}
    for t, typ, _ in topics:
        def cb(msg, t=t):
            stat[t]["times"].append(time.time())
            stat[t]["last"] = msg
        qos = qos_profile_sensor_data if t == "/image_raw" else 10
        node.create_subscription(get_message(typ), t, cb, qos)

    print(f"{args.secs:.0f}초 듣는 중…")
    t0 = time.time()
    while time.time() - t0 < args.secs:
        rclpy.spin_once(node, timeout_sec=0.05)
    t_end = time.time()

    failed = 0
    print(f"\n{'토픽':<14} {'Hz':>6} {'기대':>5} {'최대간격':>8}  판정  값")
    for t, _, hz in topics:
        ts, last = stat[t]["times"], stat[t]["last"]
        if not ts:
            print(f"{t:<14} {'0':>6} {hz:>5.0f} {'-':>8}  FAIL  수신 없음")
            failed += 1
            continue
        real = len(ts) / (t_end - t0)
        gaps = [b - a for a, b in zip(ts, ts[1:])] + [t_end - ts[-1]]
        bad_val, desc = judge_value(t, last)
        bad = real < hz * MIN_RATIO or bad_val or (t == "/rd/fix" and max(gaps) >= FIX_MAX_GAP)
        failed += bad
        print(f"{t:<14} {real:>6.1f} {hz:>5.0f} {max(gaps):>7.2f}s  {'FAIL' if bad else 'OK  '}  {desc}")

    print(f"\n{'✓ 전부 통과' if not failed else f'✗ {failed}개 실패'}  (링크가 찼는지까지만 말해 준다)")
    return 1 if failed else 0


def cmd_list(node, _args):
    t0 = time.time()
    while time.time() - t0 < 1.5:                 # 디스커버리 대기
        rclpy.spin_once(node, timeout_sec=0.1)
    names = sorted(node.get_topic_names_and_types())
    if not names:
        print("보이는 토픽이 없다 — 브리지가 떠 있나? ROS_DOMAIN_ID 가 같은가?")
        return 1
    for name, types in names:
        print(f"{name:<24} {', '.join(types):<36} 발행자 {node.count_publishers(name)}")
    return 0


def cmd_echo(node, args):
    t0 = time.time()
    typ = None
    while time.time() - t0 < 3.0 and typ is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        typ = dict(node.get_topic_names_and_types()).get(args.topic)
    if not typ:
        print(f"✗ {args.topic} 가 안 보인다. `topic_check.py list` 로 확인")
        return 1
    latest = {"msg": None, "n": 0}

    def cb(msg):
        latest["msg"] = msg
        latest["n"] += 1
    node.create_subscription(get_message(typ[0]), args.topic, cb, qos_profile_sensor_data)
    print(f"{args.topic} [{typ[0]}] — Ctrl+C 로 종료")
    last_print = time.time()
    while True:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.time() - last_print >= 1.0:
            n = latest["n"]
            print(f"{n:>4}Hz  {brief(latest['msg']) if latest['msg'] else '(수신 없음)'}", flush=True)
            latest["n"], last_print = 0, time.time()


def main():
    ap = argparse.ArgumentParser(description="브리지 토픽 수신 점검")
    ap.add_argument("--secs", type=float, default=10.0)
    ap.add_argument("--no-video", action="store_true", dest="no_video",
                    help="/image_raw 빼고 (영상 부하 없이 나머지만)")
    sub = ap.add_subparsers(dest="mode")
    sub.add_parser("list", help="보이는 토픽 · 타입 · 발행자 수")
    p = sub.add_parser("echo", help="토픽 최신 값을 1초마다")
    p.add_argument("topic")
    args = ap.parse_args()

    rclpy.init()
    node = Node("topic_check")
    try:
        return {"list": cmd_list, "echo": cmd_echo}.get(args.mode, cmd_check)(node, args)
    except (KeyboardInterrupt, ExternalShutdownException, BrokenPipeError):
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
