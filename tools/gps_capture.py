#!/usr/bin/env python3
"""좌표 측정 — RC 로 날아가 Enter 를 누르면 그 자리 좌표를 저장한다.

    python3 tools/gps_capture.py      # 1Hz 로 좌표 표시 · Enter 로 저장
    cat ~/drone_ws/capture/waypoints.jsonl

잰 좌표는 missions/*.txt 에 `gps <위도> <경도> [고도]` 로 옮긴다 (rd_cmd.py mission).
⚠ 14550 을 직접 잡는다 — 브리지(link_node)를 끄고 쓴다.
⚠ 고도는 상대고도(이륙 지점 기준)다.
"""
import argparse, datetime, json, os, select, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mav

FAKE_LAT, FAKE_LON = -32.8540743, -71.2153629  # DummyProduct.java 의 TEST 모드 지문
CAPTURE_DIR = os.path.expanduser("~/drone_ws/capture")
CAPTURE_FILE = os.path.join(CAPTURE_DIR, "waypoints.jsonl")


def fmt(v, unit="", nd=6):
    return "?" if v is None else f"{v:.{nd}f}{unit}"


def main():
    ap = argparse.ArgumentParser(description="현재 좌표를 실시간으로 보고 Enter 로 저장")
    ap.add_argument("--port", type=int, default=14550)
    args = ap.parse_args()

    link = mav.Link(port=args.port)
    print(f"앱의 송신 포트를 찾는 중… (0.0.0.0:{args.port} 에서 수신)")
    addr = link.discover(6.0)
    if not addr:
        print("  ✗ 아무것도 안 온다. 먼저: ./tools/link_check.sh")
        return 1
    print(f"  ✓ 앱 = {addr[0]}:{addr[1]}")

    os.makedirs(CAPTURE_DIR, exist_ok=True)
    print("\n1Hz 로 좌표를 계속 찍습니다. Enter 를 누르면 그 순간 값을 저장합니다.")
    print(f"저장 위치: {CAPTURE_FILE}")
    print("Ctrl+C 로 종료.\n")

    t_last_print = 0.0
    try:
        while True:
            link.heartbeat()
            link.poll()
            now = time.time()
            if now - t_last_print >= 1.0:
                t_last_print = now
                lat, lon, alt = link.tel.get("lat"), link.tel.get("lon"), link.tel.get("alt")
                fix, sats = link.tel.get("fix"), link.tel.get("sats")
                tag = ""
                if lat is not None and lon is not None:
                    if abs(lat - FAKE_LAT) < 1e-3 and abs(lon - FAKE_LON) < 1e-3:
                        tag = "  ✗ TEST 모드 가짜좌표"
                    elif lat == 0.0 and lon == 0.0:
                        tag = "  ⚠ 0,0 (실내 미수신 또는 앱 문제)"
                sys.stdout.write(
                    f"\r위도 {fmt(lat)}  경도 {fmt(lon)}  고도 {fmt(alt,'m',1)}  "
                    f"fix={fix} 위성{sats}{tag}   [Enter=저장]   "
                )
                sys.stdout.flush()
            r, _, _ = select.select([sys.stdin], [], [], 0.05)
            if r:
                sys.stdin.readline()
                lat, lon, alt = link.tel.get("lat"), link.tel.get("lon"), link.tel.get("alt")
                if lat is None or lon is None:
                    print("\n  ✗ 아직 좌표를 못 받았다 — 저장 안 함")
                    continue
                entry = {"ts": datetime.datetime.now().isoformat(timespec="seconds"),
                         "lat": lat, "lon": lon, "alt": alt}
                with open(CAPTURE_FILE, "a") as f:
                    f.write(json.dumps(entry, ensure_ascii=False) + "\n")
                alt_arg = f"{alt:.1f}" if alt is not None else "<상대고도>"
                print(f"\n  ✓ 저장: lat={lat:.7f} lon={lon:.7f} alt={fmt(alt,'m',1)}")
                print(f"    미션 파일에: gps  {lat:.7f}  {lon:.7f}  {alt_arg}")
                print(f"    바로 줄 때:  ros2 topic pub --once /nav/goal sensor_msgs/msg/NavSatFix "
                      f"'{{latitude: {lat:.7f}, longitude: {lon:.7f}, altitude: {alt_arg}}}'\n")
    except KeyboardInterrupt:
        print("\n  종료")
    return 0


if __name__ == "__main__":
    sys.exit(main())
