#!/usr/bin/env python3
"""좌표 이동 — 특정 지점을 측정해 두고, 나중에 그 좌표로 단발 이동한다.

    python3 tools/goto_gps.py capture                      # 현재 좌표를 실시간으로 보고 저장
    python3 tools/goto_gps.py goto <LAT> <LON> [--alt M]   # 그 좌표로 단발 이동

워크플로: ①RC로 지점까지 수동 비행 → ②capture 로 좌표 측정·저장 → ③RC로 수동 복귀
→ ④저장했던(또는 그때 화면에 찍힌) 좌표를 goto 에 그대로 입력.
**복귀는 항상 수동이다 — 이 도구는 왕복을 하지 않는다.**

⚠⚠ 첫 실기 시도는 반드시 짧은 거리·저고도로 하세요. flyTo() 의 고도 기준(상대고도인지)을
   앱 소스로 재확인 못 했다 — 이 도구는 GLOBAL_RELATIVE_ALT(상대고도, capture 가 읽은
   그 값) 기준으로 보낸다는 가정으로 동작한다.

────────────────────────────────────────────────────────────────
소스/문서에서 이미 확인된, 반드시 지켜야 하는 것 (할일.md 제약표와 동일)
────────────────────────────────────────────────────────────────
1. MISSION_ACK 는 "받았다"일 뿐 "성공"이 아니다(ACK 전반이 그렇다 — DroneModel.java:2029
   와 같은 종류의 함정). 실제 도착은 이동 감시 화면의 **좌표 변화**로 판정한다.
2. 멈추려면 RC 스틱을 잡으면 된다 — 건드리면 5초간 모든 MAVLink 모션이 차단된다
   (DroneModel.java:1277). 이 도구는 별도의 abort 명령을 보내지 않는다.
3. RC 모드 스위치는 P 여야 한다.
4. markWaypoint() NPE(MainActivity.java:511,528)는 앱 화면 UI 전용이고 이 MAVLink
   경로(MISSION_ITEM_INT → flyTo())와는 무관하다.
"""
import argparse, datetime, json, os, select, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mav

FAKE_LAT, FAKE_LON = -32.8540743, -71.2153629  # DummyProduct.java 의 TEST 모드 지문
CAPTURE_DIR = os.path.expanduser("~/drone_ws/capture")
CAPTURE_FILE = os.path.join(CAPTURE_DIR, "waypoints.jsonl")


def fmt(v, unit="", nd=6):
    return "?" if v is None else f"{v:.{nd}f}{unit}"


def connect(port):
    link = mav.Link(port=port)
    print(f"앱의 송신 포트를 찾는 중… (0.0.0.0:{port} 에서 수신)")
    addr = link.discover(6.0)
    if not addr:
        print("  ✗ 아무것도 안 온다. T0 부터: ./tools/link_check.sh")
        return None
    print(f"  ✓ 앱 = {addr[0]}:{addr[1]}")
    return link


def cmd_capture(args):
    link = connect(args.port)
    if link is None:
        return 1
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
                alt_arg = f"{alt:.1f}" if alt is not None else "0"
                print(f"\n  ✓ 저장: lat={lat:.7f} lon={lon:.7f} alt={fmt(alt,'m',1)}")
                print(f"    goto 로 쓸 때:")
                print(f"    python3 tools/goto_gps.py goto {lat:.7f} {lon:.7f} --alt {alt_arg}\n")
    except KeyboardInterrupt:
        print("\n  종료")
    return 0


def cmd_goto(args):
    link = connect(args.port)
    if link is None:
        return 1

    alt = args.alt
    if alt is None:
        link.poll()
        alt = link.tel.get("alt")
        if alt is None:
            print("  ✗ --alt 를 안 줬고 현재 고도도 안 들어와서 고도를 정할 수 없다")
            return 1
        print(f"  (--alt 없음 → 현재 상대고도 {alt:.1f}m 로 유지)")

    print(f"\n목표: lat={args.lat:.7f} lon={args.lon:.7f} alt={alt:.1f}m (상대고도 가정)")
    print("=" * 62)
    print("  ⚠ 기체가 이동합니다. RC 모드 스위치 P · 멈추려면 RC 스틱을 잡으세요")
    print("     (스틱을 건드리면 5초간 모든 MAVLink 모션이 차단됩니다)")
    print("=" * 62)
    for i in range(5, 0, -1):
        print(f"  {i}초 후 전송… (Ctrl+C 로 취소)", end="\r", flush=True)
        link.heartbeat(); link.poll(); time.sleep(1.0)
    print(" " * 40)

    print("[1] MISSION_ITEM_INT 전송 (UDP 신뢰성 확보용 3회)")
    for _ in range(3):
        link.mission_item_int(args.lat, args.lon, alt)
        link.heartbeat()
        time.sleep(0.3)

    print("[2] MISSION_ACK 대기 (최대 5초)")
    t0 = time.time()
    ack = None
    while time.time() - t0 < 5.0:
        link.heartbeat()
        link.poll()
        if "mission_ack" in link.tel:
            ack = link.tel.pop("mission_ack")
            break
        time.sleep(0.05)
    if ack is None:
        print("    ✗ MISSION_ACK 없음 — 앱이 #73 을 못 받았을 수 있다")
    else:
        print(f"    {mav.MAV_MISSION_RESULT.get(ack, ack)}")
    print("    ⚠ ACK 는 '받았다'일 뿐 '도착했다'가 아니다 — 아래 거리로 실제 판정")

    print("\n[3] 이동 감시 (Ctrl+C 로 중단 · 도착반경 도달 시 자동 종료)")
    try:
        while True:
            link.heartbeat()
            link.poll()
            lat, lon, alt_now = link.tel.get("lat"), link.tel.get("lon"), link.tel.get("alt")
            if lat is None or lon is None:
                sys.stdout.write("\r  좌표 수신 대기 중…   ")
            else:
                dist = mav.meters_between(lat, lon, args.lat, args.lon)
                sys.stdout.write(
                    f"\r  현재 {lat:.7f},{lon:.7f}  고도{fmt(alt_now,'m',1)}  "
                    f"목표까지 {dist:6.1f} m   "
                )
                if dist <= args.arrive_radius:
                    print("\n  ✓ 도착반경 진입")
                    break
            sys.stdout.flush()
            time.sleep(0.3)
    except KeyboardInterrupt:
        print("\n  중단됨 (RC 로 넘기셨다면 정상입니다)")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=14550)
    sub = ap.add_subparsers(dest="mode", required=True)

    sub.add_parser("capture", help="현재 좌표를 실시간으로 보고 Enter 로 저장")

    gp = sub.add_parser("goto", help="지정한 좌표로 단발 이동")
    gp.add_argument("lat", type=float)
    gp.add_argument("lon", type=float)
    gp.add_argument("--alt", type=float, default=None, metavar="M",
                     help="상대고도(m). 생략하면 현재 고도를 유지")
    gp.add_argument("--arrive-radius", type=float, default=3.0, dest="arrive_radius")

    args = ap.parse_args()
    if args.mode == "capture":
        return cmd_capture(args)
    return cmd_goto(args)


if __name__ == "__main__":
    sys.exit(main())
