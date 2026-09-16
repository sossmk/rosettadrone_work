#!/usr/bin/env python3
"""경로 기록 — RC 로 날면서 Enter 를 누를 때마다 그 자리를 미션 파일에 한 줄씩 쌓는다.

    python3 tools/gps_capture.py missions/site_a.txt            # 고도도 그때 값으로
    python3 tools/gps_capture.py missions/site_a.txt --alt 5    # 고도는 5m 로 고정
    python3 tools/gps_capture.py missions/site_a.txt --alt 0    # 고도를 안 적는다 (= 비행 때 유지)

    # 기록이 끝나면 이 도구를 끄고 브리지를 띄워 그대로 날린다
    python3 tools/rd_cmd.py mission missions/site_a.txt

파일이 있으면 뒤에 이어 붙인다. 줄은 `gps <위도> <경도> [고도]` — 손으로 고쳐도 된다.
⚠ 14550 을 직접 잡는다 — 브리지(link_node)를 끄고 쓴다.
⚠ 고도는 상대고도(이륙 지점 기준)다.
"""
import argparse, os, select, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mav

FAKE_LAT, FAKE_LON = -32.8540743, -71.2153629  # 앱 TEST 모드의 가짜 좌표


def bad_fix(lat, lon):
    """저장하면 안 되는 좌표면 그 이유. 가짜 좌표를 경로에 넣으면 수천 km 를 날아간다."""
    if lat is None or lon is None:
        return "아직 좌표를 못 받았다"
    if abs(lat - FAKE_LAT) < 1e-3 and abs(lon - FAKE_LON) < 1e-3:
        return "앱 TEST 모드의 가짜 좌표다"
    if lat == 0.0 and lon == 0.0:
        return "0,0 이다 (위성 미수신)"
    return None


def main():
    ap = argparse.ArgumentParser(description="Enter 마다 지금 좌표를 미션 파일에 쌓는다")
    ap.add_argument("file", help="missions/<이름>.txt — 없으면 만든다")
    ap.add_argument("--alt", type=float, help="고도를 이 값으로 고정 (0 = 안 적음 → 비행 때 유지)")
    ap.add_argument("--port", type=int, default=14550)
    args = ap.parse_args()

    link = mav.Link(port=args.port)
    print(f"앱의 송신 포트를 찾는 중… (0.0.0.0:{args.port})")
    addr = link.discover(6.0)
    if not addr:
        print("  ✗ 아무것도 안 온다. 브리지가 켜져 있으면 끄고, 아니면 ./tools/link_check.sh")
        return 1
    print(f"  ✓ 앱 = {addr[0]}:{addr[1]}")

    new = not os.path.exists(args.file)
    os.makedirs(os.path.dirname(os.path.abspath(args.file)), exist_ok=True)
    if new:
        with open(args.file, "w", encoding="utf-8") as f:
            f.write(f"# {time.strftime('%Y-%m-%d %H:%M')} gps_capture.py 로 기록\n"
                    f"# 줄마다: gps <위도> <경도> [고도]   (rd_cmd.py mission 으로 날린다)\n")
    n = sum(1 for line in open(args.file, encoding="utf-8") if line.split("#")[0].strip())
    prev = None
    print(f"\n{args.file} — {'새 파일' if new else f'이어 쓰기 (이미 {n}점)'}")
    print("Enter = 지금 자리 저장 · Ctrl+C = 끝\n")

    t_print = 0.0
    try:
        while True:
            link.heartbeat()
            link.poll()
            t = link.tel
            lat, lon, alt = t.get("lat"), t.get("lon"), t.get("alt")
            if time.time() - t_print >= 1.0:
                t_print = time.time()
                why = bad_fix(lat, lon)
                leg = f" · 직전 점에서 {mav.meters_between(*prev, lat, lon):.1f}m" if prev and not why else ""
                sys.stdout.write(
                    f"\r위도 {lat if lat is not None else '?'}  경도 {lon if lon is not None else '?'}  "
                    f"고도 {'?' if alt is None else f'{alt:.1f}m'}  위성{t.get('sats')}{leg}"
                    f"{'  ✗ ' + why if why else ''}   [{n}점]\x1b[K")
                sys.stdout.flush()
            if not select.select([sys.stdin], [], [], 0.05)[0]:
                continue
            sys.stdin.readline()
            why = bad_fix(lat, lon)
            if why:
                print(f"\n  ✗ 저장 안 함 — {why}")
                continue
            a = alt if args.alt is None else args.alt
            line = f"gps  {lat:.7f}  {lon:.7f}" + (f"  {a:.1f}" if a and a > 0 else "")
            with open(args.file, "a", encoding="utf-8") as f:
                f.write(line + "\n")
            n += 1
            leg = f" (직전 점에서 {mav.meters_between(*prev, lat, lon):.1f}m)" if prev else ""
            print(f"\n  ✓ {n}번 저장: {line}{leg}")
            prev = (lat, lon)
    except KeyboardInterrupt:
        pass
    print(f"\n끝 — {args.file} 에 {n}점. 날리기: python3 tools/rd_cmd.py mission {args.file}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
