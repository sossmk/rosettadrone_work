#!/usr/bin/env python3
"""앱 대역(stand-in) — 기체 없이 업링크 도구를 시험한다.

    python3 tools/fake_app.py                 # 127.0.0.1:14550 의 도구와 대화한다
    python3 tools/fake_app.py --gcs 14550

RosettaDrone 앱이 GCS 에게 보이는 행동만 흉내 낸다. 우리가 보낸 것이 **바이트로
정말 나갔는지**, **부호가 어느 필드에 어떻게 실렸는지**, **몇 Hz 로 나가는지**를
숫자로 보여 준다. cmd_vel_drive.py · cmd_vel_node · goto_gps.py 를 기체 없이
여기에 붙여 볼 수 있다.

⚠ **이것은 시뮬레이터가 아니다.** 기체가 어떻게 움직이는지는 흉내 내지 않는다.
  축·부호 규약의 근거는 어디까지나 9/9 SIM 실측이다(할일.md 제약 #2) — 이 도구는
  "우리 쪽이 그 규약대로 보내고 있는가"만 판정한다.

흉내 내는 앱의 성질 (전부 소스 근거가 있다):
  · 송신 포트가 임시 포트다 (제약 #7) — 그래서 여기도 bind(0) 으로 띄운다
  · COMMAND_ACK 은 **무조건 ACCEPTED** (제약 #1, DroneModel.java:2029)
  · MISSION_ITEM_INT 는 target_system 이 1 이 아니면 **조용히 버린다** (제약 #18) —
    9/11 에 goto 가 안 먹던 그 버그를 여기서 다시 밟으면 화면에 뜬다
  · #84 가 끊기면 200ms 뒤 모션이 끝난다 (제약 #3) — 그리고 **Virtual Stick 은
    풀리지 않는다** (cancelMotion() 은 타이머만 죽인다, DroneModel.java:2279)
"""
import argparse
import math
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mav

DEADMAN_SECS = 0.2          # velocityTicks 4 × MOTION_PERIOD_MS 50ms


def main():
    # 파일·tee 로 흘려보낼 때도 즉시 보이게 한다. 블록 버퍼링이면 Ctrl+C·timeout 에
    # 마지막 출력이 통째로 날아간다 (시험 로그를 남길 때 실제로 겪음).
    sys.stdout.reconfigure(line_buffering=True)

    ap = argparse.ArgumentParser()
    ap.add_argument("--gcs", type=int, default=14550, help="도구가 듣는 포트")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--rate", type=float, default=5.0, help="텔레메트리 Hz")
    ap.add_argument("--lat", type=float, default=37.5665)
    ap.add_argument("--lon", type=float, default=126.9780)
    ap.add_argument("--alt", type=float, default=10.0)
    ap.add_argument("--yaw", type=float, default=0.0, help="기수 (deg)")
    ap.add_argument("--sysid", type=int, default=1, help="앱의 mSystemId (기본 1)")
    args = ap.parse_args()

    link = mav.Link(port=0, sysid=args.sysid, compid=1)   # 임시 포트 — 앱과 같다
    link.addr = (args.host, args.gcs)
    myport = link.sock.getsockname()[1]

    print(f"앱 대역 시작 — 임시 포트 {myport} 에서 {args.host}:{args.gcs} 로 보낸다")
    print(f"  sysid={args.sysid} · 좌표 {args.lat:.6f},{args.lon:.6f} 고도 {args.alt}m "
          f"기수 {args.yaw}°")
    print("  Ctrl+C 로 종료\n")

    period = 1.0 / args.rate
    nxt = time.time()
    n84 = n73 = n76 = 0
    dropped73 = 0
    t_first84 = None
    t_last84 = 0.0
    in_motion = False
    vstick = False
    last_print = 0.0
    last_cmd = None

    try:
        while True:
            now = time.time()

            # ---------------------------------------------------- 받은 것 처리
            for mid, p in link.poll():
                if mid == 84:                                   # 속도 명령
                    # ⚠ 오프셋을 손으로 셀 것. time_boot_ms(0) · x(4) y(8) z(12) ·
                    #   vx(16) vy(20) vz(24) · afx(28) afy(32) afz(36) ·
                    #   yaw(40) yaw_rate(44) · type_mask(48) tsys(50) tcomp(51) frame(52).
                    #   처음에 yaw_rate 를 28(=afx) 에서 읽어 "yawRate 가 0 으로 온다"고
                    #   오판했다 — 정작 보내는 쪽은 멀쩡했다.
                    vx, vy, vz = struct.unpack_from("<fff", p, 16)
                    yawr = struct.unpack_from("<f", p, 44)[0]
                    mask, tsys, tcomp, frame = struct.unpack_from("<HBBB", p, 48)
                    n84 += 1
                    if t_first84 is None:
                        t_first84 = now
                    t_last84 = now
                    last_cmd = (vx, vy, vz, math.degrees(yawr), mask, frame)
                    if not vstick:
                        vstick = True
                        print("  [앱] startMotion() → setVirtualSticksEnabled(TRUE)"
                              " — 이 순간부터 물리 RC 스틱은 무시된다")
                    if not in_motion:
                        in_motion = True
                    if mask != mav.MASK_VEL_YAWRATE:
                        print(f"  ⚠ type_mask={mask} (기대 {mav.MASK_VEL_YAWRATE}) "
                              f"— 앱이 속도 명령으로 안 볼 수 있다")
                    link.send(77, struct.pack("<HB", 84, 0))     # 무조건 ACCEPTED

                elif mid == 73:                                  # MISSION_ITEM_INT
                    n73 += 1
                    lat = struct.unpack_from("<i", p, 16)[0] / 1e7
                    lon = struct.unpack_from("<i", p, 20)[0] / 1e7
                    z = struct.unpack_from("<f", p, 24)[0]
                    seq, cmd = struct.unpack_from("<HH", p, 28)
                    tsys, tcomp, frame = struct.unpack_from("<BBB", p, 32)
                    if tsys != args.sysid:
                        dropped73 += 1
                        print(f"  ✗ [앱] MISSION_ITEM_INT 를 **조용히 버렸다** — "
                              f"target_system={tsys} ≠ mSystemId={args.sysid} "
                              f"(제약 #18. 9/11 에 goto 가 안 먹은 바로 그 원인)")
                    else:
                        print(f"  ✓ [앱] flyTo({lat:.7f}, {lon:.7f}, {z:.1f}) "
                              f"cmd={cmd} frame={frame} → MISSION_ACCEPTED")
                        link.send(47, struct.pack("<BBB", 255, 190, 0))
                        if not vstick:
                            vstick = True
                            print("  [앱] flyTo 도 startMotion() 을 타므로 "
                                  "Virtual Sticks TRUE (제약 #19)")

                elif mid == 76:                                  # COMMAND_LONG
                    n76 += 1
                    cmd = struct.unpack_from("<H", p, 28)[0]
                    print(f"  [앱] COMMAND_LONG {cmd} → ACK ACCEPTED "
                          f"(⚠ 제약 #1 — 실패해도 이렇게 온다)")
                    link.send(77, struct.pack("<HB", cmd, 0))

            # ------------------------------------------------- 200ms 데드맨
            if in_motion and now - t_last84 > DEADMAN_SECS:
                in_motion = False
                dur = t_last84 - (t_first84 or t_last84)
                hz = (n84 / dur) if dur > 0.2 else 0.0
                print(f"  [앱] 200ms 데드맨 → cancelMotion() — 모션 종료 "
                      f"(#84 {n84}발, 약 {hz:.1f}Hz)")
                print(f"        ⚠ Virtual Stick 은 여전히 {'TRUE' if vstick else 'FALSE'} "
                      f"— 조종권은 RC 로 안 돌아간다 (앱의 Safety 버튼만이 푼다)")
                t_first84, n84 = None, 0

            # ---------------------------------------------------- 텔레메트리
            if now >= nxt:
                nxt += period
                link.heartbeat()
                link.send(33, struct.pack(
                    "<IiiiihhhH", int(now * 1000) & 0xFFFFFFFF,
                    int(args.lat * 1e7), int(args.lon * 1e7),
                    int(args.alt * 1000), int(args.alt * 1000), 0, 0, 0,
                    int(args.yaw * 100) % 36000))
                link.send(30, struct.pack(
                    "<Iffffff", int(now * 1000) & 0xFFFFFFFF,
                    0.0, 0.0, math.radians(args.yaw), 0.0, 0.0, 0.0))
                link.send(74, struct.pack(
                    "<ffffhH", 0.0, 0.0, args.alt, 0.0, int(args.yaw), 0))
                link.send(24, struct.pack(
                    "<Qiii", int(now * 1e6), int(args.lat * 1e7),
                    int(args.lon * 1e7), int(args.alt * 1000))
                    + struct.pack("<HHHHBB", 100, 150, 0, 0, 3, 14))  # fix=3D, 위성 14

            # --------------------------------------------------- 화면 한 줄
            if last_cmd and in_motion and now - last_print > 0.5:
                last_print = now
                vx, vy, vz, yawd, mask, frame = last_cmd
                up = "위" if vz > 0 else ("아래" if vz < 0 else "-")
                side = "오른쪽" if vy > 0 else ("왼쪽" if vy < 0 else "-")
                fwd = "앞" if vx > 0 else ("뒤" if vx < 0 else "-")
                turn = "시계" if yawd > 0 else ("반시계" if yawd < 0 else "-")
                print(f"  #84 vx={vx:+.2f}({fwd}) vy={vy:+.2f}({side}) "
                      f"vz={vz:+.2f}({up}) yawRate={yawd:+.1f}°/s({turn})  "
                      f"frame={frame}")

            time.sleep(0.005)

    except KeyboardInterrupt:
        print(f"\n\n받은 것: #84 속도 {n84}발 · #73 좌표이동 {n73}발"
              f"{f' (그중 {dropped73}발은 target_system 때문에 버려짐)' if dropped73 else ''}"
              f" · #76 명령 {n76}발")
        if link.addr_changes:
            print(f"상대 주소가 {link.addr_changes}번 바뀌었다")
    return 0


if __name__ == "__main__":
    sys.exit(main())
