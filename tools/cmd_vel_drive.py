#!/usr/bin/env python3
"""cmd_vel 키보드 드라이버 — 방향키로 실시간 조종, RC 로 갑자기 넘겨도 안전하게 계속 돈다.

    python3 tools/cmd_vel_drive.py             # 기본 속도캡으로 시작
    python3 tools/cmd_vel_drive.py --dry-run   # 전송 안 하고 반응만 확인 (실내 사전점검)

키:
    ↑ / ↓   전진 / 후진 (vx)
    ← / →   좌 / 우 횡이동 (vy, 기수 고정)
    a / d   좌 / 우 제자리 회전 (yaw)
    space   즉시 정지 (도구는 계속 실행 상태)
    q       종료 (0속도 1초 송신 후 종료)

고도(vz)는 이 도구의 범위 밖이다 — RC/DJI 앱으로 별도 조작할 것.

⚠⚠ 반드시 SIM 모드에서 먼저 확인하고, 실기는 저고도·저속으로 시작하세요.

────────────────────────────────────────────────────────────────
소스에서 확인한 제약 (cmd_vel_test.py 가 SIM 에서 이미 검증한 축·부호 규약을
그대로 쓴다 — 그 도구 자체는 이 파일이 건드리지 않는다)
────────────────────────────────────────────────────────────────
1. 20Hz 연속 발행 — 한 발의 효력은 200ms 뿐(velocityTicks, DroneModel.java:2250).
2. RC 모드 스위치는 P — 아니면 명령이 에러 없이 조용히 버려진다.
3. RC 스틱을 건드리면 5초간 모든 MAVLink 모션이 차단된다(DroneModel.java:1277).
   → 이 도구는 그 전환을 감지·강제하지 않는다. 스틱을 잡으면 조종기가 이기고,
     이 도구는 ACK 가 안 와도 예외 없이 계속 돌기만 한다. 다시 키보드로 넘기면
     그대로 이어서 동작해야 한다 — 오늘 확인할 것이 바로 이 부분이다.
4. vx>0=전진 · vy>0=우측 · yawRate>0=시계방향 (2026-09-09 SIM 실측 확정).

터미널에는 keyup 이벤트가 없다. 그래서 각 축마다 "마지막으로 그 키를 받은 시각"을
기록해 두고, 300ms 이상 새 입력이 없으면 그 축을 0으로 감쇠시킨다 — 키를 계속
누르고 있으면 OS 오토리핏이 새 입력을 계속 채워 넣어 유지되는 방식이다. (터미널
오토리핏은 보통 한 번에 키 하나만 흘려주므로, 두 방향키를 동시에 눌러도 대각선
이동은 잘 안 될 수 있다 — 오늘은 단일 축 조작 확인이 목적이라 무방하다.)
"""
import argparse, struct, sys, os, time, select, termios, tty

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mav

DECAY_SECS = 0.3
RATE = 20

ARROW = {"A": "up", "B": "down", "C": "right", "D": "left"}


def read_keys(timeout):
    """timeout 초 동안 기다려 눌린 키 이름들의 리스트를 돌려준다.

    한 tick 에 여러 입력이 겹쳐 들어올 수 있어 리스트로 돌려준다. 화살표는
    ESC(0x1b) 뒤에 `[`+방향문자 2바이트가 바로 붙어 오는 것을 전제로 한다
    (터미널이 한 이벤트로 통째로 흘려준다 — 드물게 끊기면 그 tick 은 무시된다).
    """
    keys = []
    end = time.time() + timeout
    while True:
        remain = end - time.time()
        if remain <= 0:
            break
        r, _, _ = select.select([sys.stdin], [], [], remain)
        if not r:
            break
        ch = sys.stdin.read(1)
        if ch == "\x1b":
            seq = sys.stdin.read(2)
            k = ARROW.get(seq[-1:], None)
            if k:
                keys.append(k)
        else:
            keys.append(ch)
    return keys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=14550)
    ap.add_argument("--max-vx", type=float, default=0.5, dest="max_vx")
    ap.add_argument("--max-vy", type=float, default=0.5, dest="max_vy")
    ap.add_argument("--max-yaw", type=float, default=30.0, dest="max_yaw",
                     help="deg/s")
    ap.add_argument("--dry-run", action="store_true", dest="dry_run",
                     help="전송 생략 — 키 반응·감쇠만 화면으로 확인 (실내 사전점검용)")
    args = ap.parse_args()

    link = None
    if not args.dry_run:
        link = mav.Link(port=args.port)
        print(f"앱의 송신 포트를 찾는 중… (0.0.0.0:{args.port} 에서 수신)")
        addr = link.discover(6.0)
        if not addr:
            print("  ✗ 아무것도 안 온다. T0 부터: ./tools/link_check.sh")
            return 1
        print(f"  ✓ 앱 = {addr[0]}:{addr[1]}")
        print("\n" + "=" * 62)
        print("  ⚠ 기체가 움직입니다. RC 모드 스위치 P · SIM 아니면 지금 Ctrl+C")
        print("  ⚠ RC 스틱을 잡으면 5초간 이 도구의 명령이 차단됩니다 (의도된 동작)")
        print("=" * 62)
        for i in range(5, 0, -1):
            print(f"  {i}초 후 시작… (Ctrl+C 로 중단)", end="\r", flush=True)
            link.heartbeat(); link.poll(); time.sleep(1.0)
        print(" " * 40)
    else:
        print("[dry-run] 전송 없이 키 입력·감쇠 타이밍만 확인합니다. q 로 종료.")

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    last_press = {"vx": 0.0, "vy": 0.0, "yaw": 0.0}
    cur = {"vx": 0.0, "vy": 0.0, "yaw": 0.0}
    acks, sent = 0, 0
    quit_requested = False

    try:
        period = 1.0 / RATE
        nxt = time.time()
        while not quit_requested:
            wait = max(0.0, nxt - time.time())
            for k in read_keys(wait):
                now = time.time()
                if k == "up":
                    cur["vx"], last_press["vx"] = args.max_vx, now
                elif k == "down":
                    cur["vx"], last_press["vx"] = -args.max_vx, now
                elif k == "left":
                    cur["vy"], last_press["vy"] = -args.max_vy, now
                elif k == "right":
                    cur["vy"], last_press["vy"] = args.max_vy, now
                elif k == "a":
                    cur["yaw"], last_press["yaw"] = -args.max_yaw, now
                elif k == "d":
                    cur["yaw"], last_press["yaw"] = args.max_yaw, now
                elif k == " ":
                    cur["vx"] = cur["vy"] = cur["yaw"] = 0.0
                    last_press["vx"] = last_press["vy"] = last_press["yaw"] = 0.0
                elif k == "q":
                    quit_requested = True

            now = time.time()
            if now >= nxt:
                for axis in ("vx", "vy", "yaw"):
                    if now - last_press[axis] > DECAY_SECS:
                        cur[axis] = 0.0
                if link is not None:
                    link.set_velocity(cur["vx"], cur["vy"], 0.0, cur["yaw"])
                    sent += 1
                    link.heartbeat()
                    for mid, p in link.poll():
                        if mid == 77:
                            c, _ = struct.unpack_from("<HB", p, 0)
                            if c == 84:
                                acks += 1
                nxt += period
                alt = link.tel.get("alt") if link else None
                hdg = link.tel.get("yaw") if link else None
                sys.stdout.write(
                    f"\rvx={cur['vx']:+.2f} vy={cur['vy']:+.2f} yaw={cur['yaw']:+.1f}°/s"
                    f"  보냄{sent} ACK{acks}  "
                    f"고도{'?' if alt is None else f'{alt:.1f}m'} "
                    f"기수{'?' if hdg is None else f'{hdg:.0f}°'}   "
                )
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print("\n\n… 정지 명령 송출(1초)")
        if link is not None:
            t0 = time.time()
            while time.time() - t0 < 1.0:
                link.set_velocity(0, 0, 0, 0)
                time.sleep(1.0 / RATE)
        print(f"총 송신 {sent}회 · ACK(84) {acks}회")
        if sent and acks == 0:
            print("⚠ ACK 가 하나도 없었다 — 앱이 #84 를 못 받고 있었을 수 있다")
    return 0


if __name__ == "__main__":
    sys.exit(main())
