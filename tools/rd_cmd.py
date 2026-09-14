#!/usr/bin/env python3
"""업링크(PC → 앱) 명령 도구 — 명령을 보내고 앱의 반응을 본다.

    python3 tools/rd_cmd.py ping                     # T2 업링크가 열렸나 (기체 안 움직임)
    python3 tools/rd_cmd.py gimbal                   # T3 짐벌 0 → -45 → -90 → 0
    python3 tools/rd_cmd.py gimbal --angle -30       #    한 각도만 / --yaw 는 좌우(상대각)
    python3 tools/rd_cmd.py drive                    # 방향키 조종 (#84 20Hz)
    python3 tools/rd_cmd.py drive --dry-run          #    전송 없이 키 반응만
    python3 tools/rd_cmd.py vel --axes               # T5 축·부호 확인 순서 (SIM 에서)
    python3 tools/rd_cmd.py vel --vx 0.5 --secs 3    #    한 축만

모든 서브명령에 `--port 14550`. UDP 14550 을 잡으므로 QGC·decode_check.py·link_node
와 **동시에 못 쓴다.** 기체 없이 시험하려면 다른 터미널에 `python3 tools/fake_app.py`.
앞으로 takeoff/land/rtl 도 여기에 붙인다 (할일.md A).

────────────────────────────────────────────────────────────────
⚠⚠ 기체가 움직이는 명령(drive · vel) 전에 알아둘 것
────────────────────────────────────────────────────────────────
1. **조종권이 RC 로 저절로 안 돌아온다** (제약 #20·#21). #84 를 한 발이라도 받으면 앱이
   Virtual Stick 을 켜고, 그동안 DJI 는 물리 스틱을 무시한다. 스틱을 잡아도, 도구를 꺼도
   안 풀린다(cancelMotion() 은 타이머만 죽인다, DroneModel.java:2279).
     → **실기에서 통한 것: S 모드 전환** (9/11 비행 중 실제로 이걸로 복구했다)
     → 소스상 경로: ① 도구 종료 → ② 스틱 놓고 5초 → ③ 앱의 Safety(자물쇠) 버튼
        (MainActivity:824. 실기 확인 전)
2. **RC 모드 스위치는 P.** 아니면 명령이 에러 없이 버려진다 (isForbiddenSwitchMode).
3. **20Hz 연속 발행.** 한 발의 효력은 200ms (velocityTicks 4 × 50ms, DroneModel.java:2250).
4. **처음은 SIM 에서.** 앱 시작화면의 SIMULATOR 버튼.
5. 축·부호 (9/9 SIM 실측): vx>0=앞 · vy>0=오른쪽 · **vz>0=위**(MAVLink NED 와 반대) ·
   yawRate>0=시계방향. 정지 명령 뒤에도 0.6~0.8m 더 간다 (제약 #6).

⚠ **ACCEPTED 는 성공이 아니다** (DroneModel.java:2029 — DJI 가 에러를 내도 무조건 ACCEPTED).
  짐벌은 눈으로, 이동은 좌표·기수 변화로 판정한다.
"""
import argparse, os, select, struct, sys, termios, time, tty

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mav

CMD_REQUEST_AUTOPILOT_CAPABILITIES = 520
CH_PITCH, CH_YAW = 9, 8


def connect(port):
    link = mav.Link(port=port)
    print(f"앱의 송신 포트를 찾는 중… (0.0.0.0:{port} 에서 수신)")
    addr = link.discover(6.0)
    if not addr:
        print("  ✗ 아무것도 안 온다. 먼저: ./tools/link_check.sh")
        return None
    print(f"  ✓ 앱 = {addr[0]}:{addr[1]}")
    return link


def countdown(link, what):
    print("\n" + "=" * 62)
    print(f"  ⚠ {what}. RC 모드 스위치 P · SIM 이 아니면 지금 Ctrl+C")
    print("  ⚠ 조종권은 저절로 RC 로 안 돌아온다 — S 모드 전환 (위 docstring 1번)")
    print("=" * 62)
    for i in range(5, 0, -1):
        print(f"  {i}초 후 시작… (Ctrl+C 로 중단)", end="\r", flush=True)
        link.heartbeat(); link.poll(); time.sleep(1.0)
    print(" " * 40)


def stop(link, rate=20):
    """무슨 일이 있어도 마지막엔 0 을 1초간 쏴서 세운다."""
    print("\n… 정지 명령 송출(1초)")
    t0 = time.time()
    while time.time() - t0 < 1.0:
        link.set_velocity(0, 0, 0, 0)
        time.sleep(1.0 / rate)


def count_ack84(link):
    n = 0
    for mid, p in link.poll():
        if mid == 77 and struct.unpack_from("<H", p, 0)[0] == 84:
            n += 1
    return n


def fmt(v, unit="", nd=1):
    return "?" if v is None else f"{v:.{nd}f}{unit}"


# ------------------------------------------------------------------ ping
def cmd_ping(args):
    """HEARTBEAT + REQUEST_AUTOPILOT_CAPABILITIES(520) → COMMAND_ACK · AUTOPILOT_VERSION."""
    link = connect(args.port)
    if link is None:
        return 1
    print(f"\nHEARTBEAT + 명령 520 송신 · {args.secs:.0f}초 대기")
    acks, version = set(), None
    t0, sent = time.time(), 0
    while time.time() - t0 < args.secs:
        if time.time() - t0 > sent:                     # 1초에 한 번
            link.heartbeat()
            link.command_long(CMD_REQUEST_AUTOPILOT_CAPABILITIES, 1.0)
            sent += 1
        for mid, p in link.poll():
            if mid == 77:
                acks.add(struct.unpack_from("<HB", p, 0))
            elif mid == 148:
                version = struct.unpack_from("<Q", p, 0)[0]
        time.sleep(0.01)

    print()
    if acks:
        print("  ✓ COMMAND_ACK 도착 — 업링크가 열려 있다")
        for c, r in sorted(acks):
            print(f"      명령 {c} → {mav.MAV_RESULT.get(r, r)}")
    else:
        print("  ✗ COMMAND_ACK 없음 — 앱이 우리 패킷을 안 받고 있다")
    if version is not None:
        print(f"  ✓ AUTOPILOT_VERSION 도착 — capabilities=0x{version:016x}")
    return 0 if acks else 1


# ---------------------------------------------------------------- gimbal
def cmd_gimbal(args):
    """DO_SET_SERVO(183). 앱에 DO_MOUNT_CONTROL(205)·GIMBAL_MANAGER(1000) 은 미구현이다.

    DroneModel.doSetGimbal(): 각도 = (PWM - 1500) / 5.5
    채널 9 = pitch (절대각) · 채널 8 = yaw (**상대각 — 누적된다**)
    """
    link = connect(args.port)
    if link is None:
        return 1
    ch = CH_YAW if args.yaw else CH_PITCH
    if args.angle is not None:
        seq = [("yaw" if args.yaw else "pitch", args.angle)]
    elif args.yaw:
        seq = [("yaw +30°", 30.0), ("yaw -30°", -30.0)]
    else:
        seq = [("수평 0°", 0.0), ("아래 -45°", -45.0), ("수직 -90°", -90.0), ("수평 복귀", 0.0)]

    print("\n짐벌을 움직인다 — **카메라를 눈으로 보세요**")
    if args.yaw:
        print("  ⚠ yaw 는 상대각이라 누적된다. 원위치는 손으로 맞춰야 한다")
    ok = 0
    for label, angle in seq:
        pwm = 1500.0 + angle * 5.5
        print(f"  {label:<12} 채널 {ch} · {angle:+.0f}° · PWM {pwm:.1f} … ", end="", flush=True)
        link.command_long(mav.CMD_DO_SET_SERVO, float(ch), pwm)
        res, dt = link.wait_ack(mav.CMD_DO_SET_SERVO, secs=8.0)
        if res is None:
            print("ACK 없음 ✗")
        else:
            print(f"{mav.MAV_RESULT.get(res, res)}  ({dt:.1f}초)")
            if res == 1:
                print("      → TEMPORARILY_REJECTED = mGimbal 이 null. 기체가 안 붙었다")
            ok += res == 0
        # ⚠ 짧게 쉬면 안 된다. 앱은 2초에 걸쳐 돌리는데 ACK 는 0.2초 만에 온다.
        #   1초만 쉬면 다음 명령이 회전을 덮어써 "안 움직였다"로 보인다 (9/9 에 속았다).
        time.sleep(args.hold)

    print(f"\n{ok}/{len(seq)} ACCEPTED — ⚠ 성공 여부는 **카메라가 실제로 움직였는지** 로만 갈린다")
    if ok == 0:
        print("  ACK 가 아예 없으면 업링크부터: rd_cmd.py ping")
    return 0 if ok == len(seq) else 1


# ----------------------------------------------------------------- drive
ARROW = {"A": "up", "B": "down", "C": "right", "D": "left"}


def read_keys(timeout):
    """timeout 초 동안 눌린 키들. 화살표는 ESC [ A~D 3바이트로 온다."""
    keys, end = [], time.time() + timeout
    while True:
        remain = end - time.time()
        if remain <= 0 or not select.select([sys.stdin], [], [], remain)[0]:
            break
        ch = sys.stdin.read(1)
        if ch == "\x1b":
            k = ARROW.get(sys.stdin.read(2)[-1:])
            if k:
                keys.append(k)
        else:
            keys.append(ch)
    return keys


def cmd_drive(args):
    """방향키 조종. ↑↓ 전진/후진 · ←→ 좌/우 · a/d 회전 · space 정지 · q 종료. 고도는 RC 로.

    터미널엔 keyup 이 없어서, 축마다 마지막 입력 시각을 두고 300ms 새 입력이 없으면
    그 축을 0 으로 만든다. 키를 누르고 있으면 OS 오토리핏이 입력을 채운다.
    """
    rate, decay = 20, 0.3
    link = None
    if args.dry_run:
        print("[dry-run] 전송 없이 키 입력·감쇠만 확인합니다. q 로 종료.")
    else:
        link = connect(args.port)
        if link is None:
            return 1
        countdown(link, "기체가 움직입니다")

    keymap = {"up": ("vx", args.max_vx), "down": ("vx", -args.max_vx),
              "right": ("vy", args.max_vy), "left": ("vy", -args.max_vy),
              "d": ("yaw", args.max_yaw), "a": ("yaw", -args.max_yaw)}
    cur = {"vx": 0.0, "vy": 0.0, "yaw": 0.0}
    last = dict(cur)
    sent = acks = 0

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        nxt, quit_ = time.time(), False
        while not quit_:
            for k in read_keys(max(0.0, nxt - time.time())):
                if k in keymap:
                    axis, v = keymap[k]
                    cur[axis], last[axis] = v, time.time()
                elif k == " ":
                    cur = {a: 0.0 for a in cur}
                    last = {a: 0.0 for a in cur}
                elif k == "q":
                    quit_ = True
            now = time.time()
            if now < nxt:
                continue
            nxt += 1.0 / rate
            for axis in cur:
                if now - last[axis] > decay:
                    cur[axis] = 0.0
            if link:
                link.set_velocity(cur["vx"], cur["vy"], 0.0, cur["yaw"])
                link.heartbeat()
                sent += 1
                acks += count_ack84(link)
            t = link.tel if link else {}
            sys.stdout.write(f"\rvx={cur['vx']:+.2f} vy={cur['vy']:+.2f} yaw={cur['yaw']:+.1f}°/s"
                             f"  보냄{sent} ACK{acks}  고도{fmt(t.get('alt'), 'm')}"
                             f" 기수{fmt(t.get('yaw'), '°', 0)}   ")
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        if link:
            stop(link, rate)
            print(f"총 송신 {sent}회 · ACK(84) {acks}회")
            print("⚠ RC 로 되돌리기: S 모드 전환 (또는 스틱 놓고 5초 → 앱 Safety 버튼)")
    return 0


# ------------------------------------------------------------------- vel
# (이름, vx, vy, vz, yawRate deg/s, 초, 무엇을 볼 것인가)
PRESET = [
    ("정지",       0.0, 0.0,  0.0,  0.0, 2.0, "ACK 만 확인"),
    ("제자리 회전", 0.0, 0.0,  0.0, 15.0, 4.0, "기수가 약 +60° 돌아야"),
    ("전진",       1.0, 0.0,  0.0,  0.0, 3.0, "약 3m. 어느 방향인지"),
    ("우측",       0.0, 1.0,  0.0,  0.0, 3.0, "약 3m. 어느 방향인지"),
    ("상승",       0.0, 0.0,  0.5,  0.0, 3.0, "vz>0 이 위"),
    ("정지",       0.0, 0.0,  0.0,  0.0, 2.0, "확실히 멈추게"),
]
AXES = [
    ("전진 vx=+1", 1.0, 0.0, 0.0, 0.0, 4.0, "기수 기준 어느 쪽으로 가는가"),
    ("정지",       0.0, 0.0, 0.0, 0.0, 2.0, ""),
    ("우측 vy=+1", 0.0, 1.0, 0.0, 0.0, 4.0, "기수 기준 어느 쪽으로 가는가"),
    ("정지",       0.0, 0.0, 0.0, 0.0, 2.0, ""),
    ("vz=+0.5",   0.0, 0.0, 0.5, 0.0, 3.0, "양수로 정말 올라가는가"),
    ("정지",       0.0, 0.0, 0.0, 0.0, 2.0, ""),
]


def run_step(link, name, vx, vy, vz, yr, secs, hint, rate):
    print(f"\n  ▶ {name}  vx={vx:+.2f} vy={vy:+.2f} vz={vz:+.2f} yawRate={yr:+.1f}°/s ({secs:.0f}초)")
    if hint:
        print(f"    볼 것: {hint}")
    link.poll()
    t = link.tel
    b = (t.get("lat"), t.get("lon"), t.get("alt"), t.get("yaw"))
    sent = acks = 0
    t0 = nxt = time.time()
    while time.time() - t0 < secs:
        if time.time() >= nxt:
            link.set_velocity(vx, vy, vz, yr)
            sent += 1
            nxt += 1.0 / rate
        link.heartbeat()
        acks += count_ack84(link)
        time.sleep(0.002)
    a = (t.get("lat"), t.get("lon"), t.get("alt"), t.get("yaw"))

    dist = mav.meters_between(b[0], b[1], a[0], a[1]) if None not in b[:2] + a[:2] else None
    dyaw = None if None in (b[3], a[3]) else (a[3] - b[3] + 540.0) % 360.0 - 180.0
    dalt = None if None in (b[2], a[2]) else a[2] - b[2]
    print(f"    보냄 {sent} · ACK(84) {acks} · 고도변화 {fmt(dalt, 'm', 2)} · "
          f"기수변화 {fmt(dyaw, '°')} · 수평이동 {fmt(dist, 'm', 2)}")
    # 거리만으론 축을 못 가린다. 기수 기준 **어느 쪽으로** 갔는지가 핵심이다.
    if dist is not None and dist >= 0.5 and b[3] is not None:
        rel = mav.relative_bearing(mav.bearing_between(b[0], b[1], a[0], a[1]), b[3])
        print(f"    이동 방향: 기수 기준 {rel:+.0f}° → **{mav.name_direction(rel)}**")
    return acks


def cmd_vel(args):
    """스크립트 속도 명령 + 기수 기준 이동 방향 측정. --takeoff 는 명시할 때만."""
    if args.rate < 5:
        print("✗ --rate 는 5 이상 (200ms 데드맨). 20 권장")
        return 1
    link = connect(args.port)
    if link is None:
        return 1
    countdown(link, "기체가 움직입니다")

    if args.takeoff is not None:
        print(f"이륙 — 목표 {args.takeoff:.0f} m")
        link.command_long(mav.CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, args.takeoff)
        res, _ = link.wait_ack(mav.CMD_NAV_TAKEOFF, secs=10.0)
        print(f"  ACK: {mav.MAV_RESULT.get(res, res) if res is not None else '없음 ✗'}")
        t0 = time.time()
        while time.time() - t0 < 25 and (link.tel.get("alt") or 0) < args.takeoff * 0.8:
            link.heartbeat(); link.poll(); time.sleep(0.1)
        print(f"  현재 고도 {fmt(link.tel.get('alt'), 'm')}")

    if any(v is not None for v in (args.vx, args.vy, args.vz, args.yr)):
        steps = [("사용자 지정", args.vx or 0.0, args.vy or 0.0, args.vz or 0.0,
                  args.yr or 0.0, args.secs, "지정한 축만 움직여야")]
    else:
        steps = AXES if args.axes else PRESET

    total = 0
    try:
        for s in steps:
            total += run_step(link, *s, rate=args.rate)
    except KeyboardInterrupt:
        print("\n  중단됨")
    finally:
        stop(link, args.rate)

    if total == 0:
        print("✗ ACK(84) 가 하나도 없다 — 업링크부터: rd_cmd.py ping")
        return 1
    print(f"✓ ACK(84) {total}회. ⚠ 판정은 ACK 가 아니라 위의 이동 방향·거리로")
    return 0


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser(description="업링크 명령 도구 (ping · gimbal · drive · vel)")
    ap.add_argument("--port", type=int, default=14550)
    sub = ap.add_subparsers(dest="mode", required=True)

    p = sub.add_parser("ping", help="업링크가 열렸나 — 기체 안 움직임")
    p.add_argument("--secs", type=float, default=10.0)

    p = sub.add_parser("gimbal", help="짐벌 틸트 (DO_SET_SERVO)")
    p.add_argument("--angle", type=float, help="이 각도 하나만 (기본: 0 → -45 → -90 → 0)")
    p.add_argument("--yaw", action="store_true", help="pitch 대신 yaw(상대각·누적)")
    p.add_argument("--hold", type=float, default=3.5, help="단계마다 머무는 초 (2초 넘게)")

    p = sub.add_parser("drive", help="방향키 조종 (#84 20Hz)")
    p.add_argument("--max-vx", type=float, default=0.5, dest="max_vx")
    p.add_argument("--max-vy", type=float, default=0.5, dest="max_vy")
    p.add_argument("--max-yaw", type=float, default=30.0, dest="max_yaw", help="deg/s")
    p.add_argument("--dry-run", action="store_true", dest="dry_run", help="전송 없이 키 반응만")

    p = sub.add_parser("vel", help="스크립트 속도 명령 + 이동 방향 측정")
    p.add_argument("--rate", type=int, default=20)
    p.add_argument("--vx", type=float)
    p.add_argument("--vy", type=float)
    p.add_argument("--vz", type=float)
    p.add_argument("--yaw-rate", type=float, dest="yr")
    p.add_argument("--secs", type=float, default=3.0)
    p.add_argument("--axes", action="store_true", help="축·부호 확인용 짧은 순서")
    p.add_argument("--takeoff", type=float, metavar="ALT", help="먼저 이 고도(m)로 이륙")

    args = ap.parse_args()
    return {"ping": cmd_ping, "gimbal": cmd_gimbal, "drive": cmd_drive, "vel": cmd_vel}[args.mode](args)


if __name__ == "__main__":
    sys.exit(main())
