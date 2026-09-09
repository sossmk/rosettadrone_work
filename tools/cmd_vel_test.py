#!/usr/bin/env python3
"""T5 — cmd_vel. 속도 명령이 실제로 기체를 움직이는지 본다.

    python3 tools/cmd_vel_test.py                     # 기본 순서 (SIM 용)
    python3 tools/cmd_vel_test.py --vx 1.0 --secs 3   # 한 축만
    python3 tools/cmd_vel_test.py --takeoff 5         # 먼저 5m 이륙시키고 시작

⚠⚠ 반드시 **SIM 모드에서 먼저** 하세요. 앱 시작화면의 SIMULATOR 버튼입니다.
   기체가 붙은 채로 날지 않고 검증됩니다.

────────────────────────────────────────────────────────────────
소스에서 확인한, 반드시 지켜야 하는 것들
────────────────────────────────────────────────────────────────
1. **20Hz 연속 발행.**  한 발의 효력은 200ms 뿐이다.
   DroneModel.java:2250  velocityTicks = 4  ×  MOTION_PERIOD_MS = 50ms
   끊기면 즉시 멈춘다. 이 도구가 알아서 계속 쏜다.

2. **RC 모드 스위치는 P.**  아니면 명령이 **에러 없이** 버려진다.
   DroneModel.setVelocities():  if(isForbiddenSwitchMode()) return;

3. **시험 중 조종스틱을 건드리지 말 것.**  건드리면 파일럿 개입으로 보고
   **5초간 MAVLink 모션이 통째로 차단**된다.
   DroneModel.java:1277  timerIgnoreMavLink = now + 5초

4. **type_mask = 1479.**  위치 3비트를 켜야 속도명령으로 인식되고
   (startMotion 의 ignorePosX && Y && Z), YAW_IGNORE 도 켜야
   yaw PID 가 yawRate 를 덮어쓰지 않는다 (processVelocities:2212).

5. **좌표계는 BODY(기체 기준).**  vx=앞, vy=오른쪽 이 의도지만,
   DJI FlightControlData 로 넘어가며 축이 한 번 뒤집힌다.
   **부호와 축 대응은 이 시험으로 실측해서 확정한다.**  vz 도 마찬가지 —
   MAVLink NED 는 아래가 +, DJI throttle 은 위가 + 라 반대일 수 있다.
"""
import argparse, sys, os, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mav

# (이름, vx, vy, vz, yawRate deg/s, 초, 무엇을 눈으로 볼 것인가)
PRESET = [
    ("정지",      0.0, 0.0,  0.0,   0.0, 2.0, "ACK 만 확인 — 파이프라인 검증"),
    ("제자리 회전", 0.0, 0.0,  0.0,  15.0, 4.0, "기수가 약 +60° 돌아야"),
    ("전진",      1.0, 0.0,  0.0,   0.0, 3.0, "약 3m 이동. 어느 방향인지 보라"),
    ("우측",      0.0, 1.0,  0.0,   0.0, 3.0, "약 3m 이동. 어느 방향인지 보라"),
    ("상승",      0.0, 0.0, -0.5,   0.0, 3.0, "MAVLink 는 음수가 위. 실제로 오르나?"),
    ("정지",      0.0, 0.0,  0.0,   0.0, 2.0, "확실히 멈추게"),
]


def snapshot(link):
    t = link.tel
    return (t.get("lat"), t.get("lon"), t.get("alt"), t.get("yaw"), t.get("gspeed"))


def fmt(v, unit="", nd=1):
    return "?" if v is None else f"{v:.{nd}f}{unit}"


def run_step(link, name, vx, vy, vz, yr, secs, hint, rate):
    period = 1.0 / rate
    print(f"\n  ▶ {name}  vx={vx:+.2f} vy={vy:+.2f} vz={vz:+.2f} yawRate={yr:+.1f}°/s"
          f"  ({secs:.0f}초 · {rate}Hz)")
    print(f"    보아야 할 것: {hint}")

    link.poll()
    before = snapshot(link)
    acks, sent = 0, 0
    t0 = time.time()
    nxt = t0
    while time.time() - t0 < secs:
        now = time.time()
        if now >= nxt:
            link.set_velocity(vx, vy, vz, yr)
            sent += 1
            nxt += period
        link.heartbeat()
        for mid, p in link.poll():
            if mid == 77:
                import struct
                c, res = struct.unpack_from("<HB", p, 0)
                if c == 84:
                    acks += 1
        time.sleep(0.002)

    after = snapshot(link)
    blat, blon, balt, byaw, _ = before
    alat, alon, aalt, ayaw, agsp = after

    dist = (mav.meters_between(blat, blon, alat, alon)
            if None not in (blat, blon, alat, alon) else None)
    dyaw = None
    if byaw is not None and ayaw is not None:
        dyaw = (ayaw - byaw + 540.0) % 360.0 - 180.0     # -180..180
    dalt = None if None in (balt, aalt) else aalt - balt

    print(f"    보냄 {sent}회 · ACK(84) {acks}회")
    print(f"    고도 {fmt(balt,'m')} → {fmt(aalt,'m')} ({fmt(dalt,'m',2)})"
          f"  ·  기수 {fmt(byaw,'°')} → {fmt(ayaw,'°')} ({fmt(dyaw,'°')})"
          f"  ·  수평이동 {fmt(dist,'m',2)}  ·  대지속도 {fmt(agsp,' m/s',2)}")
    if sent and acks == 0:
        print("    ⚠ ACK 가 하나도 없다 — 앱이 #84 를 안 받고 있다. T2 부터 다시.")
    return acks


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=14550)
    ap.add_argument("--rate", type=int, default=20, help="발행 주기 Hz (최소 5)")
    ap.add_argument("--vx", type=float)
    ap.add_argument("--vy", type=float)
    ap.add_argument("--vz", type=float)
    ap.add_argument("--yaw-rate", type=float, dest="yr")
    ap.add_argument("--secs", type=float, default=3.0)
    ap.add_argument("--takeoff", type=float, metavar="ALT",
                    help="시작 전에 이 고도(m)로 이륙시킨다. 명시할 때만 실행")
    args = ap.parse_args()

    if args.rate < 5:
        print("✗ --rate 는 5 이상이어야 한다 (200ms 데드맨). 20 권장."); return 1

    link = mav.Link(port=args.port)
    print(f"[1] 앱의 송신 포트를 찾는 중… (0.0.0.0:{args.port} 에서 수신)")
    addr = link.discover(6.0)
    if not addr:
        print("    ✗ 아무것도 안 온다. T0 부터: ./tools/link_check.sh")
        return 1
    print(f"    ✓ 앱 = {addr[0]}:{addr[1]}")

    print("\n" + "=" * 62)
    print("  ⚠ 기체가 **움직입니다**. SIM 모드가 아니면 지금 Ctrl+C.")
    print("  ⚠ RC 모드 스위치 P · 시험 중 스틱을 건드리지 말 것(5초 차단됨)")
    print("=" * 62)
    for i in range(5, 0, -1):
        print(f"  {i}초 후 시작… (Ctrl+C 로 중단)", end="\r", flush=True)
        link.heartbeat(); link.poll(); time.sleep(1.0)
    print(" " * 40)

    if args.takeoff is not None:
        print(f"[2] 이륙 — 목표 {args.takeoff:.0f} m")
        link.command_long(mav.CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, args.takeoff)
        res, dt = link.wait_ack(mav.CMD_NAV_TAKEOFF, secs=10.0)
        print(f"    ACK: {mav.MAV_RESULT.get(res, res) if res is not None else '없음 ✗'}")
        print("    고도가 오를 때까지 대기…")
        t0 = time.time()
        while time.time() - t0 < 25:
            link.heartbeat(); link.poll()
            if (link.tel.get("alt") or 0) >= args.takeoff * 0.8:
                break
            time.sleep(0.1)
        print(f"    현재 고도 {fmt(link.tel.get('alt'), 'm')}")

    if args.vx is not None or args.vy is not None or args.vz is not None or args.yr is not None:
        steps = [("사용자 지정", args.vx or 0.0, args.vy or 0.0, args.vz or 0.0,
                  args.yr or 0.0, args.secs, "지정한 축만 움직여야")]
    else:
        steps = PRESET

    print(f"\n[3] 속도 명령 — {len(steps)}단계")
    total = 0
    try:
        for s in steps:
            total += run_step(link, *s, rate=args.rate)
    except KeyboardInterrupt:
        print("\n  중단됨")
    finally:
        # 무슨 일이 있어도 마지막엔 0 을 1초간 쏴서 확실히 세운다
        print("\n  … 정지 명령 송출(1초)")
        t0 = time.time()
        while time.time() - t0 < 1.0:
            link.set_velocity(0, 0, 0, 0)
            time.sleep(1.0 / args.rate)

    print("\n=== 판정 ===")
    if total == 0:
        print("  ✗ ACK(84) 가 하나도 없다 — 앱이 SET_POSITION_TARGET_LOCAL_NED 를 못 받고 있다")
        print("    → tools/uplink_test.py 로 업링크부터 분리해서 확인")
    else:
        print(f"  ✓ ACK(84) 총 {total}회 — 앱이 속도 명령을 받고 있다")
        print("    이제 남은 건 **축 대응과 부호**다. 위의 이동 방향을 보고 확정하라:")
        print("      · '전진' 에서 기체가 실제로 앞으로 갔나, 옆으로 갔나?")
        print("      · '상승' 에서 vz=-0.5 로 정말 올라갔나, 내려갔나?")
        print("    결과를 진행상황.md 에 적어 두면 ROS 브리지에서 그대로 쓴다.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
