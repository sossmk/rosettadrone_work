#!/usr/bin/env python3
"""업링크(PC → 앱) 명령 도구 — 명령을 보내고 앱의 반응을 본다.

⚠ **브리지(link_node) 가 떠 있어야 한다.** 이 도구는 UDP 를 직접 잡지 않고 토픽으로 말한다
  (/cmd_vel · /rd/command → link_node → 앱). 그래서 조종하면서 `ros2 topic echo` 를 볼 수 있다.

    ros2 launch rd_bridge bridge.launch.py nav:=false     # 먼저 (다른 터미널)

    python3 tools/rd_cmd.py ping                     # T2 업링크가 열렸나 (기체 안 움직임)
    python3 tools/rd_cmd.py gimbal                   # T3 짐벌 0 → -45 → -90 → 0
    python3 tools/rd_cmd.py gimbal --angle -30       #    한 각도만
    python3 tools/rd_cmd.py gimbal --yaw             #    좌우 0 → +30 → 0 → -30 → 0 (방향 실기 확인용)
    python3 tools/rd_cmd.py drive                    # 방향키 조종 + w/s 고도 + r/f 틸트 (#84 20Hz)
    python3 tools/rd_cmd.py drive --dry-run          #    전송 없이 키 반응만
    python3 tools/rd_cmd.py vel --axes               # T5 축·부호 확인 순서 (SIM 에서)
    python3 tools/rd_cmd.py vel --vx 0.5 --secs 3    #    한 축만
    python3 tools/rd_cmd.py follow                  # 태그 추종 켜고 끄기 + 실기 튜닝
    python3 tools/rd_cmd.py mission missions/square10.txt   # 경유점 비행 (nav_node)

기체 없이 시험하려면 다른 터미널에 `python3 tools/fake_app.py` 를 띄우고 브리지를 붙인다.
속도는 link_node 캡(수평 2.5 · 수직 1.5 m/s · 75°/s)에서 한 번 더 깎인다.
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
import argparse, math, os, select, sys, termios, threading, time, tty

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Empty, Float64, Float64MultiArray, Int32MultiArray
from tf2_msgs.msg import TFMessage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mav   # 명령 번호 · 거리·방위 계산만 쓴다

SPIN = {"stop": False, "thread": None, "node": None}

CMD_REQUEST_AUTOPILOT_CAPABILITIES = 520
CH_PITCH, CH_YAW = 9, 8


class Bridge(Node):
    """link_node 와 이야기하는 창구. /cmd_vel 은 앱 규약(vy>0 오른쪽, yaw>0 시계)."""

    def __init__(self):
        super().__init__("rd_cmd")
        self.pub_vel = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_cmd = self.create_publisher(Float64MultiArray, "rd/command", 10)
        self.create_subscription(NavSatFix, "rd/fix", self.on_fix, 10)
        self.create_subscription(Float64, "rd/heading", self.on_hdg, 10)
        self.create_subscription(Int32MultiArray, "rd/ack", self.on_ack, 50)
        self.tel = {}
        self.acks = []       # (받은시각, 명령번호, 결과)
        self.acks84 = 0

    def on_fix(self, m):
        self.tel.update(lat=m.latitude, lon=m.longitude, alt=m.altitude)

    def on_hdg(self, m):
        self.tel["yaw"] = m.data

    def on_ack(self, m):
        cmd, res = m.data[0], m.data[1]
        if cmd == 84:
            self.acks84 += 1
        else:
            self.acks.append((time.time(), cmd, res))

    def vel(self, vx, vy, vz, yaw_deg=0.0):
        t = Twist()
        t.linear.x, t.linear.y, t.linear.z = float(vx), float(vy), float(vz)
        t.angular.z = math.radians(yaw_deg)
        self.pub_vel.publish(t)

    def command(self, cmd, *params):
        self.pub_cmd.publish(Float64MultiArray(data=[float(cmd)] + [float(x) for x in params]))

    def wait_ack(self, cmd, secs=6.0):
        t0 = time.time()
        while time.time() - t0 < secs:
            for t, c, r in self.acks:
                if t >= t0 and c == cmd:
                    return r, t - t0
            time.sleep(0.02)
        return None, secs


def connect():
    """브리지가 떠 있는지 /rd/heading 으로 확인한다."""
    rclpy.init()
    b = Bridge()

    def spin():
        while not SPIN["stop"]:
            rclpy.spin_once(b, timeout_sec=0.1)
    SPIN["node"], SPIN["thread"] = b, threading.Thread(target=spin, daemon=True)
    SPIN["thread"].start()
    print("브리지(link_node) 확인 중… (/rd/heading)")
    t0 = time.time()
    while time.time() - t0 < 6.0 and "yaw" not in b.tel:
        time.sleep(0.1)
    if "yaw" not in b.tel:
        print("  ✗ /rd/heading 이 안 온다. 먼저: ros2 launch rd_bridge bridge.launch.py nav:=false")
        return None
    time.sleep(0.5)      # 퍼블리셔가 link_node 와 짝지어질 시간
    print(f"  ✓ 기수 {b.tel['yaw']:.0f}° · 고도 {fmt(b.tel.get('alt'), 'm')}")
    return b


def countdown(link, what):
    print("\n" + "=" * 62)
    print(f"  ⚠ {what}. RC 모드 스위치 P · SIM 이 아니면 지금 Ctrl+C")
    print("  ⚠ 조종권은 저절로 RC 로 안 돌아온다 — S 모드 전환 (위 docstring 1번)")
    print("=" * 62)
    for i in range(5, 0, -1):
        print(f"  {i}초 후 시작… (Ctrl+C 로 중단)", end="\r", flush=True)
        time.sleep(1.0)
    print(" " * 40)


def stop(link, rate=20):
    """무슨 일이 있어도 마지막엔 0 을 1초간 쏴서 세운다."""
    print("\n… 정지 명령 송출(1초)")
    t0 = time.time()
    while time.time() - t0 < 1.0:
        link.vel(0, 0, 0, 0)
        time.sleep(1.0 / rate)


def fmt(v, unit="", nd=1):
    return "?" if v is None else f"{v:.{nd}f}{unit}"


# ------------------------------------------------------------------ ping
def cmd_ping(args):
    """REQUEST_AUTOPILOT_CAPABILITIES(520) → COMMAND_ACK."""
    link = connect()
    if link is None:
        return 1
    print(f"\n명령 520 송신 · {args.secs:.0f}초 대기")
    t0, sent = time.time(), 0
    while time.time() - t0 < args.secs:
        if time.time() - t0 > sent:                     # 1초에 한 번
            link.command(CMD_REQUEST_AUTOPILOT_CAPABILITIES, 1.0)
            sent += 1
        time.sleep(0.05)
    acks = {(c, r) for t, c, r in link.acks if t >= t0}

    print()
    if acks:
        print("  ✓ COMMAND_ACK 도착 — 업링크가 열려 있다")
        for c, r in sorted(acks):
            print(f"      명령 {c} → {mav.MAV_RESULT.get(r, r)}")
    else:
        print("  ✗ COMMAND_ACK 없음 — 앱이 우리 패킷을 안 받고 있다")
    return 0 if acks else 1


# ---------------------------------------------------------------- gimbal
def cmd_gimbal(args):
    """DO_SET_SERVO(183). 앱에 DO_MOUNT_CONTROL(205)·GIMBAL_MANAGER(1000) 은 미구현이다.

    DroneModel.doSetGimbal(): 각도 = (PWM - 1500) / 5.5
    채널 9 = pitch (ABSOLUTE_ANGLE) — 9/9 실기 확인
    채널 8 = yaw — RELATIVE_ANGLE 로 **(새 값 - 앱이 기억한 직전 값)** 만큼 돌린다
      (DroneModel.java:2019-2021). 기억값은 앱 시작 때 0 이므로 같은 값을 두 번 보내면
      두 번째는 0° 회전이다. 즉 "앱 시작 때 방향 기준 각도"처럼 동작하지만, 짐벌이
      다른 이유(RC 휠·기체 회전 추종)로 돌면 그 기준이 어긋난다.
      ⚠ 부호(+ 가 시계인지)와 Mavic 2 에서 yaw 가 실제로 도는지는 **실기 미확인**.
    """
    link = connect()
    if link is None:
        return 1
    ch = CH_YAW if args.yaw else CH_PITCH
    if args.angle is not None:
        seq = [("yaw" if args.yaw else "pitch", args.angle)]
    elif args.yaw:
        # 매번 0 을 거친다 — 앱이 직전 값과의 차이만큼 돌리므로 0 을 보내면 시작 방향으로 온다
        seq = [("yaw +30°", 30.0), ("yaw 0° 복귀", 0.0), ("yaw -30°", -30.0), ("yaw 0° 복귀", 0.0)]
    else:
        seq = [("수평 0°", 0.0), ("아래 -45°", -45.0), ("수직 -90°", -90.0), ("수평 복귀", 0.0)]

    print("\n짐벌을 움직인다 — **카메라를 눈으로 보세요**")
    if args.yaw:
        print("  ⚠ 볼 것: +30° 에서 카메라가 기수 기준 **오른쪽(위에서 봐서 시계)** 인가 왼쪽인가")
        print("  ⚠ 기준은 앱 시작 때 방향이다. 앱 재시작 없이 RC 휠로 돌렸다면 0° 가 정면이 아닐 수 있다")
    ok = 0
    for label, angle in seq:
        pwm = 1500.0 + angle * 5.5
        print(f"  {label:<12} 채널 {ch} · {angle:+.0f}° · PWM {pwm:.1f} … ", end="", flush=True)
        link.command(mav.CMD_DO_SET_SERVO, float(ch), pwm)
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
    """방향키 조종. ↑↓ 전진/후진 · ←→ 좌/우 · w/s 상승/하강 · a/d 회전 · space 정지 · q 종료.
    r/f 카메라 틸트 위/아래 (--tilt-step 씩, 0° 수평 ~ -90° 수직).

    w 는 vz>0 = 위 (9/9 SIM 실측, MAVLink NED 와 반대 — 제약 #2).

    틸트는 DO_SET_SERVO ch9 (ABSOLUTE_ANGLE) 로 목표각을 보낸다. 시작값은 0° 로 가정한다
    (앱의 m_ServoPos_pitch 초기값). 앱은 2초에 걸쳐 돌리므로 키를 연타하면 도중에 새 목표로
    바뀔 뿐이고, 명령은 목표가 바뀌었을 때만 0.3초에 한 번 이하로 보낸다.

    터미널엔 keyup 이 없어서, 축마다 마지막 입력 시각을 두고 300ms 새 입력이 없으면
    그 축을 0 으로 만든다. 키를 누르고 있으면 OS 오토리핏이 입력을 채운다.
    """
    rate, decay = 20, 0.3
    link = None
    if args.dry_run:
        print("[dry-run] 전송 없이 키 입력·감쇠만 확인합니다. q 로 종료.")
    else:
        link = connect()
        if link is None:
            return 1
        countdown(link, "기체가 움직입니다")

    keymap = {"up": ("vx", args.max_vx), "down": ("vx", -args.max_vx),
              "right": ("vy", args.max_vy), "left": ("vy", -args.max_vy),
              "w": ("vz", args.max_vz), "s": ("vz", -args.max_vz),
              "d": ("yaw", args.max_yaw), "a": ("yaw", -args.max_yaw)}
    cur = {"vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw": 0.0}
    last = dict(cur)
    sent = 0
    tilt = tilt_sent = 0.0
    t_tilt = 0.0

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
                elif k in ("r", "f"):
                    step = args.tilt_step if k == "r" else -args.tilt_step
                    tilt = max(-90.0, min(0.0, tilt + step))
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
                link.vel(cur["vx"], cur["vy"], cur["vz"], cur["yaw"])
                sent += 1
                if tilt != tilt_sent and now - t_tilt >= 0.3:
                    link.command(mav.CMD_DO_SET_SERVO, float(CH_PITCH), 1500.0 + tilt * 5.5)
                    tilt_sent, t_tilt = tilt, now
            t = link.tel if link else {}
            sys.stdout.write(f"\rvx={cur['vx']:+.2f} vy={cur['vy']:+.2f} vz={cur['vz']:+.2f}"
                             f" yaw={cur['yaw']:+.1f}°/s 틸트{tilt:+.0f}°"
                             f"  보냄{sent} ACK{link.acks84 if link else 0}  고도{fmt(t.get('alt'), 'm')}"
                             f" 기수{fmt(t.get('yaw'), '°', 0)}   ")
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        if link:
            stop(link, rate)
            print(f"총 송신 {sent}회 · ACK(84) {link.acks84}회")
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
    t = link.tel
    b = (t.get("lat"), t.get("lon"), t.get("alt"), t.get("yaw"))
    sent, acks0 = 0, link.acks84
    t0 = nxt = time.time()
    while time.time() - t0 < secs:
        if time.time() >= nxt:
            link.vel(vx, vy, vz, yr)
            sent += 1
            nxt += 1.0 / rate
        time.sleep(0.002)
    acks = link.acks84 - acks0
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
    link = connect()
    if link is None:
        return 1
    countdown(link, "기체가 움직입니다")

    if args.takeoff is not None:
        print(f"이륙 — 목표 {args.takeoff:.0f} m")
        link.command(mav.CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, args.takeoff)
        res, _ = link.wait_ack(mav.CMD_NAV_TAKEOFF, secs=10.0)
        print(f"  ACK: {mav.MAV_RESULT.get(res, res) if res is not None else '없음 ✗'}")
        t0 = time.time()
        while time.time() - t0 < 25 and (link.tel.get("alt") or 0) < args.takeoff * 0.8:
            time.sleep(0.1)
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


# ---------------------------------------------------------------- follow
FOLLOW_NODE = "/rd_tag_follow"

# 튜닝 키 → (파라미터, 증감, 하한, 상한). 실기에서 내려놓고 고치는 시간이 제일 아깝다
TUNE = {
    "[": ("kp",           -0.05, 0.05,  3.0), "]": ("kp",           +0.05, 0.05,  3.0),
    ",": ("max_speed",    -0.10, 0.10,  2.5), ".": ("max_speed",    +0.10, 0.10,  2.5),
    ";": ("min_speed",    -0.05, 0.00,  1.0), "'": ("min_speed",    +0.05, 0.00,  1.0),
    "-": ("hold_alt",     -0.50, 0.50, 30.0), "=": ("hold_alt",     +0.50, 0.50, 30.0),
    "o": ("deadband",     -0.10, 0.10,  3.0), "p": ("deadband",     +0.10, 0.10,  3.0),
    "n": ("yaw_deadband", -1.00, 1.00, 45.0), "m": ("yaw_deadband", +1.00, 1.00, 45.0),
}
TUNE_HELP = ("[ ] kp · , . max_speed · ; ' min_speed · - = hold_alt · "
             "o p deadband · n m yaw_deadband · y yaw 켜고끄기")


def tune(k, table, par, tuner):
    """튜닝 키 하나를 적용한다. table 에 없는 키는 무시."""
    if k in table:
        name, step, lo, hi = table[k]
        par[name] = round(min(hi, max(lo, par[name] + step)), 2)
        tuner.write(name, par[name])


def wait_future(fut, secs=2.0):
    """⚠ spin_until_future_complete 를 쓰면 안 된다 — connect() 의 spin 스레드와 겹친다."""
    t0 = time.time()
    while not fut.done() and time.time() - t0 < secs:
        time.sleep(0.01)
    return fut.result() if fut.done() else None


class Tuner:
    """추종 노드의 파라미터를 날면서 읽고 쓴다 (rcl_interfaces 서비스)."""

    def __init__(self, node, target=FOLLOW_NODE):
        self.cli_get = node.create_client(GetParameters, f"{target}/get_parameters")
        self.cli_set = node.create_client(SetParameters, f"{target}/set_parameters")
        self.ok = self.cli_get.wait_for_service(timeout_sec=4.0)

    def read(self, names):
        if not self.ok:
            return {}
        r = wait_future(self.cli_get.call_async(GetParameters.Request(names=list(names))))
        if r is None or len(r.values) != len(names):
            return {}
        pick = {ParameterType.PARAMETER_BOOL: "bool_value",
                ParameterType.PARAMETER_INTEGER: "integer_value",
                ParameterType.PARAMETER_DOUBLE: "double_value",
                ParameterType.PARAMETER_STRING: "string_value"}
        return {n: getattr(v, pick[v.type]) for n, v in zip(names, r.values)}

    def write(self, name, value):
        if isinstance(value, bool):
            pv = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
        else:
            pv = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
        r = wait_future(self.cli_set.call_async(
            SetParameters.Request(parameters=[ParamMsg(name=name, value=pv)])))
        return bool(r and r.results and r.results[0].successful)


def cmd_follow(args):
    """태그 추종을 한 터미널에서 켜고 끄고 튜닝한다 (터미널 4·5 를 대신한다).

    space 시작/정지 · q 종료 · 그 밖의 키는 TUNE_HELP.

    ⚠ 이 도구는 **/cmd_vel 을 한 발도 안 쏜다.** 기체를 움직이는 것은 추종 노드고
      이쪽은 /tag/follow 를 토글할 뿐이다. 그래서 drive·vel 과 달리 여기서 Ctrl+C 를
      눌러도 이 도구가 기체를 세우지 못한다 — 대신 나가기 전에 false 를 두 번 보내고,
      그걸 받은 노드가 0 을 내면 link_node 의 1초 브레이크가 선다.
    """
    link = connect()
    if link is None:
        return 1

    tuner = Tuner(link)
    names = ("kp", "min_speed", "max_speed", "hold_alt", "deadband", "yaw_deadband",
             "yaw_enable", "sign_fwd", "sign_right")
    par = tuner.read(names)
    if not par:
        print(f"  ✗ {FOLLOW_NODE} 의 파라미터를 못 읽는다 — 추종 노드가 안 떠 있다")
        print("     ros2 launch rd_bridge bridge.launch.py tag:=true follow:=true nav:=false \\")
        print("         camera_info_file:=$HOME/rosettadrone_work/ros2/src/rd_bridge/config/camera.yaml")
        return 1

    tag = {"t": 0.0, "x": 0.0, "y": 0.0, "z": 0.0}
    cmd = {"t": 0.0, "v": (0.0, 0.0, 0.0, 0.0)}

    def on_tf(msg):
        for tf in msg.transforms:
            if tf.child_frame_id == args.tag_frame:
                p = tf.transform.translation
                tag.update(t=time.time(), x=p.x, y=p.y, z=p.z)

    def on_cmd(m):
        cmd.update(t=time.time(), v=(m.linear.x, m.linear.y, m.linear.z,
                                     math.degrees(m.angular.z)))

    pub = link.create_publisher(Bool, "tag/follow", 10)
    link.create_subscription(TFMessage, "/tf", on_tf, 50)
    link.create_subscription(Twist, "cmd_vel", on_cmd, 10)
    time.sleep(0.6)      # 퍼블리셔가 추종 노드와 짝지어질 시간 (첫 토글을 잃지 않게)

    print("\n" + "=" * 72)
    print("  ⚠ space 를 누르면 기체가 태그를 따라 **움직인다**. RC 모드 스위치 P")
    print("  ⚠ 조종권은 저절로 RC 로 안 돌아온다 — 급하면 S 모드 전환")
    print("=" * 72)
    print(f"  space 시작/정지 · q 종료\n  {TUNE_HELP}\n")

    armed, t_arm = False, 0.0
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        nxt, quit_ = time.time(), False
        while not quit_:
            for k in read_keys(max(0.0, nxt - time.time())):
                if k == " ":
                    armed, t_arm = not armed, time.time()
                    pub.publish(Bool(data=armed))
                elif k == "q":
                    quit_ = True
                elif k == "y":
                    par["yaw_enable"] = not par["yaw_enable"]
                    tuner.write("yaw_enable", par["yaw_enable"])
                else:
                    tune(k, TUNE, par, tuner)
            now = time.time()
            if now < nxt:
                continue
            nxt += 0.2                                   # 5Hz 로만 그린다

            age = now - tag["t"]
            if tag["t"] == 0.0:
                see = "태그 본 적 없음       "
            elif age > 1.0:
                see = f"태그 {age:4.1f}초 전       "
            else:
                see = (f"앞{par['sign_fwd'] * tag['y']:+.2f} "
                       f"오{par['sign_right'] * tag['x']:+.2f} z{tag['z']:4.2f}m")
            if not armed:
                state = "대기"
            elif now - cmd["t"] < 1.5:
                state = "추종중"
            else:     # 켠 직후는 아직 첫 명령이 안 왔을 뿐이다
                state = "시작 중…" if now - t_arm < 1.5 else "명령끊김"
            vx, vy, vz, wz = cmd["v"]
            sys.stdout.write(
                f"\r[{state}] {see} · cmd {vx:+.2f} {vy:+.2f} {vz:+.2f} {wz:+3.0f}°/s"
                f" · kp{par['kp']:.2f} spd{par['min_speed']:.2f}~{par['max_speed']:.2f}"
                f" alt{par['hold_alt']:.1f} db{par['deadband']:.1f}/{par['yaw_deadband']:.0f}°"
                f" yaw{'켬' if par['yaw_enable'] else '끔'}"
                f" · 고도{fmt(link.tel.get('alt'), 'm')}\x1b[K")      # 줄 끝까지 지운다
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        for _ in range(2):                    # 무슨 일이 있어도 정지시키고 나간다
            pub.publish(Bool(data=False))
            time.sleep(0.3)
        print("\n■ /tag/follow false 송출 — 노드가 0 을 내고 link_node 가 1초 브레이크")
        print("⚠ RC 로 되돌리기: S 모드 전환 (또는 스틱 놓고 5초 → 앱 Safety 버튼)")
    return 0


# --------------------------------------------------------------- mission
NAV_NODE = "/rd_nav"
NAV_TUNE = {
    "[": ("kp",            -0.05, 0.05, 3.0), "]": ("kp",            +0.05, 0.05, 3.0),
    ",": ("max_speed",     -0.10, 0.10, 2.5), ".": ("max_speed",     +0.10, 0.10, 2.5),
    "o": ("arrive_radius", -0.50, 0.50, 10.0), "p": ("arrive_radius", +0.50, 0.50, 10.0),
}
NAV_TUNE_HELP = "[ ] kp · , . max_speed · o p arrive_radius"


def load_mission(path):
    """미션 파일 → [(종류, a, b, alt), ...]. 한 줄에 한 점, # 뒤는 주석.

        gps     <위도> <경도> [고도]
        offset  <앞 m> <오른쪽 m> [고도]

    고도를 빼거나 0 이하면 시작할 때 고도를 유지한다.
    offset 은 전부 **시작하는 순간의 위치·기수 기준**이다 (앞 점 기준이 아니다).
    """
    pts = []
    for no, line in enumerate(open(path, encoding="utf-8"), 1):
        f = line.split("#")[0].split()
        if not f:
            continue
        if f[0] not in ("gps", "offset") or len(f) not in (3, 4):
            raise ValueError(f"{path}:{no}: '{line.strip()}' — gps|offset <a> <b> [고도]")
        pts.append((f[0], float(f[1]), float(f[2]), float(f[3]) if len(f) == 4 else 0.0))
    if not pts:
        raise ValueError(f"{path}: 경유점이 없다")
    return pts


def resolve(pts, lat, lon, hdg):
    """offset 을 지금 위치·기수로 좌표로 바꾼다 → [(lat, lon, alt), ...]"""
    return [(a, b, alt) if kind == "gps" else mav.offset_to_gps(lat, lon, hdg, a, b) + (alt,)
            for kind, a, b, alt in pts]


def print_plan(pts, goals, here):
    print(f"\n  {'#':>2}  {'종류':<6} {'입력':<24} {'좌표':<25} {'고도':>5} {'구간':>8}")
    prev, total = here, 0.0
    for i, ((kind, a, b, alt), (lat, lon, _)) in enumerate(zip(pts, goals), 1):
        leg = mav.meters_between(prev[0], prev[1], lat, lon)
        total += leg
        print(f"  {i:>2}  {kind:<6} {a:>11.7g} {b:>11.7g}  {lat:.7f},{lon:.7f} "
              f"{'유지' if alt <= 0 else f'{alt:.1f}':>5} {leg:>7.1f}m")
        prev = (lat, lon)
    print(f"  {'':>2}  합계 {total:.1f}m")


def cmd_mission(args):
    """경유점 비행 — 파일을 읽어 nav_node 에 넘기고 진행을 본다.

    space 시작/중단 · q 종료(중단하고 나간다) · 그 밖의 키는 NAV_TUNE_HELP.

    ⚠ 이 도구도 /cmd_vel 을 직접 쏘지 않는다 — nav_node 가 낸다. tag_follow_node 와
      같이 켜 두지 말 것 (둘 다 /cmd_vel). 급할 때는 space.
    """
    try:
        pts = load_mission(args.file)
    except (OSError, ValueError) as e:
        print(f"✗ {e}")
        return 1

    link = connect()
    if link is None:
        return 1
    tuner = Tuner(link, NAV_NODE)
    par = tuner.read(("kp", "max_speed", "arrive_radius"))
    if not par:
        print(f"  ✗ {NAV_NODE} 의 파라미터를 못 읽는다 — nav_node 가 안 떠 있다")
        print("     ros2 launch rd_bridge bridge.launch.py        (nav:=true 가 기본)")
        return 1
    t0 = time.time()
    while "lat" not in link.tel and time.time() - t0 < 5.0:
        time.sleep(0.1)
    if "lat" not in link.tel:
        print("  ✗ /rd/fix 가 안 온다 — 좌표 없이는 시작 못 한다")
        return 1

    here = (link.tel["lat"], link.tel["lon"])
    print(f"\n{args.file} — 경유점 {len(pts)}개 (지금 위치·기수 {link.tel['yaw']:.0f}° 기준 미리보기)")
    print_plan(pts, resolve(pts, *here, link.tel["yaw"]), here)
    if any(k == "offset" for k, *_ in pts):
        print("  ⚠ offset 은 space 를 누르는 순간의 위치·기수로 다시 계산한다")

    wp = {"i": 0, "n": 0}
    cmd = {"t": 0.0, "v": (0.0, 0.0, 0.0)}
    link.create_subscription(Int32MultiArray, "nav/wp",
                             lambda m: wp.update(i=m.data[0], n=m.data[1]), 10)
    link.create_subscription(Twist, "cmd_vel", lambda m: cmd.update(
        t=time.time(), v=(m.linear.x, m.linear.y, m.linear.z)), 10)
    pub_mission = link.create_publisher(Float64MultiArray, "nav/mission", 10)
    pub_cancel = link.create_publisher(Empty, "nav/cancel", 10)
    time.sleep(0.6)      # 퍼블리셔가 nav_node 와 짝지어질 시간

    print("\n" + "=" * 72)
    print("  ⚠ space 를 누르면 기체가 **움직인다**. RC 모드 스위치 P · 스틱 놓고 5초")
    print("  ⚠ 조종권은 저절로 RC 로 안 돌아온다 — 급하면 S 모드 전환")
    print("=" * 72)
    print(f"  space 시작/중단 · q 종료\n  {NAV_TUNE_HELP}\n")

    goals, t_start = [], 0.0
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        nxt, quit_ = time.time(), False
        while not quit_:
            for k in read_keys(max(0.0, nxt - time.time())):
                if k == " " and wp["n"]:
                    pub_cancel.publish(Empty())
                elif k == " ":
                    t = link.tel
                    goals = resolve(pts, t["lat"], t["lon"], t["yaw"])
                    pub_mission.publish(Float64MultiArray(data=[v for g in goals for v in g]))
                    t_start = time.time()
                elif k == "q":
                    quit_ = True
                else:
                    tune(k, NAV_TUNE, par, tuner)
            now = time.time()
            if now < nxt:
                continue
            nxt += 0.2                                   # 5Hz 로만 그린다

            t = link.tel
            if wp["n"] and wp["n"] == len(goals):
                glat, glon, _ = goals[wp["i"] - 1]
                d = mav.meters_between(t["lat"], t["lon"], glat, glon)
                state = f"비행중 {wp['i']}/{wp['n']}] 목표까지 {d:5.1f}m"
            elif wp["n"]:      # 다른 터미널에서 준 미션 — 목표를 모른다
                state = f"비행중 {wp['i']}/{wp['n']}] (다른 곳에서 준 미션)"
            elif not t_start:
                state = "대기]"
            elif now - t_start < 1.5:          # /nav/wp 가 아직 안 왔다
                state = "시작 중…]"
            else:
                state = "끝남] 이유는 nav_node 로그"
            vx, vy, vz = cmd["v"] if now - cmd["t"] < 0.5 else (0.0, 0.0, 0.0)
            sys.stdout.write(
                f"\r[{state} · 고도{fmt(t.get('alt'), 'm')} · cmd {vx:+.2f} {vy:+.2f} {vz:+.2f}"
                f" · kp{par['kp']:.2f} spd{par['max_speed']:.1f} 반경{par['arrive_radius']:.1f}m\x1b[K")
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        for _ in range(2):                    # 무슨 일이 있어도 중단시키고 나간다
            pub_cancel.publish(Empty())
            time.sleep(0.3)
        print("\n■ /nav/cancel 송출 — nav_node 가 0 을 내고 link_node 가 1초 브레이크")
        print("⚠ RC 로 되돌리기: S 모드 전환 (또는 스틱 놓고 5초 → 앱 Safety 버튼)")
    return 0


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser(
        description="업링크 명령 도구 (ping · gimbal · drive · vel · follow · mission)")
    sub = ap.add_subparsers(dest="mode", required=True)

    p = sub.add_parser("ping", help="업링크가 열렸나 — 기체 안 움직임")
    p.add_argument("--secs", type=float, default=10.0)

    p = sub.add_parser("gimbal", help="짐벌 틸트 (DO_SET_SERVO)")
    p.add_argument("--angle", type=float, help="이 각도 하나만 (기본: 0 → -45 → -90 → 0)")
    p.add_argument("--yaw", action="store_true", help="pitch 대신 yaw (부호·동작 실기 미확인)")
    p.add_argument("--hold", type=float, default=3.5, help="단계마다 머무는 초 (2초 넘게)")

    p = sub.add_parser("drive", help="방향키 조종 (#84 20Hz)")
    p.add_argument("--max-vx", type=float, default=2.5, dest="max_vx", help="m/s (link_node 캡 2.5)")
    p.add_argument("--max-vy", type=float, default=2.5, dest="max_vy")
    p.add_argument("--max-vz", type=float, default=1.5, dest="max_vz", help="m/s, w=위 s=아래")
    p.add_argument("--tilt-step", type=float, default=15.0, dest="tilt_step",
                   help="r/f 한 번에 바뀌는 틸트 각도 (deg)")
    p.add_argument("--max-yaw", type=float, default=75.0, dest="max_yaw", help="deg/s")
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

    p = sub.add_parser("follow", help="태그 추종 켜고 끄기 + 실기 튜닝 (터미널 하나로)")
    p.add_argument("--tag-frame", default="tag0", dest="tag_frame",
                   help="tags.yaml 의 frames 와 같아야 한다")

    p = sub.add_parser("mission", help="경유점 비행 — 파일을 nav_node 에 넘긴다")
    p.add_argument("file", help="missions/*.txt — 줄마다 gps|offset <a> <b> [고도]")

    args = ap.parse_args()
    try:
        return {"ping": cmd_ping, "gimbal": cmd_gimbal, "drive": cmd_drive,
                "vel": cmd_vel, "follow": cmd_follow, "mission": cmd_mission}[args.mode](args)
    finally:
        if SPIN["thread"] is not None:
            SPIN["stop"] = True
            SPIN["thread"].join()
            SPIN["node"].destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
