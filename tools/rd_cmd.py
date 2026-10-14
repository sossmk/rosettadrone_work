#!/usr/bin/env python3
"""업링크 명령 도구 — 토픽으로 link_node 에 말한다 (UDP 를 직접 잡지 않는다).

    ros2 launch rd_bridge bridge.launch.py nav:=false     # 먼저 (다른 터미널)

    python3 tools/rd_cmd.py ping                     # 업링크가 열렸나 (기체 안 움직임)
    python3 tools/rd_cmd.py gimbal                   # 짐벌 0 → -45 → -90 → 0
    python3 tools/rd_cmd.py gimbal --angle -90       #    한 각도만
    python3 tools/rd_cmd.py gimbal --yaw             #    좌우 0 → +30 → 0 → -30 → 0
    python3 tools/rd_cmd.py takeoff 5                # 5m 로 이륙
    python3 tools/rd_cmd.py drive                    # 방향키 조종 (--dry-run: 전송 없이)
    python3 tools/rd_cmd.py follow                   # 태그 추종 시작·정지·튜닝
    python3 tools/rd_cmd.py usv                      # USV GPS 추종 시작·정지·튜닝
    python3 tools/rd_cmd.py mission missions/square10.txt   # 경유점 비행

⚠ 기체가 움직이기 전에
  · RC 모드 스위치는 **P**. 아니면 명령이 에러 없이 버려진다
  · **조종권은 저절로 RC 로 안 돌아온다.** 급하면 S 모드. 아니면 도구 종료 → 스틱 놓고 5초
    → 앱 Safety(자물쇠) 버튼
  · 앱의 ACK 는 무조건 ACCEPTED 다. 짐벌은 눈으로, 이동은 좌표로 판정한다
  · /gcs/daiv/cmd_vel 은 앱 규약: vx>0 앞 · vy>0 오른쪽 · vz>0 위 · yaw>0 시계.
    link_node 가 수평 2.5 · 수직 1.5 m/s · 75°/s 로 한 번 더 깎는다
"""
import argparse, math, os, select, sys, termios, threading, time, tty

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Empty, Float64, Float64MultiArray, Int32, Int32MultiArray
from tf2_msgs.msg import TFMessage

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.join(HERE, "../ros2/src/rd_bridge/rd_bridge")]
import mav   # 명령 번호 · 거리·방위 계산
from mission_node import CTRL_NO, NAMES as SORTIE_MODES   # 모드 표는 시퀀서의 것 하나만

SPIN = {"stop": False, "thread": None, "node": None}
CMD_REQUEST_AUTOPILOT_CAPABILITIES = 520
CH_PITCH, CH_YAW = 9, 8
RC_BACK = "⚠ RC 로 되돌리기: S 모드 전환 (또는 스틱 놓고 5초 → 앱 Safety 버튼)"


class Bridge(Node):
    """link_node 와 이야기하는 창구."""

    def __init__(self):
        super().__init__("rd_cmd")
        self.pub_vel = self.create_publisher(Twist, "gcs/daiv/cmd_vel", 10)
        self.pub_cmd = self.create_publisher(Float64MultiArray, "gcs/daiv/rd/command", 10)
        self.create_subscription(NavSatFix, "gcs/daiv/rd/fix", self.on_fix, 10)
        self.create_subscription(Float64, "gcs/daiv/rd/heading", self.on_hdg, 10)
        self.create_subscription(Int32MultiArray, "gcs/daiv/rd/ack", self.on_ack, 50)
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
    """브리지가 떠 있는지 /gcs/daiv/rd/heading 으로 확인한다. spin 은 뒤 스레드가 돌린다."""
    rclpy.init()
    b = Bridge()

    def spin():
        while not SPIN["stop"]:
            rclpy.spin_once(b, timeout_sec=0.1)
    SPIN["node"], SPIN["thread"] = b, threading.Thread(target=spin, daemon=True)
    SPIN["thread"].start()
    print("브리지(link_node) 확인 중… (/gcs/daiv/rd/heading)")
    t0 = time.time()
    while time.time() - t0 < 6.0 and "yaw" not in b.tel:
        time.sleep(0.1)
    if "yaw" not in b.tel:
        print("  ✗ /gcs/daiv/rd/heading 이 안 온다. 먼저: ros2 launch rd_bridge bridge.launch.py nav:=false")
        return None
    time.sleep(0.5)      # 퍼블리셔가 짝지어질 시간
    print(f"  ✓ 기수 {b.tel['yaw']:.0f}° · 고도 {fmt(b.tel.get('alt'), 'm')}")
    return b


def fmt(v, unit="", nd=1):
    return "?" if v is None else f"{v:.{nd}f}{unit}"


def banner(what, keys):
    print("\n" + "=" * 72)
    print(f"  ⚠ {what}. RC 모드 스위치 P · 스틱 놓고 5초")
    print("  ⚠ 조종권은 저절로 RC 로 안 돌아온다 — 급하면 S 모드 전환")
    print("=" * 72)
    print(f"  {keys}\n")


# --------------------------------------------------------------- 키 루프
ARROW = {"A": "up", "B": "down", "C": "right", "D": "left"}


def read_keys(timeout):
    """timeout 초 동안 눌린 키들. 화살표는 ESC [ A~D 로 온다."""
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


def key_loop(on_key, tick, hz):
    """키마다 on_key(k), hz 로 tick() 을 부르고 돌려준 한 줄을 다시 그린다. q·Ctrl+C 로 끝."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        nxt = time.time()
        while True:
            for k in read_keys(max(0.0, nxt - time.time())):
                if k == "q":
                    return
                on_key(k)
            if time.time() >= nxt:
                nxt += 1.0 / hz
                sys.stdout.write("\r" + tick() + "\x1b[K")
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ------------------------------------------------------------------ ping
def cmd_ping(args):
    """REQUEST_AUTOPILOT_CAPABILITIES(520) 를 1초마다 보내고 ACK 를 기다린다."""
    link = connect()
    if link is None:
        return 1
    print(f"\n명령 520 송신 · {args.secs:.0f}초 대기")
    t0, sent = time.time(), 0
    while time.time() - t0 < args.secs:
        if time.time() - t0 > sent:
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
    """DO_SET_SERVO(183) — PWM = 1500 + 각도 × 5.5. 짐벌은 이 명령으로만 움직인다.

    채널 9 = pitch (절대각). 채널 8 = yaw (**상대각** — 앱이 기억한 직전 값과의 차이만큼
    돈다. 기준은 앱 시작 때 방향. yaw 의 부호와 실제 동작은 실기 미확인).
    """
    link = connect()
    if link is None:
        return 1
    ch = CH_YAW if args.yaw else CH_PITCH
    if args.angle is not None:
        seq = [("yaw" if args.yaw else "pitch", args.angle)]
    elif args.yaw:
        # 매번 0 을 거친다 — 0 을 보내면 시작 방향으로 온다
        seq = [("yaw +30°", 30.0), ("yaw 0° 복귀", 0.0), ("yaw -30°", -30.0), ("yaw 0° 복귀", 0.0)]
    else:
        seq = [("수평 0°", 0.0), ("아래 -45°", -45.0), ("수직 -90°", -90.0), ("수평 복귀", 0.0)]

    print("\n짐벌을 움직인다 — **카메라를 눈으로 보세요**")
    if args.yaw:
        print("  ⚠ 볼 것: +30° 에서 카메라가 기수 기준 **오른쪽(위에서 봐서 시계)** 인가 왼쪽인가")
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
                print("      → TEMPORARILY_REJECTED = 짐벌이 안 붙었다")
            ok += res == 0
        # 앱은 2초에 걸쳐 돌린다. 덜 쉬면 다음 명령이 회전을 덮어쓴다
        time.sleep(args.hold)

    print(f"\n{ok}/{len(seq)} ACCEPTED — ⚠ 성공 여부는 **카메라가 실제로 움직였는지** 로만 갈린다")
    return 0 if ok == len(seq) else 1


# --------------------------------------------------------------- takeoff
def cmd_takeoff(args):
    """NAV_TAKEOFF(22) — 이 고도(상대고도 m)까지 올라간다. 9/9 SIM 에서 5m 로 확인."""
    link = connect()
    if link is None:
        return 1
    print(f"\n⚠ 이륙 — 목표 {args.alt:.1f}m. RC 모드 스위치 P. 3초 뒤 (Ctrl+C 로 중단)")
    time.sleep(3.0)
    link.command(mav.CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, args.alt)
    res, _ = link.wait_ack(mav.CMD_NAV_TAKEOFF, secs=10.0)
    print(f"  ACK: {mav.MAV_RESULT.get(res, res) if res is not None else '없음 ✗'}")
    t0 = time.time()
    while time.time() - t0 < 25 and (link.tel.get("alt") or 0) < args.alt * 0.8:
        time.sleep(0.1)
    print(f"  현재 고도 {fmt(link.tel.get('alt'), 'm')}")
    return 0


# ----------------------------------------------------------------- drive
def cmd_drive(args):
    """방향키 조종 (20Hz). ↑↓ 앞뒤 · ←→ 좌우 · w/s 위아래 · a/d 회전 · r/f 틸트 · space 정지.

    터미널엔 keyup 이 없어서 축마다 0.3초 새 입력이 없으면 0 으로 돌린다
    (누르고 있으면 오토리핏이 채운다). 틸트는 목표가 바뀔 때만, 0.3초에 한 번 이하로 보낸다.
    """
    link = None
    if args.dry_run:
        print("[dry-run] 전송 없이 키 입력만 본다.")
    else:
        link = connect()
        if link is None:
            return 1
        banner("기체가 방향키대로 **움직인다**", "↑↓←→ 이동 · w/s 위아래 · a/d 회전 · r/f 틸트 · space 정지 · q 종료")

    keymap = {"up": ("vx", args.max_vx), "down": ("vx", -args.max_vx),
              "right": ("vy", args.max_vy), "left": ("vy", -args.max_vy),
              "w": ("vz", args.max_vz), "s": ("vz", -args.max_vz),
              "d": ("yaw", args.max_yaw), "a": ("yaw", -args.max_yaw)}
    cur = {"vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw": 0.0}
    last = dict(cur)
    st = {"sent": 0, "tilt": 0.0, "tilt_sent": 0.0, "t_tilt": 0.0}

    def on_key(k):
        if k in keymap:
            axis, v = keymap[k]
            cur[axis], last[axis] = v, time.time()
        elif k in ("r", "f"):
            st["tilt"] = max(-90.0, min(0.0, st["tilt"] + (args.tilt_step if k == "r" else -args.tilt_step)))
        elif k == " ":
            for a in cur:
                cur[a] = last[a] = 0.0

    def tick():
        now = time.time()
        for a in cur:
            if now - last[a] > 0.3:
                cur[a] = 0.0
        if link:
            link.vel(cur["vx"], cur["vy"], cur["vz"], cur["yaw"])
            st["sent"] += 1
            if st["tilt"] != st["tilt_sent"] and now - st["t_tilt"] >= 0.3:
                link.command(mav.CMD_DO_SET_SERVO, float(CH_PITCH), 1500.0 + st["tilt"] * 5.5)
                st["tilt_sent"], st["t_tilt"] = st["tilt"], now
        t = link.tel if link else {}
        return (f"vx={cur['vx']:+.2f} vy={cur['vy']:+.2f} vz={cur['vz']:+.2f} yaw={cur['yaw']:+.1f}°/s"
                f" 틸트{st['tilt']:+.0f}°  보냄{st['sent']} ACK{link.acks84 if link else 0}"
                f"  고도{fmt(t.get('alt'), 'm')} 기수{fmt(t.get('yaw'), '°', 0)}")

    try:
        key_loop(on_key, tick, hz=20)
    finally:
        if link:
            print("\n… 정지 명령 송출(1초)")
            for _ in range(20):
                link.vel(0, 0, 0, 0)
                time.sleep(0.05)
            print(f"총 송신 {st['sent']}회 · ACK(84) {link.acks84}회\n{RC_BACK}")
    return 0


# ------------------------------------------------------------- 파라미터
def wait_future(fut, secs=2.0):
    """spin_until_future_complete 는 못 쓴다 — connect() 의 spin 스레드와 겹친다."""
    t0 = time.time()
    while not fut.done() and time.time() - t0 < secs:
        time.sleep(0.01)
    return fut.result() if fut.done() else None


class Tuner:
    """다른 노드의 파라미터를 날면서 읽고 쓴다."""

    def __init__(self, node, target):
        self.cli_get = node.create_client(GetParameters, f"{target}/get_parameters")
        self.cli_set = node.create_client(SetParameters, f"{target}/set_parameters")
        self.ok = self.cli_get.wait_for_service(timeout_sec=4.0)

    def read(self, names):
        if not self.ok:
            return {}
        r = wait_future(self.cli_get.call_async(GetParameters.Request(names=list(names))))
        if r is None or len(r.values) != len(names):
            return {}
        return {n: parameter_value_to_python(v) for n, v in zip(names, r.values)}

    def write(self, name, value):
        # 타입은 value 로 정해진다 (bool·int·float) — 노드가 선언한 타입과 같아야 받아 준다
        wait_future(self.cli_set.call_async(
            SetParameters.Request(parameters=[Parameter(name, value=value).to_parameter_msg()])))


def tune(k, table, par, tuner):
    """튜닝 키 → (파라미터, 증감, 하한, 상한). table 에 없는 키는 무시."""
    if k in table:
        name, step, lo, hi = table[k]
        par[name] = round(min(hi, max(lo, par[name] + step)), 2)
        tuner.write(name, par[name])


def arm_state(st, cmd, now):
    """추종 도구의 상태 한 마디 — 켰나, 명령이 흐르나."""
    if not st["armed"]:
        return "대기"
    if now - cmd["t"] < 1.5:
        return "추종중"
    # 켠 직후는 아직 첫 명령이 안 왔을 뿐이다
    return "시작 중…" if now - st["t_arm"] < 1.5 else "명령끊김"


def send_twice(pub, msg):
    """나갈 때 정지 신호 — 하나가 유실돼도 서게 두 번."""
    for _ in range(2):
        pub.publish(msg)
        time.sleep(0.3)


# ---------------------------------------------------------------- follow
FOLLOW_NODE = "/rd_tag_follow"
FOLLOW_TUNE = {
    "[": ("kp",           -0.05, 0.05,  3.0), "]": ("kp",           +0.05, 0.05,  3.0),
    ",": ("max_speed",    -0.10, 0.10,  2.5), ".": ("max_speed",    +0.10, 0.10,  2.5),
    ";": ("min_speed",    -0.05, 0.00,  1.0), "'": ("min_speed",    +0.05, 0.00,  1.0),
    "-": ("hold_alt",     -0.50, 0.50, 30.0), "=": ("hold_alt",     +0.50, 0.50, 30.0),
    "o": ("deadband",     -0.10, 0.10,  3.0), "p": ("deadband",     +0.10, 0.10,  3.0),
    "n": ("yaw_deadband", -1.00, 1.00, 45.0), "m": ("yaw_deadband", +1.00, 1.00, 45.0),
}
FOLLOW_KEYS = ("space 시작/정지 · q 종료 · y yaw 켜고끄기\n  [ ] kp · , . max_speed · ; ' min_speed"
               " · - = hold_alt · o p deadband · n m yaw_deadband")


def cmd_follow(args):
    """태그 추종 — /gcs/daiv/tag/follow 를 토글하고 파라미터를 고친다.

    ⚠ /gcs/daiv/cmd_vel 은 추종 노드가 낸다. 이 도구는 Ctrl+C 로 기체를 못 세운다 — 급할 때는 space.
    """
    link = connect()
    if link is None:
        return 1
    tuner = Tuner(link, FOLLOW_NODE)
    par = tuner.read(("kp", "min_speed", "max_speed", "hold_alt", "deadband", "yaw_deadband",
                      "yaw_enable", "sign_fwd", "sign_right"))
    if not par:
        print(f"  ✗ {FOLLOW_NODE} 가 안 떠 있다 — bridge.launch.py 에 tag:=true follow:=true nav:=false")
        return 1

    tag = {"t": 0.0, "x": 0.0, "y": 0.0, "z": 0.0}
    cmd = {"t": 0.0, "v": (0.0, 0.0, 0.0, 0.0)}
    st = {"armed": False, "t_arm": 0.0}

    def on_tf(msg):
        for tf in msg.transforms:
            if tf.child_frame_id == args.tag_frame:
                p = tf.transform.translation
                tag.update(t=time.time(), x=p.x, y=p.y, z=p.z)

    pub = link.create_publisher(Bool, "gcs/daiv/tag/follow", 10)
    link.create_subscription(TFMessage, "/tf", on_tf, 50)
    link.create_subscription(Twist, "gcs/daiv/cmd_vel", lambda m: cmd.update(
        t=time.time(), v=(m.linear.x, m.linear.y, m.linear.z, math.degrees(m.angular.z))), 10)
    time.sleep(0.6)      # 퍼블리셔가 짝지어질 시간 — 첫 토글을 잃지 않게
    banner("space 를 누르면 기체가 태그를 따라 **움직인다**", FOLLOW_KEYS)

    def on_key(k):
        if k == " ":
            st["armed"], st["t_arm"] = not st["armed"], time.time()
            pub.publish(Bool(data=st["armed"]))
        elif k == "y":
            par["yaw_enable"] = not par["yaw_enable"]
            tuner.write("yaw_enable", par["yaw_enable"])
        else:
            tune(k, FOLLOW_TUNE, par, tuner)

    def tick():
        now = time.time()
        if tag["t"] == 0.0:
            see = "태그 본 적 없음"
        elif now - tag["t"] > 1.0:
            see = f"태그 {now - tag['t']:4.1f}초 전"
        else:
            see = (f"앞{par['sign_fwd'] * tag['y']:+.2f} "
                   f"오{par['sign_right'] * tag['x']:+.2f} z{tag['z']:4.2f}m")
        state = arm_state(st, cmd, now)
        vx, vy, vz, wz = cmd["v"]
        return (f"[{state}] {see} · cmd {vx:+.2f} {vy:+.2f} {vz:+.2f} {wz:+3.0f}°/s"
                f" · kp{par['kp']:.2f} spd{par['min_speed']:.2f}~{par['max_speed']:.2f}"
                f" alt{par['hold_alt']:.1f} db{par['deadband']:.1f}/{par['yaw_deadband']:.0f}°"
                f" yaw{'켬' if par['yaw_enable'] else '끔'} · 고도{fmt(link.tel.get('alt'), 'm')}")

    try:
        key_loop(on_key, tick, hz=5)
    finally:
        send_twice(pub, Bool(data=False))
        print(f"\n■ /gcs/daiv/tag/follow false 송출 — 노드가 0 을 내고 link_node 가 1초 브레이크\n{RC_BACK}")
    return 0


# ------------------------------------------------------------------- usv
USV_NODE = "/rd_usv_follow"
USV_TUNE = {
    "[": ("kp",        -0.05, 0.05,  3.0), "]": ("kp",        +0.05, 0.05,  3.0),
    # 상한은 기본값(4.5)보다 커야 한다 — 작으면 키 한 번에 상한으로 뚝 떨어진다
    ",": ("max_speed", -0.10, 0.10,  5.0), ".": ("max_speed", +0.10, 0.10,  5.0),
}
USV_KEYS = ("space 시작/정지 · q 종료 · m 이동 켜고끄기 · g 짐벌 겨눔 켜고끄기\n"
            "  1~9 대형 (USV 기준 앞·오른쪽·고도) · [ ] kp · , . max_speed")


def usv_forms(par):
    """formations 배열 → [(앞, 오른쪽, 고도), ...]. 3 의 배수가 아니면 남는 꼬리는 버린다."""
    a = list(par["formations"])
    return [tuple(a[i:i + 3]) for i in range(0, len(a) - 2, 3)]


def usv_menu(forms):
    return "  ".join(f"{n}: 앞{f:+.0f} 오{r:+.0f} 고{a:.0f}m" for n, (f, r, a) in enumerate(forms, 1))


def set_formation(n, par, tuner):
    """대형을 n 번으로 — usv · sortie 둘 다 쓴다. par 의 formations·formation 을 고친다.

    **누를 때마다 formations 를 다시 읽는다.** 노드는 매 tick 다시 읽으니 날면서도
    대형이 늘어난다 — 시작 때 읽은 개수로 막으면 키가 **조용히 삼켜진다.**
    9/22 실험 1-B 가 그렇게 날아갔다 (T5 의 formations 를 T4 보다 늦게 넣었다)
    """
    got = tuner.read(("formations",))
    if got:
        par["formations"] = got["formations"]
    forms = usv_forms(par)
    if 1 <= n <= len(forms):
        par["formation"] = n
        tuner.write("formation", n)
    else:
        print(f"\n  ✗ 대형 {n} 번이 없다 — 지금 {len(forms)}개다. "
              f"T5 에서 formations 를 넣었나?\n  {usv_menu(forms)}")
    return forms


def cmd_usv(args):
    """USV 추종 — /gcs/daiv/usv/follow 를 토글하고 파라미터를 고친다. 급할 때는 space (follow 와 같다)."""
    link = connect()
    if link is None:
        return 1
    tuner = Tuner(link, USV_NODE)
    par = tuner.read(("kp", "max_speed", "formation", "formations", "move", "aim"))
    if not par:
        print(f"  ✗ {USV_NODE} 가 안 떠 있다 — bridge.launch.py 에 usv:=true nav:=false")
        return 1

    usv = {"t": 0.0, "lat": None, "lon": None}
    cmd = {"t": 0.0, "v": (0.0, 0.0, 0.0, 0.0)}
    st = {"armed": False, "t_arm": 0.0}
    pub = link.create_publisher(Bool, "gcs/daiv/usv/follow", 10)
    link.create_subscription(NavSatFix, "usv/fix", lambda m: usv.update(
        t=time.time(), lat=m.latitude, lon=m.longitude), 10)
    link.create_subscription(Twist, "gcs/daiv/cmd_vel", lambda m: cmd.update(
        t=time.time(), v=(m.linear.x, m.linear.y, m.linear.z, math.degrees(m.angular.z))), 10)
    time.sleep(0.6)      # 퍼블리셔가 짝지어질 시간 — 첫 토글을 잃지 않게
    forms = usv_forms(par)
    banner("space 를 누르면 기체가 USV 를 따라 **움직인다**",
           USV_KEYS + "\n  " + usv_menu(forms))

    def on_key(k):
        nonlocal forms
        if k == " ":
            st["armed"], st["t_arm"] = not st["armed"], time.time()
            pub.publish(Bool(data=st["armed"]))
        elif k in ("m", "g"):
            name = "move" if k == "m" else "aim"
            par[name] = not par[name]
            tuner.write(name, par[name])
        elif k.isdigit():
            forms = set_formation(int(k), par, tuner)
        else:
            tune(k, USV_TUNE, par, tuner)

    def tick():
        now, tel = time.time(), link.tel
        if usv["t"] == 0.0:
            see = "USV 좌표 없음"
        elif "lat" not in tel:
            see = "우리 좌표 없음"
        else:
            d = mav.meters_between(tel["lat"], tel["lon"], usv["lat"], usv["lon"])
            see = f"USV {d:5.1f}m" + (f" ({now - usv['t']:.0f}초 전)" if now - usv["t"] > 2 else "")
        state = arm_state(st, cmd, now)
        vx, vy, vz, wz = cmd["v"]
        return (f"[{state}] {see} · cmd {vx:+.2f} {vy:+.2f} {vz:+.2f} {wz:+3.0f}°/s"
                f" · 대형{par['formation']} kp{par['kp']:.2f} spd{par['max_speed']:.1f}"
                f" 이동{'켬' if par['move'] else '끔'} 짐벌{'켬' if par['aim'] else '끔'}"
                f" · 고도{fmt(tel.get('alt'), 'm')}")

    try:
        key_loop(on_key, tick, hz=5)
    finally:
        send_twice(pub, Bool(data=False))
        print(f"\n■ /gcs/daiv/usv/follow false 송출 — 노드가 0 을 내고 link_node 가 1초 브레이크\n{RC_BACK}")
    return 0


# -------------------------------------------------------------- sortie
SORTIE_NODE = "/rd_mission"
SORTIE_TUNE = {
    "[": ("volt_land", -0.10, 12.0, 17.0), "]": ("volt_land", +0.10, 12.0, 17.0),
    ",": ("cruise",    -0.50,  0.5,  2.5), ".": ("cruise",    +0.50,  0.5,  2.5),
}
SORTIE_CTRL = {n: c or "없음" for c, n in CTRL_NO.items()}
# 대형 키 — 숫자는 이미 모드 건너뛰기라 글자 줄(a s d f …)을 1·2·3·4… 번 대형에 쓴다
SORTIE_FORM_KEYS = "asdfghjkl"
SORTIE_KEYS = ("space 시작 · r 복귀 · x 중단 · q 종료\n"
               "  1~6 그 임무만 따로 (1 이륙 · 2 USV추종 · 3 복귀 · 4 태그정렬 · 5 착륙 · 6 끝)\n"
               "  a s d f … 대형 1·2·3·4… (USV추종 중이면 바로 바뀐다) · [ ] volt_land · , . cruise")


def cmd_sortie(args):
    """임무 시퀀서 — 시작·복귀·중단을 누르고, 숫자키로 개별 임무만 따로 시험한다.

    ⚠ 이 도구도 /gcs/daiv/cmd_vel 을 직접 쏘지 않는다. 급하면 x (중단) 또는 rd_cmd.py usv 의 space.
    """
    link = connect()
    if link is None:
        return 1
    tuner = Tuner(link, SORTIE_NODE)
    par = tuner.read(("volt_land", "cruise", "takeoff_alt", "start_mode"))
    if not par:
        print(f"  ✗ {SORTIE_NODE} 가 안 떠 있다 — bridge.launch.py 에 mission:=true")
        return 1

    # 대형은 rd_usv_follow 의 파라미터다 — 시퀀서는 대형을 모른다. 못 읽어도 임무는 된다
    usv_tuner = Tuner(link, USV_NODE)
    upar = usv_tuner.read(("formation", "formations"))
    if not upar:
        print(f"  ⚠ {USV_NODE} 파라미터를 못 읽었다 — 대형 키(a s d …)는 안 먹는다")

    st = {"t": 0.0, "mode": 0, "ctrl": 0, "secs": 0}
    cmd = {"t": 0.0}
    pub_start = link.create_publisher(Empty, "gcs/daiv/mission/start", 10)
    pub_ret = link.create_publisher(Empty, "gcs/daiv/mission/return", 10)
    pub_abort = link.create_publisher(Empty, "gcs/daiv/mission/abort", 10)
    pub_goto = link.create_publisher(Int32, "gcs/daiv/mission/goto", 10)
    link.create_subscription(Int32MultiArray, "gcs/daiv/mission/state", lambda m: st.update(
        t=time.time(), mode=m.data[0], ctrl=m.data[1],
        secs=m.data[2] if len(m.data) > 2 else 0), 10)
    link.create_subscription(Twist, "gcs/daiv/cmd_vel", lambda _: cmd.update(t=time.time()), 10)
    time.sleep(0.6)      # 퍼블리셔가 짝지어질 시간 — 첫 토글을 잃지 않게
    banner("space 를 누르면 기체가 **이륙해서 임무를 혼자 진행한다**",
           SORTIE_KEYS + ("\n  " + usv_menu(usv_forms(upar)) if upar else ""))

    def on_key(k):
        if k == " ":
            pub_start.publish(Empty())
        elif k == "r":
            pub_ret.publish(Empty())
        elif k == "x":
            send_twice(pub_abort, Empty())
        elif k.isdigit() and k != "0":
            n = int(k)
            if n in SORTIE_MODES:
                # 개별 임무만 — 노드의 can_jump() 가 땅/하늘·station 을 보고 거절할 수 있다
                pub_goto.publish(Int32(data=n))
                print(f"\n  → {SORTIE_MODES[n]} 로 건너뛰기 요청 (노드 로그에서 수락 여부를 본다)")
            else:
                print(f"\n  ✗ 모드 {n} 은 없다 — 1~6")
        elif k in SORTIE_FORM_KEYS:
            if upar:
                set_formation(SORTIE_FORM_KEYS.index(k) + 1, upar, usv_tuner)
        else:
            tune(k, SORTIE_TUNE, par, tuner)

    def tick():
        now, tel = time.time(), link.tel
        if st["t"] == 0.0:
            state = "상태 없음"
        elif now - st["t"] > 3.0:
            state = f"시퀀서 끊김 {now - st['t']:.0f}초"
        else:
            state = f"{SORTIE_MODES.get(st['mode'], '?')} {st['secs']:3d}s"
        drive = "조용" if now - cmd["t"] > 1.5 else "주행중"
        return (f"[{state}] 주인 {SORTIE_CTRL.get(st['ctrl'], '?')} · /gcs/daiv/cmd_vel {drive}"
                f" · 고도{fmt(tel.get('alt'), 'm')} 전압{fmt(tel.get('volt'), 'V', 2)}"
                f" · volt_land {par['volt_land']:.1f} cruise {par['cruise']:.1f}"
                f" · 이륙고도 {par['takeoff_alt']:.1f}m"
                + (f" · start_mode {par['start_mode']}" if par.get("start_mode") else "")
                + (f" · 대형{upar['formation']}" if upar else ""))

    try:
        key_loop(on_key, tick, hz=5)
    finally:
        send_twice(pub_abort, Empty())
        print(f"\n■ /gcs/daiv/mission/abort 송출 — 시퀀서가 제어기를 전부 해제한다\n{RC_BACK}")
    return 0


# --------------------------------------------------------------- mission
NAV_NODE = "/rd_nav"
NAV_TUNE = {
    "[": ("kp",            -0.05, 0.05,  3.0), "]": ("kp",            +0.05, 0.05,  3.0),
    ",": ("max_speed",     -0.10, 0.10,  5.0), ".": ("max_speed",     +0.10, 0.10,  5.0),
    "o": ("arrive_radius", -0.50, 0.50, 10.0), "p": ("arrive_radius", +0.50, 0.50, 10.0),
}
NAV_KEYS = "space 시작/중단 · q 종료\n  [ ] kp · , . max_speed · o p arrive_radius"


def load_mission(path):
    """미션 파일 → [(종류, a, b, alt), ...]. 한 줄에 한 점, # 뒤는 주석.

        gps     <위도> <경도> [고도]
        offset  <앞 m> <오른쪽 m> [고도]      # 시작 순간의 위치·기수 기준

    고도를 빼거나 0 이하면 시작할 때 고도를 유지한다.
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
    """경유점 비행 — 파일을 nav_node 에 넘기고 진행을 본다.

    ⚠ /gcs/daiv/cmd_vel 은 nav_node 가 낸다. 추종 노드와 같이 켜지 않는다. 급할 때는 space.
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
        print(f"  ✗ {NAV_NODE} 가 안 떠 있다 — bridge.launch.py 를 nav:=true(기본)로")
        return 1
    t0 = time.time()
    while "lat" not in link.tel and time.time() - t0 < 5.0:
        time.sleep(0.1)
    if "lat" not in link.tel:
        print("  ✗ /gcs/daiv/rd/fix 가 안 온다 — 좌표 없이는 시작 못 한다")
        return 1

    here = (link.tel["lat"], link.tel["lon"])
    print(f"\n{args.file} — 경유점 {len(pts)}개 (지금 위치·기수 {link.tel['yaw']:.0f}° 기준 미리보기)")
    print_plan(pts, resolve(pts, *here, link.tel["yaw"]), here)
    if any(k == "offset" for k, *_ in pts):
        print("  ⚠ offset 은 space 를 누르는 순간의 위치·기수로 다시 계산한다")

    wp = {"i": 0, "n": 0}
    cmd = {"t": 0.0, "v": (0.0, 0.0, 0.0)}
    st = {"goals": [], "t_start": 0.0}
    link.create_subscription(Int32MultiArray, "gcs/daiv/nav/wp",
                             lambda m: wp.update(i=m.data[0], n=m.data[1]), 10)
    link.create_subscription(Twist, "gcs/daiv/cmd_vel", lambda m: cmd.update(
        t=time.time(), v=(m.linear.x, m.linear.y, m.linear.z)), 10)
    pub_mission = link.create_publisher(Float64MultiArray, "gcs/daiv/nav/mission", 10)
    pub_cancel = link.create_publisher(Empty, "gcs/daiv/nav/cancel", 10)
    time.sleep(0.6)      # 퍼블리셔가 짝지어질 시간
    banner("space 를 누르면 기체가 **움직인다**", NAV_KEYS)

    def on_key(k):
        if k == " " and wp["n"]:
            pub_cancel.publish(Empty())
        elif k == " ":
            t = link.tel
            st["goals"] = resolve(pts, t["lat"], t["lon"], t["yaw"])
            pub_mission.publish(Float64MultiArray(data=[v for g in st["goals"] for v in g]))
            st["t_start"] = time.time()
        else:
            tune(k, NAV_TUNE, par, tuner)

    def tick():
        now, t, goals = time.time(), link.tel, st["goals"]
        if wp["n"] and wp["n"] == len(goals):
            glat, glon, _ = goals[wp["i"] - 1]
            d = mav.meters_between(t["lat"], t["lon"], glat, glon)
            state = f"비행중 {wp['i']}/{wp['n']}] 목표까지 {d:5.1f}m"
        elif wp["n"]:      # 다른 터미널에서 준 미션 — 목표를 모른다
            state = f"비행중 {wp['i']}/{wp['n']}] (다른 곳에서 준 미션)"
        elif not st["t_start"]:
            state = "대기]"
        elif now - st["t_start"] < 1.5:      # /gcs/daiv/nav/wp 가 아직 안 왔다
            state = "시작 중…]"
        else:
            state = "끝남] 이유는 nav_node 로그"
        vx, vy, vz = cmd["v"] if now - cmd["t"] < 0.5 else (0.0, 0.0, 0.0)
        return (f"[{state} · 고도{fmt(t.get('alt'), 'm')} · cmd {vx:+.2f} {vy:+.2f} {vz:+.2f}"
                f" · kp{par['kp']:.2f} spd{par['max_speed']:.1f} 반경{par['arrive_radius']:.1f}m")

    try:
        key_loop(on_key, tick, hz=5)
    finally:
        send_twice(pub_cancel, Empty())
        print(f"\n■ /gcs/daiv/nav/cancel 송출 — nav_node 가 0 을 내고 link_node 가 1초 브레이크\n{RC_BACK}")
    return 0


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser(description="업링크 명령 도구")
    sub = ap.add_subparsers(dest="mode", required=True)

    p = sub.add_parser("ping", help="업링크가 열렸나 — 기체 안 움직임")
    p.add_argument("--secs", type=float, default=10.0)

    p = sub.add_parser("gimbal", help="짐벌 틸트 (DO_SET_SERVO)")
    p.add_argument("--angle", type=float, help="이 각도 하나만 (기본: 0 → -45 → -90 → 0)")
    p.add_argument("--yaw", action="store_true", help="pitch 대신 yaw")
    p.add_argument("--hold", type=float, default=3.5, help="단계마다 머무는 초 (2초 넘게)")

    p = sub.add_parser("takeoff", help="이륙 (NAV_TAKEOFF)")
    p.add_argument("alt", type=float, help="목표 상대고도 m")

    p = sub.add_parser("drive", help="방향키 조종 (#84 20Hz)")
    p.add_argument("--max-vx", type=float, default=2.5, dest="max_vx", help="m/s")
    p.add_argument("--max-vy", type=float, default=2.5, dest="max_vy")
    p.add_argument("--max-vz", type=float, default=1.5, dest="max_vz", help="m/s, w=위 s=아래")
    p.add_argument("--tilt-step", type=float, default=15.0, dest="tilt_step", help="r/f 한 번의 틸트 (deg)")
    p.add_argument("--max-yaw", type=float, default=75.0, dest="max_yaw", help="deg/s")
    p.add_argument("--dry-run", action="store_true", dest="dry_run", help="전송 없이 키 반응만")

    p = sub.add_parser("follow", help="태그 추종 시작·정지·튜닝")
    p.add_argument("--tag-frame", default="tag0", dest="tag_frame", help="tags.yaml 의 frames 와 같게")

    sub.add_parser("usv", help="USV GPS 추종 시작·정지·튜닝")

    sub.add_parser("sortie", help="임무 시퀀서 — 이륙→추종→복귀→착륙. 숫자키로 개별 임무만")

    p = sub.add_parser("mission", help="경유점 비행 — 파일을 nav_node 에 넘긴다")
    p.add_argument("file", help="missions/*.txt — 줄마다 gps|offset <a> <b> [고도]")

    args = ap.parse_args()
    try:
        return {"ping": cmd_ping, "gimbal": cmd_gimbal, "takeoff": cmd_takeoff, "drive": cmd_drive,
                "follow": cmd_follow, "usv": cmd_usv, "sortie": cmd_sortie,
                "mission": cmd_mission}[args.mode](args)
    finally:
        if SPIN["thread"] is not None:
            SPIN["stop"] = True
            SPIN["thread"].join()
            SPIN["node"].destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
