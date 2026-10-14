#!/usr/bin/env python3
"""임무 시퀀서 지상 드라이런 — 기체도 link_node 도 없이 시퀀스 전체를 걷게 해 본다.

    # T1 — 제어기 넷을 띄운다 (link_node 없음 = UDP 가 이 PC 를 안 떠난다)
    for n in nav_node usv_follow_node tag_follow_node mission_node; do
        ros2 run rd_bridge $n & done
    # T2
    python3 tools/mission_dryrun.py

가짜 텔레메트리를 link_node 와 **같은 RELIABLE QoS** 로 낸다 — 센서 QoS 로 내면
nav_node 가 한 줄도 못 받아서 무장이 조용히 실패한다 (그 함정을 여기서 한 번 밟았다).

보는 것 넷
  1. /gcs/daiv/cmd_vel 퍼블리셔가 3개다 — 시퀀서는 여기 없어야 한다
  2. 대기→이륙→USV추종→복귀→태그정렬→착륙→끝 을 다 지나고, 모드마다 주인이 정확히 하나다
  3. FOLLOW 중 /gcs/daiv/cmd_vel 이 ≈20Hz — 40Hz 면 두 제어기가 동시에 쏘는 사고다
  4. 전압을 되돌려도 복귀가 또 걸리지 않는다 (Edge 소비) · 땅에서 USV추종 건너뛰기는 거절된다

⚠ 두 벌이 돌면 /gcs/daiv/mission/state 가 섞여 모드가 왕복해 보인다. 먼저 남은 노드를 다 죽인다.
"""
import itertools, os, sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import Empty, Float32, Float64, Int32, Int32MultiArray

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "../ros2/src/rd_bridge/rd_bridge"))
from mission_node import CTRL_NO, NAMES   # noqa: E402 — 모드 표는 시퀀서의 것 하나만

LAT, LON = 36.397507, 127.123456
OWNER = {n: c or "없음" for c, n in CTRL_NO.items()}

class Fake(Node):
    def __init__(self):
        super().__init__("g1_fake")
        s = 10   # link_node 와 같은 RELIABLE
        self.fix = self.create_publisher(NavSatFix, "gcs/daiv/rd/fix", s)
        self.hdg = self.create_publisher(Float64, "gcs/daiv/rd/heading", s)
        self.gsp = self.create_publisher(Float32, "gcs/daiv/rd/gspeed", s)
        self.bat = self.create_publisher(BatteryState, "gcs/daiv/rd/battery", s)
        self.ufix = self.create_publisher(NavSatFix, "usv/fix", s)
        self.uhdg = self.create_publisher(Float64, "usv/heading", s)
        self.start = self.create_publisher(Empty, "gcs/daiv/mission/start", 10)
        self.goto = self.create_publisher(Int32, "gcs/daiv/mission/goto", 10)
        self.abort = self.create_publisher(Empty, "gcs/daiv/mission/abort", 10)
        self.alt, self.lat, self.lon, self.volt, self.usv_on = 0.0, LAT, LON, 16.5, True
        self.state, self.trail = (0, 0), []
        self.cmd_pubs = 0
        self.create_subscription(Int32MultiArray, "gcs/daiv/mission/state", self.on_state, 10)
        self.hz = []
        self.create_subscription(Twist, "gcs/daiv/cmd_vel", lambda _: self.hz.append(time.time()), 10)
        self.create_timer(0.1, self.pump)

    def on_state(self, m):
        st = (m.data[0], m.data[1])
        if not self.trail or self.trail[-1] != st:
            self.trail.append(st)
            print(f"    · 모드 {NAMES[st[0]]:6s} 주인 {OWNER[st[1]]}", flush=True)
        self.state = st

    def pump(self):
        m = NavSatFix(); m.latitude, m.longitude, m.altitude = self.lat, self.lon, self.alt
        self.fix.publish(m); self.hdg.publish(Float64(data=0.0))
        self.gsp.publish(Float32(data=0.1))
        b = BatteryState(); b.voltage, b.percentage, b.present = self.volt, 0.9, True
        self.bat.publish(b)
        if self.usv_on:
            u = NavSatFix(); u.latitude, u.longitude = self.lat + 0.0005, self.lon
            self.ufix.publish(u); self.uhdg.publish(Float64(data=0.0))

def spin(n, secs):
    t = time.time()
    while time.time() - t < secs:
        rclpy.spin_once(n, timeout_sec=0.05)

def cmd_publisher_count(n):
    return len(n.get_publishers_info_by_topic("/gcs/daiv/cmd_vel"))

rclpy.init()
n = Fake()
fails = []
spin(n, 2.0)
base = cmd_publisher_count(n)
print(f"  /gcs/daiv/cmd_vel 퍼블리셔 {base}개 (nav·usv·tag — 시퀀서는 여기 없어야 한다)")
if base != 3:
    fails.append(f"퍼블리셔가 3개가 아니다: {base}")

print("\n[1] 전체 소티")
n.start.publish(Empty()); spin(n, 2.0)
n.alt = 5.0; spin(n, 3.0)                     # 이륙 완료 → FOLLOW
n.hz.clear(); spin(n, 4.0)                    # FOLLOW 중 /gcs/daiv/cmd_vel 주파수 — 40Hz 면 둘이 쏘는 것
rate = len(n.hz) / 4.0
print(f"  FOLLOW 중 /gcs/daiv/cmd_vel {rate:.1f}Hz (20 근처여야 한다 — 40 이면 두 제어기가 동시에 쏜다)")
if not 12.0 <= rate <= 28.0:
    fails.append(f"/gcs/daiv/cmd_vel 이 {rate:.1f}Hz — 주인이 하나가 아니다")
n.volt = 14.2; spin(n, 8.0)                   # 저전압 5초 연속 → RETURN
n.volt = 16.5; spin(n, 3.0)                   # 되돌려도 안 돌아와야 한다 (엣지 소비)
n.alt = 8.0; spin(n, 3.0)                     # station 도착 (거리 0) → TAG
spin(n, 28.0)                                 # 태그를 못 봄 → 시간초과 착륙
n.alt = 0.1; spin(n, 6.0)                     # 접지 → DONE → IDLE
seq = [k for k, _ in itertools.groupby([m for m, _ in n.trail])]
owners = {m: c for m, c in n.trail if c}
print(f"  걸어온 길: {[NAMES[m] for m in dict.fromkeys(seq)]}")
print(f"  무장된 주인: {{{', '.join(NAMES[m] + '→' + OWNER[c] for m, c in owners.items())}}}")
for mode, want in ((2, 1), (3, 2), (4, 3)):
    if owners.get(mode) != want:
        fails.append(f"{NAMES[mode]} 에서 {OWNER[want]} 가 무장 안 됐다")
for want in (1, 2, 3, 4, 5, 6):
    if want not in seq:
        fails.append(f"{NAMES[want]} 를 안 지났다")
if 2 in seq and 3 in seq and seq.count(3) > 1:
    fails.append("전압을 되돌렸는데 복귀가 또 걸렸다 (엣지 래치 실패)")

print("\n[2] 개별 임무 — 땅에서 USV추종으로 건너뛰기는 거절되어야 한다")
n.alt, n.trail = 0.0, []
spin(n, 1.5)
n.goto.publish(Int32(data=2)); spin(n, 2.0)
if any(m == 2 for m, _ in n.trail):
    fails.append("땅에서 USV추종 건너뛰기가 통과했다")
else:
    print("    · 거절됨 ✓")

print("\n[3] 개별 임무 — 공중에서 복귀만 따로")
n.alt = 6.0; spin(n, 1.5)
n.goto.publish(Int32(data=3)); spin(n, 3.0)
ok3 = any(m == 3 for m, _ in n.trail)
print(f"    · 복귀 진입 {'✓' if ok3 else '✗'}")
if not ok3:
    fails.append("공중에서 복귀 건너뛰기가 안 됐다")
if cmd_publisher_count(n) != base:
    fails.append("퍼블리셔 수가 변했다")

print("\n[4] 중단")
n.abort.publish(Empty()); spin(n, 2.0)
if n.state[0] != 0 or n.state[1] != 0:
    fails.append(f"중단 후에도 모드 {n.state}")
else:
    print("    · 대기 · 주인 없음 ✓")

print()
if fails:
    print("✗ 실패:"); [print("   -", f) for f in fails]
else:
    print("✓ G1 전부 통과")
rclpy.shutdown()
sys.exit(1 if fails else 0)
