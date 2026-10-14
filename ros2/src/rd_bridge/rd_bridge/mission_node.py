#!/usr/bin/env python3
"""임무 시퀀서 — 이륙 → USV 추종 → (배터리) 복귀 → 태그 정밀착륙을 한 줄로 엮는다.

    ros2 launch rd_bridge bridge.launch.py mission:=true
    python3 tools/rd_cmd.py sortie       # space 시작 · r 복귀 · x 중단
    # 또는 직접
    ros2 topic pub --once /gcs/daiv/mission/start  std_msgs/msg/Empty '{}'
    ros2 topic pub --once /gcs/daiv/mission/return std_msgs/msg/Empty '{}'
    ros2 topic pub --once /gcs/daiv/mission/abort  std_msgs/msg/Empty '{}'
    ros2 topic echo /gcs/daiv/mission/state --field data      # [모드, 주인]

⚠⚠ 이 노드는 **/gcs/daiv/cmd_vel 을 내지 않는다.** nav_node · usv_follow_node · tag_follow_node 를
    하나씩 무장시키는 감독자다. mission:=true 는 셋을 다 띄우므로, "한 번에 하나" 라는
    규칙이 런치 파일의 구조적 보장에서 **이 파일의 CTRL 표 하나**(go() 의 CTRL.get)로 옮겨온다.
    그래서 여기 손댈 때는 G1 의 `ros2 topic hz /gcs/daiv/cmd_vel` 검사를 반드시 다시 돌린다
    (≈20Hz 를 유지해야 한다. ≈40Hz 가 보이면 두 제어기가 동시에 쏘고 있는 것이다).
⛔ rd_cmd.py drive 는 임무 중에 절대 같이 쓰지 않는다 — 유일하게 /gcs/daiv/cmd_vel 을 직접 쏘는 도구다.
⛔ NAV_RETURN_TO_LAUNCH(20) 는 쓰지 않는다. 앱의 doReturnToLaunch():396 NPE 가 미패치다.
    복귀는 nav_node 로 station 까지 날아가고 NAV_LAND(21) 로 내린다.

모드 — 정수 하나(self.mode)와 20Hz 타이머 하나. 콜백 안에서 sleep 하지 않는다.
    0 대기 · 1 이륙 · 2 USV추종 · 3 복귀 · 4 태그정렬 · 5 착륙 · 6 끝

    IDLE ──start──▶ TAKEOFF ──고도 80%──▶ FOLLOW ──배터리·수동·USV끊김──▶ RETURN
                                                                          │도착
    DONE ◀──접지── LAND ◀──인계 게이트── TAG ◀───────────────────────────┘
    어느 모드에서든 abort·좌표끊김·시간초과 → IDLE (전부 해제)

전이 규칙 두 가지 — 앞선 프로젝트(come_ws main_drive_node)가 실제로 여기서 깨졌다.
  1. 사건은 Edge 로 상승엣지에 한 번만 걸리고, **소비한 키 하나만** 지운다.
  2. 한 tick 에 next_mode() 를 한 번만 부른다 → 전이는 최대 하나.
     그래서 정지선 하나에 flag 1·2·3 이 연쇄로 통과하는 일이 구조적으로 안 생긴다.
  모드가 바뀌면 abort 를 뺀 래치를 전부 비운다 — 이륙 중에 누른 복귀는 **큐잉되지 않고
  버려진다.** 운용자가 다시 누른다. 의도된 동작이다.

무장이 조용히 실패한다는 것 — usv_follow_node.start() 도 nav_node.start() 도 우리 좌표가
없거나 낡으면 armed 를 안 세우고 그냥 돌아가며, 바깥에 알리지 않는다. 유일한 관측 수단이
/gcs/daiv/cmd_vel 트래픽이라 그것을 **읽기 전용으로** 구독해 감시한다.

개별 임무만 따로 시험하기 — 원본(main_drive_node)의 `input("Enter start mode")` 자리다.
그 코드는 launch 로 띄우면 입력 대기로 멈춰버렸으므로 **파라미터와 토픽**으로 옮겼다.
    ros2 param set /rd_mission start_mode 4      # space 를 누르면 TAG 부터 시작
    ros2 topic pub --once /gcs/daiv/mission/goto std_msgs/msg/Int32 '{data: 3}'   # 날면서 건너뛰기
    python3 tools/rd_cmd.py sortie               # 숫자키 1~6 이 같은 일을 한다
건너뛰기도 전이라 같은 규칙을 탄다 — 한 tick 에 하나, can_jump() 가 땅/하늘·station 을 본다.
TAKEOFF 는 땅에서만, 나머지는 공중에서만 들어갈 수 있다.

기록은 ~/drone_ws/missionlog/ 에 소티당 CSV 하나, station 은 같은 곳 station.txt.
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mav  # noqa: E402

IDLE, TAKEOFF, FOLLOW, RETURN, TAG, LAND, DONE = 0, 1, 2, 3, 4, 5, 6
NAMES = {IDLE: "대기", TAKEOFF: "이륙", FOLLOW: "USV추종", RETURN: "복귀",
         TAG: "태그정렬", LAND: "착륙", DONE: "끝"}

# 이 모드에서 /gcs/daiv/cmd_vel 을 낼 자격이 있는 제어기. 없으면 아무도 안 낸다.
CTRL = {FOLLOW: "usv", RETURN: "nav", TAG: "tag"}
CTRL_NO = {None: 0, "usv": 1, "nav": 2, "tag": 3}       # /gcs/daiv/mission/state 용

CH_PITCH = 9                      # usv_follow_node 와 같은 짐벌 채널

DEFAULTS = [
    ("takeoff_alt", 5.0), ("alt_reach", 0.8), ("takeoff_secs", 30.0),
    ("return_alt", 10.0), ("arrive_radius", 3.0), ("return_secs", 180.0),   # 10/1 8 → 10m
    ("land_on_timeout", True),
    ("align_alt", 3.0), ("align_radius", 0.6), ("land_alt", 1.2),
    ("land_radius", 0.35), ("land_gspeed", 0.3), ("hold_secs", 1.0),
    ("tag_secs", 25.0), ("land_without_tag", True), ("land_secs", 45.0),
    # 착륙 명령 뒤 이만큼 지나도 고도가 0.3m 안 줄면 다시 낸다 (최대 3번) — 10/1 첫 명령이 씹혔다
    ("land_retry_secs", 3.0),
    ("usv_lost_secs", 15.0), ("fix_secs", 2.0),
    ("switch_secs", 0.5), ("gimbal_secs", 2.5), ("gimbal_tilt", -90.0),
    ("volt_land", 14.0), ("volt_per_sec", 0.003), ("cruise", 2.0),
    ("land_budget", 45.0), ("volt_reserve", 0.3), ("volt_valid", 10.0),
    ("volt_secs", 5.0), ("batt_pct_min", 0.20), ("batt_secs", 5.0),
    ("station", [0.0, 0.0]), ("station_age", 3600.0),
    ("start_mode", 0),      # 0 = 전체 소티. 1~6 이면 space 가 그 모드부터 시작한다
]


class Edge:
    """올라가는 순간 한 번만 걸린다. 쓴 쪽이 명시적으로 뺀다 — 한 사건이 두 번 못 쓰인다."""

    def __init__(self):
        self.prev = False
        self.latched = False

    def update(self, now_true):
        """텔레메트리 조건용. 히스테리시스는 부르기 전에 접어서 넣는다."""
        if now_true and not self.prev:
            self.latched = True
        self.prev = bool(now_true)

    def fire(self):
        """토픽 콜백용 — 사람이 누른 것은 그 자체가 엣지다."""
        self.latched = True


def volt_line(dist, p):
    """이 전압 밑이면 station 까지 가서 내릴 여유가 없다."""
    secs = dist / max(0.5, p("cruise")) + p("land_budget")
    return p("volt_land") + p("volt_per_sec") * secs + p("volt_reserve")


def need_home(volt, pct, dist, batt_age, p):
    """'지금 출발하면 돌아가서 내릴 수 있나'. 못 믿을 값이면 False — 안전측은 '안 부른다'.

    patches/03 이 안 붙은 기체는 전압이 0.00 으로 온다. volt_valid 가드가 없으면
    이륙하자마자 복귀한다. 이 가드를 지우지 말 것.
    """
    if batt_age > p("batt_secs") or volt < p("volt_valid"):
        return False
    if volt <= volt_line(dist, p):
        return True
    return p("batt_pct_min") > 0.0 and 0.0 <= pct <= p("batt_pct_min")


def centred(fwd, right, radius):
    return math.hypot(fwd, right) <= radius


def can_jump(mode, tel):
    """그 모드로 바로 들어가도 되나 — 개별 임무 시험용. 거부 사유 문자열 or None.

    땅/하늘만 본다. 이륙은 땅에서만, 나머지는 이미 떠 있어야 한다 — 땅에서 FOLLOW 로
    건너뛰면 추종기가 무장한 채 프로펠러가 도는 기체에 수평 속도를 쏜다.
    """
    if mode in (IDLE, DONE):
        return None
    if mode not in NAMES:
        return f"모르는 모드 {mode}"
    if mode == TAKEOFF:
        if not tel["station_ok"]:
            return "좌표를 못 믿는다 (bad_fix)"
        return "이미 떠 있다" if tel["alt"] >= 1.5 else None
    if tel["alt"] <= 1.0:
        return "땅에 있다 — 먼저 이륙할 것"
    if mode == RETURN and not tel["station_ok"]:
        return "station 이 없다"
    return None


def next_mode(mode, t_in, ev, tel, p):
    """(새 모드, 이유, 소비한 사건 or None). 첫 매치에서 돌아간다 — 한 tick 에 전이 하나.

    ev·tel 은 스냅샷이다. 여기서 아무것도 지우지 않는다 (부른 쪽이 돌려받은 키만 지운다).
    이유가 있는데 모드가 그대로면 '사건을 썼지만 거절했다' 는 뜻이다.
    """
    if ev.get("abort"):
        return IDLE, "중단 명령", "abort"

    if ev.get("goto"):
        want = tel.get("goto")
        bad = can_jump(want, tel) if want is not None else "목표 모드가 없다"
        if bad:
            return mode, f"건너뛰기 거부 — {bad}", "goto"
        return want, f"건너뛰기 — {NAMES[want]} 부터", "goto"

    stale = tel["fix_age"] > p("fix_secs")

    if mode == IDLE:
        if ev.get("ret"):
            if tel["station_ok"] and tel["alt"] > 1.0:
                return RETURN, "수동 복귀 — 공중에서 이어간다", "ret"
            return IDLE, "복귀 거부 — station 이 없거나 이미 땅이다", "ret"
        if ev.get("start"):
            why = can_jump(TAKEOFF, tel) or (
                "누가 /gcs/daiv/cmd_vel 을 쏘고 있다" if tel["cmd_age"] < 2.0 else None)
            if why:
                return IDLE, f"시작 거부 — {why}", "start"
            return TAKEOFF, "이륙", "start"
        return IDLE, "", None

    if mode == TAKEOFF:
        if stale:
            return IDLE, "좌표가 끊겼다", None
        if tel["alt"] >= p("takeoff_alt") * p("alt_reach"):
            return FOLLOW, f"고도 {tel['alt']:.1f}m 도달", None
        if t_in > p("takeoff_secs"):
            return IDLE, f"이륙 시간초과 ({t_in:.0f}초, 고도 {tel['alt']:.1f}m)", None
        return TAKEOFF, "", None

    if mode == FOLLOW:
        if stale:
            return IDLE, "좌표가 끊겼다", None
        if ev.get("batt"):
            return RETURN, "배터리 — 복귀", "batt"
        if ev.get("ret"):
            return RETURN, "수동 복귀", "ret"
        if tel["usv_age"] > p("usv_lost_secs"):
            return RETURN, f"USV 좌표가 {tel['usv_age']:.0f}초 없다 — 복귀", None
        return FOLLOW, "", None

    if mode == RETURN:
        if stale:
            return IDLE, "좌표가 끊겼다", None
        if tel["dist"] <= p("arrive_radius") and abs(tel["alt"] - p("return_alt")) < 1.5:
            return TAG, f"station 도착 (거리 {tel['dist']:.1f}m)", None
        if t_in > p("return_secs"):
            if p("land_on_timeout"):
                return LAND, f"복귀 시간초과 ({t_in:.0f}초) — 그 자리 착륙", None
            return IDLE, f"복귀 시간초과 ({t_in:.0f}초)", None
        return RETURN, "", None

    if mode == TAG:
        if stale:
            return IDLE, "좌표가 끊겼다", None
        if tel["land_hold"] >= p("hold_secs"):
            return LAND, f"인계 — 오차 {tel['tag_r']:.2f}m · 고도 {tel['alt']:.1f}m", None
        if t_in > p("tag_secs") and tel["tag_age"] > 1.0:
            if p("land_without_tag"):
                return LAND, "태그를 못 봤다 — 그 자리 착륙", None
            return IDLE, "태그를 못 봤다", None
        return TAG, "", None

    if mode == LAND:
        if tel["touch_hold"] >= 3.0:
            return DONE, "접지", None
        if t_in > p("land_secs"):
            return IDLE, f"착륙 시간초과 ({t_in:.0f}초) — RC S 모드로", None
        return LAND, "", None

    if mode == DONE:
        if t_in > 3.0:
            return IDLE, "소티 끝", None
        return DONE, "", None

    return IDLE, f"모르는 모드 {mode}", None


def selftest():
    p = dict(DEFAULTS).get
    fresh = dict(alt=0.0, fix_age=0.0, usv_age=0.0, tag_age=0.0, tag_r=float("inf"),
                 gspeed=0.0, dist=float("inf"), station_ok=True, cmd_age=99.0,
                 land_hold=0.0, touch_hold=0.0, goto=None)

    # 1. 정상 소티 — 0→1→2→3→4→5→6→0
    walk = []
    m = IDLE
    m, why, used = next_mode(m, 0.0, {"start": True}, fresh, p)
    walk.append(m); assert used == "start" and m == TAKEOFF, (m, why)
    m = next_mode(m, 3.0, {}, dict(fresh, alt=4.5), p)[0]
    walk.append(m)
    m, _, used = next_mode(m, 10.0, {"batt": True}, dict(fresh, alt=5.0, dist=150.0), p)
    walk.append(m); assert used == "batt"
    m = next_mode(m, 60.0, {}, dict(fresh, alt=p("return_alt"), dist=1.0), p)[0]
    walk.append(m)
    m = next_mode(m, 20.0, {}, dict(fresh, alt=1.2, tag_r=0.2, land_hold=1.5), p)[0]
    walk.append(m)
    m = next_mode(m, 12.0, {}, dict(fresh, alt=0.1, touch_hold=3.5), p)[0]
    walk.append(m)
    m = next_mode(m, 4.0, {}, fresh, p)[0]
    walk.append(m)
    assert walk == [TAKEOFF, FOLLOW, RETURN, TAG, LAND, DONE, IDLE], walk

    # 2. 연쇄 금지 — 두 사건이 동시에 걸려도 한 tick 에 한 칸만 간다
    tel = dict(fresh, alt=5.0, dist=150.0)
    m, _, used = next_mode(FOLLOW, 10.0, {"batt": True, "ret": True}, tel, p)
    assert (m, used) == (RETURN, "batt"), (m, used)
    ev = {"ret": True}                      # batt 만 지웠다 — ret 은 아직 걸려 있다
    assert next_mode(m, 0.1, ev, tel, p)[0] == RETURN     # 그래도 더 안 간다

    # 3. 출구 보장 — IDLE(사람을 기다리는 자리) 말고는 전부 시간·끊김으로 빠져나온다
    dead = dict(fresh, fix_age=99.0, usv_age=99.0, tag_age=99.0)
    for mode in (TAKEOFF, FOLLOW, RETURN, TAG, LAND, DONE):
        got = next_mode(mode, 10_000.0, {}, dead, p)[0]
        assert got != mode, f"{NAMES[mode]} 에 출구가 없다"
    # 텔레메트리가 멀쩡해도 시간으로 빠진다 (FOLLOW 만 USV 가 살아 있으면 계속이 맞다)
    for mode in (TAKEOFF, RETURN, TAG, LAND, DONE):
        assert next_mode(mode, 10_000.0, {}, dict(fresh, tag_age=99.0), p)[0] != mode

    # 4. 동시 무장 없음
    assert set(CTRL.values()) == {"usv", "nav", "tag"}
    assert all(CTRL.get(m) is None for m in (TAKEOFF, LAND, IDLE, DONE))
    assert len(set(CTRL.values())) == len(CTRL)          # 한 모드에 한 제어기, 겹치지 않는다

    # 5. 배터리 — 전압과 거리를 같이 본다
    assert need_home(14.9, 0.5, 0.0, 0.0, p) is False
    assert need_home(14.4, 0.5, 0.0, 0.0, p) is True
    assert need_home(14.6, 0.5, 300.0, 0.0, p) is True     # 멀면 더 일찍 돌아선다
    assert need_home(14.6, 0.5, 0.0, 0.0, p) is False      # 같은 전압도 가까우면 더 논다
    assert need_home(0.0, 0.9, 0.0, 0.0, p) is False       # patches/03 미적용 → 0.00 V
    assert need_home(16.5, 0.15, 0.0, 0.0, p) is True      # 잔량 하한
    assert need_home(14.0, 0.5, 0.0, 99.0, p) is False     # 낡은 값

    # 6. station 게이트 — 한 번 잘못 잡히면 몇 km 짜리 복귀 구간이 된다
    assert mav.bad_fix(0.0, 0.0) and mav.bad_fix(float("nan"), 127.0)
    assert mav.bad_fix(mav.FAKE_LAT, mav.FAKE_LON) and mav.bad_fix(None, None)
    assert mav.bad_fix(37.0, 127.0) is None
    tel_bad = dict(fresh, station_ok=False)
    m, why, used = next_mode(IDLE, 0.0, {"start": True}, tel_bad, p)
    assert (m, used) == (IDLE, "start") and "거부" in why
    # 이미 떠 있거나 남이 조종 중이면 시작하지 않는다
    assert next_mode(IDLE, 0.0, {"start": True}, dict(fresh, alt=3.0), p)[0] == IDLE
    assert next_mode(IDLE, 0.0, {"start": True}, dict(fresh, cmd_age=0.1), p)[0] == IDLE

    # 7. 착륙 기하 — 파라미터를 부주의하게 고치면 지상에서 깨진다
    assert p("land_radius") >= 0.3, "tag_follow 의 deadband 0.3 안에서는 추종기가 0 을 낸다"
    assert p("land_alt") + 0.5 < p("align_alt"), "인계 고도가 정렬 고도보다 낮아야 한다"
    assert centred(0.2, 0.2, p("land_radius")) is True     # hypot 0.283 ≤ 0.35
    assert centred(0.3, 0.3, p("land_radius")) is False    # hypot 0.424 > 0.35

    # 8. 개별 임무 건너뛰기 — 땅/하늘 조건을 지킨다
    ground, air = fresh, dict(fresh, alt=6.0)
    assert can_jump(TAKEOFF, ground) is None
    assert can_jump(TAKEOFF, air) == "이미 떠 있다"
    assert can_jump(FOLLOW, ground) == "땅에 있다 — 먼저 이륙할 것"
    assert can_jump(FOLLOW, air) is None and can_jump(TAG, air) is None
    assert can_jump(RETURN, dict(air, station_ok=False)) == "station 이 없다"
    assert can_jump(TAKEOFF, dict(ground, station_ok=False)) is not None
    assert can_jump(IDLE, ground) is None                   # 해제는 언제나 된다
    # 건너뛰기도 사건 하나를 쓰고 한 칸만 간다
    m, why, used = next_mode(FOLLOW, 5.0, {"goto": True}, dict(air, goto=TAG), p)
    assert (m, used) == (TAG, "goto"), (m, used)
    m, why, used = next_mode(IDLE, 0.0, {"goto": True}, dict(ground, goto=FOLLOW), p)
    assert (m, used) == (IDLE, "goto") and "거부" in why, (m, why)
    # abort 가 건너뛰기보다 먼저다
    assert next_mode(FOLLOW, 5.0, {"goto": True, "abort": True},
                     dict(air, goto=TAG), p)[2] == "abort"
    print("selftest ok")


if __name__ == "__main__" and "--selftest" in sys.argv:
    sys.exit(selftest())

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from rcl_interfaces.srv import SetParameters  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus  # noqa: E402
from std_msgs.msg import (Bool, Empty, Float32, Float64MultiArray,  # noqa: E402
                          Int32, Int32MultiArray)

STATION_FILE = os.path.expanduser("~/drone_ws/missionlog/station.txt")


class MissionNode(Node):

    def __init__(self):
        super().__init__("rd_mission")
        for name, val in DEFAULTS:
            self.declare_parameter(name, val)
        self.p = lambda name: self.get_parameter(name).value

        # ── 내보내는 것 — /gcs/daiv/cmd_vel 은 여기 없다. 그게 이 설계의 전부다
        self.pub_usv = self.create_publisher(Bool, "gcs/daiv/usv/follow", 10)
        self.pub_usv_x = self.create_publisher(Empty, "gcs/daiv/usv/cancel", 10)
        self.pub_tag = self.create_publisher(Bool, "gcs/daiv/tag/follow", 10)
        self.pub_tag_x = self.create_publisher(Empty, "gcs/daiv/tag/cancel", 10)
        self.pub_goal = self.create_publisher(NavSatFix, "gcs/daiv/nav/goal", 10)
        self.pub_nav_x = self.create_publisher(Empty, "gcs/daiv/nav/cancel", 10)
        self.pub_cmd = self.create_publisher(Float64MultiArray, "gcs/daiv/rd/command", 10)
        self.pub_state = self.create_publisher(Int32MultiArray, "gcs/daiv/mission/state", 10)

        sensor = qos_profile_sensor_data
        self.create_subscription(NavSatFix, "gcs/daiv/rd/fix", self.on_fix, sensor)
        self.create_subscription(Float32, "gcs/daiv/rd/gspeed", self.on_gspeed, sensor)
        self.create_subscription(BatteryState, "gcs/daiv/rd/battery", self.on_batt, sensor)
        self.create_subscription(NavSatFix, "usv/fix", self.on_usv, sensor)
        self.create_subscription(Float64MultiArray, "gcs/daiv/tag/state", self.on_tag, 10)
        # /gcs/daiv/cmd_vel 은 **읽기만** 한다 — 무장이 조용히 실패한 것을 여기서만 알 수 있다
        self.create_subscription(Twist, "gcs/daiv/cmd_vel", self.on_cmd, 10)
        self.create_subscription(Empty, "gcs/daiv/mission/start", self.on_start, 10)
        self.create_subscription(Int32, "gcs/daiv/mission/goto", self.on_goto, 10)
        self.create_subscription(Empty, "gcs/daiv/mission/return", lambda _: self.ev["ret"].fire(), 10)
        self.create_subscription(Empty, "gcs/daiv/mission/abort", lambda _: self.ev["abort"].fire(), 10)

        self.cli_tag = self.create_client(SetParameters, "/rd_tag_follow/set_parameters")

        # ── 상태
        self.mode, self.t_mode = IDLE, time.time()
        self.owner, self.pending, self.t_switch, self.arm_left = None, None, 0.0, 0
        self.gimbal_sent, self.t_armed, self.t_rearm = False, 0.0, 0.0
        self.goal_tries = 0
        self.land_sent, self.land_tries, self.takeoff_resent = False, 0, False
        self.t_land_sent, self.alt_land_sent = 0.0, 0.0
        self.aligned = False
        self.ev = {k: Edge() for k in ("start", "ret", "batt", "abort", "goto")}
        self.goto = None

        self.me, self.t_fix = None, 0.0
        self.gspeed, self.t_usv, self.t_tag, self.tag = 0.0, 0.0, 0.0, None
        self.volt, self.pct, self.t_batt = 0.0, float("nan"), 0.0
        self.t_cmd, self.t_low, self.t_align, self.t_land, self.t_touch = 0.0, 0.0, 0.0, 0.0, 0.0
        self.station = self.load_station()
        self.sweep = 0
        self.csv, self.t_log, self.t_set, self.warned_batt = None, 0.0, 0.0, False
        self.t_boot = time.time()

        self.create_timer(0.05, self.tick)   # 20Hz
        self.get_logger().warning(
            "⚠ nav·usv·tag 를 한꺼번에 띄운 상태다 — /gcs/daiv/cmd_vel 배타는 이제 CTRL 표가 지킨다. "
            "rd_cmd.py drive 를 같이 쓰지 말 것")
        self.get_logger().info(
            "대기 중 — /gcs/daiv/mission/start 로 시작"
            + (f" · station {self.station[0]:.7f},{self.station[1]:.7f}" if self.station else ""))

    # ------------------------------------------------------------------ 수신
    def on_fix(self, m):
        if mav.bad_fix(m.latitude, m.longitude) or m.status.status == NavSatStatus.STATUS_NO_FIX:
            return
        if math.isnan(m.altitude):
            return
        self.me, self.t_fix = (m.latitude, m.longitude, m.altitude), time.time()

    def on_gspeed(self, m):
        self.gspeed = m.data

    def on_batt(self, m):
        # 1Hz — 5초쯤의 EMA 로 상승 중 새그를 뭉갠다
        v = float(m.voltage)
        self.volt = v if self.volt <= 0.0 else 0.2 * v + 0.8 * self.volt
        self.pct = float(m.percentage)
        self.t_batt = time.time()

    def on_usv(self, m):
        if mav.bad_fix(m.latitude, m.longitude) or m.status.status == NavSatStatus.STATUS_NO_FIX:
            return
        self.t_usv = time.time()

    def on_tag(self, m):
        if len(m.data) >= 5:
            self.tag, self.t_tag = tuple(m.data[:5]), time.time()

    def on_cmd(self, _):
        self.t_cmd = time.time()

    def on_start(self, _):
        """start_mode 가 0 이면 전체 소티, 아니면 그 모드만 시험한다."""
        n = int(self.p("start_mode"))
        if n:
            self.goto = n
            self.ev["goto"].fire()
            self.get_logger().info(f"start_mode {n} — {NAMES.get(n, '?')} 만 시험한다")
        else:
            self.ev["start"].fire()

    def on_goto(self, m):
        self.goto = int(m.data)
        self.ev["goto"].fire()

    # ------------------------------------------------------------- station
    def load_station(self):
        """파라미터 → station.txt 순. 둘 다 없으면 소티를 시작할 때까지 복귀를 거부한다."""
        s = self.p("station")
        if len(s) == 2 and mav.bad_fix(s[0], s[1]) is None:
            self.get_logger().info(f"station 파라미터 — {s[0]:.7f},{s[1]:.7f}")
            return (float(s[0]), float(s[1]))
        try:
            lat, lon, t = open(STATION_FILE).read().split()
            if mav.bad_fix(float(lat), float(lon)) is None \
                    and time.time() - float(t) < self.p("station_age"):
                self.get_logger().warning(
                    f"⚠ 복구된 station {float(lat):.7f},{float(lon):.7f} — 좌표를 눈으로 확인할 것")
                return (float(lat), float(lon))
        except (OSError, ValueError):
            pass
        return None

    def save_station(self):
        os.makedirs(os.path.dirname(STATION_FILE), exist_ok=True)
        with open(STATION_FILE, "w") as f:
            f.write(f"{self.station[0]:.7f} {self.station[1]:.7f} {time.time():.0f}\n")

    # --------------------------------------------------------- 무장 / 해제
    def command(self, cmd, *params):
        d = [float(cmd)] + [float(x) for x in params]
        self.pub_cmd.publish(Float64MultiArray(data=d + [0.0] * (8 - len(d))))

    def disarm_all(self):
        """항상 Bool false 와 cancel 을 같이 낸다 — 하나가 유실돼도 서게."""
        self.pub_usv.publish(Bool(data=False))
        self.pub_usv_x.publish(Empty())
        self.pub_tag.publish(Bool(data=False))
        self.pub_tag_x.publish(Empty())
        self.pub_nav_x.publish(Empty())

    def arm(self, who):
        if who == "usv":
            self.pub_usv.publish(Bool(data=True))
        elif who == "tag":
            self.pub_tag.publish(Bool(data=True))
        elif who == "nav":
            # ⚠ 멱등하지 않다 — 다시 내면 nav_node 가 미션을 재시작하고 CSV 를 새로 연다
            m = NavSatFix()
            m.latitude, m.longitude = self.station
            m.altitude = float(self.p("return_alt"))
            self.pub_goal.publish(m)

    def set_owner(self, who):
        """먼저 전부 해제하고, 브레이크가 끝난 뒤에 새 주인을 무장한다. sleep 하지 않는다."""
        self.disarm_all()
        self.owner, self.pending, self.t_switch = None, who, time.time()
        self.gimbal_sent, self.arm_left, self.goal_tries = False, 0, 0

    def set_hold_alt(self, want):
        """tag_follow 의 목표 고도. /gcs/daiv/tag/state 로 확인되므로 될 때까지 0.5초마다 다시 쓴다."""
        if not self.cli_tag.service_is_ready():
            return self.get_logger().warning("rd_tag_follow 파라미터 서비스가 아직 없다",
                                             throttle_duration_sec=2.0)
        self.cli_tag.call_async(SetParameters.Request(
            parameters=[Parameter("hold_alt", value=float(want)).to_parameter_msg()]))

    # ------------------------------------------------------------- 모드 전이
    def go(self, mode, why):
        prev = self.mode
        self.mode, self.t_mode = mode, time.time()
        for k, e in self.ev.items():
            if k != "abort":
                e.latched = False            # 이전 모드에서 누른 것은 큐잉하지 않는다
        self.aligned = False
        self.land_sent = self.takeoff_resent = False
        self.land_tries = 0
        self.t_align = self.t_land = self.t_touch = 0.0
        self.set_owner(CTRL.get(mode))
        if prev == IDLE and mode != IDLE and self.csv is None:
            self.open_csv()
        self.get_logger().info(f"▶ {NAMES[prev]} → {NAMES[mode]} — {why}")
        self.publish_state()
        if mode == TAKEOFF:
            self.command(mav.CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, self.p("takeoff_alt"))
            self.get_logger().info(f"이륙 명령 — 목표 {self.p('takeoff_alt'):.1f}m")
        elif mode == IDLE:
            self.disarm_all()
            self.get_logger().warning("■ 전부 해제 — 조종권은 저절로 안 돌아온다. "
                                      "도구 종료 → RC 모드 스위치 S")
            self.close_csv()
        elif mode == DONE:
            self.close_csv()

    def publish_state(self):
        self.pub_state.publish(Int32MultiArray(
            data=[self.mode, CTRL_NO[self.owner], int(time.time() - self.t_mode)]))

    def open_csv(self):
        os.makedirs(os.path.expanduser("~/drone_ws/missionlog"), exist_ok=True)
        path = os.path.expanduser(time.strftime("~/drone_ws/missionlog/%Y%m%d-%H%M%S.csv"))
        self.csv = open(path, "w")
        self.csv.write("t,mode,ctrl,lat,lon,alt,dist,gspeed,volt,pct,"
                       "usv_age,tag_age,tag_r,why\n")

    def close_csv(self):
        if self.csv is not None:
            self.csv.close()
            self.csv = None

    # ------------------------------------------------------------------ 20Hz
    def tick(self):
        now = time.time()

        # 재시작 스윕 — 우리가 죽은 동안 armed 로 남은 제어기가 계속 날고 있을 수 있다
        if self.sweep < 2:
            self.sweep += 1
            self.disarm_all()
            if self.sweep == 1:
                self.get_logger().warning("기동 — 제어기 전부 해제 (고아 방지)")
            return

        if not self.warned_batt and now - self.t_boot > 10.0:
            self.warned_batt = True
            if now - self.t_batt > 5.0 or self.volt < self.p("volt_valid"):
                self.get_logger().warning(
                    "⚠ /gcs/daiv/rd/battery 가 안 오거나 0V 다 — **자동 복귀가 꺼진 상태로 난다** "
                    "(patches/03 미적용). 수동 복귀만 믿을 것")

        self.switchover(now)
        tel = self.telemetry(now)

        # 배터리는 텔레메트리 조건이라 히스테리시스를 먼저 접고 엣지에 올린다
        low = need_home(self.volt, self.pct, tel["dist"], now - self.t_batt, self.p)
        self.t_low = self.t_low or now if low else 0.0
        if self.mode == FOLLOW:
            self.ev["batt"].update(bool(self.t_low) and now - self.t_low >= self.p("volt_secs"))

        # ── 전이는 한 tick 에 한 번. next_mode 는 여기서만 부른다
        ev = {k: e.latched for k, e in self.ev.items()}
        mode, why, used = next_mode(self.mode, now - self.t_mode, ev, tel, self.p)
        if used:
            self.ev[used].latched = False        # 돌려받은 키 하나만 — 연쇄를 막는 곳
        if mode != self.mode:
            if mode == TAKEOFF:
                self.station = (self.me[0], self.me[1])   # 땅에서만 들어오므로 여기가 station
                self.save_station()
                self.get_logger().info(
                    f"station 저장 — {self.station[0]:.7f},{self.station[1]:.7f}")
            self.go(mode, why)
            return                                # 이 tick 은 여기서 끝. 두 번 안 간다
        # 같은 모드로의 건너뛰기는 수락이다 — 거부만 ERROR.
        # rclpy 는 호출한 줄마다 severity 를 고정하므로 한 줄에서 info/error 를 바꿔 부르면 죽는다
        if used and "거부" in why:
            self.get_logger().error(why)
        elif used and why:
            self.get_logger().info(why)

        self.work(now, tel)
        self.log(now, tel, why)

    def switchover(self, now):
        """해제 → (필요하면 짐벌) → 무장. 전부 타임스탬프 비교다."""
        if self.pending is not None:
            dt = now - self.t_switch
            if self.pending == "tag" and not self.gimbal_sent and dt >= self.p("switch_secs"):
                self.command(mav.CMD_DO_SET_SERVO, CH_PITCH,
                             1500.0 + self.p("gimbal_tilt") * 5.5)
                self.gimbal_sent = True
            need = self.p("gimbal_secs") if self.pending == "tag" else self.p("switch_secs")
            if dt >= need:
                self.owner, self.pending = self.pending, None
                self.arm_left = 3 if self.owner in ("usv", "tag") else 1
                self.t_armed = now
                self.publish_state()
        if self.arm_left > 0:
            self.arm_left -= 1
            self.arm(self.owner)                  # Bool 은 멱등이라 3틱 반복이 공짜
            self.t_rearm = now
            return
        if self.owner in ("usv", "tag") and now - self.t_rearm > 2.0:
            # 두 추종기는 재료가 giveup_secs(각 10초) 넘게 없으면 **스스로 해제한다.**
            # 재료가 돌아와도 무장은 안 돌아오므로 주기적으로 다시 켠다 — start() 는 멱등이다
            self.t_rearm = now
            self.arm(self.owner)
        if (self.owner == "nav" and now - self.t_cmd > 3.0 and now - self.t_armed > 3.0
                and self.goal_tries < 3):
            # nav 는 목표 재발행이 미션 재시작이라 멱등하지 않다 — 3번까지만
            self.goal_tries += 1
            self.t_armed = now
            self.get_logger().warning(f"nav 가 조용하다 — 목표 재발행 {self.goal_tries}/3")
            self.arm("nav")
        # 무장했는데 재료가 있는데도 조용하면 start() 가 조용히 실패한 것이다.
        # 경고만 한다 — 전부 해제해도 결국 호버라, 놀라게 하는 중단보다 낫다
        if self.owner and now - self.t_cmd > 3.0 and self.owner_should_drive(now):
            self.get_logger().warning(
                f"{self.owner} 를 무장했는데 /gcs/daiv/cmd_vel 이 3초 넘게 조용하다 — "
                "그쪽 노드 로그를 볼 것", throttle_duration_sec=3.0)

    def owner_should_drive(self, now):
        """지금 이 제어기가 /gcs/daiv/cmd_vel 을 낼 재료가 있나. 없으면 조용한 것이 정상이다.

        태그를 못 보는 동안 tag_follow 는 아무것도 안 낸다 — 그걸 고장으로 읽으면 안 된다.
        """
        if self.owner == "usv":
            return now - self.t_usv < self.p("usv_lost_secs")
        if self.owner == "tag":
            return now - self.t_tag < 1.0
        return self.owner == "nav"

    def telemetry(self, now):
        alt = self.me[2] if self.me else 0.0
        dist = (mav.meters_between(self.me[0], self.me[1], *self.station)
                if self.me and self.station else float("inf"))
        r = math.hypot(self.tag[0], self.tag[1]) if self.tag else float("inf")
        tag_age = now - self.t_tag if self.t_tag else float("inf")

        # 정렬·인계·접지는 '연속 몇 초' 가 조건이라 여기서 접어서 초 단위로 넘긴다
        ok_align = tag_age < 1.0 and centred(self.tag[0], self.tag[1], self.p("align_radius"))
        self.t_align = self.t_align or now if ok_align else 0.0
        if self.mode == TAG and self.t_align and now - self.t_align >= self.p("hold_secs"):
            self.aligned = True                   # 단조 — 한 번 내려간 목표는 안 올린다
        ok_land = (tag_age < 1.0 and centred(self.tag[0], self.tag[1], self.p("land_radius"))
                   and alt <= self.p("land_alt") + 0.5 and self.gspeed <= self.p("land_gspeed"))
        self.t_land = self.t_land or now if ok_land else 0.0
        self.t_touch = self.t_touch or now if alt <= 0.3 else 0.0

        return dict(
            alt=alt, fix_age=now - self.t_fix if self.t_fix else float("inf"),
            usv_age=now - self.t_usv if self.t_usv else float("inf"),
            tag_age=tag_age, tag_r=r, gspeed=self.gspeed, dist=dist,
            station_ok=self.me is not None and mav.bad_fix(self.me[0], self.me[1]) is None,
            cmd_age=now - self.t_cmd if self.t_cmd else float("inf"), goto=self.goto,
            land_hold=now - self.t_land if self.t_land else 0.0,
            touch_hold=now - self.t_touch if self.t_touch else 0.0)

    def work(self, now, tel):
        """모드 안에서 하는 일 — 전이는 안 한다."""
        t_in = now - self.t_mode

        if self.mode == TAKEOFF and not self.takeoff_resent and t_in > 5.0 and tel["alt"] < 0.5:
            self.takeoff_resent = True
            self.command(mav.CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, self.p("takeoff_alt"))
            self.get_logger().warning("5초가 지나도 안 뜬다 — 이륙 명령 한 번 더")

        elif self.mode == TAG and self.owner == "tag" and now - self.t_set > 0.5:
            # hold_alt 는 /gcs/daiv/tag/state 로 되읽을 수 있다 (z - alt_err) — 맞을 때까지 다시 쓴다
            want = self.p("land_alt") if self.aligned else self.p("align_alt")
            if self.tag is None or abs((self.tag[4] - self.tag[2]) - want) > 0.05:
                self.t_set = now
                self.set_hold_alt(want)

        elif self.mode == LAND:
            if not self.land_sent and t_in >= self.p("switch_secs"):
                self.send_land(now, tel)
                self.get_logger().info(f"착륙 명령 — 고도 {tel['alt']:.1f}m · "
                                       f"대지속도 {tel['gspeed']:.2f}m/s")
            elif (self.land_sent and self.land_tries < 3 and tel["alt"] > 0.5
                  and now - self.t_land_sent > self.p("land_retry_secs")
                  and tel["alt"] > self.alt_land_sent - 0.3):
                # 10/1: 첫 명령이 link_node BRAKE 의 0 에 덮여 22초를 호버했다. link_node 를
                # 고쳤지만, 앱이 명령을 흘리는 다른 경우도 있을 수 있어 짧게 다시 낸다
                self.land_tries += 1
                self.get_logger().warning(
                    f"{self.p('land_retry_secs'):.0f}초째 안 내려온다 "
                    f"({self.alt_land_sent:.1f} → {tel['alt']:.1f}m) — 착륙 명령 다시 "
                    f"{self.land_tries}/3")
                self.send_land(now, tel)

    def send_land(self, now, tel):
        self.command(mav.CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0)
        self.land_sent, self.t_land_sent, self.alt_land_sent = True, now, tel["alt"]

    def log(self, now, tel, why):
        if now - self.t_log < 1.0:
            return
        self.t_log = now
        self.publish_state()
        if self.csv is not None:
            lat, lon = (self.me[0], self.me[1]) if self.me else (0.0, 0.0)
            self.csv.write(
                f"{now:.2f},{NAMES[self.mode]},{self.owner or '-'},{lat:.7f},{lon:.7f},"
                f"{tel['alt']:.2f},{tel['dist']:.1f},{tel['gspeed']:.2f},{self.volt:.2f},"
                f"{self.pct:.2f},{tel['usv_age']:.1f},{tel['tag_age']:.1f},{tel['tag_r']:.2f},"
                f"{why}\n")
            self.csv.flush()
        if self.mode == IDLE:
            return
        thr = volt_line(min(tel["dist"], 9999.0), self.p)
        self.get_logger().info(
            f"[{NAMES[self.mode]} {now - self.t_mode:4.0f}s] 주인 {self.owner or '없음'} · "
            f"고도 {tel['alt']:5.1f}m · station {tel['dist']:6.1f}m · "
            f"전압 {self.volt:5.2f}/{thr:.2f}V · "
            f"USV {tel['usv_age']:.0f}s · 태그 {tel['tag_age']:.0f}s r{tel['tag_r']:.2f}m")


def main():
    rclpy.init()
    node = MissionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.disarm_all()
            node.get_logger().info("■ 노드 종료 — 제어기 해제. RC 모드 스위치 S")
        node.close_csv()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
