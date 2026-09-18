#!/usr/bin/env python3
"""USV 추종 노드 — 무인수상정(USV)의 GPS 를 받아 뒤에서 따라가고, 기수와 짐벌을 USV 에 겨눈다.

    ros2 launch rd_bridge bridge.launch.py usv:=true nav:=false
    python3 tools/rd_cmd.py usv          # space 시작/정지 · 파라미터 튜닝
    # 또는 직접: /usv/follow (Bool) true·false · /usv/cancel (Empty)

⚠ /usv/follow true 를 받기 전에는 /cmd_vel 도 짐벌도 안 낸다.
⚠ nav_node · tag_follow_node 와 동시에 켜지 않는다 — 셋 다 /cmd_vel 에 쓴다.

입력 (USV 팀 토픽 이름이 다르면 remap 한다 — 예: -r usv/fix:=/boat/gps)
    /usv/fix      NavSatFix  위경도 (고도는 안 쓴다 — USV = 수면)
    /usv/heading  Float64    기수 ° (0=북, 시계)
    /usv/depth    Float64    수심 m — 기록만
    /rd/fix · /rd/heading    우리 위치·상대고도·기수 (link_node)

한 tick 에 하는 일 — plan() 한 함수
    1. USV 위치 추정: 최근 vel_window 초의 좌표로 속도를 내고, (좌표 나이 + lead_secs) 만큼 등속 외삽
    2. 목표점 = 추정 위치에서 **대형**만큼 — USV 기수 기준 (앞 m, 오른쪽 m, 고도 m). 음수 = 뒤·왼쪽
    3. 수평 = kp × 목표까지 거리 (nav_node 와 같은 P) + USV 속도 feedforward, max_speed 로 자른다
    4. 기수 = USV 쪽으로 (짐벌 yaw 는 상대·누적각이라 안 쓰고 몸체를 돌린다)
    5. 틸트 = -atan2(우리 상대고도, USV 까지 수평거리) → DO_SET_SERVO ch9 (2° 넘게 바뀔 때, 최대 2Hz)

대형은 formations = [앞, 오른쪽, 고도] × N 을 한 줄로, formation = 번호(1부터).
USV 가 돌면 대형도 같이 돈다 (지도 방위가 아니라 USV 헤딩 기준). 날면서 rd_cmd usv 의 1~9 로 바꾼다.

상대고도는 이륙 지점 기준이다 — 수면 높이에서 이륙해야 "수면 위 높이" 가 된다.
publish:=false 면 /cmd_vel 을 안 낸다 (짐벌은 aim 이 켜져 있으면 움직인다 — 지상 겨눔 시험용).
기록은 ~/drone_ws/usvlog/ 에 CSV.
"""
import math
import os
import sys
import time
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mav  # noqa: E402

CH_PITCH = 9


def clamp(v, lim):
    return max(-lim, min(lim, v))


def plan(me, hdg, usv, usv_hdg, usv_vel, lead, form, p):
    """한 tick 의 계산. ROS 없이 돈다.

    me = (lat, lon, 상대고도) · usv = (lat, lon) · usv_vel = (북, 동) m/s · lead = 외삽 초
    form = 대형 (앞, 오른쪽, 고도) — USV 기수 기준.
    돌려주는 것: 추정 USV 좌표, 목표 좌표, 목표까지 m, USV 까지 수평 m, 틸트 °,
                 /cmd_vel (x=앞 · y=오른쪽 · z=위 m/s, wz=시계 rad/s).
    """
    lat, lon, alt = me
    vn, ve = usv_vel
    # ponytail: 등속 외삽 — 급선회나 수 초 지연이면 틀린다. 그때 칼만으로
    est = mav.offset_to_gps(usv[0], usv[1], 0.0, vn * lead, ve * lead)
    goal = mav.offset_to_gps(est[0], est[1], usv_hdg, form[0], form[1])

    dist = mav.meters_between(lat, lon, goal[0], goal[1])
    rel = math.radians(mav.relative_bearing(mav.bearing_between(lat, lon, goal[0], goal[1]), hdg))
    speed = min(p("max_speed"), p("kp") * dist)
    h = math.radians(hdg)
    vx = speed * math.cos(rel) + vn * math.cos(h) + ve * math.sin(h)
    vy = speed * math.sin(rel) - vn * math.sin(h) + ve * math.cos(h)
    norm = math.hypot(vx, vy)
    if norm > p("max_speed"):
        vx, vy = vx * p("max_speed") / norm, vy * p("max_speed") / norm
    vz = clamp(p("kp_alt") * (form[2] - alt), p("max_vz"))

    yaw_err = mav.relative_bearing(mav.bearing_between(lat, lon, est[0], est[1]), hdg)
    wz = math.radians(clamp(p("kp_yaw") * yaw_err, p("max_yaw_rate")))

    horiz = mav.meters_between(lat, lon, est[0], est[1])
    tilt = max(-90.0, min(0.0, -math.degrees(math.atan2(max(alt, 0.0), horiz))))
    return est, goal, dist, horiz, tilt, (vx, vy, vz, wz)


def usv_velocity(hist):
    """[(t, lat, lon), ...] 맨 앞과 맨 뒤로 (북, 동) m/s. 두 점 사이가 짧으면 0."""
    if len(hist) < 2 or hist[-1][0] - hist[0][0] < 0.5:
        return 0.0, 0.0
    (t0, a0, o0), (t1, a1, o1) = hist[0], hist[-1]
    d = mav.meters_between(a0, o0, a1, o1)
    b = math.radians(mav.bearing_between(a0, o0, a1, o1))
    return d * math.cos(b) / (t1 - t0), d * math.sin(b) / (t1 - t0)


def formation(p):
    """formations 에서 formation 번 대형 (앞, 오른쪽, 고도). 번호가 없으면 None."""
    f, i = list(p("formations")), p("formation")
    return tuple(f[3 * i - 3:3 * i]) if len(f) % 3 == 0 and 1 <= i <= len(f) // 3 else None


# 대형 기본값은 임시 (9/18) — 1 뒤 10m · 2 왼쪽 10m, 고도 둘 다 10m
DEFAULTS = [("formations", [-10.0, 0.0, 10.0, 0.0, -10.0, 10.0]), ("formation", 1),
            ("lead_secs", 0.5),
            ("vel_window", 2.0), ("kp", 0.35), ("max_speed", 2.5), ("kp_alt", 0.35),
            ("max_vz", 1.0), ("kp_yaw", 0.8), ("max_yaw_rate", 30.0),
            ("lost_secs", 2.0), ("giveup_secs", 10.0), ("aim", True), ("publish", True)]


def selftest():
    p = dict(DEFAULTS, max_speed=10.0).get      # 자르지 않게 — 자르는 것은 따로 본다
    me, north20 = (37.0, 127.0, 10.0), mav.offset_to_gps(37.0, 127.0, 0.0, 20.0, 0.0)
    back = formation(p)
    assert back == (-10.0, 0.0, 10.0) and formation(dict(DEFAULTS, formation=3).get) is None
    est, goal, dist, horiz, tilt, (vx, vy, vz, wz) = plan(me, 0.0, north20, 0.0, (0, 0), 0.0, back, p)
    assert abs(horiz - 20.0) < 0.05 and abs(dist - 10.0) < 0.05, (horiz, dist)
    assert abs(tilt - -math.degrees(math.atan2(10, 20))) < 0.1, tilt           # ≈ -26.6°
    assert abs(vx - 0.35 * 10.0) < 0.02 and abs(vy) < 0.01 and vz == 0 and abs(wz) < 1e-3
    # USV 가 북으로 1 m/s — feedforward 가 앞으로 더해진다
    *_, (vx2, _, _, _) = plan(me, 0.0, north20, 0.0, (1.0, 0.0), 0.0, back, p)
    assert abs(vx2 - 4.5) < 0.02, vx2
    *_, (vx5, vy5, _, _) = plan(me, 0.0, north20, 0.0, (1.0, 0.0), 0.0, back, dict(DEFAULTS).get)
    assert abs(math.hypot(vx5, vy5) - 2.5) < 0.01                               # max_speed
    # 우리가 동쪽을 보고 있으면 USV(북)는 왼쪽 — vy<0, 왼쪽으로 돈다(wz<0)
    *_, (vx3, vy3, _, wz3) = plan(me, 90.0, north20, 0.0, (0, 0), 0.0, back, p)
    assert abs(vx3) < 0.01 and vy3 < -3.0 and wz3 < 0
    # 바로 위면 -90°, 외삽은 lead 만큼 간다
    assert plan(me, 0.0, me[:2], 0.0, (0, 0), 0.0, back, p)[4] == -90.0
    est4 = plan(me, 0.0, north20, 0.0, (2.0, 0.0), 1.5, back, p)[0]
    assert abs(mav.meters_between(*north20, *est4) - 3.0) < 0.01
    # 2번 대형(왼쪽 10m) — USV 가 북쪽을 보면 목표는 USV 의 서쪽, 동쪽을 보면 북쪽
    left = formation(dict(DEFAULTS, formation=2).get)
    g = plan(me, 0.0, north20, 0.0, (0, 0), 0.0, left, p)[1]
    assert abs(mav.bearing_between(*north20, *g) - 270.0) < 0.5
    g = plan(me, 0.0, north20, 90.0, (0, 0), 0.0, left, p)[1]
    b = mav.bearing_between(*north20, *g)
    assert min(b, 360.0 - b) < 0.5, b
    vn, ve = usv_velocity([(0.0, *me[:2]), (2.0, *mav.offset_to_gps(37.0, 127.0, 90.0, 4.0, 0.0))])
    assert abs(vn) < 0.01 and abs(ve - 2.0) < 0.01, (vn, ve)
    print("selftest ok")


if __name__ == "__main__" and "--selftest" in sys.argv:
    sys.exit(selftest())

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import NavSatFix  # noqa: E402
from std_msgs.msg import Bool, Empty, Float64, Float64MultiArray  # noqa: E402


class UsvFollowNode(Node):

    def __init__(self):
        super().__init__("rd_usv_follow")
        for name, val in DEFAULTS:
            self.declare_parameter(name, val)
        self.p = lambda name: self.get_parameter(name).value

        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_cmd = self.create_publisher(Float64MultiArray, "rd/command", 10)
        self.create_subscription(NavSatFix, "rd/fix", self.on_fix, 10)
        self.create_subscription(Float64, "rd/heading", lambda m: setattr(self, "hdg", m.data), 10)
        self.create_subscription(NavSatFix, "usv/fix", self.on_usv_fix, 10)
        self.create_subscription(Float64, "usv/heading",
                                 lambda m: setattr(self, "usv_hdg", m.data), 10)
        self.create_subscription(Float64, "usv/depth", lambda m: setattr(self, "depth", m.data), 10)
        self.create_subscription(Bool, "usv/follow", self.on_follow, 10)
        self.create_subscription(Empty, "usv/cancel", lambda _: self.stop("취소"), 10)

        self.me, self.t_me, self.hdg = None, 0.0, None
        self.usv_hist = deque()          # (t, lat, lon) — vel_window 초
        self.usv_hdg, self.depth = None, float("nan")
        self.armed = False
        self.tilt_sent, self.t_tilt = None, 0.0
        self.form_no = None
        self.csv = None
        self.t_log = 0.0
        self.create_timer(0.05, self.tick)   # 20Hz
        self.get_logger().info(
            "대기 중 — /usv/follow true 로 시작"
            + ("" if self.p("publish") else "  [publish:=false — /cmd_vel 을 내지 않는다]"))

    # ------------------------------------------------------------- 수신
    def on_fix(self, m):
        self.me, self.t_me = (m.latitude, m.longitude, m.altitude), time.time()

    def on_usv_fix(self, m):
        now = time.time()
        self.usv_hist.append((now, m.latitude, m.longitude))
        while len(self.usv_hist) > 2 and now - self.usv_hist[0][0] > self.p("vel_window"):
            self.usv_hist.popleft()

    def on_follow(self, msg):
        if msg.data:
            self.start()
        else:
            self.stop("/usv/follow false")

    # ------------------------------------------------------------- 켜고 끄기
    def start(self):
        if self.armed:
            return
        if self.me is None or self.hdg is None:
            return self.get_logger().error("아직 우리 위치·기수가 없다 — link_node 확인")
        self.armed, self.tilt_sent = True, None
        os.makedirs(os.path.expanduser("~/drone_ws/usvlog"), exist_ok=True)
        path = os.path.expanduser(time.strftime("~/drone_ws/usvlog/%Y%m%d-%H%M%S.csv"))
        self.csv = open(path, "w")
        self.csv.write("t,form,lat,lon,alt,hdg,usv_lat,usv_lon,usv_hdg,usv_vn,usv_ve,usv_age,depth,"
                       "est_lat,est_lon,goal_lat,goal_lon,dist,horiz,tilt,vx,vy,vz,wz\n")
        self.form_no = None      # 첫 tick 에 대형을 로그로 남기게
        self.get_logger().info(f"▶ USV 추종 시작 — 최대 {self.p('max_speed'):.1f}m/s")

    def stop(self, why):
        if not self.armed:
            return
        if self.p("publish"):
            self.pub.publish(Twist())   # 0 을 한 번. 그 뒤는 link_node 가 1초 브레이크
        self.armed = False
        if self.csv is not None:
            self.csv.close()
            self.csv = None
        self.get_logger().info(f"■ 정지 — {why}")

    def aim(self, tilt):
        now = time.time()
        if not self.p("aim") or now - self.t_tilt < 0.5:
            return
        if self.tilt_sent is not None and abs(tilt - self.tilt_sent) < 2.0:
            return
        self.pub_cmd.publish(Float64MultiArray(
            data=[float(mav.CMD_DO_SET_SERVO), float(CH_PITCH), 1500.0 + tilt * 5.5]))
        self.tilt_sent, self.t_tilt = tilt, now

    # ------------------------------------------------------------------ 20Hz
    def tick(self):
        if not self.armed:
            return
        now = time.time()
        if now - self.t_me > 1.0:
            return self.stop("우리 좌표가 1초 넘게 안 온다")
        age = now - self.usv_hist[-1][0] if self.usv_hist else float("inf")
        if age > self.p("giveup_secs") or self.usv_hdg is None:
            return self.stop(f"USV 좌표·기수가 {self.p('giveup_secs'):.0f}초 넘게 없다")
        if age > self.p("lost_secs"):
            if self.p("publish"):
                self.pub.publish(Twist())          # 제자리 — 다시 오면 이어서 간다
            if now - self.t_log > 1.0:
                self.t_log = now
                self.get_logger().warning(f"USV 좌표 끊김 {age:.1f}초 — 제자리")
            return

        form = formation(self.p)
        if form is None:
            return self.stop(f"대형 {self.p('formation')} 번이 formations 에 없다")
        if self.form_no != self.p("formation"):
            self.form_no = self.p("formation")
            self.get_logger().info(f"대형 {self.form_no} — USV 기준 앞 {form[0]:+.0f}m · "
                                   f"오른쪽 {form[1]:+.0f}m · 고도 {form[2]:.0f}m")

        _, ulat, ulon = self.usv_hist[-1]
        vn, ve = usv_velocity(self.usv_hist)
        est, goal, dist, horiz, tilt, (vx, vy, vz, wz) = plan(
            self.me, self.hdg, (ulat, ulon), self.usv_hdg, (vn, ve),
            age + self.p("lead_secs"), form, self.p)

        if self.p("publish"):
            cmd = Twist()
            cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.z = vx, vy, vz, wz
            self.pub.publish(cmd)
        self.aim(tilt)

        lat, lon, alt = self.me
        self.csv.write(f"{now:.2f},{self.form_no},{lat:.7f},{lon:.7f},{alt:.2f},{self.hdg:.1f},"
                       f"{ulat:.7f},{ulon:.7f},{self.usv_hdg:.1f},{vn:.2f},{ve:.2f},{age:.2f},"
                       f"{self.depth:.2f},{est[0]:.7f},{est[1]:.7f},{goal[0]:.7f},{goal[1]:.7f},"
                       f"{dist:.2f},{horiz:.2f},{tilt:.1f},{vx:.2f},{vy:.2f},{vz:.2f},{wz:.3f}\n")
        if now - self.t_log > 1.0:
            self.t_log = now
            self.get_logger().info(
                f"USV {horiz:4.1f}m (속도 {math.hypot(vn, ve):.1f}m/s) · 목표까지 {dist:4.1f}m · "
                f"대형 {self.form_no} · 고도 {alt:.1f}→{form[2]:.1f} · 틸트 {tilt:+.0f}° · "
                f"cmd vx {vx:+.2f} vy {vy:+.2f} vz {vz:+.2f} wz {math.degrees(wz):+.0f}°/s")


def main():
    rclpy.init()
    node = UsvFollowNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop("노드 종료")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
