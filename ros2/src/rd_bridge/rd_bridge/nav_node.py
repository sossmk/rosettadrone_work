#!/usr/bin/env python3
"""항법 노드 — **유도 루프를 우리가 돌린다.** 앱의 `flyTo()` 를 대체한다.

    ros2 run rd_bridge nav_node
    # 목표를 준다 (상대고도 기준)
    ros2 topic pub --once /nav/goal sensor_msgs/msg/NavSatFix \
        '{latitude: 37.5665, longitude: 126.9780, altitude: 10.0}'
    # 중단
    ros2 topic pub --once /nav/cancel std_msgs/msg/Empty '{}'

구독 `/rd/fix` `/rd/heading` (link_node 가 낸다) · 발행 `/cmd_vel` `/nav/status` `/nav/distance`

────────────────────────────────────────────────────────────────
왜 `flyTo()` 를 안 쓰는가
────────────────────────────────────────────────────────────────
앱의 `flyTo()` 는 DJI 네이티브 웨이포인트 미션이 **아니다.** 매 틱 PID 로 속도를
다시 계산해 `setVelocities()` 를 부르는 것이고(`processFlyTo()`), 그 채널은 cmd_vel 과
**완전히 같다**(제약 #19). 즉 앱이 하는 일은 우리가 그대로 할 수 있다.

우리가 가져오면 얻는 것:
  · **이득·도착판정·감속 프로파일을 우리가 정한다.** 앱 것은 캡이 박혀 있다
    (수평 2.5m/s · 수직 3.0m/s · 요 75°/s) 그리고 고칠 수 없다
  · **태그 추종과 같은 루프에 들어간다.** 태그 상대위치(E·F)와 GPS 목표를 같은
    유도기에서 섞을 수 있다 — 정밀 착륙(G)은 이게 없으면 안 된다
  · **`MISSION_ITEM_INT` 경로를 안 탄다.** `target_system` 함정(제약 #18)도,
    미션 상태를 건드리는 `pauseMission()` 도 지나간다
  · **무슨 일이 일어나는지 보인다.** 앱 안 PID 는 로그가 없다

⚠ 물려받는 것은 그대로 물려받는다 — `#84` 를 쓰므로 **P 모드**(제약 #4)·**스틱 5초
  차단**(#5)·**Virtual Stick 조종권**(#21) 은 똑같이 적용된다.

────────────────────────────────────────────────────────────────
유도 법칙
────────────────────────────────────────────────────────────────
목표까지 거리·방위각을 재고(구면 근사), 기수 기준 상대각으로 바꿔 기체 좌표 속도를 만든다.

    dist, brg = 목표까지 거리·방위각
    rel       = brg - 기수            (0=앞, +90=오른쪽)
    speed     = min(max_speed, kp_pos × max(0, dist - coast))
    vx = speed·cos(rel)      vy = -speed·sin(rel)     ← ROS FLU 는 y 가 **왼쪽**
    vz = clamp(kp_alt × 고도오차)

⚠ `speed` 에서 **`coast` 를 빼는 이유**: 정지 명령 후에도 **0.6~0.8m 더 간다**
  (제약 #6, 9/9 실측). 그래서 목표보다 `coast` 앞에서 속도를 0 으로 만들고 **타력으로
  들어가게** 한다. 목표까지 끝까지 밀면 반드시 지나친다.

────────────────────────────────────────────────────────────────
안전 — 항법을 우리가 가져왔으므로 이건 이제 우리 책임이다
────────────────────────────────────────────────────────────────
1. **좌표가 오래됐으면 즉시 멈춘다** (`stale_secs`, 기본 0.6초). 우리 위치 추정으로
   나는 것이니 링크가 끊긴 채 계속 날리면 안 된다
2. **fix 가 없으면 목표를 거부한다** (`STATUS_NO_FIX)
3. **지오펜스** — 현재 위치에서 `max_range`(기본 50m) 보다 먼 목표는 거부하고,
   비행 중 그보다 멀어지면 중단한다
4. **진척이 없으면 중단한다** — `progress_secs` 동안 거리가 `progress_min` 만큼도
   안 줄면 중단. P 모드가 아니어서 명령이 **조용히 버려지는** 경우(제약 #4)가
   여기서 잡힌다. ACK 는 못 믿는다(제약 #1)
5. **최대 시간** (`max_secs`, 기본 120초)
6. **너무 낮은 목표 고도 거부** (`min_goal_alt`, 기본 2m)
7. 멈출 때는 0 을 짧게 쏜 뒤 **발행을 끊는다** — link_node 가 그 뒤를 알아서 한다

⚠⚠ 이 노드는 **이륙을 하지 않는다.** 이미 떠 있는 상태를 전제한다.
⚠⚠ 첫 실기는 짧은 거리·저속·저고도로. `max_speed` 기본값 0.5 m/s 를 올리지 말 것.
   그리고 **속도 캡은 두 군데 있다** — 여기(`max_speed`)와 `link_node`(`max_vx`·`max_vy`).
   link_node 쪽이 마지막 클램프이므로 올릴 때는 둘 다 올려야 한다.
"""
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Empty, Float32, Float64, String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mav  # noqa: E402  (거리·방위각 계산을 tools/mav.py 와 공유한다)

IDLE, RUNNING, ARRIVED, ABORT = "IDLE", "RUNNING", "ARRIVED", "ABORT"


class NavNode(Node):

    def __init__(self):
        super().__init__("rd_nav")

        self.declare_parameter("rate", 20.0)
        self.declare_parameter("kp_pos", 0.5)
        # ⚠ link_node 의 max_vx/max_vy(기본 0.5) 가 **마지막 클램프**다. 여기서 크게
        #   줘도 그쪽에서 깎인다(스모크 시험에서 1.0 을 줬는데 0.5 로 나갔다).
        #   올릴 때는 **두 곳을 같이** 올릴 것 — 이중 캡은 일부러 남겨 둔 안전장치다.
        self.declare_parameter("max_speed", 0.5)          # m/s — 올리지 말 것
        self.declare_parameter("coast", 0.7)              # 제약 #6 (0.6~0.8m)
        self.declare_parameter("kp_alt", 0.4)
        self.declare_parameter("max_climb", 0.5)
        self.declare_parameter("face_target", False)
        self.declare_parameter("kp_yaw", 0.5)
        self.declare_parameter("max_yaw_rate", 20.0)      # deg/s
        self.declare_parameter("arrive_radius", 1.5)      # m — coast 보다 커야 한다
        self.declare_parameter("alt_tol", 0.5)
        self.declare_parameter("arrive_hits", 7)          # 앱과 같은 값
        self.declare_parameter("stale_secs", 0.6)
        self.declare_parameter("max_range", 50.0)
        self.declare_parameter("min_goal_alt", 2.0)
        self.declare_parameter("progress_secs", 5.0)
        self.declare_parameter("progress_min", 0.5)
        self.declare_parameter("max_secs", 120.0)

        g = self.get_parameter
        self.p = {k: g(k).value for k in (
            "rate", "kp_pos", "max_speed", "coast", "kp_alt", "max_climb",
            "face_target", "kp_yaw", "max_yaw_rate", "arrive_radius", "alt_tol",
            "arrive_hits", "stale_secs", "max_range", "min_goal_alt",
            "progress_secs", "progress_min", "max_secs")}

        if self.p["arrive_radius"] <= self.p["coast"]:
            self.get_logger().warning(
                f"arrive_radius({self.p['arrive_radius']}) 가 coast({self.p['coast']}) "
                f"보다 크지 않다 — 타력 때문에 도착 판정이 안 될 수 있다")

        self.pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_st = self.create_publisher(String, "nav/status", 10)
        self.pub_d = self.create_publisher(Float32, "nav/distance", 10)

        self.create_subscription(NavSatFix, "rd/fix", self.on_fix, 10)
        self.create_subscription(Float64, "rd/heading", self.on_hdg, 10)
        self.create_subscription(NavSatFix, "nav/goal", self.on_goal, 10)
        self.create_subscription(Empty, "nav/cancel", self.on_cancel, 10)

        self.fix = None          # (lat, lon, alt, 받은시각, fix_ok)
        self.hdg = None
        self.goal = None         # (lat, lon, alt)
        self.state = IDLE
        self.hits = 0
        self.t_start = 0.0
        self.t_stop = 0.0        # 0 을 쏴서 세우는 구간의 끝
        self.d_best = None       # 진척 판정용
        self.t_best = 0.0
        self.d_now = None

        self.create_timer(1.0 / self.p["rate"], self.tick)
        self.create_timer(1.0, self.report)
        self.get_logger().info(
            f"대기 중. /nav/goal 로 목표를 주세요 (상대고도 기준). "
            f"max_speed={self.p['max_speed']}m/s · 지오펜스 {self.p['max_range']}m · "
            f"coast {self.p['coast']}m")

    # ------------------------------------------------------------------ 입력
    def on_fix(self, msg):
        ok = msg.status.status != NavSatStatus.STATUS_NO_FIX
        self.fix = (msg.latitude, msg.longitude, msg.altitude, time.time(), ok)

    def on_hdg(self, msg):
        self.hdg = msg.data

    def on_goal(self, msg):
        if self.fix is None or self.hdg is None:
            return self._refuse("텔레메트리가 아직 없다 — link_node 가 떠 있는지 확인")
        lat, lon, alt, t, ok = self.fix
        if not ok:
            return self._refuse("GPS fix 가 없다 (실내면 정상 — 야외에서 다시)")
        if msg.altitude < self.p["min_goal_alt"]:
            return self._refuse(
                f"목표 고도 {msg.altitude:.1f}m 가 하한 {self.p['min_goal_alt']}m 보다 낮다")
        d = mav.meters_between(lat, lon, msg.latitude, msg.longitude)
        if d > self.p["max_range"]:
            return self._refuse(
                f"목표가 {d:.0f}m 떨어져 있다 — 지오펜스 {self.p['max_range']:.0f}m 초과")

        self.goal = (msg.latitude, msg.longitude, msg.altitude)
        self.state = RUNNING
        self.hits = 0
        self.t_start = time.time()
        self.d_best, self.t_best = d, time.time()
        self.get_logger().info(
            f"▶ 목표 {msg.latitude:.7f},{msg.longitude:.7f} 고도 {msg.altitude:.1f}m "
            f"— 거리 {d:.1f}m · 방위 {mav.bearing_between(lat, lon, msg.latitude, msg.longitude):.0f}° "
            f"({mav.name_direction(mav.relative_bearing(mav.bearing_between(lat, lon, msg.latitude, msg.longitude), self.hdg))})")
        self._status("RUNNING")

    def on_cancel(self, _msg):
        if self.state == RUNNING:
            self.get_logger().warning("■ 취소 — 0 을 쏘고 멈춘다")
            self._stop(ABORT, "취소")
        else:
            self.get_logger().info("취소 요청 — 지금 비행 중이 아니다")

    def _refuse(self, why):
        self.get_logger().error(f"✗ 목표 거부: {why}")
        self._status(f"REFUSED: {why}")

    # ------------------------------------------------------------------ 정지
    def _stop(self, state, why):
        """0 을 0.5초 쏴서 세운 뒤 발행을 끊는다."""
        self.state = state
        self.t_stop = time.time() + 0.5
        self.goal = None
        self._status(f"{state}: {why}")

    def _status(self, text):
        m = String(); m.data = text; self.pub_st.publish(m)

    # ---------------------------------------------------------------- 20Hz
    def tick(self):
        now = time.time()

        # 정지 구간 — 0 을 쏘는 중
        if self.t_stop and now < self.t_stop:
            self.pub_cmd.publish(Twist())
            return
        if self.t_stop and now >= self.t_stop:
            self.t_stop = 0.0
            self.get_logger().info(f"발행 정지 ({self.state})")
            self.state = IDLE if self.state in (ARRIVED, ABORT) else self.state
            return

        if self.state != RUNNING:
            return

        # ── 안전 1. 좌표가 오래됐다
        lat, lon, alt, t_fix, ok = self.fix
        if now - t_fix > self.p["stale_secs"]:
            self.get_logger().error(
                f"✗ 좌표가 {now - t_fix:.1f}초 전 것이다 (한계 {self.p['stale_secs']}초) — 중단")
            return self._stop(ABORT, "좌표 끊김")
        if not ok:
            self.get_logger().error("✗ GPS fix 를 잃었다 — 중단")
            return self._stop(ABORT, "fix 상실")

        glat, glon, galt = self.goal
        dist = mav.meters_between(lat, lon, glat, glon)
        alt_err = galt - alt
        self.d_now = dist

        # ── 안전 3. 지오펜스
        if dist > self.p["max_range"]:
            self.get_logger().error(f"✗ 목표까지 {dist:.0f}m — 지오펜스 초과. 중단")
            return self._stop(ABORT, "지오펜스")

        # ── 도착 판정 (연속 hits 회)
        if dist <= self.p["arrive_radius"] and abs(alt_err) <= self.p["alt_tol"]:
            self.hits += 1
            if self.hits >= self.p["arrive_hits"]:
                self.get_logger().info(
                    f"✓ 도착 — 거리 {dist:.2f}m · 고도오차 {alt_err:+.2f}m · "
                    f"{now - self.t_start:.1f}초")
                return self._stop(ARRIVED, f"거리 {dist:.2f}m")
        else:
            self.hits = 0

        # ── 안전 4. 진척 없음
        if dist < (self.d_best - self.p["progress_min"]):
            self.d_best, self.t_best = dist, now
        elif now - self.t_best > self.p["progress_secs"]:
            self.get_logger().error(
                f"✗ {self.p['progress_secs']:.0f}초 동안 {self.p['progress_min']}m 도 "
                f"못 줄었다 (거리 {dist:.1f}m) — 중단. "
                f"**RC 모드 스위치가 P 인지 확인할 것** (제약 #4: 아니면 명령이 조용히 버려진다)")
            return self._stop(ABORT, "진척 없음")

        # ── 안전 5. 최대 시간
        if now - self.t_start > self.p["max_secs"]:
            self.get_logger().error(f"✗ {self.p['max_secs']:.0f}초 초과 — 중단")
            return self._stop(ABORT, "시간 초과")

        # ── 유도
        brg = mav.bearing_between(lat, lon, glat, glon)
        rel = mav.relative_bearing(brg, self.hdg)        # 0=앞, +90=오른쪽
        speed = min(self.p["max_speed"],
                    self.p["kp_pos"] * max(0.0, dist - self.p["coast"]))
        r = math.radians(rel)

        cmd = Twist()
        cmd.linear.x = speed * math.cos(r)
        cmd.linear.y = -speed * math.sin(r)              # ROS FLU: y 는 왼쪽
        cmd.linear.z = max(-self.p["max_climb"],
                           min(self.p["max_climb"], self.p["kp_alt"] * alt_err))
        if self.p["face_target"]:
            yaw_deg = max(-self.p["max_yaw_rate"],
                          min(self.p["max_yaw_rate"], self.p["kp_yaw"] * rel))
            cmd.angular.z = math.radians(-yaw_deg)       # ROS +z 는 반시계
        self.pub_cmd.publish(cmd)

    # ------------------------------------------------------------------ 상태
    def report(self):
        if self.state != RUNNING or self.d_now is None:
            return
        lat, lon, alt, _, _ = self.fix
        glat, glon, galt = self.goal
        brg = mav.bearing_between(lat, lon, glat, glon)
        rel = mav.relative_bearing(brg, self.hdg)
        self.get_logger().info(
            f"거리 {self.d_now:5.1f}m · {mav.name_direction(rel)}({rel:+.0f}°) · "
            f"고도 {alt:.1f}→{galt:.1f}m · 기수 {self.hdg:.0f}° · "
            f"{time.time() - self.t_start:.0f}초")
        m = Float32(); m.data = float(self.d_now); self.pub_d.publish(m)


def main():
    rclpy.init()
    node = NavNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.state == RUNNING:
            print("\n■ 종료 — 0 을 0.5초 쏘고 끝낸다")
            t0 = time.time()
            while time.time() - t0 < 0.5:
                node.pub_cmd.publish(Twist())
                time.sleep(1.0 / node.p["rate"])
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
