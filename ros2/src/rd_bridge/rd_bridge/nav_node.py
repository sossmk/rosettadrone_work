#!/usr/bin/env python3
"""항법 노드 — 경유점을 차례로 간다. 속도 = kp × 거리 (단순 P).

    ros2 run rd_bridge nav_node
    python3 tools/rd_cmd.py mission missions/square10.txt         # 파일로 (space 시작/중단)
    # 또는 직접 — alt 는 상대고도, 0 이하면 받는 순간의 고도 유지
    ros2 topic pub --once /gcs/daiv/nav/goal sensor_msgs/msg/NavSatFix '{latitude: 37.5, longitude: 126.9, altitude: 5.0}'
    ros2 topic pub --once /gcs/daiv/nav/goal_offset geometry_msgs/msg/Vector3 '{x: 5.0, y: 0.0, z: 0.0}'   # x=앞 y=오른쪽
    ros2 topic pub --once /gcs/daiv/nav/mission std_msgs/msg/Float64MultiArray '{data: [lat, lon, alt,  lat, lon, alt]}'
    ros2 topic pub --once /gcs/daiv/nav/mission_offset std_msgs/msg/Float64MultiArray '{data: [10,0,0,  10,10,0]}'
    ros2 topic pub --once /gcs/daiv/nav/cancel std_msgs/msg/Empty '{}'
    ros2 topic echo /gcs/daiv/nav/wp --field data          # 진행 [지금 번호, 전체]

· offset 은 전부 **받는 순간의 위치·기수 기준**이다 (앞 점 기준이 아니다)
· 경유점마다 감속해서 선 뒤 다음으로 간다. 중간점에서는 0 을 내지 않는다
  (내면 link_node 가 1초 브레이크를 건다)
· 새 목표가 오면 진행 중인 미션을 버린다
· 좌표가 1초 넘게 안 오면 멈춘다. 이륙은 하지 않는다
· 기록은 ~/drone_ws/navlog/ 에 미션당 CSV 하나

기수·카메라 (2026-10-01, sortie 복귀용으로 넣었다 — 경유점 비행에도 똑같이 걸린다):
· face_goal  기수를 목표 방위로 돌린다. wz = kp_yaw × 방위오차, max_yaw_rate 로 자른다.
             이동 속도는 매 tick 지금 기수 기준으로 다시 풀어서 도는 중에도 직선으로 간다.
             face_radius 안에서는 돌리지 않는다 — 가까우면 방위가 GPS 잡음에 크게 흔들린다
· aim        짐벌 틸트를 목표 지점의 **지면**(이륙 지면 높이 = 상대고도 0)에 겨눈다.
             tilt = −atan2(지금 상대고도, 남은 수평거리). 2° 넘게 바뀔 때만, 0.5초에 한 번.
             yaw 채널은 안 건드린다 (카메라 = 기수 전제 — usv_follow_node 가 켤 때 맞춰 둔다).
             sortie 에서는 station 도착 뒤 mission_node 가 태그정렬용 −90° 로 바꾼다
  끄기: ros2 param set /rd_nav face_goal false · ros2 param set /rd_nav aim false
"""
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Empty, Float64, Float64MultiArray, Int32MultiArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mav  # noqa: E402


CH_PITCH = 9                      # usv_follow_node · mission_node 와 같은 짐벌 채널


def face_and_tilt(rel, dist, alt, p):
    """(wz rad/s, 틸트 °). rel = 목표 방위 − 기수 (°, 오른쪽 +). ROS 없이 돈다."""
    wz = 0.0
    if p("face_goal") and dist > p("face_radius"):
        lim = p("max_yaw_rate")
        wz = math.radians(max(-lim, min(lim, p("kp_yaw") * rel)))
    tilt = max(-90.0, min(0.0, -math.degrees(math.atan2(max(alt, 0.0), dist))))
    return wz, tilt


def triples(data):
    d = list(data)
    return [tuple(d[i:i + 3]) for i in range(0, len(d), 3)] if d and len(d) % 3 == 0 else None


class NavNode(Node):

    def __init__(self):
        super().__init__("rd_nav")
        for name, val in [("kp", 0.5), ("max_speed", 4.5), ("kp_alt", 0.35),
                          ("max_vz", 1.5), ("arrive_radius", 1.0),
                          # 기수·카메라 — 맨 위 주석. kp_yaw · max_yaw_rate 는 usv_follow_node 와 같은 값
                          ("face_goal", True), ("kp_yaw", 0.8), ("max_yaw_rate", 30.0),
                          ("face_radius", 5.0), ("aim", True)]:
            self.declare_parameter(name, val)
        self.p = lambda name: self.get_parameter(name).value

        self.pub = self.create_publisher(Twist, "gcs/daiv/cmd_vel", 10)
        self.pub_wp = self.create_publisher(Int32MultiArray, "gcs/daiv/nav/wp", 10)
        self.pub_cmd = self.create_publisher(Float64MultiArray, "gcs/daiv/rd/command", 10)
        self.create_subscription(NavSatFix, "gcs/daiv/rd/fix", self.on_fix, 10)
        self.create_subscription(Float64, "gcs/daiv/rd/heading", self.on_hdg, 10)
        self.create_subscription(NavSatFix, "gcs/daiv/nav/goal", self.on_goal, 10)
        self.create_subscription(Vector3, "gcs/daiv/nav/goal_offset", self.on_offset, 10)
        self.create_subscription(Float64MultiArray, "gcs/daiv/nav/mission", self.on_mission, 10)
        self.create_subscription(Float64MultiArray, "gcs/daiv/nav/mission_offset",
                                 self.on_mission_offset, 10)
        self.create_subscription(Empty, "gcs/daiv/nav/cancel", lambda _: self.stop("취소"), 10)

        self.pos = None      # (lat, lon, alt)
        self.t_pos = 0.0
        self.hdg = None
        self.goals = []      # 남은 경유점 [(lat, lon, alt), ...] — 맨 앞이 지금 목표
        self.total = 0       # 이번 미션의 경유점 수
        self.csv = None
        self.t_log = 0.0
        self.tilt_sent, self.t_tilt = None, 0.0
        self.create_timer(0.05, self.tick)   # 20Hz
        self.get_logger().info("대기 중 — /gcs/daiv/nav/goal · goal_offset · mission · mission_offset")

    def on_fix(self, m):
        self.pos = (m.latitude, m.longitude, m.altitude)
        self.t_pos = time.time()

    def on_hdg(self, m):
        self.hdg = m.data

    # ------------------------------------------------------------- 목표 받기
    def on_goal(self, m):
        self.start([(m.latitude, m.longitude, m.altitude)])

    def on_offset(self, m):
        self.start_offset([(m.x, m.y, m.z)])

    def on_mission(self, m):
        pts = triples(m.data)
        if pts is None:
            return self.get_logger().error("/gcs/daiv/nav/mission 은 [lat, lon, alt] × N 이어야 한다")
        self.start(pts)

    def on_mission_offset(self, m):
        pts = triples(m.data)
        if pts is None:
            return self.get_logger().error("/gcs/daiv/nav/mission_offset 은 [앞, 오른쪽, alt] × N 이어야 한다")
        self.start_offset(pts)

    def start_offset(self, pts):
        if self.pos is None or self.hdg is None:
            return self.get_logger().error("아직 위치·기수가 없다 — link_node 확인")
        lat, lon = self.pos[0], self.pos[1]
        self.start([mav.offset_to_gps(lat, lon, self.hdg, f, r) + (z,) for f, r, z in pts])

    def start(self, pts):
        if self.pos is None or self.hdg is None:
            return self.get_logger().error("아직 위치·기수가 없다 — link_node 확인")
        # 좌표 검증은 여기 한 군데 — rd_cmd.py mission·직접 topic pub·시퀀서가 전부 지나간다
        for lat, lon, _ in pts:
            why = mav.bad_fix(lat, lon)
            if why:
                return self.get_logger().error(f"경유점을 버린다 — {why} ({lat}, {lon})")
        self.close_csv()
        alt0 = self.pos[2]
        self.goals = [(lat, lon, alt if alt > 0 else alt0) for lat, lon, alt in pts]
        self.total = len(self.goals)

        self.get_logger().info(f"▶ 경유점 {self.total}개")
        prev = self.pos
        for i, (lat, lon, alt) in enumerate(self.goals, 1):
            leg = mav.meters_between(prev[0], prev[1], lat, lon)
            self.get_logger().info(f"   {i}. {lat:.7f},{lon:.7f} 고도 {alt:.1f}m — 구간 {leg:.1f}m")
            prev = (lat, lon)

        os.makedirs(os.path.expanduser("~/drone_ws/navlog"), exist_ok=True)
        path = os.path.expanduser(time.strftime("~/drone_ws/navlog/%Y%m%d-%H%M%S.csv"))
        self.csv = open(path, "w")
        self.csv.write("t,wp,lat,lon,alt,hdg,dist,vx,vy,vz,wz,tilt\n")
        self.tilt_sent = None      # 새 미션은 첫 tick 에 틸트를 한 번 낸다
        self.publish_wp()

    # ------------------------------------------------------------- 끝내기
    def stop(self, why):
        if not self.goals:
            return
        self.pub.publish(Twist())   # 0 을 한 번. 그 뒤는 link_node 가 1초 브레이크
        self.goals = []
        self.close_csv()
        self.publish_wp()
        self.get_logger().info(f"■ 정지 — {why}")

    def close_csv(self):
        if self.csv is not None:
            self.csv.close()
            self.csv = None

    def aim(self, tilt):
        """짐벌 틸트 — usv_follow_node.aim() 과 같은 간격 (2° · 0.5초). 앱이 덮어쓰기에 약하다."""
        now = time.time()
        if not self.p("aim") or now - self.t_tilt < 0.5:
            return
        if self.tilt_sent is not None and abs(tilt - self.tilt_sent) < 2.0:
            return
        self.pub_cmd.publish(Float64MultiArray(
            data=[float(mav.CMD_DO_SET_SERVO), float(CH_PITCH), 1500.0 + tilt * 5.5]))
        self.tilt_sent, self.t_tilt = tilt, now

    def wp_no(self):
        """지금 목표의 번호 (1부터). 미션이 없으면 0."""
        return self.total - len(self.goals) + 1 if self.goals else 0

    def publish_wp(self):
        self.pub_wp.publish(Int32MultiArray(data=[self.wp_no(), self.total if self.goals else 0]))

    # ------------------------------------------------------------------ 20Hz
    def tick(self):
        if not self.goals:
            return
        if time.time() - self.t_pos > 1.0:
            return self.stop("좌표가 1초 넘게 안 온다")

        lat, lon, alt = self.pos
        glat, glon, galt = self.goals[0]
        dist = mav.meters_between(lat, lon, glat, glon)
        alt_err = galt - alt
        n = self.wp_no()
        if dist < self.p("arrive_radius") and abs(alt_err) < self.p("arrive_radius"):
            if len(self.goals) == 1:
                return self.stop(f"도착 {n}/{self.total} (거리 {dist:.2f}m, 고도오차 {alt_err:+.2f}m)")
            # 중간점 — 목표만 바꾼다. 이번 tick 은 쉬어도 link_node 가 직전 값을 계속 쏜다
            self.goals.pop(0)
            self.publish_wp()
            return self.get_logger().info(f"✓ {n}/{self.total} 도착 (거리 {dist:.2f}m) → {n + 1}번으로")

        # ── P 제어
        rel = mav.relative_bearing(mav.bearing_between(lat, lon, glat, glon), self.hdg)
        speed = min(self.p("max_speed"), self.p("kp") * dist)
        cmd = Twist()
        cmd.linear.x = speed * math.cos(math.radians(rel))
        cmd.linear.y = speed * math.sin(math.radians(rel))    # 앱 규약: y>0 오른쪽
        cmd.linear.z = max(-self.p("max_vz"), min(self.p("max_vz"), self.p("kp_alt") * alt_err))
        cmd.angular.z, tilt = face_and_tilt(rel, dist, alt, self.p)    # 앱 규약: wz>0 시계
        self.pub.publish(cmd)
        self.aim(tilt)

        now = time.time()
        self.csv.write(f"{now:.2f},{n},{lat:.7f},{lon:.7f},{alt:.2f},{self.hdg:.1f},{dist:.2f},"
                       f"{cmd.linear.x:.2f},{cmd.linear.y:.2f},{cmd.linear.z:.2f},"
                       f"{cmd.angular.z:.3f},{tilt:.1f}\n")
        if now - self.t_log > 1.0:
            self.t_log = now
            self.publish_wp()      # 중간에 붙은 구독자도 진행을 보게
            self.get_logger().info(
                f"[{n}/{self.total}] 거리 {dist:4.1f}m · {mav.name_direction(rel)} · "
                f"고도 {alt:.1f}→{galt:.1f} · "
                f"cmd vx {cmd.linear.x:+.2f} vy {cmd.linear.y:+.2f} vz {cmd.linear.z:+.2f} "
                f"wz {math.degrees(cmd.angular.z):+.0f}°/s · 틸트 {tilt:.0f}°")


def main():
    rclpy.init()
    node = NavNode()
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
