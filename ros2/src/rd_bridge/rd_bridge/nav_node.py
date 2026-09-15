#!/usr/bin/env python3
"""항법 노드 — 앱 flyTo() 를 대체하는 단순 P 제어.

    ros2 run rd_bridge nav_node
    # 현재 위치 기준으로 북 5m (x=북 m, y=동 m, z=상대고도 m, z<=0 이면 고도 유지)
    ros2 topic pub --once /nav/goal_offset geometry_msgs/msg/Vector3 '{x: 5.0, y: 0.0, z: 0.0}'
    # 좌표로 (altitude = 상대고도)
    ros2 topic pub --once /nav/goal sensor_msgs/msg/NavSatFix '{latitude: 37.5, longitude: 126.9, altitude: 5.0}'
    # 중단
    ros2 topic pub --once /nav/cancel std_msgs/msg/Empty '{}'

앱 flyTo() (DroneModel.processFlyTo) 도 이것과 같다: 속도 = 0.35 × 거리, 목표 방향으로.
이륙은 하지 않는다. 기록은 ~/drone_ws/navlog/ 에 CSV 로 남는다.
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
from std_msgs.msg import Empty, Float64

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mav  # noqa: E402

R_EARTH = 6371000.0


class NavNode(Node):

    def __init__(self):
        super().__init__("rd_nav")
        for name, val in [("kp", 0.35), ("max_speed", 0.5), ("kp_alt", 0.35),
                          ("max_vz", 0.5), ("arrive_radius", 1.0)]:
            self.declare_parameter(name, val)
        self.p = lambda name: self.get_parameter(name).value

        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(NavSatFix, "rd/fix", self.on_fix, 10)
        self.create_subscription(Float64, "rd/heading", self.on_hdg, 10)
        self.create_subscription(NavSatFix, "nav/goal", self.on_goal, 10)
        self.create_subscription(Vector3, "nav/goal_offset", self.on_offset, 10)
        self.create_subscription(Empty, "nav/cancel", lambda _: self.stop("취소"), 10)

        self.pos = None      # (lat, lon, alt)
        self.t_pos = 0.0
        self.hdg = None
        self.goal = None     # (lat, lon, alt)
        self.csv = None
        self.t_log = 0.0
        self.create_timer(0.05, self.tick)   # 20Hz
        self.get_logger().info("대기 중 — /nav/goal_offset 또는 /nav/goal 로 목표를 주세요")

    def on_fix(self, m):
        self.pos = (m.latitude, m.longitude, m.altitude)
        self.t_pos = time.time()

    def on_hdg(self, m):
        self.hdg = m.data

    def on_goal(self, m):
        self.start(m.latitude, m.longitude, m.altitude)

    def on_offset(self, m):
        if self.pos is None:
            return self.get_logger().error("아직 위치가 없다 — link_node 확인")
        lat, lon, alt = self.pos
        glat = lat + math.degrees(m.x / R_EARTH)
        glon = lon + math.degrees(m.y / (R_EARTH * math.cos(math.radians(lat))))
        self.start(glat, glon, m.z if m.z > 0 else alt)

    def start(self, lat, lon, alt):
        if self.pos is None or self.hdg is None:
            return self.get_logger().error("아직 위치·기수가 없다 — link_node 확인")
        self.goal = (lat, lon, alt)
        d = mav.meters_between(self.pos[0], self.pos[1], lat, lon)
        self.get_logger().info(f"▶ 목표 {lat:.7f},{lon:.7f} 고도 {alt:.1f}m — 거리 {d:.1f}m")
        os.makedirs(os.path.expanduser("~/drone_ws/navlog"), exist_ok=True)
        path = os.path.expanduser(time.strftime("~/drone_ws/navlog/%Y%m%d-%H%M%S.csv"))
        self.csv = open(path, "w")
        self.csv.write("t,lat,lon,alt,hdg,dist,vx,vy,vz\n")

    def stop(self, why):
        if self.goal is None:
            return
        self.pub.publish(Twist())   # 0 을 한 번. 그 뒤는 link_node 가 1초 브레이크
        self.goal = None
        self.csv.close()
        self.get_logger().info(f"■ 정지 — {why}")

    def tick(self):
        if self.goal is None:
            return
        if time.time() - self.t_pos > 1.0:
            return self.stop("좌표가 1초 넘게 안 온다")

        lat, lon, alt = self.pos
        glat, glon, galt = self.goal
        dist = mav.meters_between(lat, lon, glat, glon)
        alt_err = galt - alt
        if dist < self.p("arrive_radius") and abs(alt_err) < self.p("arrive_radius"):
            return self.stop(f"도착 (거리 {dist:.2f}m, 고도오차 {alt_err:+.2f}m)")

        # ── P 제어
        rel = mav.relative_bearing(mav.bearing_between(lat, lon, glat, glon), self.hdg)
        speed = min(self.p("max_speed"), self.p("kp") * dist)
        cmd = Twist()
        cmd.linear.x = speed * math.cos(math.radians(rel))
        cmd.linear.y = -speed * math.sin(math.radians(rel))   # ROS y 는 왼쪽
        cmd.linear.z = max(-self.p("max_vz"), min(self.p("max_vz"), self.p("kp_alt") * alt_err))
        self.pub.publish(cmd)

        now = time.time()
        self.csv.write(f"{now:.2f},{lat:.7f},{lon:.7f},{alt:.2f},{self.hdg:.1f},{dist:.2f},"
                       f"{cmd.linear.x:.2f},{cmd.linear.y:.2f},{cmd.linear.z:.2f}\n")
        if now - self.t_log > 1.0:
            self.t_log = now
            self.get_logger().info(
                f"거리 {dist:4.1f}m · {mav.name_direction(rel)} · 고도 {alt:.1f}→{galt:.1f} · "
                f"cmd vx {cmd.linear.x:+.2f} vy {cmd.linear.y:+.2f} vz {cmd.linear.z:+.2f}")


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
