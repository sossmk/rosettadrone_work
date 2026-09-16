#!/usr/bin/env python3
"""태그 추종 노드 — AprilTag 위로 가서 서고, 기수를 태그에 맞춘다 (짐벌 -90° 탑뷰 기준).

    ros2 launch rd_bridge bridge.launch.py tag:=true follow:=true nav:=false
    python3 tools/rd_cmd.py follow        # space 시작/정지 · 파라미터 튜닝
    # 또는 직접: /tag/follow (Bool) true·false · /tag/cancel (Empty)

⚠ /tag/follow true 를 받기 전에는 /cmd_vel 을 안 낸다.
⚠ nav_node 와 동시에 켜지 않는다 — 둘 다 /cmd_vel 에 쓴다.

/tf 의 태그 위치는 **카메라 광학 좌표**다 (x=화면 오른쪽 · y=화면 아래 · z=거리≈고도).
탑뷰에서 화면 위가 기수 앞이라 앞 = sign_fwd·y (-1), 오른쪽 = sign_right·x (+1).
부호가 실기에서 뒤집혀 나오면 코드가 아니라 sign_* 를 바꾼다.

yaw 오차 = 태그 x축이 기수에서 돈 각도 - yaw_offset(90°). 똑바로 선 태그를 똑바로 찍으면
x축이 화면 오른쪽(+90°)을 가리키므로, 90° 를 빼서 "맞춰진 상태 = 0°" 로 만든다.

제어는 단순 P. 기록은 ~/drone_ws/taglog/ 에 CSV — **줄이 헤더뿐이면 태그를 한 번도 못 본 것이다.**
"""
import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Empty
from tf2_msgs.msg import TFMessage


def clamp(v, lim):
    return max(-lim, min(lim, v))


def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


def tag_axes(t):
    """geometry_msgs/Transform → (x, y, z, 태그 x축 벡터). 카메라 광학 좌표.

    tf2 를 쓰지 않는다 — 프레임이 카메라→태그 하나뿐이라 /tf 를 그냥 읽는 편이 짧다.
    """
    p, q = t.translation, t.rotation
    # 회전행렬의 첫 열 = 태그의 x축을 카메라 좌표로 본 것
    ax = (1.0 - 2.0 * (q.y * q.y + q.z * q.z),
          2.0 * (q.x * q.y + q.z * q.w),
          2.0 * (q.x * q.z - q.y * q.w))
    return p.x, p.y, p.z, ax


def follow_cmd(fwd, right, alt_err, yaw_err, p):
    """오차 → 속도 명령.

    fwd·right·alt_err 는 m (기체 기준), yaw_err 는 deg (시계가 +).
    돌려주는 것은 /cmd_vel 규약: x=앞 · y=오른쪽 · z=위 m/s, wz=시계 rad/s.
    """
    dist = math.hypot(fwd, right)
    if dist < p("deadband"):
        vx = vy = 0.0
    else:
        # min_speed — 가까워지면 P 명령이 기체가 반응 못 할 만큼 작아진다. 0 = 끔
        speed = min(p("max_speed"), max(p("min_speed"), p("kp") * dist))
        vx, vy = speed * fwd / dist, speed * right / dist
    vz = clamp(p("kp_alt") * -alt_err, p("max_vz"))
    if not p("yaw_enable") or abs(yaw_err) < p("yaw_deadband"):
        wz = 0.0
    else:
        wz = math.radians(clamp(p("kp_yaw") * yaw_err, p("max_yaw_rate")))
    return vx, vy, vz, wz


class TagFollowNode(Node):

    def __init__(self):
        super().__init__("rd_tag_follow")
        for name, val in [("tag_frame", "tag0"), ("kp", 0.35),
                          ("min_speed", 0.0), ("max_speed", 0.8),
                          ("kp_alt", 0.35), ("max_vz", 0.5), ("hold_alt", 3.0),
                          ("deadband", 0.3), ("lost_secs", 0.5), ("giveup_secs", 10.0),
                          ("sign_fwd", -1.0), ("sign_right", 1.0), ("sign_yaw", 1.0),
                          ("yaw_offset", 90.0),
                          ("yaw_enable", True), ("kp_yaw", 0.5), ("max_yaw_rate", 20.0),
                          ("yaw_deadband", 5.0), ("publish", True)]:
            self.declare_parameter(name, val)
        self.p = lambda name: self.get_parameter(name).value

        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 50)
        self.create_subscription(Bool, "tag/follow", self.on_follow, 10)
        self.create_subscription(Empty, "tag/cancel", lambda _: self.stop("취소"), 10)

        self.armed = False
        self.tag = None       # (fwd, right, alt_err, yaw_err, x, y, z)
        self.t_tag = 0.0
        self.csv = None
        self.t_log = 0.0
        self.create_timer(0.05, self.tick)   # 20Hz
        self.get_logger().info(
            f"대기 중 — /tag/follow true 로 시작. 태그 프레임 '{self.p('tag_frame')}'"
            + ("" if self.p("publish") else "  [publish:=false — /cmd_vel 을 내지 않는다]"))

    # ------------------------------------------------------------- 태그 수신
    def on_tf(self, msg):
        want = self.p("tag_frame")
        for tf in msg.transforms:
            if tf.child_frame_id != want:
                continue
            x, y, z, ax = tag_axes(tf.transform)
            sf, sr = self.p("sign_fwd"), self.p("sign_right")
            fwd, right = sf * y, sr * x
            yaw = math.degrees(math.atan2(sr * ax[0], sf * ax[1])) - self.p("yaw_offset")
            self.tag = (fwd, right, z - self.p("hold_alt"),
                        wrap180(self.p("sign_yaw") * yaw), x, y, z)
            self.t_tag = time.time()

    def on_follow(self, msg):
        if msg.data:
            self.start()
        else:
            self.stop("/tag/follow false")

    # ------------------------------------------------------------- 켜고 끄기
    def start(self):
        if self.armed:
            return
        self.armed = True
        os.makedirs(os.path.expanduser("~/drone_ws/taglog"), exist_ok=True)
        path = os.path.expanduser(time.strftime("~/drone_ws/taglog/%Y%m%d-%H%M%S.csv"))
        self.csv = open(path, "w")
        self.csv.write("t,x,y,z,fwd,right,alt_err,yaw_err,vx,vy,vz,wz\n")
        self.get_logger().info(
            f"▶ 추종 시작 — 고도 {self.p('hold_alt'):.1f}m 유지 · "
            f"최대 {self.p('max_speed'):.1f}m/s · yaw {'켬' if self.p('yaw_enable') else '끔'}")

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

    # ------------------------------------------------------------------ 20Hz
    def tick(self):
        if not self.armed:
            return
        age = time.time() - self.t_tag
        if self.tag is None or age > self.p("giveup_secs"):
            return self.stop(f"태그가 {self.p('giveup_secs'):.0f}초 넘게 안 보인다")
        if age > self.p("lost_secs"):
            if self.p("publish"):
                self.pub.publish(Twist())          # 제자리 — 다시 보이면 이어서 간다
            if time.time() - self.t_log > 1.0:
                self.t_log = time.time()
                self.get_logger().warning(f"태그 안 보임 {age:.1f}초 — 제자리")
            return

        fwd, right, alt_err, yaw_err, x, y, z = self.tag
        vx, vy, vz, wz = follow_cmd(fwd, right, alt_err, yaw_err, self.p)

        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.z = vx, vy, vz, wz
        if self.p("publish"):
            self.pub.publish(cmd)

        now = time.time()
        self.csv.write(f"{now:.2f},{x:.3f},{y:.3f},{z:.3f},{fwd:.3f},{right:.3f},"
                       f"{alt_err:.3f},{yaw_err:.1f},{vx:.2f},{vy:.2f},{vz:.2f},{wz:.3f}\n")
        if now - self.t_log > 1.0:
            self.t_log = now
            self.get_logger().info(
                f"앞 {fwd:+.2f} 오른쪽 {right:+.2f} 고도오차 {alt_err:+.2f}m "
                f"yaw {yaw_err:+.0f}° · cmd vx {vx:+.2f} vy {vy:+.2f} vz {vz:+.2f} "
                f"wz {math.degrees(wz):+.0f}°/s")


def main():
    rclpy.init()
    node = TagFollowNode()
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
