"""링크 노드 — UDP 14550 을 혼자 잡고 앱과 양방향으로 말한다.

    ros2 run rd_bridge link_node
    ros2 run rd_bridge link_node --ros-args -p dry_run:=true     # 전송 없이 변환만

다운링크 (앱 → 토픽, 새 값이 올 때만 발행 — 주기는 토픽.md)
    /gcs/daiv/rd/fix       NavSatFix         위경도 · altitude 에 **상대고도**
    /gcs/daiv/rd/rel_alt   Float32           상대고도 m
    /gcs/daiv/rd/attitude  Vector3Stamped    roll · pitch · yaw **deg**
    /gcs/daiv/rd/heading   Float64           기수 deg (0=북, 시계)
    /gcs/daiv/rd/velocity  TwistStamped      대지속도 **ENU** m/s (z>0 상승)
    /gcs/daiv/rd/gspeed    Float32           대지속도 크기 m/s
    /gcs/daiv/rd/battery   BatteryState      전압 · 전류 (patches/03 필요)
    /gcs/daiv/rd/ack       Int32MultiArray   [명령번호, 결과]

업링크 (토픽 → 앱)
    /gcs/daiv/cmd_vel      Twist             → #84, 20Hz. **앱 규약**: x>0 앞 · y>0 오른쪽 · z>0 위 ·
                                      angular.z>0 시계(rad/s). REP-103 이 아니다
    /gcs/daiv/rd/command   Float64MultiArray [명령번호, p1..p7] → COMMAND_LONG

/gcs/daiv/cmd_vel 이 오기 전에는 #84 를 한 발도 안 쏜다 — 앱은 #84 를 받는 순간 Virtual Stick 을
켜고, 그동안 RC 스틱이 안 먹는다.
    IDLE → (Twist 옴) → ACTIVE 20Hz → (0.3초 끊김) → BRAKE 0 을 1초 → IDLE 송신 정지
    NAV_LAND(21) · NAV_TAKEOFF(22) 를 받으면 **그 자리에서 IDLE** — BRAKE 의 0 을 더 쏘지 않는다.
      2026-10-01: mission 의 착륙 명령이 BRAKE 한가운데(진입 0.5초)에 떨어지고, 뒤따른 0 이
      착륙을 덮어서 7회 중 6회가 22초 뒤 재전송에서야 내려왔다. 착륙·이륙은 앱이 스스로
      기체를 세우는 명령이라 BRAKE 가 할 일을 대신한다.

⚠ IDLE 로 돌아가도 Virtual Stick 은 안 풀린다. RC 로 되돌리기: 이 노드 종료 → 스틱 놓고
  5초 → 앱 Safety 버튼, 안 되면 S 모드.
⚠ 14550 은 한 프로세스만 잡는다 — QGC·gps_capture.py 와 같이 못 쓴다.
⚠ 앱의 ACK 는 무조건 ACCEPTED 다. ACK 수는 "앱이 받았다"까지만 말해 준다.
"""
import math
import os
import struct
import sys
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped, Vector3Stamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus
from std_msgs.msg import Float32, Float64, Float64MultiArray, Int32MultiArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mav  # noqa: E402  (tools/mav.py 의 심링크)

IDLE, ACTIVE, BRAKE = "IDLE", "ACTIVE", "BRAKE"


class CmdVelNode(Node):

    def __init__(self):
        super().__init__("rd_link")

        # max_* 는 안전 캡(yaw 는 deg/s). scale_* 는 전부 1 — /gcs/daiv/cmd_vel 이 이미 앱 규약이다
        for name, val in [("port", 14550), ("rate", 20.0), ("deadman", 0.3), ("brake_secs", 1.0),
                          ("max_vx", 2.5), ("max_vy", 2.5), ("max_vz", 1.5), ("max_yaw_rate", 75.0),
                          ("scale_vx", 1.0), ("scale_vy", 1.0), ("scale_vz", 1.0), ("scale_yaw", 1.0),
                          ("dry_run", False), ("discover_secs", 6.0)]:
            self.declare_parameter(name, val)

        g = self.get_parameter
        self.rate = g("rate").value
        self.deadman = g("deadman").value
        self.brake_secs = g("brake_secs").value
        self.cap = (g("max_vx").value, g("max_vy").value,
                    g("max_vz").value, g("max_yaw_rate").value)
        self.scale = (g("scale_vx").value, g("scale_vy").value,
                      g("scale_vz").value, g("scale_yaw").value)
        self.dry_run = g("dry_run").value

        self.cmd = (0.0, 0.0, 0.0, 0.0)      # 클램프된 마지막 값 (yaw 는 deg/s)
        self.t_cmd = 0.0                      # 마지막 Twist 도착 시각
        self.t_brake = 0.0                    # BRAKE 로 들어간 시각
        self.state = IDLE
        self.sent = 0
        self.acks = 0
        self.clamped = 0
        self.link = None

        if self.dry_run:
            self.get_logger().info("[dry-run] 전송하지 않는다 — 변환 결과만 로그로 본다")
        else:
            self.link = mav.Link(port=g("port").value)
            self.get_logger().info(f"앱의 송신 포트를 찾는 중… (0.0.0.0:{g('port').value})")
            addr = self.link.discover(g("discover_secs").value)
            if addr:
                self.get_logger().info(f"앱 = {addr[0]}:{addr[1]}")
            else:
                # 앱이 늦게 켜져도 된다 — poll() 이 주소를 잡는다
                self.get_logger().warning(
                    "아직 아무것도 안 온다 — 앱이 켜지면 자동으로 붙는다. "
                    "안 붙으면 ./tools/link_check.sh 로 T0 부터")

        # 다운링크 토픽
        self.pub_fix = self.create_publisher(NavSatFix, "gcs/daiv/rd/fix", 10)
        self.pub_alt = self.create_publisher(Float32, "gcs/daiv/rd/rel_alt", 10)
        self.pub_att = self.create_publisher(Vector3Stamped, "gcs/daiv/rd/attitude", 10)
        self.pub_hdg = self.create_publisher(Float64, "gcs/daiv/rd/heading", 10)
        self.pub_vel = self.create_publisher(TwistStamped, "gcs/daiv/rd/velocity", 10)
        self.pub_gsp = self.create_publisher(Float32, "gcs/daiv/rd/gspeed", 10)
        self.pub_bat = self.create_publisher(BatteryState, "gcs/daiv/rd/battery", 10)
        # 종류별로 마지막에 발행한 값의 수신 시각 — 새 값만 낸다
        self.seen = {"pos": 0.0, "att": 0.0, "hud": 0.0, "sys": 0.0}

        self.create_subscription(Twist, "gcs/daiv/cmd_vel", self.on_cmd, 10)
        self.create_subscription(Float64MultiArray, "gcs/daiv/rd/command", self.on_command, 10)
        self.pub_ack = self.create_publisher(Int32MultiArray, "gcs/daiv/rd/ack", 10)
        self.create_timer(1.0 / self.rate, self.tick)
        self.create_timer(2.0, self.report)
        self.get_logger().info(
            f"/gcs/daiv/cmd_vel 대기 중 — 오기 전에는 #84 를 쏘지 않는다. "
            f"캡 vx±{self.cap[0]} vy±{self.cap[1]} vz±{self.cap[2]} yaw±{self.cap[3]}°/s")

    # ------------------------------------------------------------ Twist 수신
    def on_cmd(self, msg):
        sx, sy, sz, syaw = self.scale
        vx = msg.linear.x * sx
        vy = msg.linear.y * sy
        vz = msg.linear.z * sz
        yaw = math.degrees(msg.angular.z) * syaw

        capped = [max(-c, min(c, v)) for c, v in zip(self.cap, (vx, vy, vz, yaw))]
        if any(abs(a - b) > 1e-6 for a, b in zip(capped, (vx, vy, vz, yaw))):
            self.clamped += 1

        self.cmd = tuple(capped)
        self.t_cmd = time.time()
        if self.state != ACTIVE:
            self.get_logger().info(f"{self.state} → ACTIVE (첫 Twist 수신)")
            self.state = ACTIVE

    # ------------------------------------------------------------ 명령 수신
    def on_command(self, msg):
        """[명령번호, p1..p7] → COMMAND_LONG (짐벌·이륙·착륙)."""
        d = list(msg.data)
        if not d:
            return
        cmd = int(d[0])
        if cmd in (mav.CMD_NAV_LAND, mav.CMD_NAV_TAKEOFF) and self.state != IDLE:
            # 먼저 입을 닫는다 — 이 뒤에 나가는 #84 0 이 착륙·이륙을 덮는다 (맨 위 주석)
            if self.state == ACTIVE:
                self.get_logger().warning(
                    f"⚠ ACTIVE 중에 {cmd} — 누가 아직 /gcs/daiv/cmd_vel 을 쏘고 있다. "
                    "다음 Twist 가 오면 다시 ACTIVE 가 되어 이 명령을 덮는다")
            self.get_logger().info(f"{self.state} → IDLE ({cmd} — 0 을 더 쏘지 않는다)")
            self.state = IDLE
        self.get_logger().info(f"COMMAND_LONG {cmd} {d[1:8]}")
        if self.link is not None:
            self.link.command_long(cmd, *d[1:8])

    # -------------------------------------------------------------- 20Hz 송신
    def tick(self):
        now = time.time()

        if self.state == ACTIVE and now - self.t_cmd > self.deadman:
            self.state, self.t_brake = BRAKE, now
            self.get_logger().info(
                f"ACTIVE → BRAKE (/gcs/daiv/cmd_vel 이 {self.deadman}초 이상 끊겼다) — "
                f"0 을 {self.brake_secs}초 쏜 뒤 입을 닫는다")
        if self.state == BRAKE and now - self.t_brake > self.brake_secs:
            self.state = IDLE
            self.get_logger().info(
                "BRAKE → IDLE (송신 정지). ⚠ Virtual Stick 은 이걸로 안 풀린다 — "
                "RC 로 넘기려면 이 노드를 끄고 앱의 Safety 버튼을 누를 것")

        if self.state == IDLE:
            # 하트비트만 — 모션을 안 일으키므로 Virtual Stick 을 안 켠다
            if self.link is not None:
                self.link.heartbeat()
                self._drain()
            return

        vx, vy, vz, yaw = (0.0, 0.0, 0.0, 0.0) if self.state == BRAKE else self.cmd

        if self.link is not None:
            self.link.set_velocity(vx, vy, vz, yaw)
            self.link.heartbeat()
            self._drain()
        self.sent += 1

    def _drain(self):
        """받은 것을 읽어 ACK 와 텔레메트리를 발행한다."""
        for mid, p in self.link.poll():
            if mid == 77:
                cmd, res = struct.unpack_from("<HB", p, 0)
                if cmd == 84:
                    self.acks += 1
                self.pub_ack.publish(Int32MultiArray(data=[cmd, res]))
        self.publish_telemetry()

    # -------------------------------------------------------------- 다운링크
    def publish_telemetry(self):
        """앱에서 **새로 온** 값만 낸다.

        같은 값을 재발행하면 링크가 끊겨도 "좌표가 살아 있다"로 보인다 —
        nav_node 의 좌표 신선도 검사(1초)가 이걸 전제로 한다.
        """
        if self.link is None:
            return
        t = self.link.tel
        stamp = self.get_clock().now().to_msg()

        # ── 위치·속도 (#33, 5Hz)
        if t.get("t_pos", 0.0) > self.seen["pos"]:
            self.seen["pos"] = t["t_pos"]
            fix = NavSatFix()
            fix.header.stamp = stamp
            fix.header.frame_id = "wgs84"
            fix.latitude = t["lat"]
            fix.longitude = t["lon"]
            fix.altitude = t.get("alt", 0.0)      # ⚠ 규격은 타원체고도지만 상대고도를 넣는다
            st = NavSatStatus()
            # GPS_FIX_TYPE: 0·1=없음 2=2D 3 이상=3D
            st.status = (NavSatStatus.STATUS_FIX if t.get("fix", 0) >= 2
                         else NavSatStatus.STATUS_NO_FIX)
            st.service = NavSatStatus.SERVICE_GPS
            fix.status = st
            self.pub_fix.publish(fix)

            m = Float32(); m.data = float(t.get("alt", 0.0)); self.pub_alt.publish(m)

            if "vn" in t:
                v = TwistStamped()
                v.header.stamp = stamp
                v.header.frame_id = "enu"
                v.twist.linear.x = t["ve"]          # 동  ← NED 의 east
                v.twist.linear.y = t["vn"]          # 북  ← NED 의 north
                v.twist.linear.z = -t["vd"]         # 위  ← NED 의 down 을 뒤집는다
                self.pub_vel.publish(v)

        # ── 자세 (#30, 10Hz)
        if t.get("t_att", 0.0) > self.seen["att"]:
            self.seen["att"] = t["t_att"]
            a = Vector3Stamped()
            a.header.stamp = stamp
            a.header.frame_id = "body"
            a.vector.x = float(t.get("roll", 0.0))      # deg
            a.vector.y = float(t.get("pitch", 0.0))     # deg
            a.vector.z = float(t.get("yaw", 0.0))       # deg (0=북, 시계방향)
            self.pub_att.publish(a)
            m = Float64(); m.data = float(t.get("yaw", 0.0)); self.pub_hdg.publish(m)

        # ── 대지속도 (#74, 10Hz)
        if t.get("t_hud", 0.0) > self.seen["hud"]:
            self.seen["hud"] = t["t_hud"]
            m = Float32(); m.data = float(t.get("gspeed", 0.0)); self.pub_gsp.publish(m)

        # ── 배터리 (#1, 1Hz)
        if t.get("t_sys", 0.0) > self.seen["sys"]:
            self.seen["sys"] = t["t_sys"]
            b = BatteryState()
            b.header.stamp = stamp
            b.voltage = float(t.get("volt", 0.0))
            b.current = float(t.get("amp", 0.0))
            pct = t.get("batt_pct", -1)
            b.percentage = float(pct) / 100.0 if pct is not None and pct >= 0 else float("nan")
            b.present = t.get("volt", 0.0) > 0.0
            self.pub_bat.publish(b)

    # ------------------------------------------------------------------ 상태
    def report(self):
        if self.state == IDLE and self.sent == 0 and not (self.link and self.link.tel):
            return
        t = self.link.tel if self.link else {}
        alt = t.get("alt")
        hdg = t.get("yaw")
        vx, vy, vz, yaw = self.cmd if self.state == ACTIVE else (0.0, 0.0, 0.0, 0.0)
        sats, volt = t.get("sats"), t.get("volt")
        self.get_logger().info(
            f"[{self.state}] vx={vx:+.2f} vy={vy:+.2f} vz={vz:+.2f} yaw={yaw:+.1f}°/s "
            f"· 보냄{self.sent} ACK{self.acks}"
            f"{f' · 캡에 걸림{self.clamped}' if self.clamped else ''}"
            f" · 고도{'?' if alt is None else f'{alt:.1f}m'}"
            f" 기수{'?' if hdg is None else f'{hdg:.0f}°'}"
            f"{f' 위성{sats}' if sats is not None else ''}"
            f"{f' {volt:.1f}V' if volt else ''}")

    # ------------------------------------------------------------------ 종료
    def shutdown(self):
        """끝낼 때 0 을 1초 쏜다."""
        if self.link is None or self.sent == 0:
            return
        self.get_logger().info("정지 명령 송출(1초)")
        t0 = time.time()
        while time.time() - t0 < 1.0:
            self.link.set_velocity(0, 0, 0, 0)
            time.sleep(1.0 / self.rate)


def main():
    rclpy.init()
    node = CmdVelNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        print(f"\n총 송신 {node.sent}회 · ACK(84) {node.acks}회")
        if node.sent and node.acks == 0:
            print("⚠ ACK 가 하나도 없었다 — 앱이 #84 를 못 받고 있었을 수 있다")
        if node.sent:
            print("⚠ RC 로 조종권을 되돌리려면: 이 노드 종료 → 스틱 놓고 5초 → "
                  "앱의 Safety(자물쇠) 버튼")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
