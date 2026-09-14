#!/usr/bin/env python3
"""/cmd_vel 구독 노드 — geometry_msgs/Twist 를 MAVLink #84 로 바꿔 20Hz 로 쏜다.

    ros2 run rd_bridge cmd_vel_node
    ros2 run rd_bridge cmd_vel_node --ros-args -p dry_run:=true     # 전송 없이 변환만

    # 손으로 한 발 (0.3m/s 전진) — 노드가 300ms 데드맨으로 알아서 멈춘다
    ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}}'

⚠⚠ 기체가 움직인다. RC 모드 스위치 P · 프로펠러를 뗀 지상시험부터.

────────────────────────────────────────────────────────────────
① 축·부호 — ROS(REP-103) 와 이 앱의 규약이 다르다
────────────────────────────────────────────────────────────────
ROS REP-103 의 기체 기준계는 **FLU**(x=앞, y=**왼쪽**, z=위, angular.z>0=**반시계**).
앱이 받는 #84 의 의미는 9/9 SIM 실측으로 확정돼 있다(할일.md 제약 #2):
vx>0=앞 · vy>0=**오른쪽** · vz>0=**위** · yawRate>0=**시계**.

    linear.x   →  vx      그대로
    linear.y   →  vy      **부호 반전** (왼쪽 → 오른쪽)
    linear.z   →  vz      **그대로**
    angular.z  →  yawRate **부호 반전** (반시계 → 시계), rad/s → deg/s

⚠ 할일.md D 의 "vz 부호를 뒤집을 것" 은 **MAVLink NED 기준으로 쓴 메모**다. NED 는
z 가 아래가 양수니 ROS(z=위) 에서 오면 뒤집어야 맞지만, 이 앱은 그 필드를 이미
"위가 양수"로 읽는다(제약 #2 가 말하는 바로 그 뒤집힘) — 두 번 뒤집으면 도로
제자리다. **ROS→앱에서 실제로 뒤집히는 축은 vy 와 yaw 두 개이고, vz 는 아니다.**
(문서 쪽은 이 커밋에서 고쳤다.)

부호가 실기에서 다르게 나오면 코드를 고치지 말고 파라미터로 뒤집을 것:
    --ros-args -p scale_vy:=1.0

────────────────────────────────────────────────────────────────
② 언제 쏘고 언제 입을 닫는가 — 9/11 RC 사고와 직결된다
────────────────────────────────────────────────────────────────
앱은 #84 를 하나 받으면 startMotion() 에서 **setVirtualSticksEnabled(true)** 를
부른다(DroneModel.java:2257). Virtual Stick 이 켜져 있는 동안 DJI 는 **물리 RC
스틱을 무시**한다. 그래서 이 노드는 "0 이라도 계속 쏘는" 설계를 하지 않는다:

    IDLE   /cmd_vel 이 온 적 없다 → **한 발도 안 쏜다** (Virtual Stick 을 안 켠다)
    ACTIVE Twist 가 계속 온다     → 20Hz 로 그 값을 쏜다
    BRAKE  Twist 가 끊겼다        → 0 을 brake_secs(기본 1.0초) 동안 쏴서 세우고
    IDLE   그 뒤 **입을 닫는다**  → 앱의 200ms 데드맨이 모션을 끝낸다

⚠⚠ **입을 닫아도 Virtual Stick 은 안 풀린다.** 소스로 확인했다 — 데드맨 만료가
부르는 cancelMotion() 은 타이머만 죽이고(DroneModel.java:2279) 조종권을 안 돌려준다.
조종권을 RC 로 되돌리는 경로는 실질적으로 **앱 화면의 Safety(자물쇠) 버튼** 하나뿐이다
(MainActivity:824 → setSafetyEnabled(true) → setVirtualSticksEnabled(false)).
RC 스틱을 건드리는 경로는 자기가 세운 타이머에 자기가 걸려 실패한다(할일.md 제약 #20).

    RC 로 조종권을 되돌리는 순서:
      1. 이 노드를 끈다 (Ctrl+C — 끄지 않으면 50ms 뒤에 다시 빼앗아 간다)
      2. 스틱에서 손을 떼고 5초 기다린다 (timerIgnoreMavLink 가 풀려야 한다)
      3. 앱 화면의 Safety(자물쇠) 버튼을 누른다
    그래도 안 되면 S 모드로 전환 (9/11 에 실제로 이걸로 복구했다)

③ ACCEPTED 는 성공이 아니다(제약 #1). ACK 수는 "앱이 #84 를 받았다"까지만 말해준다.
"""
import math
import os
import struct
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mav  # noqa: E402  (tools/mav.py 의 심링크 — 링크 계층 L1 을 그대로 쓴다)

IDLE, ACTIVE, BRAKE = "IDLE", "ACTIVE", "BRAKE"


class CmdVelNode(Node):

    def __init__(self):
        super().__init__("rd_cmd_vel")

        self.declare_parameter("port", 14550)
        self.declare_parameter("rate", 20.0)
        self.declare_parameter("deadman", 0.3)
        self.declare_parameter("brake_secs", 1.0)
        self.declare_parameter("max_vx", 0.5)
        self.declare_parameter("max_vy", 0.5)
        self.declare_parameter("max_vz", 0.5)
        self.declare_parameter("max_yaw_rate", 30.0)     # deg/s
        self.declare_parameter("scale_vx", 1.0)
        self.declare_parameter("scale_vy", -1.0)         # ROS y=왼쪽 → 앱 vy=오른쪽
        self.declare_parameter("scale_vz", 1.0)          # 둘 다 위가 양수 — 반전 없음
        self.declare_parameter("scale_yaw", -1.0)        # ROS 반시계 → 앱 시계
        self.declare_parameter("dry_run", False)
        self.declare_parameter("discover_secs", 6.0)

        g = self.get_parameter
        self.rate = g("rate").value
        self.deadman = g("deadman").value
        self.brake_secs = g("brake_secs").value
        self.cap = (g("max_vx").value, g("max_vy").value,
                    g("max_vz").value, g("max_yaw_rate").value)
        self.scale = (g("scale_vx").value, g("scale_vy").value,
                      g("scale_vz").value, g("scale_yaw").value)
        self.dry_run = g("dry_run").value

        self.cmd = (0.0, 0.0, 0.0, 0.0)      # 앱 규약으로 변환·클램프된 마지막 값
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
                # 앱이 늦게 켜져도 된다. poll() 이 봉투에서 주소를 잡아준다(제약 #7)
                self.get_logger().warning(
                    "아직 아무것도 안 온다 — 앱이 켜지면 자동으로 붙는다. "
                    "안 붙으면 ./tools/link_check.sh 로 T0 부터")

        self.create_subscription(Twist, "cmd_vel", self.on_cmd, 10)
        self.create_timer(1.0 / self.rate, self.tick)
        self.create_timer(2.0, self.report)
        self.get_logger().info(
            f"/cmd_vel 대기 중. 한 발도 오기 전에는 아무것도 쏘지 않는다 "
            f"(Virtual Stick 을 미리 켜지 않기 위해서 — docstring ② 참고). "
            f"캡 vx±{self.cap[0]} vy±{self.cap[1]} vz±{self.cap[2]} yaw±{self.cap[3]}°/s")

    # ------------------------------------------------------------ Twist 수신
    def on_cmd(self, msg):
        sx, sy, sz, syaw = self.scale
        vx = msg.linear.x * sx
        vy = msg.linear.y * sy
        vz = msg.linear.z * sz
        yaw = math.degrees(msg.angular.z) * syaw

        capped = [
            max(-self.cap[0], min(self.cap[0], vx)),
            max(-self.cap[1], min(self.cap[1], vy)),
            max(-self.cap[2], min(self.cap[2], vz)),
            max(-self.cap[3], min(self.cap[3], yaw)),
        ]
        if any(abs(a - b) > 1e-6 for a, b in zip(capped, (vx, vy, vz, yaw))):
            self.clamped += 1

        self.cmd = tuple(capped)
        self.t_cmd = time.time()
        if self.state != ACTIVE:
            self.get_logger().info(f"{self.state} → ACTIVE (첫 Twist 수신)")
            self.state = ACTIVE

    # -------------------------------------------------------------- 20Hz 송신
    def tick(self):
        now = time.time()

        if self.state == ACTIVE and now - self.t_cmd > self.deadman:
            self.state, self.t_brake = BRAKE, now
            self.get_logger().info(
                f"ACTIVE → BRAKE (/cmd_vel 이 {self.deadman}초 이상 끊겼다) — "
                f"0 을 {self.brake_secs}초 쏜 뒤 입을 닫는다")
        if self.state == BRAKE and now - self.t_brake > self.brake_secs:
            self.state = IDLE
            self.get_logger().info(
                "BRAKE → IDLE (송신 정지). ⚠ Virtual Stick 은 이걸로 안 풀린다 — "
                "RC 로 넘기려면 이 노드를 끄고 앱의 Safety 버튼을 누를 것")

        if self.state == IDLE:
            # 하트비트만 유지한다. 앱의 GCS_TIMEOUT 은 2초이고, 하트비트는
            # 모션을 일으키지 않으므로 Virtual Stick 도 켜지 않는다.
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
        """받은 것을 읽어 텔레메트리·ACK 를 갱신한다. 앱 주소도 여기서 따라간다."""
        for mid, p in self.link.poll():
            if mid == 77:
                cmd, _ = struct.unpack_from("<HB", p, 0)
                if cmd == 84:
                    self.acks += 1

    # ------------------------------------------------------------------ 상태
    def report(self):
        if self.state == IDLE and self.sent == 0:
            return
        t = self.link.tel if self.link else {}
        alt = t.get("alt")
        hdg = t.get("yaw")
        vx, vy, vz, yaw = self.cmd if self.state == ACTIVE else (0.0, 0.0, 0.0, 0.0)
        self.get_logger().info(
            f"[{self.state}] vx={vx:+.2f} vy={vy:+.2f} vz={vz:+.2f} yaw={yaw:+.1f}°/s "
            f"· 보냄{self.sent} ACK{self.acks}"
            f"{f' · 캡에 걸림{self.clamped}' if self.clamped else ''}"
            f" · 고도{'?' if alt is None else f'{alt:.1f}m'}"
            f" 기수{'?' if hdg is None else f'{hdg:.0f}°'}")

    # ------------------------------------------------------------------ 종료
    def shutdown(self):
        """멈추는 명령을 확실히 남기고 끝낸다 (cmd_vel_drive.py 와 같은 마무리)."""
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
