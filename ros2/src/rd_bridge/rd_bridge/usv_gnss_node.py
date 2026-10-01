#!/usr/bin/env python3
"""USV 위치·기수 어댑터 — USV 쪽 토픽을 /usv/* 규약으로 바꾼다.

    ros2 launch rd_bridge bridge.launch.py usv:=true usv_gnss:=true nav:=false
    ros2 run rd_bridge usv_gnss_node --ros-args \\
        -r usv/navsatfix:=/maiv/hemisphere_gps/navsatfix -r usv/euler:=/maiv/fog/euler_orientation
    python3 ros2/src/rd_bridge/rd_bridge/usv_gnss_node.py --selftest   # ROS 없이

가짜 GPS(tools/usv_sim.py) 가 있던 자리에 그대로 들어간다. usv_follow_node · mission_node ·
rd_cmd 는 한 줄도 안 고친다 — /usv/fix · /usv/heading 규약이 그대로고 앞단 공급자만 바뀐다.

⚠ **usv_sim.py 와 동시에 띄우지 않는다.** /usv/fix 에 퍼블리셔가 둘이 되면 두 좌표가
  번갈아 들어가 추종이 두 점 사이를 오간다. `ros2 topic info /usv/fix` 로 1 인지 본다.

    /usv/navsatfix NavSatFix          ← launch 에서 /maiv/hemisphere_gps/navsatfix 로 remap. 10 Hz
    /usv/euler     Float32MultiArray  ← launch 에서 /maiv/fog/euler_orientation 으로 remap. ~100 Hz
                                        라벨 roll_rad,pitch_rad,heading_rad
    /usv/fix       NavSatFix          위경도 — 받은 것을 걸러서 그대로 다시 낸다
    /usv/heading   Float64            기수 ° (0=북, 시계). heading_hz(20) 로 최근 euler 값을

**위치는 hemisphere GPS 에서 받는다** (2026-10-01 부터). 그 전에는 FOG 의 /maiv/fog/raw_gnss
  (Float64MultiArray 22칸, 라벨로 자리 찾기)를 풀어서 냈는데, 10/1 에 raw_gnss 가 잘 안 나와서
  hemisphere 로 바꾸고 raw_gnss 파싱은 지웠다 — `git show 03795f6:ros2/src/rd_bridge/rd_bridge/usv_gnss_node.py`.
  hemisphere 는 이미 NavSatFix 라 remap 만으로 될 것 같지만 이 노드를 거치게 둔다:
  못 믿을 좌표(NO_FIX · 0,0 · NaN)를 여기서 한 번 거르고, 1초마다 받는 상태를 찍기 위해서다.
  ⚠ 10/1 실측 한 줄: altitude 0.0 · covariance 전부 0 (type 0 = UNKNOWN). 고도와 정밀도는
  이 경로로 안 온다 — 틸트 보정은 어차피 고정값 usv_below 라 상관없다. 정밀도는 실기에서 본다.

**기수는 euler_orientation 에서 읽는다** (2026-09-30 부터). 9/30 실기 20분 내내 raw_gnss 의
  두 안테나 기수가 0 이었고, 같은 때 euler heading_rad 는 살아 있었다. euler 는 FOG/INS 가 낸
  선체 자세라 안테나 배치와 상관없다 → **보정 0** (euler_heading_offset_deg).
  euler 가 euler_timeout 초 넘게 안 오면 /usv/heading 을 **내지 않는다** — 옛 값으로 대형을
  돌리느니 usv_follow_node 의 giveup_secs → 정지가 받게 한다. 좌표(/usv/fix)는 그대로 낸다.
      ros2 param set /rd_usv_gnss euler_heading_offset_deg 0.0
      ros2 param set /rd_usv_gnss heading_sign -1.0        # 회전 방향이 반대일 때만

발행은 **RELIABLE(10)** 이다. usv_follow_node 는 BEST_EFFORT 로 받으니 아무거나 붙지만
rd_cmd.py 의 /usv/fix 구독이 RELIABLE 이라, BEST_EFFORT 로 내면 추종은 되는데 모니터
화면만 빈칸이 되어 사람을 헷갈리게 한다 (usv_sim.py 가 RELIABLE 인 이유도 이것).

⚠ USV 속도는 이제 장비 값이 없다 — usv_follow_node 가 vel_window 초 양 끝 두 좌표로 낸다.
수심도 이 경로로 오지 않는다. /usv/depth 가 없으면 CSV 에 NaN 으로 남을 뿐이다.
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mav  # noqa: E402

EULER_HDG = "heading_rad"       # euler_orientation 라벨: roll_rad,pitch_rad,heading_rad
STATUS_NO_FIX = -1              # sensor_msgs/NavSatStatus — ROS 없이 selftest 하려고 숫자로 둔다


def pick(data, i):
    """data[i]. 자리가 없거나 숫자가 아니면 nan."""
    if i is None or not (0 <= i < len(data)):
        return float("nan")
    v = float(data[i])
    return v if math.isfinite(v) else float("nan")


def euler_index(label, fallback):
    """euler 라벨 → 기수 자리. 라벨에 heading_rad 가 없으면 fallback(euler_hdg_index)."""
    names = [s.strip() for s in label.split(",")] if label else []
    return names.index(EULER_HDG) if EULER_HDG in names else fallback


def euler_heading(data, i, hdg_sign=1.0, hdg_offset=0.0):
    """euler 한 줄 → 기수 °(0=북, 시계). 자리가 없거나 숫자가 아니면 None."""
    v = pick(data, i)
    return None if math.isnan(v) else (hdg_sign * math.degrees(v) + hdg_offset) % 360.0


def fix_problem(status, lat, lon):
    """hemisphere NavSatFix 를 버릴 이유, 쓸 만하면 None. 좌표 판정은 mav.bad_fix() 하나로."""
    if status <= STATUS_NO_FIX:
        return f"NO_FIX (status {status})"
    return mav.bad_fix(lat, lon)


DEFAULTS = [("heading_sign", 1.0), ("euler_hdg_index", 2),
            ("euler_heading_offset_deg", 0.0), ("euler_timeout", 1.0),
            ("heading_hz", 20.0)]

# 2026-09-30 실측 euler_orientation 한 줄 (Float32). 4.5454 rad = 260.43°
EULER_SAMPLE = [-0.014577262103557587, -0.0018347874283790588, 4.5453925132751465]
EULER_LABEL = "roll_rad,pitch_rad,heading_rad"
# 2026-10-01 실측 /maiv/hemisphere_gps/navsatfix 한 줄 (군산 안벽 앞) — status 0 · service 1
HEMI_SAMPLE = (0, 35.890544696666666, 126.51697957683334)


def selftest():
    # euler — 라벨에서 heading_rad 자리를 찾고, 기본 보정은 0 (선체 자세라 안테나 배치와 무관)
    d = dict(DEFAULTS)
    assert d["euler_heading_offset_deg"] == 0.0 and d["heading_sign"] == 1.0
    ei = euler_index(EULER_LABEL, 0)
    assert ei == 2 and euler_index("", 2) == 2 and euler_index("yaw,heading_rad", 9) == 1
    assert euler_index("roll_rad, pitch_rad, heading_rad", 0) == 2            # 공백이 섞여도
    assert abs(euler_heading(EULER_SAMPLE, ei) - 260.430) < 1e-2
    assert abs(euler_heading(EULER_SAMPLE, ei, 1.0, 180.0) - 80.430) < 1e-2     # 360 을 넘어 감긴다
    assert abs(euler_heading(EULER_SAMPLE, ei, -1.0, 0.0) - 99.570) < 1e-2      # 회전 방향 반대
    assert euler_heading(EULER_SAMPLE, 5) is None and euler_heading([float("nan")] * 3, 2) is None

    # hemisphere — 실측 한 줄은 통과, NO_FIX · 0,0 · NaN 은 버린다
    assert fix_problem(*HEMI_SAMPLE) is None
    assert fix_problem(STATUS_NO_FIX, *HEMI_SAMPLE[1:])
    assert fix_problem(0, 0.0, 0.0) and fix_problem(0, float("nan"), 126.5)
    assert fix_problem(2, *HEMI_SAMPLE[1:]) is None                            # GBAS_FIX 도 쓴다
    print("selftest ok")


if __name__ == "__main__" and "--selftest" in sys.argv:
    sys.exit(selftest())

import rclpy  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import NavSatFix  # noqa: E402
from std_msgs.msg import Float32MultiArray, Float64  # noqa: E402


class UsvGnssNode(Node):

    def __init__(self):
        super().__init__("rd_usv_gnss")
        for name, val in DEFAULTS:
            self.declare_parameter(name, val)
        self.p = lambda name: self.get_parameter(name).value

        # 발행은 RELIABLE — rd_cmd.py 의 /usv/fix 구독이 RELIABLE 이다 (맨 위 주석)
        self.pub_fix = self.create_publisher(NavSatFix, "usv/fix", 10)
        self.pub_hdg = self.create_publisher(Float64, "usv/heading", 10)
        # 센서 QoS(BEST_EFFORT)로 받는다 — RELIABLE 퍼블리셔와도 붙는다 (제공 ≥ 요구)
        self.create_subscription(NavSatFix, "usv/navsatfix", self.on_fix, qos_profile_sensor_data)
        # 기수 — FOG 자세. ~100Hz 로 오니 최근 값만 들고 있다가 heading_hz 로 낸다
        self.create_subscription(Float32MultiArray, "usv/euler", self.on_euler,
                                 qos_profile_sensor_data)
        self.create_timer(1.0 / self.p("heading_hz"), self.tick_heading)

        self.euler_label, self.euler_i = None, None
        self.euler_hdg, self.t_euler = None, 0.0
        self.hdg_sent, self.hdg_why = None, "euler 가 아직 안 왔다"
        self.n, self.dropped, self.t_log = 0, 0, 0.0
        self.get_logger().info(
            "대기 중 — /usv/navsatfix (위경도) · /usv/euler (기수) 를 기다린다"
            "  (remap 예: -r usv/navsatfix:=/maiv/hemisphere_gps/navsatfix"
            " -r usv/euler:=/maiv/fog/euler_orientation)")

    def on_euler(self, m):
        label = m.layout.dim[0].label if m.layout.dim else ""
        if label != self.euler_label:
            self.euler_label = label
            self.euler_i = euler_index(label, self.p("euler_hdg_index"))
            self.get_logger().info(f"euler 라벨: {label or '(없다)'} → 기수 data[{self.euler_i}]")
        h = euler_heading(m.data, self.euler_i, self.p("heading_sign"),
                          self.p("euler_heading_offset_deg"))
        if h is not None:
            self.euler_hdg, self.t_euler = h, time.monotonic()

    def tick_heading(self):
        if self.euler_hdg is None:
            self.hdg_sent, self.hdg_why = None, "euler 가 아직 안 왔다 (-r usv/euler:=/maiv/fog/euler_orientation)"
        elif (age := time.monotonic() - self.t_euler) > self.p("euler_timeout"):
            self.hdg_sent, self.hdg_why = None, f"euler 가 {age:.1f}초째 안 온다"
        else:
            self.hdg_sent = self.euler_hdg
            self.pub_hdg.publish(Float64(data=self.hdg_sent))
            return
        # 옛 기수로 대형을 돌리지 않는다 — usv_follow_node 의 giveup_secs → 정지가 받는다
        self.get_logger().warning(f"기수를 안 낸다 — {self.hdg_why}", throttle_duration_sec=2.0)

    def on_fix(self, m):
        why = fix_problem(m.status.status, m.latitude, m.longitude)
        if why:
            self.dropped += 1
            # 내지 않는다. usv_follow_node 의 lost_secs → 제자리, giveup_secs → 정지가 받는다
            return self.get_logger().warning(f"좌표를 버린다 — {why}", throttle_duration_sec=2.0)

        fix = NavSatFix()
        # 받은 시각으로 다시 찍는다 — USV 시계와 우리 시계가 어긋나도 나이 계산이 안 틀린다
        fix.header.stamp, fix.header.frame_id = self.get_clock().now().to_msg(), "wgs84"
        fix.status = m.status
        fix.latitude, fix.longitude = m.latitude, m.longitude
        fix.altitude = 0.0 if math.isnan(m.altitude) else m.altitude
        fix.position_covariance = m.position_covariance
        fix.position_covariance_type = m.position_covariance_type
        self.pub_fix.publish(fix)

        self.n += 1
        t = time.time()
        if t - self.t_log > 1.0:
            self.t_log = t
            self.get_logger().info(
                f"{m.latitude:.7f}, {m.longitude:.7f}  status {m.status.status}  기수 "
                + (f"{self.hdg_sent:5.1f}°" if self.hdg_sent is not None else f"-  ({self.hdg_why})")
                + f"  받은 줄 {self.n}" + (f"  버린 줄 {self.dropped}" if self.dropped else ""))


def main():
    rclpy.init()
    node = UsvGnssNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
