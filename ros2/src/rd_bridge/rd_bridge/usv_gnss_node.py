#!/usr/bin/env python3
"""USV GNSS 어댑터 — USV 항법장비(FOG/INS)의 raw 배열을 /usv/* 규약으로 바꾼다.

    ros2 launch rd_bridge bridge.launch.py usv:=true usv_gnss:=true nav:=false
    ros2 run rd_bridge usv_gnss_node --ros-args -r usv/raw_gnss:=/maiv/fog/raw_gnss
    python3 ros2/src/rd_bridge/rd_bridge/usv_gnss_node.py --selftest   # ROS 없이

가짜 GPS(tools/usv_sim.py) 가 있던 자리에 그대로 들어간다. usv_follow_node 는 한 줄도
안 고친다 — /usv/fix · /usv/heading 규약이 그대로고 앞단 공급자만 진짜로 바뀐다.

⚠ **usv_sim.py 와 동시에 띄우지 않는다.** /usv/fix 에 퍼블리셔가 둘이 되면 두 좌표가
  번갈아 들어가 추종이 두 점 사이를 오간다. `ros2 topic info /usv/fix` 로 1 인지 본다.

이 노드가 필요한 이유 — 저장소 원칙은 "이름이 다르면 코드가 아니라 remap"(토픽.md) 인데
remap 은 이름만 바꾼다. Float64MultiArray → NavSatFix 는 remap 으로 안 된다.

    /usv/raw_gnss  Float64MultiArray  ← launch 에서 /maiv/fog/raw_gnss 로 remap. 20 Hz
    /usv/fix       NavSatFix          위경도(deg)·타원체고도 m · 공분산
    /usv/heading   Float64            기수 ° (0=북, 시계)
    /usv/vel       TwistStamped       **ENU** m/s (x=동 y=북 z=위) — 내기만 한다

발행은 **RELIABLE(10)** 이다. usv_follow_node 는 BEST_EFFORT 로 받으니 아무거나 붙지만
rd_cmd.py 의 /usv/fix 구독이 RELIABLE 이라, BEST_EFFORT 로 내면 추종은 되는데 모니터
화면만 빈칸이 되어 사람을 헷갈리게 한다 (usv_sim.py 가 RELIABLE 인 이유도 이것).

**자리는 라벨에서 이름으로 찾는다.** 메시지의 layout.dim[0].label 에 필드 이름이 콤마로
들어 있다. 인덱스를 코드에 박으면 드라이버가 필드를 하나 끼워 넣었을 때 **조용히 틀린다** —
GPS 에서 조용히 틀리는 것이 제일 무섭다 (위도 0.001° ≈ 111m, missions/README.txt).
이름을 못 찾으면 *_index 파라미터로 물러서고, 그때는 경고를 찍는다.

배치 (22개, 20Hz, 라벨 2026-09-29 노드 로그로 확정 — echo 는 라벨을 잘라서 못 본다):
    [0] unix_seconds · [1] microseconds · [2] latitude_rad · [3] longitude_rad · [4] height_m
    [5..7] vel_north/east/down_mps · [8] latitude_std_m · [9] longitude_std_m · [10] height_std_m
    [11] tilt_rad · [12] heading_rad · [13] tilt_std_rad · [14] heading_std_rad · [15] gnss_fix_status
    [16] doppler_velocity_valid · [17] time_valid · [18] external_gnss · [19] tilt_valid
    [20] heading_valid · [21] floating_ambiguity_heading
  'head' 가 든 이름이 넷이라 뜰 때 "여럿이다 — hdg_index=12 를 쓴다" 경고가 나온다. 정상이다.
  ⚠ [15]·[20] 유효 플래그는 아직 안 본다 — 9/29 수렴 전(std 4~10m)에 기수가 0.0 으로 나왔다.

**기수 = 센싱값 + 90°** (heading_offset_deg 기본 90, 2026-09-29 실기로 확정).
  USV 의 GNSS 안테나 두 개가 선수에 **수직**으로 놓여 있어서 heading_rad 는 두 안테나를 잇는
  선의 방위다. 9/29 센싱 15~17° 일 때 실제 선수가 약 100° 였다 (눈으로 확인).
  보정 전 9/29 비행은 대형 1·2·3 이 좌·우·뒤가 아니라 **뒤·앞·오른쪽** 에 섰다.
  안테나를 다시 달면 코드가 아니라 파라미터로 고친다:
      ros2 param set /rd_usv_gnss heading_offset_deg 90.0
      ros2 param set /rd_usv_gnss heading_sign -1.0        # 회전 방향까지 반대일 때만

⚠ **좌표가 정밀하지 않다.** 실측 latitude_std 4.2m · longitude_std 6.5m — RTK 가 아니다.
  usv_sim 의 가짜 좌표는 오차가 0 이었다. usv_follow_node 는 이 좌표에 대형(기본 10m)을
  걸고 kp=0.35 로 속도를 내므로 목표점이 6m 흔들리면 명령이 그만큼 출렁인다. 더 나쁜 것은
  usv_velocity() 가 vel_window(2초) **양 끝 두 점만** 써서 이 잡음을 가짜 속도로 읽는다는
  점이다. 그래서 장비가 직접 주는 속도를 /usv/vel 로 같이 낸다 — 이번에는 **내기만 한다.**
  추종이 어느 쪽을 쓸지는 실기에서 둘을 비교해 보고 정한다.

수심은 이 경로로 오지 않는다 (raw_gnss 에 없다). /usv/depth 가 없으면 usv_follow_node 의
CSV 에 NaN 으로 남을 뿐 추종에는 영향이 없다.
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mav  # noqa: E402

# 라벨에서 찾을 이름. heading 은 아직 이름을 몰라 따로 다룬다 (HDG_HINT)
FIELDS = {"lat": "latitude_rad", "lon": "longitude_rad", "alt": "height_m",
          "vn": "vel_north_mps", "ve": "vel_east_mps", "vd": "vel_down_mps",
          "lat_std": "latitude_std_m", "lon_std": "longitude_std_m",
          "alt_std": "height_std_m"}
HDG_HINT = "head"          # 라벨에 이 글자가 든 이름이 있으면 그것이 기수다
NEED = ("lat", "lon", "hdg")      # 이것 셋이 없으면 한 줄을 통째로 버린다


def resolve(label, fallback, use_label=True):
    """layout 라벨 → 자리 dict. 돌려주는 것: (idx, [사람이 읽을 메모]).

    라벨에서 이름으로 찾고, 못 찾으면 fallback(= *_index 파라미터)을 그대로 쓴다.
    메모는 뜰 때 한 번 로그로 찍어 "무엇을 어디서 읽고 있는지" 를 눈에 보이게 하려는 것이다.
    """
    names = [s.strip() for s in label.split(",")] if (use_label and label) else []
    idx, notes = dict(fallback), []
    if not names:
        return idx, (["라벨이 없다 — *_index 파라미터를 그대로 쓴다"] if use_label else [])
    for key, want in FIELDS.items():
        if want in names:
            got = names.index(want)
            if got != fallback.get(key):
                notes.append(f"{want} 는 data[{got}] 다 ({key}_index={fallback.get(key)} 를 덮는다)")
            idx[key] = got
        else:
            notes.append(f"{want} 가 라벨에 없다 — data[{idx.get(key)}] 를 그대로 쓴다")
    hdg = [i for i, n in enumerate(names) if HDG_HINT in n.lower()]
    if len(hdg) == 1:
        if hdg[0] != fallback.get("hdg"):
            notes.append(f"기수는 '{names[hdg[0]]}' = data[{hdg[0]}] 다"
                         f" (hdg_index={fallback.get('hdg')} 를 덮는다)")
        idx["hdg"] = hdg[0]
    elif hdg:
        notes.append(f"라벨에 '{HDG_HINT}' 가 든 이름이 여럿이다 {[names[i] for i in hdg]}"
                     f" — hdg_index={fallback.get('hdg')} 를 쓴다")
    else:
        notes.append(f"라벨에 '{HDG_HINT}' 가 든 이름이 없다 — hdg_index={fallback.get('hdg')} 를 쓴다")
    return idx, notes


def pick(data, i):
    """data[i]. 자리가 없거나 숫자가 아니면 nan. std 처럼 없어도 되는 값에 쓴다."""
    if i is None or not (0 <= i < len(data)):
        return float("nan")
    v = float(data[i])
    return v if math.isfinite(v) else float("nan")


def parse(data, idx, hdg_sign=1.0, hdg_offset=0.0):
    """raw_gnss 한 줄 → dict, 못 읽으면 None. ROS 없이 돈다.

    rad → deg 와 기수 관례 보정만 한다. 좌표를 믿을지 말지는 mav.bad_fix() 가 판정한다
    (여기서 또 검사하면 두 군데가 서로 다르게 자란다).
    """
    if any(idx.get(k) is None or not (0 <= idx[k] < len(data)) for k in NEED):
        return None
    lat, lon = math.degrees(float(data[idx["lat"]])), math.degrees(float(data[idx["lon"]]))
    hdg = (hdg_sign * math.degrees(float(data[idx["hdg"]])) + hdg_offset) % 360.0
    return {"lat": lat, "lon": lon, "alt": pick(data, idx.get("alt")), "hdg": hdg,
            "vn": pick(data, idx.get("vn")), "ve": pick(data, idx.get("ve")),
            "vd": pick(data, idx.get("vd")),
            "std": (pick(data, idx.get("lat_std")), pick(data, idx.get("lon_std")),
                    pick(data, idx.get("alt_std")))}


# *_index 기본값 — 2026-09-22 실측 배치. 라벨이 있으면 라벨이 이긴다
DEFAULTS = [("use_label", True), ("hdg_index", 12),
            ("lat_index", 2), ("lon_index", 3), ("alt_index", 4),
            ("vn_index", 5), ("ve_index", 6), ("vd_index", 7),
            ("lat_std_index", 8), ("lon_std_index", 9), ("alt_std_index", 10),
            ("heading_sign", 1.0), ("heading_offset_deg", 90.0),   # 안테나가 선수에 수직 — 맨 위 주석
            ("publish_vel", True)]

# 2026-09-22 14:15:52.05 실측 한 줄. selftest 가 이것으로 검증한다 (pytest 는 이 저장소에 없다)
SAMPLE = [1790054152.0, 50000.0, 0.6318299083483848, 2.224992173277046, 218.66723669315095,
          0.0026671099476516247, -0.0016907055396586657, -0.005498912185430527,
          4.212218761444092, 6.489093780517578, 12.295125007629395, -0.559727132320404,
          1.8704694509506226, 0.17837265133857727, 0.17837265133857727,
          4.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0]
SAMPLE_LABEL = ("unix_seconds,microseconds,latitude_rad,longitude_rad,height_m,"
                "vel_north_mps,vel_east_mps,vel_down_mps,latitude_std_m,longitude_std_m")


def selftest():
    base = {"hdg": 12, "lat": 2, "lon": 3, "alt": 4, "vn": 5, "ve": 6, "vd": 7,
            "lat_std": 8, "lon_std": 9, "alt_std": 10}

    # 실측 한 줄이 그대로 풀리는가 (위경도는 1e-6° ≈ 0.1m 까지)
    g = parse(SAMPLE, base)
    assert abs(g["lat"] - 36.201187) < 1e-5, g["lat"]
    assert abs(g["lon"] - 127.482661) < 1e-5, g["lon"]
    assert abs(g["alt"] - 218.667) < 1e-3 and abs(g["hdg"] - 107.170) < 1e-3, g
    assert abs(g["std"][0] - 4.212) < 1e-3 and abs(g["std"][1] - 6.489) < 1e-3
    assert mav.bad_fix(g["lat"], g["lon"]) is None          # 진짜 좌표로 인정되는가

    # 기수 관례 보정 — ENU(0=동, 반시계) 였다면 (90 - 107.17) % 360
    assert abs(parse(SAMPLE, base, -1.0, 90.0)["hdg"] - 342.830) < 1e-3
    assert abs(parse(SAMPLE, base, 1.0, 300.0)["hdg"] - 47.170) < 1e-3    # 360 을 넘어 감긴다
    # 기본값 = 안테나 수직 보정. 9/29 센싱 16° → 선수 106° (실제 약 100°)
    d = dict(DEFAULTS)
    assert abs(parse(SAMPLE, base, d["heading_sign"], d["heading_offset_deg"])["hdg"] - 197.170) < 1e-3

    # 짧은 줄은 통째로 버린다 (기수 자리까지 없다)
    assert parse(SAMPLE[:12], base) is None
    assert parse([], base) is None
    # std 자리가 배열 밖이어도 버리지 않는다 — nan 으로 두고 좌표는 쓴다
    no_std = parse(SAMPLE, dict(base, lat_std=30, lon_std=31, alt_std=32))
    assert no_std is not None and all(math.isnan(v) for v in no_std["std"])
    assert abs(no_std["hdg"] - 107.170) < 1e-3 and abs(no_std["lat"] - 36.201187) < 1e-5

    # 라벨에서 이름으로 찾는다
    idx, notes = resolve(SAMPLE_LABEL, base)
    assert idx["lat"] == 2 and idx["lon"] == 3 and idx["alt"] == 4 and idx["vn"] == 5
    assert idx["hdg"] == 12 and notes            # height_std_m 이 라벨에 없다는 메모가 남는다
    # 앞에 필드가 하나 끼면 전부 한 칸 밀린다 — **인덱스를 박았으면 조용히 틀렸을 자리**
    shifted = resolve("seq," + SAMPLE_LABEL, base)[0]
    assert shifted["lat"] == 3 and shifted["lon"] == 4 and shifted["vn"] == 6
    assert parse([0.0] + SAMPLE, shifted)["lat"] == parse(SAMPLE, base)["lat"]
    # 라벨에 heading 이 있으면 hdg_index 를 덮는다
    assert resolve(SAMPLE_LABEL + ",roll_rad,heading_rad", base)[0]["hdg"] == 11
    # 라벨이 없거나 use_label:=false 면 파라미터를 그대로
    assert resolve("", base)[0] == base and resolve(SAMPLE_LABEL, base, False)[0] == base
    # 공백이 섞인 라벨도 읽는다
    assert resolve("unix_seconds, microseconds, latitude_rad", base)[0]["lat"] == 2

    # 못 믿을 좌표는 bad_fix 가 잡는다 — 이 노드가 따로 검사하지 않는 근거
    assert mav.bad_fix(0.0, 0.0) and mav.bad_fix(float("nan"), 127.0)
    # ⚠ 그러나 **rad 를 그대로 deg 로 쓰는 실수는 bad_fix 가 못 잡는다.**
    #   (0.63, 2.22) 는 기니만 앞바다 — 범위 안이고 0,0 도 아니라 그냥 통과한다.
    #   변환을 빠뜨리면 조용히 틀린다. 위의 실측값 검증이 그것을 막는 유일한 그물이다
    assert mav.bad_fix(SAMPLE[2], SAMPLE[3]) is None
    print("selftest ok")


if __name__ == "__main__" and "--selftest" in sys.argv:
    sys.exit(selftest())

import rclpy  # noqa: E402
from geometry_msgs.msg import TwistStamped  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import NavSatFix, NavSatStatus  # noqa: E402
from std_msgs.msg import Float64, Float64MultiArray  # noqa: E402


class UsvGnssNode(Node):

    def __init__(self):
        super().__init__("rd_usv_gnss")
        for name, val in DEFAULTS:
            self.declare_parameter(name, val)
        self.p = lambda name: self.get_parameter(name).value

        # 발행은 RELIABLE — rd_cmd.py 의 /usv/fix 구독이 RELIABLE 이다 (맨 위 주석)
        self.pub_fix = self.create_publisher(NavSatFix, "usv/fix", 10)
        self.pub_hdg = self.create_publisher(Float64, "usv/heading", 10)
        self.pub_vel = self.create_publisher(TwistStamped, "usv/vel", 10)
        # 센서 QoS(BEST_EFFORT)로 받는다 — RELIABLE 퍼블리셔와도 붙는다 (제공 ≥ 요구)
        self.create_subscription(Float64MultiArray, "usv/raw_gnss", self.on_raw,
                                 qos_profile_sensor_data)

        self.label = None            # 라벨이 바뀌면 다시 풀고 다시 찍는다
        self.idx = None
        self.n, self.dropped, self.t_log, self.last = 0, 0, 0.0, None
        self.get_logger().info(
            "대기 중 — /usv/raw_gnss 를 기다린다"
            "  (remap 예: -r usv/raw_gnss:=/maiv/fog/raw_gnss)")

    def fallback(self):
        return {k: self.p(f"{k}_index") for k in (*FIELDS, "hdg")}

    def on_raw(self, m):
        label = m.layout.dim[0].label if m.layout.dim else ""
        if label != self.label:
            self.label = label
            self.idx, notes = resolve(label, self.fallback(), self.p("use_label"))
            # 라벨 전문을 찍는다 — ros2 topic echo 는 이것을 잘라서 보여 준다
            self.get_logger().info(f"라벨({len(m.data)}개): {label or '(없다)'}")
            for note in notes:
                self.get_logger().warning(f"⚠ {note}")
            self.get_logger().info(
                "자리: " + " ".join(f"{k}=data[{v}]" for k, v in sorted(self.idx.items())))

        g = parse(m.data, self.idx, self.p("heading_sign"), self.p("heading_offset_deg"))
        if g is None:
            self.dropped += 1
            return self.get_logger().warning(
                f"한 줄을 버린다 — data 가 {len(m.data)} 개뿐이다 (자리 {self.idx})",
                throttle_duration_sec=2.0)
        why = mav.bad_fix(g["lat"], g["lon"])
        if why:
            self.dropped += 1
            # 내지 않는다. usv_follow_node 의 lost_secs → 제자리, giveup_secs → 정지가 받는다
            return self.get_logger().warning(f"좌표를 버린다 — {why}", throttle_duration_sec=2.0)

        now = self.get_clock().now()
        stamp = now.to_msg()
        fix = NavSatFix()
        fix.header.stamp, fix.header.frame_id = stamp, "wgs84"
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude, fix.longitude = g["lat"], g["lon"]
        fix.altitude = 0.0 if math.isnan(g["alt"]) else g["alt"]
        lat_s, lon_s, alt_s = g["std"]
        if not any(math.isnan(v) for v in (lat_s, lon_s, alt_s)):
            # ROS 규약은 ENU 순서다 — [동, 북, 위]
            fix.position_covariance = [lon_s ** 2, 0.0, 0.0, 0.0, lat_s ** 2, 0.0,
                                       0.0, 0.0, alt_s ** 2]
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub_fix.publish(fix)
        self.pub_hdg.publish(Float64(data=g["hdg"]))

        if self.p("publish_vel") and not any(math.isnan(g[k]) for k in ("vn", "ve", "vd")):
            vel = TwistStamped()
            vel.header.stamp, vel.header.frame_id = stamp, "enu"
            # NED → ENU. /gcs/daiv/rd/velocity 와 같은 규약이다 (x=동 y=북 z=위)
            vel.twist.linear.x, vel.twist.linear.y = g["ve"], g["vn"]
            vel.twist.linear.z = -g["vd"]
            self.pub_vel.publish(vel)

        self.n, self.last = self.n + 1, g
        t = time.time()
        if t - self.t_log > 1.0:
            self.t_log = t
            self.get_logger().info(
                f"{g['lat']:.7f}, {g['lon']:.7f}  고도 {g['alt']:6.1f}m  기수 {g['hdg']:5.1f}°"
                f"  std {lat_s:.1f}/{lon_s:.1f}m  속도 N{g['vn']:+.2f} E{g['ve']:+.2f}"
                f"  받은 줄 {self.n}" + (f"  버린 줄 {self.dropped}" if self.dropped else ""))


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
