#!/usr/bin/env python3
"""DT0503 bag 두 개 -> outputs/ 의 CSV + 프레임 JPEG.

    python scripts/extract_bags.py --frames                       # 기본 DT0503
    python scripts/extract_bags.py --drone data/drone_XX --usv data/usv_XX --out outputs_XX --frames

내보내는 것 (모든 t 는 bag 기록 시각 = UNIX 초, KST 로 같이 적는다)
    outputs/drone_pose.csv   t, lat, lon, rel_alt, hdg, roll, pitch
    outputs/usv_pose.csv     t, lat, lon, hdg, vn, ve        (fog euler + ned_vel)
    outputs/frames.csv       t, idx, file
    outputs/camera_info.json fx, fy, cx, cy, width, height
    outputs/events.csv       t, topic, value   (usv/follow, tag/follow, 짐벌 틸트 명령 등)
    outputs/frames/000000.jpg ...             (--frames 를 줄 때만)
"""
import argparse
import json
from pathlib import Path

from rosbags.highlevel import AnyReader

HERE = Path(__file__).resolve().parent.parent
# 기본값은 DT0503. 다른 비행은 --drone / --usv / --out 으로 준다
DRONE = HERE / "data" / "drone_DT0503"
USV = HERE / "data" / "usv_DT0503"
OUT = HERE / "outputs"

# 짐벌 틸트 명령 복원용 — usv_follow_node.servo(): pwm = 1500 + angle * 5.5
SERVO_SCALE, SERVO_MID = 5.5, 1500.0
CH_PITCH, CH_YAW = 9, 8


def read(bag, topics):
    """{topic: [(t_ns, msg), ...]} — 필요한 토픽만 역직렬화한다."""
    out = {t: [] for t in topics}
    with AnyReader([bag]) as r:
        conns = [c for c in r.connections if c.topic in out]
        missing = set(out) - {c.topic for c in conns}
        if missing:
            raise SystemExit(f"{bag.name}: 토픽이 없다 {sorted(missing)}")
        for c, ts, raw in r.messages(connections=conns):
            out[c.topic].append((ts, r.deserialize(raw, c.msgtype)))
    return out


def nearest(series, t):
    """[(t, value)] 에서 t 에 가장 가까운 value 와 시간차(초). 선형보간 안 한다 —
    heading 은 각도라 wrap 이 있고, 20~100Hz 라 보간해도 차이가 없다."""
    import bisect
    ts = [x[0] for x in series]
    i = bisect.bisect_left(ts, t)
    cands = [j for j in (i - 1, i) if 0 <= j < len(series)]
    j = min(cands, key=lambda j: abs(ts[j] - t))
    return series[j][1], ts[j] - t


def main():
    global OUT, DRONE, USV
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", action="store_true", help="JPEG 도 풀어서 outputs/frames/ 에")
    ap.add_argument("--drone", type=Path, default=DRONE, help="무인기 bag 폴더")
    ap.add_argument("--usv", type=Path, default=USV, help="USV bag 폴더")
    ap.add_argument("--out", type=Path, default=OUT, help="결과를 쓸 폴더")
    a = ap.parse_args()
    OUT, DRONE, USV = a.out, a.drone, a.usv
    OUT.mkdir(parents=True, exist_ok=True)

    # ---------------------------------------------------------------- 무인기
    d = read(DRONE, ["/gcs/daiv/rd/fix", "/gcs/daiv/rd/heading", "/gcs/daiv/rd/attitude",
                     "/gcs/daiv/rd/rel_alt", "/gcs/daiv/cam/compressed",
                     "/gcs/daiv/camera_info", "/gcs/daiv/usv/follow",
                     "/gcs/daiv/tag/follow", "/gcs/daiv/rd/command"])

    # attitude.z 는 heading 과 같은 값이다 (link_node 가 같은 MAVLink 메시지로 만든다).
    # 그래서 자세는 attitude 하나로 받고, roll·pitch 만 따로 쓴다.
    att = [(ts / 1e9, (m.vector.x, m.vector.y, m.vector.z)) for ts, m in d["/gcs/daiv/rd/attitude"]]
    alt = [(ts / 1e9, m.data) for ts, m in d["/gcs/daiv/rd/rel_alt"]]
    rows = []
    for ts, m in d["/gcs/daiv/rd/fix"]:
        t = ts / 1e9
        (roll, pitch, hdg), dt_att = nearest(att, t)
        rel, dt_alt = nearest(alt, t)
        rows.append((t, m.latitude, m.longitude, rel, hdg, roll, pitch, dt_att, dt_alt))
    with (OUT / "drone_pose.csv").open("w") as f:
        f.write("t,lat,lon,rel_alt,hdg,roll,pitch,dt_att,dt_alt\n")
        for r in rows:
            f.write("%.6f,%.8f,%.8f,%.2f,%.3f,%.2f,%.2f,%.3f,%.3f\n" % r)
    print(f"drone_pose.csv  {len(rows)} 행  {rows[0][0]:.1f}~{rows[-1][0]:.1f}")

    # 카메라 내부 파라미터 — 전 구간 같은 값인지 확인하고 하나만 남긴다
    ks = {tuple(m.k) + (m.width, m.height) for _, m in d["/gcs/daiv/camera_info"]}
    if len(ks) != 1:
        raise SystemExit(f"camera_info 가 바뀐다: {ks}")
    k = list(ks)[0]
    cam = dict(fx=k[0], fy=k[4], cx=k[2], cy=k[5], width=k[9], height=k[10])
    (OUT / "camera_info.json").write_text(json.dumps(cam, indent=2))
    print("camera_info.json", cam)

    # 프레임 시각
    frames = [(ts / 1e9, m) for ts, m in d["/gcs/daiv/cam/compressed"]]
    with (OUT / "frames.csv").open("w") as f:
        f.write("idx,t,file\n")
        for i, (t, _) in enumerate(frames):
            f.write(f"{i},{t:.6f},{i:06d}.jpg\n")
    print(f"frames.csv      {len(frames)} 장  {frames[0][0]:.1f}~{frames[-1][0]:.1f}")
    if a.frames:
        fd = OUT / "frames"
        fd.mkdir(exist_ok=True)
        for i, (_, m) in enumerate(frames):
            (fd / f"{i:06d}.jpg").write_bytes(bytes(m.data))
        print(f"frames/         {len(frames)} 장 저장")

    # 이벤트 — 추종 켜고 끈 시점, 짐벌 명령
    ev = []
    for topic in ("/gcs/daiv/usv/follow", "/gcs/daiv/tag/follow"):
        ev += [(ts / 1e9, topic, "true" if m.data else "false") for ts, m in d[topic]]
    for ts, m in d["/gcs/daiv/rd/command"]:
        v = list(m.data)
        if len(v) == 3 and v[0] == 183:      # MAV_CMD_DO_SET_SERVO
            ch, ang = int(v[1]), (v[2] - SERVO_MID) / SERVO_SCALE
            name = {CH_PITCH: "gimbal_tilt_deg", CH_YAW: "gimbal_yaw_deg"}.get(ch, f"servo{ch}")
            ev.append((ts / 1e9, name, f"{ang:.2f}"))
        else:
            ev.append((ts / 1e9, "rd/command", " ".join(f"{x:g}" for x in v)))
    ev.sort()
    with (OUT / "events.csv").open("w") as f:
        f.write("t,topic,value\n")
        for t, topic, v in ev:
            f.write(f"{t:.6f},{topic},{v}\n")
    print(f"events.csv      {len(ev)} 행")

    # ------------------------------------------------------------------ USV
    u = read(USV, ["/maiv/hemisphere_gps/navsatfix", "/maiv/fog/euler_orientation",
                   "/maiv/fog/ned_vel"])
    import math
    eul = [(ts / 1e9, math.degrees(m.data[2]) % 360.0) for ts, m in u["/maiv/fog/euler_orientation"]]
    vel = [(ts / 1e9, (m.vector.x, m.vector.y)) for ts, m in u["/maiv/fog/ned_vel"]]
    rows = []
    for ts, m in u["/maiv/hemisphere_gps/navsatfix"]:
        t = ts / 1e9
        hdg, dt_h = nearest(eul, t)
        (vn, ve), dt_v = nearest(vel, t)
        rows.append((t, m.latitude, m.longitude, hdg, vn, ve, dt_h, dt_v))
    with (OUT / "usv_pose.csv").open("w") as f:
        f.write("t,lat,lon,hdg,vn,ve,dt_hdg,dt_vel\n")
        for r in rows:
            f.write("%.6f,%.8f,%.8f,%.3f,%.3f,%.3f,%.3f,%.3f\n" % r)
    print(f"usv_pose.csv    {len(rows)} 행  {rows[0][0]:.1f}~{rows[-1][0]:.1f}")


if __name__ == "__main__":
    main()
