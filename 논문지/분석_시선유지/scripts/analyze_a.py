#!/usr/bin/env python3
"""(A) bag 기반 기수각 오차 e_psi.

    python scripts/analyze_a.py                  # 추종 구간을 events.csv 에서 스스로 찾는다
    python scripts/analyze_a.py --out outputs_XX --topic /gcs/daiv/tag/follow
    python scripts/analyze_a.py --window 1790906427.7,1790906664.8   # 직접 줄 때

정의 (usv_follow_node.plan() 과 같은 식)
    psi_d = 무인기 -> USV 방위각,  e_psi = relative_bearing(psi_d, psi)   (+ = USV 가 오른쪽)
두 가지로 낸다.
    e_psi        USV 가 자기 bag 에 남긴 **같은 시각**의 좌표로 — 실제 시선 오차. (B) 와 비교할 값
    e_psi_ctrl   제어기가 그때 본 좌표로 — 링크 지연 LINK_DELAY 만큼 묵은 좌표를
                 (나이 + lead_secs) 등속 외삽한 est. 제어기가 0 으로 몰던 값
두 값의 차이가 곧 "위치 갱신 지연 + 외삽" 이 시선에 남긴 몫이다.

같이 내는 것: 수평거리, 필요한 틸트, 짐벌 틸트 명령에서 거꾸로 푼 usv_below,
그리고 (B) 와 맞대려고 각도를 픽셀로 바꾼 값 (dx_pred, dy_pred).

내보내는 것
    outputs/a_timeseries.csv   무인기 좌표 시각(5Hz)마다 한 줄
    outputs/a_frames.csv       프레임 956 장 시각마다 한 줄 — (B) 와 바로 맞댄다
    outputs/a_stats.txt        구간 통계
"""
import argparse
import bisect
import csv
import datetime
import json
import math
from pathlib import Path

from geo import bearing_between, meters_between, offset_to_gps, relative_bearing

HERE = Path(__file__).resolve().parent.parent
OUT = HERE / "outputs"
KST = datetime.timezone(datetime.timedelta(hours=9))

# USV -> 관제소 시각 차 (인수인계 문서). 무인기가 받은 좌표는 이만큼 묵어 있었다
LINK_DELAY = 0.205
LEAD_SECS = 0.5        # usv_follow_node DEFAULTS lead_secs
VEL_WINDOW = 2.0       # usv_follow_node DEFAULTS vel_window
FOLLOW_TOPIC = "/gcs/daiv/usv/follow"   # 이 토픽이 true 인 동안이 분석 구간이다


def load(name, cols):
    with (OUT / name).open() as f:
        return [tuple(float(r[c]) for c in cols) for r in csv.DictReader(f)]


def kst(t):
    return datetime.datetime.fromtimestamp(t, KST).strftime("%H:%M:%S")


class UsvTrack:
    """USV bag 의 좌표열. '그 시각의 진짜 좌표' 와 '제어기가 그때 본 추정' 을 둘 다 준다."""

    def __init__(self, rows):
        self.rows = rows                      # (t, lat, lon, hdg, vn, ve)
        self.t = [r[0] for r in rows]

    def at(self, t):
        """t 에 가장 가까운 한 줄 (10Hz 라 0.05초 안으로 들어온다)."""
        i = bisect.bisect_left(self.t, t)
        j = min((k for k in (i - 1, i) if 0 <= k < len(self.rows)),
                key=lambda k: abs(self.t[k] - t))
        return self.rows[j]

    def seen_by_drone(self, t):
        """무인기 시각 t 에 제어기가 들고 있던 **가장 최근 수신 좌표**와 그 나이."""
        i = bisect.bisect_right(self.t, t - LINK_DELAY) - 1
        if i < 0:
            return None, None
        r = self.rows[i]
        return r, t - LINK_DELAY - r[0]       # 나이 = 받은 뒤 흐른 시간

    def est(self, t):
        """제어기의 est — 묵은 좌표를 (나이 + lead_secs) 만큼 등속 외삽. usv_follow_node 와 같다."""
        r, age = self.seen_by_drone(t)
        if r is None:
            return None
        lead = age + LEAD_SECS
        # usv_velocity(): vel_window 초 창의 양 끝 두 점
        t_end = t - LINK_DELAY
        i = bisect.bisect_right(self.t, t_end) - 1
        j = bisect.bisect_left(self.t, t_end - VEL_WINDOW)
        j = min(j, i)
        (t0, a0, o0), (t1, a1, o1) = self.rows[j][:3], self.rows[i][:3]
        if t1 - t0 < 0.5:
            vn = ve = 0.0
        else:
            d = meters_between(a0, o0, a1, o1)
            b = math.radians(bearing_between(a0, o0, a1, o1))
            vn, ve = d * math.cos(b) / (t1 - t0), d * math.sin(b) / (t1 - t0)
        return offset_to_gps(r[1], r[2], 0.0, vn * lead, ve * lead)


def rms(xs):
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else float("nan")


def row_at(t, drone, usv, cam, tilt_cmd):
    """무인기 시각 t 한 줄 분의 계산. drone = (t, lat, lon, rel_alt, hdg, roll, pitch)."""
    _, lat, lon, alt, hdg, roll, pitch = drone
    ut, ulat, ulon, uhdg, vn, ve = usv.at(t)
    psi_d = bearing_between(lat, lon, ulat, ulon)
    e = relative_bearing(psi_d, hdg)
    horiz = meters_between(lat, lon, ulat, ulon)

    est = usv.est(t)
    e_ctrl = relative_bearing(bearing_between(lat, lon, *est), hdg) if est else float("nan")

    # 시선 고도각 — 아래가 음수. usv_below 는 짐벌 명령에서 거꾸로 풀어 쓴다 (추정은 main 에서)
    tilt = tilt_cmd(t)
    elev = -math.degrees(math.atan2(max(alt + row_at.usv_below, 0.0), horiz))
    dx = cam["fx"] * math.tan(math.radians(e))                      # + = 화면 오른쪽
    dy = cam["fy"] * math.tan(math.radians(tilt - elev)) if tilt == tilt else float("nan")
    return dict(t=t, lat=lat, lon=lon, rel_alt=alt, hdg=hdg, roll=roll, pitch=pitch,
                usv_lat=ulat, usv_lon=ulon, usv_hdg=uhdg, usv_spd=math.hypot(vn, ve),
                usv_dt=ut - t, psi_d=psi_d, e_psi=e, e_psi_ctrl=e_ctrl, horiz=horiz,
                rel_from_usv=relative_bearing(bearing_between(ulat, ulon, lat, lon), uhdg),
                elev_los=elev, tilt_cmd=tilt, dx_pred=dx, dy_pred=dy)


row_at.usv_below = 8.0      # main 에서 짐벌 명령으로 다시 맞춘다


def follow_window(topic):
    """events.csv 에서 topic 이 true 였던 가장 긴 구간. rd_cmd 가 2 초마다 true 를 다시 내므로
    true 가 끊긴 것으로 보려면 FOLLOW_GAP 초 넘게 비어야 한다."""
    FOLLOW_GAP = 5.0
    ts = [float(r["t"]) for r in csv.DictReader((OUT / "events.csv").open())
          if r["topic"] == topic and r["value"] == "true"]
    if not ts:
        raise SystemExit(f"events.csv 에 {topic} true 가 없다 — --window 로 직접 준다")
    runs, start, prev = [], ts[0], ts[0]
    for t in ts[1:]:
        if t - prev > FOLLOW_GAP:
            runs.append((start, prev))
            start = t
        prev = t
    runs.append((start, prev))
    lo, hi = max(runs, key=lambda r: r[1] - r[0])
    if len(runs) > 1:
        print(f"※ {topic} true 구간이 {len(runs)} 개다. 가장 긴 것을 쓴다 — "
              + ", ".join(f"{kst(a)}~{kst(b)}({b-a:.0f}s)" for a, b in runs))
    return lo, hi


def main():
    global OUT
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=OUT, help="extract_bags.py 가 쓴 폴더")
    ap.add_argument("--topic", default=FOLLOW_TOPIC, help="이 토픽이 true 인 동안을 분석한다")
    ap.add_argument("--window", help="t0,t1 (UNIX 초) — 구간을 직접 줄 때")
    a = ap.parse_args()
    OUT = a.out
    if a.window:
        lo, hi = (float(x) for x in a.window.split(","))
    else:
        lo, hi = follow_window(a.topic)

    drone = load("drone_pose.csv", ["t", "lat", "lon", "rel_alt", "hdg", "roll", "pitch"])
    usv = UsvTrack(load("usv_pose.csv", ["t", "lat", "lon", "hdg", "vn", "ve"]))
    frames = load("frames.csv", ["idx", "t"])
    cam = json.loads((OUT / "camera_info.json").read_text())

    # 짐벌 틸트 명령 — 계단식(2° 사구간 · 0.5s)이라 계단 유지로 읽는다
    tilts = []
    with (OUT / "events.csv").open() as f:
        for r in csv.DictReader(f):
            if r["topic"] == "gimbal_tilt_deg":
                tilts.append((float(r["t"]), float(r["value"])))
    tt = [x[0] for x in tilts]

    def tilt_cmd(t):
        i = bisect.bisect_right(tt, t) - 1
        return tilts[i][1] if i >= 0 else float("nan")

    win = [d for d in drone if lo <= d[0] <= hi]
    if not win:
        raise SystemExit(f"구간 {kst(lo)}~{kst(hi)} 안에 무인기 좌표가 없다")

    # ---- usv_below 를 짐벌 명령에서 거꾸로 푼다: tilt = -atan2(alt + below, horiz)
    cands = []
    for d in win:
        tilt = tilt_cmd(d[0])
        if tilt != tilt or tilt >= -1.0:
            continue
        _, _, _, alt, *_ = d
        horiz = meters_between(d[1], d[2], *usv.at(d[0])[1:3])
        cands.append(horiz * math.tan(math.radians(-tilt)) - alt)
    cands.sort()
    row_at.usv_below = cands[len(cands) // 2] if cands else 8.0

    rows = [row_at(d[0], d, usv, cam, tilt_cmd) for d in win]
    frows = []
    for idx, t in frames:
        if not lo <= t <= hi:
            continue
        r = row_at(t, min(drone, key=lambda d: abs(d[0] - t)), usv, cam, tilt_cmd)
        # idx 를 맨 앞에 둔다 — b_track.csv 와 묶을 때 쓴다
        frows.append(dict(idx=int(idx), **r))

    for name, rr in (("a_timeseries.csv", rows), ("a_frames.csv", frows)):
        with (OUT / name).open("w", newline="") as f:
            w = csv.DictWriter(f, list(rr[0]))
            w.writeheader()
            w.writerows(rr)

    e = [r["e_psi"] for r in rows]
    ec = [r["e_psi_ctrl"] for r in rows]
    dur = win[-1][0] - win[0][0]
    L = []
    L.append(f"DT0503 시선 유지 — (A) bag 기반 기수각 오차")
    L.append(f"구간: {kst(lo)}~{kst(hi)} ({dur:.1f} s, 무인기 좌표 {len(rows)} 점, "
             f"프레임 {len(frows)} 장)")
    rel_m = sum(r["rel_from_usv"] for r in rows) / len(rows)
    form = {0: "선수", 90: "우현", 180: "선미", 270: "좌현"}
    near = min(form, key=lambda k: abs(relative_bearing(rel_m + 45.0, k)))
    L.append(f"대형 추정: {form[near]} — USV 기준 상대방위 평균 {rel_m:+.1f}° 에 "
             f"euler 기수 보정 +45° 를 더하면 {(rel_m + 45.0) % 360:.0f}°")
    L.append(f"  ※ 대형 번호는 bag 에 없다. 기하로 판정한 것이니 눈으로 한 번 확인할 것")
    L.append(f"  수평거리 평균 {sum(r['horiz'] for r in rows)/len(rows):.2f} m, "
             f"RMSE(20 m 기준) {rms([r['horiz'] - 20.0 for r in rows]):.2f} m")
    L.append(f"  USV 속력 평균 {sum(r['usv_spd'] for r in rows)/len(rows):.2f} m/s "
             f"(최대 {max(r['usv_spd'] for r in rows):.2f})")
    L.append(f"  짐벌 틸트 명령에서 푼 usv_below = {row_at.usv_below:.2f} m")
    L.append("")
    L.append(f"e_psi (실제 시선 오차, USV bag 같은 시각 좌표)")
    L.append(f"  평균 {sum(e)/len(e):+.2f}°  RMS {rms(e):.2f}°  "
             f"최대 |e| {max(abs(x) for x in e):.2f}°  95% {sorted(abs(x) for x in e)[int(.95*len(e))]:.2f}°")
    L.append(f"  = 화면에서 RMS {rms(e)*cam['fx']*math.pi/180:.0f} px  "
             f"(1° = {cam['fx']*math.tan(math.radians(1)):.1f} px, 화면 폭 816 px = ±33.2°)")
    L.append(f"e_psi_ctrl (제어기가 본 오차, 지연 {LINK_DELAY}s + 외삽 {LEAD_SECS}s)")
    L.append(f"  평균 {sum(ec)/len(ec):+.2f}°  RMS {rms(ec):.2f}°  최대 |e| {max(abs(x) for x in ec):.2f}°")
    L.append(f"차이 e_psi - e_psi_ctrl: RMS {rms([a-b for a,b in zip(e,ec)]):.2f}°  "
             f"→ 위치 갱신 지연·외삽이 시선에 남긴 몫")
    L.append("")
    L.append("30초 토막별 (RMS / 최대 |e_psi|, °)")
    t0 = win[0][0]
    while t0 < hi:
        seg = [r for r in rows if t0 <= r["t"] < t0 + 30]
        if seg:
            se = [r["e_psi"] for r in seg]
            L.append(f"  {kst(t0)}~{kst(min(t0+30,hi))}  RMS {rms(se):4.2f}  "
                     f"최대 {max(abs(x) for x in se):5.2f}  "
                     f"USV 속력 {sum(r['usv_spd'] for r in seg)/len(seg):.1f} m/s")
        t0 += 30
    txt = "\n".join(L)
    (OUT / "a_stats.txt").write_text(txt + "\n")
    print(txt)


if __name__ == "__main__":
    main()
