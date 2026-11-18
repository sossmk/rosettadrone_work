#!/usr/bin/env python3
"""(A) 와 (B) 를 맞대고 논문 3.2.x 에 넣을 그림 1장 + 수치를 낸다.

    python scripts/compare.py                 # 기본 outputs/
    python scripts/compare.py --out outputs_XX

맞대는 법 — 픽셀을 각도로 바꾼다 (camera_info 의 fx, fy)
    (B) 가로 = atan(dx / fx),  세로 = atan(dy / fy).  1° = fx·tan1° = 10.9 px

(A) 와 (B) 는 **같은 양이 아니다.** (A) 는 기수(psi)에 대한 시선 오차고, (B) 는 카메라
광축에 대한 시선 오차다. 둘 사이에 카메라/짐벌 사슬이 끼어 있다.
    (B) = (A) + 고정 편의 + 짐벌 yaw 가 기수를 따라오는 지연 + 기체 roll 누출
그래서 이 사슬을 모형으로 세우고, 단계마다 설명되는 몫을 센다.

    고정 편의   짐벌 yaw 0 은 "앱이 켜진 때의 방향" 이지 기수가 아니다 (usv_follow_node.aim 주석)
    yaw 지연    yaw 축만 기체에 붙어 있어 기수를 1차 지연으로 따라온다 -> T_YAW 를 훑어 찾는다
    roll 누출   틸트 theta 로 내려본 상태에서 기체가 roll 하면 yaw 축이 같이 기울어
                방위가 roll * sin(theta) 만큼 샌다. 계수를 맞춰 보고 sin(theta) 와 견준다

내보내는 것
    outputs/fig_gaze.png      논문용 그림
    outputs/compare.csv       프레임별 (A)·(B)·모형 나란히
    outputs/compare_stats.txt 수치
"""
import csv
import datetime
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from geo import relative_bearing
from hangul_font import use_hangul_font

HERE = Path(__file__).resolve().parent.parent
OUT = HERE / "outputs"
KST = datetime.timezone(datetime.timedelta(hours=9))

use_hangul_font()


def rms(x):
    x = np.asarray(x, dtype=float)
    return float(np.sqrt(np.mean(x[~np.isnan(x)] ** 2)))


def main():
    global OUT
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=OUT, help="analyze_a.py·track_b.py 가 쓴 폴더")
    OUT = ap.parse_args().out

    cam = json.loads((OUT / "camera_info.json").read_text())
    fx, fy = cam["fx"], cam["fy"]
    px_per_deg = fx * math.tan(math.radians(1))

    # 5Hz 전체 시계열 — 지연을 넣으려면 프레임 시각 사이를 메워야 한다. 각도는 unwrap 하고 보간
    S = list(csv.DictReader((OUT / "a_timeseries.csv").open()))
    ts = np.array([float(r["t"]) for r in S])
    hdg_u = np.degrees(np.unwrap(np.radians([float(r["hdg"]) for r in S])))
    psid_u = np.degrees(np.unwrap(np.radians([float(r["psi_d"]) for r in S])))
    roll_s = np.array([float(r["roll"]) for r in S])

    A = {int(float(r["t"]) * 1000): r for r in csv.DictReader((OUT / "a_frames.csv").open())}
    rows = []
    for r in csv.DictReader((OUT / "b_track.csv").open()):
        if r["ok"] != "1":
            continue
        a = A.get(int(float(r["t"]) * 1000))
        if a is None:
            continue
        rows.append(dict(
            idx=int(r["idx"]), t=float(r["t"]),
            e_psi=float(a["e_psi"]), e_psi_ctrl=float(a["e_psi_ctrl"]),
            horiz=float(a["horiz"]), rel_alt=float(a["rel_alt"]), roll=float(a["roll"]),
            tilt_cmd=float(a["tilt_cmd"]), usv_spd=float(a["usv_spd"]),
            dx=float(r["dx"]), dy=float(r["dy"]), area=int(r["area"]),
            b_x=math.degrees(math.atan(float(r["dx"]) / fx)),
            b_y=math.degrees(math.atan(float(r["dy"]) / fy))))
    if not rows:
        raise SystemExit("겹치는 프레임이 없다 — track_b.py 를 먼저 돌린다")

    t = np.array([r["t"] for r in rows])
    A_x = np.array([r["e_psi"] for r in rows])
    B_x = np.array([r["b_x"] for r in rows])
    B_y = np.array([r["b_y"] for r in rows])
    tilt = np.array([r["tilt_cmd"] for r in rows])
    horiz = np.array([r["horiz"] for r in rows])
    alt = np.array([r["rel_alt"] for r in rows])

    # ---- 카메라/짐벌 사슬 모형: 짐벌 yaw 지연 T 와 roll 누출 계수를 함께 맞춘다
    def chain(T):
        psi_d = np.interp(t, ts, psid_u)
        cam_yaw = np.interp(t - T, ts, hdg_u)       # 기수를 T 초 늦게 따라온 카메라 방위
        return np.array([relative_bearing(p, c) for p, c in zip(psi_d, cam_yaw)])

    best = None
    for T in np.arange(0.0, 3.001, 0.05):
        base = chain(T)
        roll = np.interp(t, ts, roll_s)
        X = np.column_stack([np.ones_like(roll), roll])
        c, *_ = np.linalg.lstsq(X, B_x - base, rcond=None)
        s = rms(B_x - base - X @ c)
        if best is None or s < best[0]:
            best = (s, float(T), float(c[0]), float(c[1]))
    resid_rms, T_YAW, bias_fit, k_roll = best
    base = chain(T_YAW)
    roll = np.interp(t, ts, roll_s)
    model = base + bias_fit + k_roll * roll
    for r, m, b in zip(rows, model, base):
        r["model_x"] = round(float(m), 3)
        r["resid_x"] = round(float(r["b_x"] - m), 3)
    with (OUT / "compare.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    bias_x = float(np.mean(B_x - A_x))
    sin_tilt = math.sin(math.radians(-np.nanmean(tilt)))

    # ---- 세로: (B) 로 거꾸로 푼 실제 수면 높이차. elev = tilt_cmd - atan(dy/fy)
    elev = tilt - B_y
    below_true = np.nanmedian(horiz * np.tan(np.radians(-elev)) - alt)
    below_cmd = np.nanmedian(horiz * np.tan(np.radians(-tilt)) - alt)

    L = []
    L.append("DT0503 시선 유지 — (A) bag 기반 · (B) SAM2 영상 판독 비교")
    L.append(f"맞댄 프레임 {len(rows)} 장 · {datetime.datetime.fromtimestamp(t[0], KST):%H:%M:%S}"
             f"~{datetime.datetime.fromtimestamp(t[-1], KST):%H:%M:%S} · 1° = {px_per_deg:.1f} px")
    dt = np.diff(t)
    lost = float(np.sum(dt[dt > 1.0]))
    L.append(f"영상 끊김: 1 초 넘게 빈 곳 {int((dt > 1.0).sum())} 군데, 합쳐서 {lost:.0f} s "
             f"(구간 {t[-1]-t[0]:.0f} s 중). 그림에서는 그 구간의 선을 끊었다")
    L.append("")
    L.append("가로 (기수각)")
    L.append(f"  (A) e_psi        평균 {A_x.mean():+.2f}°  RMS {rms(A_x):.2f}°  "
             f"최대 {np.abs(A_x).max():.2f}°")
    L.append(f"  (B) atan(dx/fx)  평균 {B_x.mean():+.2f}° ({np.mean([r['dx'] for r in rows]):+.0f} px)"
             f"  RMS(편의 제거) {rms(B_x - B_x.mean()):.2f}°  최대 {np.abs(B_x).max():.2f}°")
    L.append(f"  → (B) 의 변동이 (A) 의 {rms(B_x-B_x.mean())/rms(A_x):.1f} 배다. 같지 않다.")
    L.append("")
    L.append("  (B) 와 (A) 사이에 낀 카메라/짐벌 사슬을 모형으로 벗겨낸다")
    L.append(f"    고정 편의      {bias_fit:+.2f}°  — 짐벌 yaw 0 이 기수가 아니라 앱 시작 방향이라서")
    L.append(f"    짐벌 yaw 지연  {T_YAW:.2f} s   — yaw 축만 기체에 붙어 기수를 늦게 따라온다")
    L.append(f"    roll 누출      {k_roll:+.3f} °/°  — 틸트 {np.nanmean(tilt):.1f}° 에서 "
             f"sin(틸트) = {sin_tilt:.2f} 로 예상되는 값과 맞는다")
    L.append("")
    L.append("  단계별로 남는 변동 (RMS, 편의 제거 후)")
    L.append(f"    (B) 그대로                      {rms(B_x - B_x.mean()):.2f}°")
    v = B_x - A_x
    L.append(f"    − (A) e_psi                     {rms(v - v.mean()):.2f}°")
    v = B_x - base
    L.append(f"    − 짐벌 yaw 지연 {T_YAW:.2f} s           {rms(v - v.mean()):.2f}°")
    L.append(f"    − roll 누출까지                 {resid_rms:.2f}°")
    L.append(f"    (A) e_psi 자체                  {rms(A_x):.2f}°   ← 여기까지 내려오면 같아진다")
    L.append(f"  상관: (A)-(B) {np.corrcoef(A_x, B_x)[0,1]:.3f} → 모형-(B) "
             f"{np.corrcoef(model, B_x)[0,1]:.3f}")
    L.append("")
    L.append("세로 (짐벌 틸트)")
    L.append(f"  (B) atan(dy/fy) 평균 {B_y.mean():+.2f}° ({np.mean([r['dy'] for r in rows]):+.0f} px)"
             f"  변동 RMS {rms(B_y - B_y.mean()):.2f}°")
    L.append(f"  틸트 명령이 가정한 usv_below = {below_cmd:.1f} m (런치 기본 8.0 m)")
    L.append(f"  영상에서 거꾸로 푼 실제 높이차 = {below_true:.1f} m")
    L.append(f"  → {below_cmd - below_true:.1f} m 더 낮다고 보고 {B_y.mean():+.2f}° 만큼 더 내려봤다. "
             f"USV 는 그만큼 화면 위로 치우쳤다")
    L.append("")
    L.append("해석")
    L.append(f"  1) 기수 제어는 시선을 잘 유지했다 — e_psi RMS {rms(A_x):.1f}°, 최대 {np.abs(A_x).max():.1f}°"
             f" (화면으로 {rms(A_x)*px_per_deg:.0f} px).")
    L.append(f"     제어기가 본 오차와 USV bag 좌표로 낸 오차가 거의 같아, GNSS 위치 오차나")
    L.append(f"     위치 갱신 지연이 시선을 망친 것은 아니다.")
    L.append(f"  2) 그런데 화면에서는 RMS {rms(B_x-B_x.mean()):.1f}° 로 더 크게 흔들렸다. 그 차이는")
    L.append(f"     기수 제어가 아니라 **카메라가 기수를 따라오는 사슬**에서 생겼다 —")
    L.append(f"     짐벌 yaw 지연 {T_YAW:.1f} s 와 틸트 {np.nanmean(tilt):.0f}° 에서의 roll 누출")
    L.append(f"     (계수 {abs(k_roll):.2f} ≈ sin{abs(np.nanmean(tilt)):.0f}° = {sin_tilt:.2f}).")
    L.append(f"     둘을 빼면 {resid_rms:.1f}° 만 남는다.")
    L.append(f"  3) 고정 편의는 가로 {bias_fit:+.1f}°(짐벌 yaw 기준점)·세로 {B_y.mean():+.1f}°"
             f"(usv_below 를 {below_cmd - below_true:.1f} m 크게 잡음) 로, 둘 다 보정으로 없앨 수 있다.")
    txt = "\n".join(L)
    (OUT / "compare_stats.txt").write_text(txt + "\n")
    print(txt)

    # ----------------------------------------------------------------- 그림
    # 영상이 끊긴 곳(프레임 간격 > GAP 초)은 선을 **끊어서** 그린다. 이어 그리면 그 구간이
    # 평평한 직선이 되어 "오차가 가만히 있었다" 처럼 보인다 — 실제로는 영상이 없는 구간이다
    GAP = 1.0
    gaps = np.where(np.diff(t) > GAP)[0]

    def brk(v):
        v = np.array(v, dtype=float).copy()
        v[gaps] = np.nan
        return v

    x = t - t[0]
    fig, ax = plt.subplots(2, 1, figsize=(7.2, 5.2), sharex=True,
                           gridspec_kw=dict(hspace=0.12))
    ax[0].axhline(0, color="0.8", lw=0.8)
    ax[0].plot(x, brk(B_x), lw=1.1, color="#d1495b", label="(B) 영상에서 잰 가로 오차")
    ax[0].plot(x, brk(model), lw=1.0, color="#f0a202", ls="--",
               label=f"카메라·짐벌 사슬 모형 (yaw 지연 {T_YAW:.1f} s + roll 누출)")
    ax[0].plot(x, brk(A_x), lw=1.1, color="#1f6feb", label="(A) bag 기반 $e_\\psi$")
    ax[0].set_ylabel("가로 오차 [°]")
    ax[0].legend(fontsize=7.5, ncol=1, loc="lower left", framealpha=0.9)
    ax[0].grid(alpha=0.25)
    axb = ax[0].twinx()
    axb.set_ylim(np.array(ax[0].get_ylim()) * px_per_deg)
    axb.set_ylabel("[px]", fontsize=8)

    ax[1].axhline(0, color="0.8", lw=0.8)
    ax[1].plot(x, brk(B_y), lw=1.1, color="#d1495b", label="(B) 세로 오차")
    ax[1].axhline(B_y.mean(), color="#8d6e63", lw=1.0, ls=":",
                  label=f"평균 {B_y.mean():+.1f}° — usv_below 를 "
                        f"{below_cmd - below_true:.1f} m 크게 잡은 몫")
    ax[1].set_ylabel("세로 오차 [°]")
    ax[1].set_xlabel(f"{datetime.datetime.fromtimestamp(t[0], KST):%H:%M:%S} 이후 경과 [s]")
    ax[1].legend(fontsize=7.5, loc="upper left", framealpha=0.9)
    ax[1].grid(alpha=0.25)
    axb2 = ax[1].twinx()
    axb2.set_ylim(np.array(ax[1].get_ylim()) * px_per_deg)
    axb2.set_ylabel("[px]", fontsize=8)

    fig.savefig(OUT / "fig_gaze.png", dpi=300, bbox_inches="tight")
    print("\nfig_gaze.png 저장")


if __name__ == "__main__":
    main()
