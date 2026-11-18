#!/usr/bin/env python3
"""b_track.csv 를 요약해 표와 그림으로 — **재기만 한다. 해석은 안 한다.**

    python scripts/report_b.py --out outputs_vid

내는 것
    outputs_vid/fig_gaze_b.png   dx · dy · 중심까지 거리 시계열
    outputs_vid/b_stats.txt      평균 · RMS · 최대 · 중앙값
"""
import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from hangul_font import use_hangul_font

HERE = Path(__file__).resolve().parent.parent

use_hangul_font()


def rms(x):
    x = np.asarray(x, float)
    return float(np.sqrt(np.mean(x ** 2)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=HERE / "outputs_video")
    ap.add_argument("--gap", type=float, default=1.0, help="이 초보다 벌어지면 선을 끊는다")
    a = ap.parse_args()
    OUT = a.out

    cam = json.loads((OUT / "camera_info.json").read_text())
    fx, fy, w, h = cam["fx"], cam["fy"], cam["width"], cam["height"]
    px_per_deg = fx * math.tan(math.radians(1))

    rows = [r for r in csv.DictReader((OUT / "b_track.csv").open()) if r["ok"] == "1"]
    if not rows:
        raise SystemExit("b_track.csv 에 성공한 프레임이 없다")
    t = np.array([float(r["t"]) for r in rows])
    dx = np.array([float(r["dx"]) for r in rows])
    dy = np.array([float(r["dy"]) for r in rows])
    gx = np.degrees(np.arctan(dx / fx))
    gy = np.degrees(np.arctan(dy / fy))
    rad = np.hypot(dx, dy)
    n_all = sum(1 for _ in csv.DictReader((OUT / "b_track.csv").open()))

    L = []
    L.append("USV 마스크 중심 − 화면 중심 (b_track.csv 요약)")
    L.append(f"프레임 {len(rows)}/{n_all} 장 · 구간 {t[0]:.1f}~{t[-1]:.1f} s ({t[-1]-t[0]:.0f} s)")
    L.append(f"화면 {w}×{h}, 중앙 ({cam['cx']:.0f}, {cam['cy']:.0f}) · "
             f"fx {fx:.0f} px → 1° = {px_per_deg:.1f} px")
    dt = np.diff(t)
    if (dt > a.gap).any():
        L.append(f"영상 끊김: {a.gap} 초 넘게 빈 곳 {int((dt > a.gap).sum())} 군데, "
                 f"합쳐서 {float(np.sum(dt[dt > a.gap])):.0f} s")
    L.append("")
    for nm, v, d, half in (("가로 dx", dx, gx, w / 2), ("세로 dy", dy, gy, h / 2)):
        L.append(f"{nm}")
        L.append(f"  평균   {v.mean():+7.1f} px ({d.mean():+6.2f}°)")
        L.append(f"  RMS(평균 뺀 뒤) {rms(v - v.mean()):6.1f} px ({rms(d - d.mean()):5.2f}°)")
        L.append(f"  최대   {np.abs(v).max():7.0f} px ({np.abs(d).max():6.2f}°)"
                 f"   화면 반폭의 {np.abs(v).max()/half*100:.0f}%")
    L.append("")
    L.append("중심까지 거리 dist")
    L.append(f"  중앙값 {np.median(rad):.0f} px ({np.median(rad)/px_per_deg:.1f}°)"
             f"  ·  평균 {rad.mean():.0f} px  ·  최대 {rad.max():.0f} px")
    for lim in (50, 100, 150, 200):
        L.append(f"  {lim:3d} px 안에 든 프레임: {float((rad <= lim).mean())*100:5.1f} %")
    txt = "\n".join(L)
    (OUT / "b_stats.txt").write_text(txt + "\n")
    print(txt)

    gaps = np.where(dt > a.gap)[0]

    def brk(v):
        v = np.array(v, float).copy()
        v[gaps] = np.nan
        return v

    x = t - t[0]
    fig, ax = plt.subplots(3, 1, figsize=(7.2, 6.4), sharex=True, gridspec_kw=dict(hspace=0.12))
    for i, (nm, v, half) in enumerate((("가로 dx", dx, w / 2), ("세로 dy", dy, h / 2))):
        ax[i].axhline(0, color="0.8", lw=0.8)
        ax[i].plot(x, brk(v), lw=1.1, color="#d1495b", label=nm)
        ax[i].axhline(v.mean(), color="#8d6e63", lw=1.0, ls=":", label=f"평균 {v.mean():+.0f} px")
        ax[i].set_ylabel(f"{nm} [px]")
        ax[i].legend(fontsize=8, loc="upper left", framealpha=0.9)
        ax[i].grid(alpha=0.25)
        axb = ax[i].twinx()
        axb.set_ylim(np.array(ax[i].get_ylim()) / px_per_deg)
        axb.set_ylabel("[°]", fontsize=8)
    ax[2].plot(x, brk(rad), lw=1.1, color="#1f6feb", label="중심까지 거리 dist")
    ax[2].axhline(float(np.median(rad)), color="#8d6e63", lw=1.0, ls=":",
                  label=f"중앙값 {np.median(rad):.0f} px")
    ax[2].set_ylabel("dist [px]")
    ax[2].set_ylim(bottom=0)
    ax[2].legend(fontsize=8, loc="upper left", framealpha=0.9)
    ax[2].grid(alpha=0.25)
    ax[2].set_xlabel("영상 구간 시작부터 경과 [s]")
    fig.savefig(OUT / "fig_gaze_b.png", dpi=300, bbox_inches="tight")
    print("\nfig_gaze_b.png 저장")


if __name__ == "__main__":
    main()
