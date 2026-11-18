#!/usr/bin/env python3
"""발표자료용 고화질 스틸·짧은 클립을 뽑는다.

`b_overlay.mp4` 는 눈으로 검수하라고 만든 것이라 코덱이 mpeg4(MPEG-4 Part 2) 다.
발표 화면에 띄우면 뭉개진다. 여기서는 **원본 프레임(JPEG)** 에서 다시 그려
PNG(무손실) 와 H.264(CRF 16) 로 낸다.

    # 대표 프레임 3장을 알아서 고른다 (가장 많이 벗어난 때·중앙값·가장 잘 맞은 때)
    python scripts/export_hq.py --out outputs_USV --pick 3

    # 프레임 번호를 찍어서
    python scripts/export_hq.py --out outputs --idx 120,240,360

    # 짧은 클립 — 추적 프레임만 이어 붙인다 (초당 2장이라 끊긴다)
    python scripts/export_hq.py --out outputs_USV --clip 200-260

    # 끊김 없는 클립 — 원본 영상을 원래 fps 로 읽고 표식만 보간한다 (발표용은 이쪽)
    python scripts/export_hq.py --out outputs_USV --video ~/usv_log/USV.mp4 --clip-t 334-356

결과는 <out>/hq/ 에 쌓인다.

표식: 노란 동그라미·십자가 화면 중앙, 흰 십자가 USV 중심, 둘을 잇는 선이 재는 값이다.
마스크(반투명 빨강)는 b_track.csv 에 없어서 다시 그리지 않는다 — SAM2 를 다시 돌려야
나온다. 발표용으로는 십자와 선이 더 깨끗하다.
"""
import argparse
import csv
import json
import shutil
import subprocess
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent.parent


WHITE = (255, 255, 255)


def draw(im, r, cx0, cy0, k, label=True, mask=None):
    """원본 위에 표식을 그린다. k 는 816 px 기준 배율.

    화면 중앙은 노란 십자 하나. USV 는 b_overlay.mp4 와 같게 반투명 빨강으로 칠하고,
    그 위에 흰 십자와 중앙까지 잇는 흰 선을 긋는다 (빨강 위에 빨강이면 안 보인다).
    마스크가 없으면 십자와 선만 나온다.
    """
    sz, th = int(12 * k), max(1, int(2 * k))
    if mask is not None and mask.any():
        im[mask] = (0.5 * im[mask] + np.array([0, 0, 128])).astype(np.uint8)
    if r and r["ok"]:
        c = (int(float(r["cx"])), int(float(r["cy"])))
        cv2.line(im, (int(cx0), int(cy0)), c, WHITE, max(1, int(1.5 * k)), cv2.LINE_AA)
        cv2.drawMarker(im, c, WHITE, cv2.MARKER_CROSS, sz, th + 1, cv2.LINE_AA)
    cv2.drawMarker(im, (int(cx0), int(cy0)), (0, 255, 255), cv2.MARKER_CROSS,
                   int(20 * k), th, cv2.LINE_AA)
    if label and r and r["ok"]:
        t = f"{float(r['dist']):.0f} px"
        org = (int(10 * k), int(32 * k))
        cv2.putText(im, t, org, cv2.FONT_HERSHEY_SIMPLEX, 0.8 * k, (0, 0, 0),
                    th + 2, cv2.LINE_AA)          # 검은 테두리 — 밝은 바다 위에서도 읽힌다
        cv2.putText(im, t, org, cv2.FONT_HERSHEY_SIMPLEX, 0.8 * k, (0, 255, 255),
                    th, cv2.LINE_AA)
    return im


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=HERE / "outputs")
    ap.add_argument("--idx", help="프레임 번호를 쉼표로")
    ap.add_argument("--pick", type=int, help="대표 프레임 n 장을 알아서 고른다")
    ap.add_argument("--clip", help="클립 구간, 예: 200-260 (프레임 번호)")
    ap.add_argument("--video", type=Path,
                    help="원본 영상. 주면 그 안에서 원래 fps 그대로 끊김 없이 뽑는다")
    ap.add_argument("--clip-t", dest="clip_t",
                    help="--video 와 함께. 초 구간, 예: 334-356")
    ap.add_argument("--fps", type=float, default=6.0, help="클립 재생 속도")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="확대 배율. 816x544 를 슬라이드에 꽉 채울 때 2 정도")
    ap.add_argument("--plain", action="store_true", help="표식 없이 원본 그대로")
    ap.add_argument("--crf", type=int, default=18,
                    help="H.264 화질. 낮을수록 좋고 용량이 큰다. 16 은 거의 무손실, "
                         "20 이면 발표 화면에서 차이를 못 느끼면서 용량이 1/3 이다")
    a = ap.parse_args()

    OUT = a.out
    cam = json.loads((OUT / "camera_info.json").read_text())
    cx0, cy0 = cam["cx"], cam["cy"]
    k = cam["width"] / 816.0
    rows = {int(r["idx"]): r for r in csv.DictReader((OUT / "b_track.csv").open())}
    for r in rows.values():
        # dist 는 나중에 넣은 열이라 예전 b_track.csv 에는 없다. 없으면 dx·dy 로 낸다
        if not r.get("dist"):
            r["dist"] = (float(r["dx"]) ** 2 + float(r["dy"]) ** 2) ** 0.5 if r["ok"] == "1" else ""
    good = [r for r in rows.values() if r["ok"] == "1"]
    if not good:
        raise SystemExit("b_track.csv 에 추적 성공한 프레임이 없다")
    hq = OUT / "hq"
    hq.mkdir(exist_ok=True)

    # track_b.py --save-masks 가 남긴 것. 없으면 십자만 찍는다
    MZ = np.load(OUT / "masks.npz") if (OUT / "masks.npz").exists() else None
    mshape = tuple(MZ["shape"]) if MZ is not None else None

    def mask_of(idx):
        if MZ is None or f"{idx}_bits" not in MZ:
            return None
        x0, y0, x1, y1 = MZ[f"{idx}_box"]
        m = np.zeros(mshape, dtype=bool)
        m[y0:y1, x0:x1] = np.unpackbits(
            MZ[f"{idx}_bits"], count=(y1 - y0) * (x1 - x0)).reshape(y1 - y0, x1 - x0).astype(bool)
        return m

    targets = []
    if a.idx:
        targets = [int(s) for s in a.idx.split(",")]
    elif a.pick:
        s = sorted(good, key=lambda r: float(r["dist"]))
        n = len(s)
        want = {"가장잘맞은": s[0], "중앙값": s[n // 2], "가장벗어난": s[-1]}
        picks = list(want.items())[: a.pick] if a.pick <= 3 else \
            [(f"{i}", s[round(i * (n - 1) / (a.pick - 1))]) for i in range(a.pick)]
        targets = [int(r["idx"]) for _, r in picks]
        for name, r in picks:
            print(f"  {name}: 프레임 {r['idx']}  중심까지 {float(r['dist']):.0f} px")

    def load(idx):
        p = OUT / "frames" / f"{idx:06d}.jpg"
        if not p.exists():
            p = OUT / "_b_frames" / f"{idx:06d}.jpg"
        im = cv2.imread(str(p))
        if im is None:
            raise SystemExit(f"프레임이 없다: {p}")
        if not a.plain:
            im = draw(im, rows.get(idx), cx0, cy0, k, mask=mask_of(idx))
        if a.scale != 1.0:
            # 확대는 Lanczos — 없는 해상도가 생기진 않지만 계단이 덜 보인다
            im = cv2.resize(im, None, fx=a.scale, fy=a.scale,
                            interpolation=cv2.INTER_LANCZOS4)
        return im

    for idx in targets:
        f = hq / f"frame_{idx:06d}.png"
        cv2.imwrite(str(f), load(idx), [cv2.IMWRITE_PNG_COMPRESSION, 3])
        print(f"{f}  ({f.stat().st_size/1e6:.1f} MB)")

    if a.clip_t:
        if not a.video:
            raise SystemExit("--clip-t 는 --video 와 같이 쓴다")
        t0, t1 = (float(x) for x in a.clip_t.split("-"))
        # 추적은 초당 2장이라 그대로 쓰면 뚝뚝 끊긴다. 원본 fps 로 읽으면서 표식 위치만
        # 시각으로 보간한다 — 영상은 원본 그대로고 십자만 부드럽게 따라간다
        tr = sorted((float(r["t"]), float(r["cx"]), float(r["cy"]), float(r["dist"]))
                    for r in good)
        tt = np.array([x[0] for x in tr])
        cxs, cys, dis = (np.array([x[i] for x in tr]) for i in (1, 2, 3))
        if not (tt[0] <= t0 and t1 <= tt[-1]):
            print(f"  주의: 추적 구간은 {tt[0]:.0f}~{tt[-1]:.0f} s 다. 밖은 표식이 멈춘다")

        cap = cv2.VideoCapture(str(a.video))
        fps = cap.get(cv2.CAP_PROP_FPS)
        # 마스크는 보간이 안 된다. 추적이 초당 2장인데 영상이 24 fps 면 칠한 자리가
        # 배보다 뒤처진다. 추적 간격이 영상 한 프레임 수준일 때만 칠한다
        step = float(np.median(np.diff(tt))) if len(tt) > 1 else 1e9
        fine = MZ is not None and step <= 1.5 / fps
        if MZ is not None and not fine:
            print(f"  마스크는 {1/step:.1f} fps 로만 있어 클립에는 안 쓴다 "
                  f"(영상 {fps:.0f} fps). 십자만 그린다 — "
                  f"빨간 마스크를 원하면 그 구간을 {fps:.0f} fps 로 다시 추적한다")
        idx_by_t = {float(r["t"]): int(r["idx"]) for r in good}
        tkeys = np.array(sorted(idx_by_t))

        def t2idx(t):
            return idx_by_t[float(tkeys[np.abs(tkeys - t).argmin()])]

        cap.set(cv2.CAP_PROP_POS_MSEC, t0 * 1000)
        tmp = hq / "_clip"
        shutil.rmtree(tmp, ignore_errors=True)
        tmp.mkdir()
        n = 0
        while True:
            t = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
            ok, im = cap.read()
            if not ok or t > t1:
                break
            if not a.plain:
                r = dict(ok="1", cx=np.interp(t, tt, cxs), cy=np.interp(t, tt, cys),
                         dist=np.interp(t, tt, dis))
                # 마스크는 보간이 안 된다 — 추적이 초당 2장이면 칠한 자리가 배보다 늦는다.
                # masks.npz 가 영상과 같은 fps 로 만들어졌을 때만 쓴다
                im = draw(im, r, cx0, cy0, k, mask=mask_of(t2idx(t)) if fine else None)
            if a.scale != 1.0:
                im = cv2.resize(im, None, fx=a.scale, fy=a.scale,
                                interpolation=cv2.INTER_LANCZOS4)
            cv2.imwrite(str(tmp / f"{n:05d}.png"), im)
            n += 1
        cap.release()
        mp4 = hq / f"clip_{t0:.0f}s_{t1:.0f}s.mp4"
        subprocess.run(
            ["ffmpeg", "-y", "-loglevel", "error", "-framerate", f"{fps:.6f}",
             "-i", str(tmp / "%05d.png"), "-c:v", "libx264", "-crf", str(a.crf),
             "-preset", "slow", "-pix_fmt", "yuv420p", str(mp4)], check=True)
        shutil.rmtree(tmp)
        print(f"{mp4}  ({mp4.stat().st_size/1e6:.1f} MB, {n} 장 @ {fps:.2f} fps "
              f"= {n/fps:.1f} s, 실제 속도)")

    if a.clip:
        i0, i1 = (int(s) for s in a.clip.split("-"))
        avail = sorted(i for i in rows if i0 <= i <= i1)
        if not avail:
            raise SystemExit(f"{i0}~{i1} 사이에 프레임이 없다")
        tmp = hq / "_clip"
        shutil.rmtree(tmp, ignore_errors=True)
        tmp.mkdir()
        for n, idx in enumerate(avail):
            cv2.imwrite(str(tmp / f"{n:05d}.png"), load(idx))
        mp4 = hq / f"clip_{i0}_{i1}.mp4"
        # H.264 CRF 16 — 눈으로 무손실에 가깝다. yuv420p 라야 파워포인트·브라우저가 연다
        subprocess.run(
            ["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(a.fps),
             "-i", str(tmp / "%05d.png"), "-c:v", "libx264", "-crf", str(a.crf),
             "-preset", "slow", "-pix_fmt", "yuv420p", str(mp4)], check=True)
        shutil.rmtree(tmp)
        print(f"{mp4}  ({mp4.stat().st_size/1e6:.1f} MB, {len(avail)} 장 @ {a.fps} fps)")


if __name__ == "__main__":
    main()
