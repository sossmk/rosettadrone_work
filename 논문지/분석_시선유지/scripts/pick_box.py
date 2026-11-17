#!/usr/bin/env python3
"""track_b.py 에 줄 박스 좌표를 눈으로 읽기 위한 도구.

    python scripts/pick_box.py            # 분석 구간 첫 프레임 (영상만일 때는 첫 장)
    python scripts/pick_box.py 123        # 특정 프레임
    python scripts/pick_box.py 123 --out outputs_XX

프레임에 50 px 격자와 좌표 눈금, 화면 중앙 십자를 얹어 outputs/pick_box.png 로 저장한다.
USV 를 감싸는 네모의 왼쪽위·오른쪽아래를 격자에서 읽어

    python scripts/track_b.py --box x1,y1,x2,y2

로 넘긴다. 넉넉하게 잡아도 된다 — SAM2 는 박스 안에서 물체를 알아서 찾는다.
추적이 중간에 틀어지면 그 프레임 번호로 다시 이 도구를 돌려
`--refine 번호:x1,y1,x2,y2` 로 그 지점부터 다시 잡는다.
"""
import argparse
import csv
import json
from pathlib import Path

import cv2

HERE = Path(__file__).resolve().parent.parent
OUT = HERE / "outputs"
GRID = 50


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("idx", nargs="?", type=int, help="프레임 번호 (기본: 분석 구간 첫 장)")
    ap.add_argument("--out", type=Path, default=OUT)
    ap.add_argument("--zoom", type=float, default=1.5, help="확대 배율")
    a = ap.parse_args()
    out = a.out

    # 기본 프레임: (A) 를 냈으면 그 구간의 첫 장, 영상만 볼 때는 있는 프레임의 첫 장
    idx = a.idx
    if idx is None:
        for name in ("a_frames.csv", "frames.csv"):
            f = out / name
            if f.exists():
                idx = min(int(r["idx"]) for r in csv.DictReader(f.open()))
                break
        if idx is None:
            raise SystemExit(f"{out} 에 frames.csv 가 없다 — "
                             "extract_bags.py 나 video_frames.py 를 먼저 돌린다")

    src = out / "frames" / f"{idx:06d}.jpg"
    if not src.exists():
        raise SystemExit(f"{src} 가 없다 — extract_bags.py 를 --frames 로 돌렸는지 본다")
    im = cv2.imread(str(src))
    h, w = im.shape[:2]
    cam = json.loads((out / "camera_info.json").read_text())

    for x in range(0, w, GRID):
        cv2.line(im, (x, 0), (x, h), (60, 60, 60), 1)
        cv2.putText(im, str(x), (x + 2, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)
    for y in range(0, h, GRID):
        cv2.line(im, (0, y), (w, y), (60, 60, 60), 1)
        cv2.putText(im, str(y), (2, y - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)
    cv2.drawMarker(im, (int(cam["cx"]), int(cam["cy"])), (0, 0, 255), cv2.MARKER_CROSS, 30, 2)

    dst = out / "pick_box.png"
    cv2.imwrite(str(dst), cv2.resize(im, None, fx=a.zoom, fy=a.zoom,
                                     interpolation=cv2.INTER_NEAREST))
    print(f"{dst} — 프레임 {idx}, 격자 {GRID} px, 빨간 십자가 화면 중앙 "
          f"({cam['cx']:.0f}, {cam['cy']:.0f})")
    print("USV 를 감싸는 네모를 읽어  python scripts/track_b.py --box x1,y1,x2,y2  로 넘긴다")


if __name__ == "__main__":
    main()
