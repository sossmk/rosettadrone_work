#!/usr/bin/env python3
"""(B) SAM2 로 USV 를 추적해 화면 중앙과의 차이 (dx, dy) 를 낸다.

    python scripts/pick_box.py 3                    # 먼저 첫 프레임을 보고 박스 좌표를 읽는다
    python scripts/track_b.py --box 300,165,395,280 # 그 박스로 추종 구간 전체
    python scripts/track_b.py --out outputs_XX --box 310,120,400,230
    python scripts/track_b.py --out outputs_vid --box ... --from 1:30 --to 4:00   # 구간을 직접
    python scripts/track_b.py --refine 250:310,140,380,250   # 놓친 지점에서 박스 재지정
    python scripts/track_b.py --overlay              # 검수용 영상도 만든다

놓치면 그 프레임에서 박스를 다시 준다 — --refine 은 "프레임:x1,y1,x2,y2" 를 여러 번 쓸 수 있다.
SAM2 는 같은 object_id 에 새 박스를 받으면 그 지점부터 다시 전파한다.

내보내는 것
    outputs/b_track.csv     idx, t, cx, cy, area, dx, dy, dist, ok
                            dx, dy = USV 마스크 중심 − 화면 중심 (px, + 는 오른쪽·아래)
                            dist   = 그 둘의 거리 (px)
    outputs/b_overlay.mp4   마스크를 겹친 검수용 영상 (--overlay)
"""
import argparse
import csv
import math
import json
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).resolve().parent.parent
OUT = HERE / "outputs"
FRAMES = OUT / "frames"
CKPT = HERE / "checkpoints" / "sam2.1_hiera_small.pt"
CFG = "configs/sam2.1/sam2.1_hiera_s.yaml"

# DT0503 의 값 — 첫 프레임 frames/000003.jpg 를 눈으로 보고 잡은 MAIV-1 자리.
# 다른 비행이면 pick_box.py 로 보고 --box 로 준다
DEFAULT_BOX = (300, 165, 395, 280)
OBJ = 1


def parse_time(s):
    """'90' · '1:30' → 초. video_frames.py 와 같은 규칙."""
    sec = 0.0
    for part in str(s).split(":"):
        sec = sec * 60.0 + float(part)
    return sec


def parse_box(s):
    v = [float(x) for x in s.split(",")]
    if len(v) != 4:
        raise argparse.ArgumentTypeError("x1,y1,x2,y2 네 개를 준다")
    return v


def parse_refine(s):
    frame, _, box = s.partition(":")
    return int(frame), parse_box(box)


def main():
    global OUT, FRAMES
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=OUT, help="extract_bags.py 가 쓴 폴더")
    ap.add_argument("--start", type=int, help="첫 프레임 idx")
    ap.add_argument("--end", type=int, help="끝 프레임 idx")
    ap.add_argument("--from", dest="t0", type=parse_time, help="구간 시작 초 (예: 2:10)")
    ap.add_argument("--to", dest="t1", type=parse_time, help="구간 끝 초")
    ap.add_argument("--box", type=parse_box, default=DEFAULT_BOX,
                    help="첫 프레임의 USV 자리 x1,y1,x2,y2 — pick_box.py 로 읽는다")
    ap.add_argument("--refine", type=parse_refine, action="append", default=[],
                    help="프레임:x1,y1,x2,y2 — 놓친 지점에서 다시 지정 (여러 번 가능)")
    ap.add_argument("--save-masks", dest="save_masks", action="store_true",
                    help="마스크를 masks.npz 로 남긴다 — export_hq.py 가 빨갛게 칠할 때 쓴다")
    ap.add_argument("--overlay", action="store_true")
    a = ap.parse_args()
    OUT = a.out
    FRAMES = OUT / "frames"

    # 구간 정하기 — 우선순위: --start/--end > --from/--to(초) > a_frames.csv > 있는 프레임 전부
    #   a_frames.csv 가 있으면 (A) 와 같은 구간이어야 맞댈 수 있으므로 그것을 따른다.
    #   bag 없이 영상만 볼 때는 a_frames.csv 가 없고, 구간은 video_frames.py 에서 이미 잘랐거나
    #   여기서 --from/--to 로 자른다.
    times = {int(r["idx"]): float(r["t"]) for r in csv.DictReader((OUT / "frames.csv").open())}
    if not times:
        raise SystemExit("frames.csv 가 비었다")
    af = OUT / "a_frames.csv"
    if a.t0 is not None or a.t1 is not None:
        t0 = a.t0 if a.t0 is not None else min(times.values())
        t1 = a.t1 if a.t1 is not None else max(times.values())
        sel = [i for i, t in times.items() if t0 <= t <= t1]
        if not sel:
            raise SystemExit(f"{t0}~{t1} 초 안에 프레임이 없다 "
                             f"(있는 범위 {min(times.values()):.1f}~{max(times.values()):.1f})")
        lo, hi = min(sel), max(sel)
    elif af.exists():
        fr = [int(r["idx"]) for r in csv.DictReader(af.open())]
        lo, hi = min(fr), max(fr)
    else:
        lo, hi = min(times), max(times)
    a.start = a.start if a.start is not None else lo
    a.end = a.end if a.end is not None else hi

    cam = json.loads((OUT / "camera_info.json").read_text())
    cx0, cy0 = cam["cx"], cam["cy"]

    # SAM2 는 폴더 안 JPEG 를 0 번부터 센다. 구간만 따로 모아 번호를 맞춘다
    work = OUT / "_b_frames"
    work.mkdir(exist_ok=True)
    for p in work.glob("*.jpg"):
        p.unlink()
    idxs = list(range(a.start, a.end + 1))
    for i, idx in enumerate(idxs):
        # 링크 대상은 **절대 경로**여야 한다 — --out 을 상대 경로로 주면
        # 링크가 _b_frames/ 기준으로 풀려 깨진다
        (work / f"{i:06d}.jpg").symlink_to((FRAMES / f"{idx:06d}.jpg").resolve())

    from sam2.build_sam import build_sam2_video_predictor
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"device {dev} · 프레임 {len(idxs)} 장 ({a.start}~{a.end})")
    pred = build_sam2_video_predictor(CFG, str(CKPT), device=dev)

    with torch.inference_mode(), torch.autocast(dev, dtype=torch.bfloat16):
        # 프레임이 많으면 영상을 GPU 가 아니라 RAM 에 둔다. SAM2 는 1024×1024 float 로
        # 한 장당 약 12 MB 를 쓰므로, 16 GB GPU 는 500 장쯤에서 넘친다
        big = len(idxs) > 400
        if big:
            print(f"  {len(idxs)} 장 — 영상을 RAM 에 두고 돌린다 (느리지만 GPU 메모리를 아낀다)")
        state = pred.init_state(video_path=str(work),
                                offload_video_to_cpu=big, offload_state_to_cpu=big)
        pred.add_new_points_or_box(state, frame_idx=0, obj_id=OBJ,
                                   box=np.array(a.box, dtype=np.float32))
        for frame, box in a.refine:
            if not a.start <= frame <= a.end:
                raise SystemExit(f"--refine {frame} 은 {a.start}~{a.end} 밖이다")
            pred.add_new_points_or_box(state, frame_idx=frame - a.start, obj_id=OBJ,
                                       box=np.array(box, dtype=np.float32))
        masks = {}
        for i, ids, logits in pred.propagate_in_video(state):
            m = (logits[ids.index(OBJ)] > 0).cpu().numpy()[0]
            masks[i] = m

    ts = times
    rows = []
    for i, idx in enumerate(idxs):
        m = masks.get(i)
        area = int(m.sum()) if m is not None else 0
        if area == 0:
            rows.append(dict(idx=idx, t=ts[idx], cx="", cy="", area=0,
                             dx="", dy="", dist="", ok=0))
            continue
        ys, xs = np.nonzero(m)
        cx, cy = float(xs.mean()), float(ys.mean())
        dx, dy = cx - cx0, cy - cy0
        rows.append(dict(idx=idx, t=ts[idx], cx=round(cx, 2), cy=round(cy, 2), area=area,
                         dx=round(dx, 2), dy=round(dy, 2),
                         dist=round(math.hypot(dx, dy), 2), ok=1))
    if a.save_masks:
        # 마스크는 1920x1080 bool 이라 그대로 두면 한 장에 2 MB 다. 네모로 잘라
        # 비트로 눌러 담는다 (배 모양이라 대부분이 빈 칸이다)
        z = {}
        for i, idx in enumerate(idxs):
            m = masks.get(i)
            if m is None or not m.any():
                continue
            ys, xs = np.nonzero(m)
            y0, y1, x0, x1 = ys.min(), ys.max() + 1, xs.min(), xs.max() + 1
            z[f"{idx}_box"] = np.array([x0, y0, x1, y1], dtype=np.int32)
            z[f"{idx}_bits"] = np.packbits(m[y0:y1, x0:x1])
        z["shape"] = np.array([cam["height"], cam["width"]], dtype=np.int32)
        np.savez_compressed(OUT / "masks.npz", **z)
        print(f"masks.npz — {len(z)//2} 장 "
              f"({(OUT / 'masks.npz').stat().st_size/1e6:.1f} MB)")

    with (OUT / "b_track.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    good = [r for r in rows if r["ok"]]
    print(f"b_track.csv  {len(good)}/{len(rows)} 장에서 마스크를 찾았다")
    if good:
        ar = np.array([r["area"] for r in good])
        print(f"  마스크 넓이 중앙값 {np.median(ar):.0f} px²  (최소 {ar.min()} · 최대 {ar.max()})")
        print(f"  dx 평균 {np.mean([r['dx'] for r in good]):+.1f} px · "
              f"dy 평균 {np.mean([r['dy'] for r in good]):+.1f} px · "
              f"중심까지 거리 중앙값 {np.median([r['dist'] for r in good]):.0f} px")

    if a.overlay:
        import cv2
        vw = cv2.VideoWriter(str(OUT / "b_overlay.mp4"), cv2.VideoWriter_fourcc(*"mp4v"),
                             10.0, (cam["width"], cam["height"]))
        # 표식 크기는 화면 폭에 맞춘다 — 1920 짜리에 16 px 십자를 그리면 검수할 때 안 보인다
        k = cam["width"] / 816.0
        sz, th = int(24 * k), max(1, int(2 * k))
        for i, idx in enumerate(idxs):
            im = cv2.imread(str(FRAMES / f"{idx:06d}.jpg"))
            m = masks.get(i)
            if m is not None and m.any():
                im[m] = (0.5 * im[m] + np.array([0, 0, 128])).astype(np.uint8)
                r = rows[i]
                c = (int(r["cx"]), int(r["cy"]))
                cv2.drawMarker(im, c, (255, 255, 255), cv2.MARKER_CROSS, sz, th + 1)
                cv2.line(im, (int(cx0), int(cy0)), c, (255, 255, 255), max(1, th // 2))
            # 화면 중앙 — 노란 동그라미와 십자. 여기서 얼마나 떨어졌는지가 재는 값이다
            cv2.circle(im, (int(cx0), int(cy0)), int(10 * k), (0, 255, 255), th)
            cv2.drawMarker(im, (int(cx0), int(cy0)), (0, 255, 255), cv2.MARKER_CROSS,
                           int(40 * k), th)
            cv2.putText(im, f"{idx}  dist={rows[i]['dist']} px", (int(10 * k), int(30 * k)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7 * k, (0, 255, 255), th)
            vw.write(im)
        vw.release()
        print("b_overlay.mp4 — 눈으로 검수할 것")


if __name__ == "__main__":
    main()
