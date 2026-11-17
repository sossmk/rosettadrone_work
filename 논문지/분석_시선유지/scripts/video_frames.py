#!/usr/bin/env python3
"""영상 파일에서 구간을 잘라 프레임으로 푼다 — **bag 이 없을 때** extract_bags.py 자리.

    python scripts/video_frames.py --video org.mp4 --out outputs_vid --from 2:10 --to 5:30
    python scripts/video_frames.py --video org.mp4 --out outputs_vid --from 130 --to 330 --fps 5

구간은 **사람이 정한다** (--from · --to). "1:23" 이나 초 단위 숫자 둘 다 된다.
--to 를 안 주면 끝까지. 구간을 모르면 --probe 로 길이와 해상도만 먼저 본다.

내보내는 것 — track_b.py 가 bag 에서 푼 것과 **같은 모양**이라 그대로 이어진다
    outputs_vid/frames/000000.jpg ...
    outputs_vid/frames.csv       idx, t(영상 시작부터의 초), file
    outputs_vid/camera_info.json fx, fy, cx, cy, width, height

⚠ bag 이 없으면 카메라 내부 파라미터도 없다. 중심은 화면 한가운데로 두고, fx 는 수평 화각
  (--hfov) 에서 낸다: fx = (폭/2) / tan(화각/2). 기본 77° 는 Mavic 2 Pro 사양값이다.
  **화각이 틀리면 픽셀→각도 환산만 틀어진다. 픽셀 값 자체는 영향 없다.**
  같은 영상 경로의 bag 이 따로 있으면 그쪽 camera_info 를 그대로 베끼는 편이 낫다 (--fx).
"""
import argparse
import json
import math
import shutil
import subprocess
from pathlib import Path

import cv2

HERE = Path(__file__).resolve().parent.parent


def parse_time(s):
    """'90' · '1:30' · '1:30.5' → 초."""
    if s is None:
        return None
    parts = str(s).split(":")
    try:
        sec = 0.0
        for p in parts:
            sec = sec * 60.0 + float(p)
    except ValueError:
        raise argparse.ArgumentTypeError(f"시각을 못 읽겠다: {s} (예: 90 · 1:30)")
    return sec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", type=Path, required=True)
    ap.add_argument("--out", type=Path, default=HERE / "outputs_video")
    ap.add_argument("--from", dest="t0", type=parse_time, default=0.0, help="구간 시작 (예: 2:10)")
    ap.add_argument("--to", dest="t1", type=parse_time, default=None, help="구간 끝. 없으면 끝까지")
    ap.add_argument("--fps", type=float, default=5.0, help="뽑을 초당 장수 (기본 5)")
    ap.add_argument("--hfov", type=float, default=77.0, help="수평 화각 ° (Mavic 2 Pro 77)")
    ap.add_argument("--fx", type=float, default=None, help="초점거리 px — 알면 --hfov 대신 이걸")
    ap.add_argument("--probe", action="store_true", help="길이·해상도만 보고 끝낸다")
    a = ap.parse_args()

    cap = cv2.VideoCapture(str(a.video))
    if not cap.isOpened():
        raise SystemExit(f"영상을 못 연다: {a.video}")
    src_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    dur = n / src_fps if src_fps else 0.0
    print(f"{a.video.name}: {w}×{h}, {src_fps:.2f} fps, {n} 장, 길이 {int(dur//60)}분 {dur%60:.1f}초")
    if a.probe:
        return

    t1 = dur if a.t1 is None else min(a.t1, dur)
    if not 0 <= a.t0 < t1:
        raise SystemExit(f"구간이 이상하다: {a.t0}~{t1} (영상 길이 {dur:.0f}s)")
    step = max(1, round(src_fps / a.fps))
    want = int((t1 - a.t0) * src_fps / step)
    print(f"구간 {a.t0:.1f}~{t1:.1f} s ({t1-a.t0:.0f} s) → 약 {want} 장 "
          f"(초당 {src_fps/step:.2f})")
    if want > 1500:
        print(f"⚠ {want} 장은 SAM2 에 많다. --fps 를 낮추거나 구간을 줄이는 것을 권한다")

    fd = a.out / "frames"
    fd.mkdir(parents=True, exist_ok=True)
    for p in fd.glob("*.jpg"):
        p.unlink()

    cap.release()

    # 프레임 뽑기는 ffmpeg 로 한다. OpenCV 는 깨진 패킷을 만나면 거기서 조용히 멈춘다
    # (USV.mp4 는 147 초쯤에서 "Invalid NAL unit size" 로 멈췄다). ffmpeg 는 건너뛰고 계속 간다.
    out_fps = src_fps / step
    if shutil.which("ffmpeg") is None:
        raise SystemExit("ffmpeg 가 없다 —  sudo apt install ffmpeg")
    cmd = ["ffmpeg", "-v", "error", "-nostdin",
           "-ss", f"{a.t0:.3f}", "-i", str(a.video), "-t", f"{t1 - a.t0:.3f}",
           "-vf", f"fps={out_fps:.6f}", "-q:v", "2", "-start_number", "0",
           str(fd / "%06d.jpg")]
    print("ffmpeg 로 뽑는 중 ...")
    r = subprocess.run(cmd, stderr=subprocess.PIPE, text=True)
    if r.stderr.strip():
        head = "\n".join(r.stderr.strip().splitlines()[:5])
        print(f"  ffmpeg 가 흘린 말 (깨진 구간을 건너뛴 것이면 괜찮다):\n  {head}")
    if r.returncode != 0 and not list(fd.glob("*.jpg")):
        raise SystemExit(f"ffmpeg 실패 (코드 {r.returncode})")

    files = sorted(fd.glob("*.jpg"))
    if not files:
        raise SystemExit("뽑힌 프레임이 없다 — 구간을 확인한다")
    # fps 필터는 고른 간격으로 내보내므로, 시각은 구간 시작 + n/out_fps 로 센다
    rows = [(i, a.t0 + i / out_fps) for i in range(len(files))]
    for (i, _), f in zip(rows, files):          # 혹시 번호가 비면 다시 매긴다
        if f.name != f"{i:06d}.jpg":
            f.rename(fd / f"{i:06d}.jpg")

    with (a.out / "frames.csv").open("w") as f:
        f.write("idx,t,file\n")
        for i, t in rows:
            f.write(f"{i},{t:.6f},{i:06d}.jpg\n")

    fx = a.fx if a.fx else (w / 2.0) / math.tan(math.radians(a.hfov / 2.0))
    cam = dict(fx=fx, fy=fx, cx=w / 2.0, cy=h / 2.0, width=w, height=h)
    (a.out / "camera_info.json").write_text(json.dumps(cam, indent=2))
    print(f"frames/ {len(rows)} 장 · frames.csv · camera_info.json")
    print(f"  fx = {fx:.1f} px → 1° = {fx*math.tan(math.radians(1)):.1f} px"
          + ("  (--fx 로 받은 값)" if a.fx else f"  (화각 {a.hfov:.0f}° 가정)"))
    print(f"\n다음: python scripts/pick_box.py --out {a.out}")


if __name__ == "__main__":
    main()
