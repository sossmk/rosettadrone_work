#!/usr/bin/env python3
"""AprilTag 인쇄면 생성 — mm 가 정확한 PDF 한 장.

    python3 tools/make_print.py              # tags/tag36h11-id0-120mm-A4.pdf
    python3 tools/make_print.py --mm 150     # 그림 폭을 150mm 로 (태그 100mm)

체커보드는 여기서 안 만든다 — 받아서 쓴다. 대신 **내부 코너 수와 한 칸 mm 를 자로 재서**
cameracalibrator 의 --size · --square 에 넣어야 한다. 그 두 값이 틀리면 보정값이 통째로 틀린다.

⚠ 인쇄할 때 **"페이지에 맞춤" 을 끄고 100% 로** 뽑아야 한다. 종이에 찍힌 100mm 눈금을
자로 재서 100mm 가 아니면 배율이 틀어진 것이다 — 그 상태로 보정하면 전부 틀린다.

⚠ 태그 크기(`config/tags.yaml` 의 size)는 **검은 사각형 바깥 한 변**이다. 흰 여백은 뺀다.
   tag36h11_id0.png 은 12칸 중 검은 사각형이 8칸이라 **그림 폭 × 2/3** 이 그 값이다
   (1200px 중 여백 200px·칸 100px 로 실측). 180mm 로 뽑으면 정확히 120mm.
"""
import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402
import matplotlib.image as mpimg  # noqa: E402

MM = 1.0 / 25.4          # mm → inch
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

matplotlib.rcParams["axes.unicode_minus"] = False


def _korean_font():
    """한글이 □ 로 나오면 인쇄물의 설명을 못 읽는다.

    이름(`font.family`)으로 고르면 안 된다 — 같은 "Noto Sans CJK JP" 이름으로
    한글이 없는 언어별 부분집합 파일이 잡혀서 글자가 깨졌다. fc-match 로 **파일 경로**를
    받아 그 파일을 직접 쓴다.
    """
    import subprocess
    from matplotlib.font_manager import FontProperties
    for want in ("Noto Sans CJK KR", "NanumGothic", "sans-serif:lang=ko"):
        try:
            path = subprocess.run(["fc-match", "-f", "%{file}", want],
                                  capture_output=True, text=True, timeout=5).stdout.strip()
        except Exception:
            continue
        if path and os.path.exists(path):
            return FontProperties(fname=path)
    print("⚠ 한글 폰트를 못 찾았다 — 설명 글자가 깨질 수 있다 (sudo apt install fonts-noto-cjk)")
    return FontProperties()


KO = _korean_font()

def page(w_mm, h_mm):
    """A4 한 장. 좌표는 mm, 원점은 왼쪽 아래."""
    fig = plt.figure(figsize=(w_mm * MM, h_mm * MM))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, w_mm)
    ax.set_ylim(0, h_mm)
    ax.set_axis_off()
    return fig, ax


def ruler(ax, x, y, length_mm=100.0):
    """배율 확인용 자. 인쇄본에서 이 선이 length_mm 여야 한다."""
    ax.plot([x, x + length_mm], [y, y], color="black", lw=0.8)
    for i in range(0, int(length_mm) + 1, 10):
        tall = 3.0 if i % 50 == 0 else 1.8
        ax.plot([x + i, x + i], [y, y + tall], color="black", lw=0.8)
    ax.text(x + length_mm / 2, y - 4.2, f"이 선 = {length_mm:.0f}mm (자로 재서 확인)",
            ha="center", va="top", fontsize=7, fontproperties=KO)


def read_tag_grid(png, cells=None):
    """태그 PNG → 칸 격자(bool 배열, True=검정)와 여백 칸 수.

    PNG 를 이미지로 얹지 않고 **칸마다 사각형으로 다시 그린다.** imshow 로 PDF 에
    넣었더니 그림이 대각선으로 밀려 인쇄물이 못 쓰게 나왔다. 어차피 흑백 격자라
    벡터로 그리는 편이 정확하고 파일도 작다.
    """
    g = mpimg.imread(png)
    if g.ndim == 3:
        g = g[:, :, 0]
    if g.dtype != "uint8":
        g = (g * 255).astype("uint8")
    h, w = g.shape
    if h != w:
        raise SystemExit(f"정사각형 태그가 아니다: {w}x{h}")

    if cells is None:
        # 가장 짧은 단색 구간 = 한 칸. 가운데 행·열을 함께 본다
        runs = []
        for line in (g[h // 2], g[:, w // 2]):
            n = 1
            for i in range(1, len(line)):
                if (line[i] < 128) == (line[i - 1] < 128):
                    n += 1
                else:
                    runs.append(n)
                    n = 1
            runs.append(n)
        px = min(runs)
        if px <= 0 or w % px:
            raise SystemExit(f"칸 크기를 못 읽었다 (추정 {px}px, 그림 {w}px) — --cells 로 직접 주세요")
        cells = w // px
    px = w // cells

    grid = [[g[int((r + 0.5) * px), int((c + 0.5) * px)] < 128 for c in range(cells)]
            for r in range(cells)]
    # 바깥쪽 전부 흰 줄 = 여백. 검은 사각형 한 변 = (cells - 2*여백) 칸
    margin = 0
    while margin < cells // 2 and not (any(grid[margin]) or any(row[margin] for row in grid)):
        margin += 1
    return grid, margin


def make_tag(png=None, img_mm=180.0, out=None, cells=None):
    """AprilTag 인쇄면. img_mm = 그림(흰 여백 포함) 한 변."""
    W, H = 210.0, 297.0                      # A4 세로
    png = png or os.path.join(ROOT, "tags", "tag36h11_id0.png")
    if img_mm > W - 10:
        raise SystemExit(f"A4 세로에 안 들어간다: 그림 {img_mm:.0f}mm")

    grid, margin = read_tag_grid(png, cells)
    n = len(grid)
    cell_mm = img_mm / n
    tag_mm = cell_mm * (n - 2 * margin)      # 검은 사각형 바깥 한 변

    x0 = (W - img_mm) / 2.0
    y0 = H - 12.0 - img_mm

    fig, ax = page(W, H)
    for r, row in enumerate(grid):
        c = 0
        while c < n:                          # 가로로 붙은 검은 칸은 한 덩어리로 — 이음매 방지
            if not row[c]:
                c += 1
                continue
            c2 = c
            while c2 + 1 < n and row[c2 + 1]:
                c2 += 1
            ax.add_patch(Rectangle(
                (x0 + c * cell_mm, y0 + (n - 1 - r) * cell_mm),   # PNG 는 위에서 아래로
                (c2 - c + 1) * cell_mm, cell_mm, facecolor="black", edgecolor="none"))
            c = c2 + 1

    ax.text(x0, y0 - 10.0, f"AprilTag 36h11 ID 0 — 그림 {img_mm:.0f}mm",
            ha="left", va="top", fontsize=10, fontproperties=KO)
    ax.text(x0, y0 - 18.0, f"config/tags.yaml   size: {tag_mm / 1000:.3f}",
            ha="left", va="top", fontsize=8.5, family="monospace")
    ax.text(x0 + 62.0, y0 - 18.0, f"← 검은 사각형 바깥 한 변 {tag_mm:.0f}mm",
            ha="left", va="top", fontsize=8.5, fontproperties=KO)
    ax.text(x0, y0 - 25.0,
            "인쇄 후 검은 사각형을 자로 재서 위 값과 다르면 tags.yaml 을 잰 값으로 고칠 것",
            ha="left", va="top", fontsize=7, fontproperties=KO)
    ax.text(x0, y0 - 31.0, "인쇄: 100% (페이지에 맞춤 끄기) · 흰 여백을 잘라내지 말 것",
            ha="left", va="top", fontsize=7, fontproperties=KO)
    ruler(ax, x0, 18.0)

    out = out or os.path.join(ROOT, "tags", f"tag36h11-id0-{tag_mm:.0f}mm-A4.pdf")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.savefig(out)
    plt.close(fig)
    print(f"✓ {out}")
    print(f"  검은 사각형 한 변 {tag_mm:.1f}mm → tags.yaml size: {tag_mm / 1000:.3f}")
    print("  검출 가늠 (fx≈1250): "
          + " · ".join(f"{d}m→{1250 * tag_mm / 1000 / d:.0f}px" for d in (3, 5, 8)))
    return out


def main():
    ap = argparse.ArgumentParser(description="AprilTag 인쇄면 생성")
    ap.add_argument("--mm", type=float, default=180.0, help="태그 그림 한 변 mm (기본 180 → 태그 120)")
    ap.add_argument("--png", help="태그 원본 PNG (기본 tags/tag36h11_id0.png)")
    ap.add_argument("--cells", type=int, help="태그 그림의 한 변 칸 수 (기본: 그림에서 읽는다)")
    args = ap.parse_args()
    make_tag(args.png, args.mm, cells=args.cells)


if __name__ == "__main__":
    main()
