#!/usr/bin/env python3
"""simul_main --lidar-csv 로 저장한 라이다 스캔(CSV)을 그래프로 그린다 (2D 스캔 기준).

사용:  python3 tools/plot_lidar_scan.py scan.csv out.png [--room-example]
  왼쪽: 위에서 본 지도 (맞은 점, 색은 거리). 오른쪽: 방위각에 따른 거리.
  --room-example: Data/Object/samples/lidar_room.urdf 의 벽/장애물 윤곽을 겹쳐 그리고, 오른쪽 그래프에 장애물 이름을 붙인다.
필요: matplotlib (pip install matplotlib)
"""
import csv
import math
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.patches import Circle, Rectangle

SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e6e5e1"
SERIES = "#2a78d6"  # 단일 계열: 팔레트 1번 (파랑)
SEQ = LinearSegmentedColormap.from_list("range", ["#bcd6f5", "#12407f"])  # 거리: 한 색조, 가까울수록 연하게

from matplotlib import font_manager

# 한글이 나오는 폰트를 설치된 것 중에서 하나 고른다 (없으면 기본 폰트: 한글이 네모로 보인다)
_installed = {f.name for f in font_manager.fontManager.ttflist}
for _name in ("Malgun Gothic", "NanumGothic", "Noto Sans CJK KR", "AppleGothic", "DejaVu Sans"):
    if _name in _installed:
        matplotlib.rcParams["font.family"] = _name
        break
matplotlib.rcParams["axes.unicode_minus"] = False


def load(path):
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            rows.append(r)
    return rows


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if len(args) != 2:
        print(__doc__)
        sys.exit(2)
    example = "--room-example" in sys.argv
    rows = [r for r in load(args[0]) if int(r["channel"]) == 0]
    az = [float(r["azimuth_deg"]) for r in rows]
    rng = [float(r["range_m"]) for r in rows]
    hit = [r["hit"] == "1" for r in rows]
    pts = [(float(r["x"]), float(r["y"])) for r, h in zip(rows, hit) if h]
    prng = [v for v, h in zip(rng, hit) if h]
    # 센서 위치는 맞은 점에서 거리/방위로 거꾸로 구하기 어려우니 첫 맞은 점으로 추정하지 않고 CSV 의 z 와 무관하게 별도로 받는다
    sx, sy, yaw = 0.0, 0.0, 0.0
    for a in sys.argv:
        if a.startswith("--sensor="):
            sx, sy, yaw = [float(v) for v in a.split("=")[1].split(",")]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.5, 4.9), gridspec_kw={"width_ratios": [1.15, 1]})
    fig.patch.set_facecolor(SURFACE)
    for ax in (ax1, ax2):
        ax.set_facecolor(SURFACE)
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
        for s in ("left", "bottom"):
            ax.spines[s].set_color(GRID)
        ax.tick_params(colors=INK2, labelsize=9)
        ax.grid(True, color=GRID, linewidth=0.8)
        ax.set_axisbelow(True)

    # ---- 왼쪽: 위에서 본 지도
    if example:
        ax1.add_patch(Rectangle((-4, -3), 8, 6, fill=False, ec="#9a998f", lw=1.5))
        ax1.add_patch(Rectangle((1.4, 0.8), 1.2, 0.8, fill=False, ec="#9a998f", lw=1.2))
        ax1.add_patch(Circle((-2.2, -1.3), 0.35, fill=False, ec="#9a998f", lw=1.2))
        ax1.add_patch(Rectangle((1.1, -1.9), 0.8, 0.8, fill=False, ec="#9a998f", lw=1.0, ls=(0, (3, 3))))
        ax1.text(1.5, -2.05, "낮은 상자\n(수평 스캔에 안 보임)", color=INK2, fontsize=8, ha="center", va="top")
    sc = ax1.scatter([p[0] for p in pts], [p[1] for p in pts], c=prng, cmap=SEQ, s=9, linewidths=0, zorder=3)
    ax1.plot([sx], [sy], marker="o", ms=8, color=INK, zorder=4)
    h = math.radians(yaw)
    ax1.annotate("", xy=(sx + 0.7 * math.cos(h), sy + 0.7 * math.sin(h)), xytext=(sx, sy),
                 arrowprops=dict(arrowstyle="-|>", color=INK, lw=1.6), zorder=4)
    ax1.text(sx - 0.15, sy - 0.25, "라이다", color=INK, fontsize=9, ha="right", va="top")
    ax1.set_aspect("equal")
    ax1.set_xlabel("x (m)", color=INK2)
    ax1.set_ylabel("y (m)", color=INK2)
    ax1.set_title("위에서 본 스캔: 맞은 점이 방 모양을 그린다", color=INK, fontsize=11, loc="left")
    cb = fig.colorbar(sc, ax=ax1, fraction=0.04, pad=0.02)
    cb.set_label("거리 (m)", color=INK2)
    cb.ax.tick_params(colors=INK2, labelsize=8)
    cb.outline.set_edgecolor(GRID)

    # ---- 오른쪽: 방위각 - 거리
    ax2.plot(az, rng, color=SERIES, lw=2.0, solid_capstyle="round")
    ax2.set_xlabel("방위각 (도, 센서 +x 기준 반시계)", color=INK2)
    ax2.set_ylabel("거리 (m)", color=INK2)
    ax2.set_title("방위각에 따른 거리: 벽은 완만한 곡선, 장애물은 계단", color=INK, fontsize=11, loc="left")
    ax2.set_xlim(min(az), max(az))
    ax2.set_ylim(0, max(rng) * 1.12)
    if example:
        # 어느 장애물에 맞았는지: 맞은 점이 장애물 윤곽 안(여유 5 cm)에 있으면 그 이름을 붙인다
        def label_of(x, y):
            if 1.35 <= x <= 2.65 and 0.75 <= y <= 1.65:
                return "상자"
            if math.hypot(x + 2.2, y + 1.3) <= 0.40:
                return "원기둥"
            return None
        groups = {}
        for a, (x, y) in zip([a for a, h2 in zip(az, hit) if h2], pts):
            n = label_of(x, y)
            if n:
                groups.setdefault(n, []).append(a)
        for n, lst in groups.items():
            ax2.annotate(n, xy=(sum(lst) / len(lst), min(r for a, r, h2 in zip(az, rng, hit) if h2 and a in lst)),
                         xytext=(0, -22), textcoords="offset points", ha="center", color=INK2, fontsize=9,
                         arrowprops=dict(arrowstyle="-", color=INK2, lw=0.8))
    fig.tight_layout()
    fig.savefig(args[1], dpi=150, facecolor=SURFACE)
    print("saved", args[1])


if __name__ == "__main__":
    main()
