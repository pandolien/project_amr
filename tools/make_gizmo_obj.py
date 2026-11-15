#!/usr/bin/env python3
"""선택한 오브젝트에 붙는 좌표계 모양(Data/Object/gizmo/axes.obj, axes.mtl)을 만든다.

가운데 구(반지름 0.10) + 화살표 3개(X 빨강, Y 초록, Z 파랑). 화살표 끝까지의 길이가 1 이므로
뷰어가 크기를 곱해서 쓴다. 파일을 직접 고쳐도 되고, 이 스크립트로 다시 만들어도 된다.
  사용: python3 tools/make_gizmo_obj.py [출력 폴더]
"""
import math
import os
import sys

SEG = 32            # 원둘레 분할 수
SPHERE_R = 0.10     # 가운데 구 반지름
SHAFT_R = 0.030     # 화살대 반지름
SHAFT_END = 0.72    # 화살대 끝 = 화살촉 시작
HEAD_R = 0.075      # 화살촉 밑면 반지름
TIP = 1.0           # 화살촉 끝


class Mesh(object):
    def __init__(self):
        self.v, self.n, self.f = [], [], []

    def vert(self, p, nrm):
        self.v.append(p)
        self.n.append(nrm)
        return len(self.v)  # 1부터 시작하는 번호 (v 와 vn 이 같은 번호)

    def tri(self, a, b, c):
        self.f.append((a, b, c))


def sphere(m, rings=16):
    idx = []
    for i in range(rings + 1):
        th = math.pi * i / rings
        row = []
        for j in range(SEG):
            ph = 2 * math.pi * j / SEG
            nrm = (math.sin(th) * math.cos(ph), math.sin(th) * math.sin(ph), math.cos(th))
            row.append(m.vert(tuple(SPHERE_R * c for c in nrm), nrm))
        idx.append(row)
    for i in range(rings):
        for j in range(SEG):
            a, b = idx[i][j], idx[i][(j + 1) % SEG]
            c, d = idx[i + 1][j], idx[i + 1][(j + 1) % SEG]
            if i > 0:
                m.tri(a, c, b)
            if i < rings - 1:
                m.tri(b, c, d)


def arrow_z(m):
    """+Z 방향 화살표. 구 안쪽에서 시작해서 밖으로 나온다."""
    z0 = SPHERE_R * 0.5
    ring = lambda r, z, nrm_fn: [m.vert((r * math.cos(2 * math.pi * j / SEG), r * math.sin(2 * math.pi * j / SEG), z), nrm_fn(2 * math.pi * j / SEG)) for j in range(SEG)]
    # 화살대 옆면
    side = lambda ph: (math.cos(ph), math.sin(ph), 0.0)
    lo, hi = ring(SHAFT_R, z0, side), ring(SHAFT_R, SHAFT_END, side)
    for j in range(SEG):
        k = (j + 1) % SEG
        m.tri(lo[j], lo[k], hi[j])
        m.tri(lo[k], hi[k], hi[j])
    # 화살촉 밑면 (아래를 향하는 고리 모양 면: 화살대 둘레 ~ 촉 밑면 둘레)
    down = lambda ph: (0.0, 0.0, -1.0)
    r_in, r_out = ring(SHAFT_R, SHAFT_END, down), ring(HEAD_R, SHAFT_END, down)
    for j in range(SEG):
        k = (j + 1) % SEG
        m.tri(r_in[j], r_in[k], r_out[j])
        m.tri(r_in[k], r_out[k], r_out[j])
    # 화살촉 옆면 (원뿔). 법선은 경사를 반영한다
    slope = HEAD_R / (TIP - SHAFT_END)
    cone = lambda ph: tuple(c / math.sqrt(1 + slope * slope) for c in (math.cos(ph), math.sin(ph), slope))
    base = ring(HEAD_R, SHAFT_END, cone)
    for j in range(SEG):
        ph = 2 * math.pi * (j + 0.5) / SEG
        tip = m.vert((0.0, 0.0, TIP), cone(ph))
        m.tri(base[j], base[(j + 1) % SEG], tip)


def rotate(m, fn):
    m.v = [fn(p) for p in m.v]
    m.n = [fn(p) for p in m.n]


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "Data", "Object", "gizmo")
    os.makedirs(out, exist_ok=True)
    parts = []  # (그룹 이름, 재질 이름, Mesh)
    c = Mesh(); sphere(c); parts.append(("center", "gizmo_center", c))
    # Z 화살표를 X, Y 로 돌린다.  +Z -> +X : (x,y,z) -> (z,y,-x)   +Z -> +Y : (x,y,z) -> (x,z,-y)
    for name, mat, fn in (("axis_x", "gizmo_x", lambda p: (p[2], p[1], -p[0])),
                          ("axis_y", "gizmo_y", lambda p: (p[0], p[2], -p[1])),
                          ("axis_z", "gizmo_z", lambda p: p)):
        a = Mesh(); arrow_z(a); rotate(a, fn); parts.append((name, mat, a))

    lines = ["# 좌표계 기즈모: 가운데 구 + 화살표 3개 (X 빨강, Y 초록, Z 파랑). 화살표 끝 = 1.0", "mtllib axes.mtl"]
    off = 0
    for name, mat, m in parts:
        lines.append("o " + name)
        lines.append("usemtl " + mat)
        for p in m.v:
            lines.append("v %.6f %.6f %.6f" % p)
        for nrm in m.n:
            lines.append("vn %.6f %.6f %.6f" % nrm)
        for a, b, c2 in m.f:
            lines.append("f %d//%d %d//%d %d//%d" % (a + off, a + off, b + off, b + off, c2 + off, c2 + off))
        off += len(m.v)
    open(os.path.join(out, "axes.obj"), "w").write("\n".join(lines) + "\n")
    mtl = []
    for mat, kd in (("gizmo_center", (0.88, 0.88, 0.90)), ("gizmo_x", (0.92, 0.20, 0.20)), ("gizmo_y", (0.25, 0.80, 0.25)), ("gizmo_z", (0.25, 0.45, 0.95))):
        mtl += ["newmtl " + mat, "Kd %.3f %.3f %.3f" % kd, "d 1.0", ""]
    open(os.path.join(out, "axes.mtl"), "w").write("\n".join(mtl))
    print("wrote", out, sum(len(m.f) for _, _, m in parts), "triangles")


main()
