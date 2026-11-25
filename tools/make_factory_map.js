// 공장/사무동 지도를 만든다 (사용자가 그린 도면 4장: 바깥, 1층, 2층, 3층 기준). 기존 마을 지도를 대신한다 (마을은 git 기록에 남아 있다).
//   node tools/make_factory_map.js [Data/Object 폴더]
// 만드는 것
//   factory_map.glb          지도 모델 (노드: Site, Lanterns, WH1, WH1Roof, WH2, WH2Roof, F1, F2, F3, Roof)
//   ground.txt               땅 높이 표 (부지 안은 평평한 1 m, 밖은 걸을 수 없다)
//   villagers/layout.txt     여러 층: 바닥판(slab), 계단(ramp), 벽(wall), 층 가림(hide)
//   villagers/obstacles.txt  나무/가로등 같은 원 장애물
//   villagers/waypoints.txt  주민 순찰 지점
//   villagers/portals.txt    (이 지도에는 B 키로 들어가는 방이 없어서 비어 있다)
// 좌표는 엔진 좌표(Z-up, m). GLB 에는 three.js 좌표(x, z, -y)로 적는다.
'use strict';
const fs = require('fs');
const path = require('path');
const outDir = process.argv[2] || path.join(__dirname, '..', 'Data', 'Object');

// ---------------------------------------------------------------- 도형
const lin = (c) => { const f = (v) => { v /= 255; return v <= 0.04045 ? v / 12.92 : Math.pow((v + 0.055) / 1.055, 2.4); }; return [f((c >> 16) & 255), f((c >> 8) & 255), f(c & 255)]; };
function Mesh() { this.p = []; this.n = []; this.c = []; }
Mesh.prototype.tri = function (a, b, c, col) {
  const u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]], v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
  let n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
  const l = Math.hypot(n[0], n[1], n[2]) || 1; n = n.map((x) => x / l);
  const rgb = lin(col);
  [a, b, c].forEach((q) => { this.p.push(q[0], q[2], -q[1]); this.n.push(n[0], n[2], -n[1]); this.c.push(rgb[0], rgb[1], rgb[2]); });   // 엔진(x,y,z) -> three(x,z,-y)
};
Mesh.prototype.quad = function (a, b, c, d, col) { this.tri(a, b, c, col); this.tri(a, c, d, col); };
Mesh.prototype.box = function (x0, y0, z0, x1, y1, z1, col) {
  if (x1 < x0) { const t = x0; x0 = x1; x1 = t; } if (y1 < y0) { const t = y0; y0 = y1; y1 = t; } if (z1 < z0) { const t = z0; z0 = z1; z1 = t; }
  this.quad([x1, y0, z0], [x1, y1, z0], [x1, y1, z1], [x1, y0, z1], col);
  this.quad([x0, y0, z1], [x0, y1, z1], [x0, y1, z0], [x0, y0, z0], col);
  this.quad([x1, y1, z0], [x0, y1, z0], [x0, y1, z1], [x1, y1, z1], col);
  this.quad([x0, y0, z0], [x1, y0, z0], [x1, y0, z1], [x0, y0, z1], col);
  this.quad([x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1], col);
  this.quad([x0, y1, z0], [x1, y1, z0], [x1, y0, z0], [x0, y0, z0], col);
};
Mesh.prototype.flat = function (x0, y0, x1, y1, z, col) { this.quad([x0, y0, z], [x1, y0, z], [x1, y1, z], [x0, y1, z], col); };   // 위를 보는 면
Mesh.prototype.cyl = function (cx, cy, r, z0, z1, col, n) {   // 세운 원기둥
  n = n || 10;
  for (let i = 0; i < n; i++) {
    const a0 = 2 * Math.PI * i / n, a1 = 2 * Math.PI * (i + 1) / n;
    const p = (a, z) => [cx + r * Math.cos(a), cy + r * Math.sin(a), z];
    this.quad(p(a0, z0), p(a1, z0), p(a1, z1), p(a0, z1), col);
    this.tri([cx, cy, z1], p(a0, z1), p(a1, z1), col);
  }
};
Mesh.prototype.cone = function (cx, cy, r, z0, z1, col, n) {
  n = n || 10;
  for (let i = 0; i < n; i++) {
    const a0 = 2 * Math.PI * i / n, a1 = 2 * Math.PI * (i + 1) / n;
    this.tri([cx + r * Math.cos(a0), cy + r * Math.sin(a0), z0], [cx + r * Math.cos(a1), cy + r * Math.sin(a1), z0], [cx, cy, z1], col);
  }
};
Mesh.prototype.ball = function (cx, cy, cz, r, col) {
  const W = 8, H = 6, pt = (i, j) => { const th = Math.PI * j / H, ph = 2 * Math.PI * i / W; return [cx + r * Math.sin(th) * Math.cos(ph), cy + r * Math.sin(th) * Math.sin(ph), cz + r * Math.cos(th)]; };
  for (let j = 0; j < H; j++) for (let i = 0; i < W; i++) { const a = pt(i, j), b = pt(i + 1, j), c = pt(i + 1, j + 1), d = pt(i, j + 1); if (j > 0) this.tri(a, b, d, col); if (j < H - 1) this.tri(b, c, d, col); }
};
Mesh.prototype.stairs = function (x0, y0, x1, y1, zA, zB, alongY, col, col2) {   // 층계 모양 (바닥 zA 에서 위로 zB 까지, 밑은 꽉 찬 덩어리)
  const N = 16;
  for (let i = 0; i < N; i++) {
    const t0 = i / N, t1 = (i + 1) / N, tm = (i + 0.5) / N;
    const top = zA + (zB - zA) * tm;
    const lo = Math.min(zA, zB) - 0.02;
    if (alongY) { const ya = y0 + (y1 - y0) * t0, yb = y0 + (y1 - y0) * t1; this.box(x0, ya, lo, x1, yb, top, i % 2 ? col : col2); }
    else { const xa = x0 + (x1 - x0) * t0, xb = x0 + (x1 - x0) * t1; this.box(xa, y0, lo, xb, y1, top, i % 2 ? col : col2); }
  }
};

// ---------------------------------------------------------------- 지도 데이터
const nodes = {};   // 이름 -> Mesh
const M = (name) => nodes[name] || (nodes[name] = new Mesh());
const layout = { slabs: [], ramps: [], walls: [], hides: [] };
const Z = [1.0, 4.2, 7.4], ZR = 10.6, FH = 3.2;   // 층 바닥 높이, 지붕 밑, 층 높이
const f2 = (v) => (Math.round(v * 1000) / 1000).toString();
function phys(x0, y0, x1, y1, thick, zmin, zmax) { layout.walls.push([x0, y0, x1, y1, thick, zmin, zmax]); }
// 벽: 눈에 보이는 상자 + 막는 선분. 가로(y 고정) 또는 세로(x 고정)
function wallSeg(node, x0, y0, x1, y1, thick, zlo, zhi, col, physical) {
  const h = thick / 2;
  if (Math.abs(y1 - y0) < 1e-9) M(node).box(Math.min(x0, x1), y0 - h, zlo - 0.02, Math.max(x0, x1), y0 + h, zhi, col);
  else M(node).box(x0 - h, Math.min(y0, y1), zlo - 0.02, x0 + h, Math.max(y0, y1), zhi, col);
  if (physical !== false) phys(x0, y0, x1, y1, thick, zlo, zhi);
}
// 문틈이 있는 벽. axis 'x': y 가 고정이고 x 가 a0..a1. gaps: [[g0,g1],...]
function wallGaps(node, axis, fixed, a0, a1, gaps, thick, zlo, zhi, col) {
  const g = gaps.slice().sort((p, q) => p[0] - q[0]);
  let cur = a0;
  const seg = (s, e) => { if (e - s < 1e-6) return; if (axis === 'x') wallSeg(node, s, fixed, e, fixed, thick, zlo, zhi, col); else wallSeg(node, fixed, s, fixed, e, thick, zlo, zhi, col); };
  g.forEach((q) => {
    seg(cur, q[0]);
    cur = q[1];
    const h = thick / 2;   // 문 위 가로대 (눈에 보이기만 한다)
    if (zhi - zlo > 2.6) { if (axis === 'x') M(node).box(q[0], fixed - h, zlo + 2.4, q[1], fixed + h, zhi, col); else M(node).box(fixed - h, q[0], zlo + 2.4, fixed + h, q[1], zhi, col); }
  });
  seg(cur, a1);
}
function slab(node, x0, y0, x1, y1, z, thick, col) {   // 바닥판: 보이는 판 + 걸을 수 있는 면
  M(node).box(x0, y0, z - thick, x1, y1, z, col);
  layout.slabs.push([x0, y0, x1, y1, z]);
}
// 구멍(계단 통로)이 있는 바닥판: 구멍을 뺀 칸들로 쪼갠다
function slabHoles(node, x0, y0, x1, y1, z, thick, col, holes) {
  const xs = [x0, x1], ys = [y0, y1];
  holes.forEach((h) => { xs.push(h[0], h[2]); ys.push(h[1], h[3]); });
  const ux = Array.from(new Set(xs.filter((v) => v >= x0 && v <= x1))).sort((a, b) => a - b), uy = Array.from(new Set(ys.filter((v) => v >= y0 && v <= y1))).sort((a, b) => a - b);
  const inHole = (cx, cy) => holes.some((h) => cx > h[0] && cx < h[2] && cy > h[1] && cy < h[3]);
  for (let j = 0; j + 1 < uy.length; j++) {
    let start = null;
    for (let i = 0; i + 1 < ux.length; i++) {
      const hole = inHole((ux[i] + ux[i + 1]) / 2, (uy[j] + uy[j + 1]) / 2);
      if (!hole && start === null) start = ux[i];
      if ((hole || i + 2 >= ux.length) && start !== null) { slab(node, start, uy[j], hole ? ux[i] : ux[i + 1], uy[j + 1], z, thick, col); start = null; }
    }
  }
}
// 계단: 가로(x) 또는 세로(y)로 올라간다. 눈에 보이는 층계 + 걸을 수 있는 경사 + 옆벽 + 위쪽 끝 막이
function stair(node, x0, x1, ya, yb, zA, zB, zLow, zHigh, col, col2) {   // ya(높이 zA) -> yb(높이 zB). zLow/zHigh 는 벽이 막는 높이 범위
  const lo = Math.min(ya, yb), hi = Math.max(ya, yb);
  M(node).stairs(x0, ya, x1, yb, zA, zB, true, col, col2);
  layout.ramps.push([x0, lo, x1, hi, ya < yb ? zA : zB, ya < yb ? zB : zA, 'y']);
  wallSeg(node, x0, lo, x0, hi, 0.2, zLow, zHigh, 0x6b6f78, true);
  wallSeg(node, x1, lo, x1, hi, 0.2, zLow, zHigh, 0x6b6f78, true);
  // 위쪽 끝(높은 쪽)에 가로 막이: 아래층에서 계단 밑으로 파고들지 못하게 (층 높이 범위)
  const yTop = zB > zA ? yb : ya;
  wallSeg(node, x0, yTop, x1, yTop, 0.2, zLow, zHigh, 0x6b6f78, true);
}
function rail(node, x0, y0, x1, y1, zBase, col) {   // 난간: 보이는 것 + 막는 것
  wallSeg(node, x0, y0, x1, y1, 0.1, zBase, zBase + 1.1, col || 0x5a5f6b, true);
}
function window_(node, axis, fixed, a, z, outward) {   // 창문: 바깥 면에 붙인 유리판 (보이기만 한다)
  const g = 0x4f7fa8, w = 1.6;
  if (axis === 'x') { const y0 = fixed + (outward > 0 ? 0.15 : -0.2), y1 = y0 + 0.05; M(node).box(a - w / 2, y0, z, a + w / 2, y1, z + 1.3, g); }
  else { const x0 = fixed + (outward > 0 ? 0.15 : -0.2), x1 = x0 + 0.05; M(node).box(x0, a - w / 2, z, x1, a + w / 2, z + 1.3, g); }
}
// 가구: 상자 하나. solid 면 걸어서 지나가지 못한다 (긴 쪽을 따라 선분으로 막는다)
function furn(node, cx, cy, sx, sy, h, z, col, solid) {
  M(node).box(cx - sx / 2, cy - sy / 2, z, cx + sx / 2, cy + sy / 2, z + h, col);
  if (solid !== false) {
    const half = Math.min(sx, sy) / 2;
    if (sx >= sy) phys(cx - (sx / 2 - half), cy, cx + (sx / 2 - half), cy, sy, z, z + FH);
    else phys(cx, cy - (sy / 2 - half), cx, cy + (sy / 2 - half), sx, z, z + FH);
  }
}
const obstacles = [];   // [x, y, r]

// ---------------------------------------------------------------- 도면 -> 월드 좌표
// 바깥 도면(1010x650): 부지 전체. 왼쪽 위 창고1, 왼쪽 아래 차고지, 오른쪽 위 창고2, 오른쪽 아래 건물, 건물 입구는 왼쪽 변 아래쪽, 그 왼쪽 차고지 쪽에 바깥 계단.
// 위쪽이 북(+y), 오른쪽이 동(+x). 부지 90 x 56 m 를 도면 비율대로 나눈다.
const SX0 = -45, SX1 = 45, SY0 = -28, SY1 = 28;
const GRASS = 0x6d9a4c, ASPHALT = 0x4a4d55, CONCRETE = 0xb4b8bd, LINE = 0xe8e8d8, FENCE = 0x7d838c;
const XD = 17.1, YW1 = 4.3, YW2 = -3.2;   // 창고1|창고2 경계 x, 창고1 아래쪽 y(차고지와 닿는 선), 창고2 아래쪽 y(건물과 닿는 선)
// 건물 바닥 도면(1,2,3층 공통): 가로 25 m, 세로 21.5 m. nx 0..1 은 서->동, ny 0..1 은 북->남.
const FX0 = 18, FW = 25, FY0 = -5, FHH = 21.5;
const wx = (n) => FX0 + FW * n, wy = (n) => FY0 - FHH * n;
const BX0 = wx(0), BX1 = wx(1), BY0 = wy(1), BY1 = wy(0);   // 남쪽 y(작다) .. 북쪽 y
// (Z, ZR, FH 는 위에서 선언했다)
const CW = 0x9aa3ad, IW = 0xdfe3e8, FLOORC = [0xc9ccd2, 0xb8a98f, 0xb9bec7];
const T = 0.3;
const level = (k) => ['F1', 'F2', 'F3'][k];
const gp = (c, w) => [c - w / 2, c + w / 2];
// 계단통 (세 층 도면에서 같은 자리): x 26.0..37.0, 위 칸 A(y -5..-9.41), 아래 칸 B(y -9.41..-13.78)
const SCX0 = wx(0.32), SCX1 = wx(0.76), YA0 = FY0, YAB = wy(0.205), YB1 = wy(0.4085);
const LANE = { AN: (YA0 + (YA0 - 2.2)) / 2, AS: YA0 - 3.3, BN: YAB - 1.1, BS: YAB - 3.3 };   // 길(레인) 가운데 y
const DIV_A = YA0 - 2.2, DIV_B = YAB - 2.2;
const FL0 = SCX0 + 0.5, FL1 = SCX1 - 0.5;   // 계단 경사의 x 범위


// ---------------------------------------------------------------- 부지
{
  const s = M('Site');
  s.flat(SX0 - 8, SY0 - 14, SX1 + 8, SY1 + 6, 0.97, GRASS);                // 풀밭
  s.flat(SX0 + 1, SY0 + 0.5, XD, YW1, 1.0, ASPHALT);                      // 차고지
  s.flat(XD, SY0 + 0.5, SX1 - 0.5, YW2, 1.0, CONCRETE);                   // 건물 둘레 바닥
  s.flat(SX0 - 8, SY0 - 14, SX1 + 8, SY0, 0.98, ASPHALT);                 // 정문 밖 도로
  s.flat(-27, SY0, -15, SY0 + 6, 1.0, CONCRETE);                          // 정문에서 들어오는 길
  for (let i = 0; i <= 12; i++) s.flat(SX0 + 2 + i * 3.6, -26.5, SX0 + 2.1 + i * 3.6, -12.5, 1.01, LINE);   // 주차 칸 선
  s.flat(SX0 + 2, -12.5, 14, -12.4, 1.01, LINE);
  const post = (x, y) => { s.box(x - 0.08, y - 0.08, 1.0, x + 0.08, y + 0.08, 2.4, FENCE); };
  const fenceLine = (x0, y0, x1, y1) => {
    const len = Math.hypot(x1 - x0, y1 - y0), n = Math.max(1, Math.round(len / 3));
    for (let i = 0; i <= n; i++) post(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n);
    const hor = Math.abs(y1 - y0) < 1e-9;
    [1.5, 2.1].forEach((z) => { if (hor) s.box(Math.min(x0, x1), y0 - 0.03, z, Math.max(x0, x1), y0 + 0.03, z + 0.1, FENCE); else s.box(x0 - 0.03, Math.min(y0, y1), z, x0 + 0.03, Math.max(y0, y1), z + 0.1, FENCE); });
  };
  fenceLine(SX0, SY0, -27, SY0); fenceLine(-15, SY0, SX1, SY0); fenceLine(SX0, SY1, SX1, SY1); fenceLine(SX0, SY0, SX0, SY1); fenceLine(SX1, SY0, SX1, SY1);
  // 정문 (남쪽 울타리의 열린 곳, x -27..-15)
  s.box(-27.4, SY0 - 0.3, 1.0, -26.8, SY0 + 0.3, 4.0, 0x8c929c); s.box(-15.2, SY0 - 0.3, 1.0, -14.6, SY0 + 0.3, 4.0, 0x8c929c);
  s.box(-27.4, SY0 - 0.25, 3.5, -14.6, SY0 + 0.25, 4.2, 0x2f5f8f);
  // 경비실 (정문 옆)
  s.box(-13.5, SY0 + 1.0, 1.0, -10.5, SY0 + 3.4, 3.4, 0xd9d3c2); s.box(-13.7, SY0 + 0.8, 3.4, -10.3, SY0 + 3.6, 3.6, 0x555b66); s.box(-13.5, SY0 + 0.95, 1.6, -10.5, SY0 + 1.05, 2.6, 0x4f7fa8);
  [[-13.5, SY0 + 1.0, -10.5, SY0 + 1.0], [-13.5, SY0 + 3.4, -10.5, SY0 + 3.4], [-13.5, SY0 + 1.0, -13.5, SY0 + 3.4], [-10.5, SY0 + 1.0, -10.5, SY0 + 3.4]].forEach((q) => phys(q[0], q[1], q[2], q[3], 0.1, -5, 99));
  obstacles.push([-12, SY0 + 2.2, 1.8]);
  // 차량: 트럭 2줄(위/아래), 승용차
  const truck = (cx, cy, col) => {
    s.box(cx - 1.0, cy - 2.4, 1.35, cx + 1.0, cy + 1.2, 3.1, col); s.box(cx - 1.0, cy + 1.2, 1.35, cx + 1.0, cy + 2.4, 2.7, 0xe6e9ee); s.box(cx - 0.95, cy + 2.1, 2.0, cx + 0.95, cy + 2.4, 2.6, 0x4f7fa8);
    [[-1.05, -1.6], [1.05, -1.6], [-1.05, 1.6], [1.05, 1.6]].forEach((w) => s.box(cx + w[0] - 0.12, cy + w[1] - 0.45, 1.0, cx + w[0] + 0.12, cy + w[1] + 0.45, 1.9, 0x1d1f24));
    phys(cx, cy - 1.4, cx, cy + 1.4, 2.0, -5, 99);
    obstacles.push([cx, cy - 1.5, 1.4], [cx, cy, 1.4], [cx, cy + 1.5, 1.4]);
  };
  const car = (cx, cy, col) => {
    s.box(cx - 0.9, cy - 2.1, 1.25, cx + 0.9, cy + 2.1, 2.0, col); s.box(cx - 0.8, cy - 1.0, 2.0, cx + 0.8, cy + 1.0, 2.7, 0x9fc6e3);
    [[-0.9, -1.3], [0.9, -1.3], [-0.9, 1.3], [0.9, 1.3]].forEach((w) => s.box(cx + w[0] - 0.1, cy + w[1] - 0.35, 1.0, cx + w[0] + 0.1, cy + w[1] + 0.35, 1.7, 0x1d1f24));
    phys(cx, cy - 0.9, cx, cy + 0.9, 1.8, -5, 99);
    obstacles.push([cx, cy - 1.0, 1.2], [cx, cy + 1.0, 1.2]);
  };
  [-41.6, -34.4, -27.2, -20.0].forEach((x, i) => truck(x, -22.5, i % 2 ? 0x3f78b5 : 0xd9822b));
  [-41.6, -34.4, -27.2].forEach((x, i) => truck(x, -4.0 + 0 * i, i % 2 ? 0xd9822b : 0x3f78b5));
  car(-8, -22.5, 0xc23b3b); car(-4.5, -22.5, 0xe8e8e8); car(-1, -22.5, 0x2f6f4a); car(-8, -6, 0x2f6f4a); car(-4.5, -6, 0xc23b3b);
  [[-24, -14], [-16, -14], [-8, -14], [0, -14]].forEach((c) => { s.cone(c[0], c[1], 0.3, 1.0, 1.7, 0xff7a1a, 8); });
  // 나무
  [[-43.5, -26.5], [43.5, -26.5], [30, -27.2], [8, -27], [44, -1.5], [-43.5, 2.5]].forEach((t) => {
    s.cyl(t[0], t[1], 0.25, 1.0, 2.6, 0x7a5232, 8); s.ball(t[0], t[1], 3.6, 1.7, 0x3f7f3c); s.ball(t[0] + 0.5, t[1] + 0.3, 4.5, 1.2, 0x4a8f45);
    obstacles.push([t[0], t[1], 0.55]);
  });
  // 가로등
  const L = M('Lanterns');
  [[-20, -26], [-20, -10], [-5, -14], [8, -3], [8, -20], [20, -27], [30, -3.9], [43, -3.9], [-40, 0], [17.5, 3]].forEach((q) => {
    L.cyl(q[0], q[1], 0.09, 1.0, 4.2, 0x41454d, 6); L.box(q[0] - 0.4, q[1] - 0.12, 4.15, q[0] + 0.4, q[1] + 0.12, 4.3, 0x41454d);
    obstacles.push([q[0], q[1], 0.3]);
  });
}

// ---------------------------------------------------------------- 건물 바깥벽 (층마다)
const doorWest = [[-23.4, -21.2]], balconyDoor = [[-25.2, -22.6]];   // 1층 입구, 2층 발코니 문 (서쪽 벽)
for (let k = 0; k < 3; k++) {
  const n = level(k), zl = Z[k], zh = Z[k] + FH;
  wallSeg(n, BX0, BY0, BX1, BY0, T, zl, zh, CW);                                   // 남쪽
  wallSeg(n, BX0, BY1, BX1, BY1, T, zl, zh, CW);                                   // 북쪽
  wallGaps(n, 'y', BX0, BY0, BY1, k === 0 ? doorWest : (k === 1 ? balconyDoor : []), T, zl, zh, CW);   // 서쪽
  wallSeg(n, BX1, BY0, BX1, BY1, T, zl, zh, CW);                                   // 동쪽
  for (let i = 0; i < 6; i++) window_(n, 'x', BY0, 20.5 + i * 4.2, zl + 1.0, -1);
  for (let i = 0; i < 6; i++) window_(n, 'x', BY1, 20.5 + i * 4.2, zl + 1.0, 1);
  [-8, -11, -16].forEach((y) => window_(n, 'y', BX1, y, zl + 1.0, 1));
}
// 바닥판: 1층은 땅, 2·3층은 바닥판 (계단 구멍을 뚫는다), 맨 위는 지붕
const holesF2 = [], holesF3 = [];
[LANE.AN, LANE.BN].forEach((c) => holesF2.push([FL0, c - 0.95, SCX0 + 7.4, c + 0.95]));     // 아래 계단 (2층 바닥 아래로 내려가는 쪽)
[LANE.AS, LANE.BS].forEach((c) => holesF3.push([SCX0 + 3.6, c - 0.95, FL1, c + 0.95]));     // 위 계단
M('F1').flat(BX0, BY0, BX1, BY1, 1.02, FLOORC[0]);
slabHoles('F2', BX0, BY0, BX1, BY1, Z[1], 0.25, FLOORC[1], holesF2);
slabHoles('F3', BX0, BY0, BX1, BY1, Z[2], 0.25, FLOORC[2], holesF3);
M('Roof').box(BX0 - 0.3, BY0 - 0.3, ZR - 0.1, BX1 + 0.3, BY1 + 0.3, ZR + 0.35, 0x59606b);

// ---------------------------------------------------------------- 계단통 (두 칸 A, B. 칸마다 아래 길: 1층 -> 2층, 위 길: 2층 -> 3층)
const SC1 = 0xb7bcc4, SC2 = 0xa2a8b1, SW = 0x6b6f78;
// 아래 길(북쪽 레인): 동쪽 끝(1층)에서 서쪽 끝(2층)으로 오른다. 위 길(남쪽 레인): 서쪽 끝(2층)에서 동쪽 끝(3층)으로 오른다.
[LANE.AN, LANE.BN].forEach((c) => {
  M('F1').stairs(FL0, c - 0.95, FL1, c + 0.95, Z[1], Z[0], false, SC1, SC2);
  layout.ramps.push([FL0, c - 0.95, FL1, c + 0.95, Z[1], Z[0], 'x']);
});
[LANE.AS, LANE.BS].forEach((c) => {
  M('F2').stairs(FL0, c - 0.95, FL1, c + 0.95, Z[1], Z[2], false, SC1, SC2);
  layout.ramps.push([FL0, c - 0.95, FL1, c + 0.95, Z[1], Z[2], 'x']);
});
for (let k = 0; k < 3; k++) {
  const n = level(k), zl = Z[k], zh = Z[k] + FH, W = SW;
  // 서쪽 벽 (x=SCX0): 2층에서 아래 길 꼭대기가 서쪽 홀로 열린다
  wallGaps(n, 'y', SCX0, YB1, YA0, k === 1 ? [gp(LANE.AN, 1.8), gp(LANE.BN, 1.8)] : [], 0.2, zl, zh, W);
  // 동쪽 벽 (x=SCX1): 1층에서 아래 길 입구가 동쪽 방으로 열린다
  wallGaps(n, 'y', SCX1, YB1, YA0, k === 0 ? [gp(LANE.AN, 1.8), gp(LANE.BN, 1.8)] : [], 0.2, zl, zh, W);
  // 길 사이 벽: 2층은 서쪽 끝, 3층은 동쪽 끝이 터져 있어 아래 길에서 위 길로 넘어간다
  [DIV_A, DIV_B].forEach((y) => wallGaps(n, 'x', y, SCX0, SCX1, k === 1 ? [[SCX0, SCX0 + 1.8]] : (k === 2 ? [[SCX1 - 1.8, SCX1]] : []), 0.2, zl, zh, W));
  // A 칸과 B 칸 사이: 3층 동쪽 끝에 문 (A 의 위 길 -> B 쪽)
  wallGaps(n, 'x', YAB, SCX0, SCX1, k === 2 ? [[SCX1 - 1.8, SCX1 - 0.2]] : [], 0.2, zl, zh, W);
}
// ---- 건물 안쪽 가로선 y = YB1 (계단통 아래, 방들 위): 층마다 다르다 (도면의 위쪽 칸들과 아래쪽 방들의 경계)
// 1층: 사무실 북쪽 벽 (x 24.6..43), 동쪽 방으로 가는 문 / 2층: 서쪽 홀과 방들 / 3층: 서쪽 홀 + 계단 문
wallGaps('F1', 'x', YB1, wx(0.2642), BX1, [gp(40, 2.0)], 0.2, Z[0], Z[0] + FH, IW);
wallGaps('F2', 'x', YB1, BX0, BX1, [gp(21, 2.0)], 0.2, Z[1], Z[1] + FH, IW);
wallGaps('F3', 'x', YB1, BX0, BX1, [gp(21.5, 4.0), [SCX1 - 1.8, SCX1 - 0.4]], 0.2, Z[2], Z[2] + FH, IW);

// ---------------------------------------------------------------- 1층: 입구, ㄱ자 복도(홀), 사무실, 계단 2개
{
  const n = 'F1', zl = Z[0], zh = Z[0] + FH, W = IW;
  const ox = wx(0.2642);   // 사무실 서쪽 벽
  wallGaps(n, 'y', ox, BY0, YB1, [gp(wy(0.60), 2.0), gp(wy(0.85), 2.0)], 0.2, zl, zh, W);
  // 입구 칸 (서쪽 벽 아래쪽, 도면의 작은 칸): 위/아래 벽만 있고 동쪽은 홀로 열려 있다
  const vx = wx(0.0776);
  wallSeg(n, BX0, wy(0.628), vx, wy(0.628), 0.2, zl, zh, W); wallSeg(n, BX0, wy(0.890), vx, wy(0.890), 0.2, zl, zh, W);
  M(n).box(BX0 - 1.6, -23.6, zl + 2.7, BX0, -21.0, zl + 2.85, 0x2f5f8f);   // 입구 차양
  // 사무실: 책상 두 줄
  [28, 32, 36, 40].forEach((x) => [-17.3, -21.0, -24.5].forEach((y) => { furn(n, x, y, 1.8, 0.9, 0.75, zl, 0xc2a57a); M(n).box(x - 0.25, y - 0.9, zl, x + 0.25, y - 0.5, zl + 0.5, 0x383d46); }));
  furn(n, 42.3, -15.2, 0.8, 0.8, 1.6, zl, 0x8c929c); furn(n, 25.5, -26.0, 0.8, 0.8, 1.6, zl, 0x8c929c);
  // 홀: 안내 데스크, 소파
  furn(n, 21.0, -9.5, 3.0, 0.8, 1.0, zl, 0x7a5a3c); furn(n, 22.0, -17.0, 0.7, 2.6, 0.5, zl, 0x3f5f8f); furn(n, 21.0, -5.8, 2.0, 0.6, 0.45, zl, 0x3f5f8f);
  // 동쪽 방(계단 앞 홀): 의자
  furn(n, 41.5, -11.0, 1.6, 0.6, 0.45, zl, 0x3f5f8f);
}
// ---------------------------------------------------------------- 2층: 사무실, 방1, 방2, 화장실, 발코니, 회의실, 계단
{
  const n = 'F2', zl = Z[1], zh = Z[1] + FH, W = IW;
  const yS1 = wy(0.5871), yS2 = wy(0.5124), yBal = wy(0.6998), yMeet = wy(0.6559);
  const x1a = wx(0.2251), x1b = wx(0.6197), x2b = SCX1, xBal = wx(0.2263), xMeet = wx(0.6991);
  // 방1
  wallSeg(n, x1a, YB1, x1a, yS1, 0.2, zl, zh, W); wallSeg(n, x1b, YB1, x1b, yS1, 0.2, zl, zh, W); wallGaps(n, 'x', yS1, x1a, x1b, [gp(28.6, 1.8)], 0.2, zl, zh, W);
  // 방2 (작은 방)
  wallSeg(n, x2b, YB1, x2b, yS2, 0.2, zl, zh, W); wallGaps(n, 'x', yS2, x1b, x2b, [gp(35.2, 1.8)], 0.2, zl, zh, W);
  // 화장실
  wallSeg(n, x2b, yS2 > yS1 ? YB1 : YB1, x2b, yS1, 0.2, zl, zh, W); wallGaps(n, 'x', yS1, x2b, BX1, [gp(40.2, 1.8)], 0.2, zl, zh, W);
  // 발코니 (서쪽 바깥 계단에서 들어온다)
  wallGaps(n, 'y', xBal, BY0, yBal, [gp(-23.3, 2.0)], 0.2, zl, zh, W); wallGaps(n, 'x', yBal, BX0, xBal, [gp(20.8, 2.0)], 0.2, zl, zh, W);
  // 회의실
  wallGaps(n, 'y', xMeet, BY0, yMeet, [gp(-22.8, 2.0)], 0.2, zl, zh, W); wallGaps(n, 'x', yMeet, xMeet, BX1, [gp(39.2, 2.0)], 0.2, zl, zh, W);
  // 가구
  furn(n, 30.5, -15.4, 1.8, 0.8, 0.75, zl, 0xc2a57a); furn(n, 26.0, -15.4, 0.7, 1.6, 1.6, zl, 0x8a5a3c);   // 방1
  furn(n, 40.0, -15.4, 0.1, 1.0, 1.8, zl, 0x9fc0d8); M(n).box(38.0, -14.1, zl, 38.6, -14.6, zl + 0.45, 0xf2f2f2);   // 화장실
  [26.5, 30.5, 34.0].forEach((x) => [-20.5, -24.5].forEach((y) => furn(n, x, y, 1.6, 0.9, 0.75, zl, 0xc2a57a)));    // 사무실 책상
  furn(n, 39.2, -23.5, 4.4, 1.6, 0.75, zl, 0x7a5a3c);                                                              // 회의실 탁자
  [37.4, 39.2, 41.0].forEach((x) => { M(n).box(x - 0.22, -21.6, zl, x + 0.22, -21.2, zl + 0.5, 0x383d46); M(n).box(x - 0.22, -25.6, zl, x + 0.22, -25.2, zl + 0.5, 0x383d46); });
  furn(n, 20.0, -24.8, 0.8, 0.8, 0.7, zl, 0x8a5a3c);                                                               // 발코니 탁자
  furn(n, 20.8, -11.0, 1.8, 0.7, 0.45, zl, 0x3f5f8f);                                                              // 서쪽 홀 의자
  // 바깥 계단에서 이어지는 문턱 난간은 아래 '바깥 계단' 에서
}
// ---------------------------------------------------------------- 3층: 회의실1, 방2, 화장실, 창고, 사장실, 계단
{
  const n = 'F3', zl = Z[2], zh = Z[2] + FH, W = IW;
  const yR2 = wy(0.52), yMt0 = wy(0.5988), yMt1 = wy(0.7438), yBoss = wy(0.8023);
  const x2a = wx(0.4436), x2b = 35.0, xWc = 37.4, xMa = wx(0.3203), xMb = wx(0.7141), xSto = wx(0.2598), xBoss = wx(0.5089);
  // 방2
  wallSeg(n, x2a, YB1, x2a, yR2, 0.2, zl, zh, W); wallSeg(n, x2b, YB1, x2b, yR2, 0.2, zl, zh, W); wallGaps(n, 'x', yR2, x2a, x2b, [gp(32.0, 1.8)], 0.2, zl, zh, W);
  // 화장실
  wallGaps(n, 'y', xWc, YB1, yR2, [gp(wy(0.48), 1.8)], 0.2, zl, zh, W); wallSeg(n, xWc, yR2, BX1, yR2, 0.2, zl, zh, W);
  // 회의실1
  wallGaps(n, 'y', xMa, yMt0, yMt1, [gp(-19.4, 1.8)], 0.2, zl, zh, W); wallGaps(n, 'y', xMb, yMt0, yMt1, [gp(-19.4, 1.8)], 0.2, zl, zh, W);
  wallSeg(n, xMa, yMt0, xMb, yMt0, 0.2, zl, zh, W); wallSeg(n, xMa, yMt1, xMb, yMt1, 0.2, zl, zh, W);
  // 창고, 사장실
  wallGaps(n, 'x', yBoss, BX0, xSto, [gp(21.2, 1.8)], 0.2, zl, zh, W); wallSeg(n, xSto, yBoss, xSto, BY0, 0.2, zl, zh, W);
  wallSeg(n, xSto, yBoss, xBoss, yBoss, 0.2, zl, zh, W); wallGaps(n, 'y', xBoss, yBoss, BY0, [gp(-24.4, 1.8)], 0.2, zl, zh, W);
  // 가구
  furn(n, 34.0, -15.5, 1.8, 0.8, 0.75, zl, 0xc2a57a); furn(n, 30.2, -15.5, 1.8, 0.8, 0.75, zl, 0xc2a57a);                 // 방2
  M(n).box(38.0, -14.2, zl, 38.6, -14.8, zl + 0.45, 0xf2f2f2); M(n).box(38.0, -15.6, zl, 38.6, -16.2, zl + 0.45, 0xf2f2f2); // 화장실
  furn(n, 30.9, -19.4, 4.8, 1.2, 0.75, zl, 0x7a5a3c);                                                                      // 회의실1 탁자
  [28.4, 29.9, 31.4, 32.9].forEach((x) => { M(n).box(x - 0.22, -18.3, zl, x + 0.22, -17.95, zl + 0.5, 0x383d46); M(n).box(x - 0.22, -20.85, zl, x + 0.22, -20.5, zl + 0.5, 0x383d46); });
  furn(n, 27.5, -25.4, 2.6, 1.0, 0.8, zl, 0x5a3b26);                                                                       // 사장실 책상
  furn(n, 29.8, -23.2, 0.5, 2.0, 1.9, zl, 0x6b4a30);
  furn(n, 20.0, -25.2, 0.6, 2.0, 2.0, zl, 0x6b4a30); furn(n, 22.8, -25.2, 0.6, 2.0, 2.0, zl, 0x6b4a30);                   // 창고 선반
  furn(n, 21.0, -10.0, 4.0, 1.0, 0.45, zl, 0x2f4f6f);                                                                      // 서쪽 홀 소파
  furn(n, 40.0, -30 + 20, 0.01, 0.01, 0.01, zl, 0xffffff, false);
}
// 건물 안: 내 층보다 위는 숨긴다 (카메라가 천장에 가리지 않게)
layout.hides.push(['F2', BX0, BY0, BX1, BY1, Z[1] - 0.35], ['F3', BX0, BY0, BX1, BY1, Z[2] - 0.35], ['Roof', BX0, BY0, BX1, BY1, ZR - 0.35]);

// ---------------------------------------------------------------- 바깥 계단 (입구 옆, 차고지 쪽): 2층 발코니로 올라간다
{
  const ex0 = 13.6, ex1 = 15.4;
  stair('Site', ex0, ex1, -13.0, -21.0, Z[0], Z[1], Z[0], Z[1], SC1, SC2);
  slab('Site', 11.8, -26.4, BX0 - 0.15, -21.0, Z[1], 0.25, 0x9d8b6a);   // 꼭대기 층계참 (발코니 문까지 이어진다)
  [[11.9, -26.3], [17.9, -26.3], [11.9, -21.2]].forEach((q) => M('Site').box(q[0] - 0.12, q[1] - 0.12, 1.0, q[0] + 0.12, q[1] + 0.12, Z[1] - 0.25, 0x6b6f78));   // 받침 기둥
  rail('Site', 11.8, -26.4, 11.8, -21.0, Z[1]); rail('Site', 11.8, -26.4, BX0 - 0.2, -26.4, Z[1]); rail('Site', 11.8, -21.0, ex0, -21.0, Z[1]); rail('Site', ex1, -21.0, BX0 - 0.15, -21.0, Z[1]);
}

// ---------------------------------------------------------------- 창고 1, 2 (3층 높이, 안으로 들어간다)
function warehouse(name, x0, y0, x1, y1, doors, col) {   // doors: 남쪽 벽의 문 [[a,b],...]
  const top = Z[0] + 10.6, T2 = 0.4;
  wallGaps(name, 'x', y0, x0, x1, doors.south || [], T2, Z[0], top, col);
  wallSeg(name, x0, y1, x1, y1, T2, Z[0], top, col);
  wallGaps(name, 'y', x0, y0, y1, doors.west || [], T2, Z[0], top, col);
  wallGaps(name, 'y', x1, y0, y1, doors.east || [], T2, Z[0], top, col);
  M(name).flat(x0, y0, x1, y1, 1.02, 0x8f949c);
  for (let x = x0 + 3; x < x1 - 2; x += 4) M(name).box(x - 0.8, y1 + 0.2, top - 3.0, x + 0.8, y1 + 0.25, top - 1.6, 0x4f7fa8);   // 높은 창
  (doors.south || []).forEach((d) => { M(name).box(d[0] - 0.2, y0 - 0.25, Z[0], d[0], y0 + 0.25, Z[0] + 4.2, 0xe0b440); M(name).box(d[1], y0 - 0.25, Z[0], d[1] + 0.2, y0 + 0.25, Z[0] + 4.2, 0xe0b440); });
  M(name + 'Roof').box(x0 - 0.4, y0 - 0.4, top, x1 + 0.4, y1 + 0.4, top + 0.45, 0x59606b);
  layout.hides.push([name + 'Roof', x0, y0, x1, y1, 99]);
}
function rack(wh, xa, xb, y, z0) {
  const m = M(wh), nseg = Math.max(1, Math.round((xb - xa) / 1.5));
  for (let i = 0; i <= nseg; i++) { const x = xa + (xb - xa) * i / nseg; m.box(x - 0.05, y - 0.45, z0, x + 0.05, y + 0.45, z0 + 5.4, 0x2f5f8f); }
  const pick = (v) => ((v % 3) + 3) % 3;
  for (let lv = 0; lv < 3; lv++) {
    const zz = z0 + 0.3 + lv * 1.7;
    m.box(xa, y - 0.45, zz, xb, y + 0.45, zz + 0.08, 0xe0792a);
    for (let x = xa + 0.2; x + 0.9 < xb; x += 1.3) m.box(x, y - 0.35, zz + 0.08, x + 0.9, y + 0.35, zz + 0.08 + 0.4 + 0.25 * pick(Math.round(x * 7) + lv), [0xb08a52, 0xc79c63, 0x9a7442][pick(Math.round(x * 3) + lv)]);
  }
  phys(xa, y, xb, y, 0.9, -5, 99);
}
function crate(wh, cx, cy, sz, h, col) { furn(wh, cx, cy, sz, sz, h, 1.0, col, true); }
function forklift(wh, cx, cy) {
  const m = M(wh);
  m.box(cx - 0.55, cy - 0.9, 1.25, cx + 0.55, cy + 0.9, 1.9, 0xe0b440); m.box(cx - 0.5, cy - 0.9, 1.9, cx + 0.5, cy - 0.1, 3.0, 0x2b2e35);
  m.box(cx - 0.5, cy + 0.9, 1.1, cx + 0.5, cy + 2.0, 1.2, 0x555b66); m.box(cx - 0.55, cy + 0.85, 1.1, cx - 0.45, cy + 0.95, 3.2, 0x555b66); m.box(cx + 0.45, cy + 0.85, 1.1, cx + 0.55, cy + 0.95, 3.2, 0x555b66);
  [[-0.6, -0.6], [0.6, -0.6], [-0.6, 0.5], [0.6, 0.5]].forEach((w) => m.box(cx + w[0] - 0.1, cy + w[1] - 0.3, 1.0, cx + w[0] + 0.1, cy + w[1] + 0.3, 1.6, 0x1d1f24));
  phys(cx, cy - 0.5, cx, cy + 1.7, 1.2, -5, 99);
}
// 창고1: 왼쪽 위 (x -44.2..17.1, y 4.3..27.2). 남쪽에 큰 문 둘(차고지 쪽), 동쪽 벽에 창고2 로 가는 문
warehouse('WH1', -44.2, YW1, XD, 27.2, { south: [[-30, -26], [-8, -4]], east: [[14, 18]] }, 0x7d8794);
[9.5, 13.5, 17.5, 21.5, 25.0].forEach((y) => { rack('WH1', -42, -33, y, 1.0); rack('WH1', -23, -11, y, 1.0); rack('WH1', -1, 11.5, y, 1.0); });
crate('WH1', -40.5, 6.4, 1.2, 1.2, 0xb08a52); crate('WH1', -38.9, 6.4, 1.2, 1.6, 0xc79c63); crate('WH1', 14.5, 6.4, 1.4, 1.4, 0x9a7442); crate('WH1', 14.5, 8.0, 1.4, 0.9, 0xb08a52);
forklift('WH1', -18, 6.5); forklift('WH1', 3, 6.5);
// 창고2: 오른쪽 위 (x 17.1..44.2, y -3.2..27.2). 서쪽 벽의 문으로 창고1 과 이어진다
warehouse('WH2', XD, YW2, 44.2, 27.2, { west: [[14, 18]] }, 0x7d8794);
[2.5, 5.5, 8.5, 11.0].forEach((y) => { crate('WH2', 21.0, y, 1.5, 1.4, 0xb08a52); crate('WH2', 24.0, y, 1.5, 1.8, 0xc79c63); crate('WH2', 40.5, y, 1.5, 1.4, 0xc79c63); crate('WH2', 43.0, y, 1.3, 2.0, 0xb08a52); });
[22.0, 25.0].forEach((y) => { rack('WH2', 22, 30, y, 1.0); rack('WH2', 34, 42, y, 1.0); });
furn('WH2', 31.0, 14.5, 9.0, 1.8, 2.2, 1.0, 0x2f5f8f, true); M('WH2').box(26.5, 13.7, 3.2, 35.5, 15.3, 3.5, 0x555b66);   // 컨베이어/기계
forklift('WH2', 27.0, 6.0);
// 창고 입구 근처 표지
M('Site').box(-30.2, YW1 - 0.8, 1.0, -26.2, YW1 - 0.6, 1.1, 0xe0b440); M('Site').box(-8.2, YW1 - 0.8, 1.0, -3.8, YW1 - 0.6, 1.1, 0xe0b440);

// ---------------------------------------------------------------- 파일로 쓴다
function writeGlb(file, nodeMap) {
  const names = Object.keys(nodeMap).filter((k) => nodeMap[k].p.length > 0);
  const bins = [], views = [], accs = [], gnodes = [], meshes = [];
  let off = 0;
  const mm = (a) => { const mn = [1e9, 1e9, 1e9], mx = [-1e9, -1e9, -1e9]; for (let i = 0; i < a.length; i += 3) for (let d = 0; d < 3; d++) { mn[d] = Math.min(mn[d], a[i + d]); mx[d] = Math.max(mx[d], a[i + d]); } return { mn, mx }; };
  names.forEach((name, idx) => {
    const m = nodeMap[name], cnt = m.p.length / 3, base = accs.length;
    [m.p, m.n, m.c].forEach((arr, k) => {
      const buf = Buffer.from(new Float32Array(arr).buffer);
      views.push({ buffer: 0, byteOffset: off, byteLength: buf.length, target: 34962, byteStride: 12 });
      const r = mm(arr);
      accs.push({ bufferView: views.length - 1, componentType: 5126, count: cnt, type: 'VEC3', max: r.mx, min: r.mn });
      bins.push(buf); off += buf.length;
    });
    meshes.push({ name, primitives: [{ mode: 4, attributes: { POSITION: base, NORMAL: base + 1, COLOR_0: base + 2 }, material: 0 }] });
    gnodes.push({ name, mesh: idx });
  });
  const json = { asset: { version: '2.0', generator: 'make_factory_map.js' }, scene: 0, scenes: [{ nodes: gnodes.map((_, i) => i) }], nodes: gnodes, meshes, accessors: accs, bufferViews: views, buffers: [{ byteLength: off }], materials: [{ pbrMetallicRoughness: { metallicFactor: 0, roughnessFactor: 0.95 }, doubleSided: true }] };
  let js = Buffer.from(JSON.stringify(json)); while (js.length % 4) js = Buffer.concat([js, Buffer.from(' ')]);
  const bin = Buffer.concat(bins);
  const head = Buffer.alloc(12); head.write('glTF', 0); head.writeUInt32LE(2, 4); head.writeUInt32LE(12 + 8 + js.length + 8 + bin.length, 8);
  const c1 = Buffer.alloc(8); c1.writeUInt32LE(js.length, 0); c1.write('JSON', 4);
  const c2 = Buffer.alloc(8); c2.writeUInt32LE(bin.length, 0); c2.write('BIN\0', 4);
  fs.writeFileSync(file, Buffer.concat([head, c1, js, c2, bin]));
  return names;
}
const names = writeGlb(path.join(outDir, 'factory_map.glb'), nodes);

// 땅 높이 표: 부지 안은 평평한 1 m (1 m 칸)
{
  const w = SX1 - SX0 + 1, h = SY1 - SY0 + 1;
  let t = '# 땅 높이 표 (엔진 좌표, Z-up, m). tools/make_factory_map.js 가 만든다: 부지(' + SX0 + '..' + SX1 + ', ' + SY0 + '..' + SY1 + ') 안은 평평한 1 m, 밖은 걸을 수 없다.\n';
  t += 'size ' + w + ' ' + h + '\norigin ' + SX0 + ' ' + SY0 + '\ncell 1\n';
  const row = new Array(w).fill('1.00').join(' ');
  for (let r = 0; r < h; r++) t += row + '\n';
  fs.writeFileSync(path.join(outDir, 'ground.txt'), t);
}
let lay = '# 여러 층 (엔진 좌표). tools/make_factory_map.js 가 만든다.\n# slab x0 y0 x1 y1 z / ramp x0 y0 x1 y1 z0 z1 x|y / wall x0 y0 x1 y1 두께 zmin zmax / hide 노드 x0 y0 x1 y1 zlim (그 안에서 발 높이가 zlim 보다 낮으면 숨긴다)\n';
lay += 'spawn 6 -18\n';
layout.slabs.forEach((v) => { lay += 'slab ' + v.map(f2).join(' ') + '\n'; });
layout.ramps.forEach((v) => { lay += 'ramp ' + v.slice(0, 6).map(f2).join(' ') + ' ' + v[6] + '\n'; });
layout.walls.forEach((v) => { lay += 'wall ' + v.map(f2).join(' ') + '\n'; });
layout.hides.forEach((v) => { lay += 'hide ' + v[0] + ' ' + v.slice(1).map(f2).join(' ') + '\n'; });
const vdir = path.join(outDir, 'villagers');
fs.writeFileSync(path.join(vdir, 'layout.txt'), lay);
let ob = '# 주민이 피해 갈 장애물 (엔진 좌표: x y 반지름, Z-up, 단위 m). 나무와 가로등. tools/make_factory_map.js 가 만든다.\n';
obstacles.forEach((o) => { ob += o.map((v) => v.toFixed(3)).join(' ') + '\n'; });
fs.writeFileSync(path.join(vdir, 'obstacles.txt'), ob);
fs.writeFileSync(path.join(vdir, 'portals.txt'), '# 이 지도에는 B 키로 들어가는 방이 없다 (건물은 걸어서 들어간다). 방 형식은 README 참고.\n');
// 주민 순찰: 건물 앞 마당과 차고지 앞길 (벽을 지나지 않는 직선 길)
const wps = {
  villager_1: [[-43, -14], [-12, -14], [-12, -25], [-43, -25]],
  villager_2: [[4, -26], [4, -3], [9, -3], [9, -26]],
  villager_3: [[-43, -13], [-10, -13]],
  villager_4: [[-6, -12], [-6, 2], [8, 2], [8, -12]]
};
let w = '# 주민 이동 지점 (엔진 좌표: x y z, Z-up, 단위 m).\n# [이름] 아래 "speed" 는 m/s, 이어지는 줄이 지점이다. 위에서 아래로 차례로 직선으로 가고, 끝에 닿으면 처음으로 돌아간다.\n# 주민 모델(villager_N.glb)은 첫 지점에 놓고 시작한다. tools/make_factory_map.js 가 만든다.\n';
Object.keys(wps).forEach((k) => { w += '\n[' + k + ']\nspeed 1.2\n'; wps[k].forEach((q) => { w += q[0].toFixed(3) + ' ' + q[1].toFixed(3) + ' 1.000\n'; }); });
fs.writeFileSync(path.join(vdir, 'waypoints.txt'), w);
console.log('만든 것: factory_map.glb(' + names.join(',') + '), ground.txt, layout.txt(' + layout.walls.length + ' 벽, ' + layout.slabs.length + ' 바닥, ' + layout.ramps.length + ' 계단), obstacles.txt, waypoints.txt, portals.txt');
