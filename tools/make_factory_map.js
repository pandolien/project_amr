// 공장/사무동 지도를 만든다. 기존 마을 지도를 대신한다 (마을은 git 기록에 남아 있다).
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

// ---------------------------------------------------------------- 부지
const SX0 = -32, SX1 = 32, SY0 = -24, SY1 = 24;   // 걸을 수 있는 땅
const GRASS = 0x6d9a4c, ASPHALT = 0x4a4d55, CONCRETE = 0xb4b8bd, LINE = 0xe8e8d8, FENCE = 0x7d838c;
{
  const s = M('Site');
  s.flat(SX0 - 6, SY0 - 12, SX1 + 6, SY1 + 6, 0.97, GRASS);   // 풀밭 (부지 밖까지 조금 더)
  s.flat(-30, -22, -6, -6, 1.0, ASPHALT);                       // 차고지
  s.flat(-3, -24, 3, -4, 1.0, CONCRETE);                        // 정문에서 들어오는 길
  s.flat(-6, -9, 28, -5, 1.0, CONCRETE);                        // 건물 앞 길
  s.flat(SX0 - 6, SY0 - 12, SX1 + 6, SY0, 0.98, ASPHALT);       // 정문 밖 도로
  // 차고지 주차 칸 선
  for (let i = 0; i <= 8; i++) s.flat(-29 + i * 3, -21.5, -28.9 + i * 3, -14, 1.01, LINE);
  s.flat(-29, -14, -5.1, -13.9, 1.01, LINE);
  // 울타리: 둘레, 정문(x -3..3, y -24)만 열려 있다
  const post = (x, y) => { s.box(x - 0.08, y - 0.08, 1.0, x + 0.08, y + 0.08, 2.4, FENCE); };
  const fenceLine = (x0, y0, x1, y1) => {
    const len = Math.hypot(x1 - x0, y1 - y0), n = Math.max(1, Math.round(len / 3));
    for (let i = 0; i <= n; i++) post(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n);
    if (Math.abs(y1 - y0) < 1e-9) { s.box(Math.min(x0, x1), y0 - 0.03, 1.5, Math.max(x0, x1), y0 + 0.03, 1.6, FENCE); s.box(Math.min(x0, x1), y0 - 0.03, 2.1, Math.max(x0, x1), y0 + 0.03, 2.2, FENCE); }
    else { s.box(x0 - 0.03, Math.min(y0, y1), 1.5, x0 + 0.03, Math.max(y0, y1), 1.6, FENCE); s.box(x0 - 0.03, Math.min(y0, y1), 2.1, x0 + 0.03, Math.max(y0, y1), 2.2, FENCE); }
  };
  fenceLine(SX0, SY0, -3, SY0); fenceLine(3, SY0, SX1, SY0); fenceLine(SX0, SY1, SX1, SY1); fenceLine(SX0, SY0, SX0, SY1); fenceLine(SX1, SY0, SX1, SY1);
  // 정문 기둥과 간판 (입구)
  s.box(-3.4, -24.3, 1.0, -2.8, -23.7, 4.0, 0x8c929c); s.box(2.8, -24.3, 1.0, 3.4, -23.7, 4.0, 0x8c929c);
  s.box(-3.4, -24.25, 3.5, 3.4, -23.75, 4.2, 0x2f5f8f);
  // 경비실 (정문 오른쪽)
  s.box(4.5, -23.2, 1.0, 7.5, -20.8, 3.4, 0xd9d3c2); s.box(4.3, -23.4, 3.4, 7.7, -20.6, 3.6, 0x555b66);
  s.box(4.5, -23.25, 1.6, 7.5, -23.15, 2.6, 0x4f7fa8);
  phys(4.5, -23.2, 7.5, -23.2, 0.1, -5, 99); phys(4.5, -20.8, 7.5, -20.8, 0.1, -5, 99); phys(4.5, -23.2, 4.5, -20.8, 0.1, -5, 99); phys(7.5, -23.2, 7.5, -20.8, 0.1, -5, 99);
  // 차고지 차량: 트럭 3대, 승용차 3대
  const truck = (cx, cy, col) => {
    s.box(cx - 1.0, cy - 2.4, 1.35, cx + 1.0, cy + 1.2, 3.1, col); s.box(cx - 1.0, cy + 1.2, 1.35, cx + 1.0, cy + 2.4, 2.7, 0xe6e9ee); s.box(cx - 0.95, cy + 2.1, 2.0, cx + 0.95, cy + 2.4, 2.6, 0x4f7fa8);
    [[-1.05, -1.6], [1.05, -1.6], [-1.05, 1.6], [1.05, 1.6]].forEach((w) => s.box(cx + w[0] - 0.12, cy + w[1] - 0.45, 1.0, cx + w[0] + 0.12, cy + w[1] + 0.45, 1.9, 0x1d1f24));
    phys(cx, cy - 2.4 + 1.0, cx, cy + 2.4 - 1.0, 2.0, -5, 99);
  };
  const car = (cx, cy, col) => {
    s.box(cx - 0.9, cy - 2.1, 1.25, cx + 0.9, cy + 2.1, 2.0, col); s.box(cx - 0.8, cy - 1.0, 2.0, cx + 0.8, cy + 1.0, 2.7, 0x9fc6e3);
    [[-0.9, -1.3], [0.9, -1.3], [-0.9, 1.3], [0.9, 1.3]].forEach((w) => s.box(cx + w[0] - 0.1, cy + w[1] - 0.35, 1.0, cx + w[0] + 0.1, cy + w[1] + 0.35, 1.7, 0x1d1f24));
    phys(cx, cy - 2.1 + 0.9, cx, cy + 2.1 - 0.9, 1.8, -5, 99);
  };
  truck(-27.5, -18, 0xd9822b); truck(-21.5, -18, 0x3f78b5); truck(-15.5, -18, 0xd9822b);
  car(-12, -18, 0xc23b3b); car(-9, -18, 0xe8e8e8); car(-27.5, -9.5, 0x2f6f4a);
  // 주차 안내 콘
  [[-24, -12], [-18, -12], [-12, -12]].forEach((c) => { s.cone(c[0], c[1], 0.3, 1.0, 1.7, 0xff7a1a, 8); });
  // 나무
  [[-31, 22], [31, 22], [-31, -3], [31, -3], [30, -12], [-5, 22], [14, 21], [24, 19]].forEach((t) => {
    s.cyl(t[0], t[1], 0.25, 1.0, 2.6, 0x7a5232, 8); s.ball(t[0], t[1], 3.6, 1.7, 0x3f7f3c); s.ball(t[0] + 0.5, t[1] + 0.3, 4.5, 1.2, 0x4a8f45);
    obstacles.push([t[0], t[1], 0.55]);
  });
  // 가로등
  const L = M('Lanterns');
  [[-8, -22], [8, -22], [-4, -12], [0.5, 0], [-12, -3], [-31, -3], [30, -4], [30, 12], [-14, 22], [14, -3.5]].forEach((q) => {
    L.cyl(q[0], q[1], 0.09, 1.0, 4.2, 0x41454d, 6); L.box(q[0] - 0.4, q[1] - 0.12, 4.15, q[0] + 0.4, q[1] + 0.12, 4.3, 0x41454d);
    obstacles.push([q[0], q[1], 0.3]);
  });
}

// ---------------------------------------------------------------- 사무동 (3층)
const BX0 = 4, BX1 = 26, BY0 = -2, BY1 = 16;   // 건물 바깥 선
const CW = 0x9aa3ad, IW = 0xdfe3e8, FLOORC = [0xc9ccd2, 0xb8a98f, 0xb9bec7], SLABC = 0x7f858e;
const T = 0.3;   // 바깥벽 두께
const level = (k) => ['F1', 'F2', 'F3'][k];
const doorX = 8.0;   // 정문(1층 입구) x
// ---- 바깥벽: 층마다
for (let k = 0; k < 3; k++) {
  const n = level(k), zl = Z[k], zh = Z[k] + FH;
  const southGaps = k === 0 ? [[doorX - 1.0, doorX + 1.0]] : (k === 1 ? [[14.2, 15.8]] : []);
  wallGaps(n, 'x', BY0, BX0, BX1, southGaps, T, zl, zh, CW);
  wallSeg(n, BX0, BY1, BX1, BY1, T, zl, zh, CW);
  wallSeg(n, BX0, BY0, BX0, BY1, T, zl, zh, CW);
  wallSeg(n, BX1, BY0, BX1, BY1, T, zl, zh, CW);
  // 창문
  [10.5, 13.0, 18.0, 21.0].forEach((x) => { if (!(k === 0 && Math.abs(x - doorX) < 1.5) && !(k === 1 && Math.abs(x - 15.1) < 1.5)) window_(n, 'x', BY0, x, zl + 1.0, -1); });
  [8, 12, 16, 20, 24].forEach((x) => window_(n, 'x', BY1, x, zl + 1.0, 1));
  [2, 6, 10, 14].forEach((y) => { window_(n, 'y', BX0, y, zl + 1.0, -1); window_(n, 'y', BX1, y, zl + 1.0, 1); });
}
// ---- 바닥/천장: 1층은 땅, 2·3층은 바닥판(계단 구멍은 뚫는다), 맨 위는 지붕
const holeS1 = [4.4, 3.0, 6.2, 8.5], holeS2 = [23.8, 6.5, 25.6, 12.0], holeS3 = [10.6, 4.5, 12.4, 10.0];
M('F1').flat(BX0, BY0, BX1, BY1, 1.02, FLOORC[0]);
slabHoles('F2', BX0, BY0, BX1, BY1, Z[1], 0.25, FLOORC[1], [holeS1, holeS2]);
slabHoles('F3', BX0, BY0, BX1, BY1, Z[2], 0.25, FLOORC[2], [holeS3]);
M('Roof').box(BX0 - 0.3, BY0 - 0.3, ZR - 0.1, BX1 + 0.3, BY1 + 0.3, ZR + 0.35, 0x59606b);

// ---- 계단 (너비 1.8, 길이 8 m, 한 층 3.2 m 오른다)
const SC1 = 0xb7bcc4, SC2 = 0xa2a8b1;
stair('F1', 4.4, 6.2, 0.5, 8.5, Z[0], Z[1], Z[0], Z[1], SC1, SC2);       // 계단1: 1층 -> 2층 (서쪽, 남쪽에서 북쪽으로)
stair('F1', 23.8, 25.6, 14.5, 6.5, Z[0], Z[1], Z[0], Z[1], SC1, SC2);    // 계단2: 1층 -> 2층 (동쪽, 북쪽에서 남쪽으로)
stair('F2', 10.6, 12.4, 12.5, 4.5, Z[1], Z[2], Z[1], Z[2], SC1, SC2);    // 계단3: 2층 -> 3층
// 건물 밖 계단: 입구 옆에서 2층 발코니로
stair('Site', 1.6, 3.4, -12.0, -4.0, Z[0], Z[1], Z[0], Z[1], SC1, SC2);
// 발코니 (2층 높이, 건물 앞)
slab('F2', 1.6, -4.0, 18.6, BY0, Z[1], 0.25, 0x9d8b6a);
rail('F2', 3.4, -4.0, 18.6, -4.0, Z[1]); rail('F2', 18.6, -4.0, 18.6, BY0, Z[1]); rail('F2', 1.6, -4.0, 1.6, BY0, Z[1]);
// 입구 차양
M('F1').box(doorX - 1.6, BY0 - 1.6, Z[0] + 2.7, doorX + 1.6, BY0, Z[0] + 2.85, 0x2f5f8f);
M('F1').box(doorX - 1.5, BY0 - 1.5, Z[0] + 2.85, doorX - 1.4, BY0 - 1.4, Z[0] + 3.1, 0x2f5f8f);

// ---- 1층: 입구/로비, 사무실, 계단 x2
{
  const n = 'F1', zl = Z[0], zh = Z[0] + FH, W = IW;
  wallGaps(n, 'x', 3.5, 6.2, 23.8, [[14.2, 15.8]], 0.2, zl, zh, W);       // 로비 | 사무실
  wallGaps(n, 'x', 12.6, 6.3, 23.8, [[14.2, 15.8]], 0.2, zl, zh, W);      // 사무실 | 복도
  wallSeg(n, 6.3, 8.5, 6.3, 12.6, 0.2, zl, zh, W);                        // 서쪽 구석
  // 로비: 안내 데스크와 소파
  furn(n, 14.0, -0.2, 3.0, 0.8, 1.0, zl, 0x7a5a3c); furn(n, 19.5, -1.0, 3.0, 0.8, 0.5, zl, 0x3f5f8f); furn(n, 11.0, 1.8, 1.6, 0.6, 0.5, zl, 0x3f5f8f);
  // 사무실: 책상 두 줄
  [8.5, 11.5, 18.5, 21.5].forEach((x) => [6.2, 9.4].forEach((y) => { furn(n, x, y, 1.8, 0.9, 0.75, zl, 0xc2a57a); M(n).box(x - 0.25, y - 0.9, zl, x + 0.25, y - 0.5, zl + 0.5, 0x383d46); }));
  furn(n, 23.0, 4.4, 0.8, 0.8, 1.6, zl, 0x8c929c); furn(n, 6.9, 11.8, 0.8, 0.8, 1.6, zl, 0x8c929c);
  // 복도: 화분과 의자
  furn(n, 19.0, 14.6, 0.9, 0.9, 0.9, zl, 0x4f7f3f); furn(n, 9.0, 14.8, 2.0, 0.6, 0.45, zl, 0x3f5f8f);
}
// ---- 2층: 사무실, 방1, 방2, 화장실, 발코니, 회의실, 계단
{
  const n = 'F2', zl = Z[1], zh = Z[1] + FH, W = IW;
  wallGaps(n, 'x', 12.6, 6.3, 23.8, [[7.4, 9.0], [10.6, 12.4], [14.2, 15.8], [20.2, 21.8]], 0.2, zl, zh, W);   // 복도 쪽 벽
  wallSeg(n, 6.3, 7.9, 6.3, 12.6, 0.2, zl, zh, W);                        // 계단1 출구 옆
  wallSeg(n, 6.3, 3.0, 6.3, 7.9, 0.1, zl, zl + 1.1, 0x5a5f6b);              // 계단1 구멍 난간 (동쪽)
  wallSeg(n, 4.3, 3.0, 6.3, 3.0, 0.1, zl, zl + 1.1, 0x5a5f6b);              // 계단1 구멍 난간 (남쪽)
  wallSeg(n, 6.3, 7.9, 10.5, 7.9, 0.2, zl, zh, W);                        // 방1 | 화장실
  wallGaps(n, 'y', 10.5, -1.7, 4.5, [[0.8, 2.4]], 0.2, zl, zh, W);       // 방1 | 현관홀
  wallGaps(n, 'y', 12.5, -1.7, 4.5, [[0.8, 2.4]], 0.2, zl, zh, W);       // 현관홀 | 사무실
  wallSeg(n, 10.6, 4.5, 12.4, 4.5, 0.2, zl, zh, W);                       // (계단3 위 끝 막이가 이미 있다)
  wallGaps(n, 'y', 18.9, -1.7, 12.6, [[0.8, 2.4], [7.8, 9.4]], 0.2, zl, zh, W);   // 사무실 | 회의실, 방2
  wallGaps(n, 'x', 6.3, 19.0, 23.7, [[20.2, 21.8]], 0.2, zl, zh, W);      // 회의실 | 방2
  wallGaps(n, 'y', 23.7, -1.7, 12.6, [[1.8, 3.4]], 0.2, zl, zh, W);       // 회의실/방2 | 계단2 쪽
  wallSeg(n, 23.8, 12.0, 25.6, 12.0, 0.1, zl, zl + 1.1, 0x5a5f6b);          // 계단2 구멍 난간 (북쪽)
  // 사무실(2층): 책상, 칸막이
  [14.0, 17.0].forEach((x) => [2.0, 5.5, 9.0].forEach((y) => { furn(n, x, y, 1.6, 0.9, 0.75, zl, 0xc2a57a); }));
  // 방1: 침대와 서랍, 방2: 소파와 탁자
  furn(n, 8.8, 0.2, 1.2, 2.0, 0.5, zl, 0x6f8fb0); furn(n, 7.6, 5.8, 1.5, 0.6, 1.4, zl, 0x8a5a3c);
  furn(n, 21.0, 11.0, 2.4, 0.9, 0.5, zl, 0x8b4f3f); furn(n, 21.0, 8.8, 1.2, 0.8, 0.45, zl, 0x8a5a3c);
  // 회의실: 긴 탁자와 의자
  furn(n, 21.2, 1.5, 3.4, 1.4, 0.75, zl, 0x7a5a3c);
  [19.9, 21.2, 22.5].forEach((x) => { M(n).box(x - 0.22, 0.1, zl, x + 0.22, 0.5, zl + 0.5, 0x383d46); M(n).box(x - 0.22, 2.5, zl, x + 0.22, 2.9, zl + 0.5, 0x383d46); });
  // 화장실: 칸막이와 세면대
  [9.2].forEach((x) => { furn(n, x, 9.4, 0.1, 2.2, 1.8, zl, 0x9fc0d8); M(n).box(7.0, 11.8, zl, 7.6, 12.3, zl + 0.45, 0xf2f2f2); M(n).box(8.0, 11.8, zl, 8.6, 12.3, zl + 0.45, 0xf2f2f2); });
  furn(n, 6.9, 9.6, 0.5, 1.6, 0.85, zl, 0xf2f2f2);
  // 복도
  furn(n, 20.0, 14.8, 1.6, 0.7, 0.5, zl, 0x3f5f8f);
  // 발코니: 의자와 탁자
  furn(n, 10.0, -3.0, 0.8, 0.8, 0.7, zl, 0x8a5a3c); furn(n, 12.5, -3.0, 0.8, 0.8, 0.7, zl, 0x8a5a3c);
}
// ---- 3층: 회의실1, 방2, 화장실, 창고, 사장실, 계단
{
  const n = 'F3', zl = Z[2], zh = Z[2] + FH, W = IW;
  wallGaps(n, 'y', 10.5, -1.7, 15.7, [[1.4, 3.0], [10.4, 12.0], [12.6, 14.2]], 0.2, zl, zh, W);   // 사장실/화장실/창고 | 중앙 통로
  wallGaps(n, 'y', 12.5, -1.7, 15.7, [[1.4, 3.0], [12.6, 14.2]], 0.2, zl, zh, W);                 // 중앙 통로 | 회의실1/방2
  wallSeg(n, 10.6, 10.0, 12.4, 10.0, 0.1, zl, zl + 1.1, 0x5a5f6b);                               // 계단3 구멍 난간 (북쪽)
  wallSeg(n, 4.3, 6.9, 10.4, 6.9, 0.2, zl, zh, W);                                              // 사장실 | 화장실
  wallGaps(n, 'x', 6.9, 12.6, 25.7, [[17.8, 19.4]], 0.2, zl, zh, W);                              // 회의실1 | 방2
  wallSeg(n, 4.3, 12.1, 10.4, 12.1, 0.2, zl, zh, W);                                            // 화장실 | 창고
  // 사장실: 큰 책상, 소파, 책장
  furn('F3', 7.0, 4.6, 2.6, 1.1, 0.8, zl, 0x5a3b26); M('F3').box(6.5, 3.4, zl, 7.5, 3.9, zl + 0.55, 0x2b2e35);
  furn('F3', 5.2, 0.4, 0.8, 2.2, 0.5, zl, 0x2f4f6f); furn('F3', 9.8, -0.2, 0.5, 2.0, 2.0, zl, 0x6b4a30);
  // 회의실1: 큰 탁자와 의자, 화면
  furn('F3', 19.0, 2.2, 5.2, 1.6, 0.75, zl, 0x7a5a3c);
  [16.8, 18.2, 19.6, 21.0].forEach((x) => { M('F3').box(x - 0.22, 0.7, zl, x + 0.22, 1.1, zl + 0.5, 0x383d46); M('F3').box(x - 0.22, 3.3, zl, x + 0.22, 3.7, zl + 0.5, 0x383d46); });
  M('F3').box(23.5, 1.2, zl + 0.9, 25.6, 3.4, zl + 2.2, 0x1d2430);
  // 방2: 작업 책상
  furn('F3', 15.0, 9.4, 1.8, 0.9, 0.75, zl, 0xc2a57a); furn('F3', 19.0, 9.4, 1.8, 0.9, 0.75, zl, 0xc2a57a); furn('F3', 23.0, 9.4, 1.8, 0.9, 0.75, zl, 0xc2a57a);
  furn('F3', 15.0, 13.4, 1.8, 0.9, 0.75, zl, 0xc2a57a); furn('F3', 19.0, 13.4, 1.8, 0.9, 0.75, zl, 0xc2a57a);
  // 화장실
  furn('F3', 9.6, 9.0, 0.1, 2.6, 1.8, zl, 0x9fc0d8); M('F3').box(5.0, 10.4, zl, 5.6, 11.0, zl + 0.45, 0xf2f2f2); M('F3').box(6.2, 10.4, zl, 6.8, 11.0, zl + 0.45, 0xf2f2f2);
  // 창고: 선반과 상자
  furn('F3', 5.0, 14.0, 0.6, 3.0, 2.0, zl, 0x6b4a30); furn('F3', 6.6, 14.0, 0.6, 3.0, 2.0, zl, 0x6b4a30); furn('F3', 9.4, 15.0, 1.0, 1.0, 0.9, zl, 0xb08a52);
}
// 층 가림: 건물 안에 있을 때 내 층보다 위는 숨긴다 (카메라가 천장에 가리지 않게)
layout.hides.push(['F2', BX0, BY0, BX1, BY1, Z[1] - 0.35], ['F3', BX0, BY0, BX1, BY1, Z[2] - 0.35], ['Roof', BX0, BY0, BX1, BY1, ZR - 0.35]);

// ---------------------------------------------------------------- 창고 1, 2 (3층 높이, 안으로 들어간다)
function warehouse(name, x0, y0, x1, y1, doorA, doorB, roofCol) {
  const wh = name, top = Z[0] + 10.6;   // 3층(10.6 m) 높이
  const col = 0x7d8794, T2 = 0.4;
  wallGaps(wh, 'x', y0, x0, x1, [[doorA, doorB]], T2, Z[0], top, col);     // 남쪽: 문 (너비 3 m)
  wallSeg(wh, x0, y1, x1, y1, T2, Z[0], top, col);
  wallSeg(wh, x0, y0, x0, y1, T2, Z[0], top, col);
  wallSeg(wh, x1, y0, x1, y1, T2, Z[0], top, col);
  M(wh).flat(x0, y0, x1, y1, 1.02, 0x8f949c);
  // 높은 창 (눈에 보이기만 한다)
  for (let x = x0 + 2.5; x < x1 - 1.5; x += 3.5) { M(wh).box(x - 0.8, y1 + 0.2, top - 3.0, x + 0.8, y1 + 0.25, top - 1.6, 0x4f7fa8); }
  // 노란 문틀 표시
  M(wh).box(doorA - 0.2, y0 - 0.25, Z[0], doorA, y0 + 0.25, Z[0] + 3.6, 0xe0b440); M(wh).box(doorB, y0 - 0.25, Z[0], doorB + 0.2, y0 + 0.25, Z[0] + 3.6, 0xe0b440);
  // 지붕은 안에 들어가면 숨긴다
  M(wh + 'Roof').box(x0 - 0.4, y0 - 0.4, top, x1 + 0.4, y1 + 0.4, top + 0.45, roofCol);
  layout.hides.push([wh + 'Roof', x0, y0, x1, y1, 99]);
}
function rack(wh, xa, xb, y, z0) {   // 선반: 기둥 + 3단 + 상자. 막는 것은 선분
  const m = M(wh);
  for (let i = 0; i <= Math.round((xb - xa) / 1.5); i++) { const x = xa + (xb - xa) * i / Math.max(1, Math.round((xb - xa) / 1.5)); m.box(x - 0.05, y - 0.45, z0, x + 0.05, y + 0.45, z0 + 5.4, 0x2f5f8f); }
  for (let lv = 0; lv < 3; lv++) {
    const zz = z0 + 0.3 + lv * 1.7;
    m.box(xa, y - 0.45, zz, xb, y + 0.45, zz + 0.08, 0xe0792a);
    for (let x = xa + 0.2; x + 0.9 < xb; x += 1.3) m.box(x, y - 0.35, zz + 0.08, x + 0.9, y + 0.35, zz + 0.08 + 0.4 + 0.5 * ((((Math.round(x * 7) + lv) % 3) + 3) % 3) / 2, [0xb08a52, 0xc79c63, 0x9a7442][(((Math.round(x * 3) + lv) % 3) + 3) % 3]);
  }
  phys(xa, y, xb, y, 0.9, -5, 99);
}
function crate(wh, cx, cy, s, h, col) { furn(wh, cx, cy, s, s, h, 1.0, col, true); }
function forklift(wh, cx, cy) {   // 지게차
  const m = M(wh);
  m.box(cx - 0.55, cy - 0.9, 1.25, cx + 0.55, cy + 0.9, 1.9, 0xe0b440); m.box(cx - 0.5, cy - 0.9, 1.9, cx + 0.5, cy - 0.1, 3.0, 0x2b2e35);
  m.box(cx - 0.5, cy + 0.9, 1.1, cx + 0.5, cy + 2.0, 1.2, 0x555b66); m.box(cx - 0.55, cy + 0.85, 1.1, cx - 0.45, cy + 0.95, 3.2, 0x555b66); m.box(cx + 0.45, cy + 0.85, 1.1, cx + 0.55, cy + 0.95, 3.2, 0x555b66);
  [[-0.6, -0.6], [0.6, -0.6], [-0.6, 0.5], [0.6, 0.5]].forEach((w) => m.box(cx + w[0] - 0.1, cy + w[1] - 0.3, 1.0, cx + w[0] + 0.1, cy + w[1] + 0.3, 1.6, 0x1d1f24));
  phys(cx, cy - 0.5, cx, cy + 1.7, 1.2, -5, 99);
}
warehouse('WH1', -29, 4, -15, 20, -23.5, -20.5, 0x59606b);
for (const y of [9.0, 12.5, 16.0]) { rack('WH1', -27.6, -24.2, y, 1.0); rack('WH1', -19.8, -16.4, y, 1.0); }
crate('WH1', -26.8, 6.0, 1.2, 1.2, 0xb08a52); crate('WH1', -17.6, 6.0, 1.2, 1.2, 0xc79c63); crate('WH1', -17.6, 7.4, 1.2, 0.8, 0x9a7442);
forklift('WH1', -21.6, 17.4);
warehouse('WH2', -12, 4, 0, 20, -7.5, -4.5, 0x59606b);
for (const y of [8.5, 11.5]) { crate('WH2', -10.2, y, 1.4, 1.4, 0xb08a52); crate('WH2', -10.2, y + 1.5, 1.4, 1.0, 0xc79c63); crate('WH2', -1.8, y, 1.4, 1.4, 0xc79c63); crate('WH2', -1.8, y + 1.5, 1.4, 2.0, 0xb08a52); }
rack('WH2', -10.6, -7.6, 15.5, 1.0); rack('WH2', -4.4, -1.4, 15.5, 1.0);
furn('WH2', -6.0, 18.4, 7.0, 1.8, 2.2, 1.0, 0x2f5f8f, true);   // 컨베이어/기계
M('WH2').box(-9.0, 17.6, 3.2, -3.0, 19.2, 3.5, 0x555b66);
forklift('WH2', -6.0, 6.8);

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
lay += 'spawn 0 -20\n';
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
  villager_1: [[-5, -20], [-5, -8], [2, -8], [2, -20]],
  villager_2: [[8, -18], [26, -18], [26, -7], [8, -7]],
  villager_3: [[-20, -11], [-9, -11], [-9, 1], [-20, 1]],
  villager_4: [[12, -20], [12, -10], [22, -10], [22, -20]]
};
let w = '# 주민 이동 지점 (엔진 좌표: x y z, Z-up, 단위 m).\n# [이름] 아래 "speed" 는 m/s, 이어지는 줄이 지점이다. 위에서 아래로 차례로 직선으로 가고, 끝에 닿으면 처음으로 돌아간다.\n# 주민 모델(villager_N.glb)은 첫 지점에 놓고 시작한다. tools/make_factory_map.js 가 만든다.\n';
Object.keys(wps).forEach((k) => { w += '\n[' + k + ']\nspeed 1.2\n'; wps[k].forEach((q) => { w += q[0].toFixed(3) + ' ' + q[1].toFixed(3) + ' 1.000\n'; }); });
fs.writeFileSync(path.join(vdir, 'waypoints.txt'), w);
console.log('만든 것: factory_map.glb(' + names.join(',') + '), ground.txt, layout.txt(' + layout.walls.length + ' 벽, ' + layout.slabs.length + ' 바닥, ' + layout.ramps.length + ' 계단), obstacles.txt, waypoints.txt, portals.txt');
