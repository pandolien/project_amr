// 접속자 캐릭터용 모델을 만든다. (웹 화면의 web/public/client.js 와 같은 모양/크기/색이어야 한다)
//   villager_N_p.glb  주민 모델에서 몸에 붙은 오른팔(삼각형 76~147)을 뺀 것. 무기를 쥔 팔은 따로 그린다.
//   arm.glb           어깨(원점)에서 아래로 0.36 m 뻗은 소매 + 어깨 관절 + 주먹
//   sword.glb / spear.glb / axe.glb   원점이 쥐는 자리인 무기 (검/도끼는 +Y 로 세워 들고, 창은 +Z 로 겨눈다)
// 사용법: node tools/make_fighter_models.js [Data/Object/villagers 폴더]
'use strict';
const fs = require('fs');
const path = require('path');
const dir = process.argv[2] || path.join(__dirname, '..', 'Data', 'Object', 'villagers');
const ARM_FROM = 76, ARM_TO = 147, TRIS = 544;

function readGlb(file) {
  const b = fs.readFileSync(file);
  const jl = b.readUInt32LE(12);
  const json = JSON.parse(b.slice(20, 20 + jl).toString());
  const bin = b.slice(20 + jl + 8);
  return { json, bin };
}
function writeGlb(file, name, pos, nor, col) {   // 삼각형을 풀어 놓은(인덱스 없는) 한 덩어리 메시. 주민 GLB 와 같은 구조
  const n = pos.length / 3;
  const mm = (a) => { const mn = [1e9, 1e9, 1e9], mx = [-1e9, -1e9, -1e9]; for (let i = 0; i < a.length; i += 3) for (let d = 0; d < 3; d++) { mn[d] = Math.min(mn[d], a[i + d]); mx[d] = Math.max(mx[d], a[i + d]); } return { mn, mx }; };
  const bytes = n * 12;
  const json = {
    asset: { version: '2.0', generator: 'make_fighter_models.js' },
    scenes: [{ name, nodes: [0] }], scene: 0, nodes: [{ name, mesh: 0 }],
    bufferViews: [0, 1, 2].map((i) => ({ buffer: 0, byteOffset: i * bytes, byteLength: bytes, target: 34962, byteStride: 12 })),
    buffers: [{ byteLength: bytes * 3 }],
    accessors: [pos, nor, col].map((a, i) => { const m = mm(a); return { bufferView: i, componentType: 5126, count: n, max: m.mx, min: m.mn, type: 'VEC3' }; }),
    materials: [{ pbrMetallicRoughness: { metallicFactor: 0, roughnessFactor: 1 } }],
    meshes: [{ primitives: [{ mode: 4, attributes: { POSITION: 0, NORMAL: 1, COLOR_0: 2 }, material: 0 }] }]
  };
  let js = Buffer.from(JSON.stringify(json));
  while (js.length % 4) js = Buffer.concat([js, Buffer.from(' ')]);
  const bin = Buffer.concat([pos, nor, col].map((a) => Buffer.from(new Float32Array(a).buffer)));
  const head = Buffer.alloc(12); head.write('glTF', 0); head.writeUInt32LE(2, 4); head.writeUInt32LE(12 + 8 + js.length + 8 + bin.length, 8);
  const c1 = Buffer.alloc(8); c1.writeUInt32LE(js.length, 0); c1.write('JSON', 4);
  const c2 = Buffer.alloc(8); c2.writeUInt32LE(bin.length, 0); c2.write('BIN\0', 4);
  fs.writeFileSync(file, Buffer.concat([head, c1, js, c2, bin]));
}

// 1) 오른팔을 뺀 주민
for (let k = 1; k <= 4; k++) {
  const src = path.join(dir, 'villager_' + k + '.glb');
  if (!fs.existsSync(src)) continue;
  const { json, bin } = readGlb(src);
  const p = json.meshes[0].primitives[0];
  const acc = (name) => { const a = json.accessors[p.attributes[name]], v = json.bufferViews[a.bufferView]; const o = bin.byteOffset + (v.byteOffset || 0) + (a.byteOffset || 0); return new Float32Array(bin.buffer.slice(o, o + a.count * 12)); };
  const pos = acc('POSITION'), nor = acc('NORMAL'), col = acc('COLOR_0');
  if (pos.length / 9 !== TRIS) { console.error(src + ': 삼각형이 ' + pos.length / 9 + '개라 건너뜁니다 (예상 ' + TRIS + ')'); continue; }
  const keep = (a) => { const out = []; for (let t = 0; t < TRIS; t++) if (t < ARM_FROM || t > ARM_TO) for (let i = 0; i < 9; i++) out.push(a[t * 9 + i]); return out; };
  writeGlb(path.join(dir, 'villager_' + k + '_p.glb'), 'villager_' + k + '_p', keep(pos), keep(nor), keep(col));
}

// 2) 무기와 팔: 간단한 도형을 이어 붙인다
const lin = (c) => { const f = (v) => { v /= 255; return v <= 0.04045 ? v / 12.92 : Math.pow((v + 0.055) / 1.055, 2.4); }; return [f((c >> 16) & 255), f((c >> 8) & 255), f(c & 255)]; };
function Mesh() { this.p = []; this.n = []; this.c = []; }
Mesh.prototype.tri = function (a, b, c, color) {
  const u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]], v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
  let nn = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
  const l = Math.hypot(nn[0], nn[1], nn[2]) || 1; nn = nn.map((x) => x / l);
  const rgb = lin(color);
  [a, b, c].forEach((q) => { this.p.push(q[0], q[1], q[2]); this.n.push(nn[0], nn[1], nn[2]); this.c.push(rgb[0], rgb[1], rgb[2]); });
};
Mesh.prototype.quad = function (a, b, c, d, color) { this.tri(a, b, c, color); this.tri(a, c, d, color); };
// 변환: 점 하나를 (회전 -> 이동) 시키는 함수를 받는다
Mesh.prototype.box = function (cx, cy, cz, sx, sy, sz, color) {
  const x = sx / 2, y = sy / 2, z = sz / 2, P = (i, j, k) => [cx + i * x, cy + j * y, cz + k * z];
  this.quad(P(1, -1, -1), P(1, 1, -1), P(1, 1, 1), P(1, -1, 1), color); this.quad(P(-1, -1, 1), P(-1, 1, 1), P(-1, 1, -1), P(-1, -1, -1), color);
  this.quad(P(-1, 1, -1), P(-1, 1, 1), P(1, 1, 1), P(1, 1, -1), color); this.quad(P(-1, -1, 1), P(-1, -1, -1), P(1, -1, -1), P(1, -1, 1), color);
  this.quad(P(-1, -1, 1), P(1, -1, 1), P(1, 1, 1), P(-1, 1, 1), color); this.quad(P(1, -1, -1), P(-1, -1, -1), P(-1, 1, -1), P(1, 1, -1), color);
};
Mesh.prototype.cyl = function (rTop, rBot, y0, y1, color, xf, n) {   // Y 축 원기둥/원뿔. rTop 이 0 이면 뿔
  n = n || 8; xf = xf || ((q) => q);
  const ring = (r, y, i) => { const a = 2 * Math.PI * i / n; return xf([r * Math.cos(a), y, r * Math.sin(a)]); };
  for (let i = 0; i < n; i++) {
    const a = ring(rBot, y0, i), b = ring(rBot, y0, i + 1), c = ring(rTop, y1, i + 1), d = ring(rTop, y1, i);
    this.quad(b, a, d, c, color);   // 바깥쪽이 앞면
    if (rBot > 0) this.tri(xf([0, y0, 0]), a, b, color);
    if (rTop > 0) this.tri(xf([0, y1, 0]), c, d, color);
  }
};
Mesh.prototype.sphere = function (r, cx, cy, cz, color) {
  const W = 8, H = 6, pt = (i, j) => { const th = Math.PI * j / H, ph = 2 * Math.PI * i / W; return [cx + r * Math.sin(th) * Math.cos(ph), cy + r * Math.cos(th), cz + r * Math.sin(th) * Math.sin(ph)]; };
  for (let j = 0; j < H; j++) for (let i = 0; i < W; i++) { const a = pt(i, j), b = pt(i + 1, j), c = pt(i + 1, j + 1), d = pt(i, j + 1); if (j > 0) this.tri(a, d, b, color); if (j < H - 1) this.tri(b, d, c, color); }
};
Mesh.prototype.save = function (file, name) { writeGlb(file, name, this.p, this.n, this.c); };

const WOOD = 0x8a5a3c, STEEL = 0xd5dbe3, GOLD = 0xe0b440, SLEEVE = 0xb5473f, SKIN = 0xf0c9a0;
const ARM = 0.36;
let m = new Mesh();   // 팔: 어깨가 원점, 아래(-Y)로 ARM
m.cyl(0.062, 0.055, -ARM, 0, SLEEVE); m.sphere(0.075, 0, 0, 0, SLEEVE); m.sphere(0.07, 0, -ARM, 0, SKIN);
m.save(path.join(dir, 'arm.glb'), 'arm');
m = new Mesh();   // 검
m.box(0, 0, 0, 0.05, 0.2, 0.05, WOOD); m.box(0, 0.12, 0, 0.22, 0.04, 0.07, GOLD); m.box(0, 0.52, 0, 0.07, 0.75, 0.02, STEEL);
m.save(path.join(dir, 'sword.glb'), 'sword');
m = new Mesh();   // 창: 자루와 뾰족한 끝이 +Z 쪽
const toZ = (z0) => (q) => [q[0], -q[2], q[1] + z0];   // Y 축 도형을 Z 축으로 눕힌다 (+90도 X 회전)
m.cyl(0.025, 0.025, -0.95, 0.95, WOOD, toZ(0.3), 6); m.cyl(0, 0.07, -0.15, 0.15, STEEL, toZ(1.4), 6);
m.save(path.join(dir, 'spear.glb'), 'spear');
m = new Mesh();   // 도끼
m.cyl(0.03, 0.03, -0.2, 0.7, WOOD, null, 6); m.box(0, 0.62, 0.14, 0.04, 0.28, 0.32, STEEL); m.box(0, 0.62, 0, 0.06, 0.32, 0.06, STEEL);
m.save(path.join(dir, 'axe.glb'), 'axe');
console.log('만든 것: villager_N_p.glb, arm.glb, sword.glb, spear.glb, axe.glb ->', dir);
