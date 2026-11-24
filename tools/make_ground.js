// 마을 지도(merchen_village.glb)의 Terrain 메쉬에서 땅 높이 표(ground.txt)를 만든다. (Node 만 있으면 되고 패키지는 필요 없다)
//
//   node tools/make_ground.js [지도.glb] [출력.txt]
//
// 지도의 땅은 -50..50 m 를 171x171 점으로 나눈 격자라서 점 높이를 그대로 옮긴다. 엔진 좌표(x, y=-z, 높이 z)로 쓴다.
// village_server 가 이 표로 접속자의 발 높이와 "물에 빠지지 않는 곳"을 정한다.
'use strict';
const fs = require('fs');
const path = require('path');
const src = process.argv[2] || path.join(__dirname, '..', 'Data', 'Object', 'merchen_village.glb');
const dst = process.argv[3] || path.join(__dirname, '..', 'Data', 'Object', 'ground.txt');
const buf = fs.readFileSync(src);
const jsonLen = buf.readUInt32LE(12);
const json = JSON.parse(buf.slice(20, 20 + jsonLen).toString('utf8'));
const binStart = 20 + jsonLen + 8;
const node = json.nodes.find(function (n) { return n.name === 'Terrain'; });
const prim = json.meshes[node.mesh].primitives[0];
const acc = json.accessors[prim.attributes.POSITION];
const view = json.bufferViews[acc.bufferView];
const off = binStart + (view.byteOffset || 0) + (acc.byteOffset || 0);
const stride = view.byteStride || 12;
const n = acc.count;
const N = Math.round(Math.sqrt(n));            // 171
if (N * N !== n) throw new Error('Terrain 이 정사각 격자가 아닙니다');
const size = acc.max[0] - acc.min[0];
const cell = size / (N - 1);
const h = new Array(N * N);
for (let i = 0; i < n; i++) {
  const x = buf.readFloatLE(off + i * stride), y = buf.readFloatLE(off + i * stride + 4), z = buf.readFloatLE(off + i * stride + 8);
  const ix = Math.round((x - acc.min[0]) / cell);
  const iy = Math.round(((-z) - (-acc.max[2])) / cell);   // 엔진 y = -z
  h[iy * N + ix] = y;
}
let out = '# 땅 높이 표 (엔진 좌표, Z-up, m). tools/make_ground.js 가 merchen_village.glb 에서 만든다.\n';
out += 'size ' + N + ' ' + N + '\n';
out += 'origin ' + (acc.min[0]).toFixed(4) + ' ' + (-acc.max[2]).toFixed(4) + '\n';
out += 'cell ' + cell.toFixed(6) + '\n';
for (let r = 0; r < N; r++) {
  const row = [];
  for (let c = 0; c < N; c++) row.push(h[r * N + c].toFixed(2));
  out += row.join(' ') + '\n';
}
fs.writeFileSync(dst, out);
console.log('wrote ' + dst + ' (' + N + 'x' + N + ', cell ' + cell.toFixed(3) + ' m)');
