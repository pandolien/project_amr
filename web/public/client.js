// 공장/사무동 웹 클라이언트: 내 캐릭터를 뒤에서 따라가는 3인칭으로 걷고, 다른 접속자는 주민 모델로 보인다.
// 위치/충돌은 서버(village_server)가 정한다. 여기서는 입력을 보내고, 받은 위치를 부드럽게 이어서 그린다.
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { clone as cloneSkinned } from 'three/addons/utils/SkeletonUtils.js';

const $ = (id) => document.getElementById(id);
const EYE = 0.78, EYE_FORWARD = 0.22, NAME_HEIGHT = 2.05, HUMAN_H = 1.75;

// ---- 장면 ----
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0xbfe6ff);
scene.fog = new THREE.Fog(0xbfe6ff, 60, 220);
const hemi = new THREE.HemisphereLight(0xffffff, 0x88a070, 1.0);
scene.add(hemi);
const sun = new THREE.DirectionalLight(0xfff2d8, 1.6);
sun.position.set(30, 60, 20);
scene.add(sun);
const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.05, 600);
window.addEventListener('resize', () => {
  renderer.setSize(window.innerWidth, window.innerHeight);
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
});

const loader = new GLTFLoader();
function load(url) { return new Promise((res, rej) => loader.load(url, res, undefined, rej)); }
const villagerTemplates = [];
let mapRoot = null;
const mapReady = load('/assets/factory_map.glb').then((g) => { mapRoot = g.scene; scene.add(g.scene); setupLamps(g.scene); }).catch((e) => console.warn('지도를 못 읽었습니다', e));
// 여러 층 건물: layout.txt (서버와 같은 파일). 벽은 카메라가 벽 뒤로 넘어가지 않게, hide 는 내 층 위쪽을 숨기는 데 쓴다.
const layoutWalls = [];   // { x0, y0, x1, y1, half, zmin, zmax }
const layoutHides = [];   // { name, x0, y0, x1, y1, zlim }
const layoutReady = fetch('/assets/layout.txt').then((r) => (r.ok ? r.text() : '')).then((txt) => {
  for (const line of txt.split('\n')) {
    const t = line.trim().split(/\s+/), n = (i) => parseFloat(t[i]);
    if (t[0] === 'wall') layoutWalls.push({ x0: n(1), y0: n(2), x1: n(3), y1: n(4), half: n(5) / 2, zmin: n(6), zmax: n(7) });
    else if (t[0] === 'hide') layoutHides.push({ name: t[1], x0: n(2), y0: n(3), x1: n(4), y1: n(5), zlim: n(6) });
  }
}).catch((e) => console.warn('층 정보를 못 읽었습니다', e));
function applyFloors(x, y, zb) {   // 건물 안에서는 내 층보다 위(바닥판, 지붕)를 숨긴다. 반환: 지붕 있는 곳(실내)에 있는가
  let indoor = false;
  for (const h of layoutHides) {
    const inside = x >= h.x0 && x <= h.x1 && y >= h.y0 && y <= h.y1;
    const node = mapRoot && mapRoot.getObjectByName(h.name);
    if (node) node.visible = !(inside && zb < h.zlim);
    if (inside && /Roof$/.test(h.name)) indoor = true;
  }
  return indoor;
}
function camBack(x, y, zb, dx, dy, want) {   // (x, y) 에서 (dx, dy) 방향으로 want m 뒤에 카메라를 두되, 벽(내 층에서 막는 것)을 만나면 그 앞에서 멈춘다
  const act = layoutWalls.filter((w) => zb >= w.zmin - 0.35 && zb < w.zmax - 0.35 && w.half <= 0.3 && w.zmax - w.zmin > 1.5);   // 난간과 가구, 선반처럼 낮거나 두꺼운 것은 카메라를 막지 않는다
  let s = 0.5;
  for (; s <= want; s += 0.15) {
    const px = x + dx * s, py = y + dy * s;
    let hit = false;
    for (const w of act) {
      const ex = w.x1 - w.x0, ey = w.y1 - w.y0, l2 = ex * ex + ey * ey;
      let t = l2 < 1e-9 ? 0 : ((px - w.x0) * ex + (py - w.y0) * ey) / l2;
      t = Math.max(0, Math.min(1, t));
      if (Math.hypot(px - (w.x0 + ex * t), py - (w.y0 + ey * t)) < w.half + 0.3) { hit = true; break; }
    }
    if (hit) return Math.max(0.5, s - 0.3);
  }
  return want;
}
// 사람 모델(human.glb: 뼈대가 있고 Idle/Walk/Run 동작이 들어 있다). 못 읽으면 예전 주민 모델로 대신한다.
let humanTpl = null;
const humanTpls = [];
// 사람 모델 두 가지: 군인(human.glb, 자기 동작을 그대로 쓴다)과 평상복 여자(human_w.glb, 동작이 없어 군인의 뼈 회전을 빌린다). 접속자 번호에 따라 둘 중 하나가 된다.
const humanReady = Promise.all([load('/assets/human.glb').catch(() => null), load('/assets/human_w.glb').catch(() => null)]).then(([sol, wom]) => {
  const mk = (g, flip) => {
    const scene = g.scene;
    scene.updateMatrixWorld(true);
    const box = new THREE.Box3().setFromObject(scene);
    return { scene, flip, scale: HUMAN_H / Math.max(0.1, box.max.y - box.min.y), clips: {} };
  };
  if (sol) {
    const t = mk(sol, Math.PI);   // 군인 모델은 -Z 쪽을 본다
    for (const c of sol.animations) t.clips[c.name] = c;
    humanTpls.push(t);
    if (wom) {
      const w = mk(wom, 0);
      for (const c of sol.animations) {   // 다른 몸에 입힐 때는 회전만 쓴다 (크기·위치 값은 몸마다 달라 망가진다)
        const cc = c.clone();
        cc.tracks = cc.tracks.filter((q) => q.name.endsWith('.quaternion') && !q.name.startsWith('mixamorigHips.'));
        w.clips[c.name] = cc;
      }
      humanTpls.push(w);
    }
  }
  humanTpl = humanTpls[0] ? humanTpls[0].scene : null;
}).catch((e) => console.warn('사람 모델을 못 읽었습니다', e));
function makeHuman(id) {
  const root = new THREE.Group();   // 몸 방향(yaw)은 이 그룹을 돌린다
  const T = humanTpls[(((id > 0 ? id : 7 - id) % humanTpls.length) + humanTpls.length) % humanTpls.length];   // 홀수/짝수 번호가 다른 모델
  const inner = cloneSkinned(T.scene);
  inner.scale.multiplyScalar(T.scale);
  inner.rotation.y = T.flip;
  root.add(inner);
  const hue = (((id > 0 ? id : 40 - id) * 0.173) % 1 + 1) % 1;   // 사람마다 옷 빛깔을 조금씩 다르게
  inner.traverse((o) => {
    if (!o.isMesh) return;
    o.frustumCulled = false;
    o.material = o.material.clone();
    if (T.flip && o.material.color) o.material.color.setHSL(hue, 0.35, 0.75);   // 군인 모델일 때만 옷 빛깔을 바꾼다
  });
  const mixer = new THREE.AnimationMixer(inner);
  const acts = {};
  for (const n of ['Idle', 'Walk', 'Run']) if (T.clips[n]) acts[n] = mixer.clipAction(T.clips[n]);
  if (acts.Idle) { acts.Idle.play(); mixer.update(Math.random()); }
  root.updateMatrixWorld(true);
  const handBone = inner.getObjectByName('mixamorigRightHand');
  const armBone = inner.getObjectByName('mixamorigRightArm');
  const anchor = new THREE.Group();   // 무기를 쥐는 자리 (손뼈에 붙어 팔 동작을 따라간다)
  if (handBone) {
    handBone.add(anchor);
    const ws = new THREE.Vector3(); handBone.getWorldScale(ws);
    anchor.scale.setScalar(1 / ws.x);   // 뼈대가 센티미터 단위라 줄어든 만큼 되돌린다
    anchor.position.set(0, 8, 0);
  }
  return { root, mixer, acts, cur: '', anchor, armBone };
}
const modelsReady = Promise.all([humanReady, ...[1, 2, 3, 4].map((n) => load('/assets/villager_' + n + '.glb').then((g) => { villagerTemplates[n - 1] = g.scene; }).catch(() => {}))]);

// ---- 가로등 불 + 낮/노을/밤 ----
// 가로등 전구(지도의 Lanterns 메쉬 꼭대기)마다 빛무리와 실제 조명을 단다. 낮에는 은은하게, 밤에는 환하게 켜진다.
const lamps = [];   // { x, y, z, glow }
const lampPool = [];   // 가까운 가로등으로 옮겨 다니는 조명
function glowTexture() {
  const c = document.createElement('canvas');
  c.width = c.height = 128;
  const g = c.getContext('2d');
  const grad = g.createRadialGradient(64, 64, 0, 64, 64, 64);
  grad.addColorStop(0, 'rgba(255,236,170,1)'); grad.addColorStop(0.25, 'rgba(255,214,120,0.55)'); grad.addColorStop(1, 'rgba(255,200,90,0)');
  g.fillStyle = grad; g.fillRect(0, 0, 128, 128);
  return new THREE.CanvasTexture(c);
}
function setupLamps(root) {
  const node = root.getObjectByName('Lanterns');
  if (!node) return;
  node.updateWorldMatrix(true, true);
  const clusters = [];   // 가로등 한 개 = 가까운 꼭짓점 무리 (xz 0.6 m 안)
  const v = new THREE.Vector3();
  node.traverse((o) => {
    if (!o.isMesh) return;
    const pos = o.geometry.getAttribute('position');
    for (let i = 0; i < pos.count; i++) {
      v.fromBufferAttribute(pos, i).applyMatrix4(o.matrixWorld);
      let c = clusters.find((k) => Math.hypot(k.x - v.x, k.z - v.z) < 0.6);
      if (!c) { c = { x: v.x, z: v.z, top: -1e9 }; clusters.push(c); }
      if (v.y > c.top) c.top = v.y;
    }
  });
  const tex = glowTexture();
  for (const c of clusters) {
    const y = c.top - 0.2;   // 전구 중심 (전구 반지름 0.22 m)
    const glow = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, transparent: true, depthWrite: false, blending: THREE.AdditiveBlending }));
    glow.position.set(c.x, y, c.z);
    glow.scale.set(1.9, 1.9, 1);
    const bulb = new THREE.Mesh(new THREE.SphereGeometry(0.23, 12, 10), new THREE.MeshBasicMaterial({ color: 0xffe9a8 }));   // 조명과 상관없이 늘 밝은 전구
    bulb.position.set(c.x, y, c.z);
    scene.add(bulb);
    scene.add(glow);
    lamps.push({ x: c.x, y, z: c.z, glow });
  }
  // 실제 조명은 내 가까이의 가로등 3개만 켠다 (조명이 많으면 모든 화면 픽셀이 느려진다)
  for (let i = 0; i < 3; i++) { const l = new THREE.PointLight(0xffd27a, 1, 11, 1.6); l.position.set(0, -100, 0); scene.add(l); lampPool.push(l); }
  applyTime();
}
const TIMES = [
  { name: '낮',   sky: 0xbfe6ff, hemi: [0xffffff, 0x88a070, 1.0], sun: [0xfff2d8, 1.6], sunPos: [30, 60, 20], lamp: 0.5, glow: 0.45 },
  { name: '노을', sky: 0xf2a98f, hemi: [0xffd6c0, 0x7b6a9a, 0.8], sun: [0xff9a6a, 1.0], sunPos: [60, 20, 10], lamp: 1.4, glow: 0.85 },
  { name: '밤',   sky: 0x0d1330, hemi: [0x8fa0d8, 0x2a2f4a, 0.45], sun: [0x8fa6ff, 0.25], sunPos: [-30, 50, 20], lamp: 2.4, glow: 1.0 },
];
let timeIdx = 0;
function applyTime() {
  const t = TIMES[timeIdx];
  scene.background.set(t.sky); scene.fog.color.set(t.sky);
  hemi.color.set(t.hemi[0]); hemi.groundColor.set(t.hemi[1]); hemi.intensity = t.hemi[2];
  sun.color.set(t.sun[0]); sun.intensity = t.sun[1]; sun.position.set(...t.sunPos);
  for (const l of lamps) l.glow.material.opacity = t.glow;
  for (const l of lampPool) l.intensity = t.lamp;
  document.body.style.background = '#' + t.sky.toString(16).padStart(6, '0');
}
let lampTick = 0;
function updateLampLights(px, pz) {
  if (++lampTick % 15) return;
  const near = lamps.map((l) => ({ l, d: Math.hypot(l.x - px, l.z - pz) })).sort((a, b) => a.d - b.d);
  lampPool.forEach((p, i) => { if (near[i]) p.position.set(near[i].l.x, near[i].l.y, near[i].l.z); });
}
function nextTime() { timeIdx = (timeIdx + 1) % TIMES.length; applyTime(); }

// ---- 접속자 ----
const players = new Map();   // id -> { group, model, label, tx, ty, tz, x, y, z, yaw, name }
let myId = 0, third = true, myZoneInit = false, lastZone = -1;
let lastState = null;

function makeLabel(text) {
  const c = document.createElement('canvas');
  c.width = 256; c.height = 64;
  const g = c.getContext('2d');
  g.font = 'bold 30px system-ui, "Noto Sans KR", sans-serif';
  const w = Math.min(244, g.measureText(text).width + 28);
  g.fillStyle = 'rgba(20,24,32,0.72)';
  g.beginPath(); g.roundRect((256 - w) / 2, 8, w, 48, 20); g.fill();
  g.fillStyle = '#fff'; g.textAlign = 'center'; g.textBaseline = 'middle';
  g.fillText(text, 128, 33, 232);
  const tex = new THREE.CanvasTexture(c);
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true }));
  s.scale.set(1.1, 0.275, 1);
  s.renderOrder = 10;
  return s;
}
// 말풍선: 글을 줄바꿈해서 그린 스프라이트 (캐릭터 이름표 위)
function makeBubble(text) {
  const c = document.createElement('canvas');
  c.width = 512; c.height = 192;
  const g = c.getContext('2d');
  g.font = 'bold 34px system-ui, "Noto Sans KR", sans-serif';
  const lines = [];
  let cur = '';
  for (const ch of Array.from(text)) {
    if (g.measureText(cur + ch).width > 420) { lines.push(cur); cur = ch; } else cur += ch;
  }
  if (cur) lines.push(cur);
  const shown = lines.slice(0, 3);
  const w = Math.min(480, Math.max(...shown.map((l) => g.measureText(l).width)) + 44);
  const h = shown.length * 44 + 28;
  const x = (512 - w) / 2, y = 168 - h;   // 아래쪽을 기준으로 쌓는다 (꼬리가 이름표 쪽)
  g.fillStyle = '#fff'; g.strokeStyle = '#7a63ff'; g.lineWidth = 5;
  g.beginPath(); g.roundRect(x, y, w, h, 24); g.fill(); g.stroke();
  g.beginPath(); g.moveTo(236, y + h); g.lineTo(256, y + h + 20); g.lineTo(276, y + h); g.closePath(); g.fillStyle = '#fff'; g.fill();
  g.fillStyle = '#2b2b3a'; g.textAlign = 'center'; g.textBaseline = 'middle';
  shown.forEach((l, i) => g.fillText(l, 256, y + 14 + 22 + i * 44));
  const tex = new THREE.CanvasTexture(c);
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true }));
  s.scale.set(2.0, 0.75, 1);
  s.center.set(0.5, 0.0);
  s.renderOrder = 11;
  return s;
}
function addLog(name, text) {
  const d = document.createElement('div');
  d.textContent = name + ': ' + text;
  const log = $('log');
  log.appendChild(d);
  while (log.children.length > 6) log.removeChild(log.firstChild);
  setTimeout(() => d.remove(), 15000);
}
// ---- 무기: 0 검, 1 창, 2 도끼 (손에 든 모양. 휘두르면 앞으로 내려친다/찌른다) ----
const WEAPON_NAMES = ['검', '창', '도끼'];
function makeWeapon(w) {
  const g = new THREE.Group();
  const wood = new THREE.MeshLambertMaterial({ color: 0x8a5a3c }), steel = new THREE.MeshLambertMaterial({ color: 0xd5dbe3 }), gold = new THREE.MeshLambertMaterial({ color: 0xe0b440 });
  const add = (geo, mat, x, y, z) => { const m = new THREE.Mesh(geo, mat); m.position.set(x, y, z); g.add(m); return m; };
  if (w === 0) {          // 검: 손잡이, 날밑, 칼날 (위로 세워 든다)
    add(new THREE.BoxGeometry(0.05, 0.2, 0.05), wood, 0, 0, 0);
    add(new THREE.BoxGeometry(0.22, 0.04, 0.07), gold, 0, 0.12, 0);
    add(new THREE.BoxGeometry(0.07, 0.75, 0.02), steel, 0, 0.52, 0);
  } else if (w === 1) {   // 창: 긴 자루와 뾰족한 끝 (앞으로 겨눈다)
    const shaft = add(new THREE.CylinderGeometry(0.025, 0.025, 1.9, 6), wood, 0, 0, 0.3); shaft.rotation.x = Math.PI / 2;
    const tip = add(new THREE.ConeGeometry(0.07, 0.3, 6), steel, 0, 0, 1.4); tip.rotation.x = Math.PI / 2;
  } else {                // 도끼: 자루와 넓은 날 (위로 세워 든다)
    add(new THREE.CylinderGeometry(0.03, 0.03, 0.9, 6), wood, 0, 0.25, 0);
    add(new THREE.BoxGeometry(0.04, 0.28, 0.32), steel, 0, 0.62, 0.14);
    add(new THREE.BoxGeometry(0.06, 0.32, 0.06), steel, 0, 0.62, 0);
  }
  return g;
}
function setWeapon(e, w) {   // 손(hand)에 든 무기를 바꾼다
  if (e.weaponId === w) return;
  const first = e.weaponId === -1;
  e.weaponId = w;
  if (e.human && !first) e.swap = 0.4;   // 무기를 바꾸는 동작
  if (e.weaponMesh) e.hand.remove(e.weaponMesh);
  e.weaponMesh = makeWeapon(w);
  e.hand.add(e.weaponMesh);
  e.weaponMesh.position.set(0, 0, 0);   // 손(주먹) 자리가 무기를 쥐는 곳 (창은 이 안에서 미끄러진다)
}
const SWING = 0.32;
function drawHp(e, hp) {   // 머리 위 체력 바
  if (e.hp === hp) return;
  e.hp = hp;
  const g = e.hpCanvas.getContext('2d');
  g.clearRect(0, 0, 128, 16);
  g.fillStyle = 'rgba(30,30,40,.75)'; g.fillRect(0, 0, 128, 16);
  g.fillStyle = hp > 50 ? '#5fd068' : (hp > 25 ? '#f2c14e' : '#ee5a5a');
  g.fillRect(2, 2, Math.max(0, 124 * hp / 100), 12);
  e.hpTex.needsUpdate = true;
}
// 주민 모델은 한 덩어리라 팔을 따로 움직일 수 없다. 오른팔(삼각형 76~147: 소매 76~93, 손 94~147) 만 빼고, 그 자리에 무기를 쥔 팔을 따로 그린다.
const ARM_TRIS = [76, 147], armlessCache = new Map();
function armlessCopy(tpl) {
  const model = tpl.clone(true);
  model.traverse((o) => {
    if (!o.isMesh || o.geometry.index || o.geometry.getAttribute('position').count !== 544 * 3) return;   // 예상한 모양(544 삼각형)일 때만
    let g = armlessCache.get(o.geometry);
    if (!g) {
      g = new THREE.BufferGeometry();
      for (const name of ['position', 'normal', 'color']) {
        const a = o.geometry.getAttribute(name);
        if (!a) continue;
        const n = a.itemSize, out = [];
        for (let t = 0; t < 544; t++) if (t < ARM_TRIS[0] || t > ARM_TRIS[1]) for (let i = 0; i < 9 * n / 3; i++) out.push(a.array[t * 3 * n + i]);
        g.setAttribute(name, new THREE.BufferAttribute(new Float32Array(out), n));
      }
      armlessCache.set(o.geometry, g);
    }
    o.geometry = g;
  });
  return model;
}
function makePlayer(p) {
  const group = new THREE.Group();
  const tpl = villagerTemplates[(p.id > 0 ? p.id - 1 : -p.id - 1) % 4];   // NPC 는 id 가 음수 (-k = k 번째 주민)
  const fighter = p.hp !== undefined;   // 접속자는 무기를 쥘 오른팔을 따로 그리므로 모델의 원래 오른팔은 뺀다 (NPC 는 그대로)
  let model, human = null;
  if (humanTpl) { human = makeHuman(p.id); model = human.root; }
  else if (tpl) model = fighter ? armlessCopy(tpl) : tpl.clone(true);
  else { model = new THREE.Mesh(new THREE.CapsuleGeometry(0.25, 0.6, 4, 8), new THREE.MeshLambertMaterial({ color: 0xe8a0a0 })); model.position.y = 0.55; }
  group.add(model);
  const label = makeLabel(p.n);
  label.position.y = NAME_HEIGHT;
  group.add(label);
  let hpSprite = null, hpCanvas = null, hpTex = null;
  if (p.hp !== undefined) {
    hpCanvas = document.createElement('canvas'); hpCanvas.width = 128; hpCanvas.height = 16;
    hpTex = new THREE.CanvasTexture(hpCanvas);
    hpSprite = new THREE.Sprite(new THREE.SpriteMaterial({ map: hpTex, transparent: true, depthTest: false }));
    hpSprite.scale.set(0.64, 0.08, 1); hpSprite.position.y = NAME_HEIGHT - 0.2; hpSprite.renderOrder = 998;
    group.add(hpSprite);
  }
  const shadow = new THREE.Mesh(new THREE.CircleGeometry(0.28, 14), new THREE.MeshBasicMaterial({ color: 0x000000, transparent: true, opacity: 0.28, depthWrite: false }));   // 발밑 그림자: 점프하면 땅에 남아 높이가 보인다
  shadow.rotation.x = -Math.PI / 2; shadow.renderOrder = 1;
  scene.add(shadow);
  const hand = human ? human.anchor : new THREE.Group();   // 오른손 자리 (사람 모델은 손뼈, 아니면 몸 방향을 따라 도는 그룹)
  hand.position.set(-0.3, 0.45, 0.12);
  if (human) hand.position.set(0, 8, 0); else group.add(hand);
  const sleeve = new THREE.Mesh(new THREE.CylinderGeometry(0.062, 0.055, 1, 8), new THREE.MeshLambertMaterial({ color: 0xb5473f }));   // 어깨에서 손까지 이어지는 팔
  const fist = new THREE.Mesh(new THREE.SphereGeometry(0.07, 8, 6), new THREE.MeshLambertMaterial({ color: 0xf0c9a0 }));
  hand.add(fist);   // 무기를 쥔 손은 hand 에 붙어 같이 움직인다
  const shoulder = new THREE.Mesh(new THREE.SphereGeometry(0.075, 8, 6), sleeve.material);   // 어깨 관절: 몸과 팔 사이가 벌어져 보이지 않게
  if (!human) { group.add(sleeve); group.add(shoulder); }
  scene.add(group);
  const e = { human, spd: 0, shadow, jh: 0, hand, sleeve, shoulder, weaponId: -1, weaponMesh: null, swing: 0, lastAk: p.ak || 0, hpSprite, hpCanvas, hpTex, hp: -1, group, model, label, bubble: null, bubbleText: '', lastSid: p.sid || 0, x: p.x, y: p.y, z: p.z, tx: p.x, ty: p.y, tz: p.z, yaw: p.yaw, tyaw: p.yaw, name: p.n };
  players.set(p.id, e);
  return e;
}
function onState(s) {
  lastState = s;
  const seen = new Set();
  for (const p of s.p) {
    seen.add(p.id);
    const e = players.get(p.id) || makePlayer(p);
    if (Math.hypot(p.x - e.x, p.y - e.y) > 15) { e.x = p.x; e.y = p.y; e.z = p.z; e.yaw = p.yaw; }   // 건물 안팎으로 옮겨졌다: 미끄러지지 않고 바로
    e.tx = p.x; e.ty = p.y; e.tz = p.z; e.tyaw = p.yaw;
    e.zone = zoneAt(p.x, p.y);
    e.jh = p.jh || 0;   // 땅에서 떠 있는 높이 (점프)
    if (e.hpSprite && p.w !== undefined) {
      setWeapon(e, p.w);
      if (p.ak !== e.lastAk) { e.lastAk = p.ak; e.swing = SWING; }
      if (p.id === myId) $('wpn').textContent = '무기: ' + WEAPON_NAMES[p.w] + '  (1 검 · 2 창 · 3 도끼)';
    }
    if (e.hpSprite && p.hp !== undefined) {
      drawHp(e, p.hp);
      if (p.id === myId) {   // 내 체력: 화면 아래 바, 맞으면 붉게 번쩍
        $('hpfill').style.width = p.hp + '%'; $('hptext').textContent = 'HP ' + p.hp;
        if (e.lastHp !== undefined && p.hp < e.lastHp) { $('flash').style.opacity = 1; setTimeout(() => { $('flash').style.opacity = 0; }, 160); }
        e.lastHp = p.hp;
      }
    }
    if (p.s) {
      if (p.s !== e.bubbleText) {   // 새 말이면 풍선을 다시 그린다
        if (e.bubble) { e.group.remove(e.bubble); e.bubble.material.map.dispose(); e.bubble.material.dispose(); }
        e.bubble = makeBubble(p.s);
        e.bubble.position.y = NAME_HEIGHT + 0.2;
        e.group.add(e.bubble);
        e.bubbleText = p.s;
      }
      if (p.sid !== e.lastSid) { addLog(p.n, p.s); e.lastSid = p.sid; }
    } else if (e.bubble) {
      e.group.remove(e.bubble); e.bubble.material.map.dispose(); e.bubble.material.dispose();
      e.bubble = null; e.bubbleText = '';
    }
  }
  for (const [id, e] of players) if (!seen.has(id)) { scene.remove(e.group); scene.remove(e.shadow); players.delete(id); }
  const me = players.get(myId);
  if (me && me.zone !== myZone) {   // 건물에 들어가거나 나왔다: 방을 보이게 하고 바라보는 방향을 맞춘다
    const entering = me.zone >= 0;
    setZone(me.zone);
    if (myZoneInit) { yaw = entering ? Math.PI : (rooms.find((r, i) => i === lastZone) || { outYaw: yaw }).outYaw; pitch = 0; sendInput(true); }
    lastZone = me.zone;
  }
  myZoneInit = true;
  const dp = doorPrompt(me);
  $('prompt').hidden = !dp;
  if (dp) $('prompt').textContent = dp;
  $('hud').textContent = (me ? me.name : '') + ' · 접속 ' + s.p.length + '명 | WASD 이동 · Shift 달리기 · 마우스 시점 · B 문 앞에서 들어가기/나가기 · Space 점프 · F/클릭 공격 · 1·2·3 무기 · N 낮/노을/밤 · Enter 채팅 · Esc 마우스 풀기';
}


// ---- 건물 내부 ----
// 방은 지도 밖(x=200~)에 따로 놓여 있다. portals.txt (서버와 같은 파일)로 그리고, 문 앞에서 B 를 누르면 서버가 방 안으로 옮겨 준다.
const rooms = [];   // { name, kind, outX, outY, outYaw, cx, cy, hw, hd, ceil, floorZ, group }
const roomLight = new THREE.PointLight(0xffe2b0, 2.6, 18, 1.2);   // 지금 내가 들어 있는 방으로 옮겨 다닌다 (조명 개수는 늘 같게)
roomLight.position.set(0, -100, 0);
scene.add(roomLight);
function hexColor(s) { return new THREE.Color(parseInt(s, 16)); }
function buildRooms(text) {
  const furn = [];
  for (const raw of text.split('\n')) {
    const t = raw.trim().split(/\s+/);
    if (t[0] === 'room' && t.length >= 14) {
      const n = (i) => parseFloat(t[i]);
      rooms.push({ name: t[1], kind: t[2], outX: n(3), outY: n(4), outYaw: n(5), cx: n(6), cy: n(7), hw: n(8), hd: n(9), ceil: n(10), floorZ: n(11), wall: t[12], floor: t[13], group: null });
    } else if (t[0] === 'furn' && t.length >= 10) furn.push(t);
  }
  for (const r of rooms) {
    const g = new THREE.Group();
    g.position.set(r.cx, 0, -r.cy);   // 방 중심 (엔진 x, y -> three x, -y)
    const shell = new THREE.Mesh(new THREE.BoxGeometry(r.hw * 2, r.ceil, r.hd * 2), new THREE.MeshLambertMaterial({ color: hexColor(r.wall), side: THREE.BackSide }));
    shell.position.y = r.floorZ + r.ceil / 2;
    g.add(shell);
    const floor = new THREE.Mesh(new THREE.PlaneGeometry(r.hw * 2, r.hd * 2), new THREE.MeshLambertMaterial({ color: hexColor(r.floor) }));
    floor.rotation.x = -Math.PI / 2; floor.position.y = r.floorZ + 0.005;
    g.add(floor);
    // 문: 남쪽 벽(엔진 y 가 작은 쪽 = three +z)
    const door = new THREE.Mesh(new THREE.BoxGeometry(1.2, 2.1, 0.08), new THREE.MeshLambertMaterial({ color: 0x7a4f35 }));
    door.position.set(0, r.floorZ + 1.05, r.hd - 0.05);
    g.add(door);
    const frame = new THREE.Mesh(new THREE.BoxGeometry(1.4, 2.25, 0.05), new THREE.MeshLambertMaterial({ color: 0xe9dcc3 }));
    frame.position.set(0, r.floorZ + 1.125, r.hd - 0.03);
    g.add(frame);
    const knob = new THREE.Mesh(new THREE.SphereGeometry(0.06, 8, 6), new THREE.MeshBasicMaterial({ color: 0xffd35c }));
    knob.position.set(0.42, r.floorZ + 1.0, r.hd - 0.12);
    g.add(knob);
    // 벽 장식: 천장 쪽 굵은 띠(몰딩)와 바닥 걸레받이로 방이 덜 밋밋하게
    for (const [y, hgt, col] of [[r.floorZ + 0.07, 0.14, 0x9a7a58], [r.floorZ + r.ceil - 0.07, 0.12, 0xffffff]]) {
      const band = new THREE.Mesh(new THREE.BoxGeometry(r.hw * 2 - 0.02, hgt, r.hd * 2 - 0.02), (col === 0xffffff ? new THREE.MeshBasicMaterial({ color: 0xfff6e6, side: THREE.BackSide }) : new THREE.MeshLambertMaterial({ color: col, side: THREE.BackSide })));   // 천장은 시간대와 상관없이 밝은 크림색
      band.position.y = y; g.add(band);
    }
    g.visible = false;
    scene.add(g);
    r.group = g;
  }
  for (const t of furn) {
    const r = rooms.find((q) => q.name === t[1]);
    if (!r) continue;
    const [x, y, w, d, h] = [2, 3, 4, 5, 6].map((i) => parseFloat(t[i]));
    const glow = t[9] === '1';
    const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), glow ? new THREE.MeshBasicMaterial({ color: hexColor(t[7]) }) : new THREE.MeshLambertMaterial({ color: hexColor(t[7]) }));
    m.position.set(x - r.cx, r.floorZ + h / 2, -(y - r.cy));
    r.group.add(m);
  }
}
const roomsReady = fetch('/assets/portals.txt').then((r) => (r.ok ? r.text() : '')).then(buildRooms).catch((e) => console.warn('건물 내부를 못 읽었습니다', e));
function zoneAt(x, y) {   // 이 자리가 들어 있는 방 번호, 밖이면 -1
  for (let i = 0; i < rooms.length; i++) { const r = rooms[i]; if (Math.abs(x - r.cx) <= r.hw && Math.abs(y - r.cy) <= r.hd) return i; }
  return -1;
}
let myZone = -1;
function setZone(z) {
  myZone = z;
  rooms.forEach((r, i) => { r.group.visible = i === z; });
  if (z >= 0) { const r = rooms[z]; roomLight.position.set(r.cx, r.floorZ + r.ceil - 0.5, -r.cy); }
  else roomLight.position.set(0, -100, 0);
}
// 문 앞이면 어떤 안내를 보일지: { text, zone }
function doorPrompt(me) {
  if (!me) return null;
  const z = zoneAt(me.tx, me.ty);
  if (z >= 0) {
    const r = rooms[z];
    return Math.hypot(me.tx - r.cx, me.ty - (r.cy - r.hd)) <= 2.2 ? '문 앞 — B 키: 밖으로 나가기' : null;
  }
  for (const r of rooms) if (Math.hypot(me.tx - r.outX, me.ty - r.outY) <= 2.2) return 'B 키: ' + (r.kind === 'castle' ? '성' : '집') + ' 안으로 들어가기';
  return null;
}

// ---- 입력 ----
const keys = new Set();
let yaw = 0, pitch = 0, locked = false, sentKey = '';
const chatbox = $('chatbox'), chatinput = $('chatinput');
let typing = false;
function openChat() {
  typing = true; keys.clear();
  chatbox.style.display = 'block';
  if (document.exitPointerLock) document.exitPointerLock();
  chatinput.value = ''; chatinput.focus();
}
function closeChat() { typing = false; chatbox.style.display = 'none'; chatinput.blur(); }
chatinput.addEventListener('keydown', (e) => {
  e.stopPropagation();
  if (e.key === 'Enter') {
    const t = chatinput.value.trim();
    if (t && ws && ws.readyState === 1) ws.send(JSON.stringify({ type: 'chat', text: t }));
    closeChat();
  } else if (e.key === 'Escape') closeChat();
});
window.addEventListener('keydown', (e) => {
  if ($('join').style.display !== 'none') return;
  if (typing) return;
  if (e.code === 'Enter' && myId) { e.preventDefault(); openChat(); return; }
  if (e.code === 'KeyN' && !e.repeat) { nextTime(); return; }
  if (/^Digit[123]$/.test(e.code) && !e.repeat) { if (ws && ws.readyState === 1 && myId) ws.send(JSON.stringify({ type: 'weapon', w: Number(e.code.slice(5)) - 1 })); return; }
  if (e.code === 'Space' && !e.repeat) { e.preventDefault(); if (ws && ws.readyState === 1 && myId) ws.send(JSON.stringify({ type: 'jump' })); return; }
  if (e.code === 'KeyF' && !e.repeat) { attack(); return; }
  if (e.code === 'KeyB' && !e.repeat) { if (ws && ws.readyState === 1 && myId) ws.send(JSON.stringify({ type: 'enter' })); return; }
  keys.add(e.code);
  if (['ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown', 'Space'].includes(e.code)) e.preventDefault();
});
window.addEventListener('keyup', (e) => keys.delete(e.code));
window.addEventListener('blur', () => keys.clear());
function attack() { if (ws && ws.readyState === 1 && myId) ws.send(JSON.stringify({ type: 'attack' })); }
renderer.domElement.addEventListener('mousedown', (e) => { if (locked && e.button === 0) attack(); });
document.addEventListener('pointerlockchange', () => { locked = document.pointerLockElement === renderer.domElement; $('hint').hidden = locked || !myId; });
renderer.domElement.addEventListener('click', () => { if (myId && !locked && renderer.domElement.requestPointerLock) renderer.domElement.requestPointerLock(); });
document.addEventListener('mousemove', (e) => {
  if (!locked) return;
  yaw -= e.movementX * 0.0025;
  pitch = Math.max(-1.2, Math.min(1.2, pitch - e.movementY * 0.0025));
});
// 화면을 누른 채 끌어도 돌아간다 (포인터 잠금이 안 되는 환경용)
let dragging = false;
renderer.domElement.addEventListener('pointerdown', () => { dragging = true; });
window.addEventListener('pointerup', () => { dragging = false; });
window.addEventListener('pointermove', (e) => {
  if (!dragging || locked) return;
  yaw -= e.movementX * 0.004;
  pitch = Math.max(-1.2, Math.min(1.2, pitch - e.movementY * 0.004));
});

let ws = null;
function sendInput(force) {
  if (!ws || ws.readyState !== 1 || !myId) return;
  const fwd = (keys.has('KeyW') || keys.has('ArrowUp') ? 1 : 0) - (keys.has('KeyS') || keys.has('ArrowDown') ? 1 : 0);
  const side = (keys.has('KeyD') ? 1 : 0) - (keys.has('KeyA') ? 1 : 0);
  const run = keys.has('ShiftLeft') || keys.has('ShiftRight') ? 1 : 0;
  const k = fwd + ',' + side + ',' + run + ',' + yaw.toFixed(2);
  if (!force && k === sentKey) return;
  sentKey = k;
  ws.send(JSON.stringify({ type: 'input', fwd, side, run, yaw: Number(yaw.toFixed(4)) }));
}
setInterval(() => sendInput(false), 50);
setInterval(() => sendInput(true), 500);   // 상태가 어긋나도 곧 맞춰지게

// ---- 접속 ----
async function join() {
  const name = $('name').value.trim() || '손님';
  $('msg').textContent = '접속하는 중…';
  await Promise.all([mapReady, modelsReady, roomsReady, layoutReady]);
  ws = new WebSocket((location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host);
  ws.onopen = () => ws.send(JSON.stringify({ type: 'join', name }));
  ws.onmessage = (ev) => {
    const m = JSON.parse(ev.data);
    if (m.type === 'welcome') {
      myId = m.id;
      $('join').style.display = 'none';
      $('hud').hidden = false; $('hpbar').hidden = false; $('wpn').hidden = false; $('hint').hidden = false; $('cross').hidden = true;
    } else if (m.type === 'full') {
      $('msg').textContent = '지금은 가득 찼어요 (최대 ' + m.max + '명). 조금 뒤에 다시 와 주세요.';
    } else if (m.p) onState(m);
  };
  ws.onclose = () => {
    if (myId) { myId = 0; $('join').style.display = 'flex'; }
    if (!$('msg').textContent.startsWith('지금은')) $('msg').textContent = '연결이 끊겼어요. 다시 들어와 주세요.';
  };
  ws.onerror = () => { $('msg').textContent = '서버에 연결할 수 없어요.'; };
}
$('go').addEventListener('click', join);
$('name').addEventListener('keydown', (e) => { if (e.key === 'Enter') join(); });

// ---- 그리기 ----
const _pq = new THREE.Quaternion(), _rw = new THREE.Quaternion(), _ax = new THREE.Vector3();
function animateHuman(e, id, dt, inst) {   // 걷는/달리는 속도에 맞춰 동작을 고르고, 휘두를 때는 오른팔을 들어 올린다
  const H = e.human;
  e.spd += (inst - e.spd) * Math.min(1, dt * 10);
  const want = e.spd < 0.4 ? 'Idle' : (e.spd < 3.6 ? 'Walk' : 'Run');
  if (want !== H.cur && H.acts[want]) {
    const a = H.acts[want], old = H.acts[H.cur];
    a.reset().play();
    if (old) a.crossFadeFrom(old, 0.2, false);
    H.cur = want;
  }
  const a = H.acts[H.cur];
  if (a) a.timeScale = H.cur === 'Walk' ? Math.max(0.6, Math.min(1.7, e.spd / 1.6)) : (H.cur === 'Run' ? Math.max(0.8, Math.min(1.5, e.spd / 4)) : 1);
  H.mixer.update(dt);
  if (e.swing > 0 && window.__swingFreeze === undefined) e.swing = Math.max(0, e.swing - dt);
  const tt = e.swing > 0 ? 1 - e.swing / SWING : 0, ry = id === myId ? yaw : e.yaw;
  let ang = 0.45;   // 무기를 쥔 팔은 살짝 앞으로 든다
  if (e.weaponId === 1) ang += 0.9 * Math.sin(Math.PI * tt);   // 창: 앞으로 찌른다
  else if (tt > 0) { if (tt < 0.4) ang += 2.2 * (tt / 0.4); else { const u = (tt - 0.4) / 0.6; ang += 2.2 * (1 - u) - 0.25 * Math.sin(Math.PI * u); } }   // 검/도끼: 머리 위로 들었다 내려친다
  if (e.swap > 0) {   // 무기 교체: 팔을 들어 올리고 새 무기가 손에서 커진다
    e.swap = Math.max(0, e.swap - dt);
    const u = 1 - e.swap / 0.4;
    ang += 1.4 * Math.sin(Math.PI * u);
    if (e.weaponMesh) e.weaponMesh.scale.setScalar(Math.max(0.02, u < 0.4 ? 0.02 : (u - 0.4) / 0.6));
  } else if (e.weaponMesh && e.weaponMesh.scale.x !== 1) e.weaponMesh.scale.setScalar(1);
  ang += Math.min(1, (e.jh || 0) * 2) * 0.9;   // 점프하면 팔을 든다
  if (H.armBone && H.armBone.parent) {
    H.armBone.parent.getWorldQuaternion(_pq);
    _ax.set(-Math.cos(ry), 0, Math.sin(ry));   // 몸의 오른쪽 축 (three)
    _rw.setFromAxisAngle(_ax, ang);
    _rw.premultiply(_pq.clone().invert()).multiply(_pq);   // 몸 기준 회전을 팔 부모 뼈의 좌표로 옮긴다
    H.armBone.quaternion.premultiply(_rw);
  }
}
let camZ = null;   // 카메라가 보는 높이 (캐릭터 높이를 천천히 따라간다)
const clock = new THREE.Clock();
function frame() {
  requestAnimationFrame(frame);
  const dt = Math.min(clock.getDelta(), 0.1);
  const k = 1 - Math.exp(-16 * dt);
  if (keys.has('KeyQ') || keys.has('ArrowLeft')) yaw += 1.8 * dt;     // 마우스가 없어도 돌 수 있게
  if (keys.has('KeyE') || keys.has('ArrowRight')) yaw -= 1.8 * dt;
  for (const [id, e] of players) {
    const ox = e.x, oy = e.y;
    e.x += (e.tx - e.x) * k; e.y += (e.ty - e.y) * k; e.z += (e.tz - e.z) * k;
    let d = e.tyaw - e.yaw;
    while (d > Math.PI) d -= 2 * Math.PI;
    while (d < -Math.PI) d += 2 * Math.PI;
    e.yaw += d * k;
    e.group.visible = id === myId || e.zone === myZone;   // 다른 건물 안/밖에 있는 사람은 안 보인다
    e.shadow.visible = e.group.visible && e.hpSprite !== null;
    e.shadow.position.set(e.x, e.z - (e.jh || 0) + 0.04, -e.y);
    e.shadow.scale.setScalar(Math.max(0.5, 1 - (e.jh || 0) * 0.5));
    e.group.position.set(e.x, e.z, -e.y);   // 엔진(x, y, 높이 z) -> three(x, 높이, -y)
    e.model.rotation.y = id === myId ? yaw : e.yaw;
    if (e.human) animateHuman(e, id, dt, Math.min(12, Math.hypot(e.x - ox, e.y - oy) / Math.max(dt, 1e-3)));
    else if (e.hand) {   // 무기를 쥔 손: 어깨에서 팔이 뻗어 나가고, 휘두르는 동안 손이 앞으로 나가며 내려친다/찌른다
      if (e.swing > 0 && window.__swingFreeze === undefined) e.swing = Math.max(0, e.swing - dt);   // (__swingFreeze: 시험용으로 휘두르는 동작을 멈춰 두는 스위치)
      const tt = e.swing > 0 ? 1 - e.swing / SWING : 0, ry = id === myId ? yaw : e.yaw;
      const fxh = Math.sin(ry), fzh = Math.cos(ry), rxh = -Math.cos(ry), rzh = Math.sin(ry);   // 몸 앞 / 오른쪽 (three)
      // 팔은 어깨에 붙은 채 길이가 일정하게 돈다. phi: 0 아래 .. 1.57 앞 .. 3.14 위 (처음엔 비스듬히 앞-아래)
      let phi = 1.0, rx = 0, slide = 0;
      if (e.weaponId === 1) {                       // 창: 팔을 앞으로 뻗으며 찌른다
        const s = Math.sin(Math.PI * tt);
        phi = 1.0 + 0.5 * s; slide = 0.55 * s;
      } else if (tt > 0) {                          // 검/도끼: 머리 위로 들었다가 앞으로 내려친다
        if (tt < 0.4) { phi = 1.0 + 1.3 * (tt / 0.4); rx = -(phi - 1.0); }
        else { const u = (tt - 0.4) / 0.6; phi = 2.3 - 1.3 * u - 0.9 * Math.sin(Math.PI * u); rx = -(phi - 1.0) + (e.weaponId === 2 ? 1.9 : 1.6) * Math.sin(Math.PI * u); }
      }
      const SH = 0.57, ARM = 0.36, SIDE = 0.17;    // 어깨 높이, 팔 길이, 어깨의 옆 거리
      const sx = rxh * SIDE, sz = rzh * SIDE;       // 어깨 (몸 기준 오른쪽)
      const hx = sx + fxh * ARM * Math.sin(phi), hy = SH - ARM * Math.cos(phi), hz = sz + fzh * ARM * Math.sin(phi);
      e.hand.rotation.set(rx, ry, 0, 'YXZ');
      e.hand.position.set(hx, hy, hz);
      if (e.weaponMesh) e.weaponMesh.position.z = slide;   // 창은 쥔 손 사이로 앞으로 미끄러진다
      e.shoulder.position.set(sx, SH, sz);
      const dx = hx - sx, dy = hy - SH, dz = hz - sz, len = Math.hypot(dx, dy, dz);
      e.sleeve.scale.set(1, len, 1);
      e.sleeve.position.set(sx + dx / 2, SH + dy / 2, sz + dz / 2);
      e.sleeve.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), new THREE.Vector3(dx / len, dy / len, dz / len));
    }
    e.label.visible = id !== myId;
    if (e.hpSprite) e.hpSprite.visible = id !== myId;
  }
  const me = players.get(myId);
  if (me) {
    updateLampLights(me.x, -me.y);
    const indoor = applyFloors(me.x, me.y, me.z - (me.jh || 0));
    if (indoor) roomLight.position.set(me.x, me.z + 2.4, -me.y);   // 지붕 아래: 내 머리 위에서 비춘다
    else if (myZone < 0) roomLight.position.set(0, -100, 0);
    const fx = Math.sin(yaw), fz = Math.cos(yaw);   // 내가 보는 방향 (three 좌표)
    if (!third) {
      camera.position.set(me.x + fx * EYE_FORWARD, me.z + EYE, -me.y + fz * EYE_FORWARD);
      camera.rotation.set(pitch, yaw + Math.PI, 0, 'YXZ');
    } else {
      let back = 3.2;
      // 점프하면 캐릭터가 화면에서 위로 올라갔다 내려와야 한다 (카메라가 같이 뛰면 안 뛰는 것처럼 보인다)
      const groundZ = me.z - (me.jh || 0);   // 카메라는 점프를 따라 오르지 않고 발밑 땅 높이만 따라간다
      if (camZ === null || Math.abs(groundZ - camZ) > 5) camZ = groundZ;
      camZ += (groundZ - camZ) * (1 - Math.exp(-8 * dt));
      back = camBack(me.x, me.y, groundZ, -fx, fz, back);   // 엔진 y = -three z: 카메라는 몸 뒤(-fx, +fz 방향이 엔진 좌표의 뒤쪽)
      camera.position.set(me.x - fx * back, camZ + 2.4 + pitch * -1.5 + (3.2 - back) * 0.45, -me.y - fz * back);   // 좁은 곳에서는 카메라가 가까워지는 만큼 높이 올라가 내려다본다
      if (myZone >= 0) {   // 방 안에서는 카메라가 벽/천장을 뚫지 않게
        const r = rooms[myZone];
        camera.position.x = Math.max(r.cx - r.hw + 0.3, Math.min(r.cx + r.hw - 0.3, camera.position.x));
        camera.position.z = Math.max(-r.cy - r.hd + 0.3, Math.min(-r.cy + r.hd - 0.3, camera.position.z));
        camera.position.y = Math.min(camera.position.y, r.floorZ + r.ceil - 0.2);
      }
      camera.lookAt(me.x, camZ + 1.15, -me.y);
    }
  } else {   // 접속 전: 부지를 천천히 돈다
    const t = performance.now() / 20000;
    camera.position.set(Math.cos(t) * 62, 40, Math.sin(t) * 62 - 6);
    camera.lookAt(0, 0, 0);
  }
  renderer.render(scene, camera);
}
frame();
window.__game = { project: (x, y, z) => new THREE.Vector3(x, y, z).project(camera).y, players, rooms, get zone() { return myZone; }, enter: () => ws.send(JSON.stringify({ type: 'enter' })), look: (y, p) => { yaw = y; pitch = p || 0; }, setTime: (i) => { timeIdx = i; applyTime(); }, lamps, get myId() { return myId; }, get state() { return lastState; } };   // 시험용
