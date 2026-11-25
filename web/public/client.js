// 마을 웹 클라이언트: 내 캐릭터 시점(1인칭)으로 걷고, 다른 접속자는 주민 모델로 보인다.
// 위치/충돌은 서버(village_server)가 정한다. 여기서는 입력을 보내고, 받은 위치를 부드럽게 이어서 그린다.
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

const $ = (id) => document.getElementById(id);
const EYE = 0.78, EYE_FORWARD = 0.22, NAME_HEIGHT = 1.3;

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
const mapReady = load('/assets/merchen_village.glb').then((g) => { scene.add(g.scene); setupLamps(g.scene); }).catch((e) => console.warn('마을 지도를 못 읽었습니다', e));
const modelsReady = Promise.all([1, 2, 3, 4].map((n) => load('/assets/villager_' + n + '.glb').then((g) => { villagerTemplates[n - 1] = g.scene; }).catch(() => {})));

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
let myId = 0, third = false, myZoneInit = false, lastZone = -1;
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
function makePlayer(p) {
  const group = new THREE.Group();
  const tpl = villagerTemplates[(p.id > 0 ? p.id - 1 : -p.id - 1) % 4];   // NPC 는 id 가 음수 (-k = k 번째 주민)
  let model;
  if (tpl) model = tpl.clone(true);
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
  scene.add(group);
  const e = { hpSprite, hpCanvas, hpTex, hp: -1, group, model, label, bubble: null, bubbleText: '', lastSid: p.sid || 0, x: p.x, y: p.y, z: p.z, tx: p.x, ty: p.y, tz: p.z, yaw: p.yaw, tyaw: p.yaw, name: p.n };
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
  for (const [id, e] of players) if (!seen.has(id)) { scene.remove(e.group); players.delete(id); }
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
  $('hud').textContent = (me ? me.name : '') + ' · 접속 ' + s.p.length + '명 | WASD 이동 · Shift 달리기 · 마우스 시점 · V 3인칭 · B 문 앞에서 들어가기/나가기 · F/클릭 공격 · N 낮/노을/밤 · Enter 채팅 · Esc 마우스 풀기';
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
  if (e.code === 'KeyV' && !e.repeat) { third = !third; return; }
  if (e.code === 'KeyN' && !e.repeat) { nextTime(); return; }
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
  await Promise.all([mapReady, modelsReady, roomsReady]);
  ws = new WebSocket((location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host);
  ws.onopen = () => ws.send(JSON.stringify({ type: 'join', name }));
  ws.onmessage = (ev) => {
    const m = JSON.parse(ev.data);
    if (m.type === 'welcome') {
      myId = m.id;
      $('join').style.display = 'none';
      $('hud').hidden = false; $('hpbar').hidden = false; $('hint').hidden = false; $('cross').hidden = false;
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
const clock = new THREE.Clock();
function frame() {
  requestAnimationFrame(frame);
  const dt = Math.min(clock.getDelta(), 0.1);
  const k = 1 - Math.exp(-16 * dt);
  if (keys.has('KeyQ') || keys.has('ArrowLeft')) yaw += 1.8 * dt;     // 마우스가 없어도 돌 수 있게
  if (keys.has('KeyE') || keys.has('ArrowRight')) yaw -= 1.8 * dt;
  for (const [id, e] of players) {
    e.x += (e.tx - e.x) * k; e.y += (e.ty - e.y) * k; e.z += (e.tz - e.z) * k;
    let d = e.tyaw - e.yaw;
    while (d > Math.PI) d -= 2 * Math.PI;
    while (d < -Math.PI) d += 2 * Math.PI;
    e.yaw += d * k;
    e.group.visible = id === myId || e.zone === myZone;   // 다른 건물 안/밖에 있는 사람은 안 보인다
    e.group.position.set(e.x, e.z, -e.y);   // 엔진(x, y, 높이 z) -> three(x, 높이, -y)
    e.model.rotation.y = id === myId ? yaw : e.yaw;
    e.model.visible = !(id === myId && !third);
    e.label.visible = id !== myId;
    if (e.hpSprite) e.hpSprite.visible = id !== myId;
    if (e.bubble) e.bubble.visible = id !== myId || third;   // 내 말풍선은 1인칭에서는 내 눈앞이라 숨긴다 (왼쪽 아래 대화 기록에는 보인다)
  }
  const me = players.get(myId);
  if (me) {
    updateLampLights(me.x, -me.y);
    const fx = Math.sin(yaw), fz = Math.cos(yaw);   // 내가 보는 방향 (three 좌표)
    if (!third) {
      camera.position.set(me.x + fx * EYE_FORWARD, me.z + EYE, -me.y + fz * EYE_FORWARD);
      camera.rotation.set(pitch, yaw + Math.PI, 0, 'YXZ');
    } else {
      const back = 3.2;
      camera.position.set(me.x - fx * back, me.z + 1.7 + pitch * -1.5, -me.y - fz * back);
      if (myZone >= 0) {   // 방 안에서는 카메라가 벽/천장을 뚫지 않게
        const r = rooms[myZone];
        camera.position.x = Math.max(r.cx - r.hw + 0.3, Math.min(r.cx + r.hw - 0.3, camera.position.x));
        camera.position.z = Math.max(-r.cy - r.hd + 0.3, Math.min(-r.cy + r.hd - 0.3, camera.position.z));
        camera.position.y = Math.min(camera.position.y, r.floorZ + r.ceil - 0.2);
      }
      camera.lookAt(me.x, me.z + 0.7, -me.y);
    }
  } else {   // 접속 전: 마을을 천천히 돈다
    const t = performance.now() / 20000;
    camera.position.set(Math.cos(t) * 70, 38, Math.sin(t) * 70);
    camera.lookAt(0, 0, 0);
  }
  renderer.render(scene, camera);
}
frame();
window.__game = { players, rooms, get zone() { return myZone; }, enter: () => ws.send(JSON.stringify({ type: 'enter' })), look: (y, p) => { yaw = y; pitch = p || 0; }, setTime: (i) => { timeIdx = i; applyTime(); }, lamps, get myId() { return myId; }, get state() { return lastState; } };   // 시험용
