// 마을 웹 클라이언트: 내 캐릭터 시점(1인칭)으로 걷고, 다른 접속자는 주민 모델로 보인다.
// 위치/충돌은 서버(village_server)가 정한다. 여기서는 입력을 보내고, 받은 위치를 부드럽게 이어서 그린다.
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';

const $ = (id) => document.getElementById(id);
const EYE = 0.78, EYE_FORWARD = 0.22, NAME_HEIGHT = 1.3;

// ---- 장면 ----
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0xbfe6ff);
scene.fog = new THREE.Fog(0xbfe6ff, 60, 220);
scene.add(new THREE.HemisphereLight(0xffffff, 0x88a070, 1.0));
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
const mapReady = load('/assets/merchen_village.glb').then((g) => scene.add(g.scene)).catch((e) => console.warn('마을 지도를 못 읽었습니다', e));
const modelsReady = Promise.all([1, 2, 3, 4].map((n) => load('/assets/villager_' + n + '.glb').then((g) => { villagerTemplates[n - 1] = g.scene; }).catch(() => {})));

// ---- 접속자 ----
const players = new Map();   // id -> { group, model, label, tx, ty, tz, x, y, z, yaw, name }
let myId = 0, third = false;
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
  scene.add(group);
  const e = { group, model, label, bubble: null, bubbleText: '', lastSid: p.sid || 0, x: p.x, y: p.y, z: p.z, tx: p.x, ty: p.y, tz: p.z, yaw: p.yaw, tyaw: p.yaw, name: p.n };
  players.set(p.id, e);
  return e;
}
function onState(s) {
  lastState = s;
  const seen = new Set();
  for (const p of s.p) {
    seen.add(p.id);
    const e = players.get(p.id) || makePlayer(p);
    e.tx = p.x; e.ty = p.y; e.tz = p.z; e.tyaw = p.yaw;
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
  $('hud').textContent = (me ? me.name : '') + ' · 접속 ' + s.p.length + '명 | WASD 이동 · Shift 달리기 · 마우스 시점 · V 3인칭 · Enter 채팅 · Esc 마우스 풀기';
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
  keys.add(e.code);
  if (['ArrowLeft', 'ArrowRight', 'ArrowUp', 'ArrowDown', 'Space'].includes(e.code)) e.preventDefault();
});
window.addEventListener('keyup', (e) => keys.delete(e.code));
window.addEventListener('blur', () => keys.clear());
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
  await Promise.all([mapReady, modelsReady]);
  ws = new WebSocket((location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host);
  ws.onopen = () => ws.send(JSON.stringify({ type: 'join', name }));
  ws.onmessage = (ev) => {
    const m = JSON.parse(ev.data);
    if (m.type === 'welcome') {
      myId = m.id;
      $('join').style.display = 'none';
      $('hud').hidden = false; $('hint').hidden = false; $('cross').hidden = false;
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
    e.group.position.set(e.x, e.z, -e.y);   // 엔진(x, y, 높이 z) -> three(x, 높이, -y)
    e.model.rotation.y = id === myId ? yaw : e.yaw;
    e.model.visible = !(id === myId && !third);
    e.label.visible = id !== myId;
    if (e.bubble) e.bubble.visible = id !== myId || third;   // 내 말풍선은 1인칭에서는 내 눈앞이라 숨긴다 (왼쪽 아래 대화 기록에는 보인다)
  }
  const me = players.get(myId);
  if (me) {
    const fx = Math.sin(yaw), fz = Math.cos(yaw);   // 내가 보는 방향 (three 좌표)
    if (!third) {
      camera.position.set(me.x + fx * EYE_FORWARD, me.z + EYE, -me.y + fz * EYE_FORWARD);
      camera.rotation.set(pitch, yaw + Math.PI, 0, 'YXZ');
    } else {
      const back = 3.2;
      camera.position.set(me.x - fx * back, me.z + 1.7 + pitch * -1.5, -me.y - fz * back);
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
window.__game = { players, get myId() { return myId; }, get state() { return lastState; } };   // 시험용
