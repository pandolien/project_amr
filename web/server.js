// 마을 웹 접속 게이트웨이: 브라우저(WebSocket) <-> village_server(C++, 표준 입출력)
//
//   cd web && npm install && node server.js
//   node server.js --port 8080 --max 30 --server ../build/bin/village_server --data ../Data/Object
//   node server.js --connect 127.0.0.1:9000        (엔진과 연동: simul_main --serve 9000 이 켜져 있어야 한다)
//
// 두 가지 방식: --connect 면 데스크톱 앱(simul_main)의 엔진이 접속자를 움직이고 이 화면에도 보인다(접속자가 엔진의 오브젝트).
// 아니면 화면 없는 village_server 를 직접 띄운다 (엔진 없음, 웹 접속만 시험할 때).
//
// - 정적 파일: 웹 페이지(public/), three.js(node_modules/three), 마을과 주민 모델(Data/Object) 을 내 준다.
// - 접속: {"type":"join","name":"한솔"} 을 받으면 id 를 주고, village_server 에 join 을 보낸다 (최대 --max 명).
// - 채팅: {"type":"chat","text":"안녕"} (60자, 0.8초에 한 번) -> 그 캐릭터 옆의 말풍선으로 모두에게 보인다.
// - 입력: {"type":"input","fwd":1,"side":0,"run":0,"yaw":1.2} 를 village_server 에 넘긴다 (초당 40개까지만).
// - 50 ms 마다 village_server 에 tick 을 보내고, 돌아온 상태 한 줄을 접속자 모두에게 그대로 보낸다.
'use strict';
const http = require('http');
const fs = require('fs');
const path = require('path');
const { spawn } = require('child_process');
const net = require('net');
const { WebSocketServer } = require('ws');

function arg(name, def) {
  const i = process.argv.indexOf('--' + name);
  return i >= 0 && i + 1 < process.argv.length ? process.argv[i + 1] : def;
}
const PORT = parseInt(arg('port', '8080'), 10);
const HOST = arg('host', '0.0.0.0');
const MAX = Math.max(1, Math.min(60, parseInt(arg('max', '30'), 10)));
const CONNECT = arg('connect', '');   // 예: 127.0.0.1:9000
const DATA = path.resolve(arg('data', path.join(__dirname, '..', 'Data', 'Object')));
const BIN = path.resolve(arg('server', path.join(__dirname, '..', 'build', 'bin', 'village_server' + (process.platform === 'win32' ? '.exe' : ''))));
const TICK_MS = 50;
const PUBLIC = path.join(__dirname, 'public');
const THREE = path.join(__dirname, 'node_modules', 'three');

const TYPES = { '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8', '.mjs': 'text/javascript; charset=utf-8',
                '.glb': 'model/gltf-binary', '.json': 'application/json', '.css': 'text/css; charset=utf-8' };

// 요청 경로를 허용한 폴더 밖으로 나가지 못하게 풀어 준다
function resolveUnder(root, rel) {
  const p = path.normalize(path.join(root, rel));
  return p.startsWith(root + path.sep) ? p : null;
}
function fileFor(url) {
  let u;
  try { u = decodeURIComponent(url.split('?')[0]); } catch (e) { return null; }
  if (u === '/') u = '/index.html';
  if (u.startsWith('/three/')) return resolveUnder(THREE, u.slice(7));
  if (u === '/assets/merchen_village.glb') return path.join(DATA, 'merchen_village.glb');
  const m = /^\/assets\/(villager_[1-4]\.glb)$/.exec(u);
  if (m) return path.join(DATA, 'villagers', m[1]);
  return resolveUnder(PUBLIC, u.slice(1));
}
const httpServer = http.createServer(function (req, res) {
  const file = fileFor(req.url || '/');
  if (!file) { res.writeHead(404); res.end('not found'); return; }
  fs.readFile(file, function (err, data) {
    if (err) { res.writeHead(404); res.end('not found'); return; }
    res.writeHead(200, { 'Content-Type': TYPES[path.extname(file)] || 'application/octet-stream', 'Cache-Control': 'no-cache' });
    res.end(data);
  });
});

// ---- C++ 서버: 표준 입출력의 village_server, 또는 엔진(simul_main --serve)에 TCP 로 붙는다 ----
let child = null;
let sock = null;
let ready = false;
const livePlayers = new Map();   // 접속 중인 id -> 이름 (엔진에 다시 붙을 때 다시 join 해 준다)
function onServerLine(line) {
  if (!ready) { if (line.indexOf('"ready"') >= 0) { ready = true; console.log('village_server 준비됨: ' + line); } return; }
  broadcast(line);
}
function splitLines(onLine) {
  let buf = '';
  return function (d) {
    buf += d.toString('utf8');
    let nl;
    while ((nl = buf.indexOf('\n')) >= 0) {
      const line = buf.slice(0, nl).replace(/\r$/, '');   // 윈도우의 줄바꿈(\r\n)
      buf = buf.slice(nl + 1);
      onLine(line);
    }
  };
}
function startChild() {
  child = spawn(BIN, [DATA], { stdio: ['pipe', 'pipe', 'inherit'] });
  child.on('error', function (e) { console.error('village_server 를 실행할 수 없습니다: ' + BIN + ' (' + e.message + ')'); process.exit(1); });
  child.on('exit', function (code) { console.error('village_server 가 끝났습니다 (' + code + ')'); process.exit(1); });
  child.stdout.on('data', splitLines(onServerLine));
}
function connectEngine() {
  const m = /^(.*):(\d+)$/.exec(CONNECT);
  if (!m) { console.error('--connect 는 host:port 형식이어야 합니다 (예: 127.0.0.1:9000)'); process.exit(1); }
  let warned = false;
  (function attempt() {
    const s = net.connect({ host: m[1], port: parseInt(m[2], 10) });
    s.setNoDelay(true);
    s.on('connect', function () {
      sock = s; ready = true; warned = false;
      console.log('엔진(simul_main)에 연결됨: ' + CONNECT);
      for (const [id, name] of livePlayers) s.write('join ' + id + ' ' + name + '\n');   // 엔진이 다시 켜졌으면 다시 들여보낸다
    });
    s.on('data', splitLines(function (line) { broadcast(line); }));
    s.on('error', function () {});
    s.on('close', function () {
      if (sock === s) { sock = null; ready = false; console.log('엔진과의 연결이 끊겼습니다. 다시 붙는 중…'); }
      else if (!warned) { warned = true; console.log('엔진(simul_main --serve ' + m[2] + ')이 켜지길 기다리는 중…'); }
      setTimeout(attempt, 1000);
    });
  })();
}
function toChild(line) {
  if (CONNECT) { if (sock && ready) sock.write(line + '\n'); }
  else if (child && ready) child.stdin.write(line + '\n');
}

// ---- 웹소켓 ----
const wss = new WebSocketServer({ server: httpServer, maxPayload: 1024 });
const clients = new Map();   // ws -> { id, tokens }
let nextId = 1;
function broadcast(line) {
  for (const [ws, c] of clients) if (c.id && ws.readyState === 1) ws.send(line);
}
function cleanName(s) {
  const t = String(s || '').replace(/[^\p{L}\p{N}_-]/gu, '').slice(0, 12);
  return t || 'guest';
}
wss.on('connection', function (ws) {
  const c = { id: 0, tokens: 40, last: Date.now() };
  clients.set(ws, c);
  ws.on('message', function (data) {
    let m;
    try { m = JSON.parse(data.toString('utf8')); } catch (e) { return; }
    if (!m || typeof m !== 'object') return;
    if (m.type === 'join' && !c.id) {
      let n = 0;
      for (const v of clients.values()) if (v.id) n++;
      if (n >= MAX) { ws.send(JSON.stringify({ type: 'full', max: MAX })); ws.close(); return; }
      c.id = nextId++;
      c.name = cleanName(m.name);
      livePlayers.set(c.id, c.name);
      toChild('join ' + c.id + ' ' + c.name);
      ws.send(JSON.stringify({ type: 'welcome', id: c.id, name: c.name, max: MAX }));
      console.log('접속: ' + c.name + ' (#' + c.id + '), 지금 ' + (n + 1) + '명');
    } else if (m.type === 'chat' && c.id) {
      const now = Date.now();   // 말은 0.8 초에 한 번까지
      if (now - (c.lastChat || 0) < 800) return;
      const text = String(m.text || '').replace(/[\u0000-\u001f\u007f]+/g, ' ').trim().slice(0, 60);
      if (!text) return;
      c.lastChat = now;
      toChild('chat ' + c.id + ' ' + text);
    } else if (m.type === 'input' && c.id) {
      const now = Date.now();   // 초당 40개까지만 받는다
      c.tokens = Math.min(40, c.tokens + (now - c.last) * 0.04);
      c.last = now;
      if (c.tokens < 1) return;
      c.tokens -= 1;
      const f = Math.sign(Number(m.fwd) || 0), s = Math.sign(Number(m.side) || 0), r = m.run ? 1 : 0, y = Number(m.yaw);
      if (!isFinite(y)) return;
      toChild('input ' + c.id + ' ' + f + ' ' + s + ' ' + r + ' ' + y.toFixed(4));
    }
  });
  ws.on('close', function () {
    if (c.id) { livePlayers.delete(c.id); toChild('leave ' + c.id); console.log('퇴장: ' + c.name + ' (#' + c.id + ')'); }
    clients.delete(ws);
  });
  ws.on('error', function () {});
});

if (CONNECT) connectEngine();
else {
  startChild();   // 시계는 이쪽이 쥔다: 50 ms 마다 tick
  setInterval(function () { toChild('tick ' + (TICK_MS / 1000)); }, TICK_MS);
}
httpServer.listen(PORT, HOST, function () {
  console.log('마을 웹 서버: http://localhost:' + PORT + '  (같은 네트워크: http://<이 컴퓨터 주소>:' + PORT + ', 최대 ' + MAX + '명)');
});
