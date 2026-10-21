// BPX 관리자 화면
'use strict';
const $ = (id) => document.getElementById(id);
const MOTION = { 0: '엎드림', 1: '일어서는 중', 2: '힘 풀림', 3: '앉는 중', 6: '서 있음 (주행 가능)' };
const MODE = { idle: '대기', standing: '일어서는 중', ready: '속도 명령 중', sitting: '앉는 중' };
const CM = { 0: '알 수 없음', 1: '앱/리모컨', 2: 'SDK(이 시스템)' };
const GAIT = { 0: '걷기', 3: '두 발', 4: '뒤집기', 6: '보행 변형', 7: '자세 추종', 8: '달리기', 12: '점프' };
const LEGS = ['앞왼(FL)', '앞오(FR)', '뒤왼(HL)', '뒤오(HR)'], JNT = ['엉덩이 roll', '엉덩이 pitch', '무릎'];
const TEMP_WARN = 60, TEMP_HOT = 70;   // ℃ (제조사 한계 미확인 -> 보수적으로)
let view_ = 'ctrl', HIST = [], SYS = null;

// ---------------- 탭 ----------------
document.querySelectorAll('.tab').forEach((t) => t.onclick = () => {
  view_ = t.dataset.view;
  document.querySelectorAll('.tab').forEach((x) => x.classList.toggle('on', x === t));
  for (const v of ['ctrl', 'diag', 'sys']) $('view-' + v).hidden = v !== view_;
  if (view_ === 'ctrl') drawMap(); else if (view_ === 'diag') { renderDiag(); drawCharts(); } else renderSys();
});

let ws = null, S = null, mapImg = null, mapInfo = null;
let canControl = false, motionPending = 0;

// ---------------- 서버 연결 ----------------
function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onopen = () => setPill('link', '서버 연결됨', 'ok');
  ws.onclose = () => { setPill('link', '서버 끊김 · 재연결 중', 'bad'); S = null; render(); setTimeout(connect, 1500); };
  ws.onmessage = (e) => {
    const m = JSON.parse(e.data);
    if (m.type === 'state') { S = m; render(); if (view_ === 'diag') renderDiag(); }
    else if (m.type === 'map') loadMap(m);
    else if (m.type === 'history') { HIST = m.rows; if (view_ === 'diag') drawCharts(); }
    else if (m.type === 'system') { SYS = m; if (view_ === 'sys') renderSys(); }
  };
}
const send = (o) => ws && ws.readyState === 1 && ws.send(JSON.stringify(o));
function setPill(id, text, cls) { const el = $(id); el.textContent = text; el.className = 'pill ' + (cls || ''); }

// ---------------- 상태 표시 ----------------
function render() {
  const st = S ? S.status : {};
  const alive = S && S.bridge_alive, conn = alive && st.connected;
  setPill('robot', !S ? '로봇 -' : !alive ? '브리지 없음' : conn ? '로봇 연결됨' : '로봇 미연결', !S ? '' : conn ? 'ok' : 'bad');

  $('s-motion').textContent = conn ? (MOTION[st.motion_state] ?? st.motion_state) : '-';
  $('s-gait').textContent = conn && st.gait != null ? (GAIT[st.gait] ?? st.gait) : '-';
  const mt = S && S.diag && S.diag.motor_temp;
  $('hot').hidden = !(conn && mt && Math.max(...mt) >= TEMP_HOT);
  $('s-mode').textContent = alive ? (MODE[st.mode] ?? st.mode) : '-';
  const cur = st.battery_current != null ? ` · ${st.battery_current > 0 ? '+' : ''}${st.battery_current.toFixed(2)} A` : '';
  $('s-batt').textContent = st.battery != null ? `${st.battery}%${cur}${st.charging ? ' (충전 중)' : ''}` : '-';
  $('charging').hidden = !(conn && st.charging);
  $('s-temp').textContent = st.motor_temp_max != null ? `${st.motor_temp_max} ℃` : '-';
  const o = S && S.odom;
  $('s-vel').textContent = o && o.vx != null ? `${o.vx} / ${o.vy} m/s · ${o.wz} rad/s` : '-';
  const p = S && S.pose;
  $('s-pose').textContent = p ? `${p[1].toFixed(2)}, ${p[2].toFixed(2)} · ${(p[3] * 180 / Math.PI).toFixed(0)}° (${p[0]})` : '-';
  $('s-cm').textContent = st.control_mode == null ? '미제공' : (CM[st.control_mode] ?? st.control_mode);
  $('frame').textContent = S && S.path_frame ? `(${S.path_frame} 좌표)` : '';
  $('goal-state').textContent = S && S.goal_state ? `목표: ${S.goal_state}` : '';

  if (Date.now() > motionPending) $('motion').checked = !!st.allow_motion;
  const could = canControl;
  canControl = !!(conn && st.allow_motion);
  if (could !== canControl) { if (!canControl && joyTimer) stopJoy(); drawJoy(); }
  for (const id of ['b-stand', 'b-sit', 'b-damp', 'estop']) $(id).disabled = !canControl;
  $('b-zero').disabled = !(canControl && st.motion_state === 0 && st.mode === 'idle');
  for (const id of ['b-walk', 'b-run']) $(id).disabled = !(canControl && st.mode === 'ready');
  $('b-walk').classList.toggle('on', conn && st.gait === 0 && st.motion_state === 6);
  $('b-run').classList.toggle('on', conn && st.gait === 8);
  if (st.charging) { $('b-stand').disabled = true; $('b-stand').title = '충전 중에는 로봇이 일어서지 않습니다. 충전기를 빼세요'; }
  else $('b-stand').title = '';
  $('motion').disabled = !conn;
  $('motion-hint').textContent = !conn ? '로봇이 연결되어야 조종할 수 있습니다.'
    : st.allow_motion ? '조종 모드 켜짐: 버튼과 조이스틱이 로봇에 전달됩니다.' : '조종 모드를 켜야 동작 버튼과 조종이 전달됩니다.';

  const ev = $('events'); ev.innerHTML = '';
  for (const e of (S ? S.events : []).slice().reverse()) {
    const li = document.createElement('li');
    li.textContent = `${e.t}  ${e.text}`;
    if (e.ok === true) li.className = 'ok'; else if (e.ok === false) li.className = 'bad';
    ev.appendChild(li);
  }
  drawMap();
}

// ---------------- 동작 버튼 ----------------
document.querySelectorAll('[data-act]').forEach((b) => b.onclick = () => send({ type: 'action', name: b.dataset.act }));
$('estop').onclick = () => { stopJoy(); send({ type: 'action', name: 'damping' }); };
document.querySelectorAll('[data-gait]').forEach((b) => b.onclick = () => {
  if (b.dataset.gait === 'run' && !confirm('달리기 보행으로 바꿉니다. 속도 상한(0.4 m/s)은 그대로지만 움직임이 커질 수 있습니다.\n주변이 비어 있는지 확인했나요?')) return;
  send({ type: 'action', name: b.dataset.gait });
});
$('b-restart').onclick = () => {
  if (confirm('브리지를 재시작합니다 (웹·Foxglove 도 함께 재시작, 약 10초).\n조종 모드는 꺼진 읽기 전용으로 다시 시작합니다. 계속할까요?')) send({ type: 'restart_bridge' });
};
$('b-zero').onclick = () => {
  const ok = confirm('관절 영점을 설정합니다 (앱의 Reset Joints 와 같음).\n\n' +
    '네 다리의 발 · 정강이 · 무릎이 모두 바닥에 닿은 표준 엎드림 자세인지 확인하세요.\n' +
    '자세가 틀리면 이후 동작이 비정상이 될 수 있습니다.\n\n계속할까요?');
  if (ok) send({ type: 'action', name: 'zero' });
};
$('motion').onchange = (e) => {
  motionPending = Date.now() + 2000;
  if (!e.target.checked) stopJoy();
  send({ type: 'motion', enable: e.target.checked });
};

// ---------------- 지도 ----------------
const cv = $('map'), ctx = cv.getContext('2d');
const view = { scale: 40, cx: 0, cy: 0 };     // px/m, 화면 중심의 월드 좌표
window.bpxView = view;                         // 화면 시험(헤드리스 캡처)용
let follow = true, goalMode = false, drag = null, goalDraft = null;

function loadMap(m) {
  const raw = atob(m.data), img = new ImageData(m.w, m.h);
  for (let i = 0; i < m.w * m.h; i++) {
    const v = raw.charCodeAt(i), c = v === 255 ? 205 : 255 - Math.round(v * 2.55);
    // OccupancyGrid 는 아래쪽 행부터 -> 캔버스는 위에서부터라 뒤집는다
    const row = m.h - 1 - Math.floor(i / m.w), j = (row * m.w + (i % m.w)) * 4;
    img.data[j] = img.data[j + 1] = img.data[j + 2] = c; img.data[j + 3] = 255;
  }
  const off = document.createElement('canvas'); off.width = m.w; off.height = m.h;
  off.getContext('2d').putImageData(img, 0, 0);
  mapImg = off; mapInfo = m;
  if (!S || !S.pose) { view.cx = m.ox + m.w * m.res / 2; view.cy = m.oy + m.h * m.res / 2; }
  drawMap();
}

const w2s = (x, y) => [cv.width / 2 + (x - view.cx) * view.scale, cv.height / 2 - (y - view.cy) * view.scale];
const s2w = (u, v) => [view.cx + (u - cv.width / 2) / view.scale, view.cy - (v - cv.height / 2) / view.scale];

function drawMap() {
  const r = cv.getBoundingClientRect();
  if (cv.width !== Math.round(r.width) || cv.height !== Math.round(r.height)) { cv.width = r.width; cv.height = r.height; }
  const p = S && S.pose;
  if (follow && p) { view.cx = p[1]; view.cy = p[2]; }
  ctx.clearRect(0, 0, cv.width, cv.height);

  if (mapImg && S && S.path_frame === 'map') {
    const [u, v] = w2s(mapInfo.ox, mapInfo.oy + mapInfo.h * mapInfo.res);
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(mapImg, u, v, mapInfo.w * mapInfo.res * view.scale, mapInfo.h * mapInfo.res * view.scale);
  }
  // 1 m 격자
  const step = view.scale < 8 ? 10 : 1;
  const [x0, y1] = s2w(0, 0), [x1, y0] = s2w(cv.width, cv.height);
  ctx.strokeStyle = 'rgba(0,0,0,0.07)'; ctx.lineWidth = 1; ctx.beginPath();
  for (let x = Math.floor(x0 / step) * step; x <= x1; x += step) { const [u] = w2s(x, 0); ctx.moveTo(u, 0); ctx.lineTo(u, cv.height); }
  for (let y = Math.floor(y0 / step) * step; y <= y1; y += step) { const [, v] = w2s(0, y); ctx.moveTo(0, v); ctx.lineTo(cv.width, v); }
  ctx.stroke();
  // 원점 축
  const [ou, ov] = w2s(0, 0);
  ctx.lineWidth = 2; ctx.strokeStyle = '#dc2626'; ctx.beginPath(); ctx.moveTo(ou, ov); ctx.lineTo(ou + 0.5 * view.scale, ov); ctx.stroke();
  ctx.strokeStyle = '#16a34a'; ctx.beginPath(); ctx.moveTo(ou, ov); ctx.lineTo(ou, ov - 0.5 * view.scale); ctx.stroke();
  // 궤적
  if (S && S.path.length > 1) {
    ctx.strokeStyle = '#2563eb'; ctx.lineWidth = 2; ctx.beginPath();
    S.path.forEach(([x, y], i) => { const [u, v] = w2s(x, y); i ? ctx.lineTo(u, v) : ctx.moveTo(u, v); });
    ctx.stroke();
  }
  // 목표
  const g = goalDraft || (S && S.goal);
  if (g) arrow(g[0], g[1], g[2], '#d97706', 0.5);
  if (p) drawDog(p[1], p[2], p[3], S.status || {});
}

// 4족 로봇 (위에서 본 모습). 치수는 URDF 기준(m): 엉덩이 ±0.24 / ±0.056, 서 있을 때 발끝 좌우 ±0.17
function drawDog(x, y, yaw, st) {
  const [u, v] = w2s(x, y);
  const k = Math.max(view.scale, 46 / 0.72);         // 축소해도 몸길이 최소 46 px
  const standing = [1, 6].includes(st.motion_state);
  const ring = !st.connected ? '#9ca3af' : st.charging ? '#d97706' : (st.mode === 'ready' ? '#16a34a' : null);
  const body = st.connected ? '#2b3240' : '#9ca3af', limb = st.connected ? '#4b5563' : '#b8bec7';
  const m = (a) => a * k;
  ctx.save(); ctx.translate(u, v); ctx.rotate(-yaw); ctx.lineCap = 'round'; ctx.lineJoin = 'round';

  // 다리: 엉덩이 -> (서기) 바깥으로 벌린 발 / (엎드림) 몸에 붙인 발
  const hips = [[0.24, 0.07], [0.24, -0.07], [-0.24, 0.07], [-0.24, -0.07]];
  for (const [hx, hy] of hips) {
    const s = Math.sign(hy), fx = standing ? hx + 0.02 * Math.sign(hx) : hx - 0.12 * Math.sign(hx);
    const fy = standing ? 0.17 * s : 0.13 * s, kx = standing ? hx : hx - 0.04 * Math.sign(hx), ky = 0.15 * s;
    ctx.strokeStyle = limb; ctx.lineWidth = Math.max(3, m(0.045));
    ctx.beginPath(); ctx.moveTo(m(hx), -m(hy)); ctx.lineTo(m(kx), -m(ky)); ctx.lineTo(m(fx), -m(fy)); ctx.stroke();
    ctx.fillStyle = '#111827'; ctx.beginPath(); ctx.arc(m(fx), -m(fy), Math.max(2.5, m(0.03)), 0, 7); ctx.fill();
  }
  // 상태 테두리
  if (ring) {
    ctx.strokeStyle = ring; ctx.lineWidth = 3; ctx.globalAlpha = 0.9;
    roundRect(-m(0.40), -m(0.24), m(0.84), m(0.48), m(0.1)); ctx.stroke(); ctx.globalAlpha = 1;
  }
  // 몸통 (torso)
  ctx.fillStyle = body; roundRect(-m(0.30), -m(0.11), m(0.60), m(0.22), m(0.06)); ctx.fill();
  ctx.fillStyle = 'rgba(255,255,255,0.12)'; roundRect(-m(0.24), -m(0.05), m(0.40), m(0.10), m(0.03)); ctx.fill();
  // 머리 + 눈 (앞쪽 = +x)
  ctx.fillStyle = body; roundRect(m(0.27), -m(0.085), m(0.13), m(0.17), m(0.05)); ctx.fill();
  ctx.fillStyle = '#fbbf24';
  for (const ey of [0.04, -0.04]) { ctx.beginPath(); ctx.arc(m(0.37), -m(ey), Math.max(1.8, m(0.017)), 0, 7); ctx.fill(); }
  // 진행 방향 표시
  ctx.strokeStyle = '#fbbf24'; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(m(0.44), -m(0.05)); ctx.lineTo(m(0.52), 0); ctx.lineTo(m(0.44), m(0.05)); ctx.stroke();
  ctx.restore();
}

function roundRect(x, y, w, h, r) {
  ctx.beginPath();
  ctx.moveTo(x + r, y); ctx.arcTo(x + w, y, x + w, y + h, r); ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r); ctx.arcTo(x, y, x + w, y, r); ctx.closePath();
}

function arrow(x, y, yaw, color, len) {
  const [u, v] = w2s(x, y), [u2, v2] = w2s(x + Math.cos(yaw) * len, y + Math.sin(yaw) * len);
  ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 3;
  ctx.beginPath(); ctx.arc(u, v, 6, 0, 7); ctx.fill();
  ctx.beginPath(); ctx.moveTo(u, v); ctx.lineTo(u2, v2); ctx.stroke();
}

cv.addEventListener('wheel', (e) => {
  e.preventDefault();
  view.scale = Math.min(400, Math.max(2, view.scale * (e.deltaY < 0 ? 1.15 : 1 / 1.15)));
  drawMap();
}, { passive: false });
cv.addEventListener('pointerdown', (e) => {
  cv.setPointerCapture(e.pointerId);
  const [x, y] = s2w(e.offsetX, e.offsetY);
  if (goalMode) goalDraft = [x, y, 0];
  else { drag = { u: e.offsetX, v: e.offsetY, cx: view.cx, cy: view.cy }; follow = false; $('b-follow').classList.remove('on'); }
});
cv.addEventListener('pointermove', (e) => {
  if (goalDraft) {
    const [x, y] = s2w(e.offsetX, e.offsetY);
    if (Math.hypot(x - goalDraft[0], y - goalDraft[1]) > 0.05) goalDraft[2] = Math.atan2(y - goalDraft[1], x - goalDraft[0]);
    drawMap();
  } else if (drag) {
    view.cx = drag.cx - (e.offsetX - drag.u) / view.scale; view.cy = drag.cy + (e.offsetY - drag.v) / view.scale; drawMap();
  }
});
cv.addEventListener('pointerup', () => {
  if (goalDraft) {
    send({ type: 'goal', x: goalDraft[0], y: goalDraft[1], yaw: goalDraft[2] });
    goalDraft = null; goalMode = false; $('b-goal').classList.remove('on'); cv.classList.remove('goal');
  }
  drag = null;
});
$('b-goal').onclick = () => { goalMode = !goalMode; $('b-goal').classList.toggle('on', goalMode); cv.classList.toggle('goal', goalMode); };
$('b-cancel').onclick = () => send({ type: 'cancel_goal' });
$('b-follow').onclick = () => { follow = !follow; $('b-follow').classList.toggle('on', follow); drawMap(); };
window.addEventListener('resize', drawMap);

// ---------------- 조이스틱 / 키보드 ----------------
const jc = $('joy'), jx = jc.getContext('2d');
const R = jc.width / 2, KNOB = 34;
let stick = null;              // [-1..1, -1..1] (위 = +)
const keys = new Set();
let joyTimer = null;

function drawJoy() {
  jx.clearRect(0, 0, jc.width, jc.height);
  jx.strokeStyle = '#cbd5e1'; jx.lineWidth = 2;
  jx.beginPath(); jx.arc(R, R, R - KNOB / 2, 0, 7); jx.stroke();
  jx.beginPath(); jx.moveTo(R, 12); jx.lineTo(R, jc.height - 12); jx.moveTo(12, R); jx.lineTo(jc.width - 12, R); jx.stroke();
  const [a, b] = stick || keyStick() || (gp.live ? [-(gp.wz || gp.vy), gp.vx] : [0, 0]);
  jx.fillStyle = canControl ? '#2563eb' : '#9ca3af';
  jx.beginPath(); jx.arc(R + a * (R - KNOB), R - b * (R - KNOB), KNOB / 1.4, 0, 7); jx.fill();
}
function keyStick() {
  if (!keys.size) return null;
  const f = (keys.has('w') ? 1 : 0) - (keys.has('s') ? 1 : 0), t = (keys.has('d') ? 1 : 0) - (keys.has('a') ? 1 : 0);
  return [t, f];
}
function command() {
  const sp = $('speed').value / 100, mv = S ? S.max_v : [0.4, 0.25, 1.0];
  if (gp.live) return { vx: gp.vx * mv[0] * sp, vy: gp.vy * mv[1] * sp, wz: gp.wz * mv[2] * sp };
  const [a, b] = stick || keyStick() || [0, 0];
  const side = (keys.has('q') ? 1 : 0) - (keys.has('e') ? 1 : 0);
  const strafe = $('strafe').checked;
  return {
    vx: b * mv[0] * sp,
    vy: (strafe ? -a : side) * mv[1] * sp,
    wz: (strafe ? 0 : -a) * mv[2] * sp,
  };
}
function startJoy() {
  if (joyTimer) return;
  joyTimer = setInterval(() => {
    if (!canControl) { stopJoy(); return; }
    const c = command();
    send({ type: 'teleop', ...c });
    $('joy-out').textContent = `전후 ${c.vx.toFixed(2)} · 좌우 ${c.vy.toFixed(2)} m/s · 회전 ${c.wz.toFixed(2)} rad/s`;
  }, 100);
}
function stopJoy() {
  stick = null; keys.clear();
  if (joyTimer) { clearInterval(joyTimer); joyTimer = null; }
  send({ type: 'teleop', vx: 0, vy: 0, wz: 0 });   // 한 번 0 을 보내고 송신 중단 (Nav2 명령을 덮지 않도록)
  $('joy-out').textContent = '정지'; drawJoy();
}
function stickFrom(e) {
  let a = (e.offsetX - R) / (R - KNOB), b = -(e.offsetY - R) / (R - KNOB);
  const n = Math.hypot(a, b); if (n > 1) { a /= n; b /= n; }
  return [Math.abs(a) < 0.08 ? 0 : a, Math.abs(b) < 0.08 ? 0 : b];
}
jc.addEventListener('pointerdown', (e) => { if (!canControl) return; jc.setPointerCapture(e.pointerId); stick = stickFrom(e); startJoy(); drawJoy(); });
jc.addEventListener('pointermove', (e) => { if (stick) { stick = stickFrom(e); drawJoy(); } });
for (const ev of ['pointerup', 'pointercancel', 'lostpointercapture']) jc.addEventListener(ev, () => { if (stick) stopJoy(); });

window.addEventListener('keydown', (e) => {
  const k = e.key.toLowerCase();
  if (!'wasdqe'.includes(k) || e.target.tagName === 'INPUT' || !canControl) return;
  keys.add(k); startJoy(); drawJoy();
});
window.addEventListener('keyup', (e) => {
  keys.delete(e.key.toLowerCase());
  if (!keys.size && !stick && !gp.live && joyTimer) stopJoy(); else drawJoy();
});
window.addEventListener('blur', () => { if (joyTimer) stopJoy(); });   // 창을 벗어나면 정지
$('speed').oninput = () => { $('speed-v').textContent = $('speed').value + '%'; };

// ---------------- 게임패드 (표준 매핑: Xbox 배치) ----------------
// LB(4)/RB(5) 를 누르고 있는 동안만 주행 (데드맨). 떼면 즉시 정지. B(1) = 비상 정지
const gp = { live: false, vx: 0, vy: 0, wz: 0, prev: [] };
const DZ = 0.12, dz = (v) => (Math.abs(v) < DZ ? 0 : (v - Math.sign(v) * DZ) / (1 - DZ));
function pollGamepad() {
  const pad = [...(navigator.getGamepads ? navigator.getGamepads() : [])].find((p) => p && p.connected);
  const el = $('gp-status');
  if (!pad) { if (gp.live) { gp.live = false; stopJoy(); } el.className = 'muted'; el.textContent = '게임패드 없음 (연결 후 아무 버튼이나 누르세요)'; return; }
  const btn = (i) => !!(pad.buttons[i] && pad.buttons[i].pressed);
  const edge = (i) => btn(i) && !gp.prev[i];
  const dead = btn(4) || btn(5);
  if (edge(1)) { stopJoy(); send({ type: 'action', name: 'damping' }); }           // B: 비상 정지
  if (dead && canControl && edge(3)) send({ type: 'action', name: 'stand' });      // LB/RB + Y
  if (dead && canControl && edge(0)) send({ type: 'action', name: 'sit' });        // LB/RB + A
  gp.vx = -dz(pad.axes[1] || 0); gp.vy = -dz(pad.axes[0] || 0); gp.wz = -dz(pad.axes[2] || 0);
  const wasLive = gp.live;
  gp.live = dead && canControl;
  if (gp.live && !wasLive) startJoy();
  if (!gp.live && wasLive) stopJoy();
  if (gp.live) drawJoy();
  gp.prev = pad.buttons.map((b) => b.pressed);
  el.className = gp.live ? 'live' : 'on';
  el.textContent = `${pad.id.slice(0, 40)} · ${gp.live ? '주행 허용 중 (LB/RB)' : canControl ? 'LB/RB 를 누르면 주행' : '조종 모드 꺼짐'}`;
}
window.addEventListener('gamepaddisconnected', () => { if (gp.live) { gp.live = false; stopJoy(); } });
setInterval(pollGamepad, 50);

// ---------------- 진단 탭 ----------------
const deg = (r) => (r * 180 / Math.PI).toFixed(1);
const wrapDeg = (r) => deg(Math.atan2(Math.sin(r), Math.cos(r)));   // -180~180°
const tcls = (t) => (t == null ? '' : t >= TEMP_HOT ? 'hot' : t >= TEMP_WARN ? 'warm' : '');
function renderDiag() {
  const j = (S && S.joints) || {}, d = (S && S.diag) || {}, st = (S && S.status) || {};
  let h = '<tr><th></th>' + JNT.map((n) => `<th>${n}</th>`).join('') + '</tr>';
  LEGS.forEach((leg, i) => {
    h += `<tr><th>${leg}</th>`;
    for (let k = 0; k < 3; k++) {
      const p = j.pos && j.pos[3 * i + k], v = j.vel && j.vel[3 * i + k];
      h += `<td>${p != null ? deg(p) + '°' : '-'}<br><span class="muted">${v != null ? v.toFixed(2) : '-'}</span></td>`;
    }
    h += '</tr>';
  });
  $('t-joint').innerHTML = h;
  h = '<tr><th></th>' + JNT.map((n) => `<th>${n}</th>`).join('') + '</tr>';
  LEGS.forEach((leg, i) => {
    h += `<tr><th>${leg}</th>`;
    for (let k = 0; k < 3; k++) {
      const m = d.motor_temp && d.motor_temp[3 * i + k], r = d.driver_temp && d.driver_temp[3 * i + k];
      h += `<td class="${tcls(Math.max(m ?? -1, r ?? -1))}">${m ?? '-'} / ${r ?? '-'}</td>`;
    }
    h += '</tr>';
  });
  $('t-temp').innerHTML = h;
  const rpy = d.imu_rpy, mv = d.max_vel;
  const info = [
    ['펌웨어', d.firmware || '-'],
    ['동작 상태 (이전)', `${MOTION[st.motion_state] ?? st.motion_state ?? '-'} (${MOTION[d.last_motion_state] ?? d.last_motion_state ?? '-'})`],
    ['보행 (세부)', `${GAIT[d.gait] ?? d.gait ?? '-'} (${d.sub_gait ?? '-'})`],
    ['IMU 자세', rpy ? `roll ${deg(rpy[0])}° · pitch ${deg(rpy[1])}° · yaw ${wrapDeg(rpy[2])}°` : '-'],
    ['로봇 최대 속도', mv ? `${mv[0]} / ${mv[1]} m/s · ${mv[2]} rad/s${mv.every((x) => x === 0) ? ' (잠김)' : ''}` : '-'],
    ['배터리', st.battery != null ? `${st.battery}% · ${st.battery_current != null ? st.battery_current.toFixed(2) + ' A' : '-'}${st.charging ? ' (충전 중)' : ''}` : '-'],
  ];
  $('d-info').innerHTML = info.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join('');
}

function chart(id, col, color, fixed) {
  const c = $(id), x = c.getContext('2d'), r = c.getBoundingClientRect();
  if (c.width !== Math.round(r.width) || c.height !== Math.round(r.height)) { c.width = r.width; c.height = r.height; }
  x.clearRect(0, 0, c.width, c.height);
  const pts = HIST.filter((row) => row[col] != null);
  x.fillStyle = '#9ca3af'; x.font = '11px sans-serif';
  if (pts.length < 2) { x.fillText('데이터 수집 중…', 8, 18); return; }
  const t0 = pts[0][0], t1 = pts[pts.length - 1][0];
  let lo = Math.min(...pts.map((p) => p[col])), hi = Math.max(...pts.map((p) => p[col]));
  if (fixed) { lo = Math.min(lo, fixed[0]); hi = Math.max(hi, fixed[1]); }
  if (hi - lo < 1e-6) { hi += 1; lo -= 1; }
  const L = 34, R = 6, T = 8, B = 16, W = c.width - L - R, H = c.height - T - B;
  const X = (t) => L + (t1 > t0 ? (t - t0) / (t1 - t0) : 1) * W, Y = (v) => T + (1 - (v - lo) / (hi - lo)) * H;
  x.strokeStyle = '#e5e7eb'; x.lineWidth = 1; x.beginPath();
  for (let i = 0; i <= 2; i++) { const v = lo + (hi - lo) * i / 2; x.moveTo(L, Y(v)); x.lineTo(L + W, Y(v)); x.fillText(v.toFixed(Math.abs(hi - lo) < 5 ? 1 : 0), 2, Y(v) + 4); }
  x.stroke();
  x.fillText(`${Math.round((t1 - t0) / 60)}분`, L + W - 24, c.height - 3);
  x.strokeStyle = color; x.lineWidth = 2; x.beginPath();
  pts.forEach((p, i) => (i ? x.lineTo(X(p[0]), Y(p[col])) : x.moveTo(X(p[0]), Y(p[col]))));
  x.stroke();
  const last = pts[pts.length - 1][col];
  x.fillStyle = color; x.font = 'bold 12px sans-serif'; x.fillText(String(Math.round(last * 100) / 100), L + 4, T + 12);
}
function drawCharts() {
  chart('c-batt', 1, '#2563eb', [0, 100]);
  chart('c-cur', 2, '#7c3aed');
  chart('c-mt', 3, '#dc2626', [20, TEMP_HOT]);
  chart('c-dt', 4, '#d97706', [20, TEMP_HOT]);
}

// ---------------- 시스템 탭 ----------------
const TEMP_NAME = { k10temp: 'CPU', amdgpu: 'GPU', nvme: 'SSD', mt7921_phy0: 'WiFi' };
function bar(label, pct, text) {
  const cls = pct >= 90 ? 'bad' : pct >= 75 ? 'warn' : '';
  return `<div class="bar">${label} <b>${text}</b><div class="track"><div class="fill ${cls}" style="width:${Math.min(100, pct)}%"></div></div></div>`;
}
function renderSys() {
  if (!SYS) return;
  if (SYS.error) { $('sys-info').innerHTML = `<dt>오류</dt><dd>${SYS.error}</dd>`; return; }
  $('sys-info').innerHTML = [['호스트', SYS.host], ['시각', SYS.time], ['가동 시간', `${SYS.uptime_h} 시간`],
    ['부하 (1/5/15분)', SYS.load.join(' / ') + ` (코어 ${SYS.cpu_count})`]].map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join('');
  $('sys-bars').innerHTML = bar('CPU', SYS.cpu_percent, `${SYS.cpu_percent}%`)
    + bar('메모리', SYS.memory.percent, `${SYS.memory.used_gb} / ${SYS.memory.total_gb} GB`)
    + bar('디스크', SYS.disk.percent, `${SYS.disk.used_gb} / ${SYS.disk.total_gb} GB`);
  $('sys-temps').innerHTML = Object.entries(SYS.temps).map(([k, v]) => `<dt>${TEMP_NAME[k] || k}</dt><dd class="${v >= 85 ? 'bad-t' : ''}">${v} ℃</dd>`).join('');
  $('sys-nics').innerHTML = '<tr><th>포트</th><th>용도</th><th>링크</th><th>주소</th></tr>' + SYS.nics.map((n) =>
    `<tr><td>${n.nic}</td><td>${n.role}</td><td class="${n.link ? 'ok-t' : 'bad-t'}">${n.link ? '연결' : '없음'}</td><td>${n.addr || '-'}</td></tr>`).join('');
  $('sys-svc').innerHTML = Object.entries(SYS.services).map(([k, v]) => `<dt>${k}</dt><dd class="${v === 'active' ? 'ok-t' : 'bad-t'}">${v}</dd>`).join('');
  $('sys-nodes').innerHTML = (SYS.ros_nodes || []).map((n) => `<li>${n}</li>`).join('');
}
window.addEventListener('resize', () => { if (view_ === 'diag') drawCharts(); });

drawJoy();
connect();
setInterval(() => { if (view_ === 'ctrl') drawMap(); }, 500);
