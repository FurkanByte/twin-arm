// TwinArm-5 digital twin — browser side.
// 3D scene: the MuJoCo model's own meshes (from the server, /api/mesh/i), body poses over WebSocket at 30 Hz.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/OrbitControls.js';
import { RoomEnvironment } from 'three/addons/RoomEnvironment.js';

const $ = (s) => document.querySelector(s);
const LVL = ['#2fb34d', '#f5b81c', '#e5322b'];
const SERIES = ['#7aa7ff', '#ff6a13', '#ffd166', '#62d2a2', '#c58bff', '#ff7aa8'];
let MODEL = null, ws = null, last = null, dragging = null, READY = false;

// ---------------------------------------------------------------------------------------------
// 3D scene
// ---------------------------------------------------------------------------------------------
const canvas = $('#scene');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.05;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;

const scene = new THREE.Scene();
const pmrem = new THREE.PMREMGenerator(renderer);
scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;

THREE.Object3D.DEFAULT_UP.set(0, 0, 1);              // MuJoCo: Z up
const camera = new THREE.PerspectiveCamera(38, 1, 0.01, 20);
camera.up.set(0, 0, 1);
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.target.set(0.08, 0, 0.2);
const CAMS = {
  iso: [0.62, -0.62, 0.48], front: [0.0, -0.95, 0.24], side: [0.95, 0.0, 0.24], top: [0.08, -0.001, 1.15],
};
function setCam(name) {
  const p = CAMS[name];
  camera.position.set(p[0], p[1], p[2]);
  controls.target.set(0.08, 0, name === 'top' ? 0 : 0.2);
  controls.update();
}
setCam('iso');

scene.add(new THREE.HemisphereLight(0xdfe8ff, 0x20242a, 0.6));
const sun = new THREE.DirectionalLight(0xffffff, 1.6);
sun.position.set(0.6, -0.4, 1.2);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, { left: -0.5, right: 0.5, top: 0.5, bottom: -0.5, near: 0.1, far: 3 });
sun.shadow.bias = -0.0004;
scene.add(sun);

const ground = new THREE.Mesh(new THREE.CircleGeometry(1.2, 64),
  new THREE.MeshStandardMaterial({ color: 0x1a1f26, roughness: 0.95, metalness: 0 }));
ground.receiveShadow = true;
scene.add(ground);
const grid = new THREE.GridHelper(2, 40, 0x3a4552, 0x262d36);
grid.rotation.x = Math.PI / 2;
grid.position.z = 0.0005;
scene.add(grid);

const bodies = [];          // MuJoCo body id -> THREE.Group
const servoMats = {};       // joint -> material (status color)
const shellMats = [];       // printed-part materials (turn transparent in X-ray view)
let payloadMesh = null;

function resize() {
  const r = canvas.parentElement.getBoundingClientRect();
  renderer.setSize(r.width, r.height, false);
  camera.aspect = r.width / Math.max(r.height, 1);
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(canvas.parentElement);

const matCache = new Map();
function material(g) {
  if (g.servo) {
    const m = new THREE.MeshStandardMaterial({ color: 0x1b1c1f, roughness: 0.45, metalness: 0.1, emissive: 0x000000 });
    servoMats[g.servo] = m;
    return m;
  }
  const key = g.rgba.join(',') + (g.metal ? 'm' : '');
  if (!matCache.has(key)) {
    const [r, gg, b, a] = g.rgba;
    const m = new THREE.MeshStandardMaterial({
      color: new THREE.Color(r, gg, b).convertSRGBToLinear(),
      roughness: g.metal ? 0.32 : 0.62, metalness: g.metal ? 0.85 : 0.04,
      transparent: a < 1, opacity: a,
    });
    // printed parts (PETG): dark anthracite / orange
    if (!g.metal && (Math.abs(r - 0.169) < 0.02 || Math.abs(r - 1.0) < 0.01 && gg < 0.5)) {
      m.userData.shell = true;
      shellMats.push(m);
    }
    matCache.set(key, m);
  }
  return matCache.get(key);
}

async function loadMesh(i) {
  const buf = await (await fetch(`/api/mesh/${i}`)).arrayBuffer();
  const h = new Uint32Array(buf, 0, 2);
  const nv = h[0], nf = h[1];
  const pos = new Float32Array(buf, 8, nv * 3);
  const idx = new Uint32Array(buf, 8 + nv * 12, nf * 3);
  let geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  geo.setIndex(new THREE.BufferAttribute(idx, 1));
  geo = geo.toNonIndexed();        // sharp-edged CAD look (flat face normals)
  geo.computeVertexNormals();
  return geo;
}

async function buildScene() {
  MODEL = await (await fetch('/api/model')).json();
  const geos = await Promise.all([...Array(MODEL.nmesh).keys()].map(loadMesh));
  for (let b = 0; b < MODEL.bodies.length; b++) {
    const grp = new THREE.Group();
    grp.matrixAutoUpdate = true;
    scene.add(grp);
    bodies.push(grp);
  }
  for (const g of MODEL.geoms) {
    const geo = g.type === 'mesh' ? geos[g.mesh] : new THREE.SphereGeometry(g.size[0], 32, 16);
    const mat = g.payload ? new THREE.MeshStandardMaterial({ color: 0x3aa0ff, roughness: 0.3, metalness: 0.2 }) : material(g);
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.set(...g.pos);
    mesh.quaternion.set(g.quat[1], g.quat[2], g.quat[3], g.quat[0]);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    bodies[g.body].add(mesh);
    if (g.payload) { payloadMesh = mesh; mesh.visible = false; }
  }
  $('#loading').hidden = true;
  buildPanels();
}

function applyPoses(st) {
  st.bodies.forEach((p, b) => {
    const grp = bodies[b];
    if (!grp) return;
    grp.position.set(p[0], p[1], p[2]);
    grp.quaternion.set(p[4], p[5], p[6], p[3]);
  });
  if (payloadMesh) payloadMesh.visible = st.payload > 0;
}

let t0 = performance.now();
function servoColors() {
  if (!last) return;
  const t = (performance.now() - t0) / 1000;
  for (const [j, m] of Object.entries(servoMats)) {
    const lvl = last.joints[j].lvl;
    const c = new THREE.Color(LVL[lvl]);
    m.color.copy(c).multiplyScalar(0.55);
    m.emissive.copy(c);
    m.emissiveIntensity = lvl === 2 ? 0.55 + 0.45 * Math.sin(t * 8) : lvl === 1 ? 0.45 : 0.3;
  }
}

function loop() {
  controls.update();
  servoColors();
  renderer.render(scene, camera);
  requestAnimationFrame(loop);
}

document.querySelectorAll('[data-cam]').forEach((b) => b.onclick = () => setCam(b.dataset.cam));
$('#xray').onchange = (e) => {
  for (const m of shellMats) {
    m.transparent = e.target.checked;
    m.opacity = e.target.checked ? 0.18 : 1;
    m.depthWrite = !e.target.checked;
    m.needsUpdate = true;
  }
};

// ---------------------------------------------------------------------------------------------
// WebSocket
// ---------------------------------------------------------------------------------------------
function send(o) { if (ws && ws.readyState === 1) ws.send(JSON.stringify(o)); }
function connect() {
  ws = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`);
  ws.onmessage = (e) => {
    const st = JSON.parse(e.data);
    if (st.type !== 'state') return;
    last = st;
    applyPoses(st);
    record(st);
  };
  ws.onclose = () => {
    const c = $('#conn');
    c.className = 'pill off';
    c.textContent = 'no server connection';
    setTimeout(connect, 1500);
  };
}

// ---------------------------------------------------------------------------------------------
// Panel
// ---------------------------------------------------------------------------------------------
const view = (() => { try { return localStorage.getItem('twin.view') || 'basic'; } catch { return 'basic'; } })();
setView(view);
function setView(v) {
  document.body.dataset.view = v;
  document.querySelectorAll('#viewSeg button').forEach((b) => b.classList.toggle('on', b.dataset.v === v));
  try { localStorage.setItem('twin.view', v); } catch {}
  requestAnimationFrame(resize);
}
document.querySelectorAll('#viewSeg button').forEach((b) => b.onclick = () => setView(b.dataset.v));

function segBind(sel, attr, fn) {
  document.querySelectorAll(`${sel} button`).forEach((b) => b.onclick = () => fn(b.dataset[attr]));
}
function segSet(sel, attr, val) {
  document.querySelectorAll(`${sel} button`).forEach((b) => b.classList.toggle('on', String(b.dataset[attr]) === String(val)));
}
segBind('#payloadSeg', 'g', (g) => send({ cmd: 'payload', g: +g }));
segBind('#warpSeg', 'x', (x) => send({ cmd: 'warp', x: +x }));
segBind('#modeSeg', 'm', (m) => send({ cmd: 'mode', mode: m, port: $('#portSel').value || undefined }));
$('#coolBtn').onclick = () => send({ cmd: 'cool' });
$('#demoBtn').onclick = () => send({ cmd: 'demo', on: !(last && last.demo !== null) });
$('#stopBtn').onclick = () => send({ cmd: last && last.stopped ? 'resume' : 'stop' });

async function refreshPorts() {
  const ports = await (await fetch('/api/ports')).json();
  const sel = $('#portSel');
  const cur = sel.value;
  sel.innerHTML = '<option value="">select port</option>' +
    ports.map((p) => `<option value="${p.device}">${p.device} — ${p.desc}</option>`).join('');
  sel.value = cur;
}
$('#portRefresh').onclick = refreshPorts;
$('#portSel').onchange = (e) => { if (e.target.value) send({ cmd: 'mode', mode: 'live', port: e.target.value }); };

const el = {};
function buildPanels() {
  $('#poses').innerHTML = MODEL.poses.map((p) => `<button data-pose="${p.id}">${p.name}</button>`).join('');
  document.querySelectorAll('[data-pose]').forEach((b) => b.onclick = () => send({ cmd: 'pose', name: b.dataset.pose }));

  const L = MODEL.limits;
  $('#jgrid').innerHTML = MODEL.joints.map((j, k) => `
    <div class="jcard" id="jc-${j.id}">
      <div class="jh"><b>J${k + 1} ${j.name}</b><i class="dot"></i></div>
      <small style="color:var(--dim);font-size:11px">${j.servo}</small>
      <div class="ang">0°</div>
      <div class="metric m2"><span>Load</span><b>0%</b><div class="bar"><i></i><u style="left:${L.pctWarn}%"></u></div></div>
      <div class="metric m2"><span>Temp</span><b>25°</b><div class="bar"><i></i><u style="left:${tPos(L.tLimit)}%"></u></div></div>
    </div>`).join('');

  $('#jtable').innerHTML = MODEL.joints.map((j, k) => `
    <div class="jrow" id="jr-${j.id}">
      <div class="jn"><b>J${k + 1} ${j.name}</b><small>${j.servo} · ${j.stall.toFixed(2)} N·m</small>
        <div class="q">0°<small> actual</small></div></div>
      <div class="slide"><input type="range" min="${j.range[0]}" max="${j.range[1]}" step="0.5" data-j="${j.id}"><span>0°</span></div>
      <div class="metric"><span>Torque</span><div class="bar"><i></i><u style="left:${L.pctWarn}%"></u></div><b>0%</b></div>
      <div class="metric"><span>Temp</span><div class="bar"><i></i><u style="left:${tPos(L.tLimit)}%"></u></div><b>25°</b></div>
    </div>`).join('');
  document.querySelectorAll('.slide input').forEach((inp) => {
    let tmr = null;
    inp.addEventListener('pointerdown', () => { dragging = inp.dataset.j; });
    inp.addEventListener('pointerup', () => { setTimeout(() => { dragging = null; }, 300); });
    inp.addEventListener('input', () => {
      dragging = inp.dataset.j;
      inp.nextElementSibling.textContent = `${(+inp.value).toFixed(1)}°`;
      clearTimeout(tmr);
      tmr = setTimeout(() => send({ cmd: 'target', joint: inp.dataset.j, deg: +inp.value }), 25);
    });
    inp.addEventListener('change', () => { setTimeout(() => { dragging = null; }, 300); });
  });
  $('#series').innerHTML = MODEL.joints.map((j, k) => `<span><i style="background:${SERIES[k]}"></i>${j.name}</span>`).join('');
  for (const j of MODEL.joints) {
    el[j.id] = {
      card: $(`#jc-${j.id}`), row: $(`#jr-${j.id}`),
    };
  }
  READY = true;
  refreshPorts();
}
function tPos(T) { return Math.max(0, Math.min(100, (T - 20) / 60 * 100)); }

function setBar(metric, frac, lvl, text) {
  const i = metric.querySelector('.bar > i');
  i.style.width = `${Math.max(0, Math.min(1, frac)) * 100}%`;
  i.className = `l${lvl}`;
  metric.querySelector('b').innerHTML = text;
}

function updatePanels() {
  const st = last;
  if (!st || !READY) return;
  const L = MODEL.limits;
  // connection
  const c = $('#conn');
  if (st.mode === 'live') { c.className = 'pill live'; c.textContent = `LIVE · ${st.port || 'not connected'}`; }
  else { c.className = 'pill sim'; c.textContent = 'SIM · simulation'; }
  segSet('#modeSeg', 'm', st.mode);
  segSet('#payloadSeg', 'g', st.payload);
  segSet('#warpSeg', 'x', st.warp);
  const stop = $('#stopBtn');
  stop.textContent = st.stopped ? 'RESUME' : 'STOP';
  stop.classList.toggle('resume', st.stopped);
  const demoOn = st.demo !== null;
  const db = $('#demoBtn');
  db.textContent = demoOn ? '■ Stop demo' : '▶ Play demo';
  db.classList.toggle('playing', demoOn);
  $('#demoBar').hidden = !demoOn;
  if (demoOn) {
    $('#demoFill').style.width = `${st.demo / st.demoLen * 100}%`;
    $('#demoTime').textContent = `${st.demo.toFixed(1)} / ${st.demoLen} s`;
  }

  // status
  const worst = st.alerts.length ? st.alerts[0].lvl : 0;
  $('#statusCard').className = `card status l${worst}`;
  $('#statusDot').className = `dot l${worst}`;
  $('#statusText').textContent = st.stopped ? 'Stopped: arm is held in place'
    : worst === 0 ? 'All servos normal' : worst === 1 ? 'Caution' : 'At limit: action required';
  $('#alerts').innerHTML = st.alerts.slice(0, 6).map((a) => `<li class="l${a.lvl}">${a.text}</li>`).join('');
  $('#msg').textContent = st.message || '';

  // servos
  MODEL.joints.forEach((jm) => {
    const j = st.joints[jm.id];
    const e = el[jm.id];
    const lvlT = j.pct >= L.pctCrit ? 2 : j.pct >= L.pctWarn ? 1 : 0;
    const lvlC = j.temp >= L.tLimit ? 2 : j.temp >= L.tWarn ? 1 : 0;
    const ttl = j.ttl === null ? '' : j.ttl === 0 ? ' · at limit' : ` · ${j.ttl.toFixed(1)} min`;
    // basic card
    e.card.className = `jcard l${j.lvl}`;
    e.card.querySelector('.dot').className = `dot l${j.lvl}`;
    e.card.querySelector('.ang').textContent = `${j.q.toFixed(1)}°`;
    const mc = e.card.querySelectorAll('.metric');
    setBar(mc[0], j.pct / 100, lvlT, `${j.pct.toFixed(0)}%`);
    setBar(mc[1], tPos(j.temp) / 100, lvlC, `${j.temp.toFixed(1)}°`);
    // advanced row
    e.row.className = `jrow l${j.lvl}`;
    e.row.querySelector('.q').innerHTML = `${j.q.toFixed(1)}°<small> actual</small>`;
    const inp = e.row.querySelector('input');
    if (dragging !== jm.id) { inp.value = j.cmd; inp.nextElementSibling.textContent = `${j.cmd.toFixed(1)}°`; }
    const mr = e.row.querySelectorAll('.metric');
    const exp = st.mode === 'live' ? ` <small>model ${j.exp.toFixed(0)}%</small>` : '';
    setBar(mr[0], j.pct / 100, lvlT, `${j.pct.toFixed(0)}% · ${j.cur.toFixed(2)} A${exp}`);
    setBar(mr[1], tPos(j.temp) / 100, lvlC, `${j.temp.toFixed(1)} °C<small>${ttl}</small>`);
  });

  // power
  const rails = Object.entries(st.rails).map(([k, v]) => `<div><small>${k}</small><b>${v.toFixed(2)} A</b></div>`).join('');
  $('#power').innerHTML = rails + `<div><small>12 V input${st.vin ? ` (${st.vin.toFixed(1)} V)` : ''}</small><b>${st.input12.toFixed(2)} A</b></div>`;

  // event log
  $('#log').innerHTML = st.events.map((e) => `<li class="l${e.lvl}"><time>${e.t}</time><span>${e.text}</span></li>`).join('');
}

// ---------------------------------------------------------------------------------------------
// Charts (last 60 s, 5 Hz)
// ---------------------------------------------------------------------------------------------
const HIST = [];
let lastRec = 0;
function record(st) {
  const now = performance.now();
  if (now - lastRec < 200) return;
  lastRec = now;
  HIST.push(MODEL ? MODEL.joints.map((j) => [st.joints[j.id].pct, st.joints[j.id].temp]) : []);
  if (HIST.length > 300) HIST.shift();
}
function drawChart(cv, idx, lo, hi, lines) {
  const dpr = Math.min(window.devicePixelRatio, 2);
  const w = cv.clientWidth, h = cv.clientHeight;
  if (!w) return;
  if (cv.width !== w * dpr) { cv.width = w * dpr; cv.height = h * dpr; }
  const g = cv.getContext('2d');
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, w, h);
  const y = (v) => h - 6 - (Math.max(lo, Math.min(hi, v)) - lo) / (hi - lo) * (h - 12);
  g.font = '10px Segoe UI, sans-serif';
  for (const [v, col] of lines) {
    g.strokeStyle = col; g.setLineDash([4, 4]); g.lineWidth = 1;
    g.beginPath(); g.moveTo(0, y(v)); g.lineTo(w, y(v)); g.stroke();
    g.fillStyle = col; g.fillText(String(v), 4, y(v) - 3);
  }
  g.setLineDash([]);
  if (!MODEL) return;
  MODEL.joints.forEach((_, k) => {
    g.strokeStyle = SERIES[k]; g.lineWidth = 1.6; g.beginPath();
    HIST.forEach((s, i) => {
      if (!s.length) return;
      const x = w - (HIST.length - 1 - i) * (w / 299);
      i ? g.lineTo(x, y(s[k][idx])) : g.moveTo(x, y(s[k][idx]));
    });
    g.stroke();
  });
}

setInterval(() => {
  updatePanels();
  if (document.body.dataset.view === 'advanced') {
    drawChart($('#chartTq'), 0, 0, 100, [[60, '#f5b81c'], [85, '#e5322b']]);
    drawChart($('#chartT'), 1, 20, 80, [[50, '#f5b81c'], [65, '#e5322b']]);
  }
}, 100);

buildScene().then(() => { resize(); loop(); });
connect();
