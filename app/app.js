// NXT Remote - phone controller for a LEGO NXT robot over Web Serial (Bluetooth SPP).
// Motor A drives forward/back; B+C turn D-profile wheels that step the robot sideways.
import { Nxt, SerialTransport, UsbTransport, SPP_UUID, USB_FILTER, PORT_A, PORT_B, PORT_C, SCREEN_W, SCREEN_H, screenPixel, BT_OFF, BT_VISIBLE, BT_CONNECTED, BT_PIN_REQUEST } from './nxt.js';
import { Robot, DEFAULTS, Aborted } from './motion.js';
import { planPath, pathStats } from './path.js';

const APP_VERSION = '0.1.0';   // keep in step with index.html (?v=) and sw.js
const KEEPALIVE_MS = 30000, JOY_MS = 150, LCD_MS = 200, CAL_TURNS = 10;
const JOY_DEAD = 0.15, JOY_SIDE = 0.5, MIN_POWER = 15;

// ------------------------------------------------------------ strings
const STR = {
  en: {
    langBtn: 'עברית', title: 'NXT Remote',
    connect: 'Connect', connecting: 'Connecting…', disconnect: 'Disconnect', usb: 'USB',
    tabDrive: 'Drive', tabPath: 'Path', tabBrick: 'Brick',
    joyHint: 'Up/down drives (motor A). Left/right steps sideways, one B+C turn per step.',
    stepLeft: '◀ Step', stepRight: 'Step ▶', drivePower: 'Drive speed', sidePower: 'Side-step speed',
    undo: 'Undo', clear: 'Clear', run: 'Run path', stopRun: 'Stop path', area: 'Area',
    pathEmpty: 'Draw the route with your finger. The robot starts at the first point, facing up.',
    pathInfo: '{0} moves · forward {1} cm · {2} side steps · ≈{3}',
    pathDone: 'Path finished ✓', pathStep: 'Move {0}/{1}…',
    screen: 'Brick screen', lcdLive: 'Live refresh',
    bt: 'Bluetooth: {0}', btOff: 'off', btOn: 'on', btVisible: 'visible', btHidden: 'hidden', btConnected: 'connected',
    btPin: 'waiting for the passkey - press ●',
    homeTitle: 'Side wheels home',
    homeHint: 'Turn B and C until the flat side of every D-wheel faces the floor, then tap "Set home". The brick counts from where the motors were at power-on, so switch it off with the wheels at home.',
    setHome: 'Set home here', goHome: 'Return home',
    calTitle: 'Calibration',
    calHint: 'Tap "Drive 10 turns", measure how far the robot went, type the centimetres and Save.',
    calDrive: 'Drive 10 turns', save: 'Save', calInfo: 'Motor A: {0} cm per turn',
    invA: 'Reverse driving direction (A)', invSide: 'Reverse sideways direction',
    invB: 'Reverse motor B', invC: 'Reverse motor C',
    deadman: 'Stop by itself if the phone loses the connection',
    infoTitle: 'Brick', firmware: 'Firmware', btAddr: 'Bluetooth address', battery: 'Battery', rename: 'Rename',
    stop: 'STOP',
    hintNoSerial: 'This browser cannot open Bluetooth serial ports. Use Chrome on Android (138 or newer) or Chrome on a computer.',
    hintIos: 'iPhone browsers cannot talk to the NXT (it has classic Bluetooth only). Use an Android phone with Chrome.',
    hintPair: 'First pair the NXT in the phone\'s Bluetooth settings (PIN 1234, then press ● on the brick), then tap Connect.',
    hintFail: 'Could not connect: {0}', hintLost: 'Connection lost. Tap Connect.',
    hintReconnect: 'Connection lost - reconnecting ({0}/3)…', hintConnectFirst: 'Connect to the brick first.',
    hintCalDone: 'Measure how far it went and type the distance in cm.', hintSaved: 'Saved ✓',
    hintHomeSet: 'Home set ✓', credit: 'By Arad & Claude 2026',
  },
  he: {
    langBtn: 'English', title: 'שלט NXT',
    connect: 'התחברות', connecting: 'מתחבר…', disconnect: 'התנתקות', usb: 'USB',
    tabDrive: 'נהיגה', tabPath: 'מסלול', tabBrick: 'בקר',
    joyHint: 'למעלה/למטה - נסיעה (מנוע A). ימינה/שמאלה - צעד הצידה, סיבוב אחד של B+C לכל צעד.',
    stepLeft: '◀ צעד', stepRight: 'צעד ▶', drivePower: 'מהירות נסיעה', sidePower: 'מהירות צעד הצידה',
    undo: 'ביטול', clear: 'ניקוי', run: 'הפעלת מסלול', stopRun: 'עצירת מסלול', area: 'גודל השטח',
    pathEmpty: 'ציירו את המסלול באצבע. הרובוט מתחיל בנקודה הראשונה כשהוא פונה למעלה.',
    pathInfo: '{0} תנועות · קדימה {1} ס"מ · {2} צעדי צד · ≈{3}',
    pathDone: 'המסלול הסתיים ✓', pathStep: 'תנועה {0}/{1}…',
    screen: 'מסך הבקר', lcdLive: 'רענון חי',
    bt: 'בלוטות\': {0}', btOff: 'כבוי', btOn: 'דולק', btVisible: 'גלוי', btHidden: 'מוסתר', btConnected: 'מחובר',
    btPin: 'ממתין לקוד צימוד - לחצו ●',
    homeTitle: 'נקודת אפס לגלגלי הצד',
    homeHint: 'סובבו את B ו-C עד שהצד השטוח של כל גלגלי ה-D פונה לרצפה, ולחצו "קבע כאפס". הבקר סופר מהמקום שבו היו המנועים כשהודלק, לכן כבו אותו כשהגלגלים באפס.',
    setHome: 'קבע כאפס', goHome: 'חזרה לאפס',
    calTitle: 'כיול',
    calHint: 'לחצו "סע 10 סיבובים", מדדו כמה הרובוט נסע, הקלידו בס"מ ולחצו שמירה.',
    calDrive: 'סע 10 סיבובים', save: 'שמירה', calInfo: 'מנוע A: {0} ס"מ לסיבוב',
    invA: 'היפוך כיוון הנסיעה (A)', invSide: 'היפוך כיוון התזוזה הצידה',
    invB: 'היפוך מנוע B', invC: 'היפוך מנוע C',
    deadman: 'עצירה אוטומטית אם הטלפון מאבד את החיבור',
    infoTitle: 'פרטי הבקר', firmware: 'קושחה', btAddr: 'כתובת בלוטות\'', battery: 'סוללה', rename: 'שינוי שם',
    stop: 'עצור',
    hintNoSerial: 'הדפדפן הזה לא יכול לפתוח חיבור בלוטות\' טורי. השתמשו ב-Chrome באנדרואיד (גרסה 138 ומעלה) או ב-Chrome במחשב.',
    hintIos: 'דפדפנים באייפון לא יכולים לדבר עם ה-NXT (יש לו רק בלוטות\' קלאסי). השתמשו בטלפון אנדרואיד עם Chrome.',
    hintPair: 'קודם צמדו את ה-NXT בהגדרות הבלוטות\' של הטלפון (קוד 1234, ואז לחצו ● על הבקר), ואז לחצו "התחברות".',
    hintFail: 'החיבור נכשל: {0}', hintLost: 'החיבור נותק. לחצו "התחברות".',
    hintReconnect: 'החיבור נותק - מתחבר מחדש ({0}/3)…', hintConnectFirst: 'קודם התחברו לבקר.',
    hintCalDone: 'מדדו כמה הרובוט נסע והקלידו את המרחק בס"מ.', hintSaved: 'נשמר ✓',
    hintHomeSet: 'נקודת האפס נקבעה ✓', credit: 'מאת ארד וקלוד 2026',
  },
};

function loadLang() {
  const q = new URLSearchParams(location.search).get('lang');   // ?lang=he for a shareable link
  if (STR[q]) return q;
  try { const l = localStorage.getItem('lang'); if (STR[l]) return l; } catch {}
  return (navigator.language || '').startsWith('he') ? 'he' : 'en';
}
let lang = loadLang();
const t = (key, ...args) => (STR[lang][key] ?? STR.en[key]).replace(/\{(\d)\}/g, (_, i) => args[i]);

// ------------------------------------------------------------ settings
function loadCfg() {
  let saved = {};
  try { saved = JSON.parse(localStorage.getItem('cfg') || '{}'); } catch {}
  return { ...DEFAULTS, area: 200, ...saved };
}
const cfg = loadCfg();
function saveCfg() { try { localStorage.setItem('cfg', JSON.stringify(cfg)); } catch {} }

// ------------------------------------------------------------ state
const $ = id => document.getElementById(id);
const sleep = ms => new Promise(r => setTimeout(r, ms));
let nxt = null, robot = null, transport = null, lastPort = null;
let connState = 'off', userDisconnect = false, hint = null, keepTimer = null, wakeLock = null;
let activeTab = 'drive', brickInfo = null, batteryMv = null;

function setHint(key, ...args) {
  hint = key ? { key, args } : null;
  const el = $('hint');
  el.textContent = key ? t(key, ...args.filter(a => a !== true)) : '';
  el.classList.toggle('err', args.includes(true));
}

function setConn(state) {   // 'off' | 'busy' | 'on'
  connState = state;
  const btn = $('connectBtn');
  btn.textContent = t(state === 'on' ? 'disconnect' : state === 'busy' ? 'connecting' : 'connect');
  btn.className = 'grow ' + (state === 'on' ? 'on' : state === 'busy' ? 'busy' : 'primary');
  btn.disabled = state === 'busy';
  $('dot').className = 'dot ' + state;
  if (state !== 'on') { $('stName').textContent = '—'; $('stBat').textContent = '—'; }
}

async function act(fn) {
  if (!robot || connState !== 'on') { setHint('hintConnectFirst'); return; }
  try { return await fn(); }
  catch (e) { if (!(e instanceof Aborted)) setHint('hintFail', e.message, true); }
}

// ------------------------------------------------------------ connection
async function pickSerialPort() {
  try {
    return await navigator.serial.requestPort({ filters: [{ bluetoothServiceClassId: SPP_UUID }], allowedBluetoothServiceClassIds: [SPP_UUID] });
  } catch (e) {
    if (e.name === 'TypeError') return navigator.serial.requestPort();   // older Chrome: no Bluetooth filters
    throw e;
  }
}

async function connectSerial(port, quiet = false) {
  setConn('busy');
  const tr = new SerialTransport(port);
  try {
    await tr.open();
    await attach(tr);
    lastPort = port;
  } catch (e) {
    await tr.close();
    setConn('off');
    if (!quiet) setHint('hintFail', e.message, true);
    throw e;
  }
}

async function connectUsb() {
  const dev = await navigator.usb.requestDevice({ filters: [USB_FILTER] });
  setConn('busy');
  const tr = new UsbTransport(dev);
  try { await tr.open(); await attach(tr); lastPort = null; }
  catch (e) { await tr.close(); setConn('off'); setHint('hintFail', e.message, true); }
}

async function attach(tr) {
  transport = tr;
  nxt = new Nxt(tr);
  robot = new Robot(nxt, cfg);
  robot.onPos = showPos;
  brickInfo = await nxt.deviceInfo();
  const fw = await nxt.firmware();
  batteryMv = await nxt.battery();
  tr.onclose = onClosed;
  userDisconnect = false;
  setConn('on');
  setHint(null);
  $('stName').textContent = brickInfo.name;
  showBattery();
  $('infoFw').textContent = `${fw.firmware} (protocol ${fw.protocol})`;
  $('infoAddr').textContent = brickInfo.btAddr;
  $('nameInput').value = brickInfo.name;
  clearInterval(keepTimer);
  keepTimer = setInterval(keepAlive, KEEPALIVE_MS);
  requestWakeLock();
  if (activeTab === 'brick') lcdLoop();
}

async function keepAlive() {
  if (connState !== 'on') return;
  try { await nxt.keepAlive(); batteryMv = await nxt.battery(); showBattery(); } catch {}
}

function showBattery() {
  const v = batteryMv ? (batteryMv / 1000).toFixed(2) + ' V' : '—';
  $('stBat').textContent = v;
  $('infoBat').textContent = v;
}

async function onClosed() {
  const wasOn = connState === 'on';
  clearInterval(keepTimer);
  robot = null; nxt = null; transport = null;
  setConn('off');
  releaseWakeLock();
  if (!wasOn || userDisconnect) return;
  // The link dropped by itself (brick switched off, out of range). Try the same port again.
  for (let i = 1; i <= 3 && lastPort && !userDisconnect; i++) {
    setHint('hintReconnect', i);
    await sleep(2000);
    try { await connectSerial(lastPort, true); return; } catch {}
  }
  setHint('hintLost');
}

async function onConnectClick() {
  if (connState === 'on') {
    userDisconnect = true;
    try { await robot?.stop(); } catch {}
    await transport?.close();
    return;
  }
  if (!navigator.serial) { setHint(/iPhone|iPad/.test(navigator.userAgent) ? 'hintIos' : 'hintNoSerial', true); return; }
  try { await connectSerial(await pickSerialPort()); }
  catch (e) { if (e.name === 'NotFoundError') setHint('hintPair'); }
}

async function autoConnect() {
  if (!navigator.serial) { setHint(/iPhone|iPad/.test(navigator.userAgent) ? 'hintIos' : 'hintNoSerial', true); return; }
  const ports = await navigator.serial.getPorts();
  const port = ports.find(p => p.getInfo().bluetoothServiceClassId === SPP_UUID) || ports.find(p => p.getInfo().bluetoothServiceClassId);
  if (!port) { setHint('hintPair'); return; }
  try { await connectSerial(port, true); } catch { setHint('hintLost'); }
}

async function requestWakeLock() {
  try { wakeLock = await navigator.wakeLock?.request('screen'); } catch {}
}
function releaseWakeLock() { try { wakeLock?.release(); } catch {} wakeLock = null; }

// ------------------------------------------------------------ joystick
// Snaps to one axis: the robot drives forward/back or steps sideways, never both.
const joy = $('joy'), knob = $('knob');
let joyPtr = null, joyVec = { x: 0, y: 0 }, joyTimer = null, stepping = false, driving = false;

joy.addEventListener('pointerdown', e => {
  if (joyPtr !== null) return;
  joyPtr = e.pointerId;
  try { joy.setPointerCapture(e.pointerId); } catch {}
  joy.classList.add('active');
  joyMove(e);
  joyTick();
  joyTimer = setInterval(joyTick, JOY_MS);
});
joy.addEventListener('pointermove', e => { if (e.pointerId === joyPtr) joyMove(e); });
for (const ev of ['pointerup', 'pointercancel', 'lostpointercapture']) joy.addEventListener(ev, e => {
  if (e.pointerId !== joyPtr) return;
  joyPtr = null;
  clearInterval(joyTimer);
  joyVec = { x: 0, y: 0 };
  renderKnob();
  joy.classList.remove('active', 'side');
  if (robot) act(() => robot.drive(0));
});

function joyMove(e) {
  const r = joy.getBoundingClientRect();
  const reach = r.width / 2 * 0.75;
  let x = (e.clientX - r.left - r.width / 2) / reach, y = -(e.clientY - r.top - r.height / 2) / reach;
  const m = Math.hypot(x, y);
  if (m > 1) { x /= m; y /= m; }
  if (Math.abs(x) > Math.abs(y)) y = 0; else x = 0;
  joyVec = { x, y };
  renderKnob();
  joy.classList.toggle('side', Math.abs(x) > JOY_SIDE);
}

function renderKnob() {
  knob.style.left = `${50 + joyVec.x * 37.5}%`;
  knob.style.top = `${50 - joyVec.y * 37.5}%`;
}

function joyTick() {
  if (!robot || connState !== 'on' || driving) return;
  const { x, y } = joyVec;
  if (Math.abs(x) > JOY_SIDE) {
    if (!stepping) sideStep(x > 0 ? 1 : -1);
    return;
  }
  if (stepping) return;
  const a = Math.abs(y);
  const power = a < JOY_DEAD ? 0 : Math.sign(y) * (MIN_POWER + (a - JOY_DEAD) / (1 - JOY_DEAD) * (cfg.drivePower - MIN_POWER));
  driving = true;
  act(() => robot.drive(power)).finally(() => { driving = false; });
}

async function sideStep(dir) {
  if (stepping) return;
  stepping = true;
  try { await act(() => robot.sideSteps(dir)); } finally { stepping = false; }
}

// ------------------------------------------------------------ path
const pc = $('pathCanvas'), pctx = pc.getContext('2d');
let strokes = [], drawing = null, plan = null, runStep = -1, running = false;

function toWorld(e) {
  const r = pc.getBoundingClientRect();
  return { x: (e.clientX - r.left) / r.width * cfg.area, y: (1 - (e.clientY - r.top) / r.height) * cfg.area };
}

pc.addEventListener('pointerdown', e => {
  if (running || drawing) return;
  try { pc.setPointerCapture(e.pointerId); } catch {}
  drawing = { id: e.pointerId, pts: [toWorld(e)] };
  strokes.push(drawing.pts);
  replan();
});
pc.addEventListener('pointermove', e => {
  if (!drawing || e.pointerId !== drawing.id) return;
  const p = toWorld(e), last = drawing.pts[drawing.pts.length - 1];
  if (Math.hypot(p.x - last.x, p.y - last.y) < cfg.area / 400) return;
  drawing.pts.push(p);
  replan();
});
for (const ev of ['pointerup', 'pointercancel']) pc.addEventListener(ev, e => {
  if (drawing && e.pointerId === drawing.id) { drawing = null; replan(); }
});

function replan() {
  const pts = strokes.flat();
  plan = pts.length > 1 ? planPath(pts, { stepCm: cfg.stepCm }) : null;
  showPlan();
  requestAnimationFrame(drawPath);
}

function fmtDuration(s) {
  s = Math.round(s);
  const v = s < 60 ? `${s}s` : `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}`;
  return `⁦${v}⁩`;   // LTR isolate: keeps "32s" / "1:05" intact inside Hebrew text
}

function showPlan() {
  if (running) return;
  if (!plan?.moves.length) { $('pathInfo').textContent = t('pathEmpty'); return; }
  const st = pathStats(plan.moves, cfg.stepCm);
  // Rough NXT motor speed under load: ~8 deg/s per % of power.
  const fwdS = st.fwdCm / (cfg.cmPerDegA * 8 * cfg.drivePower);
  const sideS = st.steps * (360 / (8 * cfg.sidePower) + 0.5);
  $('pathInfo').textContent = t('pathInfo', plan.moves.length, Math.round(st.fwdCm), st.steps, fmtDuration(fwdS + sideS + plan.moves.length * 0.3));
}

function drawPath() {
  const dpr = window.devicePixelRatio || 1, w = pc.clientWidth, h = pc.clientHeight;
  if (pc.width !== Math.round(w * dpr)) { pc.width = Math.round(w * dpr); pc.height = Math.round(h * dpr); }
  const ctx = pctx;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  const sx = x => x / cfg.area * w, sy = y => h - y / cfg.area * h;

  // Grid: one line per side lane when it is wide enough to see, bold every 50 cm.
  const lanePx = cfg.stepCm / cfg.area * w;
  for (let cm = 0; cm <= cfg.area; cm += cfg.stepCm) {
    const major = cm % 50 === 0;
    if (!major && lanePx < 5) continue;
    ctx.strokeStyle = major ? '#2c313c' : '#1b1f27';
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(sx(cm), 0); ctx.lineTo(sx(cm), h); ctx.stroke();
    if (major) { ctx.beginPath(); ctx.moveTo(0, sy(cm)); ctx.lineTo(w, sy(cm)); ctx.stroke(); }
  }

  ctx.lineJoin = ctx.lineCap = 'round';
  ctx.strokeStyle = '#8a90a066';
  ctx.lineWidth = 2;
  for (const s of strokes) {
    ctx.beginPath();
    s.forEach((p, i) => i ? ctx.lineTo(sx(p.x), sy(p.y)) : ctx.moveTo(sx(p.x), sy(p.y)));
    ctx.stroke();
  }

  if (!plan) return;
  const sp = plan.snapped;
  ctx.lineWidth = 4;
  for (let i = 1; i < sp.length; i++) {
    ctx.strokeStyle = !running ? '#6aa8ff' : i - 1 < runStep ? '#4cc38a' : i - 1 === runStep ? '#f0b44c' : '#6aa8ff';
    ctx.beginPath(); ctx.moveTo(sx(sp[i - 1].x), sy(sp[i - 1].y)); ctx.lineTo(sx(sp[i].x), sy(sp[i].y)); ctx.stroke();
  }
  // Robot at the start, facing up.
  const s0 = sp[0], rw = Math.max(10, 20 / cfg.area * w);
  ctx.fillStyle = '#e8eaf0';
  ctx.fillRect(sx(s0.x) - rw / 2, sy(s0.y) - rw / 2, rw, rw);
  ctx.fillStyle = '#e5534b';
  ctx.beginPath(); ctx.moveTo(sx(s0.x), sy(s0.y) - rw / 2 - 6); ctx.lineTo(sx(s0.x) - 5, sy(s0.y) - rw / 2); ctx.lineTo(sx(s0.x) + 5, sy(s0.y) - rw / 2); ctx.fill();
  const e = sp[sp.length - 1];
  ctx.fillStyle = '#4cc38a';
  ctx.beginPath(); ctx.arc(sx(e.x), sy(e.y), 6, 0, 2 * Math.PI); ctx.fill();
}

async function runPath() {
  if (running) { await robot?.stop(); return; }
  if (!plan?.moves.length) return;
  if (!robot || connState !== 'on') { setHint('hintConnectFirst'); return; }
  running = true;
  $('runBtn').textContent = t('stopRun');
  const n = plan.moves.length;
  try {
    await robot.runPath(plan.moves, i => {
      runStep = i;
      $('pathInfo').textContent = i < n ? t('pathStep', i + 1, n) : t('pathDone');
      drawPath();
    });
  } catch (e) {
    if (!(e instanceof Aborted)) setHint('hintFail', e.message, true);
  } finally {
    running = false;
    runStep = -1;
    $('runBtn').textContent = t('run');
    drawPath();
  }
}

// ------------------------------------------------------------ brick tab
const lcd = $('lcd'), lctx = lcd.getContext('2d');
const lcdImg = lctx.createImageData(SCREEN_W, SCREEN_H);
let lcdRunning = false;

function drawLcd(buf) {
  const d = lcdImg.data;
  for (let y = 0; y < SCREEN_H; y++) for (let x = 0; x < SCREEN_W; x++) {
    const on = screenPixel(buf, x, y), i = (y * SCREEN_W + x) * 4;
    d[i] = on ? 0x1C : 0xA7; d[i + 1] = on ? 0x24 : 0xB4; d[i + 2] = on ? 0x16 : 0x8F; d[i + 3] = 255;
  }
  lctx.putImageData(lcdImg, 0, 0);
}

async function lcdLoop() {
  if (lcdRunning) return;
  lcdRunning = true;
  try {
    while (connState === 'on' && activeTab === 'brick' && !document.hidden) {
      if ($('lcdLive').checked) {
        drawLcd(await nxt.readScreen());
        showBt((await nxt.uiState()).bt);
      }
      await robot.pos(PORT_B); await robot.pos(PORT_C);
      await sleep(LCD_MS);
    }
  } catch {}
  lcdRunning = false;
}

function showBt(bt) {
  let s;
  if (bt & BT_PIN_REQUEST) s = t('btPin');
  else if (bt & BT_OFF) s = t('btOff');
  else s = [t('btOn'), t(bt & BT_VISIBLE ? 'btVisible' : 'btHidden'), bt & BT_CONNECTED ? t('btConnected') : ''].filter(Boolean).join(', ');
  $('btState').textContent = t('bt', s);
}

function showPos(port, deg) {
  if (port === PORT_A) return;
  let off = ((deg % 360) + 360) % 360;
  if (off > 180) off -= 360;
  $(port === PORT_B ? 'posB' : 'posC').textContent = `${off > 0 ? '+' : ''}${off}°`;
}

// ------------------------------------------------------------ language & tabs
function applyLang() {
  document.documentElement.lang = lang;
  document.documentElement.dir = lang === 'he' ? 'rtl' : 'ltr';
  document.title = t('title');
  for (const el of document.querySelectorAll('[data-t]')) el.textContent = t(el.dataset.t);
  $('langBtn').textContent = t('langBtn');
  $('footer').textContent = `${t('title')} ${APP_VERSION} · ${t('credit')}`;
  $('runBtn').textContent = t(running ? 'stopRun' : 'run');
  setConn(connState);
  if (connState === 'on') { $('stName').textContent = brickInfo.name; showBattery(); }
  if (hint) setHint(hint.key, ...hint.args);
  showPlan();
  showCal();
  try { localStorage.setItem('lang', lang); } catch {}
}

function showTab(name) {
  activeTab = name;
  for (const b of document.querySelectorAll('nav.tabs button')) b.classList.toggle('active', b.dataset.tab === name);
  for (const s of document.querySelectorAll('.tab')) s.hidden = s.id !== `tab-${name}`;
  if (name === 'path') requestAnimationFrame(drawPath);
  if (name === 'brick' && connState === 'on') lcdLoop();
}

function showCal() { $('calInfo').textContent = t('calInfo', (cfg.cmPerDegA * 360).toFixed(2)); }

function bindSlider(id) {
  const el = $(id), out = $(id + 'Val');
  el.value = cfg[id];
  out.textContent = cfg[id] + '%';
  el.addEventListener('input', () => { cfg[id] = +el.value; out.textContent = cfg[id] + '%'; saveCfg(); showPlan(); });
}

// ------------------------------------------------------------ wiring
$('langBtn').addEventListener('click', () => { lang = lang === 'he' ? 'en' : 'he'; applyLang(); });
$('connectBtn').addEventListener('click', onConnectClick);
if (navigator.usb) { $('usbBtn').hidden = false; $('usbBtn').addEventListener('click', () => connectUsb().catch(() => {})); }
for (const b of document.querySelectorAll('nav.tabs button')) b.addEventListener('click', () => showTab(b.dataset.tab));

$('stepL').addEventListener('click', () => sideStep(-1));
$('stepR').addEventListener('click', () => sideStep(1));
$('beepBtn').addEventListener('click', () => act(() => nxt.playTone(880, 250)));
$('stopBtn').addEventListener('click', () => { stepping = false; if (robot) act(() => robot.stop()); });

$('undoBtn').addEventListener('click', () => { if (!running) { strokes.pop(); replan(); } });
$('clearBtn').addEventListener('click', () => { if (!running) { strokes = []; replan(); } });
$('runBtn').addEventListener('click', runPath);
$('area').value = cfg.area;
$('area').addEventListener('change', () => { cfg.area = +$('area').value; saveCfg(); strokes = []; replan(); });

for (const b of document.querySelectorAll('[data-btn]')) b.addEventListener('click', () => act(() => nxt.pressButton(+b.dataset.btn)));
for (const b of document.querySelectorAll('[data-jog]')) b.addEventListener('click', () => {
  const [port, deg] = b.dataset.jog.split(',').map(Number);
  act(() => robot.jog(port, deg));
});
$('setHomeBtn').addEventListener('click', () => act(async () => { await robot.setHome(); setHint('hintHomeSet'); }));
$('goHomeBtn').addEventListener('click', () => act(() => robot.goHome()));
$('calDriveBtn').addEventListener('click', () => act(async () => { await robot.driveTurns(CAL_TURNS); setHint('hintCalDone'); $('calCm').focus(); }));
$('calSaveBtn').addEventListener('click', () => {
  const cm = parseFloat($('calCm').value);
  if (!(cm > 0)) return;
  cfg.cmPerDegA = cm / (CAL_TURNS * 360);
  saveCfg(); showCal(); showPlan(); setHint('hintSaved');
});
for (const el of document.querySelectorAll('[data-cfg]')) {
  el.checked = !!cfg[el.dataset.cfg];
  el.addEventListener('change', () => { cfg[el.dataset.cfg] = el.checked; saveCfg(); });
}
$('renameBtn').addEventListener('click', () => act(async () => {
  const name = $('nameInput').value.trim();
  if (!name) return;
  await nxt.setName(name);
  brickInfo = await nxt.deviceInfo();
  $('stName').textContent = brickInfo.name;
  setHint('hintSaved');
}));
bindSlider('drivePower');
bindSlider('sidePower');

// A hidden page can't steer: stop the robot rather than let it run blind.
document.addEventListener('visibilitychange', () => {
  if (document.hidden) { if (robot) robot.stop().catch(() => {}); }
  else { if (connState === 'on') requestWakeLock(); if (activeTab === 'brick') lcdLoop(); }
});
window.addEventListener('pagehide', () => { if (robot) robot.stop().catch(() => {}); });
window.addEventListener('resize', () => requestAnimationFrame(drawPath));

if ('serviceWorker' in navigator) navigator.serviceWorker.register('sw.js').catch(() => {});

applyLang();
replan();
autoConnect();
