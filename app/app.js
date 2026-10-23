// NXT Remote - phone controller for a LEGO NXT robot over Web Serial (Bluetooth SPP).
// Motor A drives forward/back; B+C turn D-profile wheels that step the robot sideways.
import { Nxt, SerialTransport, UsbTransport, SPP_UUID, USB_FILTER, PORT_A, PORT_B, PORT_C, SCREEN_W, SCREEN_H, screenPixel, BT_OFF, BT_VISIBLE, BT_CONNECTED, BT_PIN_REQUEST } from './nxt.js';
import { Robot, DEFAULTS, Aborted } from './motion.js';
import { snakePath, pathStats } from './path.js';

const APP_VERSION = '0.4.0';   // keep in step with index.html (?v=) and sw.js
const KEEPALIVE_MS = 30000, JOY_MS = 150, LCD_MS = 200;
const JOY_DEAD = 0.15, JOY_SIDE = 0.5, MIN_POWER = 15;

// ------------------------------------------------------------ strings
const STR = {
  en: {
    langBtn: 'עברית', title: 'NXT Remote',
    connect: 'Connect', connecting: 'Connecting…', disconnect: 'Disconnect', usb: 'USB',
    tabDrive: 'Drive', tabPath: 'Path', tabBrick: 'Brick',
    joyHint: 'Up/down drives (motor A). Left/right steps sideways, one B+C turn per step.',
    stepLeft: '◀ Step', stepRight: 'Step ▶', drivePower: 'Drive speed', sidePower: 'Side-step speed',
    run: 'Run snake', stopRun: 'Stop',
    snakeTitle: 'Snake grid', rows: 'Rows', rowCm: 'Row length (cm)', sideTurns: 'Turns between rows (B+C)', sideDir: 'Shift to',
    rowInfo: '= {0} wheel turns per row · {1} cm between rows',
    pathEmpty: 'Set at least one row.',
    pathInfo: '{0} rows · {1} cm driving · {2} side turns · ≈{3}',
    pathDone: 'Finished ✓', pathStep: 'Row {0}/{1}…',
    screen: 'Brick screen', lcdLive: 'Live refresh',
    bt: 'Bluetooth: {0}', btOff: 'off', btOn: 'on', btVisible: 'visible', btHidden: 'hidden', btConnected: 'connected',
    btPin: 'waiting for the passkey - press ●',
    homeTitle: 'Side wheels home',
    homeHint: 'Turn B and C until the flat side of every D-wheel faces the floor, then tap "Set home". The brick counts from where the motors were at power-on, so switch it off with the wheels at home.',
    setHome: 'Set home here', goHome: 'Return home',
    calTitle: 'Settings', wheelCm: 'Wheel diameter (cm)', calInfo: 'Motor A: {0} cm per turn',
    calibD: 'Calibrate D wheels (flat side down)', hintCalibrating: 'Turning the D wheels to home…', hintCalibrated: 'D wheels at home ✓',
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
    hintSaved: 'Saved ✓',
    hintHomeSet: 'Home set ✓', credit: 'By Arad & Claude 2026',
  },
  he: {
    langBtn: 'English', title: 'שלט NXT',
    connect: 'התחברות', connecting: 'מתחבר…', disconnect: 'התנתקות', usb: 'USB',
    tabDrive: 'נהיגה', tabPath: 'מסלול', tabBrick: 'בקר',
    joyHint: 'למעלה/למטה - נסיעה (מנוע A). ימינה/שמאלה - צעד הצידה, סיבוב אחד של B+C לכל צעד.',
    stepLeft: '◀ צעד', stepRight: 'צעד ▶', drivePower: 'מהירות נסיעה', sidePower: 'מהירות צעד הצידה',
    run: 'הפעלת שביל נחש', stopRun: 'עצירה',
    snakeTitle: 'שביל נחש', rows: 'מספר שורות', rowCm: 'אורך שורה (ס"מ)', sideTurns: 'סיבובים בין שורות (B+C)', sideDir: 'כיוון ההזזה',
    rowInfo: '= {0} סיבובי גלגל לשורה · {1} ס"מ בין שורות',
    pathEmpty: 'הגדירו לפחות שורה אחת.',
    pathInfo: '{0} שורות · {1} ס"מ נסיעה · {2} סיבובי צד · ≈{3}',
    pathDone: 'הסתיים ✓', pathStep: 'שורה {0}/{1}…',
    screen: 'מסך הבקר', lcdLive: 'רענון חי',
    bt: 'בלוטות\': {0}', btOff: 'כבוי', btOn: 'דולק', btVisible: 'גלוי', btHidden: 'מוסתר', btConnected: 'מחובר',
    btPin: 'ממתין לקוד צימוד - לחצו ●',
    homeTitle: 'נקודת אפס לגלגלי הצד',
    homeHint: 'סובבו את B ו-C עד שהצד השטוח של כל גלגלי ה-D פונה לרצפה, ולחצו "קבע כאפס". הבקר סופר מהמקום שבו היו המנועים כשהודלק, לכן כבו אותו כשהגלגלים באפס.',
    setHome: 'קבע כאפס', goHome: 'חזרה לאפס',
    calTitle: 'הגדרות', wheelCm: 'קוטר גלגל (ס"מ)', calInfo: 'מנוע A: {0} ס"מ לסיבוב',
    calibD: 'כיול גלגלי D (צד שטוח למטה)', hintCalibrating: 'מסובב את גלגלי ה-D לאפס…', hintCalibrated: 'גלגלי ה-D באפס ✓',
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
    hintSaved: 'נשמר ✓',
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
  return { ...DEFAULTS, rows: 4, rowCm: 100, sideTurns: 2, sideDir: 1, ...saved };
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
  // The first request after opening a Bluetooth port can be lost while the link comes up.
  for (let i = 0; ; i++) {
    try { brickInfo = await nxt.deviceInfo(); break; } catch (e) { if (i === 2) throw e; }
  }
  const fw = await nxt.firmware();
  batteryMv = await nxt.battery();
  tr.onclose = onClosed;
  userDisconnect = false;
  setConn('on');
  setHint(null);
  $('stName').textContent = brickInfo.name;
  showBattery();
  $('infoFw').textContent = `${fw.firmware} (protocol ${fw.protocol})`;
  robot.posReg = await nxt.hasPosRegulation();   // NXT Improved Firmware: the brick positions motors itself
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

// ------------------------------------------------------------ path (snake grid)
const pc = $('pathCanvas'), pctx = pc.getContext('2d');
let plan = null, runStep = -1, running = false;

function replan() {
  plan = snakePath({ rows: cfg.rows, rowCm: cfg.rowCm, sideTurns: cfg.sideTurns, stepCm: cfg.stepCm, sideDir: cfg.sideDir });
  $('rowInfo').textContent = t('rowInfo', (cfg.rowCm / (Math.PI * cfg.wheelCm)).toFixed(2), cfg.sideTurns * cfg.stepCm);
  for (const b of [$('dirL'), $('dirR')]) b.classList.toggle('on', +b.dataset.dir === cfg.sideDir);
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
  const fwdS = st.fwdCm / (Math.PI * cfg.wheelCm / 360 * 8 * cfg.drivePower);
  const sideS = st.steps * (360 / (8 * cfg.sidePower) + 0.3);
  $('pathInfo').textContent = t('pathInfo', cfg.rows, Math.round(st.fwdCm), st.steps, fmtDuration(fwdS + sideS + plan.moves.length * 0.4));
}

// Top view at true scale: rows run up/down (motor A), shifts go left/right (B+C).
function drawPath() {
  const dpr = window.devicePixelRatio || 1, w = pc.clientWidth, h = pc.clientHeight;
  if (!w) return;
  if (pc.width !== Math.round(w * dpr)) { pc.width = Math.round(w * dpr); pc.height = Math.round(h * dpr); }
  const ctx = pctx;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  if (!plan?.moves.length) return;
  const pts = plan.pts, m = 34;
  const xs = pts.map(p => p.x), x0 = Math.min(...xs), x1 = Math.max(...xs), y1 = Math.max(cfg.rowCm, 1);
  const k = Math.min((w - 2 * m) / Math.max(x1 - x0, 1e-9), (h - 2 * m) / y1);
  const ox = (w - (x1 - x0) * k) / 2 - x0 * k, oy = h - (h - y1 * k) / 2;
  const sx = x => ox + x * k, sy = y => oy - y * k;

  ctx.lineJoin = ctx.lineCap = 'round';
  ctx.lineWidth = 4;
  for (let i = 1; i < pts.length; i++) {
    ctx.strokeStyle = !running ? '#6aa8ff' : i - 1 < runStep ? '#4cc38a' : i - 1 === runStep ? '#f0b44c' : '#2c3a52';
    ctx.beginPath(); ctx.moveTo(sx(pts[i - 1].x), sy(pts[i - 1].y)); ctx.lineTo(sx(pts[i].x), sy(pts[i].y)); ctx.stroke();
  }
  // Robot at the start, facing up; green dot at the end.
  const rw = 14;
  ctx.fillStyle = '#e8eaf0';
  ctx.fillRect(sx(0) - rw / 2, sy(0) - rw / 2, rw, rw);
  ctx.fillStyle = '#e5534b';
  ctx.beginPath(); ctx.moveTo(sx(0), sy(0) - rw / 2 - 7); ctx.lineTo(sx(0) - 5, sy(0) - rw / 2); ctx.lineTo(sx(0) + 5, sy(0) - rw / 2); ctx.fill();
  const e = pts[pts.length - 1];
  ctx.fillStyle = '#4cc38a';
  ctx.beginPath(); ctx.arc(sx(e.x), sy(e.y), 6, 0, 2 * Math.PI); ctx.fill();
  // Dimensions.
  ctx.fillStyle = '#8a90a0';
  ctx.font = '12px system-ui, sans-serif';
  ctx.textAlign = 'center';
  ctx.direction = 'ltr';   // the page is RTL in Hebrew; keep "100 cm" in order
  ctx.fillText(`${cfg.rowCm} cm`, sx(x0) < 44 ? sx(x0) + 26 : sx(x0) - 26, sy(y1 / 2));
  if (x1 > x0) ctx.fillText(`${Math.round(x1 - x0)} cm`, (sx(x0) + sx(x1)) / 2, Math.min(h - 6, sy(0) + 22));
}

// Tunes from the brick's speaker: [Hz, ms] notes, one at a time (a new tone replaces the last).
const TUNE_START = [[523, 110], [659, 110], [784, 220]];
const TUNE_DONE = [[784, 110], [659, 110], [523, 110], [1047, 350]];
const TUNE_STOP = [[392, 200], [262, 400]];
async function tune(notes) {
  for (const [hz, ms] of notes) {
    await nxt.playTone(hz, ms);
    await sleep(ms + 30);
  }
}

async function runPath() {
  if (running) { await robot?.stop(); return; }
  if (!plan?.moves.length) return;
  if (!robot || connState !== 'on') { setHint('hintConnectFirst'); return; }
  running = true;
  $('runBtn').textContent = t('stopRun');
  const n = plan.moves.length;
  try {
    await tune(TUNE_START);
    await robot.runPath(plan.moves, i => {
      runStep = i;
      $('pathInfo').textContent = i < n ? t('pathStep', plan.moves[i].row + 1, cfg.rows) : t('pathDone');
      drawPath();
    });
    await tune(TUNE_DONE);
  } catch (e) {
    if (!(e instanceof Aborted)) setHint('hintFail', e.message, true);
    if (nxt) tune(TUNE_STOP).catch(() => {});
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

function showCal() { $('calInfo').textContent = t('calInfo', (Math.PI * cfg.wheelCm).toFixed(2)); }

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

$('runBtn').addEventListener('click', runPath);
for (const id of ['rows', 'rowCm', 'sideTurns', 'wheelCm']) {
  const el = $(id);
  el.value = cfg[id];
  el.addEventListener('input', () => {
    const v = parseFloat(el.value);
    if (!(v >= 0)) return;
    cfg[id] = id === 'rows' || id === 'sideTurns' ? Math.round(v) : v;
    saveCfg(); replan(); showCal();
  });
}
for (const b of [$('dirL'), $('dirR')]) b.addEventListener('click', () => { cfg.sideDir = +b.dataset.dir; saveCfg(); replan(); });
$('calibBtn').addEventListener('click', () => act(async () => { setHint('hintCalibrating'); await robot.calibrate(); setHint('hintCalibrated'); }));

for (const b of document.querySelectorAll('[data-btn]')) b.addEventListener('click', () => act(() => nxt.pressButton(+b.dataset.btn)));
for (const b of document.querySelectorAll('[data-jog]')) b.addEventListener('click', () => {
  const [port, deg] = b.dataset.jog.split(',').map(Number);
  act(() => robot.jog(port, deg));
});
$('setHomeBtn').addEventListener('click', () => act(async () => { await robot.setHome(); setHint('hintHomeSet'); }));
$('goHomeBtn').addEventListener('click', () => act(() => robot.calibrate()));
for (const el of document.querySelectorAll('[data-cfg]')) {
  el.checked = !!cfg[el.dataset.cfg];
  el.addEventListener('change', () => { cfg[el.dataset.cfg] = el.checked; saveCfg(); });
}
$('renameBtn').addEventListener('click', () => act(async () => {
  const name = $('nameInput').value.trim();
  if (!name) return;
  await nxt.setName(name);
  // The first request after opening a Bluetooth port can be lost while the link comes up.
  for (let i = 0; ; i++) {
    try { brickInfo = await nxt.deviceInfo(); break; } catch (e) { if (i === 2) throw e; }
  }
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
