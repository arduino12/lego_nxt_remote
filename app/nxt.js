// LEGO MINDSTORMS NXT direct commands over Web Serial (Bluetooth SPP) or WebUSB.
// Protocol: LEGO "Bluetooth Developer Kit", appendix 1 (system commands) and 2 (direct commands).
// IO-map offsets: NBC/NXC docs (Display and Ui module IOMAP offsets).

export const PORT_A = 0, PORT_B = 1, PORT_C = 2, PORT_ALL = 0xFF;
export const MODE_ON = 0x01, MODE_BRAKE = 0x02, MODE_REGULATED = 0x04;
export const REG_IDLE = 0, REG_SPEED = 1, REG_SYNC = 2, REG_POS = 4;   // REG_POS: NXT Improved Firmware only
export const RUN_IDLE = 0x00, RUN_RAMPUP = 0x10, RUN_RUNNING = 0x20, RUN_RAMPDOWN = 0x40;
export const BTN_LEFT = 1, BTN_ENTER = 2, BTN_RIGHT = 3, BTN_EXIT = 4;
export const BT_VISIBLE = 0x01, BT_CONNECTED = 0x02, BT_OFF = 0x04, BT_PIN_REQUEST = 0x80;
export const SPP_UUID = '00001101-0000-1000-8000-00805f9b34fb';
export const USB_FILTER = { vendorId: 0x0694, productId: 0x0002 };
export const SCREEN_W = 100, SCREEN_H = 64;

const DIRECT = 0x00, SYSTEM = 0x01, NO_REPLY = 0x80, REPLY = 0x02;
const MOD_OUTPUT = 0x00020001, MOD_DISPLAY = 0x000A0001, MOD_UI = 0x000C0001;
// Output IOMAP: 32 bytes per port. Flags at 18, RegP/I/D at 22-24, MaxSpeed/MaxAcceleration at 30-31
// (the last two exist in the NXT Improved Firmware, src/c_output.iom).
const OUT_FLAGS = 18, OUT_PID = 22, OUT_MAX = 30;
const UF_RESET_COUNT = 0x08, UF_PID_VALUES = 0x10;
const DISP_FLAGS = 117, DISP_NORMAL = 119, DISP_POPUP = 919, DISP_F_POPUP = 0x08;
const UI_FLAGS = 26, UI_BUTTON = 28;
const SCREEN_BYTES = 800, IOMAP_CHUNK = 50;   // a reply must fit one 64-byte telegram
const TIMEOUT_MS = 2000;

const STATUS = {
  0x20: 'transaction in progress', 0x40: 'mailbox empty', 0xBD: 'request failed', 0xBE: 'unknown opcode',
  0xBF: 'insane packet', 0xC0: 'value out of range', 0xDD: 'bus error', 0xDE: 'no free buffer',
  0xDF: 'bad connection', 0xE0: 'busy', 0xEC: 'no active program', 0xED: 'illegal size',
  0xEE: 'bad mailbox', 0xEF: 'bad field', 0xF0: 'bad port', 0xFB: 'out of memory', 0xFF: 'bad arguments',
  0x90: 'module not found', 0x91: 'out of boundary',
};

export class NxtError extends Error {
  constructor(cmd, status) {
    super(`NXT 0x${cmd.toString(16)}: ${STATUS[status] || 'status 0x' + status.toString(16)}`);
    this.status = status;
  }
}

const u16 = v => [v & 0xFF, (v >> 8) & 0xFF];
const u32 = v => [v & 0xFF, (v >>> 8) & 0xFF, (v >>> 16) & 0xFF, (v >>> 24) & 0xFF];
const cstr = (s, n) => { const b = new Array(n).fill(0); for (let i = 0; i < Math.min(s.length, n - 1); i++) b[i] = s.charCodeAt(i) & 0x7F; return b; };
const readStr = (dv, off, n) => { let s = ''; for (let i = 0; i < n; i++) { const c = dv.getUint8(off + i); if (!c) break; s += String.fromCharCode(c); } return s; };

// ------------------------------------------------------------ transports
// Both deliver whole telegrams (command byte first) through recv(); a reader loop
// queues them so a late reply never collides with a pending read.
class FrameQueue {
  constructor() { this.frames = []; this.waiters = []; this.onclose = null; this.closed = false; }
  deliver(f) { const w = this.waiters.shift(); if (w) w.resolve(f); else this.frames.push(f); }
  drain() { this.frames.length = 0; }
  recv(ms) {
    if (this.frames.length) return Promise.resolve(this.frames.shift());
    if (this.closed) return Promise.reject(new Error('disconnected'));
    return new Promise((resolve, reject) => {
      const w = { resolve: f => { clearTimeout(w.timer); resolve(f); }, reject };
      w.timer = setTimeout(() => { this.waiters.splice(this.waiters.indexOf(w), 1); reject(new Error('NXT did not answer')); }, Math.max(ms, 1));
      this.waiters.push(w);
    });
  }
  ended(err) {
    if (this.closed) return;
    this.closed = true;
    for (const w of this.waiters.splice(0)) { clearTimeout(w.timer); w.reject(err || new Error('disconnected')); }
    this.onclose?.(err);
  }
}

// Bluetooth SPP: every telegram is prefixed with its length (2 bytes, little-endian).
export class SerialTransport extends FrameQueue {
  constructor(port) { super(); this.port = port; this.buf = new Uint8Array(0); this.kind = 'bt'; }
  async open() {
    await this.port.open({ baudRate: 115200 });   // ignored for RFCOMM, but required
    this.writer = this.port.writable.getWriter();
    this.reader = this.port.readable.getReader();
    this.loop();
  }
  async loop() {
    let err;
    try {
      for (;;) {
        const { value, done } = await this.reader.read();
        if (done) break;
        this.push(value);
      }
    } catch (e) { err = e; }
    this.ended(err);
  }
  push(chunk) {
    const b = new Uint8Array(this.buf.length + chunk.length);
    b.set(this.buf); b.set(chunk, this.buf.length);
    let i = 0;
    while (b.length - i >= 2) {
      const n = b[i] | (b[i + 1] << 8);
      if (b.length - i - 2 < n) break;
      this.deliver(b.slice(i + 2, i + 2 + n));
      i += 2 + n;
    }
    this.buf = b.slice(i);
  }
  async send(tel) {
    const f = new Uint8Array(tel.length + 2);
    f.set(u16(tel.length)); f.set(tel, 2);
    await this.writer.write(f);
  }
  async close() {
    try { await this.reader.cancel(); } catch {}
    try { this.reader.releaseLock(); this.writer.releaseLock(); } catch {}
    try { await this.port.close(); } catch {}
    this.ended();
  }
}

// USB: raw telegrams on the bulk endpoints (needs WinUSB on Windows; works as-is on Android/Linux).
export class UsbTransport extends FrameQueue {
  constructor(dev) { super(); this.dev = dev; this.kind = 'usb'; }
  async open() {
    await this.dev.open();
    if (!this.dev.configuration) await this.dev.selectConfiguration(1);
    await this.dev.claimInterface(0);
    const eps = this.dev.configuration.interfaces[0].alternate.endpoints;
    this.epIn = eps.find(e => e.direction === 'in').endpointNumber;
    this.epOut = eps.find(e => e.direction === 'out').endpointNumber;
    this.loop();
  }
  async loop() {
    let err;
    try {
      while (this.dev.opened) {
        const r = await this.dev.transferIn(this.epIn, 64);
        if (r.data?.byteLength) this.deliver(new Uint8Array(r.data.buffer, r.data.byteOffset, r.data.byteLength).slice());
      }
    } catch (e) { err = e; }
    this.ended(err);
  }
  async send(tel) { await this.dev.transferOut(this.epOut, tel); }
  async close() { try { await this.dev.close(); } catch {} this.ended(); }
}

// ------------------------------------------------------------ brick
export class Nxt {
  constructor(transport) {
    this.t = transport;
    this.lock = Promise.resolve();
  }

  // One request in flight at a time. No-reply commands skip the BT turnaround
  // (~30 ms each way on the NXT) - use them for streaming motor updates.
  request(type, cmd, payload = [], reply = true) {
    const run = async () => {
      const tel = Uint8Array.from([reply ? type : type | NO_REPLY, cmd, ...payload]);
      if (!reply) { await this.t.send(tel); return null; }
      this.t.drain();
      await this.t.send(tel);
      const deadline = performance.now() + TIMEOUT_MS;
      for (;;) {
        const r = await this.t.recv(deadline - performance.now());
        if (r.length < 3 || r[0] !== REPLY || r[1] !== cmd) continue;   // stale reply
        if (r[2]) throw new NxtError(cmd, r[2]);
        return new DataView(r.buffer, r.byteOffset, r.byteLength);
      }
    };
    const p = this.lock.then(run, run);
    this.lock = p.catch(() => {});
    return p;
  }

  playTone(hz, ms) { return this.request(DIRECT, 0x03, [...u16(hz), ...u16(ms)], false); }

  setOutput(port, power, mode, reg, turn, run, limit = 0, reply = false) {
    const pw = Math.max(-100, Math.min(100, Math.round(power)));
    return this.request(DIRECT, 0x04, [port, pw & 0xFF, mode, reg, turn & 0xFF, run, ...u32(Math.round(limit))], reply);
  }

  async getOutput(port) {
    const d = await this.request(DIRECT, 0x06, [port]);
    return {
      power: d.getInt8(4), mode: d.getUint8(5), reg: d.getUint8(6), turn: d.getInt8(7), run: d.getUint8(8),
      limit: d.getUint32(9, true), tacho: d.getInt32(13, true), block: d.getInt32(17, true), rotation: d.getInt32(21, true),
    };
  }

  // relative=false zeroes RotationCount (our absolute position); true zeroes BlockTachoCount.
  resetMotorPosition(port, relative = false) { return this.request(DIRECT, 0x0A, [port, relative ? 1 : 0]); }

  setInputMode(port, type, mode) { return this.request(DIRECT, 0x05, [port, type, mode]); }
  async getInput(port) {
    const d = await this.request(DIRECT, 0x07, [port]);
    return { valid: !!d.getUint8(4), type: d.getUint8(6), mode: d.getUint8(7), raw: d.getUint16(8, true), normalized: d.getUint16(10, true), scaled: d.getInt16(12, true) };
  }

  async battery() { return (await this.request(DIRECT, 0x0B)).getUint16(3, true); }   // mV
  async keepAlive() { return (await this.request(DIRECT, 0x0D)).getUint32(3, true); }  // sleep limit, ms

  async firmware() {
    const d = await this.request(SYSTEM, 0x88);
    return { protocol: `${d.getUint8(4)}.${d.getUint8(3)}`, firmware: `${d.getUint8(6)}.${String(d.getUint8(5)).padStart(2, '0')}` };
  }

  async deviceInfo() {
    const d = await this.request(SYSTEM, 0x9B);
    const addr = [...Array(6)].map((_, i) => d.getUint8(18 + i).toString(16).padStart(2, '0')).join(':').toUpperCase();
    return { name: readStr(d, 3, 15), btAddr: addr, signal: d.getUint32(25, true), freeFlash: d.getUint32(29, true) };
  }

  setName(name) { return this.request(SYSTEM, 0x98, cstr(name, 16)); }

  async readIOMap(mod, offset, n) {
    const d = await this.request(SYSTEM, 0x94, [...u32(mod), ...u16(offset), ...u16(n)]);
    const got = d.getUint16(7, true);
    return new Uint8Array(d.buffer, d.byteOffset + 9, got).slice();
  }
  writeIOMap(mod, offset, bytes) {
    return this.request(SYSTEM, 0x95, [...u32(mod), ...u16(offset), ...u16(bytes.length), ...bytes]);
  }

  // The LCD as 8 pages x 100 columns; each byte is 8 vertical pixels, bit 0 on top.
  // Shows the popup layer when it is up (e.g. the "Turn off?" or passkey screens).
  async readScreen() {
    const flags = (await this.readIOMap(MOD_DISPLAY, DISP_FLAGS, 1))[0];
    const base = flags & DISP_F_POPUP ? DISP_POPUP : DISP_NORMAL;
    const buf = new Uint8Array(SCREEN_BYTES);
    for (let off = 0; off < SCREEN_BYTES; off += IOMAP_CHUNK) buf.set(await this.readIOMap(MOD_DISPLAY, base + off, IOMAP_CHUNK), off);
    return buf;
  }

  // Simulates a front-panel button press (the UI module consumes and clears it).
  pressButton(btn) { return this.writeIOMap(MOD_UI, UI_BUTTON, [btn]); }

  // Ui IOMAP bytes 26..32: Flags, State, Button, RunState, BatteryState, BluetoothState, UsbState.
  async uiState() {
    const b = await this.readIOMap(MOD_UI, UI_FLAGS, 7);
    return { state: b[1], runState: b[3], bt: b[5], usb: b[6] };
  }

  // ---- absolute position regulation (NXT Improved Firmware, RegMode 4) ----
  // The brick PID-regulates the motor to a target in TachoCount units at 1 kHz, following a
  // speed/acceleration profile - no link latency in the loop. posInit zeroes TachoCount
  // (which also zeroes the regulator's target, so nothing jumps) and sets the profile.
  async hasPosRegulation() {
    const b = await this.readIOMap(MOD_OUTPUT, OUT_MAX, 2);   // LEGO 1.29 keeps these spare bytes at 0
    return b[0] !== 0 || b[1] !== 0;
  }
  async posInit(port, { maxSpeed, maxAcc, pid }) {
    const base = port * 32;
    if (pid) await this.writeIOMap(MOD_OUTPUT, base + OUT_PID, pid);
    await this.writeIOMap(MOD_OUTPUT, base + OUT_MAX, [maxSpeed & 0xFF, maxAcc & 0xFF]);
    await this.writeIOMap(MOD_OUTPUT, base + OUT_FLAGS, [UF_RESET_COUNT | UF_PID_VALUES]);
  }
  async posProfile(port, maxSpeed, maxAcc) {
    const base = port * 32;
    await this.writeIOMap(MOD_OUTPUT, base + OUT_MAX, [maxSpeed & 0xFF, maxAcc & 0xFF]);
    await this.writeIOMap(MOD_OUTPUT, base + OUT_FLAGS, [UF_PID_VALUES]);
  }
  // Target in TachoCount degrees (signed). Travels in the TachoLimit field.
  posTarget(port, tacho, reply = false) {
    return this.setOutput(port, 0, MODE_ON | MODE_BRAKE | MODE_REGULATED, REG_POS, 0, RUN_RUNNING, tacho >>> 0, reply);
  }

  // Reliable stop. Leaving position regulation, the firmware keeps the last PID output power
  // latched: a plain brake (or regulated speed 0) left motor A running at ~300 °/s. Any
  // non-zero speed command refreshes the output, so: 1 % for a moment, then brake.
  async halt(port = PORT_ALL) {
    await this.setOutput(port, 1, MODE_ON | MODE_BRAKE | MODE_REGULATED, REG_SPEED, 0, RUN_RUNNING, 0);
    await new Promise(r => setTimeout(r, 30));
    await this.brake(port);
  }

  brake(port = PORT_ALL) { return this.setOutput(port, 0, MODE_ON | MODE_BRAKE, REG_IDLE, 0, RUN_RUNNING, 0); }
  float(port = PORT_ALL) { return this.setOutput(port, 0, 0, REG_IDLE, 0, RUN_IDLE, 0); }
}

export function screenPixel(buf, x, y) { return (buf[(y >> 3) * SCREEN_W + x] >> (y & 7)) & 1; }
