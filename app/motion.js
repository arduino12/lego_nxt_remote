// Robot motion on top of NXT direct commands.
// Motor A drives all four wheels forward/back. Motors B and C turn the D-profile side
// wheels; one turn moves the robot one lane sideways, and between turns both must rest at
// "home" (RotationCount a multiple of 360, flat side down) or they drag on the floor.
// All targets are absolute RotationCount values, so position errors never accumulate.
//
// With the NXT Improved Firmware the brick regulates position itself (RegMode 4, 1 kHz PID
// with a speed/acceleration profile): every move is a target sent once and the brick drives
// to it - no Bluetooth latency in the loop. On arrival the motor is braked (holding the
// target would hunt in the gear backlash).
// Without it (LEGO firmware) a software loop here brakes the motors on target instead.
import { PORT_A, PORT_B, PORT_C, MODE_ON, MODE_BRAKE, MODE_REGULATED, REG_SPEED, RUN_RUNNING } from './nxt.js';

export const DEFAULTS = {
  invA: false, invB: false, invC: false, invSide: false,
  drivePower: 75,      // joystick full scale and path rows (max speed), %
  sidePower: 50,       // side steps (max speed), %
  fixPower: 15,        // software-loop corrections, %
  wheelCm: 6,          // drive wheel diameter; motor A turns the wheels 1:1
  stepCm: 5,           // sideways travel per B+C turn
  tolDeg: 4,           // close enough to a target / to home
  deadman: true,       // joystick drive stops by itself if the link drops
};

// Position regulation tuning, measured on this robot: PID 64/4/96 with acceleration 10
// lands within ±1° and overshoots ~9° on a full turn (firmware default 96/32/32: 24°).
const POS_PID = [64, 4, 96], POS_ACC = 10, CAL_SPEED = 30, CAL_ACC = 5;
const A_TOL = 10;           // motor A: 10° = 0.5 cm is plenty, and its gear backlash makes 4° hunt
const SYNC_LEAD = 90;       // B+C lockstep: how far the targets may run ahead of the slower motor
const SYNC_STALL_MS = 4000; // ...and how long it may be held back before we give up
const HUNT_MS = 1000;       // still swinging within 3x tolerance after this long counts as arrived
const DRIFT_CHECK_MS = 400; // after braking the D-wheels, look again: a loaded wheel can be pushed off
const STALL_MS = 1500, MOVE_TIMEOUT_MS = 30000, SETTLE_MS = 120, POLL_MS = 5, SPEED_WINDOW_MS = 20;
const BRAKE_S = 0.055;      // braking stops a motor within ~55 ms of travel (measured at 25-80 %)
const DEADMAN_S = 0.6;      // joystick drive runs out this long after the last command
const MIN_LIMIT = 250;      // LEGO firmware ignores tacho limits much below ~110°
const sleep = ms => new Promise(r => setTimeout(r, ms));
const mod360 = d => ((d % 360) + 360) % 360;

export class Aborted extends Error { constructor() { super('stopped'); } }

export class Robot {
  constructor(nxt, cfg) {
    this.nxt = nxt;
    this.cfg = cfg;
    this.posReg = false;     // brick has absolute position regulation (set after connecting)
    this.pos_ = {};          // port -> { offset: RotationCount - TachoCount, speed, acc } while regulating
    this.aTarget = null;     // absolute A target during a path run
    this.rtt = 30;           // ms per request, measured as we go
    this.abortGen = 0;
    this.drivePower = 0;
    this.sideBusy = false;
    this.onPos = null;       // (port, logicalDeg) for the UI
  }

  get cmPerDegA() { return Math.PI * this.cfg.wheelCm / 360; }

  sign(port) { return (port === PORT_A ? this.cfg.invA : port === PORT_B ? this.cfg.invB : this.cfg.invC) ? -1 : 1; }

  async pos(port) {
    const s = await this.nxt.getOutput(port);
    const p = s.rotation * this.sign(port);
    this.onPos?.(port, p);
    return { p, run: s.run, rotation: s.rotation, tacho: s.tacho };
  }

  // ------------------------------------------------------------ joystick
  // power -100..100 on A, re-sent every ~150 ms while the stick is held. Deadman: with
  // position regulation the target is only DEADMAN_S of travel ahead of the motor, so if
  // the link drops the robot rolls to the last target and stops; on LEGO firmware each
  // command is a tacho-limited chunk instead.
  async drive(power) {
    if (this.sideBusy) return false;
    power = Math.round(power);
    const dir = Math.sign(power) * this.sign(PORT_A);
    if (this.posReg && this.cfg.deadman) {
      if (power === 0) {   // brake rather than hold the last target: holding hunts in the backlash
        if (this.drivePower !== 0 || this.pos_[PORT_A]) await this.release([PORT_A]);
        this.drivePower = 0;
        return true;
      }
      const st = this.pos_[PORT_A];
      if (!st || Math.abs(st.speed - Math.abs(power)) >= 5) await this.regulate(PORT_A, Math.max(Math.abs(power), 10), POS_ACC * 2);
      const s = await this.pos(PORT_A);
      const lead = Math.max(60, Math.abs(power) * 9 * DEADMAN_S);   // ~9 °/s per %
      await this.nxt.posTarget(PORT_A, s.tacho + dir * lead);
      this.drivePower = power;
      return true;
    }
    if (power === 0) {
      if (this.drivePower !== 0) await this.nxt.halt(PORT_A);
      this.drivePower = 0;
      return true;
    }
    delete this.pos_[PORT_A];
    const limit = this.cfg.deadman ? Math.max(MIN_LIMIT, Math.abs(power) * 9 * DEADMAN_S) : 0;
    await this.nxt.setOutput(PORT_A, power * this.sign(PORT_A), MODE_ON | MODE_BRAKE | MODE_REGULATED, REG_SPEED, 0, RUN_RUNNING, limit);
    this.drivePower = power;
    return true;
  }

  // ------------------------------------------------------------ side steps
  async sideSteps(n, gen = this.abortGen) {
    if (!n) return;
    if (this.drivePower) await this.drive(0);
    this.sideBusy = true;
    try {
      const d = (this.cfg.invSide ? -n : n) * 360;
      const [b, c] = await Promise.all([this.pos(PORT_B), this.pos(PORT_C)]);
      const tb = Math.round(b.p / 360) * 360 + d, tc = Math.round(c.p / 360) * 360 + d;
      if (this.posReg) await this.lockstep([PORT_B, PORT_C], [tb, tc], this.cfg.sidePower, POS_ACC, gen);
      else await this.moveOnce([PORT_B, PORT_C], [tb, tc], this.cfg.sidePower, POS_ACC, gen);
    } finally { this.sideBusy = false; }
  }

  async isHome() {
    const [b, c] = await Promise.all([this.pos(PORT_B), this.pos(PORT_C)]);
    const off = v => Math.min(mod360(v), 360 - mod360(v));
    return off(b.p) <= this.cfg.tolDeg && off(c.p) <= this.cfg.tolDeg;
  }

  // The "calibrate D wheels" button: turn B and C slowly to the nearest home.
  async calibrate(gen = this.abortGen) {
    this.sideBusy = true;
    try {
      const [b, c] = await Promise.all([this.pos(PORT_B), this.pos(PORT_C)]);
      const targets = [Math.round(b.p / 360) * 360, Math.round(c.p / 360) * 360];
      if (this.posReg) await this.moveOnce([PORT_B, PORT_C], targets, CAL_SPEED, CAL_ACC, gen);
      else await this.moveTo([PORT_B, PORT_C], targets, this.cfg.fixPower * 2, gen);
    } finally { this.sideBusy = false; }
  }

  // Calibration: nudge one side motor, then call setHome() when the flat sides face down.
  async jog(port, deg) {
    const target = (await this.pos(port)).p + deg;
    if (this.posReg) await this.moveOnce([port], [target], CAL_SPEED, CAL_ACC);
    else await this.moveTo([port], [target], this.cfg.fixPower);
  }
  async setHome() {
    await this.nxt.resetMotorPosition(PORT_B, false);
    await this.nxt.resetMotorPosition(PORT_C, false);
    delete this.pos_[PORT_B];   // RotationCount moved under the regulator's offset
    delete this.pos_[PORT_C];
    await Promise.all([this.pos(PORT_B), this.pos(PORT_C)]);
  }

  // ------------------------------------------------------------ path runs
  async forward(cm, gen = this.abortGen) {
    if (this.aTarget === null) this.aTarget = (await this.pos(PORT_A)).p;
    this.aTarget += Math.round(cm / this.cmPerDegA);
    await this.moveOnce([PORT_A], [this.aTarget], this.cfg.drivePower, POS_ACC, gen);
  }

  async runPath(moves, onStep) {
    const gen = ++this.abortGen;
    this.aTarget = null;
    for (let i = 0; i < moves.length; i++) {
      onStep?.(i);
      const m = moves[i];
      if (m.type === 'fwd') await this.forward(m.cm, gen); else await this.sideSteps(m.steps, gen);
    }
    onStep?.(moves.length);
  }

  // Braking ends position regulation too; the next move sets it up again.
  async stop() {
    this.abortGen++;
    this.drivePower = 0;
    this.pos_ = {};
    await this.nxt.halt();
  }

  // ------------------------------------------------------------ single moves
  async moveOnce(ports, targets, speed, acc, gen = this.abortGen) {
    if (!this.posReg) { await this.approach(ports, targets, speed, gen); await this.settle(ports, gen); return; }
    for (const p of ports) {
      const st = this.pos_[p];
      if (!st || st.speed !== speed || st.acc !== acc) await this.regulate(p, speed, acc);
    }
    for (let i = 0; i < ports.length; i++) await this.target(ports[i], targets[i]);
    await this.waitTarget(ports, targets, gen);
    await this.release(ports);
  }

  // B and C turn in lockstep: both targets advance together, never more than SYNC_LEAD
  // degrees ahead of the motor that has come least far - so if one D-wheel is held back the
  // other waits for it, and both finish the same number of turns (like the firmware's
  // synchronised mode, which can't do exact position stops).
  async lockstep(ports, targets, speed, acc, gen) {
    for (const p of ports) {
      const st = this.pos_[p];
      if (!st || st.speed !== speed || st.acc !== acc) await this.regulate(p, speed, acc);
    }
    const s0 = await this.read(ports, gen);
    const tot = targets.map((t, i) => t - s0[i].p);
    const len = Math.max(...tot.map(Math.abs));
    if (len <= this.cfg.tolDeg) { await this.release(ports); return; }
    let frac = 0, best = 0, lastGain = s0.t;
    while (frac < 1) {
      const cur = await this.read(ports, gen);
      const done = Math.min(...cur.map((c, i) => tot[i] ? (c.p - s0[i].p) / tot[i] : 1));
      if (done > best + 0.01) { best = done; lastGain = cur.t; }
      if (cur.t - lastGain > SYNC_STALL_MS) { await this.stop(); throw new Error('motor stalled - is something blocking it?'); }
      const next = Math.min(1, Math.max(done, 0) + SYNC_LEAD / len);
      if (next > frac) {
        frac = next;
        for (let i = 0; i < ports.length; i++) await this.target(ports[i], s0[i].p + tot[i] * frac);
      }
      await sleep(POLL_MS);
    }
    await this.waitTarget(ports, targets, gen);
    await this.release(ports);
    // A braked motor can still be back-driven. If the robot's weight pushes a D-wheel off home,
    // hold it there with the regulator instead (a small hunt beats a dragging wheel).
    await sleep(DRIFT_CHECK_MS);
    const after = await this.read(ports, gen);
    for (let i = 0; i < ports.length; i++) {
      if (Math.abs(targets[i] - after[i].p) <= this.tol(ports[i])) continue;
      this.log?.(`port ${ports[i]} drifted to ${after[i].p - targets[i]}° after braking - holding it`);
      await this.regulate(ports[i], speed, acc);
      await this.target(ports[i], targets[i]);
    }
  }

  // Send a logical-degree target to a regulated port (RotationCount -> TachoCount units).
  target(port, logical) {
    return this.nxt.posTarget(port, Math.round(logical * this.sign(port) - this.pos_[port].offset));
  }

  // Stop and leave position regulation: a held target hunts back and forth in gear backlash.
  async release(ports) {
    for (const p of ports) { await this.nxt.halt(p); delete this.pos_[p]; }
  }

  tol(port) { return port === PORT_A ? Math.max(this.cfg.tolDeg, A_TOL) : this.cfg.tolDeg; }

  // Start (or re-profile) position regulation on a port. A fresh start zeroes TachoCount,
  // which also zeroes the regulator's own target - so it holds where the motor is now.
  async regulate(port, speed, acc) {
    const st = this.pos_[port];
    if (st) {
      await this.nxt.posProfile(port, speed, acc);
      Object.assign(st, { speed, acc });
      return;
    }
    await this.nxt.posInit(port, { maxSpeed: speed, maxAcc: acc, pid: this.cfg.posPid || POS_PID });
    const s = await this.pos(port);
    const offset = s.rotation - s.tacho;
    await this.nxt.posTarget(port, s.tacho);   // enter RegMode 4, holding here
    await this.nxt.posTarget(port, s.tacho);   // the first may land before the mode switch
    this.pos_[port] = { offset, speed, acc };
  }

  // Done when every port has stayed within tolerance for SETTLE_MS, or within 3x tolerance for
  // HUNT_MS. (Holding a target the regulator dithers, and under load with gear backlash it can
  // swing ±10° indefinitely - so "counter unchanged" would rarely come true.)
  async waitTarget(ports, targets, gen) {
    const t0 = performance.now();
    let prev = await this.read(ports, gen), inSince = null, nearSince = null, lastMove = t0;
    for (;;) {
      await sleep(POLL_MS);
      const cur = await this.read(ports, gen);
      if (cur.some((c, i) => c.p !== prev[i].p)) lastMove = cur.t;
      const ok = cur.every((c, i) => Math.abs(targets[i] - c.p) <= this.tol(ports[i]));
      const near = cur.every((c, i) => Math.abs(targets[i] - c.p) <= 3 * this.tol(ports[i]));
      inSince = ok ? inSince ?? cur.t : null;
      nearSince = near ? nearSince ?? cur.t : null;
      if (ok && cur.t - inSince >= SETTLE_MS) return;
      if (near && cur.t - nearSince >= HUNT_MS) return;   // hunting in the backlash: close enough, brake it
      if (!ok && cur.t - lastMove > STALL_MS) { await this.stop(); throw new Error('motor stalled - is something blocking it?'); }
      if (cur.t - t0 > MOVE_TIMEOUT_MS) { await this.stop(); throw new Error('move timed out'); }
      prev = cur;
    }
  }

  // ------------------------------------------------------------ software loop (LEGO firmware)
  // LEGO firmware tacho limits can't position precisely (1.01 cuts power ~110° early and
  // lands ±25° off; 1.29 coasts past the limit), so: drive, watch the counters, and brake when
  // the remaining angle is what the motor needs to stop (~55 ms of travel) plus one link
  // round trip. moveTo() then repeats at fixPower until every motor is within tolDeg.
  async moveTo(ports, targets, power, gen = this.abortGen) {
    for (let pass = 0; ; pass++) {
      const now = await this.read(ports, gen);
      const err = targets.map((t, i) => t - now[i].p);
      this.log?.(`pass ${pass}: error ${err.join(', ')}°`);
      if (err.every(e => Math.abs(e) <= this.cfg.tolDeg)) return;
      if (pass === 6) throw new Error(`could not reach target (error ${err.map(Math.round).join(', ')}°)`);
      await this.approach(ports, targets, pass === 0 ? power : this.cfg.fixPower, gen);
      await this.settle(ports, gen);
    }
  }

  async approach(ports, targets, power, gen) {
    let prev = await this.read(ports, gen);
    const dir = targets.map((t, i) => Math.sign(t - prev[i].p));
    const live = dir.map((d, i) => d !== 0 && Math.abs(targets[i] - prev[i].p) > this.cfg.tolDeg);
    for (let i = 0; i < ports.length; i++) {
      if (live[i]) await this.nxt.setOutput(ports[i], dir[i] * this.sign(ports[i]) * power, MODE_ON | MODE_BRAKE | MODE_REGULATED, REG_SPEED, 0, RUN_RUNNING, 0);
    }
    const t0 = performance.now();
    let lastMove = t0;
    const hist = [prev];   // speed over >= SPEED_WINDOW_MS: one 1° tick per 1 ms USB poll would read as 1000 °/s
    while (live.some(Boolean)) {
      await sleep(POLL_MS);
      const cur = await this.read(ports, gen);
      hist.push(cur);
      while (hist.length > 2 && cur.t - hist[1].t >= SPEED_WINDOW_MS) hist.shift();
      const ref = hist[0], dt = Math.max(cur.t - ref.t, 1) / 1000;
      for (let i = 0; i < ports.length; i++) {
        if (!live[i]) continue;
        const v = Math.abs(cur[i].p - ref[i].p) / dt;
        const left = (targets[i] - cur[i].p) * dir[i];
        if (left <= v * (BRAKE_S + 1.5 * this.rtt / 1000) + 1) { await this.nxt.brake(ports[i]); live[i] = false; }   // speed mode: a plain brake is exact
      }
      if (cur.some((c, i) => c.p !== prev[i].p)) lastMove = cur.t;
      if (cur.t - lastMove > STALL_MS) { await this.nxt.halt(); throw new Error('motor stalled - is something blocking it?'); }
      if (cur.t - t0 > MOVE_TIMEOUT_MS) { await this.nxt.halt(); throw new Error('move timed out'); }
      prev = cur;
    }
  }

  // Wait until the counters stop changing (the "idle" run state is reported long before that).
  async settle(ports, gen) {
    let prev = await this.read(ports, gen);
    const t0 = prev.t;
    let still = t0;
    while (prev.t - t0 < 3000) {
      await sleep(25);
      const cur = await this.read(ports, gen);
      if (cur.some((c, i) => c.p !== prev[i].p)) still = cur.t;
      else if (cur.t - still >= SETTLE_MS) return;
      prev = cur;
    }
  }

  // Positions of several ports, with a timestamp; keeps a running link round-trip estimate.
  async read(ports, gen) {
    if (gen !== undefined && gen !== this.abortGen) { await this.stop(); throw new Aborted(); }
    const t = performance.now();
    const r = [];
    for (const p of ports) r.push(await this.pos(p));
    const now = performance.now();
    this.rtt = this.rtt * 0.8 + (now - t) / ports.length * 0.2;
    r.t = now;
    return r;
  }
}
