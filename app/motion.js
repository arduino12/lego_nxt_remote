// Robot motion on top of NXT direct commands.
// Motor A drives all four wheels forward/back. Motors B and C turn the D-profile side
// wheels; one turn moves the robot one lane sideways, and between turns both must rest at
// "home" (RotationCount a multiple of 360, flat side down) or they drag on the floor.
// All targets are absolute RotationCount values, so position errors never accumulate.
import { PORT_A, PORT_B, PORT_C, MODE_ON, MODE_BRAKE, MODE_REGULATED, REG_SPEED, REG_SYNC, RUN_IDLE, RUN_RUNNING } from './nxt.js';

export const DEFAULTS = {
  invA: false, invB: false, invC: false, invSide: false,
  drivePower: 75,      // joystick full scale and path driving, %
  sidePower: 60,       // side steps, %
  fixPower: 25,        // correction passes, %
  cmPerDegA: 0.0489,   // 5.6 cm wheel on the motor axle; calibrate in the Brick tab
  stepCm: 5,           // sideways travel per B+C turn
  tolDeg: 4,           // close enough to a target / to home
  deadman: true,       // joystick drive runs in short tacho-limited chunks
};

const POLL_MS = 50, STALL_MS = 700, MOVE_TIMEOUT_MS = 20000, DEADMAN_S = 0.5;
const sleep = ms => new Promise(r => setTimeout(r, ms));
const mod360 = d => ((d % 360) + 360) % 360;

export class Aborted extends Error { constructor() { super('stopped'); } }

export class Robot {
  constructor(nxt, cfg) {
    this.nxt = nxt;
    this.cfg = cfg;
    this.aTarget = null;     // absolute A target during a path run
    this.softLimit = false;  // set if the firmware ignores tacho limits; then we stop motors ourselves
    this.abortGen = 0;
    this.drivePower = 0;
    this.sideBusy = false;
    this.onPos = null;       // (port, logicalDeg) for the UI
  }

  sign(port) { return (port === PORT_A ? this.cfg.invA : port === PORT_B ? this.cfg.invB : this.cfg.invC) ? -1 : 1; }

  async pos(port) {
    const s = await this.nxt.getOutput(port);
    const p = s.rotation * this.sign(port);
    this.onPos?.(port, p);
    return { p, run: s.run, power: s.power };
  }

  // ------------------------------------------------------------ joystick
  // power -100..100 on A. Re-send every ~150 ms while held: with the deadman each
  // command only covers ~0.5 s of travel, so a dropped phone or link stops the robot.
  async drive(power) {
    if (this.sideBusy) return false;
    power = Math.round(power);
    if (power === 0) {
      if (this.drivePower !== 0) await this.nxt.brake(PORT_A);
      this.drivePower = 0;
      return true;
    }
    if (this.drivePower === 0 && !(await this.isHome())) await this.goHome();
    const limit = this.cfg.deadman && !this.softLimit ? Math.max(90, Math.abs(power) * 9 * DEADMAN_S) : 0;
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
      await this.moveTo([PORT_B, PORT_C], [tb, tc], this.cfg.sidePower, gen);
    } finally { this.sideBusy = false; }
  }

  async isHome() {
    const [b, c] = await Promise.all([this.pos(PORT_B), this.pos(PORT_C)]);
    const off = v => Math.min(mod360(v), 360 - mod360(v));
    return off(b.p) <= this.cfg.tolDeg && off(c.p) <= this.cfg.tolDeg;
  }

  async goHome(gen = this.abortGen) {
    this.sideBusy = true;
    try {
      const [b, c] = await Promise.all([this.pos(PORT_B), this.pos(PORT_C)]);
      await this.moveTo([PORT_B, PORT_C], [Math.round(b.p / 360) * 360, Math.round(c.p / 360) * 360], this.cfg.fixPower * 2, gen);
    } finally { this.sideBusy = false; }
  }

  // Calibration: nudge one side motor, then call setHome() when the flat sides face down.
  async jog(port, deg) {
    await this.moveTo([port], [(await this.pos(port)).p + deg], this.cfg.fixPower);
  }
  async setHome() {
    await this.nxt.resetMotorPosition(PORT_B, false);
    await this.nxt.resetMotorPosition(PORT_C, false);
    await Promise.all([this.pos(PORT_B), this.pos(PORT_C)]);
  }

  // ------------------------------------------------------------ path runs
  async forward(cm, gen = this.abortGen) {
    if (!(await this.isHome())) await this.goHome(gen);
    if (this.aTarget === null) this.aTarget = (await this.pos(PORT_A)).p;
    this.aTarget += Math.round(cm / this.cfg.cmPerDegA);
    await this.moveTo([PORT_A], [this.aTarget], this.cfg.drivePower, gen);
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

  // Calibration helper: drive A by whole turns; the user measures the distance.
  async driveTurns(turns) {
    if (!(await this.isHome())) await this.goHome();
    const a = (await this.pos(PORT_A)).p;
    await this.moveTo([PORT_A], [a + turns * 360], this.cfg.drivePower);
  }

  async stop() {
    this.abortGen++;
    this.drivePower = 0;
    await this.nxt.brake();
  }

  // ------------------------------------------------------------ closed loop
  // First pass at full power with the firmware's tacho limit (B+C synchronised when they
  // turn the same way), then slow correction passes until every motor is within tolDeg.
  async moveTo(ports, targets, power, gen = this.abortGen) {
    for (let pass = 0; pass < 5; pass++) {
      if (gen !== this.abortGen) throw new Aborted();
      const now = await Promise.all(ports.map(p => this.pos(p)));
      const err = targets.map((t, i) => t - now[i].p);
      if (err.every(e => Math.abs(e) <= this.cfg.tolDeg)) return;
      const pw = pass === 0 ? power : this.cfg.fixPower;
      const raw = err.map((e, i) => e * this.sign(ports[i]));
      const sync = pass === 0 && ports.length === 2 && Math.sign(raw[0]) === Math.sign(raw[1]) && Math.abs(Math.abs(raw[0]) - Math.abs(raw[1])) < 20;
      const live = ports.filter((_, i) => Math.abs(err[i]) > this.cfg.tolDeg);
      if (this.softLimit) await this.softMove(ports, targets, pw, gen);
      else {
        for (let i = 0; i < ports.length; i++) {
          if (Math.abs(err[i]) <= this.cfg.tolDeg) continue;
          await this.nxt.setOutput(ports[i], Math.sign(raw[i]) * pw, MODE_ON | MODE_BRAKE | MODE_REGULATED, sync ? REG_SYNC : REG_SPEED, 0, RUN_RUNNING, Math.abs(raw[i]));
        }
        const moved = await this.waitIdle(live, gen);
        if (!moved && pass === 0) this.softLimit = true;   // firmware dropped the limited move
      }
    }
    await this.nxt.brake();
    const now = await Promise.all(ports.map(p => this.pos(p)));
    throw new Error(`could not reach target (error ${targets.map((t, i) => Math.round(t - now[i].p)).join(', ')}°)`);
  }

  // Wait until the firmware reports the limited moves done, or nothing has moved for STALL_MS.
  // Returns whether anything moved at all.
  async waitIdle(ports, gen) {
    const t0 = performance.now();
    const start = await Promise.all(ports.map(p => this.pos(p)));
    let last = start.map(s => s.p), lastChange = t0, moved = false;
    for (;;) {
      if (gen !== this.abortGen) throw new Aborted();
      await sleep(POLL_MS);
      const s = await Promise.all(ports.map(p => this.pos(p)));
      const now = performance.now();
      if (s.some((x, i) => Math.abs(x.p - last[i]) > 1)) { lastChange = now; moved = true; }
      last = s.map(x => x.p);
      if (s.every(x => x.run === RUN_IDLE)) return moved || s.some((x, i) => x.p !== start[i].p);
      if (now - lastChange > STALL_MS) { await this.nxt.brake(); return moved; }
      if (now - t0 > MOVE_TIMEOUT_MS) { await this.nxt.brake(); throw new Error('move timed out'); }
    }
  }

  // Fallback without firmware tacho limits: run free and brake just before the target,
  // leading by what the motor covers in one poll round trip.
  async softMove(ports, targets, pw, gen) {
    const dirs = [];
    for (let i = 0; i < ports.length; i++) {
      const e = targets[i] - (await this.pos(ports[i])).p;
      dirs[i] = Math.sign(e);
      if (dirs[i]) await this.nxt.setOutput(ports[i], dirs[i] * this.sign(ports[i]) * pw, MODE_ON | MODE_BRAKE | MODE_REGULATED, REG_SPEED, 0, RUN_RUNNING, 0);
    }
    const lead = 4 + pw * 0.5;
    const t0 = performance.now();
    while (dirs.some(d => d)) {
      if (gen !== this.abortGen) { await this.nxt.brake(); throw new Aborted(); }
      for (let i = 0; i < ports.length; i++) {
        if (!dirs[i]) continue;
        const left = (targets[i] - (await this.pos(ports[i])).p) * dirs[i];
        if (left <= lead) { await this.nxt.brake(ports[i]); dirs[i] = 0; }
      }
      if (performance.now() - t0 > MOVE_TIMEOUT_MS) { await this.nxt.brake(); throw new Error('move timed out'); }
    }
    await sleep(150);
  }
}
