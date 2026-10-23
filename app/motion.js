// Robot motion on top of NXT direct commands.
// Motor A drives all four wheels forward/back. Motors B and C turn the D-profile side
// wheels; one turn moves the robot one lane sideways, and between turns both must rest at
// "home" (RotationCount a multiple of 360, flat side down) or they drag on the floor.
// All targets are absolute RotationCount values, so position errors never accumulate.
import { PORT_A, PORT_B, PORT_C, MODE_ON, MODE_BRAKE, MODE_REGULATED, REG_SPEED, RUN_RUNNING } from './nxt.js';

export const DEFAULTS = {
  invA: false, invB: false, invC: false, invSide: false,
  drivePower: 75,      // joystick full scale and path driving, %
  sidePower: 60,       // side steps, %
  fixPower: 15,        // correction passes, % (regulated 15 % lands within ~1° - measured)
  cmPerDegA: 0.0489,   // 5.6 cm wheel on the motor axle; calibrate in the Brick tab
  stepCm: 5,           // sideways travel per B+C turn
  tolDeg: 4,           // close enough to a target / to home
  deadman: true,       // joystick drive runs in tacho-limited chunks that run out if the link drops
};

const STALL_MS = 800, MOVE_TIMEOUT_MS = 20000, SETTLE_MS = 80, POLL_MS = 5, SPEED_WINDOW_MS = 20;
const BRAKE_S = 0.055;      // braking stops a motor within ~55 ms of travel (measured at 25-80 %)
const DEADMAN_S = 0.6;      // joystick drive keeps going this long after the last command
const MIN_LIMIT = 250;      // firmware 1.01 ignores tacho limits much below ~110°
const sleep = ms => new Promise(r => setTimeout(r, ms));
const mod360 = d => ((d % 360) + 360) % 360;

export class Aborted extends Error { constructor() { super('stopped'); } }

export class Robot {
  constructor(nxt, cfg) {
    this.nxt = nxt;
    this.cfg = cfg;
    this.aTarget = null;     // absolute A target during a path run
    this.rtt = 30;           // ms per request, measured as we go
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
    const limit = this.cfg.deadman ? Math.max(MIN_LIMIT, Math.abs(power) * 9 * DEADMAN_S) : 0;   // ~9 °/s per %
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
  // The stock firmware's own tacho limits can't position precisely (1.01 cuts power ~110°
  // early, lands ±25° off and ignores moves under ~90°), so the loop runs here: drive, watch
  // the counters, and brake when the remaining angle is what the motor needs to stop
  // (~55 ms of travel when braked, measured) plus one link round trip. Then repeat at
  // fixPower until every motor is within tolDeg.
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
        if (left <= v * (BRAKE_S + 1.5 * this.rtt / 1000) + 1) { await this.nxt.brake(ports[i]); live[i] = false; }
      }
      if (cur.some((c, i) => c.p !== prev[i].p)) lastMove = cur.t;
      if (cur.t - lastMove > STALL_MS) { await this.nxt.brake(); throw new Error('motor stalled - is something blocking it?'); }
      if (cur.t - t0 > MOVE_TIMEOUT_MS) { await this.nxt.brake(); throw new Error('move timed out'); }
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
    if (gen !== undefined && gen !== this.abortGen) { await this.nxt.brake(); throw new Aborted(); }
    const t = performance.now();
    const r = [];
    for (const p of ports) r.push(await this.pos(p));
    const now = performance.now();
    this.rtt = this.rtt * 0.8 + (now - t) / ports.length * 0.2;
    r.t = now;
    return r;
  }
}
