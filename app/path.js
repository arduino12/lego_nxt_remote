// Turns a drawn stroke into moves this robot can make: forward/back along y (motor A)
// and sideways along x in whole lanes (one B+C turn = one lane = stepCm).
// Units are cm. The robot starts at the first point, facing +y.

// Hysteresis keeps a hand-drawn line that wobbles on a lane border from
// producing back-and-forth side steps.
const HYSTERESIS = 0.2;   // fraction of a lane past the midpoint before switching

export function planPath(points, { stepCm = 5, resCm = 1, minCm = 1 } = {}) {
  if (!points || points.length < 2) return { moves: [], snapped: points?.length ? [{ ...points[0] }] : [] };
  const x0 = points[0].x, y0 = points[0].y;
  const quant = v => Math.round(v / resCm) * resCm;

  // 1. Walk the stroke, tracking the current lane and where each lane change happens.
  let lane = 0, yLast = 0;
  const raw = [];   // {type, cm | steps}
  for (const p of points) {
    const x = p.x - x0, y = p.y - y0;
    const f = x / stepCm;
    let target = lane;
    while (f > target + 0.5 + HYSTERESIS) target++;
    while (f < target - 0.5 - HYSTERESIS) target--;
    if (target !== lane) {
      raw.push({ type: 'fwd', cm: y - yLast });
      raw.push({ type: 'side', steps: target - lane });
      lane = target; yLast = y;
    }
  }
  const last = points[points.length - 1];
  raw.push({ type: 'fwd', cm: last.y - y0 - yLast });

  // 2. Quantize forward runs against absolute y (so rounding never accumulates),
  //    drop the tiny ones and merge neighbours of the same kind.
  const moves = [];
  let yAbs = 0, yQ = 0;
  for (const m of raw) {
    if (m.type === 'fwd') {
      yAbs += m.cm;
      const cm = quant(yAbs) - yQ;
      if (Math.abs(cm) < minCm) continue;
      yQ += cm;
      push(moves, { type: 'fwd', cm });
    } else push(moves, { ...m });
  }

  // 3. The path the robot will actually drive, for the preview.
  const snapped = [{ x: x0, y: y0 }];
  let cx = x0, cy = y0;
  for (const m of moves) {
    if (m.type === 'fwd') cy += m.cm; else cx += m.steps * stepCm;
    snapped.push({ x: cx, y: cy });
  }
  return { moves, snapped };
}

function push(moves, m) {
  const prev = moves[moves.length - 1];
  if (prev && prev.type === m.type) {
    if (m.type === 'fwd') prev.cm = Math.round((prev.cm + m.cm) * 1000) / 1000;
    else prev.steps += m.steps;
    if ((prev.type === 'fwd' && prev.cm === 0) || (prev.type === 'side' && prev.steps === 0)) moves.pop();
  } else moves.push(m);
}

export function pathStats(moves, stepCm = 5) {
  let fwd = 0, steps = 0;
  for (const m of moves) { if (m.type === 'fwd') fwd += Math.abs(m.cm); else steps += Math.abs(m.steps); }
  return { fwdCm: fwd, steps, sideCm: steps * stepCm };
}
