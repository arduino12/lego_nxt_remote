// Snake ("lawn-mower") grid for this robot: drive a row forward with motor A, shift sideways
// a whole number of B+C turns, drive the next row back, and so on. Units are cm.
// The robot starts at the bottom-left corner of the first row, facing up (+y).

export function snakePath({ rows, rowCm, sideTurns, stepCm = 5, sideDir = 1 }) {
  rows = Math.max(0, Math.floor(rows));
  const moves = [];
  for (let r = 0; r < rows; r++) {
    moves.push({ type: 'fwd', cm: r % 2 ? -rowCm : rowCm, row: r });
    if (r < rows - 1 && sideTurns > 0) moves.push({ type: 'side', steps: sideDir * sideTurns, row: r });
  }
  // Corner points for the preview: one per move end.
  const pts = [{ x: 0, y: 0 }];
  let x = 0, y = 0;
  for (const m of moves) {
    if (m.type === 'fwd') y += m.cm; else x += m.steps * stepCm;
    pts.push({ x, y });
  }
  return { moves, pts };
}

export function pathStats(moves, stepCm = 5) {
  let fwd = 0, steps = 0;
  for (const m of moves) { if (m.type === 'fwd') fwd += Math.abs(m.cm); else steps += Math.abs(m.steps); }
  return { fwdCm: fwd, steps, sideCm: steps * stepCm };
}
