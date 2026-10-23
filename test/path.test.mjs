// node test/path.test.mjs
import assert from 'node:assert/strict';
import { test } from 'node:test';
import { planPath, pathStats } from '../app/path.js';

const line = (x0, y0, x1, y1, n = 50) => [...Array(n + 1)].map((_, i) => ({ x: x0 + (x1 - x0) * i / n, y: y0 + (y1 - y0) * i / n }));

test('empty and single point strokes give no moves', () => {
  assert.deepEqual(planPath([]).moves, []);
  assert.deepEqual(planPath([{ x: 3, y: 4 }]).moves, []);
});

test('straight forward and backward', () => {
  assert.deepEqual(planPath(line(10, 10, 10, 60)).moves, [{ type: 'fwd', cm: 50 }]);
  assert.deepEqual(planPath(line(10, 60, 10, 10)).moves, [{ type: 'fwd', cm: -50 }]);
});

test('pure sideways rounds to whole lanes', () => {
  assert.deepEqual(planPath(line(0, 0, 21, 0)).moves, [{ type: 'side', steps: 4 }]);
  assert.deepEqual(planPath(line(0, 0, -12, 0)).moves, [{ type: 'side', steps: -2 }]);
});

test('short sideways wobble below a lane is ignored', () => {
  const pts = line(0, 0, 0, 40).map((p, i) => ({ x: p.x + (i === 0 ? 0 : i % 2 ? 3 : -3), y: p.y }));   // ±0.6 lane
  assert.deepEqual(planPath(pts).moves, [{ type: 'fwd', cm: 40 }]);
});

test('diagonal becomes a staircase with the same end point', () => {
  const { moves, snapped } = planPath(line(0, 0, 30, 60, 200));
  const sideSteps = moves.filter(m => m.type === 'side').reduce((s, m) => s + m.steps, 0);
  const fwd = moves.filter(m => m.type === 'fwd').reduce((s, m) => s + m.cm, 0);
  assert.equal(sideSteps, 6);   // lane k is entered at (k - 0.3) lanes, so 28.5 cm reaches lane 6
  assert.equal(fwd, 60);
  for (let i = 1; i < moves.length; i++) assert.notEqual(moves[i].type, moves[i - 1].type);
  assert.deepEqual(snapped[snapped.length - 1], { x: 30, y: 60 });
});

test('L shape: forward, then sideways, then back', () => {
  const pts = [...line(0, 0, 0, 50), ...line(0, 50, 40, 50), ...line(40, 50, 40, 10)];
  assert.deepEqual(planPath(pts).moves, [{ type: 'fwd', cm: 50 }, { type: 'side', steps: 8 }, { type: 'fwd', cm: -40 }]);
});

test('forward runs are quantized without accumulating rounding', () => {
  const pts = [];
  for (let i = 0; i <= 10; i++) pts.push({ x: i % 2 ? 30 : 0, y: i * 10.4 }, { x: i % 2 ? 30 : 0, y: i * 10.4 + 0.01 });
  const { moves } = planPath(pts);
  const fwd = moves.filter(m => m.type === 'fwd').reduce((s, m) => s + m.cm, 0);
  assert.equal(fwd, 104);
  assert.ok(moves.every(m => m.type !== 'fwd' || Number.isInteger(m.cm)));
});

test('stats', () => {
  assert.deepEqual(pathStats([{ type: 'fwd', cm: 20 }, { type: 'side', steps: -3 }, { type: 'fwd', cm: -5 }]), { fwdCm: 25, steps: 3, sideCm: 15 });
});
