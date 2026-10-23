// node --test test/
import assert from 'node:assert/strict';
import { test } from 'node:test';
import { snakePath, pathStats } from '../app/path.js';

test('three rows: forward, shift, back, shift, forward', () => {
  const { moves, pts } = snakePath({ rows: 3, rowCm: 100, sideTurns: 2 });
  assert.deepEqual(moves.map(m => m.type === 'fwd' ? m.cm : `side ${m.steps}`), [100, 'side 2', -100, 'side 2', 100]);
  assert.deepEqual(pts[pts.length - 1], { x: 20, y: 100 });
});

test('even row count ends back at the start line', () => {
  const { pts } = snakePath({ rows: 4, rowCm: 50, sideTurns: 1 });
  assert.deepEqual(pts[pts.length - 1], { x: 15, y: 0 });
});

test('shift to the left', () => {
  const { moves, pts } = snakePath({ rows: 2, rowCm: 30, sideTurns: 3, sideDir: -1 });
  assert.equal(moves[1].steps, -3);
  assert.deepEqual(pts[pts.length - 1], { x: -15, y: 0 });
});

test('one row has no side shift; zero rows is empty', () => {
  assert.deepEqual(snakePath({ rows: 1, rowCm: 40, sideTurns: 2 }).moves.map(m => m.type), ['fwd']);
  assert.deepEqual(snakePath({ rows: 0, rowCm: 40, sideTurns: 2 }).moves, []);
});

test('stats', () => {
  const { moves } = snakePath({ rows: 3, rowCm: 100, sideTurns: 2 });
  assert.deepEqual(pathStats(moves), { fwdCm: 300, steps: 4, sideCm: 20 });
});
