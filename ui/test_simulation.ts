import {
  astar,
  buildObstacles,
  MOVING_OBSTACLES,
  getJointPeriod,
  isSafeAfterArrival,
  START,
  GOAL
} from './src/simulation';

import assert from 'assert';

console.log("Running TypeScript simulation tests...");

const obstacles = buildObstacles();

// Test safe goal
try {
  const { path } = astar(START, GOAL, obstacles, MOVING_OBSTACLES);
  const jointPeriod = getJointPeriod(MOVING_OBSTACLES);
  assert(path.length > 0);
  const finalState = path[path.length - 1];
  assert(isSafeAfterArrival(finalState.point, finalState.timeStep, MOVING_OBSTACLES, jointPeriod));
  console.log("Safe goal test passed.");
} catch (e) {
  console.error("Safe goal test failed:", e);
  process.exit(1);
}

// Test unsafe goal
try {
  astar(START, { x: 20, y: 9 }, obstacles, MOVING_OBSTACLES);
  console.error("Unsafe goal test failed: Expected an error but got a path.");
  process.exit(1);
} catch (e) {
  assert(e.message === "No safe path found");
  console.log("Unsafe goal test passed.");
}

console.log("All TS tests passed.");
