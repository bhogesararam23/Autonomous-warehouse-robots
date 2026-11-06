import assert from 'node:assert/strict';
import sharedScenarioCatalog from '../scenarios.json';
import {
  astar,
  buildObstacles,
  DEFAULT_SCENARIO_ID,
  getJointPeriod,
  GOAL,
  GRID_HEIGHT,
  GRID_WIDTH,
  isSafeAfterArrival,
  MinPriorityQueue,
  MOVING_OBSTACLES,
  positionAt,
  SCENARIOS,
  START,
  type MovingObstacle,
  type Point,
} from './src/simulation';
import { getPlaybackShortcutAction } from './src/keyboardShortcuts';

function samePoint(a: Point, b: Point): boolean {
  return a.x === b.x && a.y === b.y;
}

function shortestPathLength(
  start: Point,
  goal: Point,
  obstacles: Set<string>,
  gridWidth: number,
  gridHeight: number,
): number | null {
  const frontier = [{ point: start, distance: 0 }];
  const visited = new Set([`${start.x},${start.y}`]);
  for (let index = 0; index < frontier.length; index++) {
    const current = frontier[index];
    for (const point of [
      { x: current.point.x + 1, y: current.point.y },
      { x: current.point.x - 1, y: current.point.y },
      { x: current.point.x, y: current.point.y + 1 },
      { x: current.point.x, y: current.point.y - 1 },
    ]) {
      const key = `${point.x},${point.y}`;
      if (
        point.x < 0 ||
        point.x >= gridWidth ||
        point.y < 0 ||
        point.y >= gridHeight ||
        obstacles.has(key) ||
        visited.has(key)
      ) {
        continue;
      }
      if (samePoint(point, goal)) return current.distance + 1;
      visited.add(key);
      frontier.push({ point, distance: current.distance + 1 });
    }
  }
  return null;
}

function assertValidPath(
  path: ReturnType<typeof astar>['path'],
  obstacles: Set<string>,
  movingObstacles: MovingObstacle[],
  gridWidth = GRID_WIDTH,
  gridHeight = GRID_HEIGHT,
): void {
  assert.ok(path.length > 0, 'path should not be empty');

  path.forEach((state, index) => {
    assert.equal(state.timeStep, index, 'time steps should be consecutive from zero');
    assert.ok(state.point.x >= 0 && state.point.x < gridWidth, 'path x should be in bounds');
    assert.ok(state.point.y >= 0 && state.point.y < gridHeight, 'path y should be in bounds');
    assert.ok(
      !obstacles.has(`${state.point.x},${state.point.y}`),
      `path should not cross a shelf at ${state.point.x},${state.point.y}`,
    );

    if (index === 0) {
      for (const obstacle of movingObstacles) {
        assert.ok(
          !samePoint(state.point, positionAt(obstacle.route, state.timeStep)),
          `path should not start on ${obstacle.name} at time ${state.timeStep}`,
        );
      }
      return;
    }

    const previous = path[index - 1];
    const distance =
      Math.abs(state.point.x - previous.point.x) +
      Math.abs(state.point.y - previous.point.y);
    assert.ok(distance <= 1, 'each step should move at most one cell');

    for (const obstacle of movingObstacles) {
      const obstacleBefore = positionAt(obstacle.route, previous.timeStep);
      const obstacleAfter = positionAt(obstacle.route, state.timeStep);
      assert.ok(
        !samePoint(state.point, obstacleAfter),
        `path should avoid ${obstacle.name} at time ${state.timeStep}`,
      );
      assert.ok(
        !(samePoint(previous.point, obstacleAfter) && samePoint(state.point, obstacleBefore)),
        `path should not edge-swap with ${obstacle.name} at time ${state.timeStep}`,
      );
    }
  });
}

console.log('Running TypeScript simulation tests...');
const shelves = buildObstacles();
assert.equal(DEFAULT_SCENARIO_ID, 'standard', 'the original scenario should remain the default');
assert.equal(
  DEFAULT_SCENARIO_ID,
  sharedScenarioCatalog.defaultScenarioId,
  'the UI default should come from the shared scenario catalog',
);
assert.deepEqual(
  SCENARIOS,
  sharedScenarioCatalog.scenarios,
  'the UI scenario metadata and obstacle routes should match the shared catalog',
);
assert.equal(
  SCENARIOS.find((scenario) => scenario.id === DEFAULT_SCENARIO_ID)?.movingObstacles,
  MOVING_OBSTACLES,
  'the default scenario should retain the original moving obstacles',
);
const peakTrafficScenario = SCENARIOS.find((scenario) => scenario.id === 'peak-traffic');
assert.ok(peakTrafficScenario, 'the peak-traffic variant should be available');
assert.equal(
  peakTrafficScenario.movingObstacles.length,
  MOVING_OBSTACLES.length + 1,
  'the variant should add one moving obstacle to the original scenario',
);

for (const scenario of SCENARIOS) {
  const jointPeriod = getJointPeriod(scenario.movingObstacles);
  for (const obstacle of scenario.movingObstacles) {
    for (const point of obstacle.route) {
      assert.ok(
        point.x >= 0 && point.x < GRID_WIDTH && point.y >= 0 && point.y < GRID_HEIGHT,
        `${scenario.name}: ${obstacle.name} route should stay inside the warehouse`,
      );
      assert.ok(
        !shelves.has(`${point.x},${point.y}`),
        `${scenario.name}: ${obstacle.name} route should not pass through a shelf`,
      );
    }
  }

  for (let timeStep = 0; timeStep < jointPeriod; timeStep++) {
    const occupiedCells = new Set<string>();
    for (const obstacle of scenario.movingObstacles) {
      const point = positionAt(obstacle.route, timeStep);
      const key = `${point.x},${point.y}`;
      assert.ok(
        !occupiedCells.has(key),
        `${scenario.name}: moving obstacles should not occupy the same cell at time ${timeStep}`,
      );
      occupiedCells.add(key);
    }
  }

  const { path } = astar(START, GOAL, shelves, scenario.movingObstacles);
  assert.deepEqual(path[0].point, START, `${scenario.name}: path should begin at the start`);
  assert.deepEqual(path.at(-1)?.point, GOAL, `${scenario.name}: path should reach the goal`);
  assertValidPath(path, shelves, scenario.movingObstacles);
  const finalState = path[path.length - 1];
  assert.ok(
    isSafeAfterArrival(finalState.point, finalState.timeStep, scenario.movingObstacles, jointPeriod),
    `${scenario.name}: goal should remain safe for a complete obstacle cycle after arrival`,
  );
}
console.log('Default and peak-traffic plans passed route and safe-goal tests.');

const baselineGridWidth = 6;
const baselineGridHeight = 5;
const baselineStart = { x: 0, y: 2 };
const baselineGoal = { x: 5, y: 2 };
const baselineObstacles = new Set(
  Array.from({ length: baselineGridHeight - 1 }, (_, y) => `2,${y}`),
);
const shortestBaselineLength = shortestPathLength(
  baselineStart,
  baselineGoal,
  baselineObstacles,
  baselineGridWidth,
  baselineGridHeight,
);
const baselinePlan = astar(baselineStart, baselineGoal, baselineObstacles, [], {
  gridWidth: baselineGridWidth,
  gridHeight: baselineGridHeight,
  maxPlanningTime: 20,
});
assert.equal(
  shortestBaselineLength,
  9,
  'the shelf wall should force a 9-step detour',
);
assert.equal(
  baselinePlan.path.length - 1,
  shortestBaselineLength,
  'A* path length should match the independent BFS shortest-path baseline',
);
assertValidPath(
  baselinePlan.path,
  baselineObstacles,
  [],
  baselineGridWidth,
  baselineGridHeight,
);
console.log('A* path length matches the independent BFS baseline.');

const largeGridWidth = 64;
const largeGridHeight = 40;
const largeStart = { x: 1, y: 1 };
const largeGoal = { x: 62, y: 38 };
const gateRows = new Map<number, Set<number>>([
  [12, new Set([6, 7, 8])],
  [24, new Set([14, 15, 16])],
  [36, new Set([22, 23, 24])],
  [48, new Set([30, 31, 32])],
  [60, new Set([34, 35, 36])],
]);
const largeWarehouseObstacles = new Set<string>();
for (const [wallX, openRows] of gateRows) {
  for (let y = 0; y < largeGridHeight; y++) {
    if (!openRows.has(y)) largeWarehouseObstacles.add(`${wallX},${y}`);
  }
}
const largeWarehouseMovers: MovingObstacle[] = [
  {
    name: 'Gate patrol 1',
    route: [{ x: 12, y: 6 }, { x: 12, y: 7 }, { x: 12, y: 8 }, { x: 12, y: 7 }],
  },
  {
    name: 'Gate patrol 2',
    route: [
      { x: 24, y: 14 },
      { x: 24, y: 15 },
      { x: 24, y: 16 },
      { x: 24, y: 15 },
      { x: 24, y: 14 },
    ],
  },
  {
    name: 'Cross-aisle patrol',
    route: [{ x: 40, y: 27 }, { x: 41, y: 27 }, { x: 42, y: 27 }, { x: 41, y: 27 }],
  },
];
const largeWarehousePlan = astar(
  largeStart,
  largeGoal,
  largeWarehouseObstacles,
  largeWarehouseMovers,
  {
    gridWidth: largeGridWidth,
    gridHeight: largeGridHeight,
    maxPlanningTime: 180,
  },
);
assert.ok(largeWarehousePlan.expandedNodes > 0, 'large-grid search should expand states');
assert.deepEqual(largeWarehousePlan.path[0], { point: largeStart, timeStep: 0 });
assert.deepEqual(largeWarehousePlan.path.at(-1)?.point, largeGoal);
assertValidPath(
  largeWarehousePlan.path,
  largeWarehouseObstacles,
  largeWarehouseMovers,
  largeGridWidth,
  largeGridHeight,
);
const largeWarehouseFinal = largeWarehousePlan.path.at(-1)!;
assert.ok(
  isSafeAfterArrival(
    largeWarehouseFinal.point,
    largeWarehouseFinal.timeStep,
    largeWarehouseMovers,
    getJointPeriod(largeWarehouseMovers),
  ),
  'large-grid goal should remain safe for a complete moving-obstacle cycle',
);
console.log('Large-warehouse route and safe-goal tests passed.');

const corridorCells = new Set(['0,0', '1,0', '2,0']);
const corridorWalls = new Set<string>();
for (let y = 0; y < GRID_HEIGHT; y++) {
  for (let x = 0; x < GRID_WIDTH; x++) {
    if (!corridorCells.has(`${x},${y}`)) corridorWalls.add(`${x},${y}`);
  }
}

const crossingObstacle: MovingObstacle = {
  name: 'Crossing obstacle',
  route: [{ x: 1, y: 0 }, { x: 0, y: 0 }],
};
assert.throws(
  () => astar({ x: 0, y: 0 }, { x: 2, y: 0 }, corridorWalls, [crossingObstacle]),
  { message: 'No safe path found' },
  'planner should reject a corridor route that requires an edge swap',
);
console.log('Edge-swap collision test passed.');

const blockingObstacle: MovingObstacle = {
  name: 'Blocking obstacle',
  route: [{ x: 2, y: 1 }, { x: 1, y: 0 }],
};
const delayedPath = astar(
  { x: 0, y: 0 },
  { x: 2, y: 0 },
  corridorWalls,
  [blockingObstacle],
).path;
assertValidPath(delayedPath, corridorWalls, [blockingObstacle]);
assert.deepEqual(
  delayedPath[1],
  { point: { x: 0, y: 0 }, timeStep: 1 },
  'planner should wait rather than enter a cell occupied by a moving obstacle',
);
assert.deepEqual(delayedPath.at(-1)?.point, { x: 2, y: 0 });
console.log('Same-cell collision and wait test passed.');

assert.throws(
  () =>
    astar(
      { x: 1, y: 1 },
      { x: 3, y: 1 },
      new Set(),
      [{ name: 'Starting obstacle', route: [{ x: 1, y: 1 }, { x: 2, y: 1 }] }],
    ),
  { message: 'No safe path found' },
  'planner should reject a start cell occupied at time zero',
);
console.log('Initial dynamic-collision test passed.');

const invalidBoundaryRoutes = [
  { label: 'out-of-bounds start', start: { x: -1, y: 1 }, goal: { x: 1, y: 1 } },
  { label: 'out-of-bounds goal', start: { x: 1, y: 1 }, goal: { x: GRID_WIDTH, y: 12 } },
  { label: 'shelf-occupied start', start: { x: 5, y: 0 }, goal: { x: 22, y: 12 } },
  { label: 'shelf-occupied goal', start: { x: 1, y: 1 }, goal: { x: 5, y: 0 } },
];
for (const { label, start, goal } of invalidBoundaryRoutes) {
  assert.throws(
    () => astar(start, goal, shelves, []),
    { message: 'No safe path found' },
    `planner should reject an ${label}`,
  );
}
console.log('Out-of-bounds and shelf-occupied endpoint tests passed.');

const zeroStepPlan = astar(
  { x: 0, y: 0 },
  { x: 0, y: 0 },
  new Set(),
  [],
  { gridWidth: 1, gridHeight: 1, maxPlanningTime: 0 },
);
assert.deepEqual(zeroStepPlan.path, [{ point: { x: 0, y: 0 }, timeStep: 0 }]);
assert.equal(zeroStepPlan.expandedNodes, 1);

assert.throws(
  () =>
    astar(
      { x: 0, y: 0 },
      { x: 2, y: 0 },
      new Set(['1,0']),
      [],
      { gridWidth: 3, gridHeight: 1, maxPlanningTime: 3 },
    ),
  { message: 'No safe path found' },
  'planner should report when static obstacles make the goal unreachable',
);
assert.throws(
  () =>
    astar(
      { x: 0, y: 0 },
      { x: 2, y: 0 },
      new Set(),
      [],
      { gridWidth: 3, gridHeight: 1, maxPlanningTime: 1 },
    ),
  { message: 'No safe path found' },
  'planner should reject routes longer than the planning horizon',
);

const returningObstacle: MovingObstacle = {
  name: 'Returning obstacle',
  route: [{ x: 1, y: 1 }, { x: 0, y: 0 }],
};
assert.throws(
  () =>
    astar(
      { x: 0, y: 0 },
      { x: 0, y: 0 },
      new Set(),
      [returningObstacle],
      { gridWidth: 2, gridHeight: 2, maxPlanningTime: 0 },
    ),
  { message: 'No safe path found' },
  'planner should not accept a start-equals-goal state that is unsafe later in the cycle',
);

const invalidPlannerInputs = [
  { label: 'fractional width', options: { gridWidth: 1.5 } },
  { label: 'infinite width', options: { gridWidth: Number.POSITIVE_INFINITY } },
  { label: 'fractional height', options: { gridHeight: 1.5 } },
  { label: 'NaN height', options: { gridHeight: Number.NaN } },
  { label: 'fractional planning horizon', options: { maxPlanningTime: 0.5 } },
  { label: 'infinite planning horizon', options: { maxPlanningTime: Number.POSITIVE_INFINITY } },
  { label: 'NaN planning horizon', options: { maxPlanningTime: Number.NaN } },
] as const;
for (const { label, options } of invalidPlannerInputs) {
  assert.throws(
    () => astar({ x: 0, y: 0 }, { x: 0, y: 0 }, new Set(), [], options),
    RangeError,
    `planner should reject a ${label}`,
  );
}
assert.throws(
  () => astar({ x: 0.5, y: 0 }, { x: 0.5, y: 0 }, new Set(), []),
  { message: 'No safe path found' },
  'planner should reject non-integer point coordinates',
);
assert.throws(
  () => astar({ x: 0, y: 0 }, { x: 0.5, y: 0 }, new Set(), []),
  { message: 'No safe path found' },
  'planner should reject non-integer goal coordinates',
);
const invalidMovingObstacleRoutes: { label: string; route: Point[] }[] = [
  { label: 'empty route', route: [] },
  { label: 'fractional x', route: [{ x: 1.5, y: 1 }] },
  { label: 'fractional y', route: [{ x: 1, y: 1.5 }] },
  { label: 'out-of-bounds x', route: [{ x: 3, y: 1 }] },
  { label: 'out-of-bounds y', route: [{ x: 1, y: 3 }] },
];
for (const { label, route } of invalidMovingObstacleRoutes) {
  assert.throws(
    () =>
      astar(
        { x: 0, y: 0 },
        { x: 0, y: 0 },
        new Set(),
        [{ name: label, route }],
        { gridWidth: 3, gridHeight: 3, maxPlanningTime: 0 },
      ),
    RangeError,
    `planner should reject a moving obstacle with ${label}`,
  );
}
console.log('Zero-step, unreachable-route, and input-validation tests passed.');

assert.throws(
  () => astar(START, { x: 20, y: 9 }, shelves, MOVING_OBSTACLES),
  { message: 'No safe path found' },
  'planner should reject a goal occupied by a moving obstacle during its cycle',
);
console.log('Unsafe-goal test passed.');

const queue = new MinPriorityQueue<string>();
assert.equal(queue.isEmpty(), true, 'a new priority queue should be empty');
assert.equal(queue.pop(), undefined, 'popping an empty priority queue should return undefined');
queue.push('priority five', 5);
queue.push('first priority one', 1);
queue.push('second priority one', 1);
queue.push('priority negative two', -2);
assert.equal(queue.pop(), 'priority negative two', 'the minimum priority should be popped first');
queue.push('third priority one', 1);
assert.deepEqual(
  [queue.pop(), queue.pop(), queue.pop(), queue.pop()],
  ['first priority one', 'second priority one', 'third priority one', 'priority five'],
  'equal priorities should retain insertion order as new items are added and removed',
);
assert.equal(queue.isEmpty(), true, 'the queue should be empty after its final item is removed');
console.log('Min-heap ordering and stable-tie tests passed.');

type ShortcutTestOptions = {
  target?: EventTarget | null;
  altKey?: boolean;
  ctrlKey?: boolean;
  metaKey?: boolean;
  shiftKey?: boolean;
  repeat?: boolean;
  defaultPrevented?: boolean;
};

function shortcutAction(key: string, options: ShortcutTestOptions = {}) {
  return getPlaybackShortcutAction({
    key,
    target: options.target ?? null,
    altKey: options.altKey ?? false,
    ctrlKey: options.ctrlKey ?? false,
    metaKey: options.metaKey ?? false,
    shiftKey: options.shiftKey ?? false,
    repeat: options.repeat ?? false,
    defaultPrevented: options.defaultPrevented ?? false,
  });
}

function interactiveTarget(selector: string, isContentEditable = false): EventTarget {
  return {
    isContentEditable,
    closest: (selectors: string) =>
      selectors.split(', ').includes(selector) ? ({} as Element) : null,
  } as unknown as EventTarget;
}

assert.equal(shortcutAction(' '), 'toggle-play', 'Space should play or pause');
assert.equal(shortcutAction('Spacebar'), 'toggle-play', 'legacy Space key naming should also work');
assert.equal(shortcutAction('ArrowLeft'), 'step-back', 'Left Arrow should step backward');
assert.equal(shortcutAction('ArrowRight'), 'step-forward', 'Right Arrow should step forward');
assert.equal(shortcutAction('p'), null, 'unmapped keys should be left alone');

for (const selector of [
  'input',
  'textarea',
  'select',
  'button',
  'a[href]',
  '[role]',
  '[tabindex]',
  '[contenteditable]',
]) {
  assert.equal(
    shortcutAction(' ', { target: interactiveTarget(selector) }),
    null,
    `Space should not hijack a focused ${selector} control`,
  );
}
assert.equal(
  shortcutAction('ArrowLeft', { target: interactiveTarget('p', true) }),
  null,
  'shortcuts should not hijack a content-editable region',
);
for (const modifier of ['altKey', 'ctrlKey', 'metaKey', 'shiftKey', 'repeat', 'defaultPrevented'] as const) {
  assert.equal(
    shortcutAction('ArrowRight', { [modifier]: true }),
    null,
    `${modifier} key events should be left to the browser or focused control`,
  );
}
console.log('Playback keyboard-shortcut mapping and focus-safety tests passed.');
console.log('All TypeScript tests passed.');
