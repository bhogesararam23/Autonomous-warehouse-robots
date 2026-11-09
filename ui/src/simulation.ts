import rawScenarioConfig from '../../scenarios.json';
import { validateScenarioCatalog, type ScenarioCatalog } from './scenarioContract';

const scenarioConfig = rawScenarioConfig as ScenarioCatalog;
validateScenarioCatalog(scenarioConfig);
const warehouse = scenarioConfig.warehouse;

export type Point = { x: number; y: number };
export type State = { point: Point; timeStep: number };

export const GRID_WIDTH = warehouse.width;
export const GRID_HEIGHT = warehouse.height;

export const START: Point = warehouse.start;
export const GOAL: Point = warehouse.goal;

export const SHELVES = warehouse.shelves.map((shelf) => ({
  name: shelf.name,
  x: shelf.x,
  y: shelf.y,
  w: shelf.width,
  h: shelf.height,
}));

export const MAX_PLANNING_TIME = warehouse.maxPlanningTime;

export interface MovingObstacle {
  name: string;
  route: Point[];
}

export type ScenarioId = (typeof scenarioConfig.scenarios)[number]['id'];

export interface WarehouseScenario {
  id: ScenarioId;
  name: string;
  description: string;
  movingObstacles: MovingObstacle[];
}

export const DEFAULT_SCENARIO_ID: ScenarioId = scenarioConfig.defaultScenarioId;

export const SCENARIOS: readonly WarehouseScenario[] = scenarioConfig.scenarios;

export const MOVING_OBSTACLES =
  SCENARIOS.find((scenario) => scenario.id === DEFAULT_SCENARIO_ID)!
    .movingObstacles;

export function buildObstacles(): Set<string> {
  const obstacles = new Set<string>();
  for (const s of SHELVES) {
    for (let x = s.x; x < s.x + s.w; x++) {
      for (let y = s.y; y < s.y + s.h; y++) {
        obstacles.add(`${x},${y}`);
      }
    }
  }
  return obstacles;
}

export function positionAt(route: Point[], timeStep: number): Point {
  return route[timeStep % route.length];
}

function manhattanDistance(a: Point, b: Point): number {
  return Math.abs(a.x - b.x) + Math.abs(a.y - b.y);
}

function getNeighbors(
  point: Point,
  obstacles: Set<string>,
  gridWidth: number,
  gridHeight: number,
): Point[] {
  const { x, y } = point;
  const moves = [
    { x: x + 1, y },
    { x: x - 1, y },
    { x, y: y + 1 },
    { x, y: y - 1 },
    { x, y }, // wait
  ];

  return moves.filter(
    (n) =>
      n.x >= 0 &&
      n.x < gridWidth &&
      n.y >= 0 &&
      n.y < gridHeight &&
      !obstacles.has(`${n.x},${n.y}`)
  );
}

// LCM helper
function gcd(a: number, b: number): number {
  return b === 0 ? a : gcd(b, a % b);
}

function lcm(a: number, b: number): number {
  return (a * b) / gcd(a, b);
}

export function getJointPeriod(movingObstacles: MovingObstacle[]): number {
  if (movingObstacles.length === 0) return 1;
  return movingObstacles.map((o) => o.route.length).reduce((a, b) => lcm(a, b));
}

export function isSafeAfterArrival(
  point: Point,
  arrivalTime: number,
  movingObstacles: MovingObstacle[],
  jointPeriod: number
): boolean {
  if (movingObstacles.length === 0) return true;

  for (let t = arrivalTime; t < arrivalTime + jointPeriod; t++) {
    for (const obstacle of movingObstacles) {
      const pos = positionAt(obstacle.route, t);
      if (point.x === pos.x && point.y === pos.y) {
        return false;
      }
    }
  }
  return true;
}

function hasDynamicCollision(
  current: Point,
  nextPoint: Point,
  nextTime: number,
  movingObstacles: MovingObstacle[]
): boolean {
  const currentTime = nextTime - 1;
  for (const obs of movingObstacles) {
    const obsNow = positionAt(obs.route, currentTime);
    const obsNext = positionAt(obs.route, nextTime);

    const sameCell = nextPoint.x === obsNext.x && nextPoint.y === obsNext.y;
    const edgeSwap =
      current.x === obsNext.x &&
      current.y === obsNext.y &&
      nextPoint.x === obsNow.x &&
      nextPoint.y === obsNow.y;

    if (sameCell || edgeSwap) return true;
  }
  return false;
}

export class MinPriorityQueue<T> {
  private items: { item: T; priority: number; tie: number }[] = [];
  private tieBreaker = 0;

  push(item: T, priority: number) {
    const entry = { item, priority, tie: this.tieBreaker++ };
    this.items.push(entry);

    let index = this.items.length - 1;
    while (index > 0) {
      const parentIndex = Math.floor((index - 1) / 2);
      if (this.compare(this.items[index], this.items[parentIndex]) >= 0) break;
      [this.items[index], this.items[parentIndex]] = [
        this.items[parentIndex],
        this.items[index],
      ];
      index = parentIndex;
    }
  }

  pop(): T | undefined {
    const first = this.items[0];
    const last = this.items.pop();
    if (!first) return undefined;

    if (this.items.length > 0) {
      this.items[0] = last!;
      let index = 0;

      while (true) {
        const leftIndex = index * 2 + 1;
        const rightIndex = leftIndex + 1;
        let smallestIndex = index;

        if (
          leftIndex < this.items.length &&
          this.compare(this.items[leftIndex], this.items[smallestIndex]) < 0
        ) {
          smallestIndex = leftIndex;
        }
        if (
          rightIndex < this.items.length &&
          this.compare(this.items[rightIndex], this.items[smallestIndex]) < 0
        ) {
          smallestIndex = rightIndex;
        }
        if (smallestIndex === index) break;

        [this.items[index], this.items[smallestIndex]] = [
          this.items[smallestIndex],
          this.items[index],
        ];
        index = smallestIndex;
      }
    }

    return first.item;
  }

  isEmpty(): boolean {
    return this.items.length === 0;
  }

  private compare(
    a: { item: T; priority: number; tie: number },
    b: { item: T; priority: number; tie: number },
  ): number {
    if (a.priority === b.priority) return a.tie - b.tie;
    return a.priority - b.priority;
  }
}

export interface PlannerOptions {
  gridWidth?: number;
  gridHeight?: number;
  maxPlanningTime?: number;
}

export class InvalidPlannerInputError extends RangeError {
  readonly code = 'invalid-input' as const;

  constructor(message: string) {
    super(message);
    this.name = 'InvalidPlannerInputError';
  }
}

export class NoSafePathError extends Error {
  readonly code = 'no-safe-path' as const;

  constructor(message = 'No safe path found') {
    super(message);
    this.name = 'NoSafePathError';
  }
}

export type WarehousePlan =
  | { status: 'success'; path: State[]; expandedNodes: number }
  | {
      status: 'failure';
      code: InvalidPlannerInputError['code'] | NoSafePathError['code'];
      message: string;
    };

export function astar(
  start: Point,
  goal: Point,
  obstacles: Set<string>,
  movingObstacles: MovingObstacle[],
  options: PlannerOptions = {},
): { path: State[]; expandedNodes: number } {
  const gridWidth = options.gridWidth ?? GRID_WIDTH;
  const gridHeight = options.gridHeight ?? GRID_HEIGHT;
  const maxPlanningTime = options.maxPlanningTime ?? MAX_PLANNING_TIME;
  if (
    !Number.isInteger(gridWidth) ||
    gridWidth <= 0 ||
    !Number.isInteger(gridHeight) ||
    gridHeight <= 0
  ) {
    throw new InvalidPlannerInputError('Grid dimensions must be positive integers');
  }
  if (!Number.isInteger(maxPlanningTime) || maxPlanningTime < 0) {
    throw new InvalidPlannerInputError(
      'The planning time limit must be a non-negative integer',
    );
  }

  const insideGrid = (point: Point) =>
    Number.isInteger(point.x) &&
    Number.isInteger(point.y) &&
    point.x >= 0 &&
    point.x < gridWidth &&
    point.y >= 0 &&
    point.y < gridHeight;
  for (const obstacle of movingObstacles) {
    if (obstacle.route.length === 0) {
      throw new InvalidPlannerInputError(
        `Moving obstacle '${obstacle.name}' route must not be empty`,
      );
    }
    if (!obstacle.route.every(insideGrid)) {
      throw new InvalidPlannerInputError(
        `Moving obstacle '${obstacle.name}' route points must be integer coordinates inside the grid`,
      );
    }
  }
  if (!insideGrid(start) || !insideGrid(goal)) {
    throw new InvalidPlannerInputError('Start and goal must be inside the grid.');
  }
  if (obstacles.has(`${start.x},${start.y}`) || obstacles.has(`${goal.x},${goal.y}`)) {
    throw new InvalidPlannerInputError(
      'Start and goal must not be blocked by static obstacles.',
    );
  }
  if (
    movingObstacles.some((obstacle) => {
      const position = positionAt(obstacle.route, 0);
      return position.x === start.x && position.y === start.y;
    })
  ) {
    throw new NoSafePathError();
  }

  const startState: State = { point: start, timeStep: 0 };
  const pq = new MinPriorityQueue<State>();
  pq.push(startState, 0);

  const stateKey = (s: State) => `${s.point.x},${s.point.y},${s.timeStep}`;
  
  const cameFrom = new Map<string, State>();
  const costFromStart = new Map<string, number>();
  costFromStart.set(stateKey(startState), 0);
  
  const visited = new Set<string>();
  let expandedNodes = 0;

  const jointPeriod = getJointPeriod(movingObstacles);

  while (!pq.isEmpty()) {
    const currentState = pq.pop()!;
    const curKey = stateKey(currentState);

    if (visited.has(curKey)) continue;
    visited.add(curKey);
    expandedNodes++;

    if (currentState.point.x === goal.x && currentState.point.y === goal.y) {
      if (isSafeAfterArrival(currentState.point, currentState.timeStep, movingObstacles, jointPeriod)) {
        // Rebuild path
        const path: State[] = [currentState];
        let cur = currentState;
        while (cameFrom.has(stateKey(cur))) {
          cur = cameFrom.get(stateKey(cur))!;
          path.push(cur);
        }
        return { path: path.reverse(), expandedNodes };
      }
    }

    if (currentState.timeStep >= maxPlanningTime) continue;

    const neighbors = getNeighbors(currentState.point, obstacles, gridWidth, gridHeight);
    const nextTime = currentState.timeStep + 1;

    for (const n of neighbors) {
      if (hasDynamicCollision(currentState.point, n, nextTime, movingObstacles)) {
        continue;
      }

      const nextState: State = { point: n, timeStep: nextTime };
      const nextKey = stateKey(nextState);
      const newCost = costFromStart.get(curKey)! + 1;

      if (newCost < (costFromStart.get(nextKey) ?? Infinity)) {
        costFromStart.set(nextKey, newCost);
        const priority = newCost + manhattanDistance(n, goal);
        cameFrom.set(nextKey, currentState);
        pq.push(nextState, priority);
      }
    }
  }

  throw new NoSafePathError();
}

export function planWarehouse(
  start: Point,
  goal: Point,
  obstacles: Set<string>,
  movingObstacles: MovingObstacle[],
  options: PlannerOptions = {},
): WarehousePlan {
  try {
    return {
      status: 'success',
      ...astar(start, goal, obstacles, movingObstacles, options),
    };
  } catch (error: unknown) {
    if (error instanceof InvalidPlannerInputError || error instanceof NoSafePathError) {
      return { status: 'failure', code: error.code, message: error.message };
    }
    throw error;
  }
}
