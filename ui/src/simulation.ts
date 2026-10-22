export type Point = { x: number; y: number };
export type State = { point: Point; timeStep: number };

export const GRID_WIDTH = 24;
export const GRID_HEIGHT = 14;

export const START: Point = { x: 1, y: 1 };
export const GOAL: Point = { x: 22, y: 12 };

export const SHELVES = [
  { name: 'A', x: 5, y: 0, w: 2, h: 9 },
  { name: 'B', x: 10, y: 5, w: 2, h: 9 },
  { name: 'C', x: 15, y: 0, w: 2, h: 9 },
];

export const MAX_PLANNING_TIME = 120;

export interface MovingObstacle {
  name: string;
  route: Point[];
}

export const MOVING_OBSTACLES: MovingObstacle[] = [
  {
    name: 'Forklift-1',
    route: [
      { x: 7, y: 9 },
      { x: 8, y: 9 },
      { x: 9, y: 9 },
      { x: 8, y: 9 },
    ],
  },
  {
    name: 'Robot-2',
    route: [
      { x: 18, y: 9 },
      { x: 19, y: 9 },
      { x: 20, y: 9 },
      { x: 21, y: 9 },
      { x: 20, y: 9 },
      { x: 19, y: 9 },
    ],
  },
];

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

function getNeighbors(point: Point, obstacles: Set<string>): Point[] {
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
      n.x < GRID_WIDTH &&
      n.y >= 0 &&
      n.y < GRID_HEIGHT &&
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

class PriorityQueue<T> {
  private items: { item: T; priority: number; tie: number }[] = [];
  private tieBreaker = 0;

  push(item: T, priority: number) {
    this.items.push({ item, priority, tie: this.tieBreaker++ });
    this.items.sort((a, b) => {
      if (a.priority === b.priority) return a.tie - b.tie;
      return a.priority - b.priority;
    });
  }

  pop(): T | undefined {
    return this.items.shift()?.item;
  }

  isEmpty(): boolean {
    return this.items.length === 0;
  }
}

export function astar(
  start: Point,
  goal: Point,
  obstacles: Set<string>,
  movingObstacles: MovingObstacle[]
): { path: State[]; expandedNodes: number } {
  const startState: State = { point: start, timeStep: 0 };
  const pq = new PriorityQueue<State>();
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

    if (currentState.timeStep >= MAX_PLANNING_TIME) continue;

    const neighbors = getNeighbors(currentState.point, obstacles);
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

  throw new Error("No safe path found");
}
