import {
  buildObstacles,
  DEFAULT_SCENARIO_ID,
  getJointPeriod,
  GRID_HEIGHT,
  GRID_WIDTH,
  isSafeAfterArrival,
  planWarehouse,
  SCENARIOS,
  type Point,
  type ScenarioId,
} from './simulation';
import { InvalidPlannerInputError } from './simulation';

export type EpisodeEvent =
  | {
      type: 'episode-started';
      seed: number;
      scenarioId: string;
      start: Point;
      goal: Point;
    }
  | { type: 'robot-position'; timeStep: number; x: number; y: number }
  | { type: 'episode-finished'; status: 'success'; pathLength: number; expandedNodes: number }
  | { type: 'episode-finished'; status: 'failure'; code: 'invalid-input' | 'no-safe-path' };

const UINT32_MAX = 0xffffffff;
const INITIAL_STATE = 0x6d2b79f5;

function validateSeed(seed: number): void {
  if (!Number.isInteger(seed) || !Number.isSafeInteger(seed) || seed < 0 || seed > UINT32_MAX) {
    throw new InvalidPlannerInputError('Seed must be an integer in [0, 0xffffffff]');
  }
}

class XorShift32 {
  private state: number;

  constructor(seed: number) {
    validateSeed(seed);
    this.state = (seed === 0 ? INITIAL_STATE : seed) >>> 0;
  }

  nextU32(): number {
    let x = this.state;
    x = (x ^ ((x << 13) >>> 0)) >>> 0;
    x = (x ^ (x >>> 17)) >>> 0;
    x = (x ^ ((x << 5) >>> 0)) >>> 0;
    this.state = x >>> 0;
    return this.state;
  }
}

function scenarioFor(id: string) {
  const scenario = SCENARIOS.find((candidate) => candidate.id === id);
  if (!scenario) throw new InvalidPlannerInputError(`Unknown scenario '${id}'`);
  return scenario;
}

function pointKey(point: Point): string {
  return `${point.x},${point.y}`;
}

function choose<T>(items: T[], random: XorShift32): T {
  if (items.length === 0) throw new InvalidPlannerInputError('No eligible planner candidates');
  return items[random.nextU32() % items.length];
}

export function generatePlannerEpisode(
  seed: number,
  scenarioId: ScenarioId = DEFAULT_SCENARIO_ID,
): EpisodeEvent[] {
  validateSeed(seed);
  const scenario = scenarioFor(scenarioId);
  const random = new XorShift32(seed);
  const staticObstacles = buildObstacles();
  const jointPeriod = getJointPeriod(scenario.movingObstacles);
  const occupiedAtZero = new Set(
    scenario.movingObstacles.map((obstacle) => pointKey(obstacle.route[0])),
  );
  const startCandidates: Point[] = [];
  const goalCandidates: Point[] = [];

  for (let y = 0; y < GRID_HEIGHT; y++) {
    for (let x = 0; x < GRID_WIDTH; x++) {
      const point = { x, y };
      if (staticObstacles.has(pointKey(point))) continue;
      if (!occupiedAtZero.has(pointKey(point))) startCandidates.push(point);
      if (isSafeAfterArrival(point, 0, scenario.movingObstacles, jointPeriod)) {
        goalCandidates.push(point);
      }
    }
  }

  const start = choose(startCandidates, random);
  const availableGoals = goalCandidates.filter((point) => pointKey(point) !== pointKey(start));
  const goal = choose(availableGoals, random);
  const events: EpisodeEvent[] = [
    { type: 'episode-started', seed, scenarioId, start, goal },
  ];
  const result = planWarehouse(start, goal, staticObstacles, scenario.movingObstacles);

  if (result.status === 'failure') {
    events.push({ type: 'episode-finished', status: 'failure', code: result.code });
    return events;
  }

  for (const state of result.path) {
    events.push({
      type: 'robot-position',
      timeStep: state.timeStep,
      x: state.point.x,
      y: state.point.y,
    });
  }
  events.push({
    type: 'episode-finished',
    status: 'success',
    pathLength: result.path.length - 1,
    expandedNodes: result.expandedNodes,
  });
  return events;
}

function stableValue(value: unknown): unknown {
  if (Array.isArray(value)) return value.map(stableValue);
  if (typeof value === 'object' && value !== null) {
    return Object.fromEntries(
      Object.entries(value as Record<string, unknown>)
        .sort(([first], [second]) => first.localeCompare(second))
        .map(([key, entry]) => [key, stableValue(entry)]),
    );
  }
  return value;
}

function semanticallyEqual(first: unknown, second: unknown): boolean {
  return JSON.stringify(stableValue(first)) === JSON.stringify(stableValue(second));
}

export function replayPlannerEpisode(events: EpisodeEvent[]): EpisodeEvent[] {
  const first = events[0];
  if (!first || first.type !== 'episode-started') {
    throw new InvalidPlannerInputError('Episode trace must begin with episode-started');
  }
  validateSeed(first.seed);
  if (typeof first.scenarioId !== 'string') {
    throw new InvalidPlannerInputError('Episode start must include a scenario ID');
  }
  const regenerated = generatePlannerEpisode(first.seed, first.scenarioId as ScenarioId);
  if (!semanticallyEqual(events, regenerated)) {
    throw new InvalidPlannerInputError('Episode trace does not match its deterministic replay');
  }
  return regenerated;
}
