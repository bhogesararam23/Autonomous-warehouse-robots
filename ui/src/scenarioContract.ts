export const SCENARIO_SCHEMA_VERSION = 1;
const MAX_GRID_DIMENSION = 512;
const MAX_GRID_CELLS = 65_536;
const MAX_PLANNING_TIME = 10_000;
const MAX_TIME_EXPANDED_STATES = 1_000_000;
const MAX_SCENARIOS = 64;
const MAX_SHELVES = 1_024;
const MAX_MOVERS_PER_SCENARIO = 16;
const MAX_ROUTE_LENGTH = 512;
const MAX_JOINT_ROUTE_PERIOD = 100_000;
const MAX_SCENARIO_ID_LENGTH = 64;
const MAX_NAME_LENGTH = 256;
const MAX_DESCRIPTION_LENGTH = 1_024;

export interface ScenarioPoint {
  x: number;
  y: number;
}

export interface ScenarioShelf {
  name: string;
  x: number;
  y: number;
  width: number;
  height: number;
}

export interface MovingObstacleDefinition {
  name: string;
  route: ScenarioPoint[];
}

export interface WarehouseScenarioDefinition {
  id: string;
  name: string;
  description: string;
  movingObstacles: MovingObstacleDefinition[];
}

export interface ScenarioCatalog {
  schemaVersion: number;
  coordinateSystem: {
    unit: 'cell';
    origin: 'top-left';
    xAxisDirection: 'right';
    yAxisDirection: 'down';
    timeStepSemantics: 'one-move-or-wait';
  };
  warehouse: {
    width: number;
    height: number;
    start: ScenarioPoint;
    goal: ScenarioPoint;
    maxPlanningTime: number;
    shelves: ScenarioShelf[];
  };
  defaultScenarioId: string;
  scenarios: WarehouseScenarioDefinition[];
}

export class ScenarioContractError extends Error {
  constructor(message: string) {
    super(message);
    this.name = 'ScenarioContractError';
  }
}

const SCENARIO_ID = /^[a-z0-9][a-z0-9-]*$/;
const COORDINATE_SYSTEM = {
  unit: 'cell',
  origin: 'top-left',
  xAxisDirection: 'right',
  yAxisDirection: 'down',
  timeStepSemantics: 'one-move-or-wait',
} as const;

type RecordValue = Record<string, unknown>;

type MovingRoute = { name: string; route: ScenarioPoint[] };

function record(value: unknown, path: string, keys: string[]): RecordValue {
  if (typeof value !== 'object' || value === null || Array.isArray(value)) {
    throw new ScenarioContractError(`${path} must be an object`);
  }
  const result = value as RecordValue;
  const actual = Object.keys(result);
  const missing = keys.filter((key) => !actual.includes(key)).sort();
  const extra = actual.filter((key) => !keys.includes(key)).sort();
  if (missing.length > 0 || extra.length > 0) {
    const details = [
      ...(missing.length > 0 ? [`missing ${missing.join(', ')}`] : []),
      ...(extra.length > 0 ? [`unexpected ${extra.join(', ')}`] : []),
    ];
    throw new ScenarioContractError(`${path} has invalid fields: ${details.join('; ')}`);
  }
  return result;
}

function integer(
  value: unknown,
  path: string,
  minimum?: number,
  maximum?: number,
): number {
  if (typeof value !== 'number' || !Number.isInteger(value)) {
    throw new ScenarioContractError(`${path} must be an integer`);
  }
  if (!Number.isSafeInteger(value)) {
    throw new ScenarioContractError(`${path} must be a safe integer`);
  }
  const result = value;
  if (minimum !== undefined && result < minimum) {
    throw new ScenarioContractError(`${path} must be at least ${minimum}`);
  }
  if (maximum !== undefined && result > maximum) {
    throw new ScenarioContractError(`${path} must be at most ${maximum}`);
  }
  return result;
}

function nonemptyString(value: unknown, path: string, maximumLength: number): string {
  if (typeof value !== 'string' || value.trim().length === 0) {
    throw new ScenarioContractError(`${path} must be a non-empty string`);
  }
  if (Array.from(value).length > maximumLength) {
    throw new ScenarioContractError(`${path} must be at most ${maximumLength} characters`);
  }
  return value;
}

function point(value: unknown, path: string): ScenarioPoint {
  const result = record(value, path, ['x', 'y']);
  return {
    x: integer(result.x, `${path}.x`),
    y: integer(result.y, `${path}.y`),
  };
}

function inside(pointValue: ScenarioPoint, width: number, height: number): boolean {
  return (
    pointValue.x >= 0 &&
    pointValue.x < width &&
    pointValue.y >= 0 &&
    pointValue.y < height
  );
}

function gcd(a: number, b: number): number {
  return b === 0 ? a : gcd(b, a % b);
}

function rejectPeriodicMoverConflicts(
  movers: MovingRoute[],
  scenarioId: string,
): void {
  for (let firstIndex = 0; firstIndex < movers.length; firstIndex++) {
    const first = movers[firstIndex];
    for (let secondIndex = firstIndex + 1; secondIndex < movers.length; secondIndex++) {
      const second = movers[secondIndex];
      const sharedPeriod = gcd(first.route.length, second.route.length);
      const firstStates = new Set(
        first.route.map((position, time) => `${position.x},${position.y}:${time % sharedPeriod}`),
      );
      if (
        second.route.some((position, time) =>
          firstStates.has(`${position.x},${position.y}:${time % sharedPeriod}`),
        )
      ) {
        throw new ScenarioContractError(
          `scenario '${scenarioId}' has moving obstacles '${first.name}' and '${second.name}' occupying the same cell`,
        );
      }
      const firstEdges = new Set(
        first.route.map((position, time) => {
          const next = first.route[(time + 1) % first.route.length];
          return `${position.x},${position.y}>${next.x},${next.y}:${time % sharedPeriod}`;
        }),
      );
      const secondReverseEdges = second.route.map((position, time) => {
        const next = second.route[(time + 1) % second.route.length];
        return `${next.x},${next.y}>${position.x},${position.y}:${time % sharedPeriod}`;
      });
      if (secondReverseEdges.some((edge) => firstEdges.has(edge))) {
        throw new ScenarioContractError(
          `scenario '${scenarioId}' has moving obstacles '${first.name}' and '${second.name}' colliding by head-on edge swap`,
        );
      }
    }
  }
}

export function validateScenarioCatalog(value: unknown): asserts value is ScenarioCatalog {
  const root = record(value, 'catalog', [
    'schemaVersion',
    'coordinateSystem',
    'warehouse',
    'defaultScenarioId',
    'scenarios',
  ]);
  const version = integer(root.schemaVersion, 'schemaVersion');
  if (version !== SCENARIO_SCHEMA_VERSION) {
    throw new ScenarioContractError(
      `unsupported scenario schemaVersion ${version}; this planner supports ${SCENARIO_SCHEMA_VERSION}`,
    );
  }

  const coordinateSystem = record(
    root.coordinateSystem,
    'coordinateSystem',
    Object.keys(COORDINATE_SYSTEM),
  );
  if (
    coordinateSystem.unit !== COORDINATE_SYSTEM.unit ||
    coordinateSystem.origin !== COORDINATE_SYSTEM.origin ||
    coordinateSystem.xAxisDirection !== COORDINATE_SYSTEM.xAxisDirection ||
    coordinateSystem.yAxisDirection !== COORDINATE_SYSTEM.yAxisDirection ||
    coordinateSystem.timeStepSemantics !== COORDINATE_SYSTEM.timeStepSemantics
  ) {
    throw new ScenarioContractError(
      'coordinateSystem must use top-left-origin grid cells and one-move-or-wait steps',
    );
  }

  const warehouse = record(root.warehouse, 'warehouse', [
    'width',
    'height',
    'start',
    'goal',
    'maxPlanningTime',
    'shelves',
  ]);
  const width = integer(warehouse.width, 'warehouse.width', 1, MAX_GRID_DIMENSION);
  const height = integer(warehouse.height, 'warehouse.height', 1, MAX_GRID_DIMENSION);
  const gridCells = width * height;
  if (gridCells > MAX_GRID_CELLS) {
    throw new ScenarioContractError(
      `warehouse grid must contain at most ${MAX_GRID_CELLS} cells`,
    );
  }
  const maxPlanningTime = integer(
    warehouse.maxPlanningTime,
    'warehouse.maxPlanningTime',
    0,
    MAX_PLANNING_TIME,
  );
  if (gridCells * (maxPlanningTime + 1) > MAX_TIME_EXPANDED_STATES) {
    throw new ScenarioContractError(
      `warehouse grid and planning horizon exceed ${MAX_TIME_EXPANDED_STATES} time-expanded states`,
    );
  }
  const start = point(warehouse.start, 'warehouse.start');
  const goal = point(warehouse.goal, 'warehouse.goal');
  if (!inside(start, width, height)) {
    throw new ScenarioContractError('warehouse.start must be inside the grid');
  }
  if (!inside(goal, width, height)) {
    throw new ScenarioContractError('warehouse.goal must be inside the grid');
  }

  if (!Array.isArray(warehouse.shelves)) {
    throw new ScenarioContractError('warehouse.shelves must be an array');
  }
  if (warehouse.shelves.length > MAX_SHELVES) {
    throw new ScenarioContractError(
      `warehouse.shelves must contain at most ${MAX_SHELVES} entries`,
    );
  }
  const shelfNames = new Set<string>();
  const staticCells = new Set<string>();
  warehouse.shelves.forEach((rawShelf, index) => {
    const path = `warehouse.shelves[${index}]`;
    const shelf = record(rawShelf, path, ['name', 'x', 'y', 'width', 'height']);
    const name = nonemptyString(shelf.name, `${path}.name`, MAX_NAME_LENGTH);
    if (shelfNames.has(name)) {
      throw new ScenarioContractError(`duplicate shelf name '${name}'`);
    }
    shelfNames.add(name);
    const x = integer(shelf.x, `${path}.x`, 0, MAX_GRID_DIMENSION - 1);
    const y = integer(shelf.y, `${path}.y`, 0, MAX_GRID_DIMENSION - 1);
    const shelfWidth = integer(shelf.width, `${path}.width`, 1, MAX_GRID_DIMENSION);
    const shelfHeight = integer(shelf.height, `${path}.height`, 1, MAX_GRID_DIMENSION);
    if (x + shelfWidth > width || y + shelfHeight > height) {
      throw new ScenarioContractError(`${path} must fit entirely inside the grid`);
    }
    for (let cellX = x; cellX < x + shelfWidth; cellX++) {
      for (let cellY = y; cellY < y + shelfHeight; cellY++) {
        const key = `${cellX},${cellY}`;
        if (staticCells.has(key)) {
          throw new ScenarioContractError(`${path} overlaps another shelf`);
        }
        staticCells.add(key);
      }
    }
  });
  if (staticCells.has(`${start.x},${start.y}`)) {
    throw new ScenarioContractError('warehouse.start is blocked by a shelf');
  }
  if (staticCells.has(`${goal.x},${goal.y}`)) {
    throw new ScenarioContractError('warehouse.goal is blocked by a shelf');
  }

  const defaultId = nonemptyString(
    root.defaultScenarioId,
    'defaultScenarioId',
    MAX_SCENARIO_ID_LENGTH,
  );
  if (!Array.isArray(root.scenarios) || root.scenarios.length === 0) {
    throw new ScenarioContractError('scenarios must be a non-empty array');
  }
  if (root.scenarios.length > MAX_SCENARIOS) {
    throw new ScenarioContractError(`scenarios must contain at most ${MAX_SCENARIOS} entries`);
  }
  const scenarioIds = new Set<string>();
  root.scenarios.forEach((rawScenario, scenarioIndex) => {
    const scenarioPath = `scenarios[${scenarioIndex}]`;
    const scenario = record(rawScenario, scenarioPath, [
      'id',
      'name',
      'description',
      'movingObstacles',
    ]);
    const scenarioId = nonemptyString(
      scenario.id,
      `${scenarioPath}.id`,
      MAX_SCENARIO_ID_LENGTH,
    );
    if (!SCENARIO_ID.test(scenarioId)) {
      throw new ScenarioContractError(
        `${scenarioPath}.id must contain lowercase letters, digits, and hyphens`,
      );
    }
    if (scenarioIds.has(scenarioId)) {
      throw new ScenarioContractError(`duplicate scenario id '${scenarioId}'`);
    }
    scenarioIds.add(scenarioId);
    nonemptyString(scenario.name, `${scenarioPath}.name`, MAX_NAME_LENGTH);
    nonemptyString(scenario.description, `${scenarioPath}.description`, MAX_DESCRIPTION_LENGTH);
    if (!Array.isArray(scenario.movingObstacles)) {
      throw new ScenarioContractError(`${scenarioPath}.movingObstacles must be an array`);
    }
    if (scenario.movingObstacles.length > MAX_MOVERS_PER_SCENARIO) {
      throw new ScenarioContractError(
        `${scenarioPath}.movingObstacles must contain at most ${MAX_MOVERS_PER_SCENARIO} entries`,
      );
    }

    const moverNames = new Set<string>();
    const movers: MovingRoute[] = [];
    let jointPeriod = 1;
    scenario.movingObstacles.forEach((rawMover, moverIndex) => {
      const moverPath = `${scenarioPath}.movingObstacles[${moverIndex}]`;
      const mover = record(rawMover, moverPath, ['name', 'route']);
      const moverName = nonemptyString(mover.name, `${moverPath}.name`, MAX_NAME_LENGTH);
      if (moverNames.has(moverName)) {
        throw new ScenarioContractError(
          `scenario '${scenarioId}' has duplicate mover name '${moverName}'`,
        );
      }
      moverNames.add(moverName);
      if (!Array.isArray(mover.route) || mover.route.length === 0) {
        throw new ScenarioContractError(`${moverPath}.route must be a non-empty array`);
      }
      if (mover.route.length > MAX_ROUTE_LENGTH) {
        throw new ScenarioContractError(
          `${moverPath}.route must contain at most ${MAX_ROUTE_LENGTH} points`,
        );
      }
      const route = mover.route.map((rawPoint, pointIndex) =>
        point(rawPoint, `${moverPath}.route[${pointIndex}]`),
      );
      route.forEach((routePoint, pointIndex) => {
        if (!inside(routePoint, width, height)) {
          throw new ScenarioContractError(`${moverPath}.route contains a point outside the grid`);
        }
        if (staticCells.has(`${routePoint.x},${routePoint.y}`)) {
          throw new ScenarioContractError(
            `${moverPath}.route intersects a shelf at (${routePoint.x}, ${routePoint.y})`,
          );
        }
        const following = route[(pointIndex + 1) % route.length];
        if (
          Math.abs(routePoint.x - following.x) + Math.abs(routePoint.y - following.y) > 1
        ) {
          throw new ScenarioContractError(
            `${moverPath}.route must move at most one cell per step`,
          );
        }
      });
      if (route[0].x === start.x && route[0].y === start.y) {
        throw new ScenarioContractError(
          `scenario '${scenarioId}' starts with mover '${moverName}' on the robot`,
        );
      }
      movers.push({ name: moverName, route });
      jointPeriod = (jointPeriod * route.length) / gcd(jointPeriod, route.length);
      if (jointPeriod > MAX_JOINT_ROUTE_PERIOD) {
        throw new ScenarioContractError(
          `scenario '${scenarioId}' joint mover period exceeds ${MAX_JOINT_ROUTE_PERIOD} steps`,
        );
      }
    });
    rejectPeriodicMoverConflicts(movers, scenarioId);
  });
  if (!scenarioIds.has(defaultId)) {
    throw new ScenarioContractError(`defaultScenarioId '${defaultId}' does not name a scenario`);
  }
}
