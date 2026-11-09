import assert from 'node:assert/strict';
import Ajv2020 from 'ajv/dist/2020.js';
import catalogJson from '../scenarios.json';
import schemaJson from '../scenarios.schema.json';
import {
  DEFAULT_SCENARIO_ID,
  GOAL,
  GRID_HEIGHT,
  GRID_WIDTH,
  MAX_PLANNING_TIME,
  SHELVES,
  START,
} from './src/simulation';
import {
  SCENARIO_SCHEMA_VERSION,
  ScenarioContractError,
  type ScenarioCatalog,
  validateScenarioCatalog,
} from './src/scenarioContract';

function cloneCatalog(): ScenarioCatalog {
  return JSON.parse(JSON.stringify(catalogJson)) as ScenarioCatalog;
}

function rejectMutation(
  mutate: (catalog: ScenarioCatalog) => void,
  expectedMessage: RegExp,
): void {
  const catalog = cloneCatalog();
  mutate(catalog);
  assert.throws(
    () => validateScenarioCatalog(catalog),
    (error: unknown) =>
      error instanceof ScenarioContractError && expectedMessage.test(error.message),
  );
}

const validateJsonSchema = new Ajv2020({ allErrors: true }).compile(schemaJson);
validateScenarioCatalog(catalogJson);
assert.equal(
  validateJsonSchema(catalogJson),
  true,
  JSON.stringify(validateJsonSchema.errors),
);
assert.equal(catalogJson.schemaVersion, SCENARIO_SCHEMA_VERSION);
assert.equal(schemaJson.properties.schemaVersion.const, SCENARIO_SCHEMA_VERSION);
assert.deepEqual(new Set(schemaJson.required), new Set([
  'schemaVersion',
  'coordinateSystem',
  'warehouse',
  'defaultScenarioId',
  'scenarios',
]));
assert.equal(GRID_WIDTH, catalogJson.warehouse.width);
assert.equal(GRID_HEIGHT, catalogJson.warehouse.height);
assert.deepEqual(START, catalogJson.warehouse.start);
assert.deepEqual(GOAL, catalogJson.warehouse.goal);
assert.equal(MAX_PLANNING_TIME, catalogJson.warehouse.maxPlanningTime);
assert.deepEqual(
  SHELVES,
  catalogJson.warehouse.shelves.map((shelf) => ({
    name: shelf.name,
    x: shelf.x,
    y: shelf.y,
    w: shelf.width,
    h: shelf.height,
  })),
);
assert.equal(DEFAULT_SCENARIO_ID, catalogJson.defaultScenarioId);

const structurallyInvalid = cloneCatalog();
structurallyInvalid.scenarios[0].movingObstacles[0].route = [];
assert.equal(validateJsonSchema(structurallyInvalid), false);
const whitespaceName = cloneCatalog();
whitespaceName.scenarios[0].name = '   ';
assert.equal(validateJsonSchema(whitespaceName), false);
assert.throws(() => validateScenarioCatalog(whitespaceName), ScenarioContractError);
const unsafeCoordinate = cloneCatalog();
unsafeCoordinate.warehouse.start.x = Number.MAX_SAFE_INTEGER + 1;
assert.equal(validateJsonSchema(unsafeCoordinate), false);
assert.throws(() => validateScenarioCatalog(unsafeCoordinate), /safe integer/);
const trailingNewlineId = cloneCatalog();
trailingNewlineId.scenarios[0].id = `${trailingNewlineId.scenarios[0].id}\n`;
assert.equal(validateJsonSchema(trailingNewlineId), false);
assert.throws(() => validateScenarioCatalog(trailingNewlineId), /lowercase letters/);
const excessiveGrid = cloneCatalog();
excessiveGrid.warehouse.width = 512;
excessiveGrid.warehouse.height = 512;
assert.equal(validateJsonSchema(excessiveGrid), true);
assert.throws(() => validateScenarioCatalog(excessiveGrid), /at most 65536 cells/);
const oversizedField = cloneCatalog();
oversizedField.warehouse.width = 513;
assert.equal(validateJsonSchema(oversizedField), false);
assert.throws(() => validateScenarioCatalog(oversizedField), /at most 512/);
const excessiveSearch = cloneCatalog();
excessiveSearch.warehouse.width = 64;
excessiveSearch.warehouse.height = 40;
excessiveSearch.warehouse.maxPlanningTime = 10_000;
assert.equal(validateJsonSchema(excessiveSearch), true);
assert.throws(() => validateScenarioCatalog(excessiveSearch), /exceed 1000000/);
const excessiveJointPeriod = cloneCatalog();
excessiveJointPeriod.scenarios[0].movingObstacles = [
  { name: 'stationary-a', route: Array(317).fill({ x: 2, y: 0 }) },
  { name: 'stationary-b', route: Array(331).fill({ x: 3, y: 0 }) },
];
assert.equal(validateJsonSchema(excessiveJointPeriod), true);
assert.throws(() => validateScenarioCatalog(excessiveJointPeriod), /joint mover period exceeds 100000/);

rejectMutation((catalog) => {
  catalog.schemaVersion = 2;
}, /unsupported scenario schemaVersion/);
rejectMutation((catalog) => {
  catalog.coordinateSystem.yAxisDirection = 'up' as 'down';
}, /coordinateSystem/);
rejectMutation((catalog) => {
  catalog.warehouse.width = true as unknown as number;
}, /warehouse\.width must be an integer/);
rejectMutation((catalog) => {
  catalog.warehouse.goal.x = catalog.warehouse.width;
}, /warehouse\.goal must be inside the grid/);
rejectMutation((catalog) => {
  catalog.warehouse.shelves[0].x = 23;
}, /must fit entirely inside the grid/);
rejectMutation((catalog) => {
  catalog.warehouse.shelves[1].x = 6;
  catalog.warehouse.shelves[1].y = 1;
}, /overlaps another shelf/);
rejectMutation((catalog) => {
  catalog.scenarios[1].id = 'Peak Traffic';
}, /must contain lowercase letters/);
rejectMutation((catalog) => {
  catalog.scenarios[1].id = 'standard';
}, /duplicate scenario id/);
rejectMutation((catalog) => {
  catalog.defaultScenarioId = 'missing';
}, /does not name a scenario/);
rejectMutation((catalog) => {
  catalog.scenarios[0].movingObstacles[0].route[0].x = 24;
}, /outside the grid/);
rejectMutation((catalog) => {
  catalog.scenarios[0].movingObstacles[0].route[0] = { x: 5, y: 0 };
}, /intersects a shelf/);
rejectMutation((catalog) => {
  catalog.scenarios[0].movingObstacles[0].route = [{ x: 1, y: 1 }];
}, /starts with mover .* on the robot/);
rejectMutation((catalog) => {
  catalog.scenarios[0].movingObstacles[1].route = structuredClone(
    catalog.scenarios[0].movingObstacles[0].route,
  );
}, /occupying the same cell/);
rejectMutation((catalog) => {
  const movers = catalog.scenarios[0].movingObstacles;
  movers[0].route = [{ x: 18, y: 9 }, { x: 19, y: 9 }];
  movers[1].route = [{ x: 19, y: 9 }, { x: 18, y: 9 }];
}, /head-on edge swap/);
rejectMutation((catalog) => {
  catalog.scenarios[0].movingObstacles[0].route = [
    { x: 7, y: 9 },
    { x: 9, y: 9 },
  ];
}, /move at most one cell/);
rejectMutation((catalog) => {
  (catalog as ScenarioCatalog & { unversioned?: boolean }).unversioned = true;
}, /unexpected unversioned/);

console.log('Versioned scenario contract and invalid-map tests passed.');
