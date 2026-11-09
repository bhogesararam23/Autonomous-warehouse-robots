import { readFileSync } from 'node:fs';
import { planWarehouse, type MovingObstacle, type Point } from './src/simulation';

interface PlannerFixture {
  id: string;
  gridWidth: number;
  gridHeight: number;
  maxPlanningTime: number;
  start: Point;
  goal: Point;
  staticObstacles: Point[];
  movingObstacles: MovingObstacle[];
  expected: { status: string; code?: string; pathLength?: number };
}

interface FixtureFile {
  cases: PlannerFixture[];
}

const fixturesPath = process.argv[2];
if (!fixturesPath) throw new Error('Expected the planner fixture JSON path');
const fixtureFile = JSON.parse(readFileSync(fixturesPath, 'utf-8')) as FixtureFile;
const output = fixtureFile.cases.map((fixture) => {
  const staticObstacles = new Set(
    fixture.staticObstacles.map((point) => `${point.x},${point.y}`),
  );
  const result = planWarehouse(
    fixture.start,
    fixture.goal,
    staticObstacles,
    fixture.movingObstacles,
    {
      gridWidth: fixture.gridWidth,
      gridHeight: fixture.gridHeight,
      maxPlanningTime: fixture.maxPlanningTime,
    },
  );
  if (result.status === 'failure') {
    return { id: fixture.id, status: result.status, code: result.code };
  }
  return {
    id: fixture.id,
    status: result.status,
    pathLength: result.path.length - 1,
    expandedNodes: result.expandedNodes,
    path: result.path.map((state) => ({
      x: state.point.x,
      y: state.point.y,
      timeStep: state.timeStep,
    })),
  };
});

process.stdout.write(`${JSON.stringify(output)}\n`);
