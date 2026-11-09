import { readFileSync } from 'node:fs';
import { astar, type MovingObstacle, type Point } from './src/simulation';

interface PlannerFixture {
  id: string;
  gridWidth: number;
  gridHeight: number;
  maxPlanningTime: number;
  start: Point;
  goal: Point;
  staticObstacles: Point[];
  movingObstacles: MovingObstacle[];
  expected: { status: string; pathLength?: number };
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
  try {
    const result = astar(
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
    return {
      id: fixture.id,
      status: 'success',
      pathLength: result.path.length - 1,
      expandedNodes: result.expandedNodes,
      path: result.path.map((state) => ({
        x: state.point.x,
        y: state.point.y,
        timeStep: state.timeStep,
      })),
    };
  } catch (error) {
    if (error instanceof Error && error.message.includes('No safe path')) {
      return { id: fixture.id, status: 'no-safe-path' };
    }
    throw error;
  }
});

process.stdout.write(`${JSON.stringify(output)}\n`);
