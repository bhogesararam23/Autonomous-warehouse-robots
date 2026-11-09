import { readFileSync } from 'node:fs';
import {
  generatePlannerEpisode,
  replayPlannerEpisode,
  type EpisodeEvent,
} from './src/plannerEpisodes';
import type { ScenarioId } from './src/simulation';

interface EpisodeFixture {
  id: string;
  seed: number;
  scenarioId: ScenarioId;
}

interface FixtureFile {
  cases: EpisodeFixture[];
}

const fixturesPath = process.argv[2];
if (!fixturesPath) throw new Error('Expected the seeded episode fixture JSON path');
const fixtureFile = JSON.parse(readFileSync(fixturesPath, 'utf-8')) as FixtureFile;
const output = fixtureFile.cases.map((fixture) => {
  const events: EpisodeEvent[] = generatePlannerEpisode(
    fixture.seed,
    fixture.scenarioId,
  );
  const replayed = replayPlannerEpisode(events);
  if (JSON.stringify(replayed) !== JSON.stringify(events)) {
    throw new Error(`Replay changed event sequence for ${fixture.id}`);
  }
  return { id: fixture.id, events };
});

process.stdout.write(`${JSON.stringify(output)}\n`);
