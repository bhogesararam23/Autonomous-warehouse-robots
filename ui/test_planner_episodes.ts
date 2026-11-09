import assert from 'node:assert/strict';
import {
  generatePlannerEpisode,
  replayPlannerEpisode,
  type EpisodeEvent,
} from './src/plannerEpisodes';
import { InvalidPlannerInputError } from './src/simulation';

const first = generatePlannerEpisode(123456789);
assert.deepEqual(first, generatePlannerEpisode(123456789), 'fixed seeds should repeat exactly');
assert.equal(first[0].type, 'episode-started');
if (first[0].type !== 'episode-started') throw new Error('missing episode-started event');
assert.equal(first[0].seed, 123456789);

const replayed = replayPlannerEpisode(first);
assert.deepEqual(replayed, first, 'replay should reproduce the canonical event list');

const tampered = structuredClone(first) as EpisodeEvent[];
if (tampered[0].type !== 'episode-started') throw new Error('missing episode-started event');
tampered[0].start = { ...tampered[0].start, x: tampered[0].start.x + 1 };
assert.throws(
  () => replayPlannerEpisode(tampered),
  InvalidPlannerInputError,
  'tampered traces must be rejected',
);

for (const invalidSeed of [-1, 1.5, Number.NaN, Number.POSITIVE_INFINITY, 0x1_0000_0000]) {
  assert.throws(
    () => generatePlannerEpisode(invalidSeed),
    InvalidPlannerInputError,
    `seed ${invalidSeed} should be rejected`,
  );
}

let successful: EpisodeEvent[] | undefined;
for (let seed = 0; seed < 64 && !successful; seed++) {
  const episode = generatePlannerEpisode(seed);
  if (episode.some((event) => event.type === 'episode-finished' && event.status === 'success')) {
    successful = episode;
  }
}
assert.ok(successful, 'at least one deterministic seed should produce a safe route');
const positions = successful.filter(
  (event): event is Extract<EpisodeEvent, { type: 'robot-position' }> =>
    event.type === 'robot-position',
);
assert.ok(positions.length > 0, 'successful episodes should contain robot positions');
assert.equal(positions[0].timeStep, 0);
for (let index = 1; index < positions.length; index++) {
  const previous = positions[index - 1];
  const current = positions[index];
  assert.equal(current.timeStep, previous.timeStep + 1, 'route time must be continuous');
  assert.ok(
    Math.abs(current.x - previous.x) + Math.abs(current.y - previous.y) <= 1,
    'route steps must move at most one cell',
  );
}

console.log('Planner episode generation and replay tests passed.');
