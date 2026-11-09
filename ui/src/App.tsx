import { useEffect, useState, useMemo } from 'react';
import {
  GRID_WIDTH,
  GRID_HEIGHT,
  START,
  GOAL,
  DEFAULT_SCENARIO_ID,
  SCENARIOS,
  buildObstacles,
  planWarehouse,
  positionAt,
  type ScenarioId,
} from './simulation';
import { getPlaybackShortcutAction } from './keyboardShortcuts';
import './index.css';

export default function App() {
  const [currentStep, setCurrentStep] = useState(0);
  const [isPlaying, setIsPlaying] = useState(false);
  const [playbackSpeed, setPlaybackSpeed] = useState(1);
  const [selectedScenarioId, setSelectedScenarioId] = useState<ScenarioId>(DEFAULT_SCENARIO_ID);

  const obstacles = useMemo(() => buildObstacles(), []);
  const scenario = SCENARIOS.find((candidate) => candidate.id === selectedScenarioId) ?? SCENARIOS[0];

  const plan = useMemo(() => {
    return planWarehouse(START, GOAL, obstacles, scenario.movingObstacles);
  }, [obstacles, scenario]);
  const path = useMemo(() => (plan.status === 'success' ? plan.path : []), [plan]);
  const expandedNodes = plan.status === 'success' ? plan.expandedNodes : 0;
  const lastStep = Math.max(path.length - 1, 0);

  // Simulation playback
  useEffect(() => {
    let interval: number;
    if (isPlaying && path.length > 0) {
      interval = setInterval(() => {
        setCurrentStep((prev) => {
          if (prev >= path.length - 1) {
            setIsPlaying(false);
            return prev;
          }
          return prev + 1;
        });
      }, 300 / playbackSpeed) as unknown as number;
    }
    return () => clearInterval(interval);
  }, [isPlaying, path.length, playbackSpeed]);

  const togglePlay = () => {
    if (currentStep >= lastStep) {
      setCurrentStep(0);
    }
    setIsPlaying(!isPlaying);
  };

  const seekToStep = (step: number) => {
    setIsPlaying(false);
    setCurrentStep(step);
  };

  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      const action = getPlaybackShortcutAction(event);
      if (!action || path.length === 0) return;

      event.preventDefault();
      if (action === 'toggle-play') {
        if (isPlaying) {
          setIsPlaying(false);
        } else {
          if (currentStep >= lastStep) setCurrentStep(0);
          setIsPlaying(true);
        }
        return;
      }

      setIsPlaying(false);
      setCurrentStep(
        action === 'step-back'
          ? Math.max(currentStep - 1, 0)
          : Math.min(currentStep + 1, lastStep),
      );
    };

    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [currentStep, isPlaying, lastStep, path.length]);

  const robotPos = path[currentStep]?.point ?? START;
  const timeStep = path[currentStep]?.timeStep ?? 0;

  const pathCells = useMemo(() => {
    const set = new Set<string>();
    path.forEach(p => set.add(`${p.point.x},${p.point.y}`));
    return set;
  }, [path]);

  const pastPathCells = useMemo(() => {
    const set = new Set<string>();
    path.slice(0, currentStep).forEach(p => set.add(`${p.point.x},${p.point.y}`));
    return set;
  }, [path, currentStep]);

  const activeObstacleCells = useMemo(
    () =>
      new Set(
        scenario.movingObstacles.map((obstacle) => {
          const position = positionAt(obstacle.route, timeStep);
          return `${position.x},${position.y}`;
        }),
      ),
    [scenario, timeStep],
  );

  // Grid cells
  const cells = [];
  for (let y = 0; y < GRID_HEIGHT; y++) {
    for (let x = 0; x < GRID_WIDTH; x++) {
      const key = `${x},${y}`;
      let cellType = 'empty';
      let content = null;

      if (obstacles.has(key)) cellType = 'shelf';
      else if (x === START.x && y === START.y) cellType = 'start';
      else if (x === GOAL.x && y === GOAL.y) cellType = 'goal';

      if (activeObstacleCells.has(key)) cellType = 'moving-obstacle';
      if (robotPos.x === x && robotPos.y === y) cellType = 'robot';

      if (cellType === 'start') content = 'S';
      if (cellType === 'goal') content = 'G';
      if (cellType === 'robot') content = '🤖';
      if (cellType === 'moving-obstacle') content = '⚠️';

      // Mark planned path cells
      const isPath = pathCells.has(key);
      const isPastPath = pastPathCells.has(key);

      cells.push(
        <div
          key={key}
          className={`cell ${cellType} ${isPath && cellType === 'empty' ? 'path' : ''} ${
            isPastPath && cellType === 'empty' ? 'past-path' : ''
          }`}
        >
          {content}
        </div>
      );
    }
  }

  return (
    <div className="app-container">
      <header className="header">
        <h1>Warehouse A* Simulation</h1>
        <p className="subtitle">Real-time Dynamic Path Planning</p>
      </header>

      <main className="main-content">
        <div className="grid-container" style={{ '--grid-w': GRID_WIDTH, '--grid-h': GRID_HEIGHT } as React.CSSProperties}>
          {cells}
        </div>

        <aside className="panel">
          <div className="scenario-control">
            <label htmlFor="warehouse-scenario">Warehouse scenario</label>
            <select
              id="warehouse-scenario"
              className="scenario-select"
              value={scenario.id}
              aria-describedby="scenario-description"
              onChange={(event) => {
                setIsPlaying(false);
                setCurrentStep(0);
                setSelectedScenarioId(event.currentTarget.value as ScenarioId);
              }}
            >
              {SCENARIOS.map((option) => (
                <option key={option.id} value={option.id}>{option.name}</option>
              ))}
            </select>
            <p id="scenario-description" className="scenario-description">
              {scenario.description} Changing scenarios recalculates a safe plan.
            </p>
          </div>

          <div className="controls">
            <button
              className="play-btn"
              onClick={togglePlay}
              disabled={path.length === 0}
              aria-label={
                isPlaying
                  ? 'Pause simulation'
                  : currentStep >= lastStep
                    ? 'Restart simulation'
                    : 'Play simulation'
              }
              aria-keyshortcuts="Space"
            >
              {isPlaying ? '⏸ Pause' : currentStep >= path.length - 1 ? '🔄 Restart' : '▶ Play'}
            </button>
            <button
              className="reset-btn"
              onClick={() => {
                setIsPlaying(false);
                setCurrentStep(0);
              }}
            >
              ⏪ Reset
            </button>
          </div>

          <section
            className="timeline"
            aria-label="Simulation playback position"
            aria-describedby="keyboard-shortcuts"
          >
            <div className="timeline-heading">
              <label htmlFor="simulation-timeline">Time step</label>
              <output htmlFor="simulation-timeline">
                {timeStep} / {lastStep}
              </output>
            </div>
            <input
              id="simulation-timeline"
              className="timeline-slider"
              type="range"
              min={0}
              max={lastStep}
              value={currentStep}
              disabled={path.length === 0}
              aria-valuetext={`Step ${timeStep} of ${lastStep}`}
              onChange={(event) => seekToStep(Number(event.currentTarget.value))}
            />
            <div className="speed-control">
              <label htmlFor="playback-speed">Playback speed</label>
              <select
                id="playback-speed"
                className="playback-speed"
                value={playbackSpeed}
                onChange={(event) => setPlaybackSpeed(Number(event.currentTarget.value))}
              >
                <option value={0.5}>0.5×</option>
                <option value={1}>1×</option>
                <option value={2}>2×</option>
              </select>
            </div>
            <div className="step-controls">
              <button
                className="step-btn"
                onClick={() => seekToStep(Math.max(currentStep - 1, 0))}
                disabled={currentStep === 0 || path.length === 0}
                aria-label="Step backward"
                aria-keyshortcuts="ArrowLeft"
              >
                ← Step back
              </button>
              <button
                className="step-btn"
                onClick={() => seekToStep(Math.min(currentStep + 1, lastStep))}
                disabled={currentStep >= lastStep || path.length === 0}
                aria-label="Step forward"
                aria-keyshortcuts="ArrowRight"
              >
                Step forward →
              </button>
            </div>
            <p id="keyboard-shortcuts" className="shortcut-hint">
              Keyboard shortcuts (when focus is outside a control): <kbd>Space</kbd> play or pause;
              <kbd>←</kbd> / <kbd>→</kbd> step.
            </p>
          </section>

          <div className="stats">
            <h2>Simulation Stats</h2>
            {plan.status === 'failure' ? (
              <div className="error">[{plan.code}] {plan.message}</div>
            ) : (
              <>
                <div className="stat-row">
                  <span>Time Step:</span>
                  <strong>{timeStep}</strong>
                </div>
                <div className="stat-row">
                  <span>Path Length:</span>
                  <strong>{path.length ? path.length - 1 : 0} steps</strong>
                </div>
                <div className="stat-row">
                  <span>Expanded Nodes:</span>
                  <strong>{expandedNodes}</strong>
                </div>
                <div className="stat-row">
                  <span>Status:</span>
                  <strong>
                    {currentStep === 0
                      ? 'Ready'
                      : currentStep >= path.length - 1
                      ? 'Goal Reached 🎉'
                      : 'Navigating...'}
                  </strong>
                </div>
              </>
            )}
          </div>

          <div className="legend">
            <h2>Legend</h2>
            <div className="legend-item"><div className="cell shelf"></div> Fixed Shelf</div>
            <div className="legend-item"><div className="cell moving-obstacle">⚠️</div> Moving Obstacles</div>
            <div className="legend-item"><div className="cell start">S</div> Start Location</div>
            <div className="legend-item"><div className="cell goal">G</div> Goal Location</div>
            <div className="legend-item"><div className="cell robot">🤖</div> Robot</div>
            <div className="legend-item"><div className="cell path"></div> Planned Path</div>
          </div>
        </aside>
      </main>
    </div>
  );
}
