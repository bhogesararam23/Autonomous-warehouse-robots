import { useEffect, useState, useMemo } from 'react';
import {
  GRID_WIDTH,
  GRID_HEIGHT,
  START,
  GOAL,
  SHELVES,
  MOVING_OBSTACLES,
  buildObstacles,
  astar,
  positionAt,
} from './simulation';
import type { State } from './simulation';
import './index.css';

export default function App() {
  const [path, setPath] = useState<State[]>([]);
  const [expandedNodes, setExpandedNodes] = useState(0);
  const [error, setError] = useState<string | null>(null);

  const [currentStep, setCurrentStep] = useState(0);
  const [isPlaying, setIsPlaying] = useState(false);

  const obstacles = useMemo(() => buildObstacles(), []);

  // Run A* on mount
  useEffect(() => {
    try {
      const startMs = performance.now();
      const res = astar(START, GOAL, obstacles, MOVING_OBSTACLES);
      const timeMs = performance.now() - startMs;
      console.log(`Planned in ${timeMs.toFixed(2)}ms`);
      setPath(res.path);
      setExpandedNodes(res.expandedNodes);
    } catch (err: any) {
      setError(err.message);
    }
  }, [obstacles]);

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
      }, 300) as unknown as number; // 300ms per step
    }
    return () => clearInterval(interval);
  }, [isPlaying, path.length]);

  const togglePlay = () => {
    if (currentStep >= path.length - 1) {
      setCurrentStep(0);
    }
    setIsPlaying(!isPlaying);
  };

  const robotPos = path[currentStep]?.point ?? START;
  const timeStep = path[currentStep]?.timeStep ?? 0;

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

      // Check moving obstacles
      const activeObstacles = MOVING_OBSTACLES.map((mo) => positionAt(mo.route, timeStep));
      const hasObstacle = activeObstacles.some((obs) => obs.x === x && obs.y === y);

      if (hasObstacle) cellType = 'moving-obstacle';
      if (robotPos.x === x && robotPos.y === y) cellType = 'robot';

      if (cellType === 'start') content = 'S';
      if (cellType === 'goal') content = 'G';
      if (cellType === 'robot') content = '🤖';
      if (cellType === 'moving-obstacle') content = '⚠️';

      // Mark planned path cells
      const isPath = path.some((p) => p.point.x === x && p.point.y === y);
      const isPastPath = path.slice(0, currentStep).some((p) => p.point.x === x && p.point.y === y);

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
          <div className="controls">
            <button className="play-btn" onClick={togglePlay} disabled={path.length === 0}>
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

          <div className="stats">
            <h2>Simulation Stats</h2>
            {error ? (
              <div className="error">{error}</div>
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
            <div className="legend-item"><div className="cell moving-obstacle">⚠️</div> Moving Obstacle</div>
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
