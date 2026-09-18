# warehouse a* simulation

simple 2d warehouse sim in python to test dynamic path planning. basically sets up a grid with fixed shelves and moving obstacles like forklifts and finds a collision free path for robot using space time a*. it prints the run stats in terminal and also saves an svg visualization of the path

also added a small react ui in `ui/` if you want to see the animation in browser instead of just terminal output

## how to run

### terminal simulation

make sure python 3 is installed:

```bash
python warehouse_astar_sim.py
```

this will run the sim, print the map in terminal and save `warehouse_path.svg` in the folder

### web ui

if you want to see the frontend:

```bash
cd ui
npm install
npm run dev
```

then just open `http://localhost:5173/` in browser

## how it works

### grid & coords
the warehouse is a 24x14 grid
* x is column (0 to 23 left to right)
* y is row (0 to 13 top to bottom)
* robot starts at `(1, 1)` and tries to reach `(22, 12)`

### obstacles
* shelves: defined like `("Name", x, y, width, height)` and gets converted into blocked cells so robot cant pass through them
* moving obstacles: they just loop through a list of coordinates every tick like a forklift patrolling an aisle

### space-time a*
normal a* only checks `(x, y)` which breaks when obstacles are moving around. cell `(8, 9)` might be blocked at t=3 by a forklift but totally free at t=2 or t=4

so the search state tracks time too:
```text
((x, y), time_step)
```

robot can move up, down, left, right or just wait in place if it needs to let a moving obstacle pass first. no diagonal moves

### collision checks
planner checks two things so it doesnt crash:
1. same cell collision - robot and obstacle landing on the same coordinate at the same time step
2. edge swap collision - robot moving from cell A to B while obstacle moves from B to A at the exact same time (basically walking through each other)

### path finding
* uses manhattan distance for heuristic since diagonals arent allowed: `abs(x1 - x2) + abs(y1 - y2)`
* uses pythons `heapq` module as priority queue so it expands lowest cost nodes first
* once it hits the goal it backtracks using `came_from` and reverses it to get the final path

## terminal map symbols

```text
S = start
G = goal
# = shelf / obstacle
M = moving obstacle at t=0
* = planned path
. = empty floor
```