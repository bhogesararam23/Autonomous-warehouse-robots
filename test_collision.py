from warehouse_astar_sim import astar, simulate_path, MOVING_OBSTACLES, build_obstacles
start = (1, 1)
goal = (20, 9)
obstacles = build_obstacles()
try:
    path, nodes = astar(start, goal, obstacles, MOVING_OBSTACLES)
    cols, fails = simulate_path(path, MOVING_OBSTACLES)
    print("Path length:", len(path))
    print("Collisions:", cols)
except Exception as e:
    print("Error:", e)
