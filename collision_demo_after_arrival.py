from warehouse_astar_sim import astar, simulate_path, MOVING_OBSTACLES, build_obstacles
start = (1, 1)
goal = (20, 9)
obstacles = build_obstacles()
try:
    path, nodes = astar(start, goal, obstacles, MOVING_OBSTACLES)
    # extend path staying at goal
    last_pos, last_t = path[-1]
    extended_path = list(path)
    for i in range(1, 20):
        extended_path.append((last_pos, last_t + i))
    cols, fails = simulate_path(extended_path, MOVING_OBSTACLES)
    print("Path length:", len(path))
    print("Collisions after arrival:", cols)
except Exception as e:
    print("Error:", e)
