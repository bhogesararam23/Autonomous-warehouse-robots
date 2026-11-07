from warehouse_astar_sim import (
    MOVING_OBSTACLES,
    START,
    astar,
    build_obstacles,
    simulate_path,
)


def main() -> int:
    # Plan safely to a staging cell, then enter a goal that a moving robot will
    # revisit. The initial arrival is safe; remaining there is not.
    staging_point = (20, 8)
    contested_goal = (20, 9)
    path, _ = astar(START, staging_point, build_obstacles(), MOVING_OBSTACLES)

    arrival_time = path[-1][1] + 1
    while any(
        obstacle.position_at(arrival_time) == contested_goal
        for obstacle in MOVING_OBSTACLES
    ):
        path.append((staging_point, arrival_time))
        arrival_time += 1

    path.append((contested_goal, arrival_time))
    collision_time = arrival_time + 1
    while not any(
        obstacle.position_at(collision_time) == contested_goal
        for obstacle in MOVING_OBSTACLES
    ):
        path.append((contested_goal, collision_time))
        collision_time += 1
    path.append((contested_goal, collision_time))

    collisions, failures = simulate_path(path, MOVING_OBSTACLES)
    print("Unsafe post-arrival test path ends at:", path[-1])
    print("Collisions after arrival:", collisions)
    print("Execution failures:", failures)
    return 0 if collisions == 1 and failures == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
