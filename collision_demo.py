from warehouse_astar_sim import MOVING_OBSTACLES, simulate_path


def main() -> int:
    # Demonstrate a one-step edge swap with the forklift moving from (9, 9)
    # to (8, 9) while the robot attempts the opposite move.
    path = [((8, 9), 2), ((9, 9), 3)]
    collisions, failures = simulate_path(path, MOVING_OBSTACLES)

    print("Unsafe test path:", path)
    print("Collisions detected:", collisions)
    print("Execution failures:", failures)
    return 0 if collisions == 1 and failures == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
