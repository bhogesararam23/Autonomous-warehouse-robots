from __future__ import annotations

import heapq
import time
from dataclasses import dataclass
from pathlib import Path


# A grid point is written as (x, y).
# x moves left/right across columns.
# y moves up/down across rows.
Point = tuple[int, int]
State = tuple[Point, int]


GRID_WIDTH = 24
GRID_HEIGHT = 14

START: Point = (1, 1)
GOAL: Point = (22, 12)

# Each shelf is a rectangle: (name, x, y, width, height).
# The coordinate (x, y) is the shelf's top-left grid cell.
SHELVES = [
    ("A", 5, 0, 2, 9),
    ("B", 10, 5, 2, 9),
    ("C", 15, 0, 2, 9),
]

MAX_PLANNING_TIME = 120


@dataclass(frozen=True)
class MovingObstacle:
    """A predictable moving obstacle, such as another robot or forklift."""

    name: str
    route: tuple[Point, ...]

    def position_at(self, time_step: int) -> Point:
        return self.route[time_step % len(self.route)]


MOVING_OBSTACLES = [
    MovingObstacle(
        "Forklift-1",
        (
            (7, 9),
            (8, 9),
            (9, 9),
            (8, 9),
        ),
    ),
    MovingObstacle(
        "Robot-2",
        (
            (18, 9),
            (19, 9),
            (20, 9),
            (21, 9),
            (20, 9),
            (19, 9),
        ),
    ),
]


def build_obstacles() -> set[Point]:
    """Convert shelf rectangles into blocked grid cells."""
    obstacles: set[Point] = set()

    for _, shelf_x, shelf_y, shelf_width, shelf_height in SHELVES:
        for x in range(shelf_x, shelf_x + shelf_width):
            for y in range(shelf_y, shelf_y + shelf_height):
                obstacles.add((x, y))

    return obstacles


def manhattan_distance(a: Point, b: Point) -> int:
    """Estimate distance on a 4-direction grid."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def get_neighbors(point: Point, obstacles: set[Point]) -> list[Point]:
    """Return valid cells the robot can move to from the current cell."""
    x, y = point
    possible_moves = [
        (x + 1, y),  # right
        (x - 1, y),  # left
        (x, y + 1),  # down
        (x, y - 1),  # up
        (x, y),  # wait in place
    ]

    valid_neighbors: list[Point] = []
    for neighbor in possible_moves:
        nx, ny = neighbor
        inside_grid = 0 <= nx < GRID_WIDTH and 0 <= ny < GRID_HEIGHT
        blocked = neighbor in obstacles

        if inside_grid and not blocked:
            valid_neighbors.append(neighbor)

    return valid_neighbors


def moving_obstacle_positions(
    moving_obstacles: list[MovingObstacle], time_step: int
) -> dict[Point, str]:
    """Return the occupied cells for all moving obstacles at one time step."""
    positions: dict[Point, str] = {}

    for obstacle in moving_obstacles:
        positions[obstacle.position_at(time_step)] = obstacle.name

    return positions


def has_dynamic_collision(
    current: Point,
    next_point: Point,
    next_time: int,
    moving_obstacles: list[MovingObstacle],
) -> bool:
    """Check if a move would hit, or swap places with, a moving obstacle."""
    current_time = next_time - 1

    for obstacle in moving_obstacles:
        obstacle_now = obstacle.position_at(current_time)
        obstacle_next = obstacle.position_at(next_time)

        same_cell_collision = next_point == obstacle_next
        edge_swap_collision = current == obstacle_next and next_point == obstacle_now

        if same_cell_collision or edge_swap_collision:
            return True

    return False


def rebuild_path(came_from: dict[State, State], current: State) -> list[State]:
    """Walk backward from the goal to the start, then reverse the result."""
    path = [current]

    while current in came_from:
        current = came_from[current]
        path.append(current)

    path.reverse()
    return path


def astar(
    start: Point,
    goal: Point,
    obstacles: set[Point],
    moving_obstacles: list[MovingObstacle],
) -> tuple[list[State], int]:
    """Find the shortest safe path from start to goal with time-aware A*."""
    start_state = (start, 0)
    open_set: list[tuple[int, int, int, State]] = []
    heapq.heappush(open_set, (0, 0, 0, start_state))

    came_from: dict[State, State] = {}
    cost_from_start: dict[State, int] = {start_state: 0}
    visited: set[State] = set()

    tie_breaker = 0
    expanded_nodes = 0

    while open_set:
        _, _, _, current_state = heapq.heappop(open_set)

        if current_state in visited:
            continue

        visited.add(current_state)
        expanded_nodes += 1

        current, current_time = current_state
        if current == goal:
            return rebuild_path(came_from, current_state), expanded_nodes

        if current_time >= MAX_PLANNING_TIME:
            continue

        for neighbor in get_neighbors(current, obstacles):
            next_time = current_time + 1

            if has_dynamic_collision(current, neighbor, next_time, moving_obstacles):
                continue

            neighbor_state = (neighbor, next_time)
            new_cost = cost_from_start[current_state] + 1

            if new_cost < cost_from_start.get(neighbor_state, float("inf")):
                cost_from_start[neighbor_state] = new_cost
                estimated_total_cost = new_cost + manhattan_distance(neighbor, goal)
                came_from[neighbor_state] = current_state

                tie_breaker += 1
                heapq.heappush(
                    open_set,
                    (estimated_total_cost, next_time, tie_breaker, neighbor_state),
                )

    raise RuntimeError("No safe path found before the planning time limit.")


def count_wait_steps(path: list[State]) -> int:
    """Count how many times the robot chooses to stay in the same cell."""
    waits = 0

    for index in range(1, len(path)):
        previous_point, _ = path[index - 1]
        current_point, _ = path[index]

        if previous_point == current_point:
            waits += 1

    return waits


def simulate_path(
    path: list[State], moving_obstacles: list[MovingObstacle]
) -> tuple[int, int]:
    """Replay the path and count collisions or execution failures."""
    collisions = 0
    failures = 0

    for index, (robot_position, time_step) in enumerate(path):
        dynamic_positions = moving_obstacle_positions(moving_obstacles, time_step)

        if robot_position in dynamic_positions:
            collisions += 1

        if index == 0:
            continue

        previous_robot_position, previous_time = path[index - 1]
        if time_step != previous_time + 1:
            failures += 1

        for obstacle in moving_obstacles:
            obstacle_previous = obstacle.position_at(previous_time)
            obstacle_current = obstacle.position_at(time_step)
            robot_swapped_with_obstacle = (
                previous_robot_position == obstacle_current
                and robot_position == obstacle_previous
            )

            if robot_swapped_with_obstacle:
                collisions += 1

    return collisions, failures


def make_ascii_map(obstacles: set[Point], path: list[State]) -> str:
    """Create a simple terminal visualization."""
    path_cells = {point for point, _ in path}
    start_dynamic_positions = moving_obstacle_positions(MOVING_OBSTACLES, 0)
    rows: list[str] = []

    for y in range(GRID_HEIGHT):
        row = []
        for x in range(GRID_WIDTH):
            point = (x, y)

            if point == START:
                row.append("S")
            elif point == GOAL:
                row.append("G")
            elif point in start_dynamic_positions:
                row.append("M")
            elif point in path_cells:
                row.append("*")
            elif point in obstacles:
                row.append("#")
            else:
                row.append(".")

        rows.append("".join(row))

    return "\n".join(rows)


def svg_rect(x: int, y: int, width: int, height: int, fill: str, stroke: str) -> str:
    return (
        f'<rect x="{x}" y="{y}" width="{width}" height="{height}" '
        f'fill="{fill}" stroke="{stroke}" />'
    )


def save_svg(
    obstacles: set[Point],
    path: list[State],
    moving_obstacles: list[MovingObstacle],
    output_path: Path,
) -> None:
    """Save a visual map of the warehouse and planned path."""
    cell = 34
    margin = 36
    legend_height = 54
    svg_width = margin * 2 + GRID_WIDTH * cell
    svg_height = margin * 2 + GRID_HEIGHT * cell + legend_height

    def cell_center(point: Point) -> tuple[int, int]:
        x, y = point
        return margin + x * cell + cell // 2, margin + y * cell + cell // 2

    robot_path = [point for point, _ in path]
    path_points = " ".join(f"{x},{y}" for x, y in map(cell_center, robot_path))
    path_cells = set(robot_path)
    start_dynamic_positions = moving_obstacle_positions(moving_obstacles, 0)

    parts = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{svg_width}" '
            f'height="{svg_height}" viewBox="0 0 {svg_width} {svg_height}">'
        ),
        '<rect width="100%" height="100%" fill="#f8fafc" />',
        (
            '<text x="36" y="24" font-family="Arial, sans-serif" '
            'font-size="18" font-weight="700" fill="#0f172a">'
            "2D Warehouse A* Path Planning"
            "</text>"
        ),
    ]

    for y in range(GRID_HEIGHT):
        for x in range(GRID_WIDTH):
            point = (x, y)
            px = margin + x * cell
            py = margin + y * cell

            if point in obstacles:
                fill = "#475569"
            elif point in start_dynamic_positions:
                fill = "#fef3c7"
            elif point in path_cells:
                fill = "#dbeafe"
            else:
                fill = "#ffffff"

            parts.append(svg_rect(px, py, cell, cell, fill, "#cbd5e1"))

    for shelf_name, shelf_x, shelf_y, shelf_width, shelf_height in SHELVES:
        label_x = margin + shelf_x * cell + shelf_width * cell / 2
        label_y = margin + shelf_y * cell + shelf_height * cell / 2
        parts.append(
            f'<text x="{label_x}" y="{label_y}" text-anchor="middle" '
            'dominant-baseline="middle" font-family="Arial, sans-serif" '
            'font-size="16" font-weight="700" fill="#ffffff">'
            f"Shelf {shelf_name}</text>"
        )

    parts.append(
        f'<polyline points="{path_points}" fill="none" stroke="#2563eb" '
        'stroke-width="7" stroke-linecap="round" stroke-linejoin="round" />'
    )

    for point, color, label in [
        (START, "#16a34a", "S"),
        (GOAL, "#dc2626", "G"),
    ]:
        cx, cy = cell_center(point)
        parts.append(f'<circle cx="{cx}" cy="{cy}" r="12" fill="{color}" />')
        parts.append(
            f'<text x="{cx}" y="{cy + 1}" text-anchor="middle" '
            'dominant-baseline="middle" font-family="Arial, sans-serif" '
            f'font-size="13" font-weight="700" fill="#ffffff">{label}</text>'
        )

    for obstacle in moving_obstacles:
        route_points = " ".join(
            f"{x},{y}" for x, y in map(cell_center, obstacle.route)
        )
        parts.append(
            f'<polyline points="{route_points}" fill="none" stroke="#f59e0b" '
            'stroke-width="4" stroke-dasharray="6 5" stroke-linecap="round" />'
        )

        cx, cy = cell_center(obstacle.position_at(0))
        parts.append(f'<circle cx="{cx}" cy="{cy}" r="11" fill="#f59e0b" />')
        parts.append(
            f'<text x="{cx}" y="{cy + 1}" text-anchor="middle" '
            'dominant-baseline="middle" font-family="Arial, sans-serif" '
            'font-size="11" font-weight="700" fill="#111827">M</text>'
        )

    legend_y = margin + GRID_HEIGHT * cell + 32
    legend_items = [
        ("#ffffff", "free cell"),
        ("#475569", "shelf / obstacle"),
        ("#dbeafe", "planned path"),
        ("#fef3c7", "moving obstacle at t=0"),
        ("#16a34a", "start"),
        ("#dc2626", "goal"),
    ]

    x_cursor = margin
    for color, label in legend_items:
        parts.append(svg_rect(x_cursor, legend_y - 14, 18, 18, color, "#94a3b8"))
        parts.append(
            f'<text x="{x_cursor + 26}" y="{legend_y}" '
            'font-family="Arial, sans-serif" font-size="13" fill="#334155">'
            f"{label}</text>"
        )
        x_cursor += 138

    parts.append("</svg>")
    output_path.write_text("\n".join(parts), encoding="utf-8")


def main() -> None:
    obstacles = build_obstacles()
    failures = 0

    if START in obstacles:
        raise ValueError("The start position is inside a shelf.")

    if GOAL in obstacles:
        raise ValueError("The goal position is inside a shelf.")

    if START in moving_obstacle_positions(MOVING_OBSTACLES, 0):
        raise ValueError("The start position is occupied by a moving obstacle.")

    start_time = time.perf_counter()
    try:
        path, expanded_nodes = astar(START, GOAL, obstacles, MOVING_OBSTACLES)
    except RuntimeError as error:
        failures += 1
        elapsed_seconds = time.perf_counter() - start_time

        print("Dynamic A* warehouse planning log")
        print("---------------------------------")
        print(f"Grid size: {GRID_WIDTH} columns x {GRID_HEIGHT} rows")
        print(f"Start: {START}")
        print(f"Goal: {GOAL}")
        print(f"Moving obstacles: {len(MOVING_OBSTACLES)}")
        print(f"Time taken: {elapsed_seconds * 1000:.3f} ms")
        print("Collisions: 0")
        print(f"Failures: {failures}")
        print(f"Failure reason: {error}")
        return
    elapsed_seconds = time.perf_counter() - start_time
    collisions, simulation_failures = simulate_path(path, MOVING_OBSTACLES)
    failures += simulation_failures

    output_path = Path(__file__).with_name("warehouse_path.svg")
    save_svg(obstacles, path, MOVING_OBSTACLES, output_path)

    print("Dynamic A* warehouse planning log")
    print("---------------------------------")
    print(f"Grid size: {GRID_WIDTH} columns x {GRID_HEIGHT} rows")
    print(f"Start: {START}")
    print(f"Goal: {GOAL}")
    print(f"Moving obstacles: {len(MOVING_OBSTACLES)}")
    for obstacle in MOVING_OBSTACLES:
        print(f"  - {obstacle.name}: {list(obstacle.route)}")
    print(f"Path length: {len(path) - 1} time steps")
    print(f"Cells in path: {len(path)}")
    print(f"Wait steps: {count_wait_steps(path)}")
    print(f"Expanded nodes: {expanded_nodes}")
    print(f"Time taken: {elapsed_seconds * 1000:.3f} ms")
    print(f"Collisions: {collisions}")
    print(f"Failures: {failures}")
    print(f"SVG visualization: {output_path}")
    print()
    print(make_ascii_map(obstacles, path))


if __name__ == "__main__":
    main()
