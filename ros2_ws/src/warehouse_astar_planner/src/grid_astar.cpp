#include "warehouse_astar_planner/grid_astar.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace warehouse_astar_planner
{
namespace
{

constexpr std::uint8_t kUnknownCost = 255;
constexpr double kDiagonalCost = 1.4142135623730950488;
constexpr double kMaxKnownCost = 252.0;

struct Move
{
  int dx;
  int dy;
  double distance;
};

constexpr std::array<Move, 8> kMoves{{
  {1, 0, 1.0}, {-1, 0, 1.0}, {0, 1, 1.0}, {0, -1, 1.0},
  {1, 1, kDiagonalCost}, {1, -1, kDiagonalCost},
  {-1, 1, kDiagonalCost}, {-1, -1, kDiagonalCost}}};

struct QueueEntry
{
  double estimated_total;
  double cost_so_far;
  std::uint64_t order;
  std::size_t index;
};

struct LaterEntry
{
  bool operator()(const QueueEntry & left, const QueueEntry & right) const noexcept
  {
    if (left.estimated_total != right.estimated_total) {
      return left.estimated_total > right.estimated_total;
    }
    return left.order > right.order;
  }
};

std::size_t indexOf(const CostGrid & grid, std::size_t x, std::size_t y)
{
  return y * grid.width + x;
}

GridCell cellOf(const CostGrid & grid, std::size_t index)
{
  return {index % grid.width, index / grid.width};
}

double octileDistance(GridCell from, GridCell to)
{
  const auto dx = from.x > to.x ? from.x - to.x : to.x - from.x;
  const auto dy = from.y > to.y ? from.y - to.y : to.y - from.y;
  const auto diagonal = std::min(dx, dy);
  const auto straight = std::max(dx, dy) - diagonal;
  return static_cast<double>(straight) + kDiagonalCost * static_cast<double>(diagonal);
}

bool isTraversable(
  const CostGrid & grid, std::size_t x, std::size_t y, const PlannerOptions & options)
{
  const auto cost = grid.costs[indexOf(grid, x, y)];
  if (cost == kUnknownCost) {
    return options.allow_unknown;
  }
  return cost < options.lethal_threshold;
}

double entryCost(std::uint8_t cost, const PlannerOptions & options)
{
  if (cost == kUnknownCost) {
    return options.unknown_cost_multiplier;
  }
  return 1.0 + options.cost_penalty * static_cast<double>(cost) / kMaxKnownCost;
}

void validate(
  const CostGrid & grid, GridCell start, GridCell goal, const PlannerOptions & options)
{
  if (grid.width == 0 || grid.height == 0) {
    throw PlanningError("Cost grid dimensions must be positive");
  }
  if (grid.width > std::numeric_limits<std::size_t>::max() / grid.height ||
    grid.width > static_cast<std::size_t>(std::numeric_limits<long long>::max()) ||
    grid.height > static_cast<std::size_t>(std::numeric_limits<long long>::max()))
  {
    throw PlanningError("Cost grid dimensions overflow addressable storage");
  }
  if (grid.costs.size() != grid.width * grid.height) {
    throw PlanningError("Cost grid buffer size does not match its dimensions");
  }
  if (options.lethal_threshold == kUnknownCost) {
    throw PlanningError("Lethal threshold must leave 255 reserved for unknown space");
  }
  if (!std::isfinite(options.cost_penalty) || options.cost_penalty < 0.0) {
    throw PlanningError("Cost penalty must be a finite, non-negative number");
  }
  if (!std::isfinite(options.unknown_cost_multiplier) ||
    options.unknown_cost_multiplier < 1.0)
  {
    throw PlanningError("Unknown-space multiplier must be finite and at least 1");
  }
  if (options.max_expansions == 0) {
    throw PlanningError("Maximum expansion count must be positive");
  }
  if (start.x >= grid.width || start.y >= grid.height ||
    goal.x >= grid.width || goal.y >= grid.height)
  {
    throw PlanningError("Start and goal must be inside the cost grid");
  }
  if (!isTraversable(grid, start.x, start.y, options)) {
    throw PlanningError("Start cell is blocked or unknown");
  }
  if (!isTraversable(grid, goal.x, goal.y, options)) {
    throw PlanningError("Goal cell is blocked or unknown");
  }
}

}  // namespace

std::vector<GridCell> findPath(
  const CostGrid & grid,
  GridCell start,
  GridCell goal,
  const PlannerOptions & options,
  const std::function<bool()> & cancel_checker)
{
  validate(grid, start, goal, options);
  if (cancel_checker && cancel_checker()) {
    throw PlanningCancelled();
  }
  if (start == goal) {
    return {start};
  }

  const auto cell_count = grid.width * grid.height;
  const auto start_index = indexOf(grid, start.x, start.y);
  const auto goal_index = indexOf(grid, goal.x, goal.y);
  const auto no_parent = cell_count;
  std::vector<double> best_cost(cell_count, std::numeric_limits<double>::infinity());
  std::vector<std::size_t> parent(cell_count, no_parent);
  std::vector<bool> closed(cell_count, false);
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, LaterEntry> frontier;

  std::uint64_t insertion_order = 0;
  std::size_t expanded = 0;
  best_cost[start_index] = 0.0;
  frontier.push({octileDistance(start, goal), 0.0, insertion_order++, start_index});

  while (!frontier.empty()) {
    if (cancel_checker && cancel_checker()) {
      throw PlanningCancelled();
    }
    if (expanded >= options.max_expansions) {
      throw PlanningError("A* search reached the configured expansion limit");
    }

    const auto current = frontier.top();
    frontier.pop();
    if (closed[current.index] || current.cost_so_far > best_cost[current.index]) {
      continue;
    }
    closed[current.index] = true;
    ++expanded;

    if (current.index == goal_index) {
      std::vector<GridCell> path;
      auto index = goal_index;
      while (index != no_parent) {
        path.push_back(cellOf(grid, index));
        if (index == start_index) {
          break;
        }
        index = parent[index];
      }
      if (path.empty() || path.back() != start) {
        throw PlanningError("A* parent chain did not reach the start cell");
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    const auto current_cell = cellOf(grid, current.index);
    for (const auto & move : kMoves) {
      const auto next_x_signed = static_cast<long long>(current_cell.x) + move.dx;
      const auto next_y_signed = static_cast<long long>(current_cell.y) + move.dy;
      if (next_x_signed < 0 || next_y_signed < 0 ||
        next_x_signed >= static_cast<long long>(grid.width) ||
        next_y_signed >= static_cast<long long>(grid.height))
      {
        continue;
      }

      const auto next_x = static_cast<std::size_t>(next_x_signed);
      const auto next_y = static_cast<std::size_t>(next_y_signed);
      if (!isTraversable(grid, next_x, next_y, options)) {
        continue;
      }

      if (move.dx != 0 && move.dy != 0) {
        const auto side_x = static_cast<std::size_t>(
          static_cast<long long>(current_cell.x) + move.dx);
        const auto side_y = static_cast<std::size_t>(
          static_cast<long long>(current_cell.y) + move.dy);
        if (!isTraversable(grid, side_x, current_cell.y, options) ||
          !isTraversable(grid, current_cell.x, side_y, options))
        {
          continue;
        }
      }

      const auto next_index = indexOf(grid, next_x, next_y);
      if (closed[next_index]) {
        continue;
      }
      const auto candidate_cost = current.cost_so_far +
        move.distance * entryCost(grid.costs[next_index], options);
      if (candidate_cost >= best_cost[next_index]) {
        continue;
      }

      best_cost[next_index] = candidate_cost;
      parent[next_index] = current.index;
      const auto next_cell = GridCell{next_x, next_y};
      frontier.push({
        candidate_cost + octileDistance(next_cell, goal), candidate_cost,
        insertion_order++, next_index});
    }
  }

  throw PlanningError("No traversable path exists between start and goal");
}

}  // namespace warehouse_astar_planner
