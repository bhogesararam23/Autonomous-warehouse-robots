#ifndef WAREHOUSE_ASTAR_PLANNER__GRID_ASTAR_HPP_
#define WAREHOUSE_ASTAR_PLANNER__GRID_ASTAR_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

namespace warehouse_astar_planner
{

struct GridCell
{
  std::size_t x;
  std::size_t y;

  bool operator==(const GridCell & other) const noexcept
  {
    return x == other.x && y == other.y;
  }

  bool operator!=(const GridCell & other) const noexcept
  {
    return !(*this == other);
  }
};

/**
 * A row-major, immutable snapshot of a Nav2-style 2D costmap.
 * Cost values follow nav2_costmap_2d conventions: 0 is free, 253+ is
 * non-traversable by default, and 255 is unknown.
 */
struct CostGrid
{
  std::size_t width{0};
  std::size_t height{0};
  std::vector<std::uint8_t> costs;
};

struct PlannerOptions
{
  std::uint8_t lethal_threshold{253};
  bool allow_unknown{false};
  double cost_penalty{2.0};
  double unknown_cost_multiplier{5.0};
  std::size_t max_expansions{250000};
};

class PlanningError : public std::runtime_error
{
public:
  explicit PlanningError(const std::string & message)
  : std::runtime_error(message) {}
};

class PlanningCancelled : public PlanningError
{
public:
  PlanningCancelled()
  : PlanningError("Planning cancelled") {}
};

/**
 * Plan a shortest, cost-aware path over an 8-connected grid using A*.
 * Diagonal moves are rejected when either adjacent cardinal cell is blocked,
 * preventing corner-cutting. The returned path includes both endpoints.
 * Throws PlanningError for malformed grids, invalid endpoints, or no path.
 */
std::vector<GridCell> findPath(
  const CostGrid & grid,
  GridCell start,
  GridCell goal,
  const PlannerOptions & options = PlannerOptions{},
  const std::function<bool()> & cancel_checker = {});

}  // namespace warehouse_astar_planner

#endif  // WAREHOUSE_ASTAR_PLANNER__GRID_ASTAR_HPP_
