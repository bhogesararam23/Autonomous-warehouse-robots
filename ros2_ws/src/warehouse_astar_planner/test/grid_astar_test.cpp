#include "warehouse_astar_planner/grid_astar.hpp"

#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using warehouse_astar_planner::CostGrid;
using warehouse_astar_planner::GridCell;
using warehouse_astar_planner::PlanningCancelled;
using warehouse_astar_planner::PlanningError;
using warehouse_astar_planner::PlannerOptions;
using warehouse_astar_planner::findPath;

namespace
{

void require(bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

CostGrid makeGrid(std::size_t width, std::size_t height, std::uint8_t fill = 0)
{
  return {width, height, std::vector<std::uint8_t>(width * height, fill)};
}

std::uint8_t & at(CostGrid & grid, std::size_t x, std::size_t y)
{
  return grid.costs[y * grid.width + x];
}

void validatePath(const CostGrid & grid, const std::vector<GridCell> & path)
{
  require(!path.empty(), "path must not be empty");
  for (std::size_t i = 0; i < path.size(); ++i) {
    const auto cell = path[i];
    require(cell.x < grid.width && cell.y < grid.height, "path left grid bounds");
    require(grid.costs[cell.y * grid.width + cell.x] < 253, "path entered lethal cell");
    if (i == 0) {
      continue;
    }
    const auto previous = path[i - 1];
    const auto dx = previous.x > cell.x ? previous.x - cell.x : cell.x - previous.x;
    const auto dy = previous.y > cell.y ? previous.y - cell.y : cell.y - previous.y;
    require(dx <= 1 && dy <= 1 && (dx + dy) > 0, "path has a non-neighbor transition");
    if (dx == 1 && dy == 1) {
      require(grid.costs[previous.y * grid.width + cell.x] < 253,
        "diagonal path clipped blocked side cell");
      require(grid.costs[cell.y * grid.width + previous.x] < 253,
        "diagonal path clipped blocked side cell");
    }
  }
}

void testOpenGridUsesOptimalDiagonalPath()
{
  const auto grid = makeGrid(8, 6);
  const auto path = findPath(grid, {0, 0}, {7, 5});
  validatePath(grid, path);
  require(path.front() == GridCell{0, 0}, "path must include start");
  require(path.back() == GridCell{7, 5}, "path must include goal");
  require(path.size() == 8, "open-grid A* should use five diagonals and two straight steps");
}

void testBlockedPassageUsesValidDetour()
{
  auto grid = makeGrid(7, 5);
  for (std::size_t y = 0; y < grid.height; ++y) {
    if (y != 4) {
      at(grid, 3, y) = 254;
    }
  }
  const auto path = findPath(grid, {0, 2}, {6, 2});
  validatePath(grid, path);
  require(path.back() == GridCell{6, 2}, "detour did not reach the goal");
  bool uses_gap = false;
  for (const auto cell : path) {
    uses_gap = uses_gap || cell == GridCell{3, 4};
  }
  require(uses_gap, "path should route through the only open shelf gap");
}

void testCostPenaltyAvoidsInflatedCells()
{
  auto grid = makeGrid(7, 5);
  for (std::size_t x = 1; x < 6; ++x) {
    at(grid, x, 2) = 252;
  }
  const auto path = findPath(grid, {0, 2}, {6, 2});
  validatePath(grid, path);
  for (const auto cell : path) {
    require(cell.y != 2 || cell.x == 0 || cell.x == 6,
      "planner should prefer the longer free route to expensive inflated cells");
  }
}

void testUnreachableAndBlockedEndpointsFail()
{
  auto grid = makeGrid(3, 1);
  at(grid, 1, 0) = 254;
  bool unreachable = false;
  try {
    (void)findPath(grid, {0, 0}, {2, 0});
  } catch (const PlanningError &) {
    unreachable = true;
  }
  require(unreachable, "unreachable goal must fail explicitly");

  bool blocked_goal = false;
  try {
    (void)findPath(grid, {0, 0}, {1, 0});
  } catch (const PlanningError &) {
    blocked_goal = true;
  }
  require(blocked_goal, "blocked goal must fail explicitly");
}

void testStartEqualsGoalIsOneCellPath()
{
  const auto grid = makeGrid(1, 1);
  const auto path = findPath(grid, {0, 0}, {0, 0});
  require(path.size() == 1 && path.front() == GridCell{0, 0},
    "start equal to goal should return a one-cell path");
}

void testDiagonalCornerCuttingIsForbidden()
{
  auto grid = makeGrid(2, 2);
  at(grid, 1, 0) = 254;
  at(grid, 0, 1) = 254;
  bool rejected = false;
  try {
    (void)findPath(grid, {0, 0}, {1, 1});
  } catch (const PlanningError &) {
    rejected = true;
  }
  require(rejected, "planner must not pass diagonally between two blocked cells");
}

void testUnknownSpacePolicyIsExplicit()
{
  auto grid = makeGrid(3, 1);
  at(grid, 1, 0) = 255;
  bool blocked_by_default = false;
  try {
    (void)findPath(grid, {0, 0}, {2, 0});
  } catch (const PlanningError &) {
    blocked_by_default = true;
  }
  require(blocked_by_default, "unknown space should be blocked by default");

  PlannerOptions options;
  options.allow_unknown = true;
  const auto path = findPath(grid, {0, 0}, {2, 0}, options);
  require(path.size() == 3, "allow_unknown should permit a path through unknown cells");
}

void testMalformedGridAndCancellationFailExplicitly()
{
  CostGrid malformed{2, 2, {0, 0, 0}};
  bool malformed_rejected = false;
  try {
    (void)findPath(malformed, {0, 0}, {1, 1});
  } catch (const PlanningError &) {
    malformed_rejected = true;
  }
  require(malformed_rejected, "malformed cost buffer should fail validation");

  const auto grid = makeGrid(3, 3);
  bool cancelled = false;
  try {
    (void)findPath(grid, {0, 0}, {2, 2}, PlannerOptions{}, []() {return true;});
  } catch (const PlanningCancelled &) {
    cancelled = true;
  }
  require(cancelled, "planner must honor cancellation before searching");
}

}  // namespace

int main()
{
  const std::vector<std::pair<std::string, std::function<void()>>> tests{
    {"open-grid optimal path", testOpenGridUsesOptimalDiagonalPath},
    {"blocked passage detour", testBlockedPassageUsesValidDetour},
    {"inflation cost preference", testCostPenaltyAvoidsInflatedCells},
    {"unreachable and blocked endpoints", testUnreachableAndBlockedEndpointsFail},
    {"start equals goal", testStartEqualsGoalIsOneCellPath},
    {"no diagonal corner-cutting", testDiagonalCornerCuttingIsForbidden},
    {"explicit unknown-space policy", testUnknownSpacePolicyIsExplicit},
    {"validation and cancellation", testMalformedGridAndCancellationFailExplicitly}};

  try {
    for (const auto & test : tests) {
      test.second();
      std::cout << "[PASS] " << test.first << '\n';
    }
  } catch (const std::exception & error) {
    std::cerr << "[FAIL] " << error.what() << '\n';
    return 1;
  }

  std::cout << tests.size() << " C++ core tests passed\n";
  return 0;
}
