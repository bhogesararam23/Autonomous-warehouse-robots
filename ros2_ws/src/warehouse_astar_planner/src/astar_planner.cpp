#include "warehouse_astar_planner/astar_planner.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "warehouse_astar_planner/grid_astar.hpp"

namespace warehouse_astar_planner
{

void AStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  if (!node_) {
    throw nav2_core::PlannerException("AStarPlanner could not lock its lifecycle node");
  }
  if (!costmap_ros) {
    throw nav2_core::PlannerException("AStarPlanner received a null Costmap2DROS");
  }

  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  costmap_ = costmap_ros_->getCostmap();
  global_frame_ = costmap_ros_->getGlobalFrameID();
  if (costmap_ == nullptr) {
    throw nav2_core::PlannerException("AStarPlanner received a null Costmap2D");
  }

  const auto declare_if_missing = [this](const std::string & parameter_name,
      const rclcpp::ParameterValue & default_value)
    {
      if (!node_->has_parameter(parameter_name)) {
        node_->declare_parameter(parameter_name, default_value);
      }
    };

  const auto allow_unknown_parameter = name_ + ".allow_unknown";
  const auto cost_penalty_parameter = name_ + ".cost_penalty";
  const auto unknown_multiplier_parameter = name_ + ".unknown_cost_multiplier";
  const auto max_expansions_parameter = name_ + ".max_expansions";

  declare_if_missing(allow_unknown_parameter, rclcpp::ParameterValue(false));
  declare_if_missing(cost_penalty_parameter, rclcpp::ParameterValue(2.0));
  declare_if_missing(unknown_multiplier_parameter, rclcpp::ParameterValue(5.0));
  declare_if_missing(max_expansions_parameter, rclcpp::ParameterValue(250000));

  std::int64_t max_expansions = 0;
  node_->get_parameter(allow_unknown_parameter, allow_unknown_);
  node_->get_parameter(cost_penalty_parameter, cost_penalty_);
  node_->get_parameter(unknown_multiplier_parameter, unknown_cost_multiplier_);
  node_->get_parameter(max_expansions_parameter, max_expansions);
  if (max_expansions <= 0 ||
    static_cast<std::uint64_t>(max_expansions) >
    static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max()))
  {
    throw nav2_core::PlannerException(
            "AStarPlanner max_expansions must be a positive, addressable integer");
  }
  max_expansions_ = static_cast<std::size_t>(max_expansions);

  if (!std::isfinite(cost_penalty_) || cost_penalty_ < 0.0) {
    throw nav2_core::PlannerException("AStarPlanner cost_penalty must be finite and non-negative");
  }
  if (!std::isfinite(unknown_cost_multiplier_) || unknown_cost_multiplier_ < 1.0) {
    throw nav2_core::PlannerException(
            "AStarPlanner unknown_cost_multiplier must be finite and at least 1");
  }

  RCLCPP_INFO(
    node_->get_logger(), "Configured %s in frame '%s' (unknown cells %s)",
    name_.c_str(), global_frame_.c_str(), allow_unknown_ ? "allowed" : "blocked");
}

void AStarPlanner::cleanup()
{
  costmap_ = nullptr;
  costmap_ros_.reset();
  tf_.reset();
  node_.reset();
  name_.clear();
  global_frame_.clear();
}

void AStarPlanner::activate()
{
  // The planner owns no publishers or background workers.
}

void AStarPlanner::deactivate()
{
  // The planner owns no publishers or background workers.
}

nav_msgs::msg::Path AStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  nav_msgs::msg::Path output;
  if (!node_ || costmap_ == nullptr) {
    throw nav2_core::PlannerException("AStarPlanner was called before configuration");
  }

  output.header.frame_id = global_frame_;
  output.header.stamp = node_->now();
  if (cancel_checker && cancel_checker()) {
    return output;
  }
  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    throw nav2_core::PlannerException(
            "AStarPlanner start and goal must be expressed in the costmap global frame '" +
            global_frame_ + "'");
  }
  if (!std::isfinite(start.pose.position.x) || !std::isfinite(start.pose.position.y) ||
    !std::isfinite(goal.pose.position.x) || !std::isfinite(goal.pose.position.y))
  {
    throw nav2_core::PlannerException("AStarPlanner received a non-finite start or goal pose");
  }

  CostGrid grid;
  GridCell start_cell{};
  GridCell goal_cell{};
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
    unsigned int start_x = 0;
    unsigned int start_y = 0;
    unsigned int goal_x = 0;
    unsigned int goal_y = 0;
    if (!costmap_->worldToMap(
        start.pose.position.x, start.pose.position.y, start_x, start_y))
    {
      throw nav2_core::PlannerException("AStarPlanner start pose is outside the costmap bounds");
    }
    if (!costmap_->worldToMap(
        goal.pose.position.x, goal.pose.position.y, goal_x, goal_y))
    {
      throw nav2_core::PlannerException("AStarPlanner goal pose is outside the costmap bounds");
    }

    grid.width = costmap_->getSizeInCellsX();
    grid.height = costmap_->getSizeInCellsY();
    grid.costs.resize(grid.width * grid.height);
    for (std::size_t y = 0; y < grid.height; ++y) {
      for (std::size_t x = 0; x < grid.width; ++x) {
        grid.costs[y * grid.width + x] = costmap_->getCost(
          static_cast<unsigned int>(x), static_cast<unsigned int>(y));
      }
    }

    start_cell = {start_x, start_y};
    goal_cell = {goal_x, goal_y};
    resolution = costmap_->getResolution();
    origin_x = costmap_->getOriginX();
    origin_y = costmap_->getOriginY();
  }

  PlannerOptions options;
  options.lethal_threshold = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  options.allow_unknown = allow_unknown_;
  options.cost_penalty = cost_penalty_;
  options.unknown_cost_multiplier = unknown_cost_multiplier_;
  options.max_expansions = max_expansions_;

  std::vector<GridCell> cells;
  try {
    cells = findPath(grid, start_cell, goal_cell, options, cancel_checker);
  } catch (const PlanningCancelled &) {
    return output;
  } catch (const PlanningError & error) {
    throw nav2_core::PlannerException(std::string("AStarPlanner: ") + error.what());
  }

  struct PlanPoint
  {
    double x;
    double y;
  };
  std::vector<PlanPoint> points;
  points.reserve(cells.size());
  for (std::size_t i = 0; i < cells.size(); ++i) {
    if (i == 0) {
      points.push_back({start.pose.position.x, start.pose.position.y});
    } else if (i + 1 == cells.size()) {
      points.push_back({goal.pose.position.x, goal.pose.position.y});
    } else {
      points.push_back({
        origin_x + (static_cast<double>(cells[i].x) + 0.5) * resolution,
        origin_y + (static_cast<double>(cells[i].y) + 0.5) * resolution});
    }
  }
  if (cells.size() == 1) {
    const auto same_position = std::hypot(
      start.pose.position.x - goal.pose.position.x,
      start.pose.position.y - goal.pose.position.y) < 1.0e-9;
    if (!same_position) {
      points.push_back({goal.pose.position.x, goal.pose.position.y});
    }
  }

  output.poses.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = output.header;
    pose.pose.position.x = points[i].x;
    pose.pose.position.y = points[i].y;
    pose.pose.position.z = 0.0;

    double yaw = 0.0;
    if (i + 1 < points.size()) {
      yaw = std::atan2(points[i + 1].y - points[i].y, points[i + 1].x - points[i].x);
    } else {
      const auto & orientation = goal.pose.orientation;
      yaw = std::atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z));
    }
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(yaw * 0.5);
    pose.pose.orientation.w = std::cos(yaw * 0.5);
    output.poses.push_back(std::move(pose));
  }

  return output;
}

}  // namespace warehouse_astar_planner

PLUGINLIB_EXPORT_CLASS(warehouse_astar_planner::AStarPlanner, nav2_core::GlobalPlanner)
