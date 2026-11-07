#ifndef WAREHOUSE_ASTAR_PLANNER__ASTAR_PLANNER_HPP_
#define WAREHOUSE_ASTAR_PLANNER__ASTAR_PLANNER_HPP_

#include <cstddef>
#include <functional>
#include <memory>
#include <string>

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace warehouse_astar_planner
{

class AStarPlanner final : public nav2_core::GlobalPlanner
{
public:
  AStarPlanner() = default;
  ~AStarPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;
  std::string global_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};

  bool allow_unknown_{false};
  double cost_penalty_{2.0};
  double unknown_cost_multiplier_{5.0};
  std::size_t max_expansions_{250000};
};

}  // namespace warehouse_astar_planner

#endif  // WAREHOUSE_ASTAR_PLANNER__ASTAR_PLANNER_HPP_
