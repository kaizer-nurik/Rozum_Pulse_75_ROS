#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include "rozum_rrtstar_planner/rrtstar_planning_context.hpp"

namespace rozum_rrtstar_planner
{

class RRTStarPlannerManager : public planning_interface::PlannerManager
{
public:
  RRTStarPlannerManager() = default;
  ~RRTStarPlannerManager() override = default;

  bool initialize(const moveit::core::RobotModelConstPtr & model,
                  const rclcpp::Node::SharedPtr & node,
                  const std::string & parameter_namespace) override;

  std::string getDescription() const override;

  void getPlanningAlgorithms(std::vector<std::string> & algs) const override;

  bool canServiceRequest(const planning_interface::MotionPlanRequest & req) const override;

  planning_interface::PlanningContextPtr getPlanningContext(
      const planning_scene::PlanningSceneConstPtr & planning_scene,
      const planning_interface::MotionPlanRequest & req,
      moveit_msgs::msg::MoveItErrorCodes & error_code) const override;
};

}  // namespace rozum_rrtstar_planner
