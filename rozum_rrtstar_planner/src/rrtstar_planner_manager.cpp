#include "rozum_rrtstar_planner/rrtstar_planner_manager.hpp"

#include <class_loader/class_loader.hpp>

namespace rozum_rrtstar_planner
{

bool RRTStarPlannerManager::initialize(
  const moveit::core::RobotModelConstPtr & model,
  const rclcpp::Node::SharedPtr & node,
  const std::string & parameter_namespace)
{
  (void)model;
  (void)node;
  (void)parameter_namespace;

  // Здесь можно читать параметры RRT* из ROS-параметров, если понадобится.
  return true;
}

std::string RRTStarPlannerManager::getDescription() const
{
  return "Custom RRT* planner";
}

void RRTStarPlannerManager::getPlanningAlgorithms(std::vector<std::string> & algs) const
{
  algs.clear();
  algs.emplace_back("RRTstar");
}

bool RRTStarPlannerManager::canServiceRequest(
  const planning_interface::MotionPlanRequest & req) const
{
  // Пока что обслуживаем любой запрос, у которого задана группа
  return !req.group_name.empty();
}

planning_interface::PlanningContextPtr RRTStarPlannerManager::getPlanningContext(
  const planning_scene::PlanningSceneConstPtr & planning_scene,
  const planning_interface::MotionPlanRequest & req,
  moveit_msgs::msg::MoveItErrorCodes & error_code) const
{
  if (!planning_scene) {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return planning_interface::PlanningContextPtr();
  }

  if (req.group_name.empty()) {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
    return planning_interface::PlanningContextPtr();
  }

  error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

  auto context = std::make_shared<RRTStarPlanningContext>(
    "rrtstar_planning_context", req.group_name, planning_scene);

  context->setMotionPlanRequest(req);
  return context;
}

}  // namespace rozum_rrtstar_planner

CLASS_LOADER_REGISTER_CLASS(
  rozum_rrtstar_planner::RRTStarPlannerManager,
  planning_interface::PlannerManager)
