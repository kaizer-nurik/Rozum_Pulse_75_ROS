#include "rozum_rrtstar_planner/rrtstar_planning_context.hpp"

#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/planning_interface/planning_response.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include "rozum_rrtstar_planner/rrtstar_interface.hpp"

namespace rozum_rrtstar_planner
{

RRTStarPlanningContext::RRTStarPlanningContext(
    const std::string & name,
    const std::string & group,
    const planning_scene::PlanningSceneConstPtr & planning_scene)
  : planning_interface::PlanningContext(name, group)
  , planning_scene_(planning_scene)
{
}

void RRTStarPlanningContext::solve(planning_interface::MotionPlanResponse & res)
{
  planning_interface::MotionPlanDetailedResponse detailed;
  solve(detailed);

  res.error_code = detailed.error_code;

  if (!detailed.trajectory.empty()) {
    res.trajectory = detailed.trajectory.front();
  } else {
    res.trajectory.reset();
  }

  if (!detailed.processing_time.empty()) {
    res.planning_time = detailed.processing_time.front();
  } else {
    res.planning_time = 0.0;
  }
}

void RRTStarPlanningContext::solve(planning_interface::MotionPlanDetailedResponse & res)
{
  const auto & req = getMotionPlanRequest();
  auto robot_model = planning_scene_->getRobotModel();

  // стартовое состояние
  moveit::core::RobotState start_state(robot_model);
  start_state = planning_scene_->getCurrentState();
  moveit::core::robotStateMsgToRobotState(req.start_state, start_state);

  const auto * jmg = start_state.getJointModelGroup(getGroupName());
  if (!jmg) {
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
    return;
  }

  std::vector<double> q_start;
  start_state.copyJointGroupPositions(jmg, q_start);

  if (req.goal_constraints.empty()) {
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return;
  }

  // Используем только joint goal из первого goal_constraints
  std::vector<double> q_goal = q_start;
  const auto & joint_constraints = req.goal_constraints[0].joint_constraints;
  for (const auto & jc : joint_constraints) {
    int idx = jmg->getVariableGroupIndex(jc.joint_name);
    if (idx >= 0) {
      q_goal[static_cast<std::size_t>(idx)] = jc.position;
    }
  }

  RRTStarInterface rrt(planning_scene_, jmg);
  rrt.setStart(q_start);
  rrt.setGoal(q_goal);

  std::vector<std::vector<double>> path;
  const double allowed_time = req.allowed_planning_time > 0.0 ? req.allowed_planning_time : 5.0;

  const bool ok = rrt.solve(path, allowed_time);

  res.trajectory.clear();
  res.description.clear();
  res.processing_time.clear();

  if (!ok) {
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return;
  }

  robot_trajectory::RobotTrajectory traj(robot_model, getGroupName());
  moveit::core::RobotState waypoint_state(robot_model);
  waypoint_state = start_state;

  for (const auto & q : path) {
    waypoint_state.setJointGroupPositions(jmg, q);
    waypoint_state.update();
    traj.addSuffixWayPoint(waypoint_state, 0.1);
  }

  auto traj_ptr = std::make_shared<robot_trajectory::RobotTrajectory>(traj);
  res.trajectory.push_back(traj_ptr);
  res.description.push_back("rrtstar");
  res.processing_time.push_back(allowed_time);
  res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
}

bool RRTStarPlanningContext::terminate()
{
  return false;
}

void RRTStarPlanningContext::clear()
{
}

}  // namespace rozum_rrtstar_planner
