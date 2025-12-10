#pragma once

#include <memory>
#include <string>

#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_scene/planning_scene.hpp>

#include "rozum_rrtstar_planner/rrtstar_interface.hpp"

namespace rozum_rrtstar_planner
{

class RRTStarPlanningContext : public planning_interface::PlanningContext
{
public:
  RRTStarPlanningContext(const std::string & name,
                         const std::string & group,
                         const planning_scene::PlanningSceneConstPtr & planning_scene);

  void solve(planning_interface::MotionPlanResponse & res) override;
  void solve(planning_interface::MotionPlanDetailedResponse & res) override;

  bool terminate() override;
  void clear() override;

private:
  planning_scene::PlanningSceneConstPtr planning_scene_;
};

}  // namespace rozum_rrtstar_planner
