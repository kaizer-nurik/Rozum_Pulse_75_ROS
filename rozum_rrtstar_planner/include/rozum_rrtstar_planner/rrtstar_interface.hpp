#pragma once

#include <random>
#include <vector>
#include <memory>

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/robot_state.hpp>

namespace rozum_rrtstar_planner
{

struct Node
{
  std::vector<double> q;
  int parent_index;
  double cost;
};

class RRTStarInterface
{
public:
  RRTStarInterface(
    const planning_scene::PlanningSceneConstPtr & scene,
    const moveit::core::JointModelGroup * jmg);

  void setStart(const std::vector<double> & q_start);
  void setGoal(const std::vector<double> & q_goal);

  bool solve(
    std::vector<std::vector<double>> & path,
    double max_planning_time,
    std::size_t max_iterations = 2000);

private:
  planning_scene::PlanningSceneConstPtr scene_;
  const moveit::core::JointModelGroup * jmg_;

  std::vector<Node> tree_;
  std::vector<double> q_start_;
  std::vector<double> q_goal_;
  std::mt19937 rng_;

  double step_size_ = 0.05;
  double goal_bias_ = 0.05;
  double neighbor_radius_ = 0.2;

  std::vector<double> sampleRandom();

  int nearest(const std::vector<double> & q_rand) const;
  void steer(
    const std::vector<double> & q_from,
    const std::vector<double> & q_to,
    std::vector<double> & q_new) const;
  bool isStateValid(const std::vector<double> & q) const;
  double distance(
    const std::vector<double> & a,
    const std::vector<double> & b) const;
  std::vector<int> nearIndices(const std::vector<double> & q) const;
  void reconstructPath(int goal_index, std::vector<std::vector<double>> & path) const;
};

}  // namespace rozum_rrtstar_planner
