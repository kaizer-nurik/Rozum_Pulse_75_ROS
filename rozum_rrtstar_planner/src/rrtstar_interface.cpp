#include "rozum_rrtstar_planner/rrtstar_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/robot_state/robot_state.hpp>

namespace rozum_rrtstar_planner
{

RRTStarInterface::RRTStarInterface(
  const planning_scene::PlanningSceneConstPtr & scene,
  const moveit::core::JointModelGroup * jmg)
: scene_(scene), jmg_(jmg)
{
  std::random_device rd;
  rng_ = std::mt19937(rd());
}

void RRTStarInterface::setStart(const std::vector<double> & q_start)
{
  q_start_ = q_start;
}

void RRTStarInterface::setGoal(const std::vector<double> & q_goal)
{
  q_goal_ = q_goal;
}

bool RRTStarInterface::isStateValid(const std::vector<double> & q) const
{
  moveit::core::RobotState state(scene_->getRobotModel());
  state.setJointGroupPositions(jmg_, q);
  state.update();

  collision_detection::CollisionRequest req;
  collision_detection::CollisionResult res;
  scene_->checkCollision(req, res, state);
  return !res.collision;
}

double RRTStarInterface::distance(
  const std::vector<double> & a,
  const std::vector<double> & b) const
{
  double d2 = 0.0;
  const std::size_t n = a.size();
  for (std::size_t i = 0; i < n; ++i) {
    const double diff = a[i] - b[i];
    d2 += diff * diff;
  }
  return std::sqrt(d2);
}

std::vector<double> RRTStarInterface::sampleRandom()
{
  // goal bias
  std::uniform_real_distribution<double> coin(0.0, 1.0);
  if (!q_goal_.empty() && coin(rng_) < goal_bias_) {
    return q_goal_;
  }

  // случайное состояние через RobotState (пусть он сам знает границы)
  moveit::core::RobotState state(scene_->getRobotModel());
  state.setToRandomPositions(jmg_);

  std::vector<double> q;
  state.copyJointGroupPositions(jmg_, q);
  return q;
}

int RRTStarInterface::nearest(const std::vector<double> & q_rand) const
{
  int best = 0;
  double best_dist = distance(tree_[0].q, q_rand);
  for (std::size_t i = 1; i < tree_.size(); ++i) {
    double d = distance(tree_[i].q, q_rand);
    if (d < best_dist) {
      best_dist = d;
      best = static_cast<int>(i);
    }
  }
  return best;
}

void RRTStarInterface::steer(
  const std::vector<double> & q_from,
  const std::vector<double> & q_to,
  std::vector<double> & q_new) const
{
  double d = distance(q_from, q_to);
  if (d <= step_size_) {
    q_new = q_to;
    return;
  }
  const std::size_t n = q_from.size();
  q_new.resize(n);
  double scale = step_size_ / d;
  for (std::size_t i = 0; i < n; ++i) {
    q_new[i] = q_from[i] + scale * (q_to[i] - q_from[i]);
  }
}

std::vector<int> RRTStarInterface::nearIndices(const std::vector<double> & q) const
{
  std::vector<int> idx;
  for (std::size_t i = 0; i < tree_.size(); ++i) {
    if (distance(tree_[i].q, q) < neighbor_radius_) {
      idx.push_back(static_cast<int>(i));
    }
  }
  return idx;
}

void RRTStarInterface::reconstructPath(
  int goal_index,
  std::vector<std::vector<double>> & path) const
{
  path.clear();
  int idx = goal_index;
  while (idx >= 0) {
    path.push_back(tree_[idx].q);
    idx = tree_[idx].parent_index;
  }
  std::reverse(path.begin(), path.end());
}

bool RRTStarInterface::solve(
  std::vector<std::vector<double>> & path,
  double max_planning_time,
  std::size_t max_iterations)
{
  if (q_start_.empty() || q_goal_.empty()) {
    return false;
  }

  if (!isStateValid(q_start_) || !isStateValid(q_goal_)) {
    return false;
  }

  tree_.clear();
  tree_.push_back(Node{q_start_, -1, 0.0});

  const auto start_time = std::chrono::steady_clock::now();

  for (std::size_t k = 0; k < max_iterations; ++k) {
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - start_time).count();
    if (elapsed > max_planning_time) {
      break;
    }

    const std::vector<double> q_rand = sampleRandom();
    const int idx_near = nearest(q_rand);
    std::vector<double> q_new;
    steer(tree_[idx_near].q, q_rand, q_new);

    if (!isStateValid(q_new)) {
      continue;
    }

    // выбор родителя (RRT*)
    const std::vector<int> near = nearIndices(q_new);
    int best_parent = idx_near;
    double best_cost = tree_[idx_near].cost + distance(tree_[idx_near].q, q_new);

    for (int i : near) {
      const double new_cost = tree_[i].cost + distance(tree_[i].q, q_new);
      if (new_cost < best_cost) {
        best_cost = new_cost;
        best_parent = i;
      }
    }

    Node new_node{q_new, best_parent, best_cost};
    tree_.push_back(new_node);
    const int new_idx = static_cast<int>(tree_.size() - 1);

    // rewiring
    for (int i : near) {
      const double alt_cost = new_node.cost + distance(new_node.q, tree_[i].q);
      if (alt_cost < tree_[i].cost) {
        tree_[i].parent_index = new_idx;
        tree_[i].cost = alt_cost;
      }
    }

    // попадание в окрестность цели
    if (distance(q_new, q_goal_) < step_size_) {
      reconstructPath(new_idx, path);
      return true;
    }
  }

  return false;
  
}

}  // namespace rozum_rrtstar_planner
