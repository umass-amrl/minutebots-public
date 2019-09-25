// Copyright 2018 kvedder@umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
// ========================================================================

#include "search/robocup_eastar/lazy_neighbor_generator.h"

#include <algorithm>
#include <vector>

namespace search {
namespace eastar {
namespace lazy_neighbors {

Heuristic Heuristic2d(const GridVertex& p, const GridVertex& goal) {
  return GridVertex(p - goal).lpNorm<1>();
}

template <size_t kMaxRobotCount>
Heuristic HeuristicJoint(const JointPosition<kMaxRobotCount>& p,
                         const JointPosition<kMaxRobotCount>& goal,
                         const float inflation) {
  NP_CHECK_MSG(p.size() == goal.size(), "p: " << p << " goal: " << goal);
  NP_CHECK(!p.empty());
  NP_CHECK(!goal.empty());
  Heuristic h = 0;
  for (size_t i = 0; i < p.size(); ++i) {
    const GridVertex& pv = p.at(i);
    const GridVertex& goalv = goal.at(i);
    h += Heuristic2d(pv, goalv);
  }
  return static_cast<Heuristic>(static_cast<float>(h) * inflation);
}

// Returns all neighbors for a given grid vertex that are not in collision with
// the world.
template <size_t kMaxRobotCount>
Neighbors2d NeighborGenerator<kMaxRobotCount>::GetNeighbors2d(
    const GridVertex& p, const GridVertex& goal, const MovingHaltedSteps& dist,
    const OurRobotIndex& robot_index) {
  const GridPath& individual_path = slice_->individual_plans.at(robot_index);
  const navigation::production::eight_grid::CollisionGrid& collision_grid =
      slice_->collision_grid;
  Neighbors2d neighbors_2d;

  if (!search_window_->box.Inside(p)) {
    NP_CHECK(dist.transit_steps + 1 < static_cast<int>(individual_path.size()));
    neighbors_2d.push_back(
        {individual_path.at(dist.transit_steps + 1), {1, 0}, true});
    return neighbors_2d;
  }

  const auto obstacle_flag =
      obstacle::ObstacleFlag::GetAllRobotsExceptTeam(robot_index);
  std::array<Neighbor2d, kNumNeighbors - 1> possible_neighbors = {{
      {{p.x() + 1, p.y()}, {1, 0}, true},
      {{p.x() - 1, p.y()}, {1, 0}, true},
      {{p.x(), p.y() + 1}, {1, 0}, true},
      {{p.x(), p.y() - 1}, {1, 0}, true},
      {{p.x(), p.y()}, {1, 0}, true},
  }};
  for (Neighbor2d& possible_neighbor : possible_neighbors) {
    if (collision_grid.IsColliding(possible_neighbor.neighbor, obstacle_flag)) {
      // Skip all vertices that are colliding with an obstacle.
      continue;
    }
    possible_neighbor.in_window =
        search_window_->box.Inside(possible_neighbor.neighbor);
    neighbors_2d.push_back(possible_neighbor);
  }

  if (p == goal) {
    neighbors_2d.push_back({p, {0, 1}, true});
  }

  return neighbors_2d;
}

// Computes the individual neighbors for each of the agents.
template <size_t kMaxRobotCount>
void NeighborGenerator<kMaxRobotCount>::ComputeNeighborsArray(
    const SearchNode<kMaxRobotCount>& top) {
  // Constructs individual_neighbors and default_position.
  for (size_t i = 0; i < goal_position_.size(); ++i) {
    const GridVertex& individual_position =
        parent_search_node_.current_position.at(i);
    const GridVertex& individual_goal = goal_position_.at(i);
    const MovingHaltedSteps& individual_current_steps = top.joint_steps.at(i);
    const CollidingRobot& cr = search_window_->relevant_paths.at(i);
    neighbors_array_.push_back(
        GetNeighbors2d(individual_position, individual_goal,
                       individual_current_steps, cr.path_index));
  }
}

Neighbor2d const* MinNeighbor2d(Neighbor2d const* existing,
                                Neighbor2d const* candidate,
                                const GridVertex& goal, const float inflation) {
  const float existing_f = existing->steps.transit_steps +
                           Heuristic2d(existing->neighbor, goal) * inflation;
  const float candidate_f = candidate->steps.transit_steps +
                            Heuristic2d(candidate->neighbor, goal) * inflation;
  if (candidate_f < existing_f) {
    return candidate;
  }
  return existing;
}

// Computes the smallest search node which will be used to compare this
// generator to other generators.
template <size_t kMaxRobotCount>
void NeighborGenerator<kMaxRobotCount>::ComputeSmallestCostNeighbor() {
  JointPosition<kMaxRobotCount> smallest_cost_position;
  JointSteps<kMaxRobotCount> smallest_cost_position_additional_steps;
  for (size_t n = 0; n < neighbors_array_.size(); ++n) {
    const Neighbors2d& neighbors_2d = neighbors_array_.at(n);
    const GridVertex& goal_2d = goal_position_.at(n);
    NP_CHECK(!neighbors_2d.empty());
    Neighbor2d const* smallest_cost_neighbor = &(neighbors_2d.at(0));
    for (size_t i = 1; i < neighbors_2d.size(); ++i) {
      Neighbor2d const* candidate_neighbor = &(neighbors_2d.at(i));
      smallest_cost_neighbor =
          MinNeighbor2d(smallest_cost_neighbor, candidate_neighbor, goal_2d,
                        heuristic_inflation_);
    }
    smallest_cost_position.push_back(smallest_cost_neighbor->neighbor);
    smallest_cost_position_additional_steps.push_back(
        smallest_cost_neighbor->steps);
  }

  NP_CHECK_EQ(smallest_cost_position.size(), goal_position_.size());

  smallest_cost_neighbor_ = SearchNode<kMaxRobotCount>(
      parent_search_node_, smallest_cost_position,
      smallest_cost_position_additional_steps,
      HeuristicJoint(smallest_cost_position, goal_position_,
                     heuristic_inflation_));
}

template <size_t kMaxRobotCount>
SearchNode<kMaxRobotCount> NeighborGenerator<kMaxRobotCount>::GetMinimum()
    const {
  return smallest_cost_neighbor_;
}

template <class T>
void DuplicateExistingContentsNTimes(std::vector<T>* data_array,
                                     const size_t n) {
  if (n <= 1) {
    return;
  }

  std::vector<T>& deref_data_array = *data_array;
  const size_t orig_data_size = deref_data_array.size();

  deref_data_array.resize(orig_data_size * n);
  auto orig_data_begin = deref_data_array.begin();
  auto orig_data_end = deref_data_array.begin() + orig_data_size;
  auto new_data_start = deref_data_array.begin() + orig_data_size;
  for (size_t i = 1; i < n; ++i) {
    std::copy(orig_data_begin, orig_data_end, new_data_start);
    new_data_start += orig_data_size;
  }
}

template <size_t kMaxRobotCount>
void NeighborGenerator<kMaxRobotCount>::GenerateFullNeighbors() {
  NP_CHECK(!has_generated_neighbors_);
  NP_CHECK(lazy_storage_.empty());
  NP_CHECK(!neighbors_array_.empty());
  std::vector<SearchNode<kMaxRobotCount>>& joint_neighbors =
      *lazy_storage_.GetMutableVector();

  // Seed the neighbor vector with a starting node.
  SearchNode<kMaxRobotCount> starter_node = parent_search_node_;
  starter_node.previous_position = starter_node.current_position;
  joint_neighbors.push_back(starter_node);

  for (size_t neighbor_index = 0; neighbor_index < neighbors_array_.size();
       ++neighbor_index) {
    const Neighbors2d& neighbors = neighbors_array_.at(neighbor_index);
    const size_t old_count = joint_neighbors.size();
    DuplicateExistingContentsNTimes(&joint_neighbors, neighbors.size());

    // Iterate over the blocks and assign one of the eight values to
    // each block.
    for (size_t block_index = 0; block_index < neighbors.size();
         ++block_index) {
      for (size_t intrablock_index = 0; intrablock_index < old_count;
           ++intrablock_index) {
        const Neighbor2d& neighbor2d = neighbors.at(block_index);
        NP_CHECK(block_index * old_count + intrablock_index <
                 joint_neighbors.size());
        SearchNode<kMaxRobotCount>& copied_value =
            joint_neighbors[block_index * old_count + intrablock_index];
        copied_value.current_position.Set(neighbor_index, neighbor2d.neighbor);
        *(copied_value.joint_steps.GetMutable(neighbor_index)) +=
            neighbor2d.steps;
        copied_value.transit_distance += neighbor2d.steps.transit_steps;
        copied_value.halted_distance += neighbor2d.steps.at_goal_steps;
        copied_value.heuristic =
            HeuristicJoint(copied_value.current_position, goal_position_,
                           heuristic_inflation_);
      }
    }
  }

  // Check to make sure that the full neighbor count is the correct cardinality.
  if (!kProduction) {
    size_t product_total = 1;
    for (const Neighbors2d& ns : neighbors_array_) {
      product_total *= ns.size();
    }
    NP_CHECK_MSG(
        product_total >= lazy_storage_.size(),
        "product_total: " << product_total << " >= "
                          << "lazy_storage_.size(): " << lazy_storage_.size());
  }

  lazy_storage_.RebuildHeap();
}

template <size_t kMaxRobotCount>
bool NeighborGenerator<kMaxRobotCount>::GenerateNext() {
  if (!has_generated_neighbors_) {
    GenerateFullNeighbors();
  }
  has_generated_neighbors_ = true;

  if (lazy_storage_.empty()) {
    return false;
  }

  if (needs_lazy_storage_rebuild_) {
    for (SearchNode<kMaxRobotCount>& n : *lazy_storage_.GetMutableVector()) {
      n.UpdateAtGoalSteps(goal_position_);
      n.heuristic = HeuristicJoint(n.current_position, goal_position_,
                                   heuristic_inflation_);
    }

    lazy_storage_.RebuildHeap();
    needs_lazy_storage_rebuild_ = false;
  }

  smallest_cost_neighbor_ = lazy_storage_.top();
  lazy_storage_.pop();

  return true;
}

template <size_t kMaxRobotCount>
void NeighborGenerator<kMaxRobotCount>::UpdateGoal(
    const JointPosition<kMaxRobotCount>& goal, const float inflation) {
  NP_CHECK_EQ(goal.size(), goal_position_.size());
  goal_position_ = goal;
  heuristic_inflation_ = inflation;

  parent_search_node_.UpdateAtGoalSteps(goal);
  parent_search_node_.heuristic = HeuristicJoint(
      parent_search_node_.current_position, goal, heuristic_inflation_);

  if (!neighbors_array_.empty()) {
    ComputeSmallestCostNeighbor();
  } else {
    smallest_cost_neighbor_.UpdateAtGoalSteps(goal);
    smallest_cost_neighbor_.heuristic = HeuristicJoint(
        smallest_cost_neighbor_.current_position, goal, heuristic_inflation_);
  }

  needs_lazy_storage_rebuild_ = true;
}

template void
NeighborGenerator<kRoboCupEAStarMaxRobots>::ComputeSmallestCostNeighbor();

template void NeighborGenerator<kRoboCupEAStarMaxRobots>::ComputeNeighborsArray(
    const SearchNode<kRoboCupEAStarMaxRobots>& top);

template SearchNode<kRoboCupEAStarMaxRobots>
NeighborGenerator<kRoboCupEAStarMaxRobots>::GetMinimum() const;

template void
NeighborGenerator<kRoboCupEAStarMaxRobots>::GenerateFullNeighbors();

template bool NeighborGenerator<kRoboCupEAStarMaxRobots>::GenerateNext();

template void NeighborGenerator<kRoboCupEAStarMaxRobots>::UpdateGoal(
    const JointPosition<kRoboCupEAStarMaxRobots>& goal, const float inflation);

// Test definitions

template void NeighborGenerator<1>::ComputeSmallestCostNeighbor();
template void NeighborGenerator<1>::ComputeNeighborsArray(
    const SearchNode<1>& top);
template SearchNode<1> NeighborGenerator<1>::GetMinimum() const;
template void NeighborGenerator<1>::GenerateFullNeighbors();
template bool NeighborGenerator<1>::GenerateNext();

template void NeighborGenerator<2>::ComputeSmallestCostNeighbor();
template void NeighborGenerator<2>::ComputeNeighborsArray(
    const SearchNode<2>& top);
template SearchNode<2> NeighborGenerator<2>::GetMinimum() const;
template void NeighborGenerator<2>::GenerateFullNeighbors();
template bool NeighborGenerator<2>::GenerateNext();
}  // namespace lazy_neighbors
}  // namespace eastar
}  // namespace search
