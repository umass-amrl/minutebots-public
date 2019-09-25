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

#ifndef SRC_SEARCH_ROBOCUP_EASTAR_SEARCH_NODE_H_
#define SRC_SEARCH_ROBOCUP_EASTAR_SEARCH_NODE_H_

#include "constants/constants.h"
#include "datastructures/transacted_multi_vector_priority_queue.h"
#include "datastructures/transacted_vector_priority_queue.h"
#include "datastructures/vector_priority_queue.h"
#include "search/robocup_eastar/multiagent_data.h"
#include "search/robocup_eastar/path_collision_detector.h"
#include "search/robocup_eastar/search_window.h"

namespace search {
namespace eastar {
template <size_t kMaxRobotCount>
struct SearchNode {
  JointPosition<kMaxRobotCount> current_position;
  JointPosition<kMaxRobotCount> previous_position;
  JointSteps<kMaxRobotCount> joint_steps;
  GridDistance transit_distance;
  GridDistance halted_distance;
  Heuristic heuristic;

  // Default.
  SearchNode()
      : current_position(),
        previous_position(),
        joint_steps(),
        transit_distance(0),
        halted_distance(0),
        heuristic(0) {}

  // Member initializer.
  SearchNode(const JointPosition<kMaxRobotCount>& current_position,
             const JointPosition<kMaxRobotCount>& previous_position,
             const JointSteps<kMaxRobotCount>& joint_steps,
             const Heuristic& heuristic)
      : current_position(current_position),
        previous_position(previous_position),
        joint_steps(joint_steps),
        transit_distance(0),
        halted_distance(0),
        heuristic(heuristic) {
    for (const MovingHaltedSteps& s : joint_steps) {
      transit_distance += s.transit_steps;
      halted_distance += s.at_goal_steps;
    }
    NP_CHECK_EQ(current_position.size(), previous_position.size());
    NP_CHECK_EQ(current_position.size(), joint_steps.size());
  }

  // Neighbor of existing node.
  SearchNode(const SearchNode& prev,
             const JointPosition<kMaxRobotCount>& current_position,
             const JointSteps<kMaxRobotCount>& additional_steps,
             const Heuristic& current_position_heuristic)
      : current_position(current_position),
        previous_position(prev.current_position),
        joint_steps(prev.joint_steps),
        transit_distance(prev.transit_distance),
        halted_distance(prev.halted_distance),
        heuristic(current_position_heuristic) {
    for (size_t i = 0; i < additional_steps.size(); ++i) {
      const MovingHaltedSteps& s = additional_steps.at(i);
      NP_CHECK_MSG((s.transit_steps + s.at_goal_steps) <= 1,
                   "Additional steps should be at most one step.");
      transit_distance += s.transit_steps;
      halted_distance += s.at_goal_steps;
      (*joint_steps.GetMutable(i)) += s;
    }
  }

  inline float GetFValue() const { return (transit_distance + heuristic); }

  bool operator<(const SearchNode& other) const {
    if (GetFValue() == other.GetFValue()) {
      if (heuristic == other.heuristic) {
        return halted_distance > other.halted_distance;
      }
      return (heuristic > other.heuristic);
    }
    return (GetFValue() > other.GetFValue());
  }

  bool operator==(const SearchNode& other) const {
    return (current_position == other.current_position) &&
           (previous_position == other.previous_position) &&
           (transit_distance == other.transit_distance) &&
           (halted_distance == other.halted_distance) &&
           (heuristic == other.heuristic);
  }

  void UpdateAtGoalSteps(const JointPosition<kMaxRobotCount>& new_goal) {
    NP_CHECK_EQ_MSG(new_goal.size(), joint_steps.size(),
                    new_goal << " vs " << joint_steps);
    for (size_t i = 0; i < joint_steps.size(); ++i) {
      const GridVertex& g = new_goal.at(i);
      const GridVertex& c = current_position.at(i);
      if (g == c) {
        // If already at goal for that position, then you should not remove the
        // at goal amount.
        continue;
      }

      MovingHaltedSteps* s = joint_steps.GetMutable(i);
      transit_distance += s->at_goal_steps;
      halted_distance -= s->at_goal_steps;
      s->at_goal_steps = 0;
    }
  }
};

template <size_t kMaxRobotCount>
std::ostream& operator<<(std::ostream& os,
                         const SearchNode<kMaxRobotCount>& node) {
  size_t transit_distance_sum = 0;
  size_t halted_distance_sum = 0;
  for (const MovingHaltedSteps& s : node.joint_steps) {
    transit_distance_sum += s.transit_steps;
    halted_distance_sum += s.at_goal_steps;
  }

  NP_CHECK_EQ(transit_distance_sum, node.transit_distance);
  NP_CHECK_EQ(halted_distance_sum, node.halted_distance);

  os << "SearchNode: " << node.previous_position << " => "
     << node.current_position << " \t Steps: " << node.joint_steps;
  os << "(distance sum: " << node.transit_distance << " + "
     << node.halted_distance << ") heuristic: " << node.heuristic;
  return os;
}

}  // namespace eastar
}  // namespace search

#endif  // SRC_SEARCH_ROBOCUP_EASTAR_SEARCH_NODE_H_
