// Copyright 2017 - 2018 kvedder@umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
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

#ifndef SRC_NAVIGATION_REPAIR_ASTAR_EIGHT_GRID_REPAIRER_H_
#define SRC_NAVIGATION_REPAIR_ASTAR_EIGHT_GRID_REPAIRER_H_

#include <algorithm>
#include <array>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "navigation/repair/eight_grid_repairer.h"

namespace navigation {
namespace repair {
namespace repairer {

template <unsigned int kRobotCount>
class AStarEightGridRepairer : public EightGridRepairer<kRobotCount> {
  using Super = EightGridRepairer<kRobotCount>;

  // Holds the implicitly 3D position of each robot (X, Y, time), where time is
  // a function of distance.
  //
  // Also holds distance and heuristic values for the priority queue.
  struct JointPriorityQueueVertex {
    std::array<bool, kRobotCount> needs_replans;
    std::array<Eigen::Vector2i, kRobotCount> positions;
    std::array<Distance, kRobotCount> distances;
    Distance heuristic;
    std::array<Eigen::Vector2i, kRobotCount> predecessors;
    std::array<size_t, kRobotCount> path_indices;

    inline std::array<Distance, kRobotCount> DistancesToTimes(
        const std::array<Distance, kRobotCount>& distance) {
      std::array<float, kRobotCount> times;
      for (size_t i = 0; i < kRobotCount; ++i) {
        times[i] = distance[i] / kMaxRobotVelocity;
      }
      return times;
    }

    JointPriorityQueueVertex() = delete;
    JointPriorityQueueVertex(
        const std::array<bool, kRobotCount>& needs_replans,
        const std::array<Eigen::Vector2i, kRobotCount>& positions,
        const std::array<Distance, kRobotCount>& distances,
        const Distance heuristic,
        const std::array<Eigen::Vector2i, kRobotCount>& predecessors,
        const std::array<size_t, kRobotCount>& path_indices)
        : needs_replans(needs_replans),
          positions(positions),
          distances(distances),
          heuristic(heuristic),
          predecessors(predecessors),
          path_indices(path_indices) {}

    bool operator<(const JointPriorityQueueVertex& other) const {
      const auto this_distance_sum = array_util::SelectiveSumArray(
          distances, needs_replans, Distance(0, 0));
      const auto this_val = (this_distance_sum + heuristic);
      const auto other_distance_sum = array_util::SelectiveSumArray(
          other.distances, other.needs_replans, Distance(0, 0));
      const auto other_val = (other_distance_sum + other.heuristic);
      // Larger distance is better if the f-values are the same.
      if (this_val == other_val) {
        return this_distance_sum < other_distance_sum;
      }
      return (this_val > other_val);
    }

    bool operator==(const JointPriorityQueueVertex& other) const {
      return (distances == other.distances && heuristic == other.heuristic &&
              positions == other.positions &&
              predecessors == other.predecessors);
    }

    float EvaluateCost() const {
      return (array_util::SelectiveSumArray(distances, needs_replans,
                                            Distance(0, 0)) +
              heuristic);
    }

    void UpdateDistances(
        const std::array<Distance, kRobotCount>& new_distances) {
      distances = new_distances;
    }
  };

  // Array of all of the neighbors for each position. It's assumed that if
  // the path is predetermined or already at the goal, then the neighbors
  // will consist of an array only the next appropriate position, meaning
  // that the selection of which element to take can be arbitrary.
  std::array<std::array<std::pair<Eigen::Vector2i, Distance>, 9>, kRobotCount>
  CalculateNeighbors(const ReplanScenerio<kRobotCount>& scenerio,
                     const JointPriorityQueueVertex& current_jpqv) {
    // Initialize with default data for the sake of debugging.
    std::array<std::array<std::pair<Eigen::Vector2i, Distance>, 9>, kRobotCount>
        position_neighbors = array_util::MakeArray<kRobotCount>(
            array_util::MakeArray<9>(std::make_pair(
                Eigen::Vector2i(kIntMaxHack - 3, kIntMaxHack - 3),
                Distance())));

    for (size_t robot_index = 0; robot_index < kRobotCount; ++robot_index) {
      const Eigen::Vector2i& position = current_jpqv.positions[robot_index];
      if (!scenerio.needs_replans[robot_index]) {
        position_neighbors[robot_index] = Super::kIgnorableNeighborsNine;
      } else if (position == scenerio.replan_ends[robot_index]) {
        position_neighbors[robot_index] =
            Super::GetIdentityNeighborsNine(position);
      } else {
        position_neighbors[robot_index] = Super::GetNeighborsNine(position);
      }
    }
    return position_neighbors;
  }

  ReplanResult<kRobotCount> AStar(
      const size_t& iteration, const ReplanScenerio<kRobotCount>& scenerio,
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& static_invalid_vertices,
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& dynamic_invalid_vertices) {
    Super::DumpReplanScenerio(iteration, scenerio, static_invalid_vertices,
                              dynamic_invalid_vertices);

    size_t open_list_expansion_count = 0;
    size_t collision_check_count = 0;

    const std::array<Eigen::Vector2i, kRobotCount> stripped_replan_starts =
        Super::StripIgnoredData(scenerio.replan_starts, scenerio.needs_replans);
    const std::array<Eigen::Vector2i, kRobotCount> stripped_replan_ends =
        Super::StripIgnoredData(scenerio.replan_ends, scenerio.needs_replans);

    std::priority_queue<JointPriorityQueueVertex,
                        std::vector<JointPriorityQueueVertex>>
        priority_queue;

    // Push on the initial element of the queue, which is the element from the
    // given start to the given goal.
    priority_queue.push(
        {scenerio.needs_replans, stripped_replan_starts,
         array_util::MakeArray<kRobotCount>(Distance(0, 0)),
         Super::JointHeuristic(scenerio.needs_replans, stripped_replan_starts,
                               stripped_replan_ends),
         Super::kStartNodePredecessors, scenerio.replan_starts_indices});

    std::unordered_set<std::array<Eigen::Vector2i, kRobotCount>,
                       ArrayGridHasher<kRobotCount>>
        closed_list(Super::planar_total_vertex_count_);
    std::unordered_map<std::array<Eigen::Vector2i, kRobotCount>,
                       std::array<Eigen::Vector2i, kRobotCount>,
                       ArrayGridHasher<kRobotCount>>
        path_map;
    while (!priority_queue.empty()) {
      ++open_list_expansion_count;
      const JointPriorityQueueVertex top_queue_joint_vertex =
          priority_queue.top();
      priority_queue.pop();

      // If insertion into the closed list fails, it's due to the fact that
      // there already exists an item of that value in the closed list.
      if (!closed_list.insert(top_queue_joint_vertex.positions).second) {
        continue;
      }

      path_map.insert({top_queue_joint_vertex.positions,
                       top_queue_joint_vertex.predecessors});

      if (!kProduction) {
        std::vector<std::array<Eigen::Vector2i, kRobotCount>> closed_positions;
        for (const auto& e : closed_list) {
          closed_positions.push_back(e);
        }
        Super::DumpSearchTree(
            iteration, open_list_expansion_count, path_map, closed_positions,
            scenerio, top_queue_joint_vertex.predecessors,
            top_queue_joint_vertex.positions, top_queue_joint_vertex.heuristic,
            top_queue_joint_vertex.distances, Super::kStartNodePredecessors,
            "Astar");
      }

      // Check to see if we're at the goal position.
      if (stripped_replan_ends == top_queue_joint_vertex.positions) {
        ReplanResult<kRobotCount> result;
        std::array<Eigen::Vector2i, kRobotCount> current_key =
            top_queue_joint_vertex.positions;
        while (stripped_replan_starts != current_key) {
          for (size_t i = 0; i < kRobotCount; ++i) {
            // Skip robots that do not need to be replanned.
            if (!scenerio.needs_replans[i]) {
              continue;
            }
            const Eigen::Vector2i& position = current_key[i];
            // Checks to make sure that this will not push back a duplicate
            // element in the case where one path is shorter than the other
            // and
            // thus there is no progress made on one path but there is the
            // other.
            auto& current_plan = result.fixed_plans[i];
            if (current_plan.empty() ||
                current_plan[current_plan.size() - 1] != position) {
              current_plan.push_back(position);
            }
          }
          const auto current_find = path_map.find(current_key);
          if (!kProduction) {
            if (current_find == path_map.end()) {
              LOG(FATAL) << "Could not find current key in map";
            }
          }
          current_key = current_find->second;
        }

        for (size_t i = 0; i < kRobotCount; ++i) {
          // Skip robots that do not need to be replanned.
          if (!scenerio.needs_replans[i]) {
            continue;
          }
          const Eigen::Vector2i& position = scenerio.replan_starts[i];
          auto& current_plan = result.fixed_plans[i];
          if (current_plan.empty() ||
              current_plan[current_plan.size() - 1] != position) {
            current_plan.push_back(position);
          }
        }

        // Reverse the plans to all be in the forwards direction.
        for (auto& plan : result.fixed_plans) {
          std::reverse(plan.begin(), plan.end());
        }

        //             LOG(INFO) << "Open list expansion count: " <<
        //             open_list_expansion_count;
        //     LOG(INFO) << "Collision checks: " << collision_check_count;

        return result;
      }

      const auto dynamic_cartesian_product =
          Super::CalculateDynamicCartesianProductNine(
              CalculateNeighbors(scenerio, top_queue_joint_vertex),
              scenerio.needs_replans);

      for (const std::array<std::pair<Eigen::Vector2i, Distance>, kRobotCount>&
               neighbors : dynamic_cartesian_product) {
        bool reject_sample = false;
        std::array<Eigen::Vector2i, kRobotCount> next_positions =
            array_util::MakeArray<kRobotCount>(
                Eigen::Vector2i(kIntMaxHack, kIntMaxHack));
        std::array<Distance, kRobotCount> next_distances =
            Super::StripIgnoredData(top_queue_joint_vertex.distances,
                                    scenerio.needs_replans);
        for (size_t robot_index = 0; robot_index < kRobotCount; ++robot_index) {
          if (scenerio.needs_replans[robot_index]) {
            if (Super::OutsideOfPlanningBox(neighbors[robot_index].first,
                                            scenerio.grid_center,
                                            scenerio.replan_radius)) {
              reject_sample = true;
              break;
            }

            if (Super::OutsideOfField(neighbors[robot_index].first)) {
              reject_sample = true;
              break;
            }
          }

          next_positions[robot_index] = neighbors[robot_index].first;
          next_distances[robot_index] += neighbors[robot_index].second;
        }

        if (reject_sample) {
          continue;
        }

        if (Super::ContainsInvalidVertices(
                static_invalid_vertices, dynamic_invalid_vertices,
                next_positions, scenerio.needs_replans)) {
          continue;
        }

        JointPriorityQueueVertex next_jpqv(
            scenerio.needs_replans, next_positions, next_distances,
            Super::JointHeuristic(scenerio.needs_replans, next_positions,
                                  stripped_replan_ends),
            top_queue_joint_vertex.positions,
            array_util::AddToEachElement(top_queue_joint_vertex.path_indices,
                                         1ul));
        ++collision_check_count;
        if (!Super::CheckStepCollides(top_queue_joint_vertex.positions,
                                      top_queue_joint_vertex.distances,
                                      next_jpqv.positions, next_jpqv.distances,
                                      scenerio.needs_replans,
                                      scenerio.replan_start_times, path_map)) {
          priority_queue.push(next_jpqv);
        } else {
        }
      }
    }
    if (!kProduction) {
      LOG(ERROR) << "No replan found!";
    }
    LOG(INFO) << "Open list expansion count: " << open_list_expansion_count;
    LOG(INFO) << "Collision checks: " << collision_check_count;
    return ReplanResult<kRobotCount>();
  }

 public:
  AStarEightGridRepairer(const float& grid_distance_between_vertices,
                         const size_t& initial_repair_radius,
                         const size_t& repair_radius_step_size)
      : EightGridRepairer<kRobotCount>(grid_distance_between_vertices,
                                       initial_repair_radius,
                                       repair_radius_step_size) {}

  std::pair<ReplanResult<kRobotCount>, ReplanScenerio<kRobotCount>>
  PerformReplan(
      const std::vector<ReplanScenerio<kRobotCount>>& scenerio_list,
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& static_invalid_vertices,
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& dynamic_invalid_vertices) override {
    if (!kProduction) {
      if (scenerio_list.empty()) {
        LOG(FATAL) << "No scenerios in the given list!";
      }
    }

    ReplanResult<kRobotCount> final_result;
    ReplanScenerio<kRobotCount> last_finished_scenerio = scenerio_list[0];
    for (size_t i = 0; i < scenerio_list.size(); ++i) {
      const ReplanScenerio<kRobotCount>& scenerio = scenerio_list[i];
      final_result =
          AStar(i, scenerio, static_invalid_vertices, dynamic_invalid_vertices);
      last_finished_scenerio = scenerio;
    }
    return {final_result, last_finished_scenerio};
  }
};

}  // namespace repairer
}  // namespace repair
}  // namespace navigation

#endif  // SRC_NAVIGATION_REPAIR_ASTAR_EIGHT_GRID_REPAIRER_H_
