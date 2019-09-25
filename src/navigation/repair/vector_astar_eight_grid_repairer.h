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

#ifndef SRC_NAVIGATION_REPAIR_VECTOR_ASTAR_EIGHT_GRID_REPAIRER_H_
#define SRC_NAVIGATION_REPAIR_VECTOR_ASTAR_EIGHT_GRID_REPAIRER_H_

#include <algorithm>
#include <array>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "navigation/repair/eight_grid_repairer.h"
#include "util/vector_util.h"

namespace navigation {
namespace repair {
namespace repairer {

template <unsigned int kRobotCount>
class VectorAStarEightGridRepairer : public EightGridRepairer<kRobotCount> {
  using Super = EightGridRepairer<kRobotCount>;
  std::vector<Eigen::Vector2i> start_node_predecessors;

  // Holds the implicitly 3D position of each robot (X, Y, time), where time is
  // a function of distance.
  //
  // Also holds distance and heuristic values for the priority queue.
  struct JointPriorityQueueVertex {
    std::vector<Eigen::Vector2i> positions;
    std::vector<float> distances;
    float heuristic;
    std::vector<float> times;
    std::vector<Eigen::Vector2i> predecessors;
    std::vector<size_t> path_indices;

    inline std::vector<float> DistancesToTimes(
        const std::vector<float>& distance) {
      return vector_util::MultiplyToEachElement(distance,
                                                1.0f / kMaxRobotVelocity);
    }
    JointPriorityQueueVertex() = delete;
    JointPriorityQueueVertex(const std::vector<Eigen::Vector2i>& positions,
                             const std::vector<float>& distances,
                             const float heuristic,
                             const std::vector<Eigen::Vector2i>& predecessors,
                             const std::vector<size_t>& path_indices)
        : positions(positions),
          distances(distances),
          heuristic(heuristic),
          times(DistancesToTimes(distances)),
          predecessors(predecessors),
          path_indices(path_indices) {}

    JointPriorityQueueVertex(std::vector<Eigen::Vector2i>* positions,
                             std::vector<float>* distances,
                             const float heuristic,
                             const std::vector<Eigen::Vector2i>& predecessors,
                             std::vector<size_t>* path_indices)
        : positions(0),
          distances(0),
          heuristic(heuristic),
          times(DistancesToTimes(*distances)),
          predecessors(predecessors),
          path_indices(0) {
      this->positions.swap(*positions);
      this->distances.swap(*distances);
      this->path_indices.swap(*path_indices);
    }

    bool operator<(const JointPriorityQueueVertex& other) const {
      return ((vector_util::SumVector(distances) + heuristic +
               vector_util::SumVector(times)) >
              (vector_util::SumVector(other.distances) + other.heuristic +
               vector_util::SumVector(other.times)));
    }

    bool operator==(const JointPriorityQueueVertex& other) const {
      return (distances == other.distances && heuristic == other.heuristic &&
              positions == other.positions && times == other.times &&
              predecessors == other.predecessors);
    }

    float EvaluateCost() const {
      return (vector_util::SumVector(distances) + heuristic +
              vector_util::SumVector(times));
    }
  };

  struct LimitedReplanScenerio {
    std::vector<size_t> replanned_paths;
    std::vector<Eigen::Vector2i> replan_starts;
    std::vector<float> replan_start_times;
    std::vector<size_t> replan_starts_indices;
    std::vector<Eigen::Vector2i> replan_ends;
    std::vector<size_t> replan_ends_indices;
    Eigen::Vector2i grid_center;

    LimitedReplanScenerio() = delete;
    LimitedReplanScenerio(const std::vector<size_t>& replanned_paths,
                          const std::vector<Eigen::Vector2i>& replan_starts,
                          const std::vector<float>& replan_start_times,
                          const std::vector<size_t>& replan_starts_indices,
                          const std::vector<Eigen::Vector2i>& replan_ends,
                          const std::vector<size_t>& replan_ends_indices,
                          const Eigen::Vector2i& grid_center)
        : replanned_paths(replanned_paths),
          replan_starts(replan_starts),
          replan_start_times(replan_start_times),
          replan_starts_indices(replan_starts_indices),
          replan_ends(replan_ends),
          replan_ends_indices(replan_ends_indices),
          grid_center(grid_center) {}
  };

  // Array of all of the neighbors for each position. It's assumed that if
  // the path is predetermined or already at the goal, then the neighbors
  // will consist of an array only the next appropriate position, meaning
  // that the selection of which element to take can be arbitrary.
  std::vector<std::array<std::pair<Eigen::Vector2i, float>, 8>>
  CalculateNeighbors(const LimitedReplanScenerio& scenerio,
                     const JointPriorityQueueVertex& current_jpqv) {
    // Initialize with default data for the sake of debugging.
    std::vector<std::array<std::pair<Eigen::Vector2i, float>, 8>>
        position_neighbors;

    for (size_t i = 0; i < scenerio.replanned_paths.size(); ++i) {
      const Eigen::Vector2i& position = current_jpqv.positions[i];
      if (position == scenerio.replan_ends[i]) {
        position_neighbors.push_back(
            Super::GetIdentityNeighbors(position, kSqrtTwo));
      } else {
        position_neighbors.push_back(Super::GetNeighbors(position));
      }
    }
    return position_neighbors;
  }

  size_t CalculateReplanRadius(const ReplanScenerio<kRobotCount>& scenerio,
                               const size_t& iteration_count) {
    int minimum_radius = 0;
    //     LOG(INFO) << "Center: " << scenerio.grid_center.x() << ", "
    //               << scenerio.grid_center.y() << "\n~~~";
    for (size_t i = 0; i < kRobotCount; ++i) {
      if (scenerio.needs_replans[i]) {
        const Eigen::Vector2i& start = scenerio.replan_starts[i];
        const Eigen::Vector2i& end = scenerio.replan_ends[i];
        const Eigen::Vector2i start_delta = (scenerio.grid_center - start);
        const Eigen::Vector2i end_delta = (scenerio.grid_center - end);
        minimum_radius =
            std::max({minimum_radius, start_delta.lpNorm<Eigen::Infinity>(),
                      end_delta.lpNorm<Eigen::Infinity>()});
        //         LOG(INFO) << "==Path " << i;
        //         LOG(INFO) << "Start: " << start.x() << ", " << start.y();
        //         LOG(INFO) << "End: " << end.x() << ", " << end.y();
      } else {
        //         LOG(INFO) << "==Path " << i << " no replan needed!";
      }
    }

    return std::max(static_cast<size_t>(minimum_radius),
                    iteration_count * Super::repair_radius_step_size_ +
                        Super::initial_repair_radius_);
  }

  std::vector<float> CalculatePreviousTimes(
      const std::vector<Eigen::Vector2i>& current_positions,
      const std::vector<Eigen::Vector2i>& previous_positions,
      const std::vector<float>& current_times) {
    std::vector<float> previous_times(current_times.size());
    for (size_t i = 0; i < current_times.size(); ++i) {
      const Eigen::Vector2i delta =
          (current_positions[i] - previous_positions[i]);
      float distance;
      switch (delta.lpNorm<1>()) {
        case 0:
          distance = 0;
          break;
        case 1:
          distance = Super::grid_distance_between_vertices_;
          break;
        case 2:  // Fallthrough intentional.
        default:
          distance = Super::grid_distance_between_vertices_ * kSqrtTwo;
      }
      previous_times[i] = current_times[i] - (distance / kMaxRobotVelocity);
    }
    return previous_times;
  }

  bool CheckStepCollides(const JointPriorityQueueVertex& current,
                         const JointPriorityQueueVertex& next,
                         const std::vector<float>& start_times,
                         const std::unordered_map<std::vector<Eigen::Vector2i>,
                                                  std::vector<Eigen::Vector2i>,
                                                  VectorGridHasher>& path_map) {
    const auto current_jpqv_times =
        vector_util::AddVectorElements(current.times, start_times);
    const auto next_jpqv_times =
        vector_util::AddVectorElements(next.times, start_times);

    if (!kProduction) {
      if (current_jpqv_times.size() != next_jpqv_times.size()) {
        LOG(FATAL) << "Different sizes!";
      }

      for (size_t i = 0; i < current_jpqv_times.size(); ++i) {
        if (current_jpqv_times[i] >= next_jpqv_times[i]) {
          LOG(FATAL) << "current_jpqv_times[i] >= next_jpqv_times[i] ("
                     << current_jpqv_times[i] << " >= " << next_jpqv_times[i]
                     << ")";
        }
      }
    }

    if (collision_checks::SegmentsCollideWithSegments(
            current.positions, next.positions, current_jpqv_times,
            next_jpqv_times, current.positions, next.positions,
            current_jpqv_times, next_jpqv_times)) {
      return true;
    }

    const float smallest_cylinder_begin_time = vector_util::MinElement(
        vector_util::AddVectorElements(start_times, current.times));
    // Check all others against this deep dive to see if they collide.
    std::vector<Eigen::Vector2i> current_positions = current.positions;
    std::vector<float> current_times =
        vector_util::AddVectorElements(start_times, current.times);
    while (vector_util::MinElement(current_times) >=
               smallest_cylinder_begin_time &&
           current_positions != start_node_predecessors) {
      const auto find_result = path_map.find(current_positions);
      if (!kProduction) {
        if (find_result == path_map.end()) {
          LOG(FATAL) << "Find failed!";
        }
      }
      const std::vector<Eigen::Vector2i> previous_positions =
          find_result->second;
      if (previous_positions == start_node_predecessors) {
        return false;
      }
      const std::vector<float> previous_times = CalculatePreviousTimes(
          current_positions, previous_positions, current_times);

      if (collision_checks::SegmentsCollideWithSegments(
              current.positions, next.positions, current_jpqv_times,
              next_jpqv_times, previous_positions, current_positions,
              previous_times, current_times)) {
        return true;
      }

      current_positions = previous_positions;
      current_times = previous_times;
    }
    return false;
  }

  bool FoundGoal(const LimitedReplanScenerio& scenerio,
                 const std::vector<Eigen::Vector2i>& positions) const {
    if (!kProduction) {
      if (scenerio.replan_ends.size() != positions.size()) {
        LOG(FATAL) << "Sizes do not match!";
      }
    }
    for (size_t i = 0; i < scenerio.replan_ends.size(); ++i) {
      if (positions[i] != scenerio.replan_ends[i]) {
        return false;
      }
    }
    return true;
  }

  LimitedReplanScenerio ConvertToLimitedReplanScenerio(
      const ReplanScenerio<kRobotCount>& scenerio) {
    std::vector<size_t> replanned_paths;
    std::vector<Eigen::Vector2i> replan_starts;
    std::vector<float> replan_start_times;
    std::vector<size_t> replan_starts_indices;
    std::vector<Eigen::Vector2i> replan_ends;
    std::vector<size_t> replan_ends_indices;
    const Eigen::Vector2i grid_center = scenerio.grid_center;
    for (size_t i = 0; i < kRobotCount; ++i) {
      if (scenerio.needs_replans[i]) {
        replanned_paths.push_back(i);
        replan_starts.push_back(scenerio.replan_starts[i]);
        replan_start_times.push_back(scenerio.replan_start_times[i]);
        replan_starts_indices.push_back(scenerio.replan_starts_indices[i]);
        replan_ends.push_back(scenerio.replan_ends[i]);
        replan_ends_indices.push_back(scenerio.replan_ends_indices[i]);
      }
    }
    return {replanned_paths,    replan_starts,
            replan_start_times, replan_starts_indices,
            replan_ends,        replan_ends_indices,
            grid_center};
  }

 public:
  VectorAStarEightGridRepairer(const float& grid_distance_between_vertices,
                               const size_t& initial_repair_radius,
                               const size_t& repair_radius_step_size)
      : EightGridRepairer<kRobotCount>(grid_distance_between_vertices,
                                       initial_repair_radius,
                                       repair_radius_step_size) {}

  ReplanResult<kRobotCount> PerformReplan(
      const ReplanScenerio<kRobotCount>& full_scenerio,
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& static_invalid_vertices,
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& dynamic_invalid_vertices,
      const size_t& iteration_count) override {
    const size_t replan_radius =
        CalculateReplanRadius(full_scenerio, iteration_count);

    const LimitedReplanScenerio limited_scenerio =
        ConvertToLimitedReplanScenerio(full_scenerio);

    std::priority_queue<JointPriorityQueueVertex,
                        std::vector<JointPriorityQueueVertex>>
        priority_queue;

    start_node_predecessors = std::vector<Eigen::Vector2i>(
        limited_scenerio.replan_starts.size(), {kIntMaxHack, kIntMaxHack});
    // Push on the initial element of the queue, which is the element from the
    // given start to the given goal.
    priority_queue.push(
        {limited_scenerio.replan_starts,
         std::vector<float>(limited_scenerio.replan_starts.size(), 0.0f),
         Super::JointHeuristic(limited_scenerio.replan_starts,
                               limited_scenerio.replan_ends),
         start_node_predecessors, limited_scenerio.replan_starts_indices});

    std::unordered_set<std::vector<Eigen::Vector2i>, VectorGridHasher>
        closed_list(Super::planar_total_vertex_count_);
    std::unordered_map<std::vector<Eigen::Vector2i>,
                       std::vector<Eigen::Vector2i>, VectorGridHasher>
        path_map;

    while (!priority_queue.empty()) {
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

      // Check to see if we're at the goal position.
      if (FoundGoal(limited_scenerio, top_queue_joint_vertex.positions)) {
        ReplanResult<kRobotCount> result;
        std::vector<Eigen::Vector2i> current_key =
            top_queue_joint_vertex.positions;
        while (current_key != limited_scenerio.replan_starts) {
          for (size_t j = 0; j < current_key.size(); ++j) {
            const Eigen::Vector2i& position = current_key[j];
            // Checks to make sure that this will not push back a duplicate
            // element in the case where one path is shorter than the other and
            // thus there is no progress made on one path but there is the
            // other.
            std::vector<Eigen::Vector2i>& current_plan =
                result.fixed_plans[limited_scenerio.replanned_paths[j]];
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

        for (size_t j = 0; j < current_key.size(); ++j) {
          const Eigen::Vector2i& position = current_key[j];
          std::vector<Eigen::Vector2i>& current_plan =
              result.fixed_plans[limited_scenerio.replanned_paths[j]];
          if (current_plan.empty() ||
              current_plan[current_plan.size() - 1] != position) {
            current_plan.push_back(position);
          }
        }

        // Reverse the plans to all be in the forwards direction.
        for (auto& plan : result.fixed_plans) {
          std::reverse(plan.begin(), plan.end());
        }
        return result;
      }

      const auto dynamic_cartesian_product = Super::CalculateCartesianProduct(
          CalculateNeighbors(limited_scenerio, top_queue_joint_vertex));

      for (const std::vector<std::pair<Eigen::Vector2i, float>>& neighbors :
           dynamic_cartesian_product) {
        bool reject_sample = false;
        for (size_t robot_index = 0; robot_index < neighbors.size();
             ++robot_index) {
          const auto& next_position = neighbors[robot_index].first;
          if (Super::OutsideOfPlanningBox(
                  next_position, limited_scenerio.grid_center, replan_radius)) {
            reject_sample = true;
            break;
          }

          if (Super::OutsideOfField(neighbors[robot_index].first)) {
            reject_sample = true;
            break;
          }
        }

        if (reject_sample) {
          continue;
        }

        std::vector<Eigen::Vector2i> next_positions(neighbors.size());
        std::vector<float> next_distances = top_queue_joint_vertex.distances;
        for (size_t robot_index = 0; robot_index < neighbors.size();
             ++robot_index) {
          const auto& next_position = neighbors[robot_index].first;
          next_positions[robot_index] = next_position;
          next_distances[robot_index] += neighbors[robot_index].second;
        }

        auto next_indices = vector_util::AddToEachElement(
            top_queue_joint_vertex.path_indices, 1ul);
        JointPriorityQueueVertex next_jpqv(
            &next_positions, &next_distances,
            Super::JointHeuristic(next_positions, limited_scenerio.replan_ends),
            top_queue_joint_vertex.positions, &next_indices);

        if (!CheckStepCollides(top_queue_joint_vertex, next_jpqv,
                               limited_scenerio.replan_start_times, path_map)) {
          priority_queue.push(next_jpqv);
        } else {
        }
      }
    }

    if (!kProduction) {
      LOG(FATAL) << "Unfixed result";
    }
    return ReplanResult<kRobotCount>();
  }
};

}  // namespace repairer
}  // namespace repair
}  // namespace navigation

#endif  // SRC_NAVIGATION_REPAIR_VECTOR_ASTAR_EIGHT_GRID_REPAIRER_H_
