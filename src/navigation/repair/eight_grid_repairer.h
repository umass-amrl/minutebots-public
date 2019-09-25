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

#ifndef SRC_NAVIGATION_REPAIR_EIGHT_GRID_REPAIRER_H_
#define SRC_NAVIGATION_REPAIR_EIGHT_GRID_REPAIRER_H_

#include <algorithm>
#include <array>
#include <iomanip>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "constants/constants.h"
#include "navigation/eight_grid.h"
#include "navigation/repair/eight_grid_distance.h"
#include "navigation/repair/multipath_collision_checks.h"
#include "navigation/repair/replan_scenerio.h"
#include "replan_scenerio.pb.h"
#include "search.pb.h"
#include "util/array_util.h"
#include "util/serialization.h"

namespace navigation {
namespace repair {
namespace repairer {

// Number of steps to expand our replan outward with.
static constexpr size_t kReplanIterations = 4;
// Number of iterations we will do to repair a path
static constexpr size_t kMaxVerifyIterations = 2;

template <unsigned int kRobotCount>
struct ReplanResult {
  std::array<std::vector<Eigen::Vector2i>, kRobotCount> fixed_plans;
  ReplanResult()
      : fixed_plans(array_util::MakeArray<kRobotCount>(
            std::vector<Eigen::Vector2i>())) {}

  ReplanResult(
      const std::array<std::vector<Eigen::Vector2i>, kRobotCount>& fixed_plans)
      : fixed_plans(fixed_plans) {}
};

template <unsigned int kRobotCount>
struct ArrayGridHasher {
  std::size_t operator()(
      const std::array<Eigen::Vector2i, kRobotCount>& k) const {
    size_t hash = 1;
    for (size_t i = 0; i < kRobotCount; ++i) {
      const auto& p = k[i];
      hash +=
          p.dot(Eigen::Vector2i(1, field_dimensions::kFieldLength)) * (i + 1);
    }
    return hash;
  }
};

struct VectorGridHasher {
  std::size_t operator()(const std::vector<Eigen::Vector2i>& k) const {
    size_t hash = 1;
    for (const auto& p : k) {
      hash *= p.dot(Eigen::Vector2i(1, field_dimensions::kFieldLength));
    }
    return hash;
  }
};

// Holds the implicitly 3D position of each robot (X, Y, time), where time is
// a function of distance.
//
// Also holds distance and heuristic values for the priority queue.
template <unsigned int kRobotCount>
struct JointPriorityQueueVertex {
  std::array<bool, kRobotCount> needs_replans;
  std::array<Eigen::Vector2i, kRobotCount> positions;
  std::array<float, kRobotCount> distances;
  float heuristic;
  std::array<float, kRobotCount> times;
  std::array<Eigen::Vector2i, kRobotCount> predecessors;
  std::array<size_t, kRobotCount> path_indices;

  inline std::array<float, kRobotCount> DistancesToTimes(
      const std::array<float, kRobotCount>& distance) {
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
      const std::array<float, kRobotCount>& distances, const float heuristic,
      const std::array<Eigen::Vector2i, kRobotCount>& predecessors,
      const std::array<size_t, kRobotCount>& path_indices)
      : needs_replans(needs_replans),
        positions(positions),
        distances(distances),
        heuristic(heuristic),
        times(DistancesToTimes(distances)),
        predecessors(predecessors),
        path_indices(path_indices) {}

  bool operator<(const JointPriorityQueueVertex& other) const {
    const auto this_distance_sum =
        array_util::SelectiveSumArray(distances, needs_replans);
    const auto this_val = (this_distance_sum + heuristic +
                           array_util::SelectiveSumArray(times, needs_replans));
    const auto other_distance_sum =
        array_util::SelectiveSumArray(other.distances, other.needs_replans);
    const auto other_val =
        (other_distance_sum + other.heuristic +
         array_util::SelectiveSumArray(other.times, other.needs_replans));
    if (this_val == other_val) {
      return this_distance_sum > other_distance_sum;
    }
    return (this_val > other_val);
  }

  bool operator==(const JointPriorityQueueVertex& other) const {
    return (distances == other.distances &&
            heuristic == other.heuristic &&
            positions == other.positions &&
            times == other.times &&
            predecessors == other.predecessors);
  }

  float EvaluateCost() const {
    return (array_util::SelectiveSumArray(distances, needs_replans) +
            heuristic + array_util::SelectiveSumArray(times, needs_replans));
  }
};

// This class is a base class designed to specify an interface as well as
// general functions for eight grid repairers.
template <unsigned int kRobotCount>
class EightGridRepairer {
  static_assert(kRobotCount > 0, "Cannot have 0 robots.");
  friend struct ReplanScenerio<kRobotCount>;

 private:
  inline static Eigen::Vector2i Vector2fCastToVector2i(
      const Eigen::Vector2f& f) {
    return f.cast<int>();
  }

  std::vector<ReplanScenerio<kRobotCount>> MakeReplanScenerios(
      const std::array<std::vector<Eigen::Vector2i>, kRobotCount>&
          individual_plans,
      const std::array<std::vector<Distance>, kRobotCount>&
          individual_plan_times,
      const collision_checks::CollisionEvent<kRobotCount>& collision_event,
      const size_t& radius_steps, const size_t& radius_step_size) {
    const Eigen::Vector2i grid_center =
        Vector2fCastToVector2i(collision_event.event_center);
    int max_nessicary_start_radius = 0;
    std::array<bool, kRobotCount> needs_replans =
        array_util::MakeArray<kRobotCount>(false);
    for (size_t i = 0; i < kRobotCount; ++i) {
      needs_replans[i] = collision_event.path_data_array[i].enabled;
    }

    for (size_t i = 0; i < kRobotCount; ++i) {
      if (!needs_replans[i]) {
        continue;
      }
      const auto& plan = individual_plans[i];
      const auto& path_data = collision_event.path_data_array[i];
      const size_t start_replan_index =
          (path_data.lower_index > (kIndexExpansionConstant - 1))
              ? path_data.lower_index - kIndexExpansionConstant
              : 0;
      const size_t end_replan_index = std::min(
          path_data.upper_index + kIndexExpansionConstant, plan.size() - 1);
      if (!kProduction) {
        if (start_replan_index >= plan.size()) {
          LOG(FATAL) << "start_replan_index out of bounds!";
        }

        if (end_replan_index >= plan.size()) {
          LOG(FATAL) << "end_replan_index out of bounds!";
        }
      }
      const Eigen::Vector2i& start_position = plan[start_replan_index];
      const Eigen::Vector2i& end_position = plan[end_replan_index];
      const Eigen::Vector2i start_delta = (start_position - grid_center);
      const Eigen::Vector2i end_delta = (end_position - grid_center);
      max_nessicary_start_radius = std::max(
          {max_nessicary_start_radius, start_delta.lpNorm<Eigen::Infinity>(),
           end_delta.lpNorm<Eigen::Infinity>()});
    }
    const int min_replan_radius = max_nessicary_start_radius;

    std::array<size_t, kRobotCount> replan_starts_indices =
        array_util::MakeArray<kRobotCount>(std::numeric_limits<size_t>::max());
    std::array<size_t, kRobotCount> replan_ends_indices =
        array_util::MakeArray<kRobotCount>(std::numeric_limits<size_t>::max());

    for (size_t i = 0; i < kRobotCount; ++i) {
      if (!needs_replans[i]) {
        continue;
      }
      size_t start_index = std::numeric_limits<size_t>::max();
      size_t end_index = std::numeric_limits<size_t>::max();
      const auto& plan = individual_plans[i];
      for (size_t j = 0; j < plan.size(); ++j) {
        const Eigen::Vector2i delta = (plan[j] - grid_center);
        if (delta.lpNorm<Eigen::Infinity>() <= min_replan_radius) {
          start_index = j;
          break;
        }
      }
      if (!kProduction) {
        if (start_index == std::numeric_limits<size_t>::max()) {
          LOG(FATAL) << "Start index not found!";
        }
      }

      for (size_t j = start_index + 1; j < plan.size(); ++j) {
        const Eigen::Vector2i delta = (plan[j] - grid_center);
        if (delta.lpNorm<Eigen::Infinity>() > min_replan_radius) {
          end_index = j - 1;
          break;
        }
      }

      if (end_index == std::numeric_limits<size_t>::max()) {
        end_index = plan.size() - 1;
      }

      if (!kProduction) {
        if (start_index >= plan.size()) {
          LOG(FATAL) << "start_index out of bounds!";
        }

        if (end_index >= plan.size()) {
          LOG(FATAL) << "end_index out of bounds!";
        }
      }

      replan_starts_indices[i] = start_index;
      replan_ends_indices[i] = end_index;
    }

    std::vector<ReplanScenerio<kRobotCount>> replan_scenerios;
    for (size_t i = 0; i < radius_steps; ++i) {
      const std::array<Distance, kRobotCount> replan_start_times =
          array_util::GetIndexedElements(individual_plan_times, needs_replans,
                                         replan_starts_indices, Distance(0, 0));
      const std::array<Eigen::Vector2i, kRobotCount> replan_starts =
          array_util::GetIndexedElements(individual_plans, needs_replans,
                                         replan_starts_indices,
                                         kIgnorablePosition);
      const std::array<Eigen::Vector2i, kRobotCount> replan_ends =
          array_util::GetIndexedElements(individual_plans, needs_replans,
                                         replan_ends_indices,
                                         kIgnorablePosition);

      replan_scenerios.push_back(
          {individual_plans, needs_replans, replan_starts, replan_start_times,
           replan_starts_indices, replan_ends, replan_ends_indices, grid_center,
           (min_replan_radius + i)});

      // Shift the indices back one, keeping in mind the existence of bounds.
      for (size_t i = 0; i < kRobotCount; ++i) {
        if (!needs_replans[i]) {
          continue;
        }
        const auto& plan = individual_plans[i];
        if (replan_starts_indices[i] >= radius_step_size) {
          replan_starts_indices[i] -= radius_step_size;
        }
        if ((replan_ends_indices[i] + radius_step_size) < plan.size()) {
          replan_ends_indices[i] += radius_step_size;
        }

        if (!kProduction) {
          if (replan_starts_indices[i] >= plan.size()) {
            LOG(FATAL) << "replan_starts_indices out of bounds!";
          }

          if (replan_ends_indices[i] >= plan.size()) {
            LOG(FATAL) << "replan_ends_indices out of bounds!";
          }
        }
      }
    }
    return replan_scenerios;
  }

  void IntegrateReplanResult(
      std::array<std::vector<Eigen::Vector2i>, kRobotCount>* individual_plans,
      const ReplanScenerio<kRobotCount>& scenerio,
      const ReplanResult<kRobotCount>& result) const {
    for (size_t i = 0; i < kRobotCount; ++i) {
      if (scenerio.needs_replans[i]) {
        const auto& replacement = result.fixed_plans[i];
        auto& full_plan = (*individual_plans)[i];
        const size_t& start_index = scenerio.replan_starts_indices[i];
        const size_t& end_index = scenerio.replan_ends_indices[i];

        // Validate the returned path.
        if (!kProduction) {
          if (replacement.empty()) {
            LOG(FATAL) << "Plan " << i << " replacement path is empty!";
          }

          if (replacement[0] != scenerio.replan_starts[i]) {
            LOG(FATAL) << "Plan " << i
                       << " does not start in the proper place! ("
                       << replacement[0].x() << ", " << replacement[0].y()
                       << ") vs (" << scenerio.replan_starts[i].x() << ", "
                       << scenerio.replan_starts[i].y() << ")";
          }

          if (replacement[replacement.size() - 1] != scenerio.replan_ends[i]) {
            LOG(FATAL) << "Plan " << i << " does not end in the proper place!";
          }

          // Zero case handled above.
          Eigen::Vector2i previous_position = replacement[0];
          for (size_t i = 1; i < replacement.size(); ++i) {
            // Is 0 or 1 iff a single component is 1, is 2 if both X component
            // and Y
            // component is 1.
            const int l1_norm =
                static_cast<Eigen::Vector2i>(replacement[i] - previous_position)
                    .lpNorm<1>();
            const int linfinity_norm =
                static_cast<Eigen::Vector2i>(replacement[i] - previous_position)
                    .lpNorm<Eigen::Infinity>();
            if (linfinity_norm > 1) {
              LOG(FATAL) << "This is not a valid eight grid path! LInfinity "
                            "norm should "
                            "not be greater than 1 (is "
                         << linfinity_norm << ") between index " << i << " ("
                         << replacement[i].x() << ", " << replacement[i].y()
                         << ") and index " << (i - 1) << " ("
                         << previous_position.x() << ", "
                         << previous_position.y() << ")";
            }
            if (l1_norm > 2) {
              LOG(FATAL)
                  << "This is not a valid eight grid path! L1 norm should "
                     "not be greater than 2 (is "
                  << l1_norm << ")";
            }
            if (l1_norm < 0) {
              LOG(FATAL) << "Negative l1 norm (" << l1_norm
                         << ") between index " << i << " ("
                         << replacement[i].x() << ", " << replacement[i].y()
                         << ") and index " << (i - 1) << " ("
                         << previous_position.x() << ", "
                         << previous_position.y() << ")";
            }
            previous_position = replacement[i];
          }
        }
        // Remove old section.
        full_plan.erase(full_plan.begin() + start_index,
                        full_plan.begin() + end_index + 1);
        full_plan.insert(full_plan.begin() + start_index, replacement.begin(),
                         replacement.end());
      } else if (!kProduction) {
        if (!result.fixed_plans[i].empty()) {
          LOG(FATAL) << "Non-repaired paths should be empty! (Offending path: "
                     << i << ")";
        }
      }
    }
  }

  // =================
  // Path Manipulation/Conversion
  // =================
  float FreeSpacePathTime(const std::vector<Eigen::Vector2i>& path,
                          const size_t& index) const {
    if (path.empty()) {
      return 0;
    }

    if (!kProduction) {
      // Can never be less than zero, as type is unsigned.
      if (index >= path.size()) {
        LOG(FATAL) << "Index " << index << " not in path bounds (length "
                   << path.size() << ")";
      }
    }
    float total_distance = 0;
    Eigen::Vector2i previous_position = path[0];
    for (size_t i = 1; i <= index; ++i) {
      total_distance += this->DistanceBetweenContiguiousPathSegments(
          path[i], previous_position);
      previous_position = path[i];
    }
    return total_distance / kMaxRobotVelocity;
  }

  // Computes the time it takes to travel to reach each node. Assumes that
  // robot
  // is always traveling at max velocity directly along the path.
  //
  // Returns a list of times which the robot is at
  // exactly this paticular vertex. Other times can be calculated by
  // interpolating between this point and neighbor points.
  std::vector<Distance> TimedPath(
      const std::vector<Eigen::Vector2i>& path) const {
    if (path.empty()) {
      return {};
    }
    std::vector<Distance> timed_path(path.size());
    timed_path[0] = Distance(0, 0);
    Eigen::Vector2i previous_position = path[0];
    Distance total_distance(0, 0);
    // Zero case handled above.
    for (size_t i = 1; i < path.size(); ++i) {
      total_distance += this->DistanceBetweenContiguiousPathSegments(
          path[i], previous_position);
      timed_path[i] = total_distance;
      previous_position = path[i];
    }
    return timed_path;
  }

  std::array<std::vector<Distance>, kRobotCount> TimedPaths(
      const std::array<std::vector<Eigen::Vector2i>, kRobotCount>& paths)
      const {
    std::array<std::vector<Distance>, kRobotCount> timed_paths =
        array_util::MakeArray<kRobotCount>(std::vector<Distance>());
    for (size_t i = 0; i < kRobotCount; ++i) {
      timed_paths[i] = TimedPath(paths[i]);
    }
    return timed_paths;
  }

  std::array<std::vector<Eigen::Vector2f>, kRobotCount>
  ConvertGridPathsToFreeSpacePaths(
      const std::array<std::vector<Eigen::Vector2i>, kRobotCount>&
          initial_plans) const {
    std::array<std::vector<Eigen::Vector2f>, kRobotCount> final_float_plans =
        array_util::MakeArray<kRobotCount>(std::vector<Eigen::Vector2f>());
    for (size_t robot_index = 0; robot_index < kRobotCount; ++robot_index) {
      const auto& inital_int_vector = initial_plans[robot_index];
      auto& final_float_vector = final_float_plans[robot_index];
      final_float_vector.resize(inital_int_vector.size());
      for (size_t plan_index = 0; plan_index < inital_int_vector.size();
           ++plan_index) {
        final_float_vector[plan_index] =
            GridVertexToFreeSpace(inital_int_vector[plan_index]);
      }
    }
    return final_float_plans;
  }

 protected:
  const std::array<Eigen::Vector2i, kRobotCount> kStartNodePredecessors =
      array_util::MakeArray<kRobotCount>(
          Eigen::Vector2i(kIntMaxHack, kIntMaxHack));

  bool ContainsInvalidVertices(
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& static_invalid_vertices,
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& dynamic_invalid_vertices,
      const std::array<Eigen::Vector2i, kRobotCount>& next_positions,
      const std::array<bool, kRobotCount>& needs_replans) {
    for (size_t i = 0; i < kRobotCount; ++i) {
      if (!needs_replans[i]) {
        continue;
      }
      const auto& static_map = static_invalid_vertices[i];
      if (static_map.find(next_positions[i]) != static_map.end()) {
        return true;
      }
      const auto& dynamic_map = dynamic_invalid_vertices[i];
      if (dynamic_map.find(next_positions[i]) != dynamic_map.end()) {
        return true;
      }
    }
    return false;
  }

  std::array<Eigen::Vector2i, kRobotCount> StripIgnoredData(
      const std::array<Eigen::Vector2i, kRobotCount>& positions,
      const std::array<bool, kRobotCount>& needs_replans) {
    std::array<Eigen::Vector2i, kRobotCount> stripped_positions;
    for (size_t i = 0; i < kRobotCount; ++i) {
      stripped_positions[i] =
          ((needs_replans[i]) ? positions[i] : kIgnorablePosition);
    }
    return stripped_positions;
  }

  std::array<Distance, kRobotCount> StripIgnoredData(
      const std::array<Distance, kRobotCount>& distances,
      const std::array<bool, kRobotCount>& needs_replans) {
    std::array<Distance, kRobotCount> stripped_positions;
    for (size_t i = 0; i < kRobotCount; ++i) {
      stripped_positions[i] =
          ((needs_replans[i]) ? distances[i] : Distance(0, 0));
    }
    return stripped_positions;
  }

  bool OutsideOfField(const Eigen::Vector2i& p) const {
    const Eigen::Vector2i relative_to_field_edge_location =
        (p.cwiseAbs() -
         Eigen::Vector2i(static_cast<int>(field_dimensions::kHalfFieldLength /
                                          grid_distance_between_vertices_),
                         static_cast<int>(field_dimensions::kHalfFieldWidth /
                                          grid_distance_between_vertices_)));
    return (relative_to_field_edge_location.x() > 0 ||
            relative_to_field_edge_location.y() > 0);
  }

  bool OutsideOfField(
      const std::array<bool, kRobotCount>& needs_replans,
      const std::array<Eigen::Vector2i, kRobotCount>& positions) const {
    for (size_t i = 0; i < kRobotCount; ++i) {
      if (needs_replans[i]) {
        const auto& p = positions[i];
        if (OutsideOfField(p)) {
          return true;
        }
      }
    }
    return false;
  }

  bool OutsideOfPlanningBox(const Eigen::Vector2i& p,
                            const Eigen::Vector2i& box_center,
                            const int& radius) const {
    const Eigen::Vector2i delta = (p - box_center);
    return (delta.lpNorm<Eigen::Infinity>() > radius);
  }

  bool OutsideOfPlanningBox(
      const std::array<bool, kRobotCount>& needs_replans,
      const std::array<Eigen::Vector2i, kRobotCount>& positions,
      const Eigen::Vector2i& box_center, const int& radius) const {
    for (size_t i = 0; i < kRobotCount; ++i) {
      if (needs_replans[i]) {
        const auto& p = positions[i];
        if (OutsideOfPlanningBox(p, box_center, radius)) {
          return true;
        }
      }
    }
    return false;
  }

  Distance DistanceBetweenContiguiousPathSegments(
      const Eigen::Vector2i& s1, const Eigen::Vector2i& s2) const {
    const Eigen::Vector2i delta = (s1 - s2);

    if (!kProduction) {
      if (delta.lpNorm<Eigen::Infinity>() > 1) {
        LOG(FATAL) << "Not a valid eight grid path! Linfty > 1";
      }
    }

    const int l1norm = delta.lpNorm<1>();
    switch (l1norm) {
      case 0: {
        return {0, 0};
      } break;
      case 1: {
        return {1, 0};
      } break;
      case 2: {
        return {0, 1};
      } break;
      default: {
        if (!kProduction) {
          LOG(FATAL) << "Non-standard l1norm: " << l1norm;
        }
        return {0, 2};
      }
    }
  }

  std::array<Distance, kRobotCount> CalculatePreviousTimes(
      const std::array<Eigen::Vector2i, kRobotCount>& current_positions,
      const std::array<Eigen::Vector2i, kRobotCount>& previous_positions,
      const std::array<Distance, kRobotCount>& current_times,
      const std::array<bool, kRobotCount>& needs_replans) {
    std::array<Distance, kRobotCount> previous_times =
        array_util::MakeArray<kRobotCount>(Distance());
    for (size_t i = 0; i < kRobotCount; ++i) {
      if (!needs_replans[i]) {
        continue;
      }
      const Distance time = DistanceBetweenContiguiousPathSegments(
          current_positions[i], previous_positions[i]);

      //       if (!kProduction) {
      //         const Eigen::Vector2i delta =
      //             (current_positions[i] - previous_positions[i]);
      //         LOG(INFO) << current_positions[i].x() << ", "
      //                   << current_positions[i].y() << " => "
      //                   << previous_positions[i].x() << ", "
      //                   << previous_positions[i].y()
      //                   << " delta: " << delta.lpNorm<1>();
      //         LOG(INFO) << time.ToString();
      //       }

      previous_times[i] = (Distance(0, 0) != current_times[i])
                              ? current_times[i] - time
                              : Distance(0, 0);
      if (!kProduction) {
        if (current_times[i].IsNegative()) {
          LOG(FATAL) << "Current times negative! "
                     << current_times[i].ToString();
        }
        //         if ((current_times[i] - time).IsNegative()) {
        //           LOG(INFO) << "Current position: " <<
        //           current_positions[i].x() << ", "
        //                     << current_positions[i].y();
        //           LOG(INFO) << "Previous position: " <<
        //           previous_positions[i].x()
        //                     << ", " << previous_positions[i].y();
        //           LOG(FATAL) << "Negative time! (" <<
        //           current_times[i].ToString()
        //                      << " - " << time.ToString() << " = "
        //                      << (current_times[i] - time).ToString() << ")";
        //         }
      }
    }
    return previous_times;
  }

  bool CheckStepCollides(
      const std::array<Eigen::Vector2i, kRobotCount>& current_step_positions,
      const std::array<Distance, kRobotCount>& current_step_times,
      const std::array<Eigen::Vector2i, kRobotCount>& next_step_positions,
      const std::array<Distance, kRobotCount>& next_step_times,
      const std::array<bool, kRobotCount>& needs_replans,
      const std::array<Distance, kRobotCount>& start_times,
      const std::unordered_map<std::array<Eigen::Vector2i, kRobotCount>,
                               std::array<Eigen::Vector2i, kRobotCount>,
                               ArrayGridHasher<kRobotCount>>& path_map) {
    if (!kProduction) {
      for (size_t i = 0; i < kRobotCount; ++i) {
        if (start_times[i].IsNegative()) {
          LOG(FATAL) << "Start time " << i << " (" << start_times[i].ToString()
                     << ") is negative!";
        }

        if (current_step_times[i].IsNegative()) {
          LOG(FATAL) << "Current step time " << i << " ("
                     << current_step_times[i].ToString() << ") is negative!";
        }

        if (next_step_times[i].IsNegative()) {
          LOG(FATAL) << "Next step time " << i << " ("
                     << next_step_times[i].ToString() << ") is negative!";
        }
      }
    }

    const auto current_jpqv_times =
        array_util::AddArrayElements(current_step_times, start_times);
    const auto next_jpqv_times =
        array_util::AddArrayElements(next_step_times, start_times);

    if (!kProduction) {
      for (size_t i = 0; i < kRobotCount; ++i) {
        if (needs_replans[i]) {
          if (current_jpqv_times[i].IsNegative()) {
            LOG(FATAL) << "Current jpqv time negative!";
          }

          if (next_jpqv_times[i].IsNegative()) {
            LOG(FATAL) << "Next jpqv time negative!";
          }
        }
      }
    }

    // If next step collides with itself.
    if (collision_checks::SegmentsCollideWithSegments<kRobotCount>(
            current_step_positions, next_step_positions, current_jpqv_times,
            next_jpqv_times, current_step_positions, next_step_positions,
            current_jpqv_times, next_jpqv_times, needs_replans)) {
      return true;
    }

    // This is as far as we need to look back in order to verify we don't
    // collide.
    const Distance smallest_cylinder_begin_time =
        array_util::SelectiveMinElement(current_jpqv_times, needs_replans);
    // Check all others against this deep dive to see if they collide.
    std::array<Eigen::Vector2i, kRobotCount> current_positions =
        current_step_positions;

    // These times have been offset with start times, so they should never be
    // smaller than start times.
    std::array<Distance, kRobotCount> current_times = current_jpqv_times;
    while (array_util::SelectiveMaxElement(current_times, needs_replans) >=
               smallest_cylinder_begin_time &&
           current_positions != kStartNodePredecessors) {
      if (!kProduction) {
        for (size_t i = 0; i < kRobotCount; ++i) {
          if (needs_replans[i]) {
            if (current_times[i] < Distance(0, 0)) {
              LOG(FATAL) << "Current time negative!";
            }
          }
        }
      }
      const auto find_result = path_map.find(current_positions);
      if (!kProduction) {
        if (find_result == path_map.end()) {
          for (size_t i = 0; i < kRobotCount; ++i) {
            if (!needs_replans[i]) {
              LOG(INFO) << i << ": skipped";
              continue;
            }
            LOG(INFO) << i << ": " << current_positions[i].x() << ", "
                      << current_positions[i].y();
          }
          LOG(FATAL) << "Find failed!";
        }
      }
      const std::array<Eigen::Vector2i, kRobotCount> previous_positions =
          find_result->second;
      if (previous_positions == kStartNodePredecessors) {
        return false;
      }

      const std::array<Distance, kRobotCount> previous_times =
          CalculatePreviousTimes(current_positions, previous_positions,
                                 current_times, needs_replans);

      if (!kProduction) {
        for (size_t i = 0; i < kRobotCount; ++i) {
          if (needs_replans[i]) {
            if (current_times[i] < start_times[i]) {
              LOG(ERROR) << "Current:";
              for (size_t i = 0; i < kRobotCount; ++i) {
                if (!needs_replans[i]) {
                  LOG(ERROR) << i << ": skip";
                  continue;
                }
                LOG(ERROR) << i << ": " << current_positions[i].x() << ", "
                           << current_positions[i].y() << " @ "
                           << current_times[i].ToString();
              }

              LOG(ERROR) << "Start:";
              for (size_t i = 0; i < kRobotCount; ++i) {
                if (!needs_replans[i]) {
                  LOG(ERROR) << i << ": skip";
                  continue;
                }
                LOG(ERROR) << i << ": " << start_times[i].ToString();
              }
              LOG(ERROR) << "Current time comes before start time!!!";
            }

            if (current_times[i] < Distance(0, 0)) {
              LOG(FATAL) << "Current time negative!";
            }
            if (previous_times[i] < Distance(0, 0)) {
              LOG(FATAL) << "Previous time negative!";
            }

            if (previous_times[i].ComponentsNegative()) {
              LOG(ERROR) << "Current:";
              for (size_t i = 0; i < kRobotCount; ++i) {
                if (!needs_replans[i]) {
                  LOG(ERROR) << i << ": skip";
                  continue;
                }
                LOG(ERROR) << i << ": " << current_positions[i].x() << ", "
                           << current_positions[i].y() << " @ "
                           << current_times[i].ToString();
              }

              LOG(ERROR) << "Previous:";
              for (size_t i = 0; i < kRobotCount; ++i) {
                if (!needs_replans[i]) {
                  LOG(ERROR) << i << ": skip";
                  continue;
                }
                LOG(ERROR) << i << ": " << previous_positions[i].x() << ", "
                           << previous_positions[i].y() << " @ "
                           << previous_times[i].ToString();
              }

              LOG(ERROR) << "Start:";
              for (size_t i = 0; i < kRobotCount; ++i) {
                if (!needs_replans[i]) {
                  LOG(ERROR) << i << ": skip";
                  continue;
                }
                LOG(ERROR) << i << ": " << start_times[i].ToString();
              }

              LOG(FATAL) << "Previous time has negative components, when it "
                            "should just be a walk back of the current time! "
                         << previous_times[i].ToString();
            }
          }
        }
      }

      if (collision_checks::SegmentsCollideWithSegments<kRobotCount>(
              current_step_positions, next_step_positions, current_jpqv_times,
              next_jpqv_times, previous_positions, current_positions,
              previous_times, current_times, needs_replans)) {
        return true;
      }

      current_positions = previous_positions;
      current_times = previous_times;
    }
    return false;
  }

  // =================
  // Collision Checks
  // =================
  bool CheckPointsCollide(const Eigen::Vector2i& p1,
                          const Eigen::Vector2i& p2) const {
    const Eigen::Vector2i delta = (p1 - p2);
    return (delta.lpNorm<Eigen::Infinity>() <= 1);
  }

  bool CheckTimedPointsCollide(const Eigen::Vector2i& p1, const float& p1_t,
                               const Eigen::Vector2i& p2,
                               const float& p2_t) const {
    if (CheckPointsCollide(p1, p2)) {
      const float time_drive_at_eachother_from_neighboring_squares =
          grid_distance_between_vertices_ / 2.0f / kMaxRobotVelocity;
      return (fabs(p1_t - p2_t) <=
              time_drive_at_eachother_from_neighboring_squares);
    } else {
      return false;
    }
  }

  bool CheckProposedStepCollides(
      const std::array<Eigen::Vector2i, kRobotCount>& positions,
      const std::array<float, kRobotCount>& times,
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& static_invalid_vertices,
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& dynamic_invalid_vertices) const {
    for (size_t i = 0; i < kRobotCount; ++i) {
      const Eigen::Vector2i& current_vertex = positions[i];

      const auto static_find = static_invalid_vertices[i].find(current_vertex);
      const auto dynamic_find =
          dynamic_invalid_vertices[i].find(current_vertex);

      const bool static_find_result =
          (static_find != static_invalid_vertices[i].end());
      if (static_find_result) {
        return true;
      }
      const bool dynamic_find_result =
          (dynamic_find != dynamic_invalid_vertices[i].end());
      if (dynamic_find_result) {
        return true;
      }

      const float current_time = times[i];
      for (size_t j = 0; j < kRobotCount; ++j) {
        if (i != j) {
          const Eigen::Vector2i& other_current_vertex = positions[j];
          const float other_current_time = times[j];
          if (CheckTimedPointsCollide(current_vertex, current_time,
                                      other_current_vertex,
                                      other_current_time)) {
            return true;
          }
        }
      }
    }
    return false;
  }

  Distance Heuristic2d(const Eigen::Vector2i& position,
                       const Eigen::Vector2i& goal) const {
    const auto& diff_vector = (goal - position).cwiseAbs();
    const int x_diff = diff_vector.x();
    const int y_diff = diff_vector.y();

    int smaller, larger;
    if (x_diff < y_diff) {
      smaller = x_diff;
      larger = y_diff;
    } else {
      smaller = y_diff;
      larger = x_diff;
    }

    return {larger - smaller, smaller};
  }

  Distance JointHeuristic(
      const std::array<bool, kRobotCount>& needs_replans,
      const std::array<Eigen::Vector2i, kRobotCount>& current,
      const std::array<Eigen::Vector2i, kRobotCount>& goal) const {
    Distance sum(0, 0);
    for (size_t i = 0; i < kRobotCount; ++i) {
      if (needs_replans[i]) {
        sum += Heuristic2d(current[i], goal[i]);
      }
    }
    return sum;
  }

  Distance JointHeuristic(const std::vector<Eigen::Vector2i>& current,
                          const std::vector<Eigen::Vector2i>& goal) const {
    if (!kProduction) {
      if (current.size() != goal.size()) {
        LOG(FATAL) << "Sizes do not match!";
      }
    }
    Distance sum(0, 0);
    for (size_t i = 0; i < current.size(); ++i) {
      sum += Heuristic2d(current[i], goal[i]);
    }
    return sum;
  }

  std::vector<std::array<std::pair<Eigen::Vector2i, Distance>, kRobotCount>>
  CalculateDynamicCartesianProductNine(
      const std::array<std::array<std::pair<Eigen::Vector2i, Distance>, 9>,
                       kRobotCount>& position_neighbors_or_self,
      const std::array<bool, kRobotCount>& explorable) {
    const auto default_positions = array_util::MakeArray<kRobotCount>(
        std::make_pair(Eigen::Vector2i(std::numeric_limits<int>::max(),
                                       std::numeric_limits<int>::max()),
                       Distance()));

    std::vector<std::array<std::pair<Eigen::Vector2i, Distance>, kRobotCount>>
        cartesian_product_vector = {default_positions};

    for (size_t robot_index = 0; robot_index < kRobotCount; ++robot_index) {
      if (!explorable[robot_index]) {
        for (auto& e : cartesian_product_vector) {
          e[robot_index] = position_neighbors_or_self[robot_index][0];
        }
        continue;
      }

      const size_t old_count = cartesian_product_vector.size();
      cartesian_product_vector.reserve(9 * old_count);
      for (size_t i = 1; i < 9; ++i) {
        std::copy_n(cartesian_product_vector.begin(), old_count,
                    std::back_inserter(cartesian_product_vector));
      }

      // Iterate over the eight blocks and assign one of the eight values to
      // each block.
      for (size_t block_index = 0; block_index < 9; ++block_index) {
        for (size_t intrablock_index = 0; intrablock_index < old_count;
             ++intrablock_index) {
          cartesian_product_vector[block_index * old_count + intrablock_index]
                                  [robot_index] =
                                      position_neighbors_or_self[robot_index]
                                                                [block_index];
        }
      }
    }

    return cartesian_product_vector;
  }

  std::vector<std::array<std::pair<Eigen::Vector2i, Distance>, kRobotCount>>
  CalculateDynamicCartesianProduct(
      const std::array<std::array<std::pair<Eigen::Vector2i, Distance>, 8>,
                       kRobotCount>& position_neighbors_or_self,
      const std::array<bool, kRobotCount>& explorable) {
    const auto default_positions = array_util::MakeArray<kRobotCount>(
        std::make_pair(Eigen::Vector2i(std::numeric_limits<int>::max(),
                                       std::numeric_limits<int>::max()),
                       std::numeric_limits<float>::max()));

    std::vector<std::array<std::pair<Eigen::Vector2i, Distance>, kRobotCount>>
        cartesian_product_vector = {default_positions};

    for (size_t robot_index = 0; robot_index < kRobotCount; ++robot_index) {
      if (!explorable[robot_index]) {
        for (auto& e : cartesian_product_vector) {
          e[robot_index] = position_neighbors_or_self[robot_index][0];
        }
        continue;
      }

      const size_t old_count = cartesian_product_vector.size();
      cartesian_product_vector.reserve(8 * old_count);
      for (size_t i = 1; i < 8; ++i) {
        std::copy_n(cartesian_product_vector.begin(), old_count,
                    std::back_inserter(cartesian_product_vector));
      }

      // Iterate over the eight blocks and assign one of the eight values to
      // each block.
      for (size_t block_index = 0; block_index < 8; ++block_index) {
        for (size_t intrablock_index = 0; intrablock_index < old_count;
             ++intrablock_index) {
          cartesian_product_vector[block_index * old_count + intrablock_index]
                                  [robot_index] =
                                      position_neighbors_or_self[robot_index]
                                                                [block_index];
        }
      }
    }

    return cartesian_product_vector;
  }

  // Array of (array whose elements are positions to look for each robot).
  std::array<std::array<std::pair<Eigen::Vector2i, Distance>, kRobotCount>,
             math_util::ConstexprPow(8, kRobotCount)>
  CalculateCartesianProduct(
      const std::array<std::array<std::pair<Eigen::Vector2i, Distance>, 8>,
                       kRobotCount>& position_neighbors) {
    std::array<std::array<std::pair<Eigen::Vector2i, Distance>, kRobotCount>,
               math_util::ConstexprPow(8, kRobotCount)>
        cartesian_product_array =
            array_util::MakeArray<math_util::ConstexprPow(8, kRobotCount)>(
                array_util::MakeArray<kRobotCount>(std::make_pair(
                    Eigen::Vector2i(std::numeric_limits<int>::max(),
                                    std::numeric_limits<int>::max()),
                    Distance())));

    for (size_t robot_index = 0; robot_index < kRobotCount; ++robot_index) {
      // 1 in the initial case, 8 in the next case, 64 in the next case, etc.
      // This determines the size of the current chunk.
      const size_t current_chunk_size = math_util::Pow<size_t>(8, robot_index);

      // Should take the first chunk and copy it over 7 times.
      for (size_t chunk_copy_index = 1; chunk_copy_index < 8;
           ++chunk_copy_index) {
        for (size_t chunk_element_copy_index = 0;
             chunk_element_copy_index < current_chunk_size;
             ++chunk_element_copy_index) {
          cartesian_product_array[chunk_copy_index * current_chunk_size +
                                  chunk_element_copy_index] =
              cartesian_product_array[chunk_element_copy_index];
        }
      }

      for (size_t chunk_index = 0; chunk_index < 8; ++chunk_index) {
        for (size_t chunk_element_write_index = 0;
             chunk_element_write_index < current_chunk_size;
             ++chunk_element_write_index) {
          cartesian_product_array[chunk_index * current_chunk_size +
                                  chunk_element_write_index][robot_index] =
              position_neighbors[robot_index][chunk_index];
        }
      }
    }
    return cartesian_product_array;
  }

  // Array of (array whose elements are positions to look for each robot).
  std::vector<std::vector<std::pair<Eigen::Vector2i, Distance>>>
  CalculateCartesianProduct(
      const std::vector<std::array<std::pair<Eigen::Vector2i, Distance>, 8>>&
          position_neighbors) {
    const std::vector<std::pair<Eigen::Vector2i, Distance>> inner_vector(
        position_neighbors.size(), {{kIntMaxHack, kIntMaxHack}, Distance()});
    std::vector<std::vector<std::pair<Eigen::Vector2i, Distance>>>
        cartesian_product_array(math_util::Pow(8, position_neighbors.size()),
                                inner_vector);

    for (size_t robot_index = 0; robot_index < position_neighbors.size();
         ++robot_index) {
      // 1 in the initial case, 8 in the next case, 64 in the next case, etc.
      // This determines the size of the current chunk.
      const size_t current_chunk_size = math_util::Pow<size_t>(8, robot_index);

      // Should take the first chunk and copy it over 7 times.
      for (size_t chunk_copy_index = 1; chunk_copy_index < 8;
           ++chunk_copy_index) {
        for (size_t chunk_element_copy_index = 0;
             chunk_element_copy_index < current_chunk_size;
             ++chunk_element_copy_index) {
          cartesian_product_array[chunk_copy_index * current_chunk_size +
                                  chunk_element_copy_index] =
              cartesian_product_array[chunk_element_copy_index];
        }
      }

      for (size_t chunk_index = 0; chunk_index < 8; ++chunk_index) {
        for (size_t chunk_element_write_index = 0;
             chunk_element_write_index < current_chunk_size;
             ++chunk_element_write_index) {
          cartesian_product_array[chunk_index * current_chunk_size +
                                  chunk_element_write_index][robot_index] =
              position_neighbors[robot_index][chunk_index];
        }
      }
    }
    return cartesian_product_array;
  }

  // =================
  // Neighbor Generation
  // =================
  std::array<std::pair<Eigen::Vector2i, Distance>, 8> GetNeighbors(
      const Eigen::Vector2i& vertex) const {
    return {{{Eigen::Vector2i(vertex.x() + 1, vertex.y() - 1), {0, 1}},
             {Eigen::Vector2i(vertex.x() + 1, vertex.y() + 1), {0, 1}},
             {Eigen::Vector2i(vertex.x() + 1, vertex.y()), {1, 0}},
             {Eigen::Vector2i(vertex.x(), vertex.y() - 1), {1, 0}},
             {Eigen::Vector2i(vertex.x(), vertex.y() + 1), {1, 0}},
             {Eigen::Vector2i(vertex.x() - 1, vertex.y() - 1), {0, 1}},
             {Eigen::Vector2i(vertex.x() - 1, vertex.y() + 1), {0, 1}},
             {Eigen::Vector2i(vertex.x() - 1, vertex.y()), {1, 0}}}};
  }

  std::array<std::pair<Eigen::Vector2i, Distance>, 9> GetNeighborsNine(
      const Eigen::Vector2i& vertex) const {
    return {{{Eigen::Vector2i(vertex.x() + 1, vertex.y() - 1), {0, 1}},
             {Eigen::Vector2i(vertex.x() + 1, vertex.y() + 1), {0, 1}},
             {Eigen::Vector2i(vertex.x() + 1, vertex.y()), {1, 0}},
             {Eigen::Vector2i(vertex.x(), vertex.y() - 1), {1, 0}},
             {Eigen::Vector2i(vertex.x(), vertex.y() + 1), {1, 0}},
             {Eigen::Vector2i(vertex.x() - 1, vertex.y() - 1), {0, 1}},
             {Eigen::Vector2i(vertex.x() - 1, vertex.y() + 1), {0, 1}},
             {Eigen::Vector2i(vertex.x() - 1, vertex.y()), {1, 0}},
             {vertex, {0, 0}}}};
  }

  std::array<std::pair<Eigen::Vector2i, Distance>, 8> GetIdentityNeighbors(
      const Eigen::Vector2i& vertex) const {
    return {{{vertex, {0, 0}},
             {vertex, {0, 0}},
             {vertex, {0, 0}},
             {vertex, {0, 0}},
             {vertex, {0, 0}},
             {vertex, {0, 0}},
             {vertex, {0, 0}},
             {vertex, {0, 0}}}};
  }

  std::array<std::pair<Eigen::Vector2i, Distance>, 9> GetIdentityNeighborsNine(
      const Eigen::Vector2i& vertex) const {
    return {{{vertex, {0, 0}},
             {vertex, {0, 0}},
             {vertex, {0, 0}},
             {vertex, {0, 0}},
             {vertex, {0, 0}},
             {vertex, {0, 0}},
             {vertex, {0, 0}},
             {vertex, {0, 0}},
             {vertex, {0, 0}}}};
  }

  // =================
  // Grid Vertex Managment
  // =================
  Eigen::Vector2i FreeSpaceToGridVertex(
      const Eigen::Vector2f& free_space_vector) const {
    const float x_multiple =
        free_space_vector.x() / grid_distance_between_vertices_;
    const float y_multiple =
        free_space_vector.y() / grid_distance_between_vertices_;
    const float x_remaining =
        x_multiple - static_cast<float>(static_cast<int>(x_multiple));
    const float y_remaining =
        y_multiple - static_cast<float>(static_cast<int>(y_multiple));
    const int x_index =
        static_cast<int>(x_multiple) +
        ((fabs(x_remaining) > 0.5) ? math_util::Sign<float>(x_remaining) : 0);
    const int y_index =
        static_cast<int>(y_multiple) +
        ((fabs(y_remaining) > 0.5) ? math_util::Sign<float>(y_remaining) : 0);
    return {x_index, y_index};
  }

  Eigen::Vector2i FreeSpaceToOpenGridVertex(
      const obstacle::ObstacleFlag& obstacle_flag,
      const obstacle::SafetyMargin& safety_margin,
      const Eigen::Vector2f& free_space_vector) const {
    const Eigen::Vector2i closest_vector =
        FreeSpaceToGridVertex(free_space_vector);
    Eigen::Vector2i proposed_vector = closest_vector;

    int neighbors_index = 8;
    std::array<std::pair<Eigen::Vector2i, float>, 8> neighbors;

    // Guarenteed termination.
    for (int i = 0; i < planar_total_vertex_count_; ++i) {
      bool proposed_free = true;
      for (const auto* obstacle : obstacle_flag) {
        if (obstacle->PointCollision(
                GridVertexToFreeSpace(proposed_vector),
                safety_margin.GetMargin(obstacle->GetType()))) {
          proposed_free = false;
          break;
        }
      }

      if (proposed_free) {
        return proposed_vector;
      }

      if (neighbors_index >= 8) {
        neighbors = GetNeighbors(proposed_vector);
        neighbors_index = 0;
      }

      if (!kProduction) {
        if (neighbors_index < 0 || neighbors_index >= 8) {
          LOG(FATAL) << "Neighbors index is out of range.";
        }
      }

      proposed_vector = neighbors[neighbors_index].first;
      neighbors_index++;
    }

    return closest_vector;
  }

  Eigen::Vector2f GridVertexToFreeSpace(const Eigen::Vector2i& in) const {
    return in.cast<float>() * grid_distance_between_vertices_;
  }

  void DumpReplanScenerio(
      const size_t& iteration_index,
      const ReplanScenerio<kRobotCount>& scenerio,
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& static_invalid_vertices,
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& dynamic_invalid_vertices) {
    //     LOG(INFO) << "Dumping replan scenerio!";
    // PrintReplanScenerio(scenerio);
    scenerio.Dump(
        "replan_scenerio_" + std::to_string(iteration_index) + ".proto",
        static_invalid_vertices, dynamic_invalid_vertices);
    //     LOG(INFO) << "Dumped replan scenerio!";
  }

  void DumpParentMap(
      const size_t& scenerio_index, const size_t& step_count,
      const std::string& file_name_modifier,
      const std::unordered_map<std::array<Eigen::Vector2i, kRobotCount>,
                               std::array<Eigen::Vector2i, kRobotCount>,
                               ArrayGridHasher<kRobotCount>>& path_map) {
    const std::string file_name =
        "parentmap_" + std::to_string(scenerio_index) + "_step_" +
        std::to_string(step_count) + "_" + file_name_modifier + ".proto";
    int file_descriptor =
        ::util::serialization::CreateOrEraseFileForWrite(file_name);
    MinuteBotsProto::ParentMap parent_map_proto;
    for (const auto& p : path_map) {
      auto* edge_proto = parent_map_proto.add_edges();
      for (const auto& s : p.first) {
        auto* position_proto = edge_proto->add_current_positions();
        position_proto->set_x(s.x());
        position_proto->set_y(s.y());
      }
      for (const auto& s : p.second) {
        auto* position_proto = edge_proto->add_prev_positions();
        position_proto->set_x(s.x());
        position_proto->set_y(s.y());
      }
    }

    google::protobuf::io::FileOutputStream file_output(file_descriptor);
    file_output.SetCloseOnDelete(true);
    if (!google::protobuf::TextFormat::Print(parent_map_proto, &file_output)) {
      LOG(ERROR) << "Failed to write path proto to " << file_name;
    }
  }

  void DumpSearchTree(
      const size_t& scenerio_index, const size_t& step_count,
      const std::unordered_map<std::array<Eigen::Vector2i, kRobotCount>,
                               std::array<Eigen::Vector2i, kRobotCount>,
                               ArrayGridHasher<kRobotCount>>& path_map,
      const std::vector<std::array<Eigen::Vector2i, kRobotCount>>&
          closed_positions,
      const ReplanScenerio<kRobotCount>& scenerio,
      const std::array<Eigen::Vector2i, kRobotCount>& current_position,
      const std::array<Eigen::Vector2i, kRobotCount>& next_position,
      const Distance heuristic,
      const std::array<Distance, kRobotCount>& distances,
      const std::array<Eigen::Vector2i, kRobotCount>& start_position,
      const std::string& status) {
    const std::string file_name = "scenerio_" + std::to_string(scenerio_index) +
                                  "_step_" + std::to_string(step_count) +
                                  ".proto";
    int file_descriptor =
        ::util::serialization::CreateOrEraseFileForWrite(file_name);
    MinuteBotsProto::SearchTree search_tree;
    search_tree.set_status(status);
    auto* current_edge_proto = search_tree.mutable_current_edge();
    auto* heuristic_proto = current_edge_proto->mutable_heuristic();
    heuristic_proto->set_straight(heuristic.straight_count);
    heuristic_proto->set_angled(heuristic.angle_count);
    auto* current_edge_search_tree_edge_proto =
        current_edge_proto->mutable_search_tree_edge();
    for (size_t i = 0; i < kRobotCount; ++i) {
      if (!scenerio.needs_replans[i]) {
        continue;
      }
      auto* distance_proto = current_edge_proto->add_distances();
      distance_proto->set_straight(distances[i].straight_count);
      distance_proto->set_angled(distances[i].angle_count);
      auto* times_proto = current_edge_proto->add_times();
      times_proto->set_straight(distances[i].straight_count);
      times_proto->set_angled(distances[i].angle_count);
      auto* current_position_proto =
          current_edge_search_tree_edge_proto->add_current_positions();
      auto* prev_position_proto =
          current_edge_search_tree_edge_proto->add_prev_positions();
      current_position_proto->set_x(next_position[i].x());
      current_position_proto->set_y(next_position[i].y());
      prev_position_proto->set_x(current_position[i].x());
      prev_position_proto->set_y(current_position[i].y());
    }

    //     for (const auto& c : closed_positions) {
    //       auto* closed_position_proto = search_tree.add_closed_positions();
    //       for (const auto& p : c) {
    //         auto* shared_position_proto =
    //         closed_position_proto->add_position();
    //         shared_position_proto->set_x(p.x());
    //         shared_position_proto->set_y(p.y());
    //       }
    //     }

    std::array<Eigen::Vector2i, kRobotCount> current_key = current_position;
    while (current_key != start_position) {
      const auto current_find = path_map.find(current_key);
      if (!kProduction) {
        if (current_find == path_map.end()) {
          for (size_t i = 0; i < kRobotCount; ++i) {
            if (!scenerio.needs_replans[i]) {
              LOG(INFO) << i << ": skip";
              continue;
            }
            LOG(INFO) << i << ": " << current_key[i].x() << ", "
                      << current_key[i].y();
          }
          LOG(FATAL) << "Could not find current key in map";
        }
      }

      auto* current_edge_proto = search_tree.add_other_edge_list();
      for (size_t i = 0; i < kRobotCount; ++i) {
        if (!scenerio.needs_replans[i]) {
          continue;
        }
        auto* current_position_proto =
            current_edge_proto->add_current_positions();
        auto* prev_position_proto = current_edge_proto->add_prev_positions();
        current_position_proto->set_x(current_key[i].x());
        current_position_proto->set_y(current_key[i].y());
        prev_position_proto->set_x(current_find->second[i].x());
        prev_position_proto->set_y(current_find->second[i].y());
      }

      current_key = current_find->second;
    }

    google::protobuf::io::FileOutputStream file_output(file_descriptor);
    file_output.SetCloseOnDelete(true);
    if (!google::protobuf::TextFormat::Print(search_tree, &file_output)) {
      LOG(ERROR) << "Failed to write path proto to " << file_name;
    }
  }

  void PrintReplanScenerio(const ReplanScenerio<kRobotCount>& scenerio) {
    LOG(INFO) << "Replan Scenerio";

    //     LOG(INFO) << "Plans:";
    //     for (const auto& plan: scenerio.individual_plans) {
    //       LOG(INFO) << "P";
    //       for (const auto& position : plan) {
    //         LOG(INFO) << position.transpose();
    //       }
    //     }

    for (size_t i = 0; i < kRobotCount; ++i) {
      if (!scenerio.needs_replans[i]) {
        LOG(INFO) << i << ": no replan needed";
      } else {
        LOG(INFO)
            << i << ": " << std::fixed << std::setprecision(4)
            << "Grid Center: " << scenerio.grid_center.x() << ", "
            << scenerio.grid_center.y()
            << " Start Position: " << scenerio.replan_starts[i].x() << ", "
            << scenerio.replan_starts[i].y()
            << " Time: " << scenerio.replan_start_times[i].ToString()
            << " Index: " << scenerio.replan_starts_indices[i]
            << " (Indexed position: "
            << scenerio.individual_plans[i][scenerio.replan_starts_indices[i]]
                   .x()
            << ", "
            << scenerio.individual_plans[i][scenerio.replan_starts_indices[i]]
                   .y()
            << ")"
            << " End Position: " << scenerio.replan_ends[i].x() << ", "
            << scenerio.replan_ends[i].y()
            << " Index: " << scenerio.replan_ends_indices[i]
            << " (Indexed position: "
            << scenerio.individual_plans[i][scenerio.replan_ends_indices[i]].x()
            << ", "
            << scenerio.individual_plans[i][scenerio.replan_ends_indices[i]].y()
            << ")"
            << " Radius: " << scenerio.replan_radius;
      }
    }
  }

 private:
  void PrintReplanResult(const ReplanScenerio<kRobotCount>& scenerio,
                         const ReplanResult<kRobotCount>& result) {
    LOG(INFO) << "Replan Result";

    std::array<std::vector<float>, kRobotCount> timed_replacements =
        TimedPaths(result.fixed_plans);
    for (size_t i = 0; i < kRobotCount; ++i) {
      if (!scenerio.needs_replans[i]) {
        continue;
      }
      for (auto& e : timed_replacements[i]) {
        e += scenerio.replan_start_times[i];
      }
    }

    for (size_t i = 0; i < kRobotCount; ++i) {
      if (!scenerio.needs_replans[i]) {
        LOG(INFO) << i << ": no replan needed";
      } else {
        LOG(INFO) << i << " >>>>>>>: len: " << result.fixed_plans[i].size();
        for (size_t j = 0; j < result.fixed_plans[i].size(); ++j) {
          const auto& e = result.fixed_plans[i][j];
          const float& t = timed_replacements[i][j];
          LOG(INFO) << e.x() << ", " << e.y() << " @ " << t;
        }
      }
    }
  }

  void PrintPairIntersections(
      const std::vector<collision_checks::ContiguiousPathCollision>&
          pair_intersections) {
    for (const collision_checks::ContiguiousPathCollision& intersection :
         pair_intersections) {
      LOG(INFO) << std::fixed << std::setprecision(4) << "Intersection: p"
                << intersection.path1_index << " from "
                << intersection.path1_min << " to " << intersection.path1_max
                << " \tp" << intersection.path2_index << " from "
                << intersection.path2_min << " to " << intersection.path2_max
                << " \tt: " << intersection.time_center;
    }
  }

  void ValidateStartEndPositions(
      const ReplanScenerio<kRobotCount>& replan_scenerio,
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& static_invalid_vertices,
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& dynamic_invalid_vertices) {
    for (size_t j = 0; j < kRobotCount; ++j) {
      const auto& obstacle_flag = obstacles_array_[j];
      const auto& safety_margin = safety_margin_array_[j];
      const auto& start_integer_position = replan_scenerio.replan_starts[j];
      const auto& goal_integer_position = replan_scenerio.replan_ends[j];

      const auto goal_freespace_position =
          GridVertexToFreeSpace(goal_integer_position);
      for (const auto* obstacle : obstacle_flag) {
        if (obstacle->PointCollision(
                goal_freespace_position,
                safety_margin.GetMargin(obstacle->GetType()))) {
          LOG(FATAL) << "Collides with obstacle";
        }
        if ((obstacle->GetPose().translation - goal_freespace_position)
                .squaredNorm() <
            Sq(kRobotRadius * 2 +
               safety_margin.GetMargin(obstacle->GetType()))) {
          LOG(FATAL) << "Two robot radius... is collision";
        }
      }

      if (static_invalid_vertices[j].find(start_integer_position) !=
          static_invalid_vertices[j].end()) {
        LOG(FATAL) << "Start in static obstacle!";
      }

      if (dynamic_invalid_vertices[j].find(start_integer_position) !=
          dynamic_invalid_vertices[j].end()) {
        LOG(FATAL) << "Start in dynamic obstacle!";
      }

      if (static_invalid_vertices[j].find(goal_integer_position) !=
          static_invalid_vertices[j].end()) {
        LOG(FATAL) << "Goal in static obstacle!";
      }

      if (dynamic_invalid_vertices[j].find(goal_integer_position) !=
          dynamic_invalid_vertices[j].end()) {
        LOG(FATAL) << "Goal in dynamic obstacle!";
      }
    }
  }

 public:
  EightGridRepairer(const float grid_distance_between_vertices,
                    const size_t initial_repair_radius,
                    const size_t repair_radius_step_size)
      : obstacles_array_(array_util::MakeArray<kRobotCount>(
            obstacle::ObstacleFlag::GetEmpty())),
        safety_margin_array_(
            array_util::MakeArray<kRobotCount>(obstacle::SafetyMargin())),
        grid_distance_between_vertices_(grid_distance_between_vertices),
        planar_total_vertex_count_(
            (static_cast<int>(field_dimensions::kFieldLength /
                              grid_distance_between_vertices_) +
             1) *
            (static_cast<int>(field_dimensions::kFieldWidth /
                              grid_distance_between_vertices_) +
             1)),
        initial_repair_radius_(initial_repair_radius),
        repair_radius_step_size_(repair_radius_step_size) {}

  virtual ~EightGridRepairer() = default;

  void Update(
      const std::array<obstacle::ObstacleFlag, kRobotCount>& obstacles_array,
      const std::array<obstacle::SafetyMargin, kRobotCount>&
          safety_margin_array) {
    obstacles_array_ = obstacles_array;
    safety_margin_array_ = safety_margin_array;
  }

  std::array<std::vector<Eigen::Vector2i>, kRobotCount> VerifyAndRepairPlans(
      const std::array<std::vector<Eigen::Vector2i>, kRobotCount>&
          individual_plans,
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& static_invalid_vertices,
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& dynamic_invalid_vertices) {
    std::array<std::vector<Eigen::Vector2i>, kRobotCount> repaired_plans =
        individual_plans;

    const auto start_time = GetMonotonicTime();
    // TODO(kvedder): Instead of throwing away and refinding collisions each
    // time, forward carry index changes to the other scenerios.

    //     LOG(ERROR) << "remove me!";
    //     return repaired_plans;

    size_t i = 0;
    do {
      const std::array<std::vector<Distance>, kRobotCount>
          repaired_plans_times = TimedPaths(repaired_plans);

      if (!kProduction) {
        for (size_t i = 0; i < kRobotCount; ++i) {
          const auto& plan = repaired_plans[i];
          const auto& plan_times = repaired_plans_times[i];
          if (plan.size() != plan_times.size()) {
            LOG(FATAL) << "Sizes are not equal";
          }
          for (size_t j = 0; j < plan.size(); ++j) {
            LOG(INFO) << "p: " << plan[j].x() << ", " << plan[j].y() << " @ "
                      << plan_times[j].ToString();
          }
        }
      }

      const std::vector<collision_checks::CollisionEvent<kRobotCount>>
          collision_events =
              ::navigation::repair::collision_checks::CollisionEvents<
                  kRobotCount>(repaired_plans, repaired_plans_times);

      // No repairs to be made, so exit.
      if (collision_events.empty()) {
        return repaired_plans;
      }

      const std::vector<ReplanScenerio<kRobotCount>> scenerios_list =
          MakeReplanScenerios(repaired_plans, repaired_plans_times,
                              collision_events[0], kReplanIterations,
                              repair_radius_step_size_);

      if (!kProduction) {
        for (const auto& scenerio : scenerios_list) {
          ValidateStartEndPositions(scenerio, static_invalid_vertices,
                                    dynamic_invalid_vertices);
        }
      }

      const std::pair<ReplanResult<kRobotCount>, ReplanScenerio<kRobotCount>>
          replan_result_pair =
              PerformReplan(scenerios_list, static_invalid_vertices,
                            dynamic_invalid_vertices);

      if (!kProduction) {
        std::array<std::vector<Distance>, kRobotCount> timed_replacements =
            TimedPaths(replan_result_pair.first.fixed_plans);
        for (size_t i = 0; i < kRobotCount; ++i) {
          if (!replan_result_pair.second.needs_replans[i]) {
            continue;
          }
          for (auto& e : timed_replacements[i]) {
            e += replan_result_pair.second.replan_start_times[i];
          }
        }

        for (size_t i = 0; i < kRobotCount; ++i) {
          const std::vector<Eigen::Vector2i>& outer_path =
              replan_result_pair.first.fixed_plans[i];
          for (size_t j = 0; j < kRobotCount; ++j) {
            const std::vector<Eigen::Vector2i>& inner_path =
                replan_result_pair.first.fixed_plans[j];
            if (i != j) {
              const auto result = collision_checks::PathPathCollisionIndices(
                  i, outer_path, timed_replacements[i], j, inner_path,
                  timed_replacements[j]);
              const auto result2 = collision_checks::PathPathCollisionIndices(
                  j, inner_path, timed_replacements[j], i, outer_path,
                  timed_replacements[i]);
              if (!result.empty() || !result2.empty()) {
                LOG(INFO) << "Result";
                for (const auto& r : result) {
                  const auto& plan1 =
                      replan_result_pair.first.fixed_plans[r.path1_index];
                  const auto& plan2 =
                      replan_result_pair.first.fixed_plans[r.path2_index];

                  const auto& plan1_start = plan1[r.path1_min];
                  const auto& plan1_end = plan1[r.path1_max];
                  const Distance plan1_start_time =
                      timed_replacements[r.path1_index][r.path1_min];
                  const Distance plan1_end_time =
                      timed_replacements[r.path1_index][r.path1_max];

                  const auto& plan2_start = plan2[r.path2_min];
                  const auto& plan2_end = plan2[r.path2_max];
                  const Distance plan2_start_time =
                      timed_replacements[r.path2_index][r.path2_min];
                  const Distance plan2_end_time =
                      timed_replacements[r.path2_index][r.path2_max];

                  LOG(INFO) << std::fixed << std::setprecision(10) << "Path "
                            << r.path1_index << " from " << r.path1_min << " ("
                            << plan1_start.x() << ", " << plan1_start.y()
                            << ") @ " << plan1_start_time.ToString() << " to "
                            << r.path1_max << " (" << plan1_end.x() << ", "
                            << plan1_end.y() << ") @ "
                            << plan1_end_time.ToString();
                  LOG(INFO) << std::fixed << std::setprecision(10) << "Path "
                            << r.path2_index << " from " << r.path2_min << " ("
                            << plan2_start.x() << ", " << plan2_start.y()
                            << ") @ " << plan2_start_time.ToString() << " to "
                            << r.path2_max << " (" << plan2_end.x() << ", "
                            << plan2_end.y() << ") @ "
                            << plan2_end_time.ToString();
                }

                LOG(INFO) << "Result2";
                for (const auto& r : result2) {
                  const auto& plan1 =
                      replan_result_pair.first.fixed_plans[r.path1_index];
                  const auto& plan2 =
                      replan_result_pair.first.fixed_plans[r.path2_index];

                  const auto& plan1_start = plan1[r.path1_min];
                  const auto& plan1_end = plan1[r.path1_max];
                  const Distance plan1_start_time =
                      timed_replacements[r.path1_index][r.path1_min];
                  const Distance plan1_end_time =
                      timed_replacements[r.path1_index][r.path1_max];
                  const size_t plan1_mid_index =
                      (r.path1_max + r.path1_min) / 2;
                  const Distance plan1_mid_time =
                      timed_replacements[r.path1_index][plan1_mid_index];
                  const auto& plan1_mid = plan1[plan1_mid_index];

                  const auto& plan2_start = plan2[r.path2_min];
                  const auto& plan2_end = plan2[r.path2_max];
                  const Distance plan2_start_time =
                      timed_replacements[r.path2_index][r.path2_min];
                  const Distance plan2_end_time =
                      timed_replacements[r.path2_index][r.path2_max];
                  const size_t plan2_mid_index =
                      (r.path2_max + r.path2_min) / 2;
                  const Distance plan2_mid_time =
                      timed_replacements[r.path2_index][plan2_mid_index];
                  const auto& plan2_mid = plan2[plan2_mid_index];

                  LOG(INFO) << std::fixed << std::setprecision(10) << "Path "
                            << r.path1_index << " from " << r.path1_min << " ("
                            << plan1_start.x() << ", " << plan1_start.y()
                            << ") @ " << plan1_start_time.ToString() << " to "
                            << r.path1_max << " (" << plan1_end.x() << ", "
                            << plan1_end.y() << ") @ "
                            << plan1_end_time.ToString()
                            << " mid: " << plan1_mid_time.ToString() << " ("
                            << plan1_mid.x() << ", " << plan1_mid.y() << ")";
                  LOG(INFO) << std::fixed << std::setprecision(10) << "Path "
                            << r.path2_index << " from " << r.path2_min << " ("
                            << plan2_start.x() << ", " << plan2_start.y()
                            << ") @ " << plan2_start_time.ToString() << " to "
                            << r.path2_max << " (" << plan2_end.x() << ", "
                            << plan2_end.y() << ") @ "
                            << plan2_end_time.ToString()
                            << " mid: " << plan2_mid_time.ToString() << " ("
                            << plan2_mid.x() << ", " << plan2_mid.y() << ")";
                }
                LOG(ERROR) << "Still incurrs a collision!";
              }
            }
          }
        }
      }

      IntegrateReplanResult(&repaired_plans, replan_result_pair.second,
                            replan_result_pair.first);
      ++i;
    } while (i < kMaxVerifyIterations);
    if (!kProduction) {
      const auto end_time = GetMonotonicTime();
      LOG(ERROR) << "Exceeded max verify iterations (" << kMaxVerifyIterations
                 << "), took in ms: " << (end_time - start_time) * 1000.0;
    }
    return repaired_plans;
  }

  virtual std::pair<ReplanResult<kRobotCount>, ReplanScenerio<kRobotCount>>
  PerformReplan(
      const std::vector<ReplanScenerio<kRobotCount>>& scenerio_list,
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& static_invalid_vertices,
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& dynamic_invalid_vertices) = 0;

 protected:
  std::array<obstacle::ObstacleFlag, kRobotCount> obstacles_array_;
  std::array<obstacle::SafetyMargin, kRobotCount> safety_margin_array_;

  // Linear distance between two adjacent vertices of the grid.
  const float grid_distance_between_vertices_;
  // Total number of vertices in the 2D plane.
  const int planar_total_vertex_count_;
  const size_t initial_repair_radius_;
  const size_t repair_radius_step_size_;

  static const Eigen::Vector2i kIgnorablePosition;
  const std::array<std::pair<Eigen::Vector2i, Distance>, 8>
      kIgnorableNeighbors = array_util::MakeArray<8>(std::make_pair(
          EightGridRepairer<kRobotCount>::kIgnorablePosition, Distance()));

  const std::array<std::pair<Eigen::Vector2i, Distance>, 9>
      kIgnorableNeighborsNine = array_util::MakeArray<9>(std::make_pair(
          EightGridRepairer<kRobotCount>::kIgnorablePosition, Distance()));
};

template <unsigned int kRobotCount>
const Eigen::Vector2i EightGridRepairer<kRobotCount>::kIgnorablePosition =
    (Eigen::Vector2i() << 1337, 1337).finished();

}  // namespace repairer
}  // namespace repair
}  // namespace navigation

#endif  // SRC_NAVIGATION_REPAIR_EIGHT_GRID_REPAIRER_H_
