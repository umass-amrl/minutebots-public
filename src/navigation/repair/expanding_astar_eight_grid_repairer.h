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

#ifndef SRC_NAVIGATION_REPAIR_EXPANDING_ASTAR_EIGHT_GRID_REPAIRER_H_
#define SRC_NAVIGATION_REPAIR_EXPANDING_ASTAR_EIGHT_GRID_REPAIRER_H_

#include <gtest/gtest_prod.h>
#include <algorithm>
#include <array>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "datastructures/vector_priority_queue.h"
#include "graph/graph_util.h"
#include "navigation/repair/eight_grid_repairer.h"
#include "util/serialization.h"

namespace navigation {
namespace repair {
namespace repairer {

template <unsigned int kRobotCount>
class ExpandingAStarEightGridRepairer : public EightGridRepairer<kRobotCount> {
  FRIEND_TEST(ExpandingAStar, AddPathBetweenOldAndNewStart);

  using Super = EightGridRepairer<kRobotCount>;

  enum JPQVSource {
    CURRENT_SEARCH,
    PREV_SEARCH_FILL,
    PREV_OPEN_LIST,
    OUT_OF_BOX
  };

  std::string JPQVSourceToString(const JPQVSource& source) {
    switch (source) {
      case CURRENT_SEARCH:
        return "CURRENT_SEARCH";
      case PREV_SEARCH_FILL:
        return "PREV_SEARCH_FILL";
      case PREV_OPEN_LIST:
        return "PREV_OPEN_LIST";
      case OUT_OF_BOX:
        return "OUT_OF_BOX";
      default:
        return "UNKNOWN";
    }
  }

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
    JPQVSource source;

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
        const std::array<size_t, kRobotCount>& path_indices,
        const JPQVSource& source)
        : needs_replans(needs_replans),
          positions(positions),
          distances(distances),
          heuristic(heuristic),
          predecessors(predecessors),
          path_indices(path_indices),
          source(source) {}

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
  CalculateNeighbors(const std::array<bool, kRobotCount>& needs_replans,
                     const std::array<Eigen::Vector2i, kRobotCount>& goal,
                     const JointPriorityQueueVertex& current_jpqv) {
    // Initialize with default data for the sake of debugging.
    const Distance default_distance(kIntMaxHack, kIntMaxHack);
    std::array<std::array<std::pair<Eigen::Vector2i, Distance>, 9>, kRobotCount>
        position_neighbors = array_util::MakeArray<kRobotCount>(
            array_util::MakeArray<9>(std::make_pair(
                Eigen::Vector2i(std::numeric_limits<int>::max() - 3,
                                std::numeric_limits<int>::max() - 3),
                default_distance)));

    for (size_t robot_index = 0; robot_index < kRobotCount; ++robot_index) {
      const Eigen::Vector2i& position = current_jpqv.positions[robot_index];
      if (!needs_replans[robot_index]) {
        position_neighbors[robot_index] = Super::kIgnorableNeighborsNine;
      } else {
        position_neighbors[robot_index] = Super::GetNeighborsNine(position);
      }
    }
    return position_neighbors;
  }

  std::array<Distance, kRobotCount> CalculatePathDistances(
      const std::array<std::vector<Eigen::Vector2i>, kRobotCount>& paths,
      const std::array<bool, kRobotCount>& needs_replans,
      const std::array<size_t, kRobotCount>& start_indices,
      const std::array<size_t, kRobotCount>& end_indices) {
    if (array_util::SelectiveEqual(needs_replans, start_indices, end_indices)) {
      return array_util::MakeArray<kRobotCount>(Distance(0, 0));
    }

    std::array<Distance, kRobotCount> distance_array =
        array_util::MakeArray<kRobotCount>(Distance(0, 0));
    for (size_t i = 0; i < kRobotCount; ++i) {
      if (!needs_replans[i]) {
        continue;
      }
      const auto& start_index = start_indices[i];
      const auto& end_index = end_indices[i];
      const auto& path = paths[i];
      if (!kProduction) {
        if (start_index > end_index) {
          LOG(FATAL) << "Indices out of order";
        }

        if (start_index >= path.size()) {
          LOG(FATAL) << "Start index out of bounds!";
        }
        if (end_index >= path.size()) {
          LOG(FATAL) << "End index out of bounds!";
        }
      }

      Eigen::Vector2i prev_position = path[start_index];
      for (size_t j = start_index + 1; j <= end_index; ++j) {
        const Eigen::Vector2i& current_position = path[j];
        const Distance distance_delta =
            Super::DistanceBetweenContiguiousPathSegments(current_position,
                                                          prev_position);
        distance_array[i] += distance_delta;
        prev_position = current_position;
      }
    }
    return distance_array;
  }

  std::array<Eigen::Vector2i, kRobotCount> IndicesToPositions(
      const std::array<std::vector<Eigen::Vector2i>, kRobotCount>& paths,
      const std::array<bool, kRobotCount>& needs_replans,
      const std::array<size_t, kRobotCount>& indices) {
    std::array<Eigen::Vector2i, kRobotCount> positions;
    for (size_t i = 0; i < kRobotCount; ++i) {
      const auto& path = paths[i];
      if (!needs_replans[i]) {
        continue;
      }
      const auto& index = indices[i];
      if (!kProduction) {
        if (index >= path.size()) {
          LOG(FATAL) << "Index out of range!";
        }
      }
      positions[i] = path[index];
    }
    return Super::StripIgnoredData(positions, needs_replans);
  }

  // Adds the states between the current start and the previous start to the
  // open list with a zero heuristic, ensuring that these states will be
  // expanded before any others.
  void AddIndividualOptimalPaths(
      const std::array<std::vector<Eigen::Vector2i>, kRobotCount>& paths,
      const std::array<bool, kRobotCount>& needs_replans,
      const std::array<size_t, kRobotCount>& prev_window_start_indices,
      const std::array<size_t, kRobotCount>& current_window_start_indicies,
      datastructures::VectorPriorityQueue<JointPriorityQueueVertex>* open_list,
      std::unordered_map<std::array<Eigen::Vector2i, kRobotCount>,
                         std::array<Eigen::Vector2i, kRobotCount>,
                         ArrayGridHasher<kRobotCount>>* parent_map) {
    // Early out if the start and end are the same.
    if (array_util::SelectiveEqual(needs_replans, prev_window_start_indices,
                                   current_window_start_indicies)) {
      return;
    }

    // Verify that the windows are in the proper order.
    if (!kProduction) {
      for (size_t i = 0; i < kRobotCount; ++i) {
        if (!needs_replans[i]) {
          continue;
        }
        // Current window is larger, thus grabbing an earlier index.
        if (current_window_start_indicies[i] >= prev_window_start_indices[i]) {
          LOG(ERROR) << current_window_start_indicies[i]
                     << " >= " << prev_window_start_indices[i];
          LOG(FATAL) << "Indices out of order.";
        }
      }
    }

    size_t max_index_delta = 0;
    for (size_t i = 0; i < kRobotCount; ++i) {
      max_index_delta =
          std::max(max_index_delta, prev_window_start_indices[i] -
                                        current_window_start_indicies[i]);
    }

    LOG(INFO) << "Max index delta: " << max_index_delta;

    std::array<size_t, kRobotCount> current_indices =
        current_window_start_indicies;
    std::array<Distance, kRobotCount> distances =
        array_util::MakeArray<kRobotCount>(Distance(0, 0));
    Distance distance_sum(0, 0);

    std::array<Eigen::Vector2i, kRobotCount> prev_positions =
        Super::kStartNodePredecessors;
    std::array<Eigen::Vector2i, kRobotCount> current_positions =
        IndicesToPositions(paths, needs_replans, current_window_start_indicies);

    // Step forward through the paths from the current start to the previous
    // start.
    for (size_t i = 0; i <= max_index_delta; ++i) {
      // Note that the heuristic of zero distance is intentionally designed to
      // ensure that these vertices will be expanded first.
      const JointPriorityQueueVertex vertex(
          needs_replans, current_positions, distances, Distance(0, 0),
          prev_positions, current_indices, JPQVSource::PREV_SEARCH_FILL);

      open_list->Push(vertex);

      // Advance forward the current indices, capping out at the previous
      // window.
      for (size_t robot_index = 0; robot_index < kRobotCount; ++robot_index) {
        if (needs_replans[robot_index] &&
            current_indices[robot_index] <
                prev_window_start_indices[robot_index]) {
          current_indices[robot_index]++;
        }
      }

      if (i == max_index_delta) {
        continue;
      }

      // Prep work for the next iteration.
      prev_positions = current_positions;
      current_positions =
          IndicesToPositions(paths, needs_replans, current_indices);
      for (size_t robot_index = 0; robot_index < kRobotCount; ++robot_index) {
        if (needs_replans[robot_index]) {
          const auto distance = Super::DistanceBetweenContiguiousPathSegments(
              prev_positions[robot_index], current_positions[robot_index]);
          distance_sum += distance;
          distances[robot_index] += distance;
        }
      }
    }

    if (!kProduction) {
      if (current_positions == prev_positions) {
        LOG(FATAL) << "Current and prev positions are the same.";
      }
    }

    // Rewrite the parent of the previous start to be on the correct path.
    (*parent_map)[current_positions] = prev_positions;
    for (auto& e : *open_list->GetMutableVector()) {
      if (e.positions == current_positions) {
        e.predecessors = prev_positions;
      }
    }

    if (!kProduction) {
      const std::array<Eigen::Vector2i, kRobotCount> expected_end_positions =
          IndicesToPositions(paths, needs_replans, prev_window_start_indices);
      if (expected_end_positions != current_positions) {
        LOG(INFO) << "Expected end:";
        for (const auto& p : expected_end_positions) {
          LOG(INFO) << p.x() << ", " << p.y();
        }
        LOG(INFO) << "Actual end:";
        for (const auto& p : current_positions) {
          LOG(INFO) << p.x() << ", " << p.y();
        }
        LOG(FATAL)
            << "Expansion from new start to old start did not properly end!";
      }
    }
  }

  ReplanResult<kRobotCount> UnwindPath(
      const std::unordered_map<std::array<Eigen::Vector2i, kRobotCount>,
                               std::array<Eigen::Vector2i, kRobotCount>,
                               ArrayGridHasher<kRobotCount>>& path_map,
      const std::array<Eigen::Vector2i, kRobotCount>& stripped_replan_starts,
      const std::array<Eigen::Vector2i, kRobotCount>& stripped_replan_ends,
      const std::array<bool, kRobotCount>& needs_replans) {
    if (!kProduction) {
      if (Super::kStartNodePredecessors == stripped_replan_starts ||
          Super::kStartNodePredecessors == stripped_replan_ends) {
        LOG(FATAL) << "Start or end is kStartNodePredecessors!";
      }
      for (const auto& pair : path_map) {
        if (pair.first == pair.second) {
          LOG(ERROR) << "Self loop detected!!!";
          for (const auto& p : pair.first) {
            LOG(ERROR) << p.x() << ", " << p.y();
          }
          LOG(FATAL) << "Path map has self loops!";
        }
      }
    }

    ReplanResult<kRobotCount> result;
    std::array<Eigen::Vector2i, kRobotCount> current_key = stripped_replan_ends;
    while (stripped_replan_starts != current_key) {
      if (!kProduction && Super::kStartNodePredecessors == current_key) {
        LOG(FATAL) << "Current key is kStartNodePredecessors!";
      }

      for (size_t i = 0; i < kRobotCount; ++i) {
        // Skip robots that do not need to be replanned.
        if (!needs_replans[i]) {
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
          for (size_t i = 0; i < kRobotCount; ++i) {
            if (!needs_replans[i]) {
              LOG(ERROR) << "Skipped";
              continue;
            }
            const Eigen::Vector2i& p = current_key[i];
            LOG(ERROR) << p.x() << ", " << p.y();
          }
          LOG(FATAL)
              << "Could not find current key in path map; unwind failed!";
        }
      }
      current_key = current_find->second;
    }

    for (size_t i = 0; i < kRobotCount; ++i) {
      // Skip robots that do not need to be replanned.
      if (!needs_replans[i]) {
        continue;
      }
      const Eigen::Vector2i& position = stripped_replan_starts[i];
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

    if (!kProduction) {
      for (size_t i = 0; i < kRobotCount; ++i) {
        if (!needs_replans[i]) {
          continue;
        }
        if (result.fixed_plans[i][0] != stripped_replan_starts[i]) {
          LOG(FATAL) << "Final path does not start at the start!";
        }
        if (result.fixed_plans[i][result.fixed_plans[i].size() - 1] !=
            stripped_replan_ends[i]) {
          LOG(FATAL) << "Final path does not end at the end!";
        }
      }
    }

    return result;
  }

  // Expands the existing search tree to the next scenerio.
  void AStarRepair(
      const std::string& phase_name, const size_t& iteration, size_t* counter,
      const ReplanScenerio<kRobotCount>& scenerio,
      std::array<Distance, kRobotCount> replan_start_times,
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& static_invalid_vertices,
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& dynamic_invalid_vertices,
      datastructures::VectorPriorityQueue<JointPriorityQueueVertex>* open_list,
      std::unordered_map<std::array<Eigen::Vector2i, kRobotCount>, Distance,
                         ArrayGridHasher<kRobotCount>>* closed_list,
      std::unordered_map<std::array<Eigen::Vector2i, kRobotCount>,
                         std::array<Eigen::Vector2i, kRobotCount>,
                         ArrayGridHasher<kRobotCount>>* parent_map,
      std::vector<JointPriorityQueueVertex>* out_of_box_vertices,
      const std::array<Eigen::Vector2i, kRobotCount>& starts,
      const std::array<Eigen::Vector2i, kRobotCount>& goals,
      const JointPriorityQueueVertex& last_searched) {
    //     LOG(FATAL) << "I KNOW THE BUG!!! It's caused by feeding the start
    //     times of "
    //                   "the next scenerio into the colision checks, while we
    //                   should "
    //                   "be using the previous scenerio during the window
    //                   growing "
    //                   "phase.";
    size_t& open_list_add_count = *counter;
    while (!open_list->Empty() && !(open_list->Top() < last_searched)) {
      const JointPriorityQueueVertex top = open_list->Top();
      open_list->Pop();
      const auto find_result = closed_list->find(top.positions);
      const Distance g_value = array_util::SelectiveSumArray<kRobotCount>(
          top.distances, scenerio.needs_replans, Distance(0, 0));
      if (find_result != closed_list->end() && find_result->second <= g_value) {
        continue;
      }
      (*closed_list)[top.positions] = g_value;
      (*parent_map)[top.positions] = top.predecessors;
      const auto dynamic_cartesian_product =
          Super::CalculateDynamicCartesianProductNine(
              CalculateNeighbors(scenerio.needs_replans, goals, top),
              scenerio.needs_replans);
      for (const std::array<std::pair<Eigen::Vector2i, Distance>, kRobotCount>&
               neighbors : dynamic_cartesian_product) {
        std::array<Eigen::Vector2i, kRobotCount> next_positions =
            array_util::MakeArray<kRobotCount>(
                Eigen::Vector2i(kIntMaxHack, kIntMaxHack));
        std::array<Distance, kRobotCount> next_distances = top.distances;
        for (size_t robot_index = 0; robot_index < kRobotCount; ++robot_index) {
          next_positions[robot_index] = neighbors[robot_index].first;
          next_distances[robot_index] += neighbors[robot_index].second;
        }

        JointPriorityQueueVertex next_jpqv(
            scenerio.needs_replans, next_positions, next_distances,
            Super::JointHeuristic(scenerio.needs_replans, next_positions,
                                  goals),
            top.positions, array_util::AddToEachElement(top.path_indices, 1ul),
            JPQVSource::PREV_SEARCH_FILL);

        if (Super::OutsideOfField(scenerio.needs_replans, next_positions)) {
          if (!kProduction) {
            ++open_list_add_count;
            std::vector<std::array<Eigen::Vector2i, kRobotCount>>
                closed_positions;
            for (const auto& e : *closed_list) {
              closed_positions.push_back(e.first);
            }
            Super::DumpSearchTree(iteration, open_list_add_count, *parent_map,
                                  closed_positions, scenerio,
                                  next_jpqv.predecessors, next_jpqv.positions,
                                  next_jpqv.heuristic, next_jpqv.distances,
                                  Super::kStartNodePredecessors,
                                  phase_name + "A* repair Outside field");
          }
          continue;
        }

        if (Super::ContainsInvalidVertices(
                static_invalid_vertices, dynamic_invalid_vertices,
                next_positions, scenerio.needs_replans)) {
          if (!kProduction) {
            ++open_list_add_count;
            std::vector<std::array<Eigen::Vector2i, kRobotCount>>
                closed_positions;
            for (const auto& e : *closed_list) {
              closed_positions.push_back(e.first);
            }
            Super::DumpSearchTree(iteration, open_list_add_count, *parent_map,
                                  closed_positions, scenerio,
                                  next_jpqv.predecessors, next_jpqv.positions,
                                  next_jpqv.heuristic, next_jpqv.distances,
                                  Super::kStartNodePredecessors,
                                  phase_name + "A* repair Static collide");
          }
          continue;
        }

        if (Super::OutsideOfPlanningBox(scenerio.needs_replans, next_positions,
                                        scenerio.grid_center,
                                        scenerio.replan_radius)) {
          next_jpqv.source = JPQVSource::OUT_OF_BOX;
          out_of_box_vertices->push_back(next_jpqv);
          if (!kProduction) {
            ++open_list_add_count;
            std::vector<std::array<Eigen::Vector2i, kRobotCount>>
                closed_positions;
            for (const auto& e : *closed_list) {
              closed_positions.push_back(e.first);
            }
            Super::DumpSearchTree(iteration, open_list_add_count, *parent_map,
                                  closed_positions, scenerio,
                                  next_jpqv.predecessors, next_jpqv.positions,
                                  next_jpqv.heuristic, next_jpqv.distances,
                                  Super::kStartNodePredecessors,
                                  phase_name + "A* repair out of planning box");
          }
          continue;
        }

        if (!Super::CheckStepCollides(top.positions, top.distances,
                                      next_jpqv.positions, next_jpqv.distances,
                                      scenerio.needs_replans,
                                      replan_start_times, *parent_map)) {
          if (!kProduction) {
            ++open_list_add_count;
            std::vector<std::array<Eigen::Vector2i, kRobotCount>>
                closed_positions;
            for (const auto& e : *closed_list) {
              closed_positions.push_back(e.first);
            }
            Super::DumpSearchTree(
                iteration, open_list_add_count, *parent_map, closed_positions,
                scenerio, next_jpqv.predecessors, next_jpqv.positions,
                next_jpqv.heuristic, next_jpqv.distances,
                Super::kStartNodePredecessors, phase_name + "A* repair Clear");
          }
          open_list->Push(next_jpqv);
        } else {
          if (!kProduction) {
            ++open_list_add_count;
            std::vector<std::array<Eigen::Vector2i, kRobotCount>>
                closed_positions;
            for (const auto& e : *closed_list) {
              closed_positions.push_back(e.first);
            }
            Super::DumpSearchTree(iteration, open_list_add_count, *parent_map,
                                  closed_positions, scenerio,
                                  next_jpqv.predecessors, next_jpqv.positions,
                                  next_jpqv.heuristic, next_jpqv.distances,
                                  Super::kStartNodePredecessors,
                                  phase_name + "A* repair Robot-robot collide");
          }
        }
      }
    }

    if (!kProduction) {
      const auto parent_find_result = parent_map->find(starts);
      if (parent_find_result == parent_map->end()) {
        LOG(FATAL) << "Starts not found in parent map!";
      }
      if (parent_find_result->second != Super::kStartNodePredecessors) {
        LOG(ERROR) << "Found predecessor:";
        for (const auto& e : parent_find_result->second) {
          LOG(ERROR) << e.x() << ", " << e.y();
        }
        LOG(FATAL) << "No proper start predecessor!";
      }
      const auto closed_find_result = closed_list->find(starts);
      if (closed_find_result == closed_list->end() ||
          closed_find_result->second != Distance(0, 0)) {
        LOG(FATAL) << "No proper closed value!";
      }
    }
  }

  ReplanResult<kRobotCount> StandardAStar(
      const size_t& iteration, size_t* counter,
      const ReplanScenerio<kRobotCount>& scenerio,
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& static_invalid_vertices,
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& dynamic_invalid_vertices,
      datastructures::VectorPriorityQueue<JointPriorityQueueVertex>* open_list,
      std::unordered_map<std::array<Eigen::Vector2i, kRobotCount>, Distance,
                         ArrayGridHasher<kRobotCount>>* closed_list,
      std::unordered_map<std::array<Eigen::Vector2i, kRobotCount>,
                         std::array<Eigen::Vector2i, kRobotCount>,
                         ArrayGridHasher<kRobotCount>>* parent_map,
      std::vector<JointPriorityQueueVertex>* out_of_box_vertices,
      const std::array<Eigen::Vector2i, kRobotCount>& starts,
      const std::array<Eigen::Vector2i, kRobotCount>& goals) {
    size_t& open_list_add_count = *counter;
    JointPriorityQueueVertex last_jpqv = open_list->Top();
    while (!open_list->Empty()) {
      last_jpqv = open_list->Top();
      const JointPriorityQueueVertex top = open_list->Top();
      open_list->Pop();
      const auto find_result = closed_list->find(top.positions);
      if (find_result != closed_list->end()) {
        continue;
      }
      (*closed_list)[top.positions] =
          array_util::SelectiveSumArray<kRobotCount>(
              top.distances, scenerio.needs_replans, Distance(0, 0));
      (*parent_map)[top.positions] = top.predecessors;

      if (goals == top.positions) {
        LOG(INFO) << "Unwinding path!";
        open_list->Push(top);
        return UnwindPath(*parent_map, starts, goals, scenerio.needs_replans);
      }

      const auto dynamic_cartesian_product =
          Super::CalculateDynamicCartesianProductNine(
              CalculateNeighbors(scenerio.needs_replans, goals, top),
              scenerio.needs_replans);
      for (const std::array<std::pair<Eigen::Vector2i, Distance>, kRobotCount>&
               neighbors : dynamic_cartesian_product) {
        std::array<Eigen::Vector2i, kRobotCount> next_positions =
            array_util::MakeArray<kRobotCount>(
                Eigen::Vector2i(kIntMaxHack, kIntMaxHack));
        std::array<Distance, kRobotCount> next_distances = top.distances;
        for (size_t robot_index = 0; robot_index < kRobotCount; ++robot_index) {
          next_positions[robot_index] = neighbors[robot_index].first;
          next_distances[robot_index] += neighbors[robot_index].second;
        }

        JointPriorityQueueVertex next_jpqv(
            scenerio.needs_replans, next_positions, next_distances,
            Super::JointHeuristic(scenerio.needs_replans, next_positions,
                                  goals),
            top.positions, array_util::AddToEachElement(top.path_indices, 1ul),
            JPQVSource::PREV_SEARCH_FILL);

        if (Super::OutsideOfField(scenerio.needs_replans, next_positions)) {
          if (!kProduction) {
            ++open_list_add_count;
            std::vector<std::array<Eigen::Vector2i, kRobotCount>>
                closed_positions;
            for (const auto& e : *closed_list) {
              closed_positions.push_back(e.first);
            }
            Super::DumpSearchTree(
                iteration, open_list_add_count, *parent_map, closed_positions,
                scenerio, next_jpqv.predecessors, next_jpqv.positions,
                next_jpqv.heuristic, next_jpqv.distances,
                Super::kStartNodePredecessors, "Outside field");
          }
          continue;
        }

        if (Super::ContainsInvalidVertices(
                static_invalid_vertices, dynamic_invalid_vertices,
                next_positions, scenerio.needs_replans)) {
          if (!kProduction) {
            ++open_list_add_count;
            std::vector<std::array<Eigen::Vector2i, kRobotCount>>
                closed_positions;
            for (const auto& e : *closed_list) {
              closed_positions.push_back(e.first);
            }
            Super::DumpSearchTree(
                iteration, open_list_add_count, *parent_map, closed_positions,
                scenerio, next_jpqv.predecessors, next_jpqv.positions,
                next_jpqv.heuristic, next_jpqv.distances,
                Super::kStartNodePredecessors, "Static collide");
          }
          continue;
        }

        if (Super::OutsideOfPlanningBox(scenerio.needs_replans, next_positions,
                                        scenerio.grid_center,
                                        scenerio.replan_radius)) {
          next_jpqv.source = JPQVSource::OUT_OF_BOX;
          out_of_box_vertices->push_back(next_jpqv);
          if (!kProduction) {
            ++open_list_add_count;
            std::vector<std::array<Eigen::Vector2i, kRobotCount>>
                closed_positions;
            for (const auto& e : *closed_list) {
              closed_positions.push_back(e.first);
            }
            Super::DumpSearchTree(iteration, open_list_add_count, *parent_map,
                                  closed_positions, scenerio,
                                  next_jpqv.predecessors, next_jpqv.positions,
                                  next_jpqv.heuristic, next_jpqv.distances,
                                  Super::kStartNodePredecessors, "Outside box");
          }
          continue;
        }

        if (!Super::CheckStepCollides(
                top.positions, top.distances, next_jpqv.positions,
                next_jpqv.distances, scenerio.needs_replans,
                scenerio.replan_start_times, *parent_map)) {
          if (!kProduction) {
            ++open_list_add_count;
            std::vector<std::array<Eigen::Vector2i, kRobotCount>>
                closed_positions;
            for (const auto& e : *closed_list) {
              closed_positions.push_back(e.first);
            }
            Super::DumpSearchTree(iteration, open_list_add_count, *parent_map,
                                  closed_positions, scenerio,
                                  next_jpqv.predecessors, next_jpqv.positions,
                                  next_jpqv.heuristic, next_jpqv.distances,
                                  Super::kStartNodePredecessors, "Clear");
          }
          open_list->Push(next_jpqv);
        } else {
          if (!kProduction) {
            ++open_list_add_count;
            std::vector<std::array<Eigen::Vector2i, kRobotCount>>
                closed_positions;
            for (const auto& e : *closed_list) {
              closed_positions.push_back(e.first);
            }
            Super::DumpSearchTree(
                iteration, open_list_add_count, *parent_map, closed_positions,
                scenerio, next_jpqv.predecessors, next_jpqv.positions,
                next_jpqv.heuristic, next_jpqv.distances,
                Super::kStartNodePredecessors, "Robot-robot collide");
          }
        }
      }
    }
    open_list->Push(last_jpqv);
    if (!kProduction) {
      LOG(FATAL) << "No replan found!";
    }
    return {};
  }

 public:
  ExpandingAStarEightGridRepairer(const float& grid_distance_between_vertices,
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

    // Initialize with the start state of the smallest scenerio.
    datastructures::VectorPriorityQueue<JointPriorityQueueVertex>
        prev_open_list;
    std::unordered_map<std::array<Eigen::Vector2i, kRobotCount>, Distance,
                       ArrayGridHasher<kRobotCount>>
        prev_closed_list(Super::planar_total_vertex_count_);
    std::unordered_map<std::array<Eigen::Vector2i, kRobotCount>,
                       std::array<Eigen::Vector2i, kRobotCount>,
                       ArrayGridHasher<kRobotCount>>
        prev_path_map;
    std::vector<JointPriorityQueueVertex> prev_out_of_box_vertices;

    // Prepare the previous datastructures for their first use.
    {
      const JointPriorityQueueVertex initial_start(
          last_finished_scenerio.needs_replans,
          Super::StripIgnoredData(last_finished_scenerio.replan_starts,
                                  last_finished_scenerio.needs_replans),
          array_util::MakeArray<kRobotCount>(Distance(0, 0)), Distance(0, 0),
          Super::kStartNodePredecessors,
          last_finished_scenerio.replan_starts_indices,
          JPQVSource::PREV_SEARCH_FILL);
      prev_open_list.Push(initial_start);
    }

    ReplanScenerio<kRobotCount> prev_scenerio = scenerio_list[0];

    for (size_t iteration = 0; iteration < scenerio_list.size(); ++iteration) {
      LOG(WARNING) << "Iteration " << iteration;
      const ReplanScenerio<kRobotCount>& scenerio = scenerio_list[iteration];
      LOG(INFO) << "Current scenerio replan starts";
      for (size_t i = 0; i < kRobotCount; ++i) {
        if (!scenerio.needs_replans[i]) {
          LOG(INFO) << i << ": skipped";
          continue;
        }
        LOG(INFO) << i << ": " << scenerio.replan_starts[i].x() << ", "
                  << scenerio.replan_starts[i].y();
      }
      LOG(INFO) << "Previous scenerio replan starts";
      for (size_t i = 0; i < kRobotCount; ++i) {
        if (!prev_scenerio.needs_replans[i]) {
          LOG(INFO) << i << ": skipped";
          continue;
        }
        LOG(INFO) << i << ": " << prev_scenerio.replan_starts[i].x() << ", "
                  << prev_scenerio.replan_starts[i].y();
      }
      Super::DumpReplanScenerio(iteration, scenerio, static_invalid_vertices,
                                dynamic_invalid_vertices);
      // Previous replan start and goal.
      const std::array<Eigen::Vector2i, kRobotCount>
          prev_stripped_replan_starts = Super::StripIgnoredData(
              prev_scenerio.replan_starts, prev_scenerio.needs_replans);
      const std::array<Eigen::Vector2i, kRobotCount> prev_stripped_replan_ends =
          Super::StripIgnoredData(prev_scenerio.replan_ends,
                                  prev_scenerio.needs_replans);

      // Current replan start and goal.
      const std::array<Eigen::Vector2i, kRobotCount> stripped_replan_starts =
          Super::StripIgnoredData(scenerio.replan_starts,
                                  scenerio.needs_replans);
      const std::array<Eigen::Vector2i, kRobotCount> stripped_replan_ends =
          Super::StripIgnoredData(scenerio.replan_ends, scenerio.needs_replans);

      // Distance between the previous and current starts.
      const std::array<Distance, kRobotCount> prev_start_to_start_distances =
          CalculatePathDistances(scenerio.individual_plans,
                                 scenerio.needs_replans,
                                 scenerio.replan_starts_indices,
                                 prev_scenerio.replan_starts_indices);
      // Verify that the stated distances between starts of successive scenerios
      // are sensible.
      if (!kProduction) {
        for (size_t i = 0; i < kRobotCount; ++i) {
          const Distance& d = prev_start_to_start_distances[i];
          if (!scenerio.needs_replans[i]) {
            if (d != Distance(0, 0)) {
              LOG(FATAL) << "Not zeroing distances!";
              continue;
            }
            if (d.ComponentsNegative()) {
              LOG(FATAL) << "Has negative components!";
            }

            if (prev_start_to_start_distances[i] !=
                (prev_scenerio.replan_start_times[i] -
                 scenerio.replan_start_times[i])) {
              LOG(FATAL) << "Distance deltas do not line up!";
            }
            LOG(INFO) << "Hello!";
          }
        }
      }

      std::unordered_map<std::array<Eigen::Vector2i, kRobotCount>,
                         std::array<Eigen::Vector2i, kRobotCount>,
                         ArrayGridHasher<kRobotCount>>
          path_map = prev_path_map;
      std::vector<JointPriorityQueueVertex> out_of_box_vertices;
      // Phase 1.1 open list initialization.
      datastructures::VectorPriorityQueue<JointPriorityQueueVertex> open_list(
          prev_open_list.GetVector(), prev_out_of_box_vertices);
      // Phase 1.1 closed list initialization.
      std::unordered_map<std::array<Eigen::Vector2i, kRobotCount>, Distance,
                         ArrayGridHasher<kRobotCount>>
          closed_list = prev_closed_list;

      // Verify that the previous open list has the required one or more
      // elements.
      if (!kProduction) {
        if (prev_open_list.Empty()) {
          LOG(FATAL) << "Previous open list empty!";
        }

        if (iteration > 0) {
          //           LOG(INFO) << "Verifying that path exists from old start
          //           to old goal!";
          const auto result =
              UnwindPath(path_map, prev_stripped_replan_starts,
                         prev_stripped_replan_ends, scenerio.needs_replans);
          //           LOG(INFO) << "Verified that path exists from old start to
          //           old goal!";
        }
      }

      size_t counter = 0;

      LOG(INFO) << "Open list size pre 1.1: " << open_list.Size();

      Super::DumpParentMap(iteration, 0, "pre11", path_map);

      // Perform Phase 1.1: expand the A* search tree from one in the previous
      // window to one in the current window while maintaining the start and
      // goal.
      AStarRepair("phase1.1 ", iteration, &counter, scenerio,
                  prev_scenerio.replan_start_times, static_invalid_vertices,
                  dynamic_invalid_vertices, &open_list, &closed_list, &path_map,
                  &out_of_box_vertices, prev_stripped_replan_starts,
                  prev_stripped_replan_ends, prev_open_list.Top());

      Super::DumpParentMap(iteration, 1, "post11", path_map);

      LOG(INFO) << "Open list size post 1.1: " << open_list.Size();

      if (!kProduction) {
        const auto find_result = path_map.find(prev_stripped_replan_starts);
        if (find_result == path_map.end()) {
          LOG(FATAL) << "No Phase 1.1 parent start found in the parent map.";
        }
        if (find_result->second != Super::kStartNodePredecessors) {
          LOG(FATAL) << "Start is not terminating!";
        }
      }

      // Distance extend the nodes for new start.
      for (auto& e : *open_list.GetMutableVector()) {
        e.UpdateDistances(array_util::AddArrayElements(
            e.distances, prev_start_to_start_distances));
        e.source = JPQVSource::OUT_OF_BOX;
      }
      open_list.RebuildHeap();

      // Distance extend the closed g-values for the new start.
      for (auto& e : closed_list) {
        e.second += array_util::SelectiveSumArray(prev_start_to_start_distances,
                                                  scenerio.needs_replans,
                                                  Distance(0, 0));
      }

      AddIndividualOptimalPaths(
          scenerio.individual_plans, scenerio.needs_replans,
          prev_scenerio.replan_starts_indices, scenerio.replan_starts_indices,
          &open_list, &path_map);

      auto distance_extended_prev_open_list_top = prev_open_list.Top();
      distance_extended_prev_open_list_top.UpdateDistances(
          array_util::AddArrayElements(
              distance_extended_prev_open_list_top.distances,
              prev_start_to_start_distances));

      LOG(INFO) << "Open list size pre 1.2: " << open_list.Size();

      Super::DumpParentMap(iteration, 2, "pre21", path_map);

      // Perform Phase 1.2: expand the A* search tree from the previous start to
      // the current start.
      AStarRepair("phase1.2 ", iteration, &counter, scenerio,
                  scenerio.replan_start_times, static_invalid_vertices,
                  dynamic_invalid_vertices, &open_list, &closed_list, &path_map,
                  &out_of_box_vertices, stripped_replan_starts,
                  prev_stripped_replan_ends,
                  distance_extended_prev_open_list_top);

      Super::DumpParentMap(iteration, 3, "post21", path_map);

      LOG(INFO) << "Open list size post 1.2: " << open_list.Size();

      if (!kProduction) {
        //         LOG(INFO) << "Verifying that path exists from new start to
        //         old start!";
        auto result =
            UnwindPath(path_map, stripped_replan_starts,
                       prev_stripped_replan_starts, scenerio.needs_replans);
        //         LOG(INFO) << "Verified that path exists from new start to old
        //         start!";

        if (iteration > 0) {
          //           LOG(INFO) << "Verifying that path exists from new start
          //           to old goal!";
          auto result =
              UnwindPath(path_map, stripped_replan_starts,
                         prev_stripped_replan_ends, scenerio.needs_replans);
          //           LOG(INFO) << "Verified that path exists from new start to
          //           old goal!";
        }
      }

      for (auto& e : *open_list.GetMutableVector()) {
        e.heuristic = Super::JointHeuristic(scenerio.needs_replans, e.positions,
                                            stripped_replan_ends);
      }
      open_list.RebuildHeap();

      // Try early termination.
      const auto find_goal = closed_list.find(stripped_replan_ends);
      if (find_goal != closed_list.end()) {
        LOG(INFO) << "Found goal already, short circuiting.";
        final_result = UnwindPath(path_map, stripped_replan_starts,
                                  stripped_replan_ends, scenerio.needs_replans);
      } else {
        LOG(INFO) << "Goal not already known, running A*.";
        final_result = StandardAStar(
            iteration, &counter, scenerio, static_invalid_vertices,
            dynamic_invalid_vertices, &open_list, &closed_list, &path_map,
            &out_of_box_vertices, stripped_replan_starts, stripped_replan_ends);
      }

      Super::DumpParentMap(iteration, 4, "postsearch", path_map);

      last_finished_scenerio = scenerio;
      prev_scenerio = scenerio;
      prev_out_of_box_vertices = out_of_box_vertices;
      prev_closed_list = closed_list;
      prev_open_list = open_list;
      prev_path_map = path_map;
    }

    //     LOG(INFO) << "Open list expansion count: " <<
    //     open_list_expansion_count;
    //     LOG(INFO) << "Collision checks: " << collision_check_count;
    return {final_result, last_finished_scenerio};
  }
};

}  // namespace repairer
}  // namespace repair
}  // namespace navigation

#endif  // SRC_NAVIGATION_REPAIR_EXPANDING_ASTAR_EIGHT_GRID_REPAIRER_H_
