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

#ifndef SRC_NAVIGATION_REPAIR_UNUSED_DSTAR_EIGHT_GRID_REPAIRER_H_
#define SRC_NAVIGATION_REPAIR_UNUSED_DSTAR_EIGHT_GRID_REPAIRER_H_

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
class DStarEightGridRepairer : public EightGridRepairer<kRobotCount> {
  using Super = EightGridRepairer<kRobotCount>;
  const std::array<Eigen::Vector2i, kRobotCount> kStartNodePredecessors =
      array_util::MakeArray<kRobotCount>(
          Eigen::Vector2i(kIntMaxHack, kIntMaxHack));

  static constexpr size_t kMaxSteps = 100000;

  struct DStarState {
    std::array<bool, kRobotCount> needs_replans;
    std::array<Eigen::Vector2i, kRobotCount> positions;
    // The 2D vector key mentioned in the paper.
    Eigen::Vector2f key;

    DStarState()
        : DStarState(
              array_util::MakeArray<kRobotCount>(false),
              array_util::MakeArray<kRobotCount>(Super::kIgnorablePosition)) {}

    DStarState(const std::array<bool, kRobotCount>& needs_replans,
               const std::array<Eigen::Vector2i, kRobotCount>& positions)
        : needs_replans(needs_replans),
          positions(positions),
          key(-1.0f, -1.0f) {}

    DStarState(const std::array<bool, kRobotCount>& needs_replans,
               const std::array<Eigen::Vector2i, kRobotCount>& positions,
               const Eigen::Vector2f& key)
        : needs_replans(needs_replans), positions(positions), key(key) {}

    bool operator==(const DStarState& other) const {
      for (size_t i = 0; i < kRobotCount; ++i) {
        if (needs_replans[i] && positions[i] != other.positions[i]) {
          return false;
        }
      }
      return true;
    }

    bool operator!=(const DStarState& other) const {
      for (size_t i = 0; i < kRobotCount; ++i) {
        if (needs_replans[i] && positions[i] != other.positions[i]) {
          return true;
        }
      }
      return false;
    }

    bool operator<(const DStarState& other) const {
      // If outside +/- kEpsilon range.
      if (key.x() + kEpsilon < other.key.x()) {
        return true;
      } else if (key.x() > other.key.x() + kEpsilon) {
        return false;
      }

      return key.y() < other.key.y();
    }
  };

  struct DStarStateHasher {
    size_t operator()(const DStarState& s) const {
      size_t result = 7;
      for (size_t i = 0; i < kRobotCount; ++i) {
        if (s.needs_replans[i]) {
          continue;
        }
        const Eigen::Vector2i& e = s.positions[i];
        result += e.dot(Eigen::Vector2i(
            Eigen::Vector2i(1, field_dimensions::kFieldLength)));
      }
      return result;
    }
  };

  struct StateInfo {
    float g, rhs, cost;
    StateInfo() : StateInfo(0.0f) {}
    explicit StateInfo(const float& cost) : g(0.0f), rhs(0.0f), cost(cost) {}
    StateInfo(const float& g, const float& rhs, const float& cost)
        : g(g), rhs(rhs), cost(cost) {}
  };

  size_t CalculateMinimumReplanRadius(
      const ReplanScenerio<kRobotCount>& scenerio) {
    int minimum_radius = 0;
    for (size_t i = 0; i < kRobotCount; ++i) {
      if (scenerio.needs_replans[i]) {
        const Eigen::Vector2i& start = scenerio.replan_starts[i];
        const Eigen::Vector2i& end = scenerio.replan_ends[i];
        const Eigen::Vector2i start_delta = (scenerio.grid_center - start);
        const Eigen::Vector2i end_delta = (scenerio.grid_center - end);
        minimum_radius =
            std::max({minimum_radius, start_delta.lpNorm<Eigen::Infinity>(),
                      end_delta.lpNorm<Eigen::Infinity>()});
      }
    }

    return std::max(static_cast<size_t>(minimum_radius),
                    Super::initial_repair_radius_);
  }

  bool ContainsInvalidVertices(
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& static_invalid_vertices,
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& dynamic_invalid_vertices,
      const std::array<Eigen::Vector2i, kRobotCount>& next_positions) {
    for (size_t i = 0; i < kRobotCount; ++i) {
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

  // Priority queue of cells which are inconsistent. Referred to as "U" in the
  // literature.
  std::priority_queue<DStarState, std::vector<DStarState>> open_list;
  // Hashmap of state to the key mentioned in the paper. This is used to throw
  // away no longer valid states when they are pulled from the open list. This
  // circumvents the need to remove said elements from the open_list as
  // specified on line 8.
  std::unordered_map<DStarState, Eigen::Vector2f, DStarStateHasher> open_hash;
  // Hashmap of state to the associated rhs and g data.
  std::unordered_map<DStarState, StateInfo, DStarStateHasher> cell_hash;
  DStarState start_state;
  DStarState goal_state;
  DStarState last_state;
  float k_m;
  const float unseen_state_cost;

  // ==========
  // DStar Functions
  // ==========

  bool ApproxEqual(const float& a, const float& b) {
    return (fabs(a - b) < kEpsilon);
  }

  float GetRHS(const DStarState& state) {
    if (state == goal_state) {
      return 0;
    }
    const auto find_result = cell_hash.find(state);
    if (find_result != cell_hash.end()) {
      return find_result->second.rhs;
    }
    return Super::JointHeuristic(state.needs_replans, state.positions,
                                 goal_state.positions);
  }

  bool ApproxEqual(const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
    const Eigen::Vector2f delta = (a - b);
    return delta.lpNorm<Eigen::Infinity>() < kEpsilon;
  }

  float GetG(const DStarState& state) {
    const auto find_result = cell_hash.find(state);
    if (find_result != cell_hash.end()) {
      return find_result->second.rhs;
    }
    return Super::JointHeuristic(state.needs_replans, state.positions,
                                 goal_state.positions);
  }

  void SetRHS(const DStarState& state, const float& rhs_value) {
    auto find_result = cell_hash.find(state);
    if (find_result != cell_hash.end()) {
      find_result->second.rhs = rhs_value;
    } else {
      const auto heuristic_val = Super::JointHeuristic(
          state.needs_replans, state.positions, goal_state.positions);
      cell_hash[state] = {heuristic_val, rhs_value, unseen_state_cost};
    }
  }

  void SetG(const DStarState& state, const float& g_value) {
    auto find_result = cell_hash.find(state);
    if (find_result != cell_hash.end()) {
      find_result->second.g = g_value;
    } else {
      const auto heuristic_val = Super::JointHeuristic(
          state.needs_replans, state.positions, goal_state.positions);
      cell_hash[state] = {g_value, heuristic_val, unseen_state_cost};
    }
  }

  // Procedure CalculateKey(s) from D* Lite Unoptimized Version modified to
  // return a copy of the given state with the key set, for implementation
  // reasons.
  DStarState CalculateAndSetKey(const DStarState& initial_state) {
    DStarState state = initial_state;

    const float g = GetG(state);
    const float rhs = GetRHS(state);

    // D* Lite Unoptimized Version Line 1.
    const float min_val = std::min(g, rhs);
    const float first_half =
        min_val + Super::JointHeuristic(state.needs_replans, state.positions,
                                        goal_state.positions) +
        k_m;
    const float& second_half = min_val;
    // Set key.
    state.key = {first_half, second_half};
    return state;
  }

  void Initialize(
      const std::array<bool, kRobotCount>& needs_replans,
      const std::array<Eigen::Vector2i, kRobotCount>& stripped_starts,
      const std::array<Eigen::Vector2i, kRobotCount>& stripped_ends) {
    // D* Lite Unoptimized Version Line 2.
    open_list = std::priority_queue<DStarState, std::vector<DStarState>>();

    // D* Lite Unoptimized Version Line 3.
    k_m = 0;

    // D* Lite Unoptimized Version Line 4. Sets the values not to infinity, but
    // to the heuristic value if they cannot be found in the table. This is
    // orchestrated by the GetG() and GetRHS() functions.
    cell_hash.clear();
    open_hash.clear();

    // Housekeeping, not listed in the algorithm.
    start_state.positions = stripped_starts;
    goal_state.positions = stripped_ends;

    // D* Lite Unoptimized Version Line 21. Performed at the end of Initialize()
    // for ease of implementation; should not impact algorithm.
    last_state = start_state;

    // D* Lite Unoptimized Version Line 5.
    StateInfo state_info(unseen_state_cost);
    cell_hash[goal_state] = state_info;

    // D* Lite Unoptimized Version Line 6.
    open_list.push(CalculateAndSetKey(goal_state));

    //     const float heuristic_val = Super::JointHeuristic(
    //         needs_replans, start_state.positions, goal_state.positions);
    //     state_info.g = heuristic_val;
    //     state_info.rhs = heuristic_val;
    //     cell_hash[start_state] = state_info;
  }

  std::vector<std::array<std::pair<Eigen::Vector2i, float>, kRobotCount>>
  GetSuccessors(const DStarState& state) {
    std::array<std::array<std::pair<Eigen::Vector2i, float>, 8>, kRobotCount>
        neighbors;
    for (size_t i = 0; i < kRobotCount; ++i) {
      if (state.needs_replans[i]) {
        neighbors[i] = Super::GetNeighbors(state.positions[i]);
      } else {
        neighbors[i] = Super::kIgnorableNeighbors;
      }
    }
    return Super::CalculateDynamicCartesianProduct(neighbors,
                                                   state.needs_replans);
  }

  inline std::vector<std::array<std::pair<Eigen::Vector2i, float>, kRobotCount>>
  GetPredecessors(const DStarState& state) {
    return GetSuccessors(state);
  }

  void UpdateVertex(const DStarState& state) {
    // D* Lite Unoptimized Version Line 7.
    if (state != goal_state) {
      // D* Lite Unoptimized Version Line 7. This line asks for the minumum sum
      // of costs to move to a valid neighbor plus the distance of that neighbor
      // to the goal. This cannot be calculated in constant time because not all
      // neighbors are going to be valid, and thus we must instead resign
      // ourselves to enumerating all successors.
      float best_rhs_value = std::numeric_limits<float>::max();
      std::array<Eigen::Vector2i, kRobotCount> successor_positions;
      for (const std::array<std::pair<Eigen::Vector2i, float>, kRobotCount>&
               successor_positions_costs : GetSuccessors(state)) {
        float cost = 0;
        for (size_t i = 0; i < kRobotCount; ++i) {
          successor_positions[i] = successor_positions_costs[i].first;
          cost += successor_positions_costs[i].second;
        }
        const float g_value = GetG({state.needs_replans, successor_positions});
        const float sum = g_value + cost;
        best_rhs_value = std::min(best_rhs_value, sum);
      }
      SetRHS(state, best_rhs_value);
    }

    // D* Lite Unoptimized Version Line 8 IGNORED. The key for the associated
    // state is now going to be out of date, which will be noted as it will be
    // different than the value inside of the open_hash table.

    // D* Lite Unoptimized Version Line 9.
    if (!ApproxEqual(GetG(state), GetRHS(state))) {
      const DStarState updated_state = CalculateAndSetKey(state);
      open_list.push(updated_state);
      // This updates the key in the open_hash table such that states pulled
      // from the open_list later which have different keys will be thrown away.
      open_hash[updated_state] = updated_state.key;
    }
  }

  enum ComputeStatus { SUCCESS, OPENLIST_EMPTY, EXCEED_MAX_ITERATIONS };

  bool IsStillValidState(const DStarState& state) {
    const auto find_result = open_hash.find(state);
    if (find_result != open_hash.end()) {
      return ApproxEqual(state.key, find_result->second);
    }
    return false;
  }

  ComputeStatus ComputeShortestPath(
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& static_invalid_vertices,
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& dynamic_invalid_vertices) {
    for (size_t iteration_count = 0; !open_hash.empty(); ++iteration_count) {
      if (iteration_count > kMaxSteps) {
        return EXCEED_MAX_ITERATIONS;
      }

      // Throws away all no longer valid vertices from the open_list. This is a
      // byproduct of our non-removal of these vertices from UpdateVertex()'s
      // Line 8.
      DStarState top_state = open_list.top();
      while (!IsStillValidState(top_state)) {
        open_list.pop();
        if (open_hash.empty()) {
          return OPENLIST_EMPTY;
        }
        top_state = open_list.top();
      }
      open_list.pop();

      // Recalculates the key of start state, as used in line 10.
      start_state = CalculateAndSetKey(start_state);
      // Inverse logic of D* Lite Unoptimized Version Line 10.
      if (!(top_state < start_state) &&
          ApproxEqual(GetRHS(start_state), GetG(start_state))) {
        return SUCCESS;
      }

      // This state, having been removed from the open_list in a valid manner,
      // ought also be removed frpm the open_hash.
      open_hash.erase(open_hash.find(top_state));

      const DStarState recalculated_top_state = CalculateAndSetKey(top_state);
      // D* Lite Unoptimized Version Line 13.
      if (top_state < recalculated_top_state) {
        // D* Lite Unoptimized Version Line 14.
        open_list.push(recalculated_top_state);
      } else {
        const float g = GetG(recalculated_top_state);
        const float rhs = GetRHS(recalculated_top_state);
        // D* Lite Unoptimized Version Line 15.
        if (g > rhs) {
          // D* Lite Unoptimized Version Line 16.
          SetG(recalculated_top_state, rhs);
          // D* Lite Unoptimized Version Line 18.
        } else {
          // D* Lite Unoptimized Version Line 19.
          SetG(recalculated_top_state, std::numeric_limits<float>::max());
          // Some of D* Lite Unoptimized Version Line 20.
          UpdateVertex(recalculated_top_state);
        }

        // Common code between D* Lite Unoptimized Version Line 17 and Line 20.
        std::array<Eigen::Vector2i, kRobotCount> predecessor_positions;
        for (const std::array<std::pair<Eigen::Vector2i, float>, kRobotCount>&
                 predecessor_positions_costs :
             GetPredecessors(recalculated_top_state)) {
          float cost = 0;
          for (size_t i = 0; i < kRobotCount; ++i) {
            predecessor_positions[i] = predecessor_positions_costs[i].first;
            cost += predecessor_positions_costs[i].second;
          }
          if (!ContainsInvalidVertices(static_invalid_vertices,
                                       dynamic_invalid_vertices,
                                       predecessor_positions)) {
            UpdateVertex(
                {recalculated_top_state.needs_replans, predecessor_positions});
          }
        }
      }
    }
    return OPENLIST_EMPTY;
  }

 public:
  DStarEightGridRepairer(const float& grid_distance_between_vertices,
                         const size_t& initial_repair_radius,
                         const size_t& repair_radius_step_size)
      : EightGridRepairer<kRobotCount>(grid_distance_between_vertices,
                                       initial_repair_radius,
                                       repair_radius_step_size),
        unseen_state_cost(Super::grid_distance_between_vertices_) {}

  ReplanResult<kRobotCount> PerformReplan(
      const ReplanScenerio<kRobotCount>& scenerio,
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& static_invalid_vertices,
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& dynamic_invalid_vertices,
      const size_t& iteration_count) override {
    // D* Lite Unoptimized Version Line 22.
    Initialize(
        scenerio.needs_replans,
        Super::StripIgnoredData(scenerio.replan_starts, scenerio.needs_replans),
        Super::StripIgnoredData(scenerio.replan_ends, scenerio.needs_replans));

    const size_t minimum_replan_radius = CalculateMinimumReplanRadius(scenerio);
    LOG(INFO) << "Minumim replan radius: " << minimum_replan_radius;
    // D* Lite Unoptimized Version Line 23.
    switch (ComputeShortestPath(static_invalid_vertices,
                                dynamic_invalid_vertices)) {
      case SUCCESS: {
        LOG(INFO) << "Success!";
      } break;
      case OPENLIST_EMPTY: {
        LOG(INFO) << "Openlist empty!";
      } break;
      case EXCEED_MAX_ITERATIONS: {
        LOG(INFO) << "Exceed max iterations!";
        if (!kProduction) {
          LOG(FATAL) << "Quitting!";
        }
      } break;
      default: {
        if (!kProduction) {
          LOG(FATAL) << "Default!";
        }
      }
    }

    //     while ()

    //     for (size_t replan_step = 0; replan_step < iteration_count;
    //     ++replan_step) {
    //       const size_t replan_radius =
    //           minimum_replan_radius + Super::repair_radius_step_size_ *
    //           replan_step;
    //       LOG(INFO) << "Replan radius: " << replan_radius;
    //     }

    return {};
  }
};

}  // namespace repairer
}  // namespace repair
}  // namespace navigation

#endif  // SRC_NAVIGATION_REPAIR_UNUSED_DSTAR_EIGHT_GRID_REPAIRER_H_
