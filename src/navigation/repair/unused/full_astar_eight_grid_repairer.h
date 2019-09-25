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

#ifndef SRC_NAVIGATION_REPAIR_UNUSED_FULL_ASTAR_EIGHT_GRID_REPAIRER_H_
#define SRC_NAVIGATION_REPAIR_UNUSED_FULL_ASTAR_EIGHT_GRID_REPAIRER_H_

#include <algorithm>
#include <array>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "navigation/repair/eight_grid_repairer.h"
#include "navigation/repair/high_dimensional_eight_grid.h"

namespace navigation {
namespace repair {
namespace repairer {

template <unsigned int kRobotCount>
class FullAStarEightGridRepairer : public EightGridRepairer<kRobotCount> {
  using Super = EightGridRepairer<kRobotCount>;

  ::navigation::high_dimensional::full_replan::HighDimensionalEightGrid<
      kRobotCount>
      full_astar_;

 public:
  FullAStarEightGridRepairer(const float& grid_distance_between_vertices,
                             const size_t& initial_repair_radius,
                             const size_t& repair_radius_step_size)
      : EightGridRepairer<kRobotCount>(grid_distance_between_vertices,
                                       initial_repair_radius,
                                       repair_radius_step_size),
        full_astar_(
            array_util::MakeArray<kRobotCount>(
                obstacle::ObstacleFlag::GetEmpty()),
            array_util::MakeArray<kRobotCount>(obstacle::SafetyMargin()),
            Super::grid_distance_between_vertices_) {}

  ReplanResult<kRobotCount> PerformReplan(
      const ReplanScenerio<kRobotCount>& scenerio,
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& static_invalid_vertices,
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& dynamic_invalid_vertices,
      const size_t& iteration_count) override {
    ReplanResult<kRobotCount> result;
    std::array<Eigen::Vector2f, kRobotCount> start_free_space;
    std::array<Eigen::Vector2f, kRobotCount> goal_free_space;
    for (size_t i = 0; i < kRobotCount; ++i) {
      start_free_space[i] =
          Super::GridVertexToFreeSpace(scenerio.replan_starts[i]);
      goal_free_space[i] =
          Super::GridVertexToFreeSpace(scenerio.replan_ends[i]);
    }
    logger::Logger logger;
    full_astar_.Update(Super::obstacles_array_, Super::safety_margin_array_,
                       start_free_space, goal_free_space, &logger);
    const auto plan_pair = full_astar_.Plan(&logger);
    const auto& plan_array = plan_pair.second;

    for (size_t i = 0; i < kRobotCount; ++i) {
      auto& integer_vector = result.fixed_plans[i];
      const auto& float_vector = plan_array[i];
      integer_vector.resize(float_vector.size());
      for (size_t j = 0; j < float_vector.size(); ++j) {
        integer_vector[j] = Super::FreeSpaceToGridVertex(float_vector[j]);
      }
    }

    for (size_t i = 0; i < kRobotCount; ++i) {
      const auto& plan = result.fixed_plans[i];
      if (!plan.empty()) {
        if (plan[0] != scenerio.replan_starts[i]) {
          LOG(FATAL) << "Improper start!";
        }
        if (plan[plan.size() - 1] != scenerio.replan_ends[i]) {
          LOG(FATAL) << "Improper end!";
        }
      } else {
        LOG(WARNING) << "Empty plan!";
      }
    }

    return result;
  }
};

}  // namespace repairer
}  // namespace repair
}  // namespace navigation

#endif  // SRC_NAVIGATION_REPAIR_UNUSED_FULL_ASTAR_EIGHT_GRID_REPAIRER_H_
