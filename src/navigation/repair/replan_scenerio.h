// Copyright 2018 kvedder@umass.edu
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

#ifndef SRC_NAVIGATION_REPAIR_REPLAN_SCENERIO_H_
#define SRC_NAVIGATION_REPAIR_REPLAN_SCENERIO_H_

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <algorithm>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "navigation/eight_grid.h"
#include "navigation/repair/eight_grid_distance.h"
#include "replan_scenerio.pb.h"
#include "util/array_util.h"
#include "util/serialization.h"

namespace navigation {
namespace repair {
namespace repairer {

// Padding to add on either end of the range of colliding vertices.
// Must be >=  1.
static constexpr size_t kIndexExpansionConstant = 1;

template <unsigned int kRobotCount>
struct ReplanScenerio {
  std::array<std::vector<Eigen::Vector2i>, kRobotCount> individual_plans;
  std::array<bool, kRobotCount> needs_replans;
  std::array<Eigen::Vector2i, kRobotCount> replan_starts;
  std::array<Distance, kRobotCount> replan_start_times;
  std::array<size_t, kRobotCount> replan_starts_indices;
  std::array<Eigen::Vector2i, kRobotCount> replan_ends;
  std::array<size_t, kRobotCount> replan_ends_indices;
  Eigen::Vector2i grid_center;
  size_t replan_radius;

  ReplanScenerio() = delete;

  inline Eigen::Vector2i Vector2fToVector2i(const Eigen::Vector2f& f) {
    return f.cast<int>();
  }

  ReplanScenerio(
      const std::array<std::vector<Eigen::Vector2i>, kRobotCount>&
          individual_plans,
      const std::array<std::vector<Distance>, kRobotCount>&
          individual_plan_times,
      const collision_checks::CollisionEvent<kRobotCount>& collision_event)
      : individual_plans(individual_plans),
        needs_replans(array_util::MakeArray<kRobotCount>(false)),
        replan_starts(array_util::MakeArray<kRobotCount>(
            Eigen::Vector2i(kIntMaxHack, kIntMaxHack))),
        replan_start_times(array_util::MakeArray<kRobotCount>(Distance())),
        replan_ends(array_util::MakeArray<kRobotCount>(
            Eigen::Vector2i(kIntMaxHack, kIntMaxHack))),
        grid_center(Vector2fToVector2i(collision_event.event_center)),
        replan_radius(kIntMaxHack) {
    int radius = 0;
    for (size_t i = 0; i < kRobotCount; ++i) {
      const auto& plan = individual_plans[i];
      const auto& path_data = collision_event.path_data_array[i];
      // Cannot use std::max with zero as potential for negative ends up being
      // massive positive.
      const size_t start_replan_index =
          (path_data.lower_index > (kIndexExpansionConstant - 1))
              ? path_data.lower_index - kIndexExpansionConstant
              : 0;
      const size_t end_replan_index = std::min(
          path_data.upper_index + kIndexExpansionConstant, plan.size() - 1);

      if (!kProduction) {
        if (start_replan_index >= individual_plan_times[i].size()) {
          LOG(FATAL) << "start_replan_index out of bounds!";
        }

        if (end_replan_index >= individual_plan_times[i].size()) {
          LOG(FATAL) << "end_replan_index out of bounds!";
        }
      }

      needs_replans[i] = collision_event.path_data_array[i].enabled;
      replan_starts[i] = plan[start_replan_index];
      replan_starts_indices[i] = start_replan_index;
      replan_start_times[i] = individual_plan_times[i][start_replan_index];
      replan_ends[i] = plan[end_replan_index];
      replan_ends_indices[i] = end_replan_index;
      if (needs_replans[i]) {
        const Eigen::Vector2i starts_delta = (grid_center - replan_starts[i]);
        const Eigen::Vector2i ends_delta = (grid_center - replan_ends[i]);
        radius = std::max({static_cast<int>(radius),
                           starts_delta.lpNorm<Eigen::Infinity>(),
                           ends_delta.lpNorm<Eigen::Infinity>()});
      }
    }
    replan_radius = radius;
  }

  ReplanScenerio(const std::array<std::vector<Eigen::Vector2i>, kRobotCount>&
                     individual_plans,
                 const std::array<bool, kRobotCount>& needs_replans,
                 const std::array<Eigen::Vector2i, kRobotCount>& replan_starts,
                 const std::array<Distance, kRobotCount>& replan_start_times,
                 const std::array<size_t, kRobotCount>& replan_starts_indices,
                 const std::array<Eigen::Vector2i, kRobotCount>& replan_ends,
                 const std::array<size_t, kRobotCount>& replan_ends_indices,
                 const Eigen::Vector2i& grid_center,
                 const size_t& replan_radius)
      : individual_plans(individual_plans),
        needs_replans(needs_replans),
        replan_starts(replan_starts),
        replan_start_times(replan_start_times),
        replan_starts_indices(replan_starts_indices),
        replan_ends(replan_ends),
        replan_ends_indices(replan_ends_indices),
        grid_center(grid_center),
        replan_radius(replan_radius) {}

  void Dump(
      const std::string& file_name,
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& static_invalid_vertices,
      const std::array<std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                                          eight_grid::EightGrid::GridHasher>,
                       kRobotCount>& dynamic_invalid_vertices) const {
    int file_descriptor =
        ::util::serialization::CreateOrEraseFileForWrite(file_name);
    MinuteBotsProto::ReplanScenerio replan_proto;

    replan_proto.set_radius(this->replan_radius);
    replan_proto.mutable_center()->set_x(this->grid_center.x());
    replan_proto.mutable_center()->set_y(this->grid_center.y());

    std::unordered_set<Eigen::Vector2i, eight_grid::EightGrid::GridHasher>
        invalid_positions;

    for (const auto& map : static_invalid_vertices) {
      for (const auto& position : map) {
        invalid_positions.insert(position.first);
      }
    }

    for (const auto& map : dynamic_invalid_vertices) {
      for (const auto& position : map) {
        invalid_positions.insert(position.first);
      }
    }

    // Setup the point obstacles.
    auto* point_obstacle_proto = replan_proto.mutable_point_obstacle();
    for (const auto& obstacle_point : invalid_positions) {
      auto* position2d_proto = point_obstacle_proto->add_point_list();
      position2d_proto->set_x(obstacle_point.x());
      position2d_proto->set_y(obstacle_point.y());
    }

    for (size_t i = 0; i < kRobotCount; ++i) {
      if (!this->needs_replans[i]) {
        continue;
      }
      auto* start_position_proto =
          replan_proto.mutable_start()->add_positions();
      auto* goal_position_proto = replan_proto.mutable_goal()->add_positions();
      start_position_proto->set_x(this->replan_starts[i].x());
      start_position_proto->set_y(this->replan_starts[i].y());
      goal_position_proto->set_x(this->replan_ends[i].x());
      goal_position_proto->set_y(this->replan_ends[i].y());
    }

    google::protobuf::io::FileOutputStream file_output(file_descriptor);
    file_output.SetCloseOnDelete(true);
    if (!google::protobuf::TextFormat::Print(replan_proto, &file_output)) {
      LOG(ERROR) << "Failed to write path proto to " << file_name;
    }
  }
};

}  // namespace repairer
}  // namespace repair
}  // namespace navigation

#endif  // SRC_NAVIGATION_REPAIR_REPLAN_SCENERIO_H_
