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

#ifndef SRC_NAVIGATION_REPAIR_MULTIPATH_COLLISION_CHECKS_H_
#define SRC_NAVIGATION_REPAIR_MULTIPATH_COLLISION_CHECKS_H_

#include <algorithm>
#include <vector>

#include "constants/constants.h"
#include "navigation/repair/eight_grid_distance.h"
#include "util/array_util.h"

namespace navigation {
namespace repair {
namespace collision_checks {

// Holds all information related to a cylinder colliding with another path.
struct CylinderPathCollisionSpan {
  bool has_collision;
  int start_index;
  int end_index;
  float time_center;

  size_t update_count;

  CylinderPathCollisionSpan()
      : has_collision(false),
        start_index(-1),
        end_index(-1),
        time_center(-1.0f),
        update_count(0) {}

  // Must be fed indices in monotonically increasing order.
  void UpdateIndex(const size_t& lower_index, const size_t& upper_index,
                   const float& additional_time_center) {
    ++update_count;
    time_center =
        static_cast<float>(update_count - 1) /
            static_cast<float>(update_count) * time_center +
        1.0f / static_cast<float>(update_count) * additional_time_center;
    if (!has_collision) {
      has_collision = true;
      start_index = lower_index;
      end_index = upper_index;
    } else {
      if (!kProduction) {
        if (upper_index < lower_index) {
          LOG(FATAL) << "upper_index < lower_index";
        }
        const int int_upper_index = static_cast<int>(upper_index);
        const int int_lower_index = static_cast<int>(lower_index);
        if (start_index > int_upper_index) {
          LOG(FATAL) << "Too small upper index!";
        }
        if (end_index > int_lower_index) {
          LOG(FATAL) << "Too small lower index";
        }
      }
      end_index = upper_index;
    }
  }
};

struct ContiguiousPathCollision {
  // Path 1 values.
  size_t path1_index;
  size_t path1_min;
  size_t path1_max;

  // Path 2 values.
  size_t path2_index;
  size_t path2_min;
  size_t path2_max;

  // General collision data.
  float time_center;
  size_t update_count;

  // Create default entries that indicate no collision.
  ContiguiousPathCollision() = delete;
  ContiguiousPathCollision(const size_t& current_path_index,
                           const size_t& lower_index, const size_t& upper_index,
                           const size_t& cylinder_collision_path_index,
                           const CylinderPathCollisionSpan& cylinder_collision)
      : path1_index(current_path_index),
        path1_min(lower_index),
        path1_max(upper_index),
        path2_index(cylinder_collision_path_index),
        path2_min(cylinder_collision.start_index),
        path2_max(cylinder_collision.end_index),
        time_center(cylinder_collision.time_center),
        update_count(cylinder_collision.update_count) {
    if (!kProduction) {
      if (!cylinder_collision.has_collision) {
        LOG(FATAL) << "No collision, yet trying to create one!";
      }
    }
  }

  void ExpandCollisionRange(
      const size_t& current_index,
      const CylinderPathCollisionSpan& cylinder_collision) {
    time_center =
        static_cast<float>(update_count) /
            static_cast<float>(update_count + cylinder_collision.update_count) *
            time_center +
        static_cast<float>(cylinder_collision.update_count) /
            static_cast<float>(update_count + cylinder_collision.update_count) *
            cylinder_collision.time_center;
    update_count += cylinder_collision.update_count;
    if (!kProduction) {
      if (current_index < path1_min || current_index < path1_max) {
        LOG(FATAL) << "Handed indices out of order!";
      }

      if (!cylinder_collision.has_collision) {
        LOG(FATAL) << "No collision, yet trying to create one!";
      }
    }
    path1_max = current_index;
    path2_min = std::min(path2_min,
                         static_cast<size_t>(cylinder_collision.start_index));
    path2_max =
        std::max(path2_max, static_cast<size_t>(cylinder_collision.end_index));
  }
};

template <unsigned int kRobotCount>
struct CollisionEvent {
  struct PathData {
    bool enabled;
    size_t lower_index;
    size_t upper_index;
    PathData() : enabled(false), lower_index(0), upper_index(0) {}
    void Update(const size_t& lower, const size_t& upper) {
      if (!enabled) {
        enabled = true;
        lower_index = lower;
        upper_index = upper;
      } else {
        lower_index = std::min(lower_index, lower);
        upper_index = std::max(upper_index, upper);
      }
    }

    size_t GetCenterIndex() const { return (lower_index + upper_index) / 2; }
  };
  const std::array<std::vector<Eigen::Vector2i>, kRobotCount>& paths;
  std::array<PathData, kRobotCount> path_data_array;
  Eigen::Vector2f event_center;
  float event_time;
  size_t merged_collision_segment_count;

  CollisionEvent() = delete;
  CollisionEvent(
      const std::array<std::vector<Eigen::Vector2i>, kRobotCount>& paths,
      const ContiguiousPathCollision& contiguious_collision)
      : paths(paths),
        path_data_array(array_util::MakeArray<kRobotCount>(PathData())),
        event_center(),
        event_time(contiguious_collision.time_center),
        merged_collision_segment_count(contiguious_collision.update_count) {
    path_data_array[contiguious_collision.path1_index].Update(
        contiguious_collision.path1_min, contiguious_collision.path1_max);
    path_data_array[contiguious_collision.path2_index].Update(
        contiguious_collision.path2_min, contiguious_collision.path2_max);
    event_center = CalculateContiguiousCollisionCenter(contiguious_collision);
  }

  Eigen::Vector2f CalculateContiguiousCollisionCenter(
      const ContiguiousPathCollision& contiguious_collision) {
    const Eigen::Vector2i& path1_center =
        paths[contiguious_collision.path1_index]
             [path_data_array[contiguious_collision.path1_index]
                  .GetCenterIndex()];
    const Eigen::Vector2i& path2_center =
        paths[contiguious_collision.path2_index]
             [path_data_array[contiguious_collision.path2_index]
                  .GetCenterIndex()];
    return (path1_center.cast<float>() + path2_center.cast<float>()) / 2.0f;
  }

  bool UpdateOrReject(const ContiguiousPathCollision& contiguious_collision) {
    const Eigen::Vector2f proposed_center =
        CalculateContiguiousCollisionCenter(contiguious_collision);
    const Eigen::Vector2f delta = (event_center - proposed_center);
    const float l1norm = delta.lpNorm<1>();
    // TODO(kvedder): Tune these magic numbers or, ideally, set them to
    // something less arbitrary.
    if (l1norm > 3.0f) {
      return false;
    }
    if (fabs(event_time - contiguious_collision.time_center) >
        2 * (kRobotRadius / kMaxRobotVelocity)) {
      return false;
    }

    path_data_array[contiguious_collision.path1_index].Update(
        contiguious_collision.path1_min, contiguious_collision.path1_max);
    path_data_array[contiguious_collision.path2_index].Update(
        contiguious_collision.path2_min, contiguious_collision.path2_max);

    const float existing_weight =
        static_cast<float>(merged_collision_segment_count) /
        static_cast<float>(merged_collision_segment_count +
                           contiguious_collision.update_count);
    const float new_weight =
        static_cast<float>(contiguious_collision.update_count) /
        static_cast<float>(merged_collision_segment_count +
                           contiguious_collision.update_count);

    event_center =
        existing_weight * event_center + new_weight * proposed_center;
    event_time = existing_weight * event_time +
                 new_weight * contiguious_collision.time_center;

    merged_collision_segment_count += contiguious_collision.update_count;
    return true;
  }
};

// Given a cylinder segment of two 3D paths, check to see if they collide.
bool PathCylinderSegmentCollides(
    const Eigen::Vector2i& c1_start, const Eigen::Vector2i& c1_end,
    const ::navigation::repair::repairer::Distance& c1_start_time,
    const ::navigation::repair::repairer::Distance& c1_end_time,
    const Eigen::Vector2i& c2_start, const Eigen::Vector2i& c2_end,
    const ::navigation::repair::repairer::Distance& c2_start_time,
    const ::navigation::repair::repairer::Distance& c2_end_time);

bool CylinderPathCollides(
    const Eigen::Vector2i& cylinder_start, const Eigen::Vector2i& cylinder_end,
    const ::navigation::repair::repairer::Distance& cylinder_start_time,
    const ::navigation::repair::repairer::Distance& cylinder_end_time,
    const std::vector<Eigen::Vector2i>& path_positions,
    const std::vector<::navigation::repair::repairer::Distance>& path_times);

// Calculates the path collision span for a single cylinder. We
// need not worry about multiple discontinuious collisions as a cylinder can
// collide with at most 3 other path cylinders, and must be contiguious.
CylinderPathCollisionSpan CalculateCylinderPathCollisionSpan(
    const Eigen::Vector2i& cylinder_start, const Eigen::Vector2i& cylinder_end,
    const ::navigation::repair::repairer::Distance& cylinder_start_time,
    const ::navigation::repair::repairer::Distance& cylinder_end_time,
    const std::vector<Eigen::Vector2i>& path_positions,
    const std::vector<::navigation::repair::repairer::Distance>& path_times);

std::vector<ContiguiousPathCollision> PathPathCollisionIndices(
    const std::size_t& current_path_index,
    const std::vector<Eigen::Vector2i>& current_path_positions,
    const std::vector<::navigation::repair::repairer::Distance>&
        current_path_times,
    const std::size_t& other_path_index,
    const std::vector<Eigen::Vector2i>& other_path_positions,
    const std::vector<::navigation::repair::repairer::Distance>&
        other_path_times);

bool SegmentsCollideWithSegments(
    const std::vector<Eigen::Vector2i>& c1_starts,
    const std::vector<Eigen::Vector2i>& c1_ends,
    const std::vector<::navigation::repair::repairer::Distance>& c1_start_times,
    const std::vector<::navigation::repair::repairer::Distance>& c1_end_times,
    const std::vector<Eigen::Vector2i>& c2_starts,
    const std::vector<Eigen::Vector2i>& c2_ends,
    const std::vector<::navigation::repair::repairer::Distance>& c2_start_times,
    const std::vector<::navigation::repair::repairer::Distance>& c2_end_times);

template <unsigned int kRobotCount>
bool SegmentsCollideWithSegments(
    const std::array<Eigen::Vector2i, kRobotCount>& c1_starts,
    const std::array<Eigen::Vector2i, kRobotCount>& c1_ends,
    const std::array<float, kRobotCount>& c1_start_times,
    const std::array<float, kRobotCount>& c1_end_times,
    const std::array<Eigen::Vector2i, kRobotCount>& c2_starts,
    const std::array<Eigen::Vector2i, kRobotCount>& c2_ends,
    const std::array<float, kRobotCount>& c2_start_times,
    const std::array<float, kRobotCount>& c2_end_times) {
  for (size_t outer_index = 0; outer_index < kRobotCount; ++outer_index) {
    const Eigen::Vector2i& outer_current_position = c1_starts[outer_index];
    const ::navigation::repair::repairer::Distance& outer_current_time =
        c1_start_times[outer_index];
    const Eigen::Vector2i& outer_next_position = c1_ends[outer_index];
    const ::navigation::repair::repairer::Distance& outer_next_time =
        c1_end_times[outer_index];
    for (size_t inner_index = 0; inner_index < kRobotCount; ++inner_index) {
      if (outer_index != inner_index) {
        const Eigen::Vector2i& inner_current_position = c2_starts[inner_index];
        const ::navigation::repair::repairer::Distance& inner_current_time =
            c2_start_times[inner_index];
        const Eigen::Vector2i& inner_next_position = c2_ends[inner_index];
        const ::navigation::repair::repairer::Distance& inner_next_time =
            c2_end_times[inner_index];
        if (collision_checks::PathCylinderSegmentCollides(
                outer_current_position, outer_next_position, outer_current_time,
                outer_next_time, inner_current_position, inner_next_position,
                inner_current_time, inner_next_time)) {
          return true;
        }
      }
    }
  }
  return false;
}

template <unsigned int kRobotCount>
bool SegmentsCollideWithSegments(
    const std::array<Eigen::Vector2i, kRobotCount>& c1_starts,
    const std::array<Eigen::Vector2i, kRobotCount>& c1_ends,
    const std::array<::navigation::repair::repairer::Distance, kRobotCount>&
        c1_start_times,
    const std::array<::navigation::repair::repairer::Distance, kRobotCount>&
        c1_end_times,
    const std::array<Eigen::Vector2i, kRobotCount>& c2_starts,
    const std::array<Eigen::Vector2i, kRobotCount>& c2_ends,
    const std::array<::navigation::repair::repairer::Distance, kRobotCount>&
        c2_start_times,
    const std::array<::navigation::repair::repairer::Distance, kRobotCount>&
        c2_end_times,
    const std::array<bool, kRobotCount>& should_evaluate) {
  for (size_t outer_index = 0; outer_index < kRobotCount; ++outer_index) {
    if (!should_evaluate[outer_index]) {
      continue;
    }
    const Eigen::Vector2i& outer_current_position = c1_starts[outer_index];
    const ::navigation::repair::repairer::Distance& outer_current_time =
        c1_start_times[outer_index];
    const Eigen::Vector2i& outer_next_position = c1_ends[outer_index];
    const ::navigation::repair::repairer::Distance& outer_next_time =
        c1_end_times[outer_index];

    if (!kProduction) {
      if (outer_next_time < outer_current_time) {
        LOG(FATAL) << "Times out of order";
      }
    }

    for (size_t inner_index = 0; inner_index < kRobotCount; ++inner_index) {
      if (should_evaluate[inner_index] && outer_index != inner_index) {
        const Eigen::Vector2i& inner_current_position = c2_starts[inner_index];
        const ::navigation::repair::repairer::Distance& inner_current_time =
            c2_start_times[inner_index];
        const Eigen::Vector2i& inner_next_position = c2_ends[inner_index];
        const ::navigation::repair::repairer::Distance& inner_next_time =
            c2_end_times[inner_index];

        if (!kProduction) {
          if (inner_next_time < inner_current_time) {
            LOG(FATAL) << "Times out of order";
          }
        }

        if (collision_checks::PathCylinderSegmentCollides(
                outer_current_position, outer_next_position, outer_current_time,
                outer_next_time, inner_current_position, inner_next_position,
                inner_current_time, inner_next_time)) {
          return true;
        }
      }
    }
  }
  return false;
}

template <unsigned int kRobotCount>
std::vector<CollisionEvent<kRobotCount>> CollisionEvents(
    const std::array<std::vector<Eigen::Vector2i>, kRobotCount>& paths,
    const std::array<std::vector<::navigation::repair::repairer::Distance>,
                     kRobotCount>& times) {
  std::vector<CollisionEvent<kRobotCount>> existing_collision_events;
  for (size_t outer_index = 0; outer_index < kRobotCount; ++outer_index) {
    const auto& outer_path = paths[outer_index];
    const auto& outer_time = times[outer_index];
    for (size_t inner_index = outer_index + 1; inner_index < kRobotCount;
         ++inner_index) {
      const auto& inner_path = paths[inner_index];
      const auto& inner_time = times[inner_index];
      const std::vector<ContiguiousPathCollision> contiguious_collisions =
          PathPathCollisionIndices(outer_index, outer_path, outer_time,
                                   inner_index, inner_path, inner_time);
      for (const ContiguiousPathCollision& contiguious_collision :
           contiguious_collisions) {
        bool updated_event = false;
        for (CollisionEvent<kRobotCount>& existing_collision_event :
             existing_collision_events) {
          if (existing_collision_event.UpdateOrReject(contiguious_collision)) {
            updated_event = true;
            break;
          }
        }
        if (!updated_event) {
          existing_collision_events.push_back({paths, contiguious_collision});
        }
      }
    }
  }
  return existing_collision_events;
}

}  // namespace collision_checks
}  // namespace repair
}  // namespace navigation

#endif  // SRC_NAVIGATION_REPAIR_MULTIPATH_COLLISION_CHECKS_H_
