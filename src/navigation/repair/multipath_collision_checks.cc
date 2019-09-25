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
#include "navigation/repair/multipath_collision_checks.h"

#include <algorithm>
#include <vector>

namespace navigation {
namespace repair {
namespace collision_checks {

// Given a cylinder segment of two 3D paths, check to see if they collide.
bool PathCylinderSegmentCollides(
    const Eigen::Vector2i& c1_start, const Eigen::Vector2i& c1_end,
    const ::navigation::repair::repairer::Distance& c1_start_time,
    const ::navigation::repair::repairer::Distance& c1_end_time,
    const Eigen::Vector2i& c2_start, const Eigen::Vector2i& c2_end,
    const ::navigation::repair::repairer::Distance& c2_start_time,
    const ::navigation::repair::repairer::Distance& c2_end_time) {
  constexpr bool kAgressiveChecks = false;
  const Eigen::Vector2i c1_delta = (c1_start - c1_end);
  const Eigen::Vector2i c2_delta = (c2_start - c2_end);
  if (!kProduction) {
    // Check to ensure that these could come from a real path.
    const auto c1_delta_linf = c1_delta.lpNorm<Eigen::Infinity>();
    const auto c2_delta_linf = c2_delta.lpNorm<Eigen::Infinity>();
    if (c1_delta_linf > 1) {
      LOG(FATAL) << "c1 LInfinity Norm is greater than 1 (" << c1_delta_linf
                 << ")";
    }
    if (c2_delta_linf > 1) {
      LOG(FATAL) << "c2 LInfinity Norm is greater than 1 (" << c2_delta_linf
                 << ")";
    }

    if (c1_end_time < c1_start_time) {
      LOG(INFO) << "(" << c2_start.x() << ", " << c2_start.y() << ") -> ("
                << c2_end.x() << ", " << c2_end.y() << ")";
      LOG(FATAL) << "C1 End time < start time";
    }

    if (c2_end_time < c2_start_time) {
      LOG(INFO) << "(" << c2_start.x() << ", " << c2_start.y() << ") -> ("
                << c2_end.x() << ", " << c2_end.y() << ")";
      LOG(FATAL) << "C1 End time < start time";
    }

    if (c1_start_time < ::navigation::repair::repairer::Distance(0, 0)) {
      LOG(FATAL) << "c1_start negative";
    }

    if (c1_end_time < ::navigation::repair::repairer::Distance(0, 0)) {
      LOG(FATAL) << "c1_end negative";
    }

    if (c2_start_time < ::navigation::repair::repairer::Distance(0, 0)) {
      LOG(FATAL) << "c2_start negative (" << c2_start_time.straight_count
                 << ", " << c2_start_time.angle_count << ")";
    }

    if (c2_end_time < ::navigation::repair::repairer::Distance(0, 0)) {
      LOG(FATAL) << "c2_end negative";
    }
  }

  // Cannot collide if the cylinders do not occupy any of the same time slice.
  if ((c1_end_time < c2_start_time) || (c2_end_time < c1_start_time)) {
    return false;
  }

  if (c1_delta.lpNorm<1>() == 0 && c1_start != c2_start && c1_start != c2_end) {
    return false;
  }

  if (c2_delta.lpNorm<1>() == 0 && c2_start != c1_start && c2_start != c1_end) {
    return false;
  }

  const Eigen::Vector2i start_delta = (c1_start - c2_start);
  const Eigen::Vector2i end_delta = (c1_end - c2_end);
  // If starts and ends too far away to possibly interact, then they cannot
  // collide.
  if (start_delta.lpNorm<Eigen::Infinity>() > 2 &&
      end_delta.lpNorm<Eigen::Infinity>() > 2) {
    return false;
  }

  // TODO(kvedder): Add sound + complete checks. These currently are complete
  // but they are willing to reject potentially valid cylinder configurations to
  // ensure invalid ones are not accepted.

  if (c1_start == c2_start) {
    return true;
  }
  if (c1_end == c2_end) {
    return true;
  }

  // Reject swapping of positions OR ending another's start. This is
  // overzealous, but never misses a collision.
  if (c1_start == c2_end) {
    return true;
  }
  if (c2_start == c1_end) {
    return true;
  }

  const Eigen::Vector2i c1start_c2end_delta = (c1_start - c2_end);
  const Eigen::Vector2i c2start_c1end_delta = (c2_start - c1_end);

  // Reject X configuration or opposite direction diagonals.
  if (c1_delta.lpNorm<1>() == 2 && c2_delta.lpNorm<1>() == 2) {
    if (start_delta.lpNorm<Eigen::Infinity>() == 1 &&
        end_delta.lpNorm<Eigen::Infinity>() == 1) {
      return true;
    } else if (c1start_c2end_delta.lpNorm<Eigen::Infinity>() == 1 &&
               c2start_c1end_delta.lpNorm<Eigen::Infinity>() == 1) {
      return true;
    }
  }

  // These checks are technically correct in that they will stop cases where
  // robots might glance off one another if they partake in a half V shape, but
  // it also is very overzealous in its rejections. For example, these checks
  // were causing no valid solutions to be found in the constrained U planning
  // problem.
  if (kAgressiveChecks) {
    if (c1_delta.lpNorm<1>() == 2) {
      if (start_delta.lpNorm<1>() == 1 &&
          c2start_c1end_delta.lpNorm<1>() == 1) {
        return true;
      }
      if (end_delta.lpNorm<1>() == 1 && c1start_c2end_delta.lpNorm<1>() == 1) {
        return true;
      }
    }

    if (c2_delta.lpNorm<1>() == 2) {
      if (start_delta.lpNorm<1>() == 1 &&
          c1start_c2end_delta.lpNorm<1>() == 1) {
        return true;
      }
      if (end_delta.lpNorm<1>() == 1 && c2start_c1end_delta.lpNorm<1>() == 1) {
        return true;
      }
    }
  }

  return false;
}

bool CylinderPathCollides(
    const Eigen::Vector2i& cylinder_start, const Eigen::Vector2i& cylinder_end,
    const ::navigation::repair::repairer::Distance& cylinder_start_time,
    const ::navigation::repair::repairer::Distance& cylinder_end_time,
    const std::vector<Eigen::Vector2i>& path_positions,
    const std::vector<::navigation::repair::repairer::Distance>& path_times) {
  if (!kProduction) {
    if (path_positions.size() != path_times.size()) {
      LOG(FATAL) << "Path positions and times vector are not the same length!";
    }
  }

  // Traverses the path in reverse direction as most collisions happen near the
  // end of the list.
  for (size_t i = 1; i < path_positions.size(); ++i) {
    const size_t index = path_positions.size() - i - 1;
    const Eigen::Vector2i& path_cylinder_start = path_positions[index - 1];
    const ::navigation::repair::repairer::Distance& path_cylinder_start_time =
        path_times[index - 1];
    const Eigen::Vector2i& path_cylinder_end = path_positions[index];
    const ::navigation::repair::repairer::Distance& path_cylinder_end_time =
        path_times[index];

    if (PathCylinderSegmentCollides(
            cylinder_start, cylinder_end, cylinder_start_time,
            cylinder_end_time, path_cylinder_start, path_cylinder_end,
            path_cylinder_start_time, path_cylinder_end_time)) {
      return true;
    }
  }
  return false;
}

// Calculates the path collision span for a single cylinder. We
// need not worry about multiple discontinuious collisions as a cylinder can
// collide with at most 3 other path cylinders, and must be contiguious.
CylinderPathCollisionSpan CalculateCylinderPathCollisionSpan(
    const Eigen::Vector2i& cylinder_start, const Eigen::Vector2i& cylinder_end,
    const ::navigation::repair::repairer::Distance& cylinder_start_time,
    const ::navigation::repair::repairer::Distance& cylinder_end_time,
    const std::vector<Eigen::Vector2i>& path_positions,
    const std::vector<::navigation::repair::repairer::Distance>& path_times) {
  constexpr bool kDebug = false;
  if (!kProduction) {
    if (path_positions.size() != path_times.size()) {
      LOG(FATAL) << "Path positions and times vector are not the same length!";
    }
  }

  CylinderPathCollisionSpan indices;
  for (size_t i = 1; i < path_positions.size(); ++i) {
    const Eigen::Vector2i& path_cylinder_start = path_positions[i - 1];
    const ::navigation::repair::repairer::Distance& path_cylinder_start_time =
        path_times[i - 1];
    const Eigen::Vector2i& path_cylinder_end = path_positions[i];
    const ::navigation::repair::repairer::Distance& path_cylinder_end_time =
        path_times[i];

    if (kDebug) {
      LOG(INFO) << "Current (" << cylinder_start.x() << ", "
                << cylinder_start.y() << ") @ "
                << cylinder_start_time.straight_count << ", "
                << cylinder_start_time.angle_count << " -> ("
                << cylinder_end.x() << ", " << cylinder_end.y() << ") @ "
                << cylinder_end_time.straight_count << ", "
                << cylinder_end_time.angle_count;
      LOG(INFO) << "Path (" << path_cylinder_start.x() << ", "
                << path_cylinder_start.y() << ") @ "
                << path_cylinder_start_time.straight_count << ", "
                << path_cylinder_start_time.angle_count << " -> ("
                << path_cylinder_end.x() << ", " << path_cylinder_end.y()
                << ") @ " << path_cylinder_end_time.straight_count << ", "
                << path_cylinder_end_time.angle_count;
    }

    if (PathCylinderSegmentCollides(
            cylinder_start, cylinder_end, cylinder_start_time,
            cylinder_end_time, path_cylinder_start, path_cylinder_end,
            path_cylinder_start_time, path_cylinder_end_time)) {
      indices.UpdateIndex(
          i - 1, i, path_cylinder_start_time.AverageWith(path_cylinder_end_time)
                        .ToFloat(1.0f));
    }
  }
  return indices;
}

std::vector<ContiguiousPathCollision> PathPathCollisionIndices(
    const size_t& current_path_index,
    const std::vector<Eigen::Vector2i>& current_path_positions,
    const std::vector<::navigation::repair::repairer::Distance>&
        current_path_times,
    const size_t& other_path_index,
    const std::vector<Eigen::Vector2i>& other_path_positions,
    const std::vector<::navigation::repair::repairer::Distance>&
        other_path_times) {
  if (!kProduction) {
    if (current_path_positions.size() != current_path_times.size()) {
      LOG(FATAL) << "Current path vs times sizes do not match!";
    }
    if (other_path_positions.size() != other_path_times.size()) {
      LOG(FATAL) << "Other path vs times sizes do not match!";
    }
  }

  if (current_path_positions.empty() || other_path_positions.empty()) {
    return {};
  }

  std::vector<ContiguiousPathCollision> path_collisions;
  Eigen::Vector2i current_cylinder_start = current_path_positions[0];
  ::navigation::repair::repairer::Distance current_cylinder_start_time =
      current_path_times[0];
  bool create_new_collision = true;
  for (size_t i = 1; i < current_path_positions.size(); ++i) {
    const Eigen::Vector2i& current_cylinder_end = current_path_positions[i];
    const ::navigation::repair::repairer::Distance& current_cylinder_end_time =
        current_path_times[i];
    const CylinderPathCollisionSpan cylinder_collision =
        CalculateCylinderPathCollisionSpan(
            current_cylinder_start, current_cylinder_end,
            current_cylinder_start_time, current_cylinder_end_time,
            other_path_positions, other_path_times);

    if (cylinder_collision.has_collision) {
      if (create_new_collision) {
        path_collisions.push_back({current_path_index, i - 1, i,
                                   other_path_index, cylinder_collision});
        create_new_collision = false;
      } else {
        path_collisions[path_collisions.size() - 1].ExpandCollisionRange(
            i, cylinder_collision);
      }
    } else {
      create_new_collision = true;
    }

    current_cylinder_start = current_cylinder_end;
    current_cylinder_start_time = current_cylinder_end_time;
  }

  return path_collisions;
}

bool SegmentsCollideWithSegments(
    const std::vector<Eigen::Vector2i>& c1_starts,
    const std::vector<Eigen::Vector2i>& c1_ends,
    const std::vector<::navigation::repair::repairer::Distance>& c1_start_times,
    const std::vector<::navigation::repair::repairer::Distance>& c1_end_times,
    const std::vector<Eigen::Vector2i>& c2_starts,
    const std::vector<Eigen::Vector2i>& c2_ends,
    const std::vector<::navigation::repair::repairer::Distance>& c2_start_times,
    const std::vector<::navigation::repair::repairer::Distance>& c2_end_times) {
  if (!kProduction) {
    if (c1_starts.size() != c1_ends.size() ||
        c1_starts.size() != c1_start_times.size() ||
        c1_starts.size() != c1_end_times.size() ||
        c1_starts.size() != c2_starts.size() ||
        c1_starts.size() != c2_ends.size() ||
        c1_starts.size() != c2_start_times.size() ||
        c1_starts.size() != c2_end_times.size()) {
      LOG(FATAL) << "Sizes do not line up!";
    }
  }

  for (size_t outer_index = 0; outer_index < c1_starts.size(); ++outer_index) {
    const Eigen::Vector2i& outer_current_position = c1_starts[outer_index];
    const ::navigation::repair::repairer::Distance& outer_current_time =
        c1_start_times[outer_index];
    const Eigen::Vector2i& outer_next_position = c1_ends[outer_index];
    const ::navigation::repair::repairer::Distance& outer_next_time =
        c1_end_times[outer_index];
    for (size_t inner_index = 0; inner_index < c2_starts.size();
         ++inner_index) {
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

}  // namespace collision_checks
}  // namespace repair
}  // namespace navigation
