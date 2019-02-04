// Copyright 2016 - 2017 jaholtz@cs.umass.edu
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
#include <vector>
#include "zone/zone.h"
#include "math/geometry.h"
#include "math/math_util.h"

using geometry::Angle;
using math_util::AngleDiff;
using std::vector;

namespace zone {

std::map<std::string, const ZoneType> ZoneMap = {
  { "our_half", OUR_HALF },
  { "their_half", THEIR_HALF },
  { "full_field", FULL_FIELD},
  { "first_quad", FIRST_QUAD },
  { "second_quad", SECOND_QUAD },
  { "third_quad", THIRD_QUAD },
  { "fourth_quad", FOURTH_QUAD},
  { "midfield", MID_FIELD}
};

FieldZone::FieldZone(const ZoneType& type) {
  type_ = type;
  switch (type_) {
    case ZoneType::OUR_HALF :
      min_x_ = -kFieldXMax;
      max_x_ = 0;
      min_y_ = -kFieldYMax;
      max_y_ = kFieldYMax;
      break;
    case ZoneType::THEIR_HALF :
      min_x_ = 0;
      max_x_ = kFieldXMax;
      min_y_ = -kFieldYMax;
      max_y_ = kFieldYMax;
      break;
    case ZoneType::FULL_FIELD :
      min_x_ = -kFieldXMax;
      max_x_ = kFieldXMax;
      min_y_ = -kFieldYMax;
      max_y_ = kFieldYMax;
      break;
    case ZoneType::FIRST_QUAD :
      min_x_ = 0.0;
      max_x_ = kFieldXMax;
      min_y_ = 0.0;
      max_y_ = kFieldYMax;
      break;
    case ZoneType::SECOND_QUAD :
      min_x_ = -kFieldXMax;
      max_x_ = 0.0;
      min_y_ = 0.0;
      max_y_ = kFieldYMax;
      break;
    case ZoneType::THIRD_QUAD :
      min_x_ = -kFieldXMax;
      max_x_ = 0.0;
      min_y_ = -kFieldYMax;
      max_y_ = 0.0;
      break;
    case ZoneType::FOURTH_QUAD :
      min_x_ = 0.0;
      max_x_ = kFieldXMax;
      min_y_ = -kFieldYMax;
      max_y_ = 0.0;
      break;
    case ZoneType::MID_FIELD :
      min_x_ = -kFieldXMax/2;
      max_x_ = kFieldXMax/2;
      min_y_ = -kFieldYMax;
      max_y_ = kFieldYMax;
      break;
    default :
      break;
  }
}

bool FieldZone::IsInZone(Vector2f point, const float& margin) const {
  if ((point(0) < max_x_ - margin) && (point(0) > min_x_ + margin)
      && (point(1) < max_y_ - margin) && (point(1) > min_y_ + margin)) {
    return true;
  } else {
    return false;
  }
}

bool FieldZone::IsInZone(Vector2f point) const {
  if ((point(0) < max_x_) && (point(0) > min_x_)
    && (point(1) < max_y_) && (point(1) > min_y_)) {
    return true;
    } else {
      return false;
    }
}

bool FieldZone::CropToZone(const Vector2f& source,
                           const Vector2f& targetr,
                           const Vector2f& targetl,
                           Vector2f* new_target_r,
                           Vector2f* new_target_l,
                           float margin) const {
    if (!IsInZone(targetl, margin) || !IsInZone(targetr, margin)) {
    vector<Vector2f> new_targets;
    if (IsInZone(targetl, margin)) new_targets.push_back(targetl);
    if (IsInZone(targetr, margin)) new_targets.push_back(targetr);

    vector<Vector2f> zone_corners;
    zone_corners.push_back(Vector2f(min_x_ + margin, min_y_ + margin));
    zone_corners.push_back(Vector2f(max_x_ - margin, min_y_ + margin));
    zone_corners.push_back(Vector2f(max_x_ - margin, max_y_ - margin));
    zone_corners.push_back(Vector2f(min_x_ + margin, max_y_ - margin));

    // Add the first corner again for looping back to it when calculating
    // ray directions using the corners.
    zone_corners.push_back(Vector2f(min_x_ + margin, min_y_ + margin));

    vector<Vector2f> cropped_end_points;
    for (int i = 0; i < 4; i++) {
      Vector2f ray_dir = zone_corners[i + 1] - zone_corners[i];
      float squared_dist;
      Vector2f intersection_point;
      bool intersects = geometry::RayIntersect(
          zone_corners[i], ray_dir, targetr, targetl,
          &squared_dist, &intersection_point);

      if (intersects) {
        cropped_end_points.push_back(intersection_point);
      }
    }

    // If there are no intersections with the zone sides and both end points are
    // outside of the zone, then it means that the whole receiving region is
    // outside the zone and zero is returned as the open angle.
    if (!IsInZone(targetl, margin) && !IsInZone(targetr, margin) &&
        cropped_end_points.empty()) {
      return false;
    } else if (!cropped_end_points.empty()) {
      for (size_t k = 0; k < cropped_end_points.size(); k++) {
        new_targets.push_back(cropped_end_points[k]);
      }

      // The size of new_targets now should be 2
      if (new_targets.size() != 2) {
//         LOG(WARNING) << "The number of end points of the cropped line"
//                      << " segment does not equal 2!" << std::endl;
        return false;
      }

      // Find the left and right targets
      vector<float> target_angle(2);
      for (size_t k = 0; k < new_targets.size(); k++) {
        target_angle[k] = Angle<float>(new_targets[k] - source);
      }
      float angle_diff = AngleDiff(target_angle[0] , target_angle[1]);

      if (angle_diff >= 0.0) {
        *new_target_l = new_targets[0];
        *new_target_r = new_targets[1];
      } else {
        *new_target_l = new_targets[1];
        *new_target_r = new_targets[0];
      }
    }
  } else {
    *new_target_l = targetl;
    *new_target_r = targetr;
  }
  return true;
}



}  // namespace zone

