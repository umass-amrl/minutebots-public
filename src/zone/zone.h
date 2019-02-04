// Copyright 2017 jaholtz@cs.umass.edu
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

#ifndef SRC_ZONE_ZONE_H_
#define SRC_ZONE_ZONE_H_

#include <map>
#include <string>
#include "obstacles/obstacle_type.h"
#include "shared/common_includes.h"


namespace zone {
enum ZoneType {
  OUR_HALF,
  THEIR_HALF,
  FULL_FIELD,
  FIRST_QUAD,
  SECOND_QUAD,
  THIRD_QUAD,
  FOURTH_QUAD,
  MID_FIELD
};

extern std::map<std::string, const ZoneType> ZoneMap;

class FieldZone {
 public:
  explicit FieldZone(const ZoneType& type);
  bool IsInZone(Eigen::Vector2f point, const float& margin) const;
  bool IsInZone(Eigen::Vector2f point) const;

  // Crops the input line segment to fit in the zone. Returns false if the
  // line segment is completely outside the zone and true if at least some
  // part of the line segment is inside the zone.
  bool CropToZone(const Eigen::Vector2f& source,
                  const Eigen::Vector2f& targetr,
                  const Eigen::Vector2f& targetl,
                  Eigen::Vector2f* new_target_r,
                  Eigen::Vector2f* new_target_l,
                  float margin) const;
  ZoneType type_;
  float min_x_;
  float max_x_;
  float min_y_;
  float max_y_;
};

}  // namespace zone

#endif  // SRC_ZONE_ZONE_H_
