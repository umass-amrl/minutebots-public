// Copyright 2016 - 2018 slane@cs.umass.edu
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

#include "obstacles/obstacle.h"
#include "constants/constants.h"
#include "obstacles/integer_bounding_box.h"

namespace obstacle {
Obstacle::Obstacle(ObstacleType type)
    : bounding_box_(0, 0, static_cast<int>(std::ceil(kFieldXMax)),
                    static_cast<int>(std::ceil(kFieldYMax))),
      type_(type),
      is_enabled_(false) {}

bool Obstacle::LineCollision(const Eigen::Vector2f& line_start,
                             const Eigen::Vector2f& line_end, float margin,
                             IntegerBoundingBox line_bounding_box) const {
  if (bounding_box_.BoundingBoxCollision(line_bounding_box, margin)) {
    return LineCollision(line_start, line_end, margin);
  }
  return false;
}

ObstacleType Obstacle::GetType() const { return type_; }

const bool Obstacle::IsEnabled() const { return is_enabled_; }

void Obstacle::SetEnabled(const bool enabled) { is_enabled_ = enabled; }

}  // namespace obstacle
