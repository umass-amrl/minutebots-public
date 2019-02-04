// Copyright 2017 slane@cs.umass.edu
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

#ifndef SRC_OBSTACLES_INTEGER_BOUNDING_BOX_H_
#define SRC_OBSTACLES_INTEGER_BOUNDING_BOX_H_

#include "eigen3/Eigen/Core"

namespace obstacle {
class IntegerBoundingBox {
 public:
  IntegerBoundingBox() = delete;
  IntegerBoundingBox(const int x, const int y, const int width,
                     const int height);

  bool PointCollision(const Eigen::Vector2f& point, float margin) const;

  bool BoundingBoxCollision(IntegerBoundingBox other_box, float margin) const;

  int x_;
  int y_;
  int width_;
  int half_width_;
  int height_;
  int half_height_;
};
}  // namespace obstacle

#endif  // SRC_OBSTACLES_INTEGER_BOUNDING_BOX_H_
