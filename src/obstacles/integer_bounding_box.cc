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

#include "obstacles/integer_bounding_box.h"

#include "eigen3/Eigen/Core"

using Eigen::Vector2f;

namespace obstacle {

IntegerBoundingBox::IntegerBoundingBox(const int x, const int y,
                                       const int width, const int height)
    : x_(x),
      y_(y),
      width_(width),
      half_width_(width / 2),
      height_(height),
      half_height_(height / 2) {}

bool IntegerBoundingBox::PointCollision(const Vector2f& point,
                                        float margin) const {
  return (point.x() + margin > x_ - half_width_ &&
          point.x() - margin < x_ + half_width_ &&
          point.y() + margin > y_ - half_height_ &&
          point.y() - margin < y_ + half_height_);
}

bool IntegerBoundingBox::BoundingBoxCollision(IntegerBoundingBox other_box,
                                              float margin) const {
  float double_margin = 2.0f * margin;
  return (x_ < other_box.x_ + other_box.width_ + double_margin &&
          x_ + width_ + double_margin > other_box.x_ &&
          y_ < other_box.y_ + other_box.height_ + double_margin &&
          y_ + height_ + double_margin > other_box.y_);

  return true;
}

}  // namespace obstacle
