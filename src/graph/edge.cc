// Copyright 2017 kvedder@umass.edu
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
#include "graph/edge.h"

#include <algorithm>

namespace graph {

Edge::Edge(const bool is_valid, const float weight, const VertexIndex index_1,
           const VertexIndex index_2, const Eigen::Vector2f& point_1,
           const Eigen::Vector2f& point_2)
    : is_valid(is_valid),
      weight(weight),
      index_1(index_1),
      index_2(index_2),
      bounding_box(
          std::floor(std::min(point_1.x(), point_2.x()) +
                     std::ceil(std::fabs(point_1.x() - point_2.x())) / 2.0f),
          std::floor(std::min(point_1.y(), point_2.y()) +
                     std::ceil(std::fabs(point_1.y() - point_2.y())) / 2.0f),
          std::ceil(std::fabs(point_1.x() - point_2.x())),
          std::ceil(std::fabs(point_1.y() - point_2.y()))) {}
}  // namespace graph
