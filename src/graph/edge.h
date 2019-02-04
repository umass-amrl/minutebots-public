// Copyright 2016 - 2017 kvedder@umass.edu slane@cs.umass.edu
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

#ifndef SRC_GRAPH_EDGE_H_
#define SRC_GRAPH_EDGE_H_

#include <cmath>
#include "eigen3/Eigen/Core"
#include "graph/vertex.h"
#include "obstacles/integer_bounding_box.h"

namespace graph {
struct Edge {
 public:
  bool is_valid;
  float weight;
  VertexIndex index_1;
  VertexIndex index_2;
  obstacle::IntegerBoundingBox bounding_box;

  Edge() = delete;
  Edge(const bool is_valid, const float weight, const VertexIndex index_1,
       const VertexIndex index_2, const Eigen::Vector2f& from_point,
       const Eigen::Vector2f& to_point);
  Edge(const Edge& other) = default;
  Edge(Edge&& other) = default;
};
}  // namespace graph

#endif  // SRC_GRAPH_EDGE_H_
