// Copyright 2018 kvedder@umass.edu slane@cs.umass.edu
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

#ifndef SRC_GRAPH_GENERAL_GENERAL_EDGE_H_
#define SRC_GRAPH_GENERAL_GENERAL_EDGE_H_

#include <cmath>
#include "eigen3/Eigen/Core"
#include "graph/general/general_vertex.h"

namespace graph {
namespace general {

template <class Weight>
struct Edge {
 public:
  bool is_valid;
  Weight weight;
  VertexIndex index_1;
  VertexIndex index_2;

  Edge() = delete;
  Edge(const bool is_valid, const Weight weight, const VertexIndex index_1,
       const VertexIndex index_2)
      : is_valid(is_valid),
        weight(weight),
        index_1(index_1),
        index_2(index_2) {}
  Edge(const Edge& other) = default;
  Edge(Edge&& other) = default;
};
}  // namespace general
}  // namespace graph

#endif  // SRC_GRAPH_GENERAL_GENERAL_EDGE_H_
