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

#include <vector>
#include "eigen3/Eigen/Core"

#ifndef SRC_GRAPH_GENERAL_GENERAL_VERTEX_H_
#define SRC_GRAPH_GENERAL_GENERAL_VERTEX_H_

namespace graph {
namespace general {

struct EdgeIndex {
  uint64_t index;
  EdgeIndex() = delete;
  explicit EdgeIndex(const uint64_t index) : index(index) {}
  EdgeIndex(const EdgeIndex& other) = default;
  EdgeIndex(EdgeIndex&& other) = default;
  ~EdgeIndex() = default;
  EdgeIndex& operator=(const EdgeIndex& other) = default;
  EdgeIndex& operator=(EdgeIndex&& other) = default;

  bool operator==(const EdgeIndex& other) const { return index == other.index; }

  bool operator!=(const EdgeIndex& other) const { return index != other.index; }

  bool operator<(const EdgeIndex& other) const { return index < other.index; }

  EdgeIndex& operator++() {
    ++index;
    return *this;
  }
};

struct VertexIndex {
  uint64_t index;
  VertexIndex() = delete;
  explicit VertexIndex(const uint64_t index) : index(index) {}
  VertexIndex(const VertexIndex& other) = default;
  VertexIndex(VertexIndex&& other) = default;
  ~VertexIndex() = default;
  VertexIndex& operator=(const VertexIndex& other) = default;
  VertexIndex& operator=(VertexIndex&& other) = default;
  bool operator==(const VertexIndex& other) const {
    return index == other.index;
  }

  bool operator!=(const VertexIndex& other) const {
    return index != other.index;
  }

  bool operator<(const VertexIndex& other) const { return index < other.index; }

  VertexIndex& operator++() {
    ++index;
    return *this;
  }
};

template <class Position>
struct Vertex {
  Position position;
  std::vector<EdgeIndex> edge_indices;
  Vertex() = delete;
  explicit Vertex(const Position& position) : position(position) {}
  ~Vertex() = default;

  const std::vector<EdgeIndex>& GetEdgeIndices() const { return edge_indices; }
};

}  // namespace general
}  // namespace graph

#endif  // SRC_GRAPH_GENERAL_GENERAL_VERTEX_H_
