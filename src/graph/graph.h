// Copyright 2016 - 2018 kvedder@umass.edu, slane@cs.umass.edu
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

#include "eigen3/Eigen/Sparse"
#include "graph/edge.h"
#include "graph/vertex.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/safety_margin.h"

#include "util/timer.h"

#ifndef SRC_GRAPH_GRAPH_H_
#define SRC_GRAPH_GRAPH_H_

namespace graph {

class Graph {
 private:
  // This is the master list of all edges that exist in the graph. Any other
  // structure that intends to reference an edge should store an index into this
  // list.
  std::vector<Edge> master_edge_list;
  std::vector<Vertex> vertices;

 public:
  Graph(const Graph& other) = default;
  Graph(Graph&& other) = default;
  Graph() = default;
  ~Graph() = default;

  Graph& operator=(const Graph& other) = default;
  Graph& operator=(Graph&& other) = default;

  // Adds a new vertex at the given position. Returns the index of that vertex.
  VertexIndex AddVertex(const Eigen::Vector2f& position);

  const Vertex& GetVertex(const VertexIndex& vertex_index) const;

  Vertex* GetMutableVertex(const VertexIndex& vertex_index);

  const std::vector<Vertex>& GetVertices() const;

  std::vector<Vertex>* GetMutableVertices();

  // Adds an edge between the given indices of the given weight.
  EdgeIndex AddEdge(const VertexIndex first_index,
                    const VertexIndex second_index, float weight);

  const Edge& GetEdge(const EdgeIndex& edge_index) const;

  Edge* GetMutableEdge(const EdgeIndex& edge_index);

  const std::vector<Edge>& GetEdges() const;

  void ResetEdges();

  void InvalidateBlockedEdges(const obstacle::ObstacleFlag& obstacles,
                              const obstacle::SafetyMargin& margin);

  size_t GetNumEdges() const;

  size_t GetNumVertices() const;
};
}  // namespace graph

#endif  // SRC_GRAPH_GRAPH_H_
