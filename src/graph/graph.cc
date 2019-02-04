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
#include "graph/graph.h"

#include "navigation/navigation_util.h"

namespace graph {
VertexIndex Graph::AddVertex(const Vector2f& position) {
  // Emplace new vertex using the single parameter constructor for position.
  vertices.emplace_back(position);
  return VertexIndex(vertices.size() - 1);
}

const Vertex& Graph::GetVertex(const VertexIndex& vertex_index) const {
  return vertices[vertex_index.index];
}

Vertex* Graph::GetMutableVertex(const VertexIndex& vertex_index) {
  return &(vertices[vertex_index.index]);
}

const std::vector<Vertex>& Graph::GetVertices() const { return vertices; }

std::vector<Vertex>* Graph::GetMutableVertices() { return &vertices; }

EdgeIndex Graph::AddEdge(const VertexIndex first_index,
                         const VertexIndex second_index, float weight) {
  if (!kProduction) {
    if (first_index.index >= vertices.size()) {
      LOG(FATAL) << "First index: " << first_index.index
                 << " is out of range of the vertex vector, which is of size "
                 << vertices.size();
    }
    if (second_index.index >= vertices.size()) {
      LOG(FATAL) << "Second index: " << second_index.index
                 << " is out of range of the vertex vector, which is of size "
                 << vertices.size();
    }
  }

  Edge edge(/* is_active = */ true, weight, first_index, second_index,
            vertices[first_index.index].position,
            vertices[second_index.index].position);
  master_edge_list.push_back(edge);
  const EdgeIndex edge_index(master_edge_list.size() - 1);
  vertices[first_index.index].edge_indices.push_back(edge_index);
  vertices[second_index.index].edge_indices.push_back(edge_index);
  return edge_index;
}

const Edge& Graph::GetEdge(const EdgeIndex& edge_index) const {
  return master_edge_list[edge_index.index];
}

Edge* Graph::GetMutableEdge(const EdgeIndex& edge_index) {
  return &(master_edge_list[edge_index.index]);
}

const std::vector<Edge>& Graph::GetEdges() const { return master_edge_list; }

void Graph::ResetEdges() {
  for (Edge& edge : master_edge_list) {
    edge.is_valid = true;
  }
}

void Graph::InvalidateBlockedEdges(const obstacle::ObstacleFlag& obstacles,
                                   const obstacle::SafetyMargin& margin) {
  for (Edge& edge : master_edge_list) {
    edge.is_valid = navigation::CollisionFreePath(
        obstacles, margin, vertices[edge.index_1.index].position,
        vertices[edge.index_2.index].position);
  }
}

size_t Graph::GetNumEdges() const { return master_edge_list.size(); }

std::size_t Graph::GetNumVertices() const { return vertices.size(); }

}  // namespace graph
