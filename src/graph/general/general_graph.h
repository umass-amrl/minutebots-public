// Copyright 2018 kvedder@umass.edu, slane@cs.umass.edu
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

#include "constants/constants.h"
#include "eigen3/Eigen/Sparse"
#include "graph/general/general_edge.h"
#include "graph/general/general_vertex.h"

#ifndef SRC_GRAPH_GENERAL_GENERAL_GRAPH_H_
#define SRC_GRAPH_GENERAL_GENERAL_GRAPH_H_

namespace graph {
namespace general {

template <class Position, class Weight>
class GeneralGraph {
 private:
  // This is the master list of all edges that exist in the graph. Any other
  // structure that intends to reference an edge should store an index into this
  // list.
  std::vector<Edge<Weight>> master_edge_list;
  std::vector<Vertex<Position>> vertices;

 public:
  GeneralGraph(const GeneralGraph& other) = default;
  GeneralGraph(GeneralGraph&& other) = default;
  GeneralGraph() = default;
  ~GeneralGraph() = default;

  GeneralGraph& operator=(const GeneralGraph& other) = default;
  GeneralGraph& operator=(GeneralGraph&& other) = default;

  // Adds a new vertex at the given position. Returns the index of that vertex.
  VertexIndex AddVertex(const Position& position) {
    // Emplace new vertex using the single parameter constructor for position.
    vertices.push_back(Vertex<Position>(position));
    return VertexIndex(vertices.size() - 1);
  }

  VertexIndex GetVertexIndex(const Position& position) const {
    for (size_t i = 0; i < GetVertices().size(); ++i) {
      const auto& v = GetVertices()[i];
      if (v.position == position) {
        return VertexIndex(i);
      }
    }

    LOG(FATAL) << "No index found!";
    return VertexIndex(0);
  }

  const Vertex<Position>& GetVertex(const VertexIndex& vertex_index) const {
    return vertices[vertex_index.index];
  }

  Vertex<Position>* GetMutableVertex(const VertexIndex& vertex_index) {
    return &(vertices[vertex_index.index]);
  }

  const std::vector<Vertex<Position>>& GetVertices() const { return vertices; }

  std::vector<Vertex<Position>>* GetMutableVertices() { return &vertices; }

  // Adds an edge between the given indices of the given weight.
  EdgeIndex AddEdge(const VertexIndex first_index,
                    const VertexIndex second_index, Weight weight) {
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

    Edge<Weight> edge(/* is_active = */ true, weight, first_index,
                      second_index);
    master_edge_list.push_back(edge);
    const EdgeIndex edge_index(master_edge_list.size() - 1);
    vertices[first_index.index].edge_indices.push_back(edge_index);
    vertices[second_index.index].edge_indices.push_back(edge_index);
    return edge_index;
  }

  const Edge<Weight>& GetEdge(const EdgeIndex& edge_index) const {
    return master_edge_list[edge_index.index];
  }

  Edge<Weight>* GetMutableEdge(const EdgeIndex& edge_index) {
    return &(master_edge_list[edge_index.index]);
  }

  const std::vector<Edge<Weight>>& GetEdges() const { return master_edge_list; }

  void ResetEdges() {
    for (Edge<Weight>& edge : master_edge_list) {
      edge.is_valid = true;
    }
  }

  size_t GetNumEdges() const { return master_edge_list.size(); }

  size_t GetNumVertices() const { return vertices.size(); }
};

}  // namespace general
}  // namespace graph

#endif  // SRC_GRAPH_GENERAL_GENERAL_GRAPH_H_
