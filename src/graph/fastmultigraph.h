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

#include <map>
#include <utility>
#include <vector>

#include "eigen3/Eigen/Sparse"
#include "graph/graph.h"
#include "graph/vertex.h"
#include "obstacles/obstacle.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/safety_margin.h"

#include "util/timer.h"

#ifndef SRC_GRAPH_FASTMULTIGRAPH_H_
#define SRC_GRAPH_FASTMULTIGRAPH_H_

namespace graph {

using GraphIndex = uint64_t;

namespace edges {
using InterGraphEdgeIndex = size_t;

// Stores information about the edge that goes between two graphs in a
// multigraph.
struct InterGraphEdge {
  bool is_valid;
  float weight;
  GraphIndex graph_1;
  VertexIndex vertex_1;
  GraphIndex graph_2;
  VertexIndex vertex_2;
  obstacle::IntegerBoundingBox bounding_box;

  InterGraphEdge() = delete;
  InterGraphEdge(const bool is_valid, const float weight, GraphIndex graph_1,
                 VertexIndex vertex_1, const Vector2f& point_1,
                 GraphIndex graph_2, VertexIndex vertex_2,
                 const Vector2f& point_2);
};

}  // namespace edges

namespace multi_graph {
// This is intended to be a class that is *fast*, and thus we will make design
// decisions that value performance and simplicity over API abstractions.
class FastMultiGraph {
 private:
  std::vector<Graph> graphs_;
  std::map<std::pair<GraphIndex, size_t>,
           std::vector<edges::InterGraphEdgeIndex>>
      inter_graph_edge_indices_;
  std::vector<edges::InterGraphEdge> inter_graph_edges_;

 public:
  FastMultiGraph() = default;
  FastMultiGraph(const Graph& base_graph, GraphIndex* base_graph_index);
  ~FastMultiGraph() = default;

  // This is the intended method of adding subgraphs to the multigraph. Once
  // constructed, you pass them into this function which makes a copy. From then
  // on, the owner of the graph should be the multigraph copy. To interact with
  // it, fetch it from the multigraph using the index and the provided
  // functions.
  GraphIndex AddAdditionalGraph(const Graph& graph);

  // Note that this invalidates *all* pointers to graphs.
  void DeleteAdditionalGraph(GraphIndex graph_index);

  // This is the function intended to be able to update graph locations.
  const Graph& GetGraph(GraphIndex graph_index);

  // This is the function intended to be able to update graph locations.
  Graph* GetMutableGraph(GraphIndex graph_index);

  void AddInterGraphEdge(const edges::InterGraphEdge& edge);

  void GetVertexEdges(
      GraphIndex graph_index, VertexIndex vertex_index,
      std::vector<EdgeIndex>* next_graph_edges,
      std::vector<edges::InterGraphEdgeIndex>* next_inter_graph_edges) const;

  const Edge& GetGraphEdge(GraphIndex graph_index,
                           const EdgeIndex& edge_index) const;

  const edges::InterGraphEdge& GetInterGraphEdge(
      edges::InterGraphEdgeIndex edge_index) const;

  const Vertex& GetVertex(GraphIndex graph_index,
                          const VertexIndex& vertex_index) const;

  const std::vector<Graph>& GetGraphs() const;

  const std::vector<edges::InterGraphEdge>& GetInterGraphEdges() const;

  void ResetEdges();

  void InvalidateBlockedEdges(const obstacle::ObstacleFlag& obstacles,
                              const obstacle::SafetyMargin& margin);
};
}  // namespace multi_graph
}  // namespace graph

#endif  // SRC_GRAPH_FASTMULTIGRAPH_H_
