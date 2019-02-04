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
#include "graph/fastmultigraph.h"

#include <algorithm>
#include <utility>

#include "constants/constants.h"
#include "navigation/navigation_util.h"

namespace graph {
namespace edges {
InterGraphEdge::InterGraphEdge(const bool is_valid, const float weight,
                               GraphIndex graph_1, VertexIndex vertex_1,
                               const Vector2f& point_1, GraphIndex graph_2,
                               VertexIndex vertex_2, const Vector2f& point_2)
    : is_valid(is_valid),
      weight(weight),
      graph_1(graph_1),
      vertex_1(vertex_1),
      graph_2(graph_2),
      vertex_2(vertex_2),
      bounding_box(
          std::floor(std::min(point_1.x(), point_2.x()) +
                     std::ceil(std::fabs(point_1.x() - point_2.x())) / 2.0f),
          std::floor(std::min(point_1.y(), point_2.y()) +
                     std::ceil(std::fabs(point_1.y() - point_2.y())) / 2.0f),
          std::ceil(std::fabs(point_1.x() - point_2.x())),
          std::ceil(std::fabs(point_1.y() - point_2.y()))) {}
}  // namespace edges

namespace multi_graph {
FastMultiGraph::FastMultiGraph(const Graph& base_graph,
                               GraphIndex* base_graph_index) {
  if (!kProduction) {
    CHECK_NOTNULL(base_graph_index);
  }
  graphs_.push_back(base_graph);
  *base_graph_index = (graphs_.size() - 1);
}

GraphIndex FastMultiGraph::AddAdditionalGraph(const Graph& graph) {
  graphs_.push_back(graph);
  return (graphs_.size() - 1);
}

inline void BoundsCheckGraphIndex(const GraphIndex graph_index,
                                  const std::vector<Graph>& graphs_) {
  if (!kProduction && graph_index >= graphs_.size()) {
    LOG(FATAL) << "Graph index " << graph_index
               << " out of range. (Vector size: " << graphs_.size() << ")";
  }
}

inline void BoundsCheckInterGraphEdges(
    const edges::InterGraphEdgeIndex& edge_index,
    const std::vector<edges::InterGraphEdge>& inter_graph_edges_) {
  if (!kProduction && edge_index >= inter_graph_edges_.size()) {
    LOG(FATAL) << "Edge index " << edge_index
               << " out of range. (Vector size: " << inter_graph_edges_.size()
               << ")";
  }
}

void FastMultiGraph::DeleteAdditionalGraph(GraphIndex graph_index) {
  BoundsCheckGraphIndex(graph_index, graphs_);
  graphs_.erase(graphs_.begin() + graph_index);
}

const Graph& FastMultiGraph::GetGraph(GraphIndex graph_index) {
  BoundsCheckGraphIndex(graph_index, graphs_);
  return (graphs_[graph_index]);
}

Graph* FastMultiGraph::GetMutableGraph(GraphIndex graph_index) {
  BoundsCheckGraphIndex(graph_index, graphs_);
  return &(graphs_[graph_index]);
}

inline std::pair<GraphIndex, size_t> GetMapKey(GraphIndex graph_index,
                                               VertexIndex vertex_index) {
  return std::make_pair(graph_index, vertex_index.index);
}

void FastMultiGraph::AddInterGraphEdge(const edges::InterGraphEdge& edge) {
  BoundsCheckGraphIndex(edge.graph_1, graphs_);
  BoundsCheckGraphIndex(edge.graph_2, graphs_);
  if (!kProduction) {
    if (edge.vertex_1.index >= graphs_[edge.graph_1].GetNumVertices()) {
      LOG(FATAL) << "Index 1 " << edge.vertex_1.index << " out of range (max: "
                 << graphs_[edge.graph_1].GetNumVertices() << ")";
    }
    if (edge.vertex_2.index >= graphs_[edge.graph_2].GetNumVertices()) {
      LOG(FATAL) << "Index 2 " << edge.vertex_2.index << " out of range (max: "
                 << graphs_[edge.graph_2].GetNumVertices() << ")";
    }
    const auto& v1 = graphs_[edge.graph_1].GetVertex(edge.vertex_1).position;
    const auto& v2 = graphs_[edge.graph_2].GetVertex(edge.vertex_2).position;
    if (edge.weight != (v2 - v1).norm()) {
      LOG(FATAL) << "Edge weight is wrong! (" << edge.weight << " vs "
                 << (v2 - v1).norm() << ")";
    }
  }

  inter_graph_edges_.push_back(edge);
  const edges::InterGraphEdgeIndex edge_index = (inter_graph_edges_.size() - 1);

  auto it = inter_graph_edge_indices_.insert(
      {GetMapKey(edge.graph_1, edge.vertex_1), {edge_index}});
  // If did not insert a new key, instead push back in existing vector.
  if (!it.second) {
    it.first->second.push_back(edge_index);
  }

  it = inter_graph_edge_indices_.insert(
      {GetMapKey(edge.graph_2, edge.vertex_2), {edge_index}});
  // If did not insert a new key, instead push back in existing vector.
  if (!it.second) {
    it.first->second.push_back(edge_index);
  }
}

void FastMultiGraph::GetVertexEdges(
    GraphIndex graph_index, VertexIndex vertex_index,
    std::vector<EdgeIndex>* next_graph_edges,
    std::vector<edges::InterGraphEdgeIndex>* next_inter_graph_edges) const {
  if (!kProduction) {
    CHECK_NOTNULL(next_graph_edges);
    CHECK_NOTNULL(next_inter_graph_edges);
    CHECK(next_graph_edges->empty());
    CHECK(next_inter_graph_edges->empty());
  }
  BoundsCheckGraphIndex(graph_index, graphs_);
  *next_graph_edges =
      graphs_[graph_index].GetVertex(vertex_index).GetEdgeIndices();
  auto search =
      inter_graph_edge_indices_.find(GetMapKey(graph_index, vertex_index));
  if (search != inter_graph_edge_indices_.end()) {
    *next_inter_graph_edges = search->second;
  }
}

const Edge& FastMultiGraph::GetGraphEdge(GraphIndex graph_index,
                                         const EdgeIndex& edge_index) const {
  BoundsCheckGraphIndex(graph_index, graphs_);
  return graphs_[graph_index].GetEdge(edge_index);
}

const edges::InterGraphEdge& FastMultiGraph::GetInterGraphEdge(
    edges::InterGraphEdgeIndex edge_index) const {
  BoundsCheckInterGraphEdges(edge_index, inter_graph_edges_);
  return inter_graph_edges_[edge_index];
}

const Vertex& FastMultiGraph::GetVertex(GraphIndex graph_index,
                                        const VertexIndex& vertex_index) const {
  BoundsCheckGraphIndex(graph_index, graphs_);
  return graphs_[graph_index].GetVertex(vertex_index);
}

const std::vector<Graph>& FastMultiGraph::GetGraphs() const { return graphs_; }

const std::vector<edges::InterGraphEdge>& FastMultiGraph::GetInterGraphEdges()
    const {
  return inter_graph_edges_;
}

void FastMultiGraph::ResetEdges() {
  inter_graph_edge_indices_.clear();
  inter_graph_edges_.clear();
  for (Graph& graph : graphs_) {
    graph.ResetEdges();
  }
}

void FastMultiGraph::InvalidateBlockedEdges(
    const obstacle::ObstacleFlag& obstacles,
    const obstacle::SafetyMargin& margin) {
  for (Graph& graph : graphs_) {
    graph.InvalidateBlockedEdges(obstacles, margin);
  }
  for (edges::InterGraphEdge& edge : inter_graph_edges_) {
    const Vector2f& position_1 =
        GetVertex(edge.graph_1, edge.vertex_1).position;
    const Vector2f& position_2 =
        GetVertex(edge.graph_2, edge.vertex_2).position;
    edge.is_valid = navigation::CollisionFreePath(obstacles, margin, position_1,
                                                  position_2);
  }
}

}  // namespace multi_graph
}  // namespace graph
