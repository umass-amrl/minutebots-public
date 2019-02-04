// Copyright 2017 - 2018 kvedder@umass.edu
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

#include <fcntl.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <fstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "graph.pb.h"
#include "graph/fastmultigraph.h"
#include "graph/graph.h"
#include "multigraph.pb.h"
#include "navigation/eight_grid.h"
#include "obstacle.pb.h"
#include "obstacles/obstacle_flag.h"
#include "path.pb.h"
#include "pathable_multigraph.pb.h"
#include "point_obstacle.pb.h"
#include "search.pb.h"
#include "util/serialization.h"

#ifndef SRC_GRAPH_GRAPH_UTIL_H_
#define SRC_GRAPH_GRAPH_UTIL_H_

namespace graph {
namespace util {

std::string GraphToTextProto(const Graph& graph);

void WriteGraphToProtoFile(const std::string& file_path, const Graph& graph);

void WriteMultiGraphToProtoFile(const std::string& file_path,
                                const multi_graph::FastMultiGraph& multi_graph);

void WritePathToProtoFile(const std::string& file_path,
                          const std::vector<Vector2f>& path_waypoints);

void WritePointObstacleToProtoFile(
    const std::string& file_path,
    const std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                             navigation::eight_grid::EightGrid::GridHasher>&
        points,
    const float scale_factor);

void WritePathableMultiGraphToProtoFile(
    const std::string& file_path,
    const multi_graph::FastMultiGraph& multi_graph,
    const std::vector<Vector2f>& path_waypoints,
    const std::vector<Vector2f>& smoothed_path_waypoints);

void WriteObstacleFlagToFile(const std::string& file_path,
                             const obstacle::ObstacleFlag& obstacle_flag);

Graph TextProtoToGraph(const std::string& str);

Graph ReadGraphFromTextProtoFile(const std::string& file_path,
                                 bool read_from_test_outputs = true);

multi_graph::FastMultiGraph ReadMultiGraphFromTextProtoFile(
    const std::string& file_path);

std::pair<multi_graph::FastMultiGraph,
          std::pair<std::vector<Eigen::Vector2f>, std::vector<Eigen::Vector2f>>>
ReadPathableMultiGraphFromTextProtoFile(const std::string& file_path);

template <unsigned int kRobotCount>
void WriteSearchTree(
    const string& file_name,
    const std::pair<std::array<Eigen::Vector2i, kRobotCount>,
                    std::array<Eigen::Vector2i, kRobotCount>>&
        current_tree_edge,
    const float heuristic, const std::array<float, kRobotCount> distances,
    const std::vector<std::pair<std::array<Eigen::Vector2i, kRobotCount>,
                                std::array<Eigen::Vector2i, kRobotCount>>>&
        prev_tree_edges) {
  int file_descriptor =
      ::util::serialization::CreateOrEraseFileForWrite(file_name);
  MinuteBotsProto::SearchTree search_tree;
  auto* current_edge_proto = search_tree.mutable_current_edge();
  auto* current_edge_search_tree_edge =
      current_edge_proto->mutable_search_tree_edge();
  for (size_t i = 0; i < kRobotCount; ++i) {
    auto* current_position =
        current_edge_search_tree_edge->add_current_positions();
    auto* prev_position = current_edge_search_tree_edge->add_prev_positions();
    current_position->set_x(current_tree_edge.first[i].x());
    current_position->set_y(current_tree_edge.first[i].y());
    prev_position->set_x(current_tree_edge.second[i].x());
    prev_position->set_y(current_tree_edge.second[i].y());
  }

  for (const auto& prev_tree_edge : prev_tree_edges) {
    auto* current_edge_proto = search_tree.add_other_edge_list();
    for (size_t i = 0; i < kRobotCount; ++i) {
      auto* current_position = current_edge_proto->add_current_positions();
      auto* prev_position = current_edge_proto->add_prev_positions();
      current_position->set_x(prev_tree_edge.first[i].x());
      current_position->set_y(prev_tree_edge.first[i].y());
      prev_position->set_x(prev_tree_edge.second[i].x());
      prev_position->set_y(prev_tree_edge.second[i].y());
    }
  }

  google::protobuf::io::FileOutputStream file_output(file_descriptor);
  file_output.SetCloseOnDelete(true);
  if (!google::protobuf::TextFormat::Print(search_tree, &file_output)) {
    LOG(ERROR) << "Failed to write path proto to " << file_name;
  }
}

}  // namespace util
}  // namespace graph

#endif  // SRC_GRAPH_GRAPH_UTIL_H_
