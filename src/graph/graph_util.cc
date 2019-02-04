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
#include "graph/graph_util.h"

#include <string.h>
#include <sys/stat.h>
#include <time.h>

#include "constants/constants.h"
#include "util/serialization.h"

STANDARD_USINGS;

namespace graph {
namespace util {

MinuteBotsProto::GraphProto GraphToGraphProto(const Graph& graph) {
  MinuteBotsProto::GraphProto graph_proto;

  for (const Vertex& vertex : graph.GetVertices()) {
    auto* vertex_proto = graph_proto.add_vertex_list();
    vertex_proto->set_x(vertex.position.x());
    vertex_proto->set_y(vertex.position.y());
  }

  for (const Edge& edge : graph.GetEdges()) {
    auto* edge_proto = graph_proto.add_edge_list();
    edge_proto->set_weight(edge.weight);
    edge_proto->set_vertex_1_index(edge.index_1.index);
    edge_proto->set_vertex_2_index(edge.index_2.index);
  }
  return graph_proto;
}

MinuteBotsProto::MultiGraphProto MultiGraphToMultiGraphProto(
    const multi_graph::FastMultiGraph& multi_graph) {
  MinuteBotsProto::MultiGraphProto multi_graph_proto;

  for (const Graph& graph : multi_graph.GetGraphs()) {
    *multi_graph_proto.add_graph_list() = GraphToGraphProto(graph);
  }

  for (const edges::InterGraphEdge& inter_graph_edge :
       multi_graph.GetInterGraphEdges()) {
    auto* intra_graph_edge_proto =
        multi_graph_proto.add_intra_graph_edge_list();
    intra_graph_edge_proto->set_is_valid(inter_graph_edge.is_valid);
    intra_graph_edge_proto->set_weight(inter_graph_edge.weight);
    intra_graph_edge_proto->set_graph_1_index(inter_graph_edge.graph_1);
    intra_graph_edge_proto->set_vertex_1_index(inter_graph_edge.vertex_1.index);
    intra_graph_edge_proto->set_graph_2_index(inter_graph_edge.graph_2);
    intra_graph_edge_proto->set_vertex_2_index(inter_graph_edge.vertex_2.index);
  }
  return multi_graph_proto;
}

MinuteBotsProto::PathableMultiGraphProto
PathableMultiGraphToPathableMultiGraphProto(
    const multi_graph::FastMultiGraph& multi_graph,
    const vector<Vector2f>& path_waypoints,
    const vector<Vector2f>& smooth_path_waypoints) {
  MinuteBotsProto::PathableMultiGraphProto pathable_multi_graph_proto;
  *pathable_multi_graph_proto.mutable_multi_graph() =
      MultiGraphToMultiGraphProto(multi_graph);

  for (const auto& waypoint : path_waypoints) {
    auto* waypoint_proto = pathable_multi_graph_proto.add_path_waypoint_list();
    waypoint_proto->set_x(waypoint.x());
    waypoint_proto->set_y(waypoint.y());
  }

  for (const auto& waypoint : smooth_path_waypoints) {
    auto* waypoint_proto =
        pathable_multi_graph_proto.add_smoothed_path_waypoint_list();
    waypoint_proto->set_x(waypoint.x());
    waypoint_proto->set_y(waypoint.y());
  }

  return pathable_multi_graph_proto;
}

Graph GraphProtoToGraph(const MinuteBotsProto::GraphProto& graph_proto) {
  Graph graph;
  for (const auto& vertex_proto : graph_proto.vertex_list()) {
    graph.AddVertex(Vector2f(vertex_proto.x(), vertex_proto.y()));
  }
  for (const auto& edge_proto : graph_proto.edge_list()) {
    graph.AddEdge(VertexIndex(edge_proto.vertex_1_index()),
                  VertexIndex(edge_proto.vertex_2_index()),
                  edge_proto.weight());
  }
  return graph;
}

multi_graph::FastMultiGraph MultiGraphProtoToMultiGraph(
    const MinuteBotsProto::MultiGraphProto& multi_graph_proto) {
  multi_graph::FastMultiGraph multi_graph;
  for (const auto& graph_proto : multi_graph_proto.graph_list()) {
    multi_graph.AddAdditionalGraph(GraphProtoToGraph(graph_proto));
  }

  for (const auto& inter_graph_edge_proto :
       multi_graph_proto.intra_graph_edge_list()) {
    const Vector2f& point_1 =
        multi_graph
            .GetVertex(inter_graph_edge_proto.graph_1_index(),
                       VertexIndex(inter_graph_edge_proto.vertex_1_index()))
            .position;
    const Vector2f& point_2 =
        multi_graph
            .GetVertex(inter_graph_edge_proto.graph_2_index(),
                       VertexIndex(inter_graph_edge_proto.vertex_2_index()))
            .position;
    multi_graph.AddInterGraphEdge(edges::InterGraphEdge(
        inter_graph_edge_proto.is_valid(), inter_graph_edge_proto.weight(),
        inter_graph_edge_proto.graph_1_index(),
        VertexIndex(inter_graph_edge_proto.vertex_1_index()), point_1,
        inter_graph_edge_proto.graph_2_index(),
        VertexIndex(inter_graph_edge_proto.vertex_2_index()), point_2));
  }
  return multi_graph;
}

std::pair<multi_graph::FastMultiGraph,
          std::pair<std::vector<Eigen::Vector2f>, std::vector<Eigen::Vector2f>>>
PathableMultiGraphProtoToPathableMultiGraph(
    const MinuteBotsProto::PathableMultiGraphProto&
        pathable_multi_graph_proto) {
  vector<Vector2f> waypoints;
  for (const auto& waypoint_proto :
       pathable_multi_graph_proto.path_waypoint_list()) {
    waypoints.emplace_back(waypoint_proto.x(), waypoint_proto.y());
  }

  vector<Vector2f> smoothed_waypoints;
  for (const auto& waypoint_proto :
       pathable_multi_graph_proto.smoothed_path_waypoint_list()) {
    smoothed_waypoints.emplace_back(waypoint_proto.x(), waypoint_proto.y());
  }

  return {MultiGraphProtoToMultiGraph(pathable_multi_graph_proto.multi_graph()),
          {waypoints, smoothed_waypoints}};
}

void WriteGraphToProtoFile(const std::string& file_name, const Graph& graph) {
  int file_descriptor =
      ::util::serialization::CreateOrEraseFileForWrite(file_name);

  google::protobuf::io::FileOutputStream file_output(file_descriptor);
  file_output.SetCloseOnDelete(true);
  const MinuteBotsProto::GraphProto graph_proto = GraphToGraphProto(graph);

  if (!google::protobuf::TextFormat::Print(graph_proto, &file_output)) {
    LOG(ERROR) << "Failed to write graph proto to " << file_name;
  }
}

void WriteMultiGraphToProtoFile(
    const std::string& file_name,
    const multi_graph::FastMultiGraph& multi_graph) {
  int file_descriptor =
      ::util::serialization::CreateOrEraseFileForWrite(file_name);

  google::protobuf::io::FileOutputStream file_output(file_descriptor);
  file_output.SetCloseOnDelete(true);
  const MinuteBotsProto::MultiGraphProto multi_graph_proto =
      MultiGraphToMultiGraphProto(multi_graph);

  if (!google::protobuf::TextFormat::Print(multi_graph_proto, &file_output)) {
    LOG(ERROR) << "Failed to write graph proto to " << file_name;
  }
}

void WritePathToProtoFile(const string& file_name,
                          const vector<Vector2f>& path_waypoints) {
  int file_descriptor =
      ::util::serialization::CreateOrEraseFileForWrite(file_name);
  MinuteBotsProto::Path path_proto;
  for (const auto& waypoint : path_waypoints) {
    auto* waypoint_proto = path_proto.add_path_waypoint_list();
    waypoint_proto->set_x(waypoint.x());
    waypoint_proto->set_y(waypoint.y());
  }

  google::protobuf::io::FileOutputStream file_output(file_descriptor);
  file_output.SetCloseOnDelete(true);
  if (!google::protobuf::TextFormat::Print(path_proto, &file_output)) {
    LOG(ERROR) << "Failed to write path proto to " << file_name;
  }
}

void WritePointObstacleToProtoFile(
    const string& file_name,
    const std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                             navigation::eight_grid::EightGrid::GridHasher>&
        points,
    const float scale_factor) {
  int file_descriptor =
      ::util::serialization::CreateOrEraseFileForWrite(file_name);
  MinuteBotsProto::PointObstacle path_proto;
  for (const auto& waypoint : points) {
    auto* waypoint_proto = path_proto.add_point_list();
    waypoint_proto->set_x(waypoint.first.x() * scale_factor);
    waypoint_proto->set_y(waypoint.first.y() * scale_factor);
  }

  google::protobuf::io::FileOutputStream file_output(file_descriptor);
  file_output.SetCloseOnDelete(true);
  if (!google::protobuf::TextFormat::Print(path_proto, &file_output)) {
    LOG(ERROR) << "Failed to write path proto to " << file_name;
  }
}

void WritePathableMultiGraphToProtoFile(
    const string& file_name, const multi_graph::FastMultiGraph& multi_graph,
    const vector<Vector2f>& path_waypoints,
    const vector<Vector2f>& smoothed_path_waypoints) {
  int file_descriptor =
      ::util::serialization::CreateOrEraseFileForWrite(file_name);

  google::protobuf::io::FileOutputStream file_output(file_descriptor);
  file_output.SetCloseOnDelete(true);
  const MinuteBotsProto::PathableMultiGraphProto pathable_multi_graph_proto =
      PathableMultiGraphToPathableMultiGraphProto(multi_graph, path_waypoints,
                                                  smoothed_path_waypoints);

  if (!google::protobuf::TextFormat::Print(pathable_multi_graph_proto,
                                           &file_output)) {
    LOG(ERROR) << "Failed to write graph proto to " << file_name;
  }
}

void WriteObstacleFlagToFile(const string& file_name,
                             const obstacle::ObstacleFlag& obstacle_flag) {
  MinuteBotsProto::Obstacles obstacles;

  for (const obstacle::Obstacle* obstacle : obstacle_flag) {
    switch (obstacle->GetType()) {
      case obstacle::ObstacleType::BALL:
      case obstacle::ObstacleType::ROBOT: {
        const auto& pose = obstacle->GetPose();
        auto* circle = obstacles.add_circle_list();
        circle->set_x(pose.translation.x());
        circle->set_y(pose.translation.y());
        circle->set_radius(obstacle->GetRadius());
      } break;
      default:
        break;
    }
  }

  int file_descriptor =
      ::util::serialization::CreateOrEraseFileForWrite(file_name);
  google::protobuf::io::FileOutputStream file_output(file_descriptor);
  file_output.SetCloseOnDelete(true);
  if (!google::protobuf::TextFormat::Print(obstacles, &file_output)) {
    LOG(ERROR) << "Failed to write graph proto to " << file_name;
  }
}

std::string GraphToTextProto(const Graph& graph) {
  const MinuteBotsProto::GraphProto graph_proto = GraphToGraphProto(graph);
  std::string str;
  google::protobuf::TextFormat::PrintToString(graph_proto, &str);
  return str;
}

Graph ReadGraphFromTextProtoFile(const std::string& file_name,
                                 bool read_from_test_outputs) {
  int file_descriptor =
      (read_from_test_outputs)
          ? ::util::serialization::OpenFileForRead(file_name)
          : ::util::serialization::OpenGeneralFileForRead(file_name);

  MinuteBotsProto::GraphProto graph_proto;
  google::protobuf::io::FileInputStream file_input(file_descriptor);
  file_input.SetCloseOnDelete(true);
  if (!google::protobuf::TextFormat::Parse(&file_input, &graph_proto)) {
    LOG(FATAL) << "Failed to parse proto into Graph.";
  }

  return GraphProtoToGraph(graph_proto);
}

multi_graph::FastMultiGraph ReadMultiGraphFromTextProtoFile(
    const std::string& file_name) {
  int file_descriptor = ::util::serialization::OpenFileForRead(file_name);

  MinuteBotsProto::MultiGraphProto multi_graph_proto;
  google::protobuf::io::FileInputStream file_input(file_descriptor);
  file_input.SetCloseOnDelete(true);
  if (!google::protobuf::TextFormat::Parse(&file_input, &multi_graph_proto)) {
    LOG(FATAL) << "Failed to parse proto into MultiGraph.";
  }

  return MultiGraphProtoToMultiGraph(multi_graph_proto);
}

std::pair<multi_graph::FastMultiGraph,
          std::pair<std::vector<Eigen::Vector2f>, std::vector<Eigen::Vector2f>>>
ReadPathableMultiGraphFromTextProtoFile(const std::string& file_name) {
  int file_descriptor = ::util::serialization::OpenFileForRead(file_name);

  MinuteBotsProto::PathableMultiGraphProto pathable_multi_graph_proto;
  google::protobuf::io::FileInputStream file_input(file_descriptor);
  file_input.SetCloseOnDelete(true);
  if (!google::protobuf::TextFormat::Parse(&file_input,
                                           &pathable_multi_graph_proto)) {
    LOG(FATAL) << "Failed to parse proto into MultiGraph.";
  }

  return PathableMultiGraphProtoToPathableMultiGraph(
      pathable_multi_graph_proto);
}

Graph TextProtoToGraph(const std::string& str) {
  MinuteBotsProto::GraphProto graph_proto;
  google::protobuf::TextFormat::ParseFromString(str, &graph_proto);
  return GraphProtoToGraph(graph_proto);
}

}  // namespace util
}  // namespace graph
