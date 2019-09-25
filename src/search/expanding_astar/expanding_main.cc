// Copyright 2018 kvedder@umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
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

#include <fstream>
#include <sstream>
#include <string>

#include "constants/constants.h"
#include "graph/general/general_graph.h"
#include "graph/graph.h"
#include "graph/graph_util.h"
#include "math/math_util.h"
#include "search/expanding_astar/expanding_astar_solver.h"
#include "third_party/googletest/googletest-release-1.8.0/googletest/include/gtest/gtest.h"
#include "util/array_util.h"

static constexpr size_t kRobotCount = 2;
using Graph2d = ::graph::Graph;

void Setup(char* name) {
  google::InitGoogleLogging(name);
  FLAGS_stderrthreshold = 0;
  FLAGS_colorlogtostderr = 1;
  FLAGS_logtostderr = true;
}

std::string PositionToString(const JointPosition<kRobotCount>& jp) {
  std::stringstream ss;
  ss << "<" << jp[0].x() << ", " << jp[0].y() << "> | <" << jp[1].x() << ", "
     << jp[1].y() << ">";
  return ss.str();
}

::graph::VertexIndex SetupVertex(Graph2d* g2d, const Eigen::Vector2f& p) {
  const auto index = g2d->AddVertex(p);
  g2d->AddEdge(index, index, 0);
  g2d->AddEdge(index, index, 1);
  return index;
}

Graph2d GenerateLine() {
  Graph2d g2d;

  const auto first = SetupVertex(&g2d, {0, 0});
  const auto second = SetupVertex(&g2d, {1, 0});

  g2d.AddEdge(first, second, 1);
  return g2d;
}

Graph2d GenerateBiggerDemoGraph() {
  Graph2d g2d;

  // Inner diamond
  const auto first = SetupVertex(&g2d, {0, 0});
  const auto second = SetupVertex(&g2d, {1, 1});
  const auto third = SetupVertex(&g2d, {1, -1});
  const auto fourth = SetupVertex(&g2d, {2, 0});

  g2d.AddEdge(first, second, kSqrtTwo);
  g2d.AddEdge(first, third, kSqrtTwo);
  g2d.AddEdge(first, fourth, 2);
  g2d.AddEdge(second, fourth, kSqrtTwo);
  g2d.AddEdge(third, fourth, kSqrtTwo);

  // Outer diamond
  const auto fifth = SetupVertex(&g2d, {-1, 0});
  const auto sixth = SetupVertex(&g2d, {1, 2});
  const auto seventh = SetupVertex(&g2d, {1, -2});
  const auto eigth = SetupVertex(&g2d, {3, 0});

  g2d.AddEdge(fifth, first, 1);
  g2d.AddEdge(fourth, eigth, 1);
  g2d.AddEdge(fifth, sixth, 2 * kSqrtTwo);
  g2d.AddEdge(fifth, seventh, 2 * kSqrtTwo);
  g2d.AddEdge(eigth, sixth, 2 * kSqrtTwo);
  g2d.AddEdge(eigth, seventh, 2 * kSqrtTwo);

  return g2d;
}

Graph2d GeneratePGraph() {
  Graph2d g2d;

  // Inner diamond
  const auto first = SetupVertex(&g2d, {0, 0});
  const auto second = SetupVertex(&g2d, {0, 1});
  const auto third = SetupVertex(&g2d, {0, 2});
  const auto fourth = SetupVertex(&g2d, {1, 1});
  const auto fifth = SetupVertex(&g2d, {1, 0});
  const auto sixth = SetupVertex(&g2d, {0.5, 0});

  g2d.AddEdge(first, second, 1);
  g2d.AddEdge(second, third, 1);
  g2d.AddEdge(second, fourth, 1);
  g2d.AddEdge(fourth, fifth, 1);
  g2d.AddEdge(fifth, sixth, 0.5);
  g2d.AddEdge(first, sixth, 0.5);

  return g2d;
}

Graph2d GenerateUGraph() {
  Graph2d g2d;

  // Inner diamond
  const auto l3 = SetupVertex(&g2d, {-1, 2});
  const auto l2 = SetupVertex(&g2d, {-1, 1});
  const auto l1 = SetupVertex(&g2d, {-1, 0});
  const auto l0 = SetupVertex(&g2d, {-1, -1});
  const auto c1 = SetupVertex(&g2d, {0, 0});
  const auto c0 = SetupVertex(&g2d, {0, -1});
  const auto r0 = SetupVertex(&g2d, {1, -1});
  const auto r1 = SetupVertex(&g2d, {1, 0});
  const auto r2 = SetupVertex(&g2d, {1, 1});
  const auto r3 = SetupVertex(&g2d, {1, 2});

  g2d.AddEdge(l3, l2, 1);
  g2d.AddEdge(l2, l1, 1);
  g2d.AddEdge(l1, l0, 1);
  g2d.AddEdge(l1, c1, 1);
  g2d.AddEdge(l0, c0, 1);
  g2d.AddEdge(r1, c1, 1);
  g2d.AddEdge(r0, c0, 1);
  g2d.AddEdge(r1, r2, 1);
  g2d.AddEdge(r2, r3, 1);

  return g2d;
}

Graph2d GenerateTGraph() {
  Graph2d g2d;

  // Inner diamond
  const auto first = SetupVertex(&g2d, {0, 0});
  const auto second = SetupVertex(&g2d, {1, 0});
  const auto third = SetupVertex(&g2d, {2, 0});
  const auto fourth = SetupVertex(&g2d, {3, 0});
  const auto fifth = SetupVertex(&g2d, {4, 0});

  const auto sixth = SetupVertex(&g2d, {2, 1});
  const auto seventh = SetupVertex(&g2d, {2, 2});

  g2d.AddEdge(first, second, 1);
  //   g2d.AddEdge(second, first, 1);

  g2d.AddEdge(second, third, 1);
  //   g2d.AddEdge(third, second, 1);

  g2d.AddEdge(third, fourth, 1);
  //   g2d.AddEdge(fourth, third, 1);

  g2d.AddEdge(fourth, fifth, 1);
  //   g2d.AddEdge(fifth, fourth, 1);

  g2d.AddEdge(third, sixth, 1);
  //   g2d.AddEdge(sixth, third, 1);

  g2d.AddEdge(sixth, seventh, 1);
  //   g2d.AddEdge(seventh, sixth, 1);

  LOG(INFO) << "For position: " << g2d.GetVertex(third).position.x() << ", "
            << g2d.GetVertex(third).position.y();
  for (const auto& edge_index : g2d.GetVertex(third).edge_indices) {
    const ::graph::Edge& edge = g2d.GetEdge(edge_index);
    const Eigen::Vector2f p1 = g2d.GetVertex(edge.index_1).position;
    const Eigen::Vector2f p2 = g2d.GetVertex(edge.index_2).position;
    LOG(INFO) << p1.x() << ", " << p1.y() << " <=> " << p2.x() << ", "
              << p2.y();
    if (p1 == Eigen::Vector2f(3, 0) || p2 == Eigen::Vector2f(3, 0)) {
      LOG(WARNING) << "Found (3, 0)";
    }
  }

  return g2d;
}

// GeneralGraph ExpandGraphOne(const Graph2d& two_dimensional_graph) {
//   static_assert(kRobotCount == 1, "Cannot build 2D graph with non 1 robots");
//   GeneralGraph gg;
//   for (const auto& outer_v : two_dimensional_graph.GetVertices()) {
//     gg.AddVertex({{ outer_v.position }});
//   }
//
//   for (const auto& outer_edge : two_dimensional_graph.GetEdges()) {
//     const ::graph::general::VertexIndex e1(outer_edge.index_1.index);
//     const ::graph::general::VertexIndex e2(outer_edge.index_2.index);
//     gg.AddEdge(e1, e2,
//                  {{outer_edge.weight}});
//   }
// }

// Shim to convert a two dimensional graph into a four dimensional graph by
// taking the cartesian product of the edges and vertices.
GeneralGraph<kRobotCount> ExpandGraph(const Graph2d& two_dimensional_graph) {
  static_assert(kRobotCount == 2, "Cannot build 2D graph with non 2 robots");
  GeneralGraph<kRobotCount> gg;
  LOG(INFO) << "NUM VERT: " << gg.GetNumVertices();
  LOG(INFO) << "TWO D VERT: " << two_dimensional_graph.GetNumVertices();
  // Add all vertices to the graph.
  for (const auto& outer_v : two_dimensional_graph.GetVertices()) {
    for (const auto& inner_v : two_dimensional_graph.GetVertices()) {
      const JointPosition<kRobotCount> position = {
          {outer_v.position, inner_v.position}};
      gg.AddVertex(position);
    }
  }

  // Add all edges between vertices.
  for (const auto& outer_edge : two_dimensional_graph.GetEdges()) {
    const ::graph::Vertex outer_1 =
        two_dimensional_graph.GetVertex(outer_edge.index_1);
    const ::graph::Vertex outer_2 =
        two_dimensional_graph.GetVertex(outer_edge.index_2);
    const Eigen::Vector2f outer_start(2, 0);
    const Eigen::Vector2f outer_end(3, 0);

    const bool has_outer_start =
        (outer_1.position == outer_start) || (outer_2.position == outer_start);
    const bool has_outer_end =
        (outer_1.position == outer_end) || (outer_2.position == outer_end);

    for (const auto& inner_edge : two_dimensional_graph.GetEdges()) {
      const ::graph::Vertex inner_1 =
          two_dimensional_graph.GetVertex(inner_edge.index_1);
      const ::graph::Vertex inner_2 =
          two_dimensional_graph.GetVertex(inner_edge.index_2);
      const Eigen::Vector2f inner_start(2, 1);
      const Eigen::Vector2f inner_end(2, 0);
      const JointPosition<kRobotCount> position_1 = {
          {outer_1.position, inner_1.position}};
      const JointPosition<kRobotCount> position_2 = {
          {outer_2.position, inner_2.position}};
      const JointPosition<kRobotCount> position_3 = {
          {outer_1.position, inner_2.position}};
      const JointPosition<kRobotCount> position_4 = {
          {outer_2.position, inner_1.position}};

      const bool has_inner_start = (inner_1.position == inner_start) ||
                                   (inner_2.position == inner_start);
      const bool has_inner_end =
          (inner_1.position == inner_end) || (inner_2.position == inner_end);

      if (has_outer_start && has_outer_end && has_inner_start &&
          has_inner_end) {
        LOG(INFO) << "Outer: " << outer_1.position.x() << ", "
                  << outer_1.position.y() << " => " << outer_2.position.x()
                  << ", " << outer_2.position.y();
        LOG(INFO) << "Inner: " << inner_1.position.x() << ", "
                  << inner_1.position.y() << " => " << inner_2.position.x()
                  << ", " << inner_2.position.y();

        LOG(INFO) << "Position 1: " << PositionToString(position_1);
        LOG(INFO) << "Position 2: " << PositionToString(position_2);
        LOG(INFO) << "Position 3: " << PositionToString(position_3);
        LOG(INFO) << "Position 4: " << PositionToString(position_4);

        //         LOG(FATAL) << "Found edge componenets!";
      }

      const auto e1_position1 = gg.GetVertexIndex(position_1);
      const auto e1_position2 = gg.GetVertexIndex(position_2);
      const auto e2_position1 = gg.GetVertexIndex(position_3);
      const auto e2_position2 = gg.GetVertexIndex(position_4);
      gg.AddEdge(e1_position1, e1_position2,
                 {{outer_edge.weight, inner_edge.weight}});
      gg.AddEdge(e2_position1, e2_position2,
                 {{outer_edge.weight, inner_edge.weight}});
    }
  }

  NP_CHECK_EQ(gg.GetNumVertices(), Sq(two_dimensional_graph.GetNumVertices()));
  NP_CHECK_EQ(gg.GetNumEdges(), 2 * Sq(two_dimensional_graph.GetNumEdges()));

  return gg;
}

bool IsInInnerU(const JointPosition<kRobotCount>& position) {
  static const std::vector<Eigen::Vector2f> kOuterPositions = {{-1, 2}, {1, 2}};
  for (const Eigen::Vector2f& p : position) {
    if (std::find(kOuterPositions.begin(), kOuterPositions.end(), p) !=
        kOuterPositions.end()) {
      return false;
    }
  }
  return true;
}

void UGraph() {
  const GeneralGraph<kRobotCount> gg = ExpandGraph(GenerateUGraph());
  ::search::eastar::ExpandingAStarSolver<kRobotCount> solver(gg);

  std::vector<bool> index_enabled;
  for (size_t i = 0; i < gg.GetNumVertices(); ++i) {
    index_enabled.push_back(IsInInnerU(gg.GetVertices()[i].position));
  }

  ::search::eastar::Iteration<kRobotCount> itr0(
      index_enabled, {}, {}, {{1.0f, 1.0f}}, {{{-1, 1}, {1, 1}}},
      {{{1, 1}, {-1, 1}}});

  index_enabled.clear();
  for (size_t i = 0; i < gg.GetNumVertices(); ++i) {
    index_enabled.push_back(true);
  }

  const std::vector<JointPosition<kRobotCount>> start_connection_positions = {
      {{{-1, 2}, {1, 2}}}, {{{-1, 1}, {1, 1}}}};
  const std::vector<JointDistance<kRobotCount>> start_connection_distances = {
      {{0, 0}}, {{1, 1}}};
  ::search::eastar::Iteration<kRobotCount> itr1(
      index_enabled, start_connection_positions, start_connection_distances,
      {{0.0f, 0.0f}}, {{{-1, 2}, {1, 2}}}, {{{1, 2}, {-1, 2}}});

  static constexpr size_t kTimeIterations = 1;

  ::search::eastar::PathReturnType<kRobotCount> result;

  LOG(INFO) << "Standard A*";
  const auto sa_start = GetMonotonicTime();
  for (size_t i = 0; i < kTimeIterations; ++i) {
    result = solver.SolveAStar({itr0, itr1});
  }
  const auto sa_end = GetMonotonicTime();
  LOG(INFO) << "Delta (ms): " << (sa_end - sa_start) * 1000 / kTimeIterations;
  LOG(INFO) << "Expanding A*";
  const auto ea_start = GetMonotonicTime();
  for (size_t i = 0; i < kTimeIterations; ++i) {
    result = solver.SolveExpandingAStar({itr0, itr1});
  }
  const auto ea_end = GetMonotonicTime();
  LOG(INFO) << "Delta (ms): " << (ea_end - ea_start) * 1000 / kTimeIterations;
}

bool IsInInnerP(const JointPosition<kRobotCount>& position) {
  static const std::vector<Eigen::Vector2f> kOuterPositions = {{0.5, 0}};
  for (const Eigen::Vector2f& p : position) {
    if (std::find(kOuterPositions.begin(), kOuterPositions.end(), p) !=
        kOuterPositions.end()) {
      return false;
    }
  }
  return true;
}

void PGraph() {
  const GeneralGraph<kRobotCount> gg = ExpandGraph(GeneratePGraph());
  ::search::eastar::ExpandingAStarSolver<kRobotCount> solver(gg);

  std::vector<bool> index_enabled;
  for (size_t i = 0; i < gg.GetNumVertices(); ++i) {
    index_enabled.push_back(IsInInnerP(gg.GetVertices()[i].position));
  }

  ::search::eastar::Iteration<kRobotCount> itr0(
      index_enabled, {}, {}, {{0, 0}}, {{{0, 0}, {1, 0}}}, {{{1, 0}, {0, 0}}});

  index_enabled.clear();
  for (size_t i = 0; i < gg.GetNumVertices(); ++i) {
    index_enabled.push_back(true);
  }

  const std::vector<JointPosition<kRobotCount>> start_connection_positions = {
      {{{0, 0}, {1, 0}}}, {{{0, 0}, {1, 0}}}};
  const std::vector<JointDistance<kRobotCount>> start_connection_distances = {
      {{0, 0}}, {{0, 0}}};
  ::search::eastar::Iteration<kRobotCount> itr1(
      index_enabled, start_connection_positions, start_connection_distances,
      {{0.0f, 0.0f}}, {{{0, 0}, {1, 0}}}, {{{1, 0}, {0, 0}}});

  static constexpr size_t kTimeIterations = 1;

  ::search::eastar::PathReturnType<kRobotCount> result;

  LOG(INFO) << "Standard A*";
  const auto sa_start = GetMonotonicTime();
  for (size_t i = 0; i < kTimeIterations; ++i) {
    result = solver.SolveAStar({itr0, itr1});
  }
  const auto sa_end = GetMonotonicTime();
  LOG(INFO) << "Delta (ms): " << (sa_end - sa_start) * 1000 / kTimeIterations;
  LOG(INFO) << "Expanding A*";
  const auto ea_start = GetMonotonicTime();
  for (size_t i = 0; i < kTimeIterations; ++i) {
    result = solver.SolveExpandingAStar({itr0, itr1});
  }
  const auto ea_end = GetMonotonicTime();
  LOG(INFO) << "Delta (ms): " << (ea_end - ea_start) * 1000 / kTimeIterations;
}

bool IsInInnerT(const JointPosition<kRobotCount>& position) {
  static const std::vector<Eigen::Vector2f> kOuterPositions = {{0, 0}, {4, 0}};
  for (const Eigen::Vector2f& p : position) {
    if (std::find(kOuterPositions.begin(), kOuterPositions.end(), p) !=
        kOuterPositions.end()) {
      return false;
    }
  }
  return true;
}

void TGraph() {
  const GeneralGraph<kRobotCount> gg = ExpandGraph(GenerateTGraph());
  ::search::eastar::ExpandingAStarSolver<kRobotCount> solver(gg);

  std::vector<bool> index_enabled;
  for (size_t i = 0; i < gg.GetNumVertices(); ++i) {
    index_enabled.push_back(IsInInnerT(gg.GetVertices()[i].position));
  }

  ::search::eastar::Iteration<kRobotCount> itr0(
      index_enabled, {}, {}, {{1.0f, 1.0f}}, {{{1, 0}, {3, 0}}},
      {{{3, 0}, {1, 0}}});

  index_enabled.clear();
  for (size_t i = 0; i < gg.GetNumVertices(); ++i) {
    index_enabled.push_back(true);
  }
  const std::vector<JointPosition<kRobotCount>> start_connection_positions = {
      {{{0, 0}, {4, 0}}}, {{{1, 0}, {3, 0}}}};
  const std::vector<JointDistance<kRobotCount>> start_connection_distances = {
      {{0, 0}}, {{1, 1}}};
  ::search::eastar::Iteration<kRobotCount> itr1(
      index_enabled, start_connection_positions, start_connection_distances,
      {{0.0f, 0.0f}}, {{{0, 0}, {4, 0}}}, {{{4, 0}, {0, 0}}});

  static constexpr size_t kTimeIterations = 1;

  ::search::eastar::PathReturnType<kRobotCount> result;

  LOG(INFO) << "Standard A*";
  const auto sa_start = GetMonotonicTime();
  for (size_t i = 0; i < kTimeIterations; ++i) {
    result = solver.SolveAStar({itr0, itr1});
  }
  const auto sa_end = GetMonotonicTime();
  LOG(INFO) << "Delta (ms): " << (sa_end - sa_start) * 1000 / kTimeIterations;
  LOG(INFO) << "Expanding A*";
  const auto ea_start = GetMonotonicTime();
  for (size_t i = 0; i < kTimeIterations; ++i) {
    result = solver.SolveExpandingAStar({itr0, itr1});
  }
  const auto ea_end = GetMonotonicTime();
  LOG(INFO) << "Delta (ms): " << (ea_end - ea_start) * 1000 / kTimeIterations;
}

bool IsInInnerDiamond(const JointPosition<kRobotCount>& position) {
  static const std::vector<Eigen::Vector2f> kInnerPositions = {
      {0, 0}, {1, 1}, {1, -1}, {2, 0}};
  for (const Eigen::Vector2f& p : position) {
    if (std::find(kInnerPositions.begin(), kInnerPositions.end(), p) ==
        kInnerPositions.end()) {
      return false;
    }
  }
  return true;
}

void DiamondGraph() {
  const GeneralGraph<kRobotCount> gg = ExpandGraph(GenerateBiggerDemoGraph());
  ::search::eastar::ExpandingAStarSolver<kRobotCount> solver(gg);

  std::vector<bool> index_enabled;
  for (size_t i = 0; i < gg.GetNumVertices(); ++i) {
    index_enabled.push_back(IsInInnerDiamond(gg.GetVertices()[i].position));
  }

  ::search::eastar::Iteration<kRobotCount> itr0(
      index_enabled, {}, {}, {{1.0f, 1.0f}}, {{{0, 0}, {2, 0}}},
      {{{2, 0}, {0, 0}}});

  index_enabled.clear();
  for (size_t i = 0; i < gg.GetNumVertices(); ++i) {
    index_enabled.push_back(true);
  }

  const std::vector<JointPosition<kRobotCount>> start_connection_positions = {
      {{{-1, 0}, {3, 0}}}, {{{0, 0}, {2, 0}}}};
  const std::vector<JointDistance<kRobotCount>> start_connection_distances = {
      {{0, 0}}, {{1, 1}}};
  ::search::eastar::Iteration<kRobotCount> itr1(
      index_enabled, start_connection_positions, start_connection_distances,
      {{0.0f, 0.0f}}, {{{-1, 0}, {3, 0}}}, {{{3, 0}, {-1, 0}}});

  static constexpr size_t kTimeIterations = 1;

  ::search::eastar::PathReturnType<kRobotCount> result;

  LOG(INFO) << "Expanding A*";
  const auto ea_start = GetMonotonicTime();
  for (size_t i = 0; i < kTimeIterations; ++i) {
    result = solver.SolveExpandingAStar({itr0, itr1});
  }
  const auto ea_end = GetMonotonicTime();
  LOG(INFO) << "Delta (ms): " << (ea_end - ea_start) * 1000 / kTimeIterations;
  LOG(INFO) << "Standard A*";
  const auto sa_start = GetMonotonicTime();
  for (size_t i = 0; i < kTimeIterations; ++i) {
    result = solver.SolveAStar({itr0, itr1});
  }
  const auto sa_end = GetMonotonicTime();
  LOG(INFO) << "Delta (ms): " << (sa_end - sa_start) * 1000 / kTimeIterations;
}

int main(int argc, char** argv) {
  Setup(argv[0]);
  if (argc < 2) {
    std::cout << "Arguments: <graph type>\n";
    return -1;
  }

  switch (argv[1][0]) {
    case 'd': {
      DiamondGraph();
    } break;
    case 't': {
      TGraph();
    } break;
    case 'p': {
      PGraph();
    } break;
    case 'u': {
      UGraph();
    } break;
    case 'g': {
      GeneralGraph<kRobotCount> gg = ExpandGraph(GenerateLine());
      for (const auto& e : gg.GetEdges()) {
        const auto& p1 = gg.GetVertex(e.index_1).position;
        const auto& p2 = gg.GetVertex(e.index_2).position;
        LOG(INFO) << "Edge: " << PositionToString(p1) << " => "
                  << PositionToString(p2) << " cost: " << e.weight[0] << ", "
                  << e.weight[1];
      }
    } break;

    default: { LOG(INFO) << "Unknown argument: " << argv[1][0]; } break;
  }

  return 0;
}
