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

static constexpr size_t kRobotCount = 1;
using Graph2d = ::graph::Graph;

void Setup(char* name) {
  google::InitGoogleLogging(name);
  FLAGS_stderrthreshold = 0;
  FLAGS_colorlogtostderr = 1;
  FLAGS_logtostderr = true;
}

std::string PositionToString(const JointPosition<kRobotCount>& jp) {
  std::stringstream ss;
  ss << "<" << jp[0].x() << ", " << jp[0].y() << ">";
  return ss.str();
}

::graph::VertexIndex SetupVertex(Graph2d* g2d, const Eigen::Vector2f& p) {
  const auto index = g2d->AddVertex(p);
  g2d->AddEdge(index, index, 0);
  g2d->AddEdge(index, index, 1);
  return index;
}

Graph2d GenerateEightGrid() {
  Graph2d g2d;

  constexpr size_t kGridWidth = 9;
  constexpr size_t kGridHeight = 9;
  std::vector<::graph::VertexIndex> row;
  for (size_t x = 0; x < kGridWidth; ++x) {
    row.push_back(::graph::VertexIndex(0));
  }
  std::vector<std::vector<::graph::VertexIndex>> grid;
  for (size_t y = 0; y < kGridHeight; ++y) {
    grid.push_back(row);
  }

  for (size_t x = 0; x < kGridWidth; ++x) {
    for (size_t y = 0; y < kGridHeight; ++y) {
      NP_CHECK(grid[y][x] == ::graph::VertexIndex(0));
      grid[y][x] = SetupVertex(&g2d, {x, y});
    }
  }

  for (size_t x = 1; x < kGridWidth - 1; x += 2) {
    for (size_t y = 1; y < kGridHeight - 1; y += 2) {
      g2d.AddEdge(grid[y - 1][x], grid[y][x], 1);
      g2d.AddEdge(grid[y + 1][x], grid[y][x], 1);
      g2d.AddEdge(grid[y][x - 1], grid[y][x], 1);
      g2d.AddEdge(grid[y][x + 1], grid[y][x], 1);
      g2d.AddEdge(grid[y - 1][x - 1], grid[y][x], kSqrtTwo);
      g2d.AddEdge(grid[y - 1][x + 1], grid[y][x], kSqrtTwo);
      g2d.AddEdge(grid[y + 1][x - 1], grid[y][x], kSqrtTwo);
      g2d.AddEdge(grid[y + 1][x + 1], grid[y][x], kSqrtTwo);

      g2d.AddEdge(grid[y - 1][x - 1], grid[y - 1][x], 1);
      g2d.AddEdge(grid[y - 1][x], grid[y - 1][x + 1], 1);
      g2d.AddEdge(grid[y - 1][x - 1], grid[y][x - 1], 1);
      g2d.AddEdge(grid[y][x - 1], grid[y + 1][x - 1], 1);
    }
  }

  return g2d;
}

Graph2d GeneratePGraph() {
  Graph2d g2d;

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

GeneralGraph<kRobotCount> ExpandGraph(const Graph2d& two_dimensional_graph) {
  static_assert(kRobotCount == 1, "Cannot build 2D graph with non 1 robots");
  GeneralGraph<kRobotCount> gg;
  for (const auto& outer_v : two_dimensional_graph.GetVertices()) {
    gg.AddVertex({{outer_v.position}});
  }

  for (const auto& outer_edge : two_dimensional_graph.GetEdges()) {
    const ::graph::general::VertexIndex e1(outer_edge.index_1.index);
    const ::graph::general::VertexIndex e2(outer_edge.index_2.index);
    gg.AddEdge(e1, e2, {{outer_edge.weight}});
  }
  return gg;
}

bool IsFreeEightGrid(const JointPosition<kRobotCount>& position) {
  static const std::vector<Eigen::Vector2f> kBlockedPositions = {
      {3, 4}, {3, 3}, {3, 5}, {3, 6}, {4, 4}, {4, 3},
      {4, 5}, {4, 6}, {5, 4}, {5, 3}, {5, 5}, {5, 6}};
  for (const Eigen::Vector2f& p : position) {
    if (std::find(kBlockedPositions.begin(), kBlockedPositions.end(), p) !=
        kBlockedPositions.end()) {
      return false;
    }
  }
  return true;
}

void EightGridGraph() {
  const GeneralGraph<kRobotCount> gg = ExpandGraph(GenerateEightGrid());
  ::search::eastar::ExpandingAStarSolver<kRobotCount> solver(gg);

  std::vector<bool> index_enabled;
  for (size_t i = 0; i < gg.GetNumVertices(); ++i) {
    index_enabled.push_back(IsFreeEightGrid(
        gg.GetVertex(::graph::general::VertexIndex(i)).position));
  }

  ::search::eastar::Iteration<kRobotCount> itr0(index_enabled, {}, {}, {{2.0f}},
                                                {{{2, 4}}}, {{{6, 4}}});

  const std::vector<JointPosition<kRobotCount>> start_connection_positions = {
      {{{0, 4}}}, {{{1, 4}}}, {{{2, 4}}}};
  const std::vector<JointDistance<kRobotCount>> start_connection_distances = {
      {{0}}, {{1}}, {{2}}};
  ::search::eastar::Iteration<kRobotCount> itr1(
      index_enabled, start_connection_positions, start_connection_distances,
      {{0.0f}}, {{{0, 4}}}, {{{8, 4}}});

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

bool IsFreeP(const JointPosition<kRobotCount>& position) {
  static const std::vector<Eigen::Vector2f> kBlockedPositions = {{0.5, 0}};
  for (const Eigen::Vector2f& p : position) {
    if (std::find(kBlockedPositions.begin(), kBlockedPositions.end(), p) !=
        kBlockedPositions.end()) {
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
    index_enabled.push_back(
        IsFreeP(gg.GetVertex(::graph::general::VertexIndex(i)).position));
  }

  ::search::eastar::Iteration<kRobotCount> itr0(index_enabled, {}, {}, {{0.0f}},
                                                {{{0, 0}}}, {{{1, 0}}});

  index_enabled.clear();
  for (size_t i = 0; i < gg.GetNumVertices(); ++i) {
    index_enabled.push_back(true);
  }

  const std::vector<JointPosition<kRobotCount>> start_connection_positions = {};
  const std::vector<JointDistance<kRobotCount>> start_connection_distances = {};
  ::search::eastar::Iteration<kRobotCount> itr1(
      index_enabled, start_connection_positions, start_connection_distances,
      {{0.0f}}, {{{0, 0}}}, {{{1, 0}}});

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

int main(int argc, char** argv) {
  Setup(argv[0]);
  if (argc < 2) {
    std::cout << "Arguments: <graph type>\n";
    return -1;
  }

  switch (argv[1][0]) {
    case 'e': {
      EightGridGraph();
    } break;
    case 'p': {
      PGraph();
    } break;
    default: { LOG(INFO) << "Unknown argument: " << argv[1][0]; } break;
  }

  return 0;
}
