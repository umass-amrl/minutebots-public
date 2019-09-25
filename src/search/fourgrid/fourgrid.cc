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

#include "search/fourgrid/fourgrid.h"

#include <fstream>
#include <sstream>

namespace search {
namespace fourgrid {

template <size_t kRobotCount>
FourGridDirectedEdge<kRobotCount>::FourGridDirectedEdge(
    const JointPosition<kRobotCount>& from,
    const JointPosition<kRobotCount>& to, const int& cost)
    : from(from), to(to), cost(cost) {}

template <size_t kRobotCount>
bool FourGridDirectedEdge<kRobotCount>::operator==(
    const FourGridDirectedEdge& other) const {
  return (from == other.from) && (to == other.to) && (cost == other.cost);
}

template <size_t kRobotCount>
FourGrid<kRobotCount>::FourGrid(
    const std::vector<FourGridDirectedEdge<kRobotCount>>& edges)
    : edges(edges) {}

template <size_t kRobotCount>
FourGrid<kRobotCount>::FourGrid(const std::string& file_name) {
  std::ifstream infile(file_name);

  if (!infile) {
    LOG(FATAL) << "File " << file_name << " not found!";
  }

  std::string line;
  while (std::getline(infile, line)) {
    std::istringstream iss(line);

    JointPosition<kRobotCount> from;
    JointPosition<kRobotCount> to;
    int cost = 0;

    for (size_t i = 0; i < kRobotCount; ++i) {
      char robot;
      int x, y;
      if (!(iss >> robot >> x >> y)) {
        LOG(FATAL)
            << "Unable to find all left requested robot positions! String: "
            << line;
      }
      from[i] = {x, y};
    }
    {
      char equals, arrow;
      if (!(iss >> equals >> cost >> arrow)) {
        LOG(FATAL) << "Unable to find arrow! String: " << line;
      }
    }
    for (size_t i = 0; i < kRobotCount; ++i) {
      char robot;
      int x, y;
      if (!(iss >> robot >> x >> y)) {
        LOG(FATAL)
            << "Unable to find all right requested robot positions! String: "
            << line;
      }
      to[i] = {x, y};
    }

    edges.push_back({from, to, cost});
  }
}

template <size_t kRobotCount>
std::vector<FourGridDirectedEdge<kRobotCount>> FourGrid<kRobotCount>::GetEdges(
    const JointPosition<kRobotCount>& position) const {
  std::vector<FourGridDirectedEdge<kRobotCount>> filtered_edges;
  for (const FourGridDirectedEdge<kRobotCount>& edge : edges) {
    if (edge.from == position) {
      filtered_edges.push_back(edge);
    }
  }
  return filtered_edges;
}

template <size_t kRobotCount>
std::vector<FourGridDirectedEdge<kRobotCount>>
FourGrid<kRobotCount>::GetBackwardEdges(
    const JointPosition<kRobotCount>& position) const {
  std::vector<FourGridDirectedEdge<kRobotCount>> filtered_edges;
  for (const FourGridDirectedEdge<kRobotCount>& edge : edges) {
    if (edge.to == position) {
      filtered_edges.push_back(edge);
    }
  }
  return filtered_edges;
}

template <size_t kRobotCount>
std::vector<FourGridDirectedEdge<kRobotCount>> FourGrid<
    kRobotCount>::GetUniqueEdges(const FourGrid<kRobotCount>& other) const {
  std::vector<FourGridDirectedEdge<kRobotCount>> unique_edges;
  for (const auto& e : edges) {
    if (std::find(other.edges.begin(), other.edges.end(), e) ==
        other.edges.end()) {
      unique_edges.push_back(e);
    }
  }
  return unique_edges;
}

template <size_t kRobotCount>
std::vector<JointPosition<kRobotCount>> FourGrid<kRobotCount>::GetPositions()
    const {
  std::vector<JointPosition<kRobotCount>> positions;
  for (const auto& edge : edges) {
    // Add unseen froms.
    if (std::find(positions.begin(), positions.end(), edge.from) ==
        positions.end()) {
      positions.push_back(edge.from);
    }
    // Add unseen tos.
    if (std::find(positions.begin(), positions.end(), edge.to) ==
        positions.end()) {
      positions.push_back(edge.to);
    }
  }
  return positions;
}

template <size_t kRobotCount>
std::vector<JointPosition<kRobotCount>> FourGrid<
    kRobotCount>::GetUniquePositions(const FourGrid<kRobotCount>& other) const {
  std::vector<JointPosition<kRobotCount>> unique_positions;
  const auto positions = GetPositions();
  const auto other_positions = other.GetPositions();
  for (const auto& p : positions) {
    if (std::find(other_positions.begin(), other_positions.end(), p) ==
        other_positions.end()) {
      unique_positions.push_back(p);
    }
  }
  return unique_positions;
}

template <size_t kRobotCount>
bool InL1(const JointPosition<kRobotCount>& center, const int& radius,
          const JointPosition<kRobotCount>& query) {
  for (size_t i = 0; i < kRobotCount; ++i) {
    const Eigen::Vector2i delta = (center[i] - query[i]);
    if (delta.lpNorm<1>() > radius) {
      return false;
    }
  }
  return true;
}

template <size_t kRobotCount>
bool InLInf(const JointPosition<kRobotCount>& center, const int& radius,
            const JointPosition<kRobotCount>& query) {
  for (size_t i = 0; i < kRobotCount; ++i) {
    const Eigen::Vector2i delta = (center[i] - query[i]);
    if (delta.lpNorm<Eigen::Infinity>() > radius) {
      return false;
    }
  }
  return true;
}

template <size_t kRobotCount>
FourGrid<kRobotCount> FourGrid<kRobotCount>::RestrictL1Norm(
    const JointPosition<kRobotCount>& center, const int& radius) const {
  std::vector<FourGridDirectedEdge<kRobotCount>> filtered_edges;
  for (const auto& edge : edges) {
    if (InL1(center, radius, edge.from) && InL1(center, radius, edge.to)) {
      filtered_edges.push_back(edge);
    }
  }
  return FourGrid(filtered_edges);
}

template <size_t kRobotCount>
FourGrid<kRobotCount> FourGrid<kRobotCount>::RestrictLInfNorm(
    const JointPosition<kRobotCount>& center, const int& radius) const {
  std::vector<FourGridDirectedEdge<kRobotCount>> filtered_edges;
  for (const auto& edge : edges) {
    if (InLInf(center, radius, edge.from) && InLInf(center, radius, edge.to)) {
      filtered_edges.push_back(edge);
    }
  }
  return FourGrid(filtered_edges);
}

template <size_t kRobotCount>
void FourGrid<kRobotCount>::AugmentPartialGoalsWithSelfLoops(
    const JointPosition<kRobotCount>& goal) {
  for (auto& e : edges) {
    for (size_t i = 0; i < kRobotCount; ++i) {
      if (e.from == goal && e.to == goal) {
        NP_CHECK(e.cost == 0);
        continue;
      }
      if (e.from[i] == goal[i] && e.to[i] == goal[i]) {
        if (e.cost <= 0) {
          LOG(INFO) << ::search::fourgrid::util::JointPositionToString(e.from)
                    << " =" << e.cost << "> "
                    << ::search::fourgrid::util::JointPositionToString(e.to);
        }
        e.cost--;
        NP_CHECK(e.cost >= 0);
      }
    }
  }
  edges.push_back({goal, goal, 0});
}

}  // namespace fourgrid
}  // namespace search

template class search::fourgrid::FourGrid<1>;
template struct search::fourgrid::FourGridDirectedEdge<1>;

template class search::fourgrid::FourGrid<2>;
template struct search::fourgrid::FourGridDirectedEdge<2>;

template class search::fourgrid::FourGrid<3>;
template struct search::fourgrid::FourGridDirectedEdge<3>;
