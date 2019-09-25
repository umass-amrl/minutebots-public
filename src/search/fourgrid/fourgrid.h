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

#ifndef SRC_SEARCH_FOURGRID_FOURGRID_H_
#define SRC_SEARCH_FOURGRID_FOURGRID_H_

#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "constants/constants.h"

namespace search {
namespace fourgrid {

template <size_t kRobotCount>
using JointPosition = std::array<Eigen::Vector2i, kRobotCount>;

template <size_t kRobotCount>
using Path = std::vector<JointPosition<kRobotCount>>;

template <size_t kRobotCount>
using CostPath = std::vector<std::pair<JointPosition<kRobotCount>, int>>;

template <size_t kRobotCount>
struct JointPositionHasher {
  std::size_t operator()(const JointPosition<kRobotCount>& jp) const {
    size_t i = 1;
    for (const auto& p : jp) {
      i *= p.dot(Eigen::Vector2i(7, 13));
    }
    return i;
  }
};

template <size_t kRobotCount>
struct FourGridDirectedEdge {
  JointPosition<kRobotCount> from;
  JointPosition<kRobotCount> to;
  int cost;
  FourGridDirectedEdge(const JointPosition<kRobotCount>& from,
                       const JointPosition<kRobotCount>& to, const int& cost);

  bool operator==(const FourGridDirectedEdge& other) const;
};

template <size_t kRobotCount>
struct FourGridDirectedEdgeHasher {
  std::size_t operator()(const FourGridDirectedEdge<kRobotCount>& jpp) const {
    size_t i = 1;
    for (const auto& p : jpp.from) {
      i *= p.dot(Eigen::Vector2i(7, 13));
    }
    for (const auto& p : jpp.to) {
      i *= p.dot(Eigen::Vector2i(2, 5));
    }
    i *= jpp.cost;
    return i;
  }
};

template <size_t kRobotCount>
class FourGrid {
 public:
  std::vector<FourGridDirectedEdge<kRobotCount>> edges;

  FourGrid() = default;
  explicit FourGrid(const std::string& file_name);
  explicit FourGrid(
      const std::vector<FourGridDirectedEdge<kRobotCount>>& edges);

  std::vector<FourGridDirectedEdge<kRobotCount>> GetEdges(
      const JointPosition<kRobotCount>& position) const;

  std::vector<FourGridDirectedEdge<kRobotCount>> GetBackwardEdges(
      const JointPosition<kRobotCount>& position) const;

  std::vector<FourGridDirectedEdge<kRobotCount>> GetUniqueEdges(
      const FourGrid<kRobotCount>& other) const;

  std::vector<JointPosition<kRobotCount>> GetPositions() const;

  std::vector<JointPosition<kRobotCount>> GetUniquePositions(
      const FourGrid<kRobotCount>& other) const;

  FourGrid<kRobotCount> RestrictL1Norm(const JointPosition<kRobotCount>& center,
                                       const int& radius) const;
  FourGrid<kRobotCount> RestrictLInfNorm(
      const JointPosition<kRobotCount>& center, const int& radius) const;

  void AugmentPartialGoalsWithSelfLoops(const JointPosition<kRobotCount>& goal);
};

namespace util {

template <size_t kRobotCount>
std::string JointPositionToString(const JointPosition<kRobotCount>& jp) {
  std::stringstream ss;
  for (size_t i = 0; i < kRobotCount; ++i) {
    ss << "<" << jp[i].x() << ", " << jp[i].y() << ">";
    if (i < kRobotCount - 1) {
      ss << " | ";
    }
  }
  return ss.str();
}

template <size_t kRobotCount>
std::string FourGridDirectedEdgeToString(
    const FourGridDirectedEdge<kRobotCount>& edge) {
  std::stringstream ss;
  ss << JointPositionToString(edge.from) << " =" << edge.cost << "> "
     << JointPositionToString(edge.to);
  return ss.str();
}

template <size_t kRobotCount>
JointPosition<kRobotCount> FileToJointPosition(const std::string& file_name) {
  std::ifstream infile(file_name);
  std::string line;
  NP_CHECK_MSG(std::getline(infile, line),
               "Failed to read joint position from file " << file_name);
  std::istringstream iss(line);

  JointPosition<kRobotCount> pos;

  for (size_t i = 0; i < kRobotCount; ++i) {
    char robot;
    int x, y;
    if (!(iss >> robot >> x >> y)) {
      LOG(FATAL) << "Unable to find all requested robot positions! String: "
                 << line;
    }
    pos[i] = {x, y};
  }

  return pos;
}

template <size_t kRobotCount>
bool JointPositionCollides(const JointPosition<kRobotCount>& jp) {
  for (size_t i = 0; i < kRobotCount; ++i) {
    for (size_t j = 0; j < kRobotCount; ++j) {
      if (i == j) {
        continue;
      }
      if (jp[i] == jp[j]) {
        return true;
      }
    }
  }
  return false;
}

template <size_t kRobotCount>
bool JointEdgeCollides(const FourGridDirectedEdge<kRobotCount>& edge) {
  if (edge.cost <= 0) {
    return false;
  }
  for (size_t i = 0; i < kRobotCount; ++i) {
    for (size_t j = 0; j < kRobotCount; ++j) {
      if (i == j) {
        continue;
      }
      if (JointPositionCollides(edge.from) || JointPositionCollides(edge.to)) {
        return true;
      }
      if (edge.from[i] == edge.to[j] && edge.from[j] == edge.to[i]) {
        return true;
      }
    }
  }
  return false;
}

template <size_t kRobotCount>
std::pair<bool, FourGridDirectedEdge<kRobotCount>> CostPathCollides(
    const CostPath<kRobotCount>& path) {
  NP_CHECK(!path.empty())
  for (size_t i = 1; i < path.size(); ++i) {
    const FourGridDirectedEdge<kRobotCount> edge(
        path[i - 1].first, path[i].first, path[i].second - path[i - 1].second);
    if (JointEdgeCollides(edge)) {
      return {true, edge};
    }
  }
  return {false, {path[0].first, path[0].first, 0}};
}

template <size_t kRobotCount>
bool PositionInGrid(const JointPosition<kRobotCount>& jp,
                    const FourGrid<kRobotCount> four_grid) {
  for (const auto& edge : four_grid.edges) {
    if (edge.from == jp || edge.to == jp) {
      return true;
    }
  }
  return false;
}

template <size_t kRobotCount>
std::pair<bool, JointPosition<kRobotCount>> CostPathEntrance(
    const CostPath<kRobotCount>& path, const FourGrid<kRobotCount> four_grid) {
  NP_CHECK(!path.empty());
  for (const auto& jp_pair : path) {
    const auto& jp = jp_pair.first;
    if (PositionInGrid(jp, four_grid)) {
      return {true, jp};
    }
  }
  return {false, path[0].first};
}

template <size_t kRobotCount>
std::pair<bool, JointPosition<kRobotCount>> CostPathExit(
    const CostPath<kRobotCount>& path, const FourGrid<kRobotCount> four_grid) {
  NP_CHECK(!path.empty());
  for (int i = static_cast<int>(path.size() - 1); i >= 0; --i) {
    const auto& jp = path[i].first;
    if (PositionInGrid(jp, four_grid)) {
      return {true, jp};
    }
  }
  return {false, path[0].first};
}

template <size_t kRobotCount>
JointPosition<kRobotCount> CostPathCenter(const CostPath<kRobotCount>& path) {
  NP_CHECK(!path.empty())
  std::vector<size_t> indices;
  for (size_t i = 1; i < path.size(); ++i) {
    const FourGridDirectedEdge<kRobotCount> edge(
        path[i - 1].first, path[i].first, path[i].second - path[i - 1].second);
    if (JointEdgeCollides(edge)) {
      indices.push_back(i);
    }
  }
  NP_CHECK_MSG(!indices.empty(), "No collisions found!");
  float sum = 0;
  for (const auto& i : indices) {
    sum += i;
  }
  return path[static_cast<size_t>(sum / static_cast<float>(indices.size()))]
      .first;
}

template <size_t kRobotCount>
JointPosition<kRobotCount> GridChangeCollideCenter(
    const CostPath<kRobotCount>& path, const FourGrid<kRobotCount>& old_grid,
    const FourGrid<kRobotCount>& new_grid) {
  NP_CHECK(!path.empty());
  NP_CHECK(!old_grid.edges.empty());
  NP_CHECK(!new_grid.edges.empty());

  // Get all the positions that were in the old grid but not in the new grid.
  const auto unique_positions = old_grid.GetUniquePositions(new_grid);
  NP_CHECK(!unique_positions.empty());
  for (const auto& position : unique_positions) {
    for (const auto& p : path) {
      if (p.first == position) {
        return p.first;
      }
    }
  }

  NP_CHECK_MSG(false, "No relevant change found!");
  return {};
}

template <size_t kRobotCount>
CostPath<kRobotCount> PathBetweenStarts(
    const CostPath<kRobotCount>& cost_path,
    const JointPosition<kRobotCount>& larger_start,
    const JointPosition<kRobotCount>& smaller_start) {
  Path<kRobotCount> path(cost_path.size());
  std::transform(
      cost_path.begin(), cost_path.end(), path.begin(),
      [](const std::pair<JointPosition<kRobotCount>, int>& pair_pos) {
        return pair_pos.first;
      });
  NP_CHECK_EQ(path.size(), cost_path.size());
  NP_CHECK(std::find(path.begin(), path.end(), larger_start) != path.end());
  NP_CHECK(std::find(path.begin(), path.end(), smaller_start) != path.end());

  size_t start_idx = 0;
  size_t end_idx = 0;
  for (size_t i = 0; i < cost_path.size(); ++i) {
    if (cost_path[i].first == larger_start) {
      start_idx = i;
    }
    if (cost_path[i].first == smaller_start) {
      end_idx = i;
    }
  }
  NP_CHECK(start_idx <= end_idx);
  CostPath<kRobotCount> result_path(cost_path.begin() + start_idx,
                                    cost_path.begin() + end_idx + 1);
  if (start_idx == end_idx) {
    result_path.push_back(*(cost_path.begin() + start_idx));
  }
  NP_CHECK(result_path.size() > 1);
  const int min_cost = result_path[0].second;
  for (auto& p : result_path) {
    p.second -= min_cost;
    NP_CHECK(p.second >= 0);
  }
  return result_path;
}

template <size_t kRobotCount>
CostPath<kRobotCount> PathBetweenGoals(
    const CostPath<kRobotCount>& cost_path,
    const JointPosition<kRobotCount>& larger_goal,
    const JointPosition<kRobotCount>& smaller_goal) {
  Path<kRobotCount> path(cost_path.size());
  std::transform(
      cost_path.begin(), cost_path.end(), path.begin(),
      [](const std::pair<JointPosition<kRobotCount>, int>& pair_pos) {
        return pair_pos.first;
      });
  NP_CHECK_EQ(path.size(), cost_path.size());
  NP_CHECK(std::find(path.begin(), path.end(), larger_goal) != path.end());
  NP_CHECK(std::find(path.begin(), path.end(), smaller_goal) != path.end());

  size_t start_idx = 0;
  size_t end_idx = 0;
  for (size_t i = 0; i < cost_path.size(); ++i) {
    if (cost_path[i].first == smaller_goal) {
      start_idx = i;
    }
    if (cost_path[i].first == larger_goal) {
      end_idx = i;
    }
  }
  NP_CHECK(start_idx <= end_idx);
  NP_CHECK(end_idx < cost_path.size());

  if (start_idx == end_idx) {
    return {};
  }

  CostPath<kRobotCount> result_path(cost_path.begin() + start_idx,
                                    cost_path.begin() + end_idx + 1);
  if (start_idx == end_idx) {
    result_path.push_back(*(cost_path.begin() + start_idx));
  }
  NP_CHECK(result_path.size() > 1);
  const int min_cost = result_path[0].second;
  for (auto& p : result_path) {
    p.second -= min_cost;
    NP_CHECK(p.second >= 0);
  }
  return result_path;
}

}  // namespace util
}  // namespace fourgrid
}  // namespace search

#endif  // SRC_SEARCH_FOURGRID_FOURGRID_H_
