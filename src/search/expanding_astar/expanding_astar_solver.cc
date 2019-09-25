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

#include "search/expanding_astar/expanding_astar_solver.h"

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <sstream>
#include <string>
#include <limits>
#include <vector>

#include "constants/constants.h"
#include "search.pb.h"
#include "util/array_util.h"
#include "util/serialization.h"

namespace search {
namespace eastar {

template <size_t kRobotCount>
float PositionDelta(const JointPosition<kRobotCount>& current,
                    const JointPosition<kRobotCount>& next) {
  float cost = 0;
  for (size_t i = 0; i < kRobotCount; ++i) {
    cost += (current[i] - next[i]).norm();
  }
  return cost;
}

template <size_t kRobotCount>
std::string PositionToString(const JointPosition<kRobotCount>& jp) {
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
std::string DistanceToString(const JointDistance<kRobotCount>& jd) {
  std::stringstream ss;
  for (size_t i = 0; i < kRobotCount; ++i) {
    ss << "<" << jd[i] << ">";
    if (i < kRobotCount - 1) {
      ss << " | ";
    }
  }
  return ss.str();
}

template <size_t kRobotCount>
bool JointPositionContains(const JointPosition<kRobotCount>& jp,
                           const Eigen::Vector2f& p) {
  for (const auto jpp : jp) {
    if (jpp == p) {
      return true;
    }
  }
  return false;
}

template <size_t kRobotCount>
std::vector<SearchNode<kRobotCount>>
ExpandingAStarSolver<kRobotCount>::GetNeighbors(
    const std::vector<bool>& vertex_enabled,
    const SearchNode<kRobotCount>& node, const JointPosition<kRobotCount>& goal,
    OutOfWindowNodes<kRobotCount>* own) {
  std::vector<SearchNode<kRobotCount>> neighbor_list;
  const graph::general::VertexIndex vi =
      gg_.GetVertexIndex(node.LastPathPosition());

  for (const auto& edge_index : gg_.GetVertex(vi).edge_indices) {
    const auto& edge = gg_.GetEdge(edge_index);

    const auto& non_source_index =
        (edge.index_1 != vi) ? edge.index_1 : edge.index_2;
    NP_CHECK(non_source_index.index < vertex_enabled.size());
    const auto next_position = gg_.GetVertex(non_source_index).position;

    SearchNode<kRobotCount> sn(node, next_position, edge.weight, goal);
    NP_CHECK(non_source_index.index < vertex_enabled.size());
    if (vertex_enabled[non_source_index.index]) {
      neighbor_list.push_back(sn);
    } else {
      own->push_back(sn);
    }
  }
  return neighbor_list;
}

template <size_t kRobotCount>
std::vector<std::pair<JointPosition<kRobotCount>, size_t>>
ExpandingAStarSolver<kRobotCount>::GetNeighborPositionsAndLevel(
    const std::vector<bool>& vertex_enabled,
    const JointPosition<kRobotCount>& current_position,
    const size_t& current_level) {
  const graph::general::VertexIndex vi = gg_.GetVertexIndex(current_position);

  std::vector<std::pair<JointPosition<kRobotCount>, size_t>> neighbor_list;
  for (const auto& edge_index : gg_.GetVertex(vi).edge_indices) {
    const auto& edge = gg_.GetEdge(edge_index);
    const auto& non_source_index =
        (edge.index_1 != vi) ? edge.index_1 : edge.index_2;
    if (vertex_enabled[non_source_index.index]) {
      NP_CHECK(non_source_index.index < vertex_enabled.size());
      neighbor_list.push_back(
          {gg_.GetVertex(non_source_index).position, current_level});
    }
  }

  if (current_level > 0) {
    neighbor_list.push_back({current_position, current_level - 1});
  }

  return neighbor_list;
}

template <size_t kRobotCount>
float Heurisitic(const JointPosition<kRobotCount>& p1,
                 const JointPosition<kRobotCount>& p2) {
  return PositionDelta(p1, p2);
}

template <size_t kRobotCount>
Iteration<kRobotCount>::Iteration(const std::vector<bool>& vertex_enabled,
                                  const std::vector<JointPosition<kRobotCount>>&
                                      path_between_prev_and_current_starts,
                                  const std::vector<JointDistance<kRobotCount>>&
                                      distance_between_prev_and_current_starts,
                                  const JointDistance<kRobotCount>& start_time,
                                  const JointPosition<kRobotCount>& start,
                                  const JointPosition<kRobotCount>& goal)
    : vertex_enabled(vertex_enabled),
      path_cur_prev_starts(path_between_prev_and_current_starts),
      dist_cur_prev_starts(distance_between_prev_and_current_starts),
      start_time(start_time),
      start(start),
      goal(goal) {
  NP_CHECK_EQ(path_between_prev_and_current_starts.size(),
              distance_between_prev_and_current_starts.size());
}
template <size_t kRobotCount>
JointDistance<kRobotCount> Iteration<kRobotCount>::DistanceStartEndDelta()
    const {
  NP_CHECK_EQ(dist_cur_prev_starts.size(), path_cur_prev_starts.size());
  if (dist_cur_prev_starts.size() < 2) {
    return array_util::MakeArray<kRobotCount>(0.0f);
  }

  return array_util::SubtractArrayElements(
      dist_cur_prev_starts[dist_cur_prev_starts.size() - 1],
      dist_cur_prev_starts[0]);
}

template <size_t kRobotCount>
SearchNode<kRobotCount>::SearchNode()
    : path(), distances(), heuristic(0), weighted_self_loops(0) {}

template <size_t kRobotCount>
SearchNode<kRobotCount>::SearchNode(
    const std::vector<JointPosition<kRobotCount>>& path,
    const std::vector<JointDistance<kRobotCount>>& distances,
    const float heuristic)
    : path(path),
      distances(distances),
      heuristic(heuristic),
      weighted_self_loops(0) {
  NP_CHECK_EQ(path.size(), distances.size());
  for (size_t i = 1; i < path.size(); ++i) {
    const JointPosition<kRobotCount>& prev_jp = path[i - 1];
    const JointPosition<kRobotCount>& current_jp = path[i];
    const JointDistance<kRobotCount>& prev_jd = distances[i - 1];
    const JointDistance<kRobotCount>& current_jd = distances[i];
    for (size_t r = 0; r < kRobotCount; ++r) {
      if (prev_jp[r] == current_jp[r] && prev_jd[r] != current_jd[r]) {
        NP_CHECK_EQ(current_jd[r] - prev_jd[r], 1);
        ++weighted_self_loops;
      }
    }
  }
}

template <size_t kRobotCount>
SearchNode<kRobotCount>::SearchNode(
    const JointPosition<kRobotCount>& start_position,
    const JointDistance<kRobotCount>& start_distance,
    const JointPosition<kRobotCount>& goal)
    : path({start_position}),
      distances({start_distance}),
      heuristic(Heurisitic(start_position, goal)),
      weighted_self_loops(0) {}

template <size_t kRobotCount>
SearchNode<kRobotCount>::SearchNode(
    const SearchNode<kRobotCount>& prev,
    const JointPosition<kRobotCount>& additional_step,
    const JointDistance<kRobotCount>& additional_distance,
    const JointPosition<kRobotCount>& goal)
    : path(prev.path),
      distances(prev.distances),
      heuristic(Heurisitic(additional_step, goal)),
      weighted_self_loops(prev.weighted_self_loops) {
  path.push_back(additional_step);
  distances.push_back(
      array_util::AddArrayElements(LastDistance(), additional_distance));
  NP_CHECK_EQ(path.size(), distances.size());
  if (!kProduction) {
    for (size_t i = 0; i < kRobotCount; ++i) {
      NP_CHECK(prev.LastDistance()[i] <= LastDistance()[i]);
    }
  }
  for (size_t r = 0; r < kRobotCount; ++r) {
    if (prev.LastPathPosition()[r] == LastPathPosition()[r] &&
        prev.LastDistance()[r] != LastDistance()[r]) {
      NP_CHECK(fabs(LastDistance()[r] - prev.LastDistance()[r] - 1) < 0.0001f);
      ++weighted_self_loops;
    }
  }
}

template <size_t kRobotCount>
const JointPosition<kRobotCount>& SearchNode<kRobotCount>::LastPathPosition()
    const {
  NP_CHECK(!path.empty());
  return path[path.size() - 1];
}

template <size_t kRobotCount>
const JointDistance<kRobotCount>& SearchNode<kRobotCount>::LastDistance()
    const {
  NP_CHECK(!distances.empty());
  return distances[distances.size() - 1];
}

template <size_t kRobotCount>
const float SearchNode<kRobotCount>::LastDistanceSum() const {
  return array_util::SumArray(LastDistance());
}

template <size_t kRobotCount>
bool SearchNode<kRobotCount>::operator<(
    const SearchNode<kRobotCount>& other) const {
  if (array_util::SumArray(LastDistance()) + heuristic ==
      array_util::SumArray(other.LastDistance()) + other.heuristic) {
    return heuristic > other.heuristic;
  }
  return (array_util::SumArray(LastDistance()) + heuristic >
          array_util::SumArray(other.LastDistance()) + other.heuristic);
}

template <size_t kRobotCount>
bool SearchNode<kRobotCount>::operator==(
    const SearchNode<kRobotCount>& other) const {
  return (LastDistance() == other.LastDistance()) && (path == other.path);
}

template <size_t kRobotCount>
ExpandingAStarSolver<kRobotCount>::ExpandingAStarSolver(
    const GeneralGraph<kRobotCount>& gg)
    : gg_(gg) {}

template <size_t kRobotCount>
std::string TimeToString(const JointDistance<kRobotCount>& jt) {
  std::stringstream ss;
  for (size_t i = 0; i < kRobotCount; ++i) {
    ss << jt[i];
    if (i < kRobotCount - 1) {
      ss << ", ";
    }
  }
  return ss.str();
}

template <size_t kRobotCount>
void ValidateIterations(const GeneralGraph<kRobotCount>& gg,
                        const std::vector<Iteration<kRobotCount>>& iterations) {
  NP_CHECK(!iterations.empty());
  for (const auto& i : iterations) {
    NP_CHECK_EQ(gg.GetNumVertices(), i.vertex_enabled.size());
    gg.GetVertexIndex(i.start);
    gg.GetVertexIndex(i.goal);
  }
}

template <size_t kRobotCount>
std::vector<JointDistance<kRobotCount>> CalculateDistances(
    const std::vector<JointDistance<kRobotCount>>& distances,
    const JointDistance<kRobotCount>& start_time) {
  if (distances.empty()) {
    return {};
  }
  std::vector<JointDistance<kRobotCount>> extended_distances = distances;

  for (auto& d : extended_distances) {
    d = array_util::AddArrayElements(d, start_time);
  }

  NP_CHECK(extended_distances[0] == start_time);
  return extended_distances;
}

template <size_t kRobotCount>
bool ExpandingAStarSolver<kRobotCount>::PathsCollide(
    const SearchNode<kRobotCount>& node,
    const JointDistance<kRobotCount>& start_time) {
  NP_CHECK(kRobotCount == 2);
  static constexpr bool kDebug = false;
  if (node.path.size() <= 1) {
    return false;
  }

  static constexpr float kTimeEpsilon = 0.001;

  const std::vector<JointPosition<kRobotCount>>& path = node.path;
  const std::vector<JointDistance<kRobotCount>> times =
      CalculateDistances(node.distances, start_time);
  NP_CHECK_EQ(path.size(), times.size());

  for (size_t i = 1; i < path.size(); ++i) {
    const Eigen::Vector2f& outer_segment_start = path[i - 1][0];
    const Eigen::Vector2f& outer_segment_end = path[i][0];
    const float& outer_time_start = times[i - 1][0];
    const float& outer_time_end = times[i][0];
    NP_CHECK(outer_time_end >= outer_time_start);

    const bool is_outer_stationary = (outer_segment_start == outer_segment_end);

    if (outer_segment_start == outer_segment_end &&
        outer_segment_start != outer_segment_end) {
      LOG(FATAL) << "Outer times do not match segments";
    }

    for (size_t j = 1; j < path.size(); ++j) {
      const Eigen::Vector2f& inner_segment_start = path[j - 1][1];
      const Eigen::Vector2f& inner_segment_end = path[j][1];
      const float& inner_time_start = times[j - 1][1];
      const float& inner_time_end = times[j][1];
      NP_CHECK(inner_time_end >= inner_time_start);

      const bool is_inner_stationary =
          (inner_segment_start == inner_segment_end);

      if (inner_time_start - kTimeEpsilon > outer_time_end ||
          outer_time_start - kTimeEpsilon > inner_time_end) {
        // No time interval overlap.
        continue;
      }

      if (outer_segment_end == inner_segment_end) {
        return true;
      }

      if (is_outer_stationary && (outer_segment_start == inner_segment_start ||
                                  outer_segment_start == inner_segment_end)) {
        return true;
      }

      if (is_inner_stationary && (outer_segment_start == inner_segment_start ||
                                  outer_segment_end == inner_segment_start)) {
        return true;
      }

      if ((outer_segment_start == inner_segment_start &&
           outer_segment_end == inner_segment_end) ||
          (outer_segment_end == inner_segment_start &&
           outer_segment_start == inner_segment_end)) {
        // Traveling along the same path, either same direction or different
        // direction.

        if (kDebug) {
          LOG(INFO) << "<" << outer_segment_start.x() << ", "
                    << outer_segment_start.y() << "> @ " << outer_time_start
                    << " => <" << outer_segment_end.x() << ", "
                    << outer_segment_end.y() << "> @ " << outer_time_end
                    << " vs <" << inner_segment_start.x() << ", "
                    << inner_segment_start.y() << "> @ " << inner_time_start
                    << " => <" << inner_segment_end.x() << ", "
                    << inner_segment_end.y() << "> @ " << inner_time_end;
        }
        return true;
      }
    }
  }

  return false;
}

template <size_t kRobotCount>
void ExpandingAStarSolver<kRobotCount>::AddOWNStates(
    OpenList<kRobotCount>* open_list, OutOfWindowNodes<kRobotCount>* own,
    const JointDistance<kRobotCount>& start_time, size_t* collision_checks) {
  for (const auto& n : *own) {
    ++(*collision_checks);
    if (kRobotCount <= 1 || !PathsCollide(n, start_time)) {
      open_list->push(n);
    }
  }
  own->clear();
}

template <size_t kRobotCount>
void ExpandingAStarSolver<kRobotCount>::DistanceExtendOpenClosedLists(
    const Iteration<kRobotCount>& iteration, OpenList<kRobotCount>* open_list,
    ClosedList<kRobotCount>* closed_list) {
  if (iteration.dist_cur_prev_starts.empty()) {
    return;
  }
  NP_CHECK(iteration.dist_cur_prev_starts.size() > 1);
  const JointDistance<kRobotCount> new_to_old_start_dist =
      iteration.DistanceStartEndDelta();
  for (SearchNode<kRobotCount>& top : *open_list->GetMutableVector()) {
    top.path.insert(top.path.begin(), iteration.path_cur_prev_starts.begin(),
                    iteration.path_cur_prev_starts.end());
    for (JointDistance<kRobotCount>& distance : top.distances) {
      distance = array_util::AddArrayElements(distance, new_to_old_start_dist);
    }
    top.distances.insert(top.distances.begin(),
                         iteration.dist_cur_prev_starts.begin(),
                         iteration.dist_cur_prev_starts.end());
  }

  for (auto& pair : *closed_list) {
    for (auto& distance : pair.second.distances) {
      distance = array_util::AddArrayElements(distance, new_to_old_start_dist);
    }
  }
}

template <size_t kRobotCount>
void ExpandingAStarSolver<kRobotCount>::AddPathBetweenStarts(
    const Iteration<kRobotCount>& iteration, OpenList<kRobotCount>* open_list) {
  std::vector<JointPosition<kRobotCount>> jp_sofar;
  std::vector<JointDistance<kRobotCount>> jd_sofar;
  NP_CHECK_EQ(iteration.path_cur_prev_starts.size(),
              iteration.dist_cur_prev_starts.size());
  for (size_t i = 0; i < iteration.path_cur_prev_starts.size(); ++i) {
    const JointPosition<kRobotCount>& jp = iteration.path_cur_prev_starts[i];
    const JointDistance<kRobotCount>& jd = iteration.dist_cur_prev_starts[i];
    jp_sofar.push_back(jp);
    jd_sofar.push_back(jd);
    open_list->push({jp_sofar, jd_sofar, Heurisitic(jp, iteration.goal)});
  }
}

template <size_t kRobotCount>
void ExpandingAStarSolver<kRobotCount>::ChangeOpenListGoal(
    const Iteration<kRobotCount>& iteration, OpenList<kRobotCount>* open_list) {
  for (SearchNode<kRobotCount>& top : *open_list->GetMutableVector()) {
    top.heuristic = Heurisitic(top.LastPathPosition(), iteration.goal);
  }
  open_list->RebuildHeap();
}

template <size_t kRobotCount>
bool FlagNode(const SearchNode<kRobotCount>& next_node) {
  if (kRobotCount != 2) {
    return false;
  }
  JointPosition<kRobotCount> positions;
  positions[0] = Eigen::Vector2f(2, 1);
  positions[1] = Eigen::Vector2f(4, 0);
  JointDistance<kRobotCount> distances;
  distances[0] = 3;
  distances[1] = 0;
  if (next_node.LastPathPosition() == positions &&
      next_node.LastDistance() == distances) {
    //     NP_CHECK_EQ(next_node.path.size(), next_node.distances.size());
    //     for (size_t i = 0; i < next_node.path.size(); ++i) {
    //       const JointPosition<kRobotCount>& p = next_node.path[i];
    //       const JointDistance<kRobotCount>& d = next_node.distances[i];
    //       LOG(INFO) << PositionToString(p) << " @ " << DistanceToString(d);
    //     }
    //     LOG(ERROR) << "Found!";
    return true;
  }
  return false;
}

template <size_t kRobotCount>
void ExpandingAStarSolver<kRobotCount>::AStarRepair(
    const std::vector<bool>& vertex_enabled, const float minimum_f_value,
    const JointDistance<kRobotCount>& start_time,
    const JointPosition<kRobotCount>& goal, OpenList<kRobotCount>* open_list,
    ClosedList<kRobotCount>* closed_list, OutOfWindowNodes<kRobotCount>* own,
    size_t* expansions, size_t* collision_checks) {
  while (!open_list->empty()) {
    const SearchNode<kRobotCount> top = open_list->top();
    if ((top.LastDistanceSum() + top.heuristic) > minimum_f_value) {
      break;
    }
    open_list->pop();

    const auto insert_attempt =
        closed_list->insert({{top.LastPathPosition(), top.weighted_self_loops},
                             top.LastDistance()});
    if (!insert_attempt.second &&
        array_util::SumArray(insert_attempt.first->second) <=
            top.LastDistanceSum()) {
      // Already in closed list with smaller value, should be ignored.
      continue;
    }

    // Force position into closed list with new weight.
    (*closed_list)[{top.LastPathPosition(), top.weighted_self_loops}] =
        top.LastDistance();

    ++(*expansions);

    for (const SearchNode<kRobotCount>& next_node :
         GetNeighbors(vertex_enabled, top, goal, own)) {
      const auto find_result = closed_list->find(
          {next_node.LastPathPosition(), next_node.weighted_self_loops});
      if (find_result != closed_list->end() &&
          array_util::SumArray(find_result->second) <=
              next_node.LastDistanceSum()) {
        // Already in closed list with smaller value, should be ignored.
        continue;
      }
      ++(*collision_checks);
      if (kRobotCount <= 1 || !PathsCollide(next_node, start_time)) {
        FlagNode(next_node);
        open_list->push(next_node);
      }
    }
  }
}

template <size_t kRobotCount>
void DumpSearchTree(const std::string& file_name,
                    OpenList<kRobotCount> open_list,
                    const SearchNode<kRobotCount>& top,
                    ClosedList<kRobotCount> closed_list) {
  MinuteBotsProto::SimpleSearchTree simple_search_tree;

  for (const auto& closed_pair : closed_list) {
    auto* closed_state_proto = simple_search_tree.add_closed_list();
    auto* position_proto = closed_state_proto->mutable_position();
    auto* p1_proto = position_proto->mutable_p1();
    auto* p2_proto = position_proto->mutable_p2();
    p1_proto->set_x(closed_pair.first.first[0].x());
    p1_proto->set_y(closed_pair.first.first[0].y());
    p2_proto->set_x(closed_pair.first.first[1].x());
    p2_proto->set_y(closed_pair.first.first[1].y());
    closed_state_proto->set_weight(closed_pair.second);
  }

  {
    auto* search_node_proto = simple_search_tree.add_open_list();
    search_node_proto->set_weight(top.LastDistanceSum());
    search_node_proto->set_heuristic(top.heuristic);
    for (const auto& p : top.path) {
      auto* path_proto = search_node_proto->add_path();
      auto* p1_proto = path_proto->mutable_p1();
      auto* p2_proto = path_proto->mutable_p2();
      p1_proto->set_x(p[0].x());
      p1_proto->set_y(p[0].y());
      p2_proto->set_x(p[1].x());
      p2_proto->set_y(p[1].y());
    }
  }

  while (!open_list.empty()) {
    const auto top = open_list.top();
    open_list.pop();
    auto* search_node_proto = simple_search_tree.add_open_list();
    search_node_proto->set_weight(top.LastDistanceSum());
    search_node_proto->set_heuristic(top.heuristic);
    for (const auto& p : top.path) {
      auto* path_proto = search_node_proto->add_path();
      auto* p1_proto = path_proto->mutable_p1();
      auto* p2_proto = path_proto->mutable_p2();
      p1_proto->set_x(p[0].x());
      p1_proto->set_y(p[0].y());
      p2_proto->set_x(p[1].x());
      p2_proto->set_y(p[1].y());
    }
  }

  int file_descriptor =
      ::util::serialization::CreateOrEraseFileForWrite(file_name);
  google::protobuf::io::FileOutputStream file_output(file_descriptor);
  file_output.SetCloseOnDelete(true);
  if (!google::protobuf::TextFormat::Print(simple_search_tree, &file_output)) {
    LOG(ERROR) << "Failed to write search tree proto to " << file_name;
  }
}

template <size_t kRobotCount>
PathReturnType<kRobotCount> ExpandingAStarSolver<kRobotCount>::AStarContinued(
    const Iteration<kRobotCount>& iteration, OpenList<kRobotCount>* open_list,
    ClosedList<kRobotCount>* closed_list, OutOfWindowNodes<kRobotCount>* own,
    size_t* expansions, size_t* collision_checks) {
  while (!open_list->empty()) {
    const SearchNode<kRobotCount> top = open_list->top();
    open_list->pop();

    const bool flag_node = FlagNode(top);

    if (!closed_list
             ->insert({{top.LastPathPosition(), top.weighted_self_loops},
                       top.LastDistance()})
             .second) {
      // Already in closed list, should be ignored.
      continue;
    }
    ++(*expansions);

    //     DumpSearchTree("sst" + std::to_string(*expansions) + "result.proto",
    //                    *open_list, top, *closed_list);

    if (flag_node) {
      LOG(INFO) << "FLAGGED";
      LOG(INFO) << "Position: " << PositionToString(top.LastPathPosition())
                << " @ " << DistanceToString(top.LastDistance());
    }

    for (const SearchNode<kRobotCount>& next_node :
         GetNeighbors(iteration.vertex_enabled, top, iteration.goal, own)) {
      if (flag_node) {
        LOG(INFO) << "Neighbor: "
                  << PositionToString(next_node.LastPathPosition()) << " @ "
                  << DistanceToString(next_node.LastDistance());
      }

      if (closed_list->find(
              {next_node.LastPathPosition(), next_node.weighted_self_loops}) !=
          closed_list->end()) {
        if (flag_node) {
          LOG(INFO) << "In closed list";
        }
        continue;
      }
      ++(*collision_checks);
      if (kRobotCount <= 1 || !PathsCollide(next_node, iteration.start_time)) {
        FlagNode(next_node);
        open_list->push(next_node);
        if (flag_node) {
          LOG(INFO) << "Accepted";
        }
      } else {
        if (flag_node) {
          LOG(INFO) << "Collide";
        }
      }
    }

    if (top.LastPathPosition() == iteration.goal) {
      // Ensure that the goal position will stay on the open list for reference.
      open_list->push(top);
      return {top.path, top.distances};
    }
  }

  return {{}, {}};
}

template <size_t kRobotCount>
void VerifyOWN(const OutOfWindowNodes<kRobotCount>& own,
               const Iteration<kRobotCount>& prev_iteration,
               const GeneralGraph<kRobotCount>& gg) {
  for (const SearchNode<kRobotCount>& n : own) {
    NP_CHECK(!n.path.empty());

    const JointPosition<kRobotCount>& first_jp = n.path[0];
    NP_CHECK(first_jp == prev_iteration.start);

    // Check that all but the last element of the path is in window.
    for (size_t i = 0; i < n.path.size() - 1; ++i) {
      const JointPosition<kRobotCount>& jp = n.path[i];
      const auto vertex_index = gg.GetVertexIndex(jp);
      NP_CHECK(prev_iteration.vertex_enabled[vertex_index.index]);
    }

    // Check that the last element of the path is out of window.
    const JointPosition<kRobotCount>& last_jp = n.path[n.path.size() - 1];
    const auto vertex_index = gg.GetVertexIndex(last_jp);
    NP_CHECK(!prev_iteration.vertex_enabled[vertex_index.index]);
  }
}

template <size_t kRobotCount>
void ExpandingAStarSolver<kRobotCount>::VerifyOpenList(
    const OpenList<kRobotCount>& ol, const JointDistance<kRobotCount>& start) {
  for (const auto& n : ol.GetVector()) {
    NP_CHECK(kRobotCount <= 1 || !PathsCollide(n, start));
  }
}

template <size_t kRobotCount>
void PrintPath(const PathReturnType<kRobotCount>& result) {
  for (size_t i = 0; i < result.first.size(); ++i) {
    const JointPosition<kRobotCount>& jp = result.first[i];
    const JointDistance<kRobotCount>& jd = result.second[i];
    LOG(INFO) << PositionToString(jp) << " @ " << DistanceToString(jd);
  }
}

template <size_t kRobotCount>
std::pair<bool, size_t> FindGoalInClosedList(
    const ClosedList<kRobotCount>& closed_list,
    const Iteration<kRobotCount>& iteration,
    const float maximum_existing_f_value) {
  NP_CHECK(maximum_existing_f_value >= 0.0f);
  const size_t maximum_self_loops =
      static_cast<size_t>(std::ceil(maximum_existing_f_value));
  // Find the minimum self loop value that has a goal, where we know the
  // upperbound on the need to check.
  bool found_i = false;
  size_t best_i = 0;
  float best_i_cost = std::numeric_limits<float>::max();
  for (size_t i = 0; i < maximum_self_loops; ++i) {
    const auto find_result = closed_list.find({iteration.goal, i});
    if (find_result != closed_list.end()) {
      LOG(INFO) << i << ": found! Cost: "
                << array_util::SumArray(find_result->second);
      found_i = true;
      if (array_util::SumArray(find_result->second) < best_i_cost) {
        best_i = i;
        best_i_cost = array_util::SumArray(find_result->second);
      }
    } else {
      LOG(INFO) << i << ": not found!";
    }
  }

  return {found_i, best_i};
}

template <size_t kRobotCount>
std::pair<bool, PathReturnType<kRobotCount>>
ExpandingAStarSolver<kRobotCount>::ExtractPossiblePathInClosedList(
    const ClosedList<kRobotCount>& closed_list,
    const Iteration<kRobotCount>& iteration,
    const float maximum_existing_f_value) {
  const auto find_goal_result =
      FindGoalInClosedList(closed_list, iteration, maximum_existing_f_value);
  // No goal found!
  if (!find_goal_result.first) {
    return {false, {{}, {}}};
  }

  LOG(INFO) << "Optimal level: " << find_goal_result.second;

  // Perform unwind.
  std::vector<JointPosition<kRobotCount>> positions;
  std::vector<JointDistance<kRobotCount>> distances;
  JointPosition<kRobotCount> current_position = iteration.goal;
  size_t current_level = find_goal_result.second;
  JointDistance<kRobotCount> current_distance =
      closed_list.find({current_position, current_level})->second;
  while (current_position != iteration.start) {
    positions.push_back(current_position);
    distances.push_back(current_distance);

    bool found_any_neighbor = false;
    JointPosition<kRobotCount> cheapest_neighbor = current_position;
    JointDistance<kRobotCount> cheapest_neighbor_distance =
        array_util::MakeArray<kRobotCount>(std::numeric_limits<float>::max());
    size_t cheapest_neighbor_level = current_level;
    for (const std::pair<JointPosition<kRobotCount>, size_t>&
             neighbor_level_pair : GetNeighborPositionsAndLevel(
                 iteration.vertex_enabled, current_position, current_level)) {
      const auto find_result = closed_list.find(
          {neighbor_level_pair.first, neighbor_level_pair.second});
      if (find_result != closed_list.end()) {
        found_any_neighbor = true;
        if (array_util::SumArray(cheapest_neighbor_distance) >
            array_util::SumArray(find_result->second)) {
          cheapest_neighbor = find_result->first.first;
          cheapest_neighbor_distance = find_result->second;
          cheapest_neighbor_level = neighbor_level_pair.second;
        }
      }
    }
    if (!found_any_neighbor) {
      LOG(FATAL) << "Unable to complete unwind, closed list broken!";
    }

    current_position = cheapest_neighbor;
    current_level = cheapest_neighbor_level;
    current_distance = cheapest_neighbor_distance;
  }

  positions.push_back(iteration.start);
  distances.push_back(array_util::MakeArray<kRobotCount>(0.0f));

  std::reverse(positions.begin(), positions.end());
  std::reverse(distances.begin(), distances.end());

  return {true, {positions, distances}};
}

template <size_t kRobotCount>
PathReturnType<kRobotCount>
ExpandingAStarSolver<kRobotCount>::SolveExpandingAStar(
    const std::vector<Iteration<kRobotCount>>& iterations) {
  if (!kProduction) {
    ValidateIterations(gg_, iterations);
  }
  PathReturnType<kRobotCount> result;

  OpenList<kRobotCount> open_list;
  ClosedList<kRobotCount> closed_list;
  OutOfWindowNodes<kRobotCount> own;
  Iteration<kRobotCount> prev_iteration = iterations[0];
  open_list.push({prev_iteration.start,
                  array_util::MakeArray<kRobotCount>(0.0f),
                  prev_iteration.goal});

  size_t expansions = 0;
  size_t collision_checks = 0;
  size_t itr = 0;
  for (const Iteration<kRobotCount>& iteration : iterations) {
    LOG(INFO) << "Expanding A* start position: "
              << PositionToString(iteration.start);
    LOG(INFO) << "Expanding A* goal position: "
              << PositionToString(iteration.goal);
    const float prev_open_list_top_f_value =
        open_list.top().LastDistanceSum() + open_list.top().heuristic;
    VerifyOWN(own, prev_iteration, gg_);
    AddOWNStates(&open_list, &own, prev_iteration.start_time,
                 &collision_checks);

    VerifyOpenList(open_list, iteration.start_time);

    //     DumpSearchTree("sst" + std::to_string(itr) + "pre11.proto",
    //     open_list,
    //                    closed_list);

    AStarRepair(iteration.vertex_enabled, prev_open_list_top_f_value,
                prev_iteration.start_time, prev_iteration.goal, &open_list,
                &closed_list, &own, &expansions, &collision_checks);

    //     DumpSearchTree("sst" + std::to_string(itr) + "post11.proto",
    //     open_list,
    //                    closed_list);

    DistanceExtendOpenClosedLists(iteration, &open_list, &closed_list);
    AddPathBetweenStarts(iteration, &open_list);

    //     DumpSearchTree("sst" + std::to_string(itr) + "pre12.proto",
    //     open_list,
    //                    closed_list);

    //     VerifyOpenList(open_list, iteration.start_time);

    AStarRepair(iteration.vertex_enabled,
                prev_open_list_top_f_value +
                    array_util::SumArray(iteration.DistanceStartEndDelta()),
                iteration.start_time, prev_iteration.goal, &open_list,
                &closed_list, &own, &expansions, &collision_checks);

    //     VerifyOpenList(open_list, iteration.start_time);

    ChangeOpenListGoal(iteration, &open_list);

    //     VerifyOpenList(open_list, iteration.start_time);

    //     DumpSearchTree("sst" + std::to_string(itr) + "post12.proto",
    //     open_list,
    //                    closed_list);

    const auto extract_attempt = ExtractPossiblePathInClosedList(
        closed_list, iteration,
        prev_open_list_top_f_value +
            array_util::SumArray(iteration.DistanceStartEndDelta()));

    if (extract_attempt.first) {
      LOG(INFO) << "Closed list extract";
      result = extract_attempt.second;
    } else {
      LOG(INFO) << "A Star extract";
      result = AStarContinued(iteration, &open_list, &closed_list, &own,
                              &expansions, &collision_checks);
    }

    //     DumpSearchTree("sst" + std::to_string(itr) + "result.proto",
    //     open_list,
    //                    closed_list);

    LOG(INFO) << "Iter result:";
    if (result.first.empty()) {
      LOG(ERROR) << "No path found!";
    }
    PrintPath(result);

    prev_iteration = iteration;
    ++itr;
  }
  LOG(INFO) << "Expansions: " << expansions;
  LOG(INFO) << "Collision checks: " << collision_checks;
  return result;
}

template <size_t kRobotCount>
PathReturnType<kRobotCount> ExpandingAStarSolver<kRobotCount>::SolveAStar(
    const std::vector<Iteration<kRobotCount>>& iterations) {
  if (!kProduction) {
    ValidateIterations(gg_, iterations);
  }
  PathReturnType<kRobotCount> result;
  size_t expansions = 0;
  size_t collision_checks = 0;
  for (const Iteration<kRobotCount>& iteration : iterations) {
    LOG(INFO) << "Standard A* start position: "
              << PositionToString(iteration.start);
    LOG(INFO) << "Standard A* goal position:  "
              << PositionToString(iteration.goal);
    OpenList<kRobotCount> open_list;
    ClosedList<kRobotCount> closed_list;
    OutOfWindowNodes<kRobotCount> own;
    open_list.push({iteration.start, array_util::MakeArray<kRobotCount>(0.0f),
                    iteration.goal});

    result = AStarContinued(iteration, &open_list, &closed_list, &own,
                            &expansions, &collision_checks);

    LOG(INFO) << "Iter result:";
    if (result.first.empty()) {
      LOG(FATAL) << "No path found!";
    }
    PrintPath(result);
  }
  LOG(INFO) << "Expansions: " << expansions;
  LOG(INFO) << "Collision checks: " << collision_checks;
  return result;
}

}  // namespace eastar
}  // namespace search

// Explicit instantiation of template classes minimizes redundant compilation,
// separates declarations from definitions, and prevents use of the class in
// ways other than those that have been tested.

template struct search::eastar::PositionAndWeightedSelfLoopsHasher<1>;
template struct search::eastar::Iteration<1>;
template class search::eastar::ExpandingAStarSolver<1>;

template struct search::eastar::PositionAndWeightedSelfLoopsHasher<2>;
template struct search::eastar::Iteration<2>;
template class search::eastar::ExpandingAStarSolver<2>;
