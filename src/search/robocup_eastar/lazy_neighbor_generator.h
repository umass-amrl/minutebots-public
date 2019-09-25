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

#ifndef SRC_SEARCH_ROBOCUP_EASTAR_LAZY_NEIGHBOR_GENERATOR_H_
#define SRC_SEARCH_ROBOCUP_EASTAR_LAZY_NEIGHBOR_GENERATOR_H_

#include <limits>

#include "datastructures/dense_array.h"
#include "navigation/production/collision_grid.h"
#include "search/robocup_eastar/common_defines.h"
#include "search/robocup_eastar/search_node.h"
#include "search/robocup_eastar/search_window.h"

namespace search {
namespace eastar {
namespace lazy_neighbors {

static constexpr size_t kNumNeighbors = 6;

struct Neighbor2d {
  GridVertex neighbor;
  MovingHaltedSteps steps;
  bool in_window;
  Neighbor2d() : neighbor(0, 0), steps(), in_window(true) {}
  Neighbor2d(const GridVertex& neighbor, const MovingHaltedSteps& steps,
             const bool in_window)
      : neighbor(neighbor), steps(steps), in_window(in_window) {}
};

using Neighbors2d = datastructures::DenseArray<Neighbor2d, kNumNeighbors>;

template <size_t kMaxRobotCount>
using NeighborLazyStorage =
    datastructures::TransactedVectorPriorityQueue<SearchNode<kMaxRobotCount>>;

template <size_t kMaxRobotCount>
class NeighborGenerator {
 private:
  datastructures::DenseArray<Neighbors2d, kMaxRobotCount> neighbors_array_;
  NeighborLazyStorage<kMaxRobotCount> lazy_storage_;
  bool needs_lazy_storage_rebuild_;
  bool has_generated_neighbors_;
  SearchNode<kMaxRobotCount> parent_search_node_;
  SearchNode<kMaxRobotCount> smallest_cost_neighbor_;
  JointPosition<kMaxRobotCount> goal_position_;
  float heuristic_inflation_;
  const IndividuallyPlannedDataSlice<kMaxRobotCount>* slice_;
  const SearchWindow<kMaxRobotCount>* search_window_;

  // Computes the individual neighbors for each of the agents.
  void ComputeNeighborsArray(const SearchNode<kMaxRobotCount>& top);

  // Computes the smallest search node which will be used to compare this
  // generator to other generators.
  void ComputeSmallestCostNeighbor();

  void GenerateFullNeighbors();

  Neighbors2d GetNeighbors2d(const GridVertex& p, const GridVertex& goal,
                             const MovingHaltedSteps& dist,
                             const OurRobotIndex& robot_index);

 public:
  NeighborGenerator() = delete;
  NeighborGenerator(const SearchNode<kMaxRobotCount>& parent_search_node,
                    const JointPosition<kMaxRobotCount>& goal_position,
                    const float heuristic_inflation,
                    const IndividuallyPlannedDataSlice<kMaxRobotCount>* slice,
                    const SearchWindow<kMaxRobotCount>* search_window)
      : needs_lazy_storage_rebuild_(false),
        has_generated_neighbors_(false),
        parent_search_node_(parent_search_node),
        goal_position_(goal_position),
        heuristic_inflation_(heuristic_inflation),
        slice_(slice),
        search_window_(search_window) {
    ComputeNeighborsArray(parent_search_node_);
    ComputeSmallestCostNeighbor();
  }

  NeighborGenerator(const NeighborGenerator<kMaxRobotCount>& o) = delete;
  NeighborGenerator(NeighborGenerator<kMaxRobotCount>&& o) = delete;

  void operator=(const NeighborGenerator<kMaxRobotCount>& o) const = delete;
  void operator=(NeighborGenerator<kMaxRobotCount>&& o) const = delete;

  NeighborGenerator(const SearchNode<kMaxRobotCount>& single_node,
                    const JointPosition<kMaxRobotCount>& goal_position)
      : lazy_storage_(),
        needs_lazy_storage_rebuild_(false),
        has_generated_neighbors_(true),
        parent_search_node_(single_node),
        smallest_cost_neighbor_(single_node),
        goal_position_(goal_position),
        heuristic_inflation_(std::numeric_limits<float>::max()),
        slice_(nullptr),
        search_window_(nullptr) {}

  // Returns the smallest search node from the generator.
  SearchNode<kMaxRobotCount> GetMinimum() const;

  // Moves to the next element in the generator. Returns true if another element
  // exists, false otherwise.
  bool GenerateNext();

  void UpdateGoal(const JointPosition<kMaxRobotCount>& goal,
                  const float inflation);

  bool operator<(const NeighborGenerator<kMaxRobotCount>& other) const {
    return GetMinimum() < other.GetMinimum();
  }
};
}  // namespace lazy_neighbors
}  // namespace eastar
}  // namespace search

#endif  // SRC_SEARCH_ROBOCUP_EASTAR_LAZY_NEIGHBOR_GENERATOR_H_
