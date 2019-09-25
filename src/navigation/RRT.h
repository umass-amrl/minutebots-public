// Copyright 2017 - 2019 kvedder@umass.edu
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

#ifndef SRC_NAVIGATION_RRT_H_
#define SRC_NAVIGATION_RRT_H_

#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "constants/includes.h"
#include "eigen3/Eigen/Core"
#include "graph/fastmultigraph.h"
#include "graph/graph.h"
#include "graph/vertex.h"
#include "math/poses_2d.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/safety_margin.h"
#include "util/random.h"

namespace navigation {
namespace rrt {
class RRT {
 public:
  RRT() = delete;
  RRT(const obstacle::ObstacleFlag& obstacles,
      const obstacle::SafetyMargin& safety_margin, const bool use_scaffolding,
      const float scaffold_bias);
  RRT(const obstacle::ObstacleFlag& obstacles,
      const obstacle::SafetyMargin& safety_margin, const size_t rng_seed,
      const bool use_scaffolding, const float scaffold_bias);
  ~RRT() = default;

  void Update(const obstacle::ObstacleFlag& obstacles,
              const Eigen::Vector2f& current_pose,
              const Eigen::Vector2f& goal_pose,
              const obstacle::SafetyMargin& safety_margin);

  // Returns a bool indicating if a successful path was found as well as either
  // a complete path, or an attempted path. Path may not be obstacle free if a
  // complete path is not found.
  std::pair<std::pair<bool, size_t>, std::vector<Eigen::Vector2f>> Plan();

  const graph::multi_graph::FastMultiGraph& GetScaffoldGraphs() const;

 private:
  obstacle::ObstacleFlag obstacles_;
  obstacle::SafetyMargin safety_margin_;

  Eigen::Vector2f start_position_;
  Eigen::Vector2f goal_position_;

  graph::multi_graph::FastMultiGraph scaffold_graphs_;
  util_random::Random random_;

  const bool use_scaffolding_;
  const float scaffold_bias_;
};

// Goal bias on [0 to 1] scale.
static constexpr float kGoalBias = 0.15;

// Amount out any given extension will go.
static constexpr float kExtensionDistance = 100;

// Index of the root node parent, which does not exist.
static constexpr int kRootNodeParent = -1;

// Amount extension must be from the goal to terminate.
static constexpr float kDistanceFromGoal = 100;

}  // namespace rrt
}  // namespace navigation

#endif  // SRC_NAVIGATION_RRT_H_
