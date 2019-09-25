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

#include "navigation/RRT.h"

#include <algorithm>
#include <functional>
#include <limits>
#include <map>
#include <queue>
#include <random>
#include <vector>

#include "constants/constants.h"
#include "datastructures/better_map.h"
#include "eigen3/Eigen/Sparse"
#include "graph/graph.h"
#include "graph/vertex.h"
#include "math/poses_2d.h"
#include "navigation/navigation_util.h"
#include "navigation/scaffolding/scaffold.h"
#include "obstacles/circle_obstacle.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/obstacle_type.h"
#include "obstacles/safety_margin.h"

STANDARD_USINGS;

// Scaffolding constants.
static const float kCircleRadius = kRobotRadius + 20;
static constexpr float kCircleNumLayers = 2;
static constexpr float kCirclePointsPerLayer = 16;
static constexpr float kCircleDistanceBetweenLayers = 40;

namespace navigation {
namespace rrt {
// This value will be set for default positions. This is to indicate at
// runtime if Update() was not called prior to Plan().
//
// It is set to (infinity, infinity), and thus is not in the valid space of
// locations to plan.
static const Eigen::Vector2f kDefaultVector(
    std::numeric_limits<int>::infinity() - 1,
    std::numeric_limits<int>::infinity() - 1);

RRT::RRT(const obstacle::ObstacleFlag& obstacles,
         const obstacle::SafetyMargin& safety_margin,
         const bool use_scaffolding, const float scaffold_bias)
    : obstacles_(obstacles),
      safety_margin_(safety_margin),
      start_position_(kDefaultVector),
      goal_position_(kDefaultVector),
      random_(),
      use_scaffolding_(use_scaffolding),
      scaffold_bias_(scaffold_bias) {
  if (use_scaffolding_) {
    for (const auto* obstacle : obstacles_) {
      switch (obstacle->GetType()) {
        case obstacle::ObstacleType::ROBOT: {
          //           LOG(WARNING) << "Inserting circle!";
          scaffold_graphs_.AddAdditionalGraph(
              scaffolding::GenerateCircleScaffold(
                  obstacle->GetPose(), kCircleRadius, kCircleNumLayers,
                  kCirclePointsPerLayer, kCircleDistanceBetweenLayers));
        } break;
        default:
          //           LOG(WARNING) << "Skipping!";
          continue;
      }
    }
  }
}

RRT::RRT(const obstacle::ObstacleFlag& obstacles,
         const obstacle::SafetyMargin& safety_margin,
         const std::size_t rng_seed, const bool use_scaffolding,
         const float scaffold_bias)
    : obstacles_(obstacles),
      safety_margin_(safety_margin),
      start_position_(kDefaultVector),
      goal_position_(kDefaultVector),
      random_(rng_seed),
      use_scaffolding_(use_scaffolding),
      scaffold_bias_(scaffold_bias) {
  if (use_scaffolding_) {
    for (const auto* obstacle : obstacles_) {
      switch (obstacle->GetType()) {
        case obstacle::ObstacleType::ROBOT: {
          //           LOG(WARNING) << "Inserting circle!";
          scaffold_graphs_.AddAdditionalGraph(
              scaffolding::GenerateCircleScaffold(
                  obstacle->GetPose(),
                  kCircleRadius + safety_margin_.GetMargin(obstacle->GetType()),
                  kCircleNumLayers, kCirclePointsPerLayer,
                  kCircleDistanceBetweenLayers));
        } break;
        default:
          //           LOG(WARNING) << "Skipping!";
          continue;
      }
    }
  }
}

void RRT::Update(const obstacle::ObstacleFlag& obstacles,
                 const Eigen::Vector2f& current_pose, const Vector2f& goal_pose,
                 const obstacle::SafetyMargin& safety_margin) {
  safety_margin_ = safety_margin;
  start_position_ = current_pose;
  goal_position_ = goal_pose;

  // TODO(slane): Add scaffolds to the multigraph based on obstacle positions.
  if (use_scaffolding_) {
    std::vector<pose_2d::Pose2Df> old_poses;
    std::vector<pose_2d::Pose2Df> new_poses;

    for (const auto* obstacle : obstacles_) {
      old_poses.push_back(obstacle->GetPose());
    }
    for (const auto* obstacle : obstacles) {
      new_poses.push_back(obstacle->GetPose());
    }

    if (!kProduction) {
      if (old_poses.size() != new_poses.size()) {
        LOG(FATAL) << "Obstacle flags are different sizes. RIP my test code.";
      } else {
        //         LOG(WARNING) << "Sizes are same (" << new_poses.size() <<
        //         ")";
      }
    }

    for (size_t i = 0; i < old_poses.size(); ++i) {
      const auto& old_pose = old_poses[i];
      const auto& new_pose = new_poses[i];
      scaffolding::ModifyScaffoldPose(old_pose, new_pose,
                                      scaffold_graphs_.GetMutableGraph(i));
    }
  }

  obstacles_ = obstacles;
}

// Handles the special case of normalized(), where given a zero vector, it
// returns a zero vector rather that <NaN, NaN>.
const Vector2f GetNormalizedOrZero(const Vector2f& vec) {
  if (vec.x() == 0.0 && vec.y() == 0.0) {
    return vec;
  } else {
    return vec.normalized();
  }
}

inline Vector2f RandomSampleFromField(util_random::Random* random) {
  const auto rand_x = random->UniformRandom(-field_dimensions::kHalfFieldLength,
                                            field_dimensions::kHalfFieldLength);
  const auto rand_y = random->UniformRandom(-field_dimensions::kHalfFieldWidth,
                                            field_dimensions::kHalfFieldWidth);
  return Vector2f(rand_x, rand_y);
}

Vector2f SelectExtensionSteeringPoint(
    const bool use_scaffolding, const Vector2f& goal_position,
    const graph::multi_graph::FastMultiGraph& scaffold_graphs,
    util_random::Random* random, const float scaffold_bias) {
  static constexpr bool kDebug = true;
  const float rand_value = random->UniformRandom();
  if (rand_value <= kGoalBias) {
    // Goal bias.
    return goal_position;
  } else if (use_scaffolding && rand_value <= kGoalBias + scaffold_bias) {
    // Scaffold bias.
    size_t num_vertices = 0;
    for (const auto& graph : scaffold_graphs.GetGraphs()) {
      num_vertices += graph.GetNumVertices();
    }

    // There are no scaffold graphs, so there is no point to select.
    if (num_vertices <= 0) {
      if (!kProduction && kDebug) {
        LOG(ERROR) << "No scaffold graph point to select! Defaulting to random "
                      "sample.";
      }
      return RandomSampleFromField(random);
    }

    const int rand_vertex_index = random->RandomInt<int>(0, num_vertices - 1);

    int current_index = 0;
    for (const auto& graph : scaffold_graphs.GetGraphs()) {
      for (const auto& vertex : graph.GetVertices()) {
        if (current_index == rand_vertex_index) {
          return vertex.position;
        }
        ++current_index;
      }
    }
    LOG(ERROR) << "Did not find vertex that matched the given index "
               << rand_vertex_index;
    return scaffold_graphs.GetGraphs()[0].GetVertices()[0].position;
  } else {
    // Uniform random sample.
    return RandomSampleFromField(random);
  }
}

struct TreeData {
  int parent;
  Vector2f point;

  TreeData() = delete;
  TreeData(int parent, Vector2f point) : parent(parent), point(point) {}
  ~TreeData() = default;
};

// TODO(kvedder): Replace with k-d tree.
size_t ClosestPointIndex(const Vector2f& query_point,
                         const vector<TreeData>& points) {
  if (!kProduction) {
    if (points.empty()) {
      LOG(FATAL) << "Passed empty point list, cannot find nearest point!";
    }
  }

  size_t closest_index = 0;
  float closest_distance = std::numeric_limits<float>::max();

  for (size_t current_index = 0; current_index < points.size();
       ++current_index) {
    if (closest_distance >
        (points[current_index].point - query_point).squaredNorm()) {
      closest_index = current_index;
      closest_distance =
          (points[current_index].point - query_point).squaredNorm();
    }
  }

  return closest_index;
}

bool CheckExtension(const Vector2f& start_position,
                    const Vector2f& end_position,
                    const obstacle::ObstacleFlag& obstacles,
                    const obstacle::SafetyMargin& safety_margin) {
  for (const auto* obstacle : obstacles) {
    if (obstacle->LineCollision(start_position, end_position,
                                safety_margin.GetMargin(obstacle->GetType()))) {
      return false;
    }
  }
  return true;
}

std::pair<std::pair<bool, size_t>, std::vector<Eigen::Vector2f>> RRT::Plan() {
  if (!kProduction) {
    if (start_position_ == kDefaultVector) {
      LOG(FATAL)
          << "Start position was not initialized! Make sure you call Update()!";
    }
    if (goal_position_ == kDefaultVector) {
      LOG(FATAL)
          << "End position was not initialized! Make sure you call Update()!";
    }
  }

  vector<TreeData> tree = {TreeData(kRootNodeParent, start_position_)};

  //   vector<Vector2f> sample_points;

  bool found_goal = false;
  size_t sample_count = 0;
  for (size_t i = 0; true; ++i) {
    const Vector2f extension_steering_point = SelectExtensionSteeringPoint(
        use_scaffolding_, goal_position_, scaffold_graphs_, &random_,
        scaffold_bias_);
    //     sample_points.push_back(extension_steering_point);
    const size_t closest_index =
        ClosestPointIndex(extension_steering_point, tree);
    const Vector2f& closest_point = tree[closest_index].point;
    const Vector2f extension_point =
        GetNormalizedOrZero(extension_steering_point - closest_point) *
            kExtensionDistance +
        closest_point;
    if (CheckExtension(closest_point, extension_point, obstacles_,
                       safety_margin_)) {
      TreeData tree_data(static_cast<int>(closest_index), extension_point);
      tree.push_back(tree_data);
      // If within set distance of goal, then exit.
      if ((extension_point - goal_position_).squaredNorm() <
          Sq(kDistanceFromGoal)) {
        found_goal = true;
        sample_count = i;
        break;
      }
    }
  }

  vector<Vector2f> final_path;
  // Add the goal position to the path.
  final_path.push_back(goal_position_);

  // Last element will always be in the path.
  int unwind_index =
      ((found_goal)
           ? static_cast<int>(tree.size()) - 1
           : static_cast<int>(ClosestPointIndex(goal_position_, tree)));
  while (tree[unwind_index].parent != kRootNodeParent) {
    if (!kProduction && unwind_index < 0) {
      LOG(FATAL) << "Unwind index negative! Value: " << unwind_index;
    }
    final_path.push_back(tree[unwind_index].point);
    unwind_index = tree[unwind_index].parent;
  }

  // Add the start position to the path.
  final_path.push_back(tree[unwind_index].point);

  // Swap direction of the path to ensure 0th element is the start.
  std::reverse(final_path.begin(), final_path.end());

  return {{found_goal, sample_count}, final_path};
  //   return {found_goal, sample_points};
}

const graph::multi_graph::FastMultiGraph& RRT::GetScaffoldGraphs() const {
  return scaffold_graphs_;
}

}  // namespace rrt
}  // namespace navigation
