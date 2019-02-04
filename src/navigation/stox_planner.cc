// Copyright 2016 - 2018 slane@cs.umass.edu
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

#include "navigation/stox_planner.h"

#include <vector>
#include <utility>

#include "eigen3/Eigen/Core"
#include "logging/logger.h"
#include "math/geometry.h"
#include "math/poses_2d.h"
#include "navigation/navigation_util.h"
#include "obstacles/obstacle.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/safety_margin.h"

using std::vector;
using obstacle::ObstacleFlag;
using obstacle::Obstacle;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using obstacle::SafetyMargin;
using obstacle::ObstacleType;
using geometry::ProjectPointOntoLine;
using geometry::Perp;

namespace navigation {
void StoxPlanner::Init(const ObstacleFlag& obstacles) {
  // obstacles_ = obstacles;
}

void StoxPlanner::Update(const ObstacleFlag& obstacles,
                         const Vector2f& current_position,
                         const Vector2f& goal_position,
                         logger::Logger* local_logger) {
  SafetyMargin margin;
  Update(obstacles, margin, current_position, goal_position, local_logger);
}

void StoxPlanner::Update(const ObstacleFlag& obstacles,
                         const SafetyMargin& safety_margin,
                         const Vector2f& current_position,
                         const Vector2f& goal_position,
                         logger::Logger* local_logger) {
  static const bool kDebug = true;
  current_position_ = current_position;
  start_position_ = current_position;
  goal_position_ = goal_position;
  // obstacles_ = obstacles;
  safety_margin_ = safety_margin;

  // Add the current position to the path
  current_path_.clear();
  current_path_.push_back(current_position_);

  // Move the start and goal points so they are not in collision with any
  // obstacles
  if (kDebug) {
    local_logger->LogPrint("Current robot safety margin: %f",
                           safety_margin_.GetMargin(ObstacleType::ROBOT));
  }

  bool start_aborted;
  started_in_collision_ =
      ClosestPointInFreeSpace(obstacles, safety_margin_, &start_position_,
                              &start_aborted, local_logger);

  if (kDebug) {
    local_logger->LogPrint("Original Goal: %f, %f", goal_position_.x(),
                           goal_position_.y());
  }

  bool goal_aborted;
  bool goal_in_collision = ClosestPointInFreeSpace(
      obstacles, safety_margin_, &goal_position_, &goal_aborted, local_logger);

  if (kDebug) {
    local_logger->LogPrint("Current Goal: %f, %f", goal_position_.x(),
                           goal_position_.y());
  }

  // TODO(slane): Figure out a more sensible thing to do when start has aborted
  // It won't lock up unless 4 robots are all equidistant and close to each
  // other
  if (start_aborted) {
    if (kDebug) {
      local_logger->LogPrint(
          "Start was in collision but could not find free point");
    }
  }

  if (goal_aborted) {
    if (kDebug) {
      local_logger->LogPrint(
          "Goal was in collision but could not find free point");
    }

    Vector2f safe_goal_;
    OneShotMode(obstacles, safety_margin, start_position_, goal_position_,
                &safe_goal_);
    goal_position_ = safe_goal_;
  }

  if (started_in_collision_ && kDebug)
    local_logger->LogPrint("Started in collision");

  if (goal_in_collision && kDebug) local_logger->LogPrint("Goal in collision");

  if (started_in_collision_) current_path_.push_back(start_position_);
}

std::pair<bool, std::vector<Eigen::Vector2f>> StoxPlanner::Plan(
    logger::Logger* local_logger) {
  vector<Vector2f> plan;
  if (fabs(goal_position_.x() - start_position_.x()) < kEpsilon &&
      fabs(goal_position_.y() - start_position_.y()) < kEpsilon) {
    current_path_.push_back(goal_position_);
    plan = current_path_;
    return {true, plan};
  } else if (std::isnan(goal_position_.x()) || std::isnan(goal_position_.y()) ||
             std::isnan(start_position_.x()) ||
             std::isnan(start_position_.y())) {
    current_path_.push_back(start_position_);
    plan = current_path_;
    return {false, plan};
  }

  float cost = FastPathPlanning(obstacles_, start_position_, goal_position_, 0,
                                &current_path_, local_logger);

  plan = current_path_;
  // static const bool kDebug = false;
  // If there was no path found, enter one-shot mode
  if (fabs(cost + 1.0) < kEpsilon) {
    local_logger->LogPrint("STOX Planner entering one shot mode");

    current_path_.clear();
    Vector2f one_shot_point;
    OneShotMode(obstacles_, safety_margin_, start_position_, goal_position_,
                &one_shot_point);

    current_path_.push_back(current_position_);

    if (started_in_collision_) {
      current_path_.push_back(start_position_);
    }

    current_path_.push_back(one_shot_point);
    plan = current_path_;
    return {false, plan};
  }

  return {true, plan};
}

bool StoxPlanner::CollisionFreePath(const obstacle::ObstacleFlag& obstacles,
                                    const Vector2f& position1,
                                    const Vector2f& position2,
                                    pose_2d::Pose2Df* closest_obstacle_center,
                                    float* closest_obstacle_radius,
                                    Vector2f* closest_point) const {
  bool found_collision = false;
  float closest_distance = -1;

  for (const Obstacle* obstacle : obstacles) {
    Vector2f current_point;
    float current_distance;

    if (obstacle->FurthestFreePointOnLine(
            position1, position2, &current_point, &current_distance,
            safety_margin_.GetMargin(obstacle->GetType()))) {
      if (!found_collision || current_distance < closest_distance) {
        closest_distance = current_distance;
        closest_point = &current_point;
        *closest_obstacle_center = obstacle->GetPose();
        *closest_obstacle_radius =
            safety_margin_.GetMargin(obstacle->GetType()) +
            obstacle->GetRadius();
      }
      found_collision = true;
    }
  }

  return found_collision;
}

float StoxPlanner::FastPathPlanning(const obstacle::ObstacleFlag& obstacles,
                                    const Vector2f& start, const Vector2f& goal,
                                    int search_depth,
                                    vector<Vector2f>* return_trajectory,
                                    logger::Logger* local_logger) {
  if (fabs(goal.x() - start.x()) < kEpsilon &&
      fabs(goal.y() - start.y()) < kEpsilon) {
    return 0.0f;
  }

  // static const bool kDebug = false;
  // First check if you can go from the current start point directly to the end
  // point
  Pose2Df closest_obstacle_center;
  float closest_obstacle_radius;
  Vector2f closest_point;
  bool is_blocked =
      CollisionFreePath(obstacles, start, goal, &closest_obstacle_center,
                        &closest_obstacle_radius, &closest_point);
  if (!is_blocked) {
    return_trajectory->push_back(goal);
  } else if (search_depth < kMaxSearchDepth) {
    Vector2f positive_search_point;
    bool found_positive = SearchPoint(
        obstacles, start, goal, closest_obstacle_center.translation,
        closest_obstacle_radius, 1, &positive_search_point, local_logger);
    Vector2f negative_search_point;
    bool found_negative = SearchPoint(
        obstacles, start, goal, closest_obstacle_center.translation,
        closest_obstacle_radius, -1, &negative_search_point, local_logger);

    vector<Vector2f> start_to_positive_point;
    vector<Vector2f> start_to_negative_point;
    vector<Vector2f> positive_point_to_goal;
    vector<Vector2f> negative_point_to_goal;

    float start_to_positive_length = -1;
    float positive_point_to_goal_length = -1;

    if (found_positive) {
      //       if (search_depth == 0)
      //             local_logger->AddPoint(positive_search_point.x(),
      //                                    positive_search_point.y(), 1, 1, 1,
      //                                    1);
      start_to_positive_length = FastPathPlanning(
          obstacles, start, positive_search_point, search_depth + 1,
          &start_to_positive_point, local_logger);

      if (start_to_positive_length != -1) {
        positive_point_to_goal_length = FastPathPlanning(
            obstacles, positive_search_point, goal, search_depth + 1,
            &positive_point_to_goal, local_logger);
      }
    }

    float start_to_negative_length = -1;
    float negative_point_to_goal_length = -1;

    if (found_negative) {
      //        if (search_depth == 0)
      //             local_logger->AddPoint(negative_search_point.x(),
      //                                    negative_search_point.y(), 1, 1, 1,
      //                                    1);
      start_to_negative_length = FastPathPlanning(
          obstacles, start, negative_search_point, search_depth + 1,
          &start_to_negative_point, local_logger);

      if (start_to_negative_length != -1) {
        negative_point_to_goal_length = FastPathPlanning(
            obstacles, negative_search_point, goal, search_depth + 1,
            &negative_point_to_goal, local_logger);
      }
    }

    if (fabs(start_to_positive_length + 1) > kEpsilon &&
        fabs(positive_point_to_goal_length + 1) > kEpsilon && found_positive) {
      if (fabs(start_to_negative_length + 1) < kEpsilon ||
          fabs(negative_point_to_goal_length + 1) < kEpsilon ||
          !found_negative) {
        JoinSegments(start_to_positive_point, positive_point_to_goal,
                     return_trajectory);
      } else if (start_to_positive_length + positive_point_to_goal_length >
                 start_to_negative_length + negative_point_to_goal_length) {
        JoinSegments(start_to_negative_point, negative_point_to_goal,
                     return_trajectory);
      } else {
        JoinSegments(start_to_positive_point, positive_point_to_goal,
                     return_trajectory);
      }
    } else if (fabs(start_to_negative_length + 1) > kEpsilon &&
               fabs(negative_point_to_goal_length + 1) > kEpsilon &&
               found_negative) {
      JoinSegments(start_to_negative_point, negative_point_to_goal,
                   return_trajectory);
    } else {
      return -1;
    }
  } else {
    return -1;
  }

  return CalculatePathLength(*return_trajectory);
}

bool StoxPlanner::SearchPoint(const obstacle::ObstacleFlag& obstacles,
                              const Vector2f& start, const Vector2f& goal,
                              const Vector2f& obstacle_center,
                              const float obstacle_radius, int angle_sign,
                              Vector2f* return_point,
                              logger::Logger* local_logger) {
  if (fabs(goal.x() - start.x()) < kEpsilon &&
      fabs(goal.y() - start.y()) < kEpsilon) {
    return false;
  } else if (std::isnan(start.x()) || std::isnan(start.y()) ||
             std::isnan(goal.x()) || std::isnan(goal.y())) {
    return false;
  }

  Vector2f direction = goal - start;
  Vector2f search_start_point;
  ProjectPointOntoLine(obstacle_center, start, goal, &search_start_point);

  Vector2f perpendicular = Perp(direction);
  perpendicular.normalize();

  float search_radius = obstacle_radius;

  if (search_radius > 2 * kRobotRadius) {
    search_radius = 2 * kRobotRadius;
  }

  *return_point =
      angle_sign * search_radius * perpendicular + search_start_point;

  if (return_point->x() > kInflatedFieldXMax ||
      return_point->x() < -kInflatedFieldXMax ||
      return_point->y() > kInflatedFieldYMax ||
      return_point->y() < -kInflatedFieldYMax) {
    return false;
  }

  // todo(slane): Modify this such that it will terminate eventually if it goes
  // out of certain bounds.
  while (PointCollidesWithObstacle(obstacles, *return_point)) {
    search_radius += kRobotRadius;
    *return_point =
        angle_sign * search_radius * perpendicular + search_start_point;

    if (return_point->x() > kInflatedFieldXMax ||
        return_point->x() < -kInflatedFieldXMax ||
        return_point->y() > kInflatedFieldYMax ||
        return_point->y() < -kInflatedFieldYMax) {
      return false;
    }
  }
  return true;
}

void StoxPlanner::JoinSegments(const vector<Vector2f>& start_trajectory,
                               const vector<Vector2f>& end_trajectory,
                               vector<Vector2f>* full_trajectory) {
  for (const Vector2f& point : start_trajectory) {
    full_trajectory->push_back(point);
  }

  for (const Vector2f& point : end_trajectory) {
    full_trajectory->push_back(point);
  }
}

float StoxPlanner::CalculatePathLength(const vector<Vector2f>& path) const {
  float path_length = 0.0f;
  for (unsigned int i = 1; i < path.size(); i++) {
    path_length += (path[i] - path[i - 1]).norm();
  }
  return path_length;
}

bool StoxPlanner::PointCollidesWithObstacle(const ObstacleFlag& obstacles,
                                            const Vector2f& point) const {
  for (const Obstacle* obstacle : obstacles) {
    if (obstacle->PointCollision(
            point, safety_margin_.GetMargin(obstacle->GetType()))) {
      return true;
    }
  }
  return false;
}

}  // namespace navigation
