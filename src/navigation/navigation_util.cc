// Copyright 2017 - 2018 kvedder@umass.edu slane@cs.umass.edu
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

#include "navigation/navigation_util.h"

#include <iomanip>
#include <limits>
#include <vector>

#include "logging/logger.h"
#include "math/geometry.h"
#include "math/poses_2d.h"
#include "obstacles/obstacle.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/obstacle_type.h"
#include "obstacles/safety_margin.h"

using geometry::RayIntersect;
using geometry::FurthestFreePointCircle;
using geometry::EuclideanDistance;
using geometry::SafeVectorNorm;
using geometry::SquaredDistance;
using obstacle::Obstacle;
using obstacle::ObstacleFlag;
using obstacle::ObstacleType;
using obstacle::SafetyMargin;
using Eigen::Vector2f;
using std::vector;

namespace navigation {

vector<Vector2f> SmoothPath(const vector<Vector2f>& path,
                            const ObstacleFlag& obstacles,
                            const SafetyMargin& safety_margin) {
  if (path.size() <= 2) {
    return path;
  }

  vector<Vector2f> smoothed_path = {path[0]};
  int clear_path_index = path.size() - 1;
  for (clear_path_index = path.size() - 1; clear_path_index > 0;
       --clear_path_index) {
    bool collision = false;
    for (const auto* obstacle : obstacles) {
      if (obstacle->LineCollision(
              path[0], path[clear_path_index],
              safety_margin.GetMargin(obstacle->GetType()))) {
        collision = true;
        break;
      }
    }

    if (!collision) {
      break;
    }
  }
  for (int i = clear_path_index; i < static_cast<int>(path.size() - 1); ++i) {
    smoothed_path.push_back(path[i]);
  }
  return smoothed_path;
}

// Given an input point, moves that point until it is no longer in collision
// with any obstacles
bool ClosestPointInFreeSpace(const ObstacleFlag& obstacles,
                             const SafetyMargin& safety_margin,
                             Vector2f* position, bool* aborted,
                             logger::Logger* local_logger) {
  static const bool kDebug = false;

  // Check to make sure that a NearestFreePoint does not collide with another
  // obstacle, including those previously checked.
  bool point_reset = false;
  bool has_collision = false;

  int num_recursions = 0;

  *aborted = false;

  do {
    point_reset = false;
    for (const Obstacle* obstacle : obstacles) {
      if (obstacle->PointCollision(
              *position, safety_margin.GetMargin(obstacle->GetType()))) {
        if (kDebug) {
          local_logger->LogPrint("Colliding with obstacle of type: %d",
                                 obstacle->GetType());

          local_logger->LogPrint("Obstacle at positon: %f, %f",
                                 obstacle->GetPose().translation.x(),
                                 obstacle->GetPose().translation.y());

          obstacle->DrawObstacle(local_logger);
        }

        has_collision = true;
        Vector2f safe_position = obstacle->NearestFreePoint(
            *position, safety_margin.GetMargin(obstacle->GetType()));

        *position = safe_position;

        point_reset = true;
        num_recursions++;
        break;
      }
    }

    if (num_recursions >= kMaxClosestPointRecursions) {
      *aborted = true;
      break;
    }
  } while (point_reset);

  if (kDebug) {
    local_logger->LogPrint("Num Recursions: %d", num_recursions);
  }

  if (!kProduction && !(*aborted)) {
    for (const auto* obstacle : obstacles) {
      NP_CHECK(!obstacle->PointCollision(
          *position, safety_margin.GetMargin(obstacle->GetType())));
    }
  }

  return has_collision;
}

bool IsValidPath(const ObstacleFlag& obstacles,
                 const SafetyMargin& safety_margin,
                 const vector<Eigen::Vector2f>& path, const unsigned int step) {
  for (unsigned int i = step; i < path.size() - 1; i++) {
    if (!CollisionFreePath(obstacles, safety_margin, path[i], path[i + 1])) {
      return false;
    }
  }
  return true;
}

// Collision Test PRM Primitive Function
// Given two points, this function will return if the line segment between
// them is free of obstacles.
bool CollisionFreePath(const ObstacleFlag& obstacles,
                       const SafetyMargin& safety_margin,
                       const Vector2f& position1, const Vector2f& position2) {
  for (const Obstacle* obstacle : obstacles) {
    if (obstacle->LineCollision(position1, position2,
                                safety_margin.GetMargin(obstacle->GetType()))) {
      return false;
    }
  }
  return true;
}

// Same as above but sets colliding_obstacle to the obstacle that it is
// colliding with
std::pair<bool, const obstacle::Obstacle*> CollisionFreePathGetObstacle(
    const ObstacleFlag& obstacles, const SafetyMargin& safety_margin,
    const Vector2f& position1, const Vector2f& position2) {
  for (const Obstacle* obstacle : obstacles) {
    if (obstacle->LineCollision(position1, position2,
                                safety_margin.GetMargin(obstacle->GetType()))) {
      return {false, obstacle};
    }
  }
  return {true, nullptr};
}

// Travels to the goal or to the furthest point on the line between the goal
// and the start such that there are no obstacles between the start and that
// point
float OneShotMode(const obstacle::ObstacleFlag& obstacles,
                  const SafetyMargin& safety_margin,
                  const Eigen::Vector2f& start_position,
                  const Eigen::Vector2f& goal_position,
                  Eigen::Vector2f* waypoint) {
  if (!kProduction) {
    for (const auto* obstacle : obstacles) {
      if (obstacle->PointCollision(
              start_position,
              safety_margin.GetMargin(obstacle->GetType()) - 1)) {
        LOG(ERROR) << "Starting in collision with an obstacle!";
      }
    }
  }

  Vector2f closest_point = goal_position;
  float closest_distance = std::numeric_limits<float>::max();

  for (const Obstacle* obstacle : obstacles) {
    Vector2f current_point(0, 0);
    float current_distance = std::numeric_limits<float>::max();
    const float margin_value = safety_margin.GetMargin(obstacle->GetType());
    if (obstacle->FurthestFreePointOnLine(start_position, goal_position,
                                          &current_point, &current_distance,
                                          margin_value) &&
        current_distance < closest_distance) {
      closest_distance = current_distance;
      closest_point = current_point;
    }
  }

  *waypoint = closest_point;

  // if (!kProduction) {
  //   for (const auto* obstacle : obstacles) {
  //     NP_CHECK_MSG(!obstacle->PointCollision(
  //                      *waypoint,
  //                      safety_margin.GetMargin(obstacle->GetType())),
  //                  "Waypoint: " << waypoint->x() << ", " << waypoint->y()
  //                               << " Obstacle type: " << obstacle->GetType()
  //                               << " Obstacle position: "
  //                               << obstacle->GetPose().translation.x() << ",
  //                               "
  //                               << obstacle->GetPose().translation.y());
  //   }
  // }

  return EuclideanDistance(start_position, closest_point);
}

const Obstacle* ClosestCollidingObstacle(
    const obstacle::ObstacleFlag& obstacles, const SafetyMargin& safety_margin,
    Vector2f start_point, Vector2f end_point) {
  float closest_distance = -1;
  bool found_collision = false;

  const Obstacle* return_obstacle = NULL;

  for (const Obstacle* obstacle : obstacles) {
    Vector2f current_point;
    float current_distance;

    if (obstacle->FurthestFreePointOnLine(
            start_point, end_point, &current_point, &current_distance,
            safety_margin.GetMargin(obstacle->GetType()))) {
      if (!found_collision || current_distance < closest_distance) {
        closest_distance = current_distance;
        return_obstacle = obstacle;
      }
      found_collision = true;
    }
  }

  return return_obstacle;
}

// Modifies the input path so that the robot will travel to the furthest point
// possible safely
// Returns the new path cost
float SmoothPath(const obstacle::ObstacleFlag& obstacles,
                 const SafetyMargin& safety_margin, bool use_tangents,
                 std::vector<Eigen::Vector2f>* path,
                 logger::Logger* local_logger) {
  static const bool kDebug = false;
  if (kDebug) {
    local_logger->LogPrint("Smooth Path");
    local_logger->Push();
    local_logger->LogPrint("Smoothing path of length: %d", path->size());
  }
  if (path->size() > 2) {
    Vector2f start_point = (*path)[0];

    if (CollisionFreePath(obstacles, safety_margin, start_point,
                          path->back())) {
      if (kDebug) local_logger->LogPrint("Going straight to goal");
      Vector2f goal_point = path->back();
      path->clear();
      path->push_back(start_point);
      path->push_back(goal_point);

      if (kDebug) local_logger->Pop();
      return EuclideanDistance(start_point, goal_point);
    }

    // Find first point that collides with an obstacle
    unsigned int furthest_free_index = 1;
    for (unsigned int i = 2; i < path->size() - 1; i++) {
      if (!CollisionFreePath(obstacles, safety_margin, start_point,
                             (*path)[i])) {
        if (kDebug)
          local_logger->LogPrint("Found a collision at segment %d", i);
        break;
      }
      furthest_free_index = i;
    }

    // Find furthest point along the relevent path segment that is free
    Vector2f furthest_free_point = (*path)[furthest_free_index];

    // If the furthest point is not the goal
    if (use_tangents && furthest_free_index != path->size() - 1) {
      furthest_free_point = TangentSmooth(
          obstacles, safety_margin, furthest_free_point,
          (*path)[furthest_free_index + 1], start_point, local_logger);
    }

    // Add start, furthest free point, and then any remaining path points to
    // the "smoothed" path
    std::vector<Eigen::Vector2f> new_path;
    new_path.push_back(start_point);
    new_path.push_back(furthest_free_point);

    float cost = EuclideanDistance(start_point, furthest_free_point);
    Vector2f prev_point = furthest_free_point;

    for (unsigned int i = furthest_free_index + 1; i < path->size(); i++) {
      new_path.push_back((*path)[i]);
      cost += EuclideanDistance((*path)[i], prev_point);
      prev_point = (*path)[i];
    }

    path->clear();
    *path = new_path;

    if (kDebug) local_logger->Pop();
    return cost;
  } else if (!path->empty()) {
    if (kDebug) local_logger->Pop();
    return EuclideanDistance((*path)[0], (*path)[1]);
  } else {
    return -1;
  }
}

bool PointCollision(const Vector2f& point, const ObstacleFlag& obstacles,
                    const SafetyMargin& safety_margin) {
  for (const Obstacle* obstacle : obstacles) {
    if (obstacle->PointCollision(point,
                                 safety_margin.GetMargin(obstacle->GetType())))
      return true;
  }
  return false;
}

Vector2f TangentSmooth(const ObstacleFlag& obstacles,
                       const SafetyMargin& safety_margin,
                       const Vector2f& segment_start,
                       const Vector2f& segment_end,
                       const Vector2f& start_position,
                       logger::Logger* local_logger) {
  static const bool kDebug = false;
  Vector2f return_point(segment_end);
  // Get the closest colliding obstacle
  const Obstacle* closest_obstacle = ClosestCollidingObstacle(
      obstacles, safety_margin, start_position, return_point);

  if (kDebug) {
    local_logger->LogPrint("Segment start: %0.1f, %0.1f", segment_start.x(),
                           segment_start.y());
    local_logger->LogPrint("Segment end: %0.1f, %0.1f", segment_end.x(),
                           segment_end.y());
  }

  int num_recursions = 0;

  while (closest_obstacle != NULL) {
    if (kDebug) {
      local_logger->LogPrint("Num Recursions: %d", num_recursions);
    }
    if (num_recursions > kMaxTangentRecursions) {
      return_point = segment_start;
      if (kDebug) {
        local_logger->LogPrint("Max Recursions hit");
      }
      break;
    }

    if (closest_obstacle->GetType() == ObstacleType::BALL) {
      break;
    }

    // Get the tangent points the the relevent obstacle
    Vector2f left_tangent;
    Vector2f right_tangent;

    bool has_tangents = closest_obstacle->GetTangents(
        start_position, safety_margin.GetMargin(closest_obstacle->GetType()) +
                            kTangentSmoothPadding,
        &left_tangent, &right_tangent);

    if (!has_tangents) {
      return_point = segment_start;
      if (kDebug) {
        local_logger->LogPrint("Point is inside tangent smoothing margin");
      }
      break;
    }

    // Set furthest_free_point to the intersection between the tangent line
    // and the line segment
    float left_squared_distance;
    Vector2f left_intersect_point;

    Vector2f left_direction = left_tangent - start_position;
    bool left_intersects =
        RayIntersect(start_position, left_direction, segment_start, segment_end,
                     &left_squared_distance, &left_intersect_point);

    float right_squared_distance;
    Vector2f right_intersect_point;
    Vector2f right_direction = right_tangent - start_position;
    bool right_intersects = RayIntersect(
        start_position, right_direction, segment_start, segment_end,
        &right_squared_distance, &right_intersect_point);

    if (left_intersects) {
      left_squared_distance =
          (left_intersect_point - segment_start).squaredNorm();

      if (kDebug) {
        local_logger->LogPrint("Left tangent intersects");
      }
    }

    if (right_intersects) {
      right_squared_distance =
          (right_intersect_point - segment_start).squaredNorm();
      if (kDebug) {
        local_logger->LogPrint("Right tangent intersects");
      }
    }

    if (left_intersects) {
      if (right_intersects && right_squared_distance < left_squared_distance) {
        return_point = right_intersect_point;
      } else {
        return_point = left_intersect_point;
      }
    } else if (right_intersects) {
      return_point = right_intersect_point;
    } else {
      if (kDebug) {
        local_logger->LogPrint("Neither tangent intersects");
      }
      return_point = segment_start;
      break;
    }

    const Obstacle* prev_closest_obstacle = closest_obstacle;
    closest_obstacle = ClosestCollidingObstacle(obstacles, safety_margin,
                                                start_position, return_point);

    if (prev_closest_obstacle == closest_obstacle) {
      break;
    }

    num_recursions++;
  }

  if (kDebug) {
    local_logger->LogPrint("Return point: %0.1f, %0.1f", return_point.x(),
                           return_point.y());
  }

  closest_obstacle = ClosestCollidingObstacle(obstacles, safety_margin,
                                              start_position, return_point);

  if (closest_obstacle != NULL) {
    return_point = segment_start;
  }

  return return_point;
}

Vector2f ProjectToSafety(const Vector2f& goal_point,
                         const Vector2f& start_point, float margin,
                         logger::Logger* local_logger) {
  static const bool kDebug = false;
  if (kDebug) {
    local_logger->LogPrintPush("Project To Safety");
  }
  zone::FieldZone field_zone(zone::FULL_FIELD);
  Vector2f return_point = goal_point;

  if (!field_zone.IsInZone(goal_point, margin)) {
    Vector2f v1(field_zone.min_x_ + margin, field_zone.min_y_ + margin);
    Vector2f v2(field_zone.max_x_ - margin, field_zone.min_y_ + margin);
    Vector2f v3(field_zone.max_x_ - margin, field_zone.max_y_ - margin);
    Vector2f v4(field_zone.min_x_ + margin, field_zone.max_y_ - margin);

    float square_dist;
    float min_square_dist = INFINITY;
    Vector2f intersect;
    Vector2f min_intersect;
    bool does_intersect;

    Vector2f displace = start_point - goal_point;

    does_intersect =
        RayIntersect(goal_point, displace, v1, v2, &square_dist, &intersect);
    if (does_intersect && min_square_dist > square_dist) {
      min_intersect = intersect;
      min_square_dist = square_dist;
    }

    if (kDebug) {
      if (does_intersect)
        local_logger->LogPrint("Intersects with segment 1");
      else
        local_logger->LogPrint("Does not intersect segment 1");
    }

    does_intersect =
        RayIntersect(goal_point, displace, v2, v3, &square_dist, &intersect);
    if (does_intersect && min_square_dist > square_dist) {
      min_intersect = intersect;
      min_square_dist = square_dist;
    }

    if (kDebug) {
      if (does_intersect)
        local_logger->LogPrint("Intersects with segment 2");
      else
        local_logger->LogPrint("Does not intersect segment 2");
    }

    does_intersect =
        RayIntersect(goal_point, displace, v3, v4, &square_dist, &intersect);
    if (does_intersect && min_square_dist > square_dist) {
      min_intersect = intersect;
      min_square_dist = square_dist;
    }

    if (kDebug) {
      if (does_intersect)
        local_logger->LogPrint("Intersects with segment 3");
      else
        local_logger->LogPrint("Does not intersect segment 3");
    }

    does_intersect =
        RayIntersect(goal_point, displace, v4, v1, &square_dist, &intersect);
    if (does_intersect && min_square_dist > square_dist) {
      min_intersect = intersect;
      min_square_dist = square_dist;
    }

    if (kDebug) {
      if (does_intersect)
        local_logger->LogPrint("Intersects with segment 4");
      else
        local_logger->LogPrint("Does not intersect segment 4");
    }

    return_point = min_intersect;
  } else {
    if (kDebug) {
      local_logger->LogPrint("In zone");
    }
  }

  for (const Obstacle* obstacle : ObstacleFlag::GetDefenseAreas()) {
    const auto new_return_point =
        obstacle->NearestFreePoint(return_point, margin);
    if (kDebug) {
      if (new_return_point != return_point) {
        local_logger->LogPrint("Old point: (%f, %f); New point: (%f, %f)",
                               return_point.x(), return_point.y(),
                               new_return_point.x(), new_return_point.y());
      }
    }
    return_point = new_return_point;
  }

  /*
    bool is_in_our_defense = (kOurGoalCenter - end_point).norm() <
  kDefenseBoundary;

    if (is_in_our_defense) {
      float square_dist;
      Vector2f intersect;

      FurthestFreePointCircle(start_point, end_point,
                              kOurGoalCenter, kDefenseBoundary,
                              &square_dist, &intersect);
      end_point = intersect;
    }

    bool is_in_their_defense = (kTheirGoalCenter - end_point).norm()
        < kDefenseBoundary;

    if (is_in_their_defense) {
      float square_dist;
      Vector2f intersect;

      FurthestFreePointCircle(start_point, end_point,
                              kTheirGoalCenter, kDefenseBoundary,
                              &square_dist, &intersect);
      end_point = intersect;
    }*/
  if (kDebug) {
    local_logger->Pop();
  }
  return return_point;
}

Eigen::Vector2f GetRestPoint(const Vector2f& start_point,
                             const Vector2f& velocity,
                             const float& acceleration) {
  if (velocity.norm() < kEpsilon) {
    return start_point;
  }
  const float time = velocity.norm() / acceleration;
  return start_point + velocity * time -
         0.5 * velocity.normalized() * acceleration * Sq(time);
}

float GetPathLength(const vector<Vector2f>& path) {
  float length = 0.0f;
  for (unsigned int i = 0; i < path.size() - 1; i++) {
    length += EuclideanDistance(path[i], path[i + 1]);
  }

  return length;
}

}  // namespace navigation
