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

#include <vector>
#include <utility>

#include "logging/logger.h"
#include "math/poses_2d.h"
#include "obstacles/obstacle.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/safety_margin.h"
#include "zone/zone.h"

#ifndef SRC_NAVIGATION_NAVIGATION_UTIL_H_
#define SRC_NAVIGATION_NAVIGATION_UTIL_H_

namespace navigation {

std::vector<Eigen::Vector2f> SmoothPath(
    const std::vector<Eigen::Vector2f>& path,
    const obstacle::ObstacleFlag& obstacles,
    const obstacle::SafetyMargin& safety_margin);

// Given an input point, moves that point until it is no longer in collision
// with any obstacles
// Returns whether or not the obstacle was in collision in the first place
// Sets aborted to false when it can't find a point in free space
bool ClosestPointInFreeSpace(const obstacle::ObstacleFlag& obstacles,
                             const obstacle::SafetyMargin& safety_margin,
                             Eigen::Vector2f* position, bool* aborted,
                             logger::Logger* local_logger);

bool IsValidPath(const obstacle::ObstacleFlag& obstacles,
                 const obstacle::SafetyMargin& safety_margin,
                 const std::vector<Eigen::Vector2f>& path,
                 const unsigned int step = 0);

// Collision Test PRM Primitive Function
// Given two points, this function will return if the line segment between
// them is free of obstacles.
bool CollisionFreePath(const obstacle::ObstacleFlag& obstacles,
                       const obstacle::SafetyMargin& safety_margin,
                       const Eigen::Vector2f& position1,
                       const Eigen::Vector2f& position2);

std::pair<bool, const obstacle::Obstacle*> CollisionFreePathGetObstacle(
    const obstacle::ObstacleFlag& obstacles,
    const obstacle::SafetyMargin& safety_margin,
    const Eigen::Vector2f& position1, const Eigen::Vector2f& position2);

// Travels to the goal or to the furthest point on the line between the goal
// and the start such that there are no obstacles between the start and that
// point
float OneShotMode(const obstacle::ObstacleFlag& obstacles,
                  const obstacle::SafetyMargin& safety_margin,
                  const Eigen::Vector2f& start_position,
                  const Eigen::Vector2f& goal_position,
                  Eigen::Vector2f* waypoint);

// Modifies the input path so that the robot will travel to the furthest point
// possible safely
// Returns the new path cost
float SmoothPath(const obstacle::ObstacleFlag& obstacles,
                 const obstacle::SafetyMargin& safety_margin, bool use_tangents,
                 std::vector<Eigen::Vector2f>* path,
                 logger::Logger* local_logger);

bool PointCollision(const Eigen::Vector2f& point,
                    const obstacle::ObstacleFlag& obstacles,
                    const obstacle::SafetyMargin& safety_margin);

Eigen::Vector2f TangentSmooth(const obstacle::ObstacleFlag& obstacles,
                              const obstacle::SafetyMargin& safety_margin,
                              const Eigen::Vector2f& segment_start,
                              const Eigen::Vector2f& segment_end,
                              const Eigen::Vector2f& start_position,
                              logger::Logger* local_logger);

// Given a Start and End Point of a Path, if the End Point lies either outside
// the field or inside a defense region, returns the nearest point along
// the trajectory in a safe region
Eigen::Vector2f ProjectToSafety(const Vector2f& goal_point,
                                const Vector2f& start_point, float margin,
                                logger::Logger* logger);

Eigen::Vector2f GetRestPoint(const Vector2f& start_point,
                             const Vector2f& velocity,
                             const float& acceleration);

float GetPathLength(const std::vector<Eigen::Vector2f>& path);

}  // namespace navigation
#endif  // SRC_NAVIGATION_NAVIGATION_UTIL_H_
