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

#include "constants/constants.h"
#include "eigen3/Eigen/Core"
#include "math/poses_2d.h"
#include "obstacles/obstacle.h"

#ifndef SRC_OBSTACLES_BALL_OBSTACLE_H_
#define SRC_OBSTACLES_BALL_OBSTACLE_H_

namespace obstacle {

class BallObstacle : public Obstacle {
 public:
  enum BallType { SMALL, MEDIUM, LARGE };
  // Constructor function. pose defines the initial position of center_x_ and
  // center_y_.
  BallObstacle(const pose_2d::Pose2Df& pose, const BallType& type);

  // Constructor function. pose defines the initial position of center_x_ and
  // center_y_. Adding this constructor as the ball does not actually have an
  // orientation
  BallObstacle(const Eigen::Vector2f& pose, const BallType& type);

  BallObstacle& operator=(const BallObstacle& other) = default;

  // Check whether a given point lies within the obstacle
  bool PointCollision(const Eigen::Vector2f& point, float margin) const;

  // Check whether a given line segment crosses the obstacle
  // Line segment goes from point_a to point_b
  bool LineCollision(const Eigen::Vector2f& line_start,
                     const Eigen::Vector2f& line_end, float margin) const;

  // Get the distance from a point to the closest point on your obstacle
  float PointDistance(const Eigen::Vector2f& point, float margin) const;

  // Get the distance from a line segment to the closest point on your obstacle
  float LineDistance(const Eigen::Vector2f& line_start,
                     const Eigen::Vector2f& line_end, float margin) const;

  // Given a point that lies within your obstacle, get the nearest point
  // outside the obstacle
  Eigen::Vector2f NearestFreePoint(const Eigen::Vector2f& point,
                                   float margin) const;

  bool FurthestFreePointOnLine(const Eigen::Vector2f& line_start,
                               const Eigen::Vector2f& line_end,
                               Eigen::Vector2f* free_point,
                               float* distance_from_start, float margin) const;

  // Gets the center point of the obstacle.
  pose_2d::Pose2Df GetPose() const;

  // Updates the internal representation of the pose of the object
  // Only used for dynamic objects
  // Pose contains 3 components, x,y, and theta
  void UpdatePose(const pose_2d::Pose2Df& pose);

  // Updates the internal representation of the pose of the object
  // Implementing this because the ball does not actually have an orientation
  void UpdatePose(const Eigen::Vector2f& pose);

  // Returns the radius
  const float GetRadius() const;

  IntegerBoundingBox GetInternalBoundingBox() const;

  IntegerBoundingBox GetExternalBoundingBox() const;

  bool GetTangents(const Eigen::Vector2f& desired_point, const float margin,
                   Eigen::Vector2f* right_tangent,
                   Eigen::Vector2f* left_tangent) const;

  void DrawObstacle(logger::Logger* local_log) const;

 private:
  // A vector length 2 that represents the center of the circle.
  Eigen::Vector2f center_;

  // radius_ is the Radius of the obstacle.
  float radius_;
};
}  // namespace obstacle

#endif  // SRC_OBSTACLES_BALL_OBSTACLE_H_
