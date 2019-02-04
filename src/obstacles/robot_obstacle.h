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
#include "obstacles/integer_bounding_box.h"
#include "obstacles/obstacle.h"

#ifndef SRC_OBSTACLES_ROBOT_OBSTACLE_H_
#define SRC_OBSTACLES_ROBOT_OBSTACLE_H_

namespace obstacle {

class RobotObstacle : public Obstacle {
 public:
  // Constructor function. pose defines the initial position of center_x_ and
  // center_y_. radius defines the radius of the circle.
  explicit RobotObstacle(const pose_2d::Pose2Df& pose);

  RobotObstacle& operator=(const RobotObstacle& other);

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

  // UpdatePose() nothing for Robot Obstacles! Updates should be applied to the
  // Pose2df backed by the const ref passed into this constructor.
  void UpdatePose(const pose_2d::Pose2Df& pose);

  bool GetTangents(const Eigen::Vector2f& desired_point,
                   const float margin,
                   Eigen::Vector2f* right_tangent,
                   Eigen::Vector2f* left_tangent) const;

  const float GetRadius() const;

  void DrawObstacle(logger::Logger* local_logger) const;

  IntegerBoundingBox GetInternalBoundingBox() const;

  IntegerBoundingBox GetExternalBoundingBox() const;

 private:
  // The pose of the robot
  pose_2d::Pose2Df pose_;

  // radius_ is the Radius of the obstacle.
  static const float radius_;

  IntegerBoundingBox bounding_box_;
};
}  // namespace obstacle

#endif  // SRC_OBSTACLES_ROBOT_OBSTACLE_H_
