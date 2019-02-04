// Copyright 2017 - 2018 slane@cs.umass.edu
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

#ifndef SRC_OBSTACLES_RECTANGLE_OBSTACLE_H_
#define SRC_OBSTACLES_RECTANGLE_OBSTACLE_H_

#include "eigen3/Eigen/Core"
#include "obstacles/obstacle.h"
#include "obstacles/obstacle_type.h"

namespace obstacle {

class RectangleObstacle : public Obstacle {
 public:
  // Rectangle is the constructor function. Pose defines the initial pose of
  // the obstacle. height defines the height of the rectangle. width defines
  // the width of the rectangle.
  RectangleObstacle(const pose_2d::Pose2Df& pose, ObstacleType type,
                    float width, float height);

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

  // Given a directional line defined by the start and end points of the line,
  // find the furthest point on the line that isn't colliding with the obstacle.
  // If no collision is found, return false and set free point to be the end of
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

  bool GetTangents(const Eigen::Vector2f& desired_point,
                   const float margin,
                   Eigen::Vector2f* right_tangent,
                   Eigen::Vector2f* left_tangent) const;

  void SetHeightWidth(const float height, const float width);

  const float GetRadius() const;

  const float GetWidth() const;

  const float GetHeight() const;

  IntegerBoundingBox GetInternalBoundingBox() const;

  IntegerBoundingBox GetExternalBoundingBox() const;

  void DrawObstacle(logger::Logger* local_logger) const;

 private:
  // A vector that represents the (x, y, theta) of the rectangle.
  pose_2d::Pose2Df pose_;

  // width_ represents the length of the rectangle.
  float width_;

  // height_ represents the length of the rectangle.
  float height_;

  // 2D Positions of each of the corners of the rectangle
  // At 0 theta:
  // 0 index is at the bottom left
  // 1 index is at bottom right
  // 2 index is at top right
  // 3 index is at top left
  // Line segments that form rectangle are 0->1, 1->2, 2->3, 3->0
  std::array<Eigen::Vector2f, 4> corner_coordinates_;
};
}  // namespace obstacle.

#endif  // SRC_OBSTACLES_RECTANGLE_OBSTACLE_H_
