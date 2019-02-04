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

#ifndef SRC_OBSTACLES_OBSTACLE_H_
#define SRC_OBSTACLES_OBSTACLE_H_

#include "eigen3/Eigen/Core"
#include "math/poses_2d.h"
#include "obstacles/integer_bounding_box.h"
#include "obstacles/obstacle_type.h"

// Forward declaration of logger
namespace logger {
class Logger;
}  // namespace logger

namespace obstacle {

// Padding for nearest free point calculations to account for floating point
// error.
const float kEpsilonPad = 0.1f;

// Virtual Class representing the generic obstacles for a path planning problem
class Obstacle {
 public:
  explicit Obstacle(ObstacleType type);
  virtual ~Obstacle() = default;

  // Virtual Function
  // Check whether a given point lies within the obstacle
  virtual bool PointCollision(const Eigen::Vector2f& point,
                              float margin) const = 0;

  // Virtual Function
  // Check whether a given line segment crosses the obstacle
  // Line segment goes from point_a to point_b
  virtual bool LineCollision(const Eigen::Vector2f& line_start,
                             const Eigen::Vector2f& line_end,
                             float margin) const = 0;

  // Virtual Function
  // Check whether a given line segment crosses the obstacle
  // Line segment goes from point_a to point_b
  virtual bool LineCollision(const Eigen::Vector2f& line_start,
                             const Eigen::Vector2f& line_end, float margin,
                             IntegerBoundingBox line_bounding_box) const;

  // Virtual Function
  // Get the distance from a point to the closest point on your obstacle
  virtual float PointDistance(const Eigen::Vector2f& point,
                              float margin) const = 0;

  // Virtual Function
  // Get the distance from a line segment to the closest point on your obstacle
  virtual float LineDistance(const Eigen::Vector2f& line_start,
                             const Eigen::Vector2f& line_end,
                             float margin) const = 0;

  // Virtual Function
  // Given a point that lies within your obstacle, get the nearest point
  // outside the obstacle
  virtual Eigen::Vector2f NearestFreePoint(const Eigen::Vector2f& point,
                                           float margin) const = 0;

  // Virtual Function
  // Given a directional line defined by the start and end points of the line,
  // find the furthest point on the line that isn't colliding with the obstacle.
  // If no collision is found, return false and set free point to be the end of
  // the line.
  virtual bool FurthestFreePointOnLine(const Eigen::Vector2f& line_start,
                                       const Eigen::Vector2f& line_end,
                                       Eigen::Vector2f* free_point,
                                       float* distance_from_start,
                                       float margin) const = 0;

  // Virtual Function
  // Gets the center point of the obstacle.
  virtual pose_2d::Pose2Df GetPose() const = 0;

  // Virtual Function
  // Updates the internal representation of the pose of the object
  // Only used for dynamic objects
  // Pose contains 3 components, x,y, and theta
  virtual void UpdatePose(const pose_2d::Pose2Df& pose) = 0;

  virtual bool GetTangents(const Eigen::Vector2f& desired_point,
                           const float margin,
                           Eigen::Vector2f* right_tangent,
                           Eigen::Vector2f* left_tangent) const = 0;

  // Returns the radius of the bounding circle around the obstacle
  virtual const float GetRadius() const = 0;

  // Returns an axis aligned integer bounding box that fits completely inside of
  // the obstacle.
  virtual IntegerBoundingBox GetInternalBoundingBox() const = 0;

  // Returns an axis aligned integer bounding box that fits completly outside of
  // the obstacle.
  virtual IntegerBoundingBox GetExternalBoundingBox() const = 0;

  // Returns the class of obstacle. Possible values are: ROBOT, BALL, RULE,
  // and STATIC
  ObstacleType GetType() const;

  void SetEnabled(const bool enabled);

  const bool IsEnabled() const;

  virtual void DrawObstacle(logger::Logger* local_log) const = 0;

 protected:
  IntegerBoundingBox bounding_box_;

 private:
  ObstacleType type_;
  bool is_enabled_;
};

}  // namespace obstacle

#endif  // SRC_OBSTACLES_OBSTACLE_H_
