// Copyright 2016 - 2018 slane@cs.umass.edu, kvedder@umass.edu
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

#include "obstacles/robot_obstacle.h"

#include <glog/logging.h>
#include <cmath>
#include <iomanip>
#include <limits>

#include "eigen3/Eigen/Core"
#include "math/geometry.h"
#include "math/math_util.h"
#include "math/poses_2d.h"
#include "obstacles/obstacle.h"
#include "obstacles/obstacle_type.h"

using Eigen::Vector2f;
using pose_2d::Pose2Df;
using math_util::Sq;
using geometry::SafeVectorNorm;
using geometry::ProjectPointOntoLineSegment;

namespace obstacle {

const float RobotObstacle::radius_ = kRobotRadius;

void UpdateBoundingBox(obstacle::IntegerBoundingBox* bb, const Pose2Df& pose,
                       const float radius) {
  bb->x_ = std::ceil(pose.translation.x());
  bb->y_ = std::floor(pose.translation.y());
  bb->width_ = std::ceil(2.0f * radius) + 2;
  bb->half_width_ = std::ceil(radius) + 2;
  bb->height_ = std::ceil(2.0f * radius) + 2;
  bb->half_height_ = std::ceil(radius) + 2;
}

RobotObstacle::RobotObstacle(const pose_2d::Pose2Df& pose)
    : Obstacle(ObstacleType::ROBOT), pose_(pose), bounding_box_(0, 0, 0, 0) {
  // Set internal representation of center and radius
  UpdateBoundingBox(&bounding_box_, pose_, radius_);
}

RobotObstacle& RobotObstacle::operator=(const RobotObstacle& other) {
  if (this != &other) {
    pose_ = other.GetPose();
    UpdateBoundingBox(&bounding_box_, pose_, radius_);
  }
  return *this;
}

bool RobotObstacle::PointCollision(const Eigen::Vector2f& point,
                                   float margin) const {
  // If the distance from the point to the center of the circle is less than the
  // radius, there is a collision. Otherwise there is no collision. Obstacles
  // shall be defined such that the robot can safely sit on their edge.
  if (bounding_box_.PointCollision(point, margin)) {
    const float distance = (point - pose_.translation).squaredNorm();
    return distance < Sq(radius_ + margin);
  }
  return false;
}

bool RobotObstacle::LineCollision(const Eigen::Vector2f& line_start,
                                  const Eigen::Vector2f& line_end,
                                  float margin) const {
  // If the distance from the closest point on the line to the center of
  // the circle is less than the radius, there is a collision. Otherwise there
  // is no collision. Obstacles shall be defined such that the robot can safely
  // sit on their edge.
  float squared_distance;
  Eigen::Vector2f projection;

  ProjectPointOntoLineSegment(pose_.translation, line_start, line_end,
                              &projection, &squared_distance);

  return squared_distance < Sq(radius_ + margin);
}

float RobotObstacle::PointDistance(const Eigen::Vector2f& point,
                                   float margin) const {
  // The distance between the two points is the magnitude of the difference
  // vector.
  float distance = (point - pose_.translation).norm();
  return fmax(distance - radius_ - margin, 0);
}

float RobotObstacle::LineDistance(const Eigen::Vector2f& line_start,
                                  const Eigen::Vector2f& line_end,
                                  float margin) const {
  float squared_distance;
  Eigen::Vector2f projection;

  ProjectPointOntoLineSegment(pose_.translation, line_start, line_end,
                              &projection, &squared_distance);

  return fmax(std::sqrt(squared_distance) - radius_ - margin, 0);
}

Eigen::Vector2f RobotObstacle::NearestFreePoint(const Eigen::Vector2f& point,
                                                float margin) const {
  // Get a vector from the center to the point
  Eigen::Vector2f nearest_point = point - pose_.translation;

  if (fabs(nearest_point.x()) <= kEpsilon &&
      fabs(nearest_point.y()) <= kEpsilon) {
    nearest_point.x() += -radius_ + margin + kEpsilonPad;
  } else if (SafeVectorNorm(nearest_point) < radius_ + margin + kEpsilonPad) {
    // Normalize that vector
    nearest_point.normalize();

    // Multiply by the radius
    nearest_point *= radius_ + margin + kEpsilonPad;
  }

  // Translate by the center
  nearest_point += pose_.translation;
  return nearest_point;
}

bool RobotObstacle::FurthestFreePointOnLine(const Eigen::Vector2f& line_start,
                                            const Eigen::Vector2f& line_end,
                                            Eigen::Vector2f* free_point,
                                            float* distance_from_start,
                                            const float margin) const {
  // Handle case where line starts inside of the obstacle.
  if (PointCollision(line_start, margin)) {
    *free_point = line_start;
    *distance_from_start = 0;
    return true;
  }
  if (!LineCollision(line_start, line_end, margin)) {
    *free_point = line_end;
    *distance_from_start = (line_end - line_start).norm();
    NP_CHECK_MSG(!PointCollision((*free_point), margin),
                 "Margin: " << std::setprecision(20) << margin);
    return false;
  }
  Vector2f projected_point(0, 0);
  float current_distance = std::numeric_limits<float>::max();
  ProjectPointOntoLineSegment(pose_.translation, line_start, line_end,
                              &projected_point, &current_distance);

  Vector2f translation_vector = projected_point - line_start;
  constexpr float kNumericInstabilityThreshold = 0.001f;
  const float slide_back_distance =
      sqrt(Sq(radius_ + margin) -
           (projected_point - pose_.translation).squaredNorm()) +
      // Additional padding is to deal with the fact that the computation above
      // is not numerically stable.
      kEpsilon + kNumericInstabilityThreshold;
  translation_vector -=
      (geometry::GetNormalizedOrZero(translation_vector) * slide_back_distance);

  *free_point = line_start + translation_vector;
  *distance_from_start = translation_vector.norm();

  NP_CHECK_MSG(!PointCollision(line_start, margin),
               "Margin: " << std::setprecision(20) << margin);
  return true;
}

Pose2Df RobotObstacle::GetPose() const { return pose_; }

void RobotObstacle::UpdatePose(const pose_2d::Pose2Df& pose) {
  pose_ = pose;
  UpdateBoundingBox(&bounding_box_, pose_, radius_);
}

bool RobotObstacle::GetTangents(const Vector2f& desired_point,
                                const float margin, Vector2f* right_tangent,
                                Vector2f* left_tangent) const {
  if (PointCollision(desired_point, margin)) {
    return false;
  }
  geometry::GetTangentPoints(desired_point, pose_.translation, radius_ + margin,
                             right_tangent, left_tangent);
  return true;
}

const float RobotObstacle::GetRadius() const { return radius_; }

IntegerBoundingBox RobotObstacle::GetInternalBoundingBox() const {
  const int axis_distance_from_center =
      static_cast<int>(std::floor(radius_ * kSqrtTwo / 2.0f)) - 1;
  const int x = static_cast<int>(pose_.translation.x());
  const int y = static_cast<int>(pose_.translation.y());
  return IntegerBoundingBox(x, y, axis_distance_from_center * 2,
                            axis_distance_from_center * 2);
}

IntegerBoundingBox RobotObstacle::GetExternalBoundingBox() const {
  return bounding_box_;
}

void RobotObstacle::DrawObstacle(logger::Logger* local_logger) const {}

}  // namespace obstacle
