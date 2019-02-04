// Copyright 2017-2018 slane@cs.umass.edu, kvedder@umass.edu
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

#include "obstacles/rectangle_obstacle.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <utility>
#include <vector>
#include "constants/constants.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "logging/logger.h"
#include "math/geometry.h"
#include "obstacles/obstacle.h"
#include "obstacles/obstacle_type.h"

STANDARD_USINGS;
using geometry::ProjectPointOntoLineSegment;
using geometry::RayIntersect;

namespace obstacle {

RectangleObstacle::RectangleObstacle(const pose_2d::Pose2Df& pose,
                                     ObstacleType type, float width,
                                     float height)
    : Obstacle(type), width_(width), height_(height) {
  // Sets the center pose as well as the corner corner coordinates.
  UpdatePose(pose);
}

bool RectangleObstacle::PointCollision(const Vector2f& point,
                                       const float margin) const {
  // Translate and rotate point to basis defined by the lines 0->1 and 0->3,
  // and compute the delta of it to the center.
  const Vector2f transformed_point =
      ((Eigen::Rotation2Df(-pose_.angle) * point) - pose_.translation)
          .cwiseAbs();

  if (transformed_point.x() > ((width_ / 2.0f) + margin)) {
    return false;
  }
  if (transformed_point.y() > ((height_ / 2.0f) + margin)) {
    return false;
  }
  return true;
}

bool RectangleObstacle::LineCollision(const Vector2f& line_start,
                                      const Vector2f& line_end,
                                      float margin) const {
  // Check if end points are inside rectangle
  if (PointCollision(line_end, margin)) {
    return true;
  }
  if (PointCollision(line_start, margin)) {
    return true;
  }
  // For each side
  for (int i = 0; i < 4; i++) {
    const int index_a = i;
    // Performs (i + 1) % 4
    const int index_b = (i + 1) & 0b11;

    const Vector2f margin_sized_edge_hat_norm =
        (corner_coordinates_[index_a] - corner_coordinates_[index_b])
            .normalized() *
        margin;

    const Vector2f edge_point_a =
        corner_coordinates_[index_a] + margin_sized_edge_hat_norm;
    const Vector2f edge_point_b =
        corner_coordinates_[index_b] - margin_sized_edge_hat_norm;

    const Vector2f margin_sized_edge_hat_perp =
        Perp(margin_sized_edge_hat_norm);

    // Check if line segment collides with side of rectangle
    // Lines can be written as x(t) = point_a + t*point_b
    // Two lines intersect_vector iff you can find a solution to :
    // point_a1 + t1*(point_b1-point_a1) = point_a2 + t2*(point_b2-point_a2)
    // If both t1 and t2 are in the range [0,1] then the line segments
    // Intesect
    // The above equation can be rewritten as a matrix expression (assuming
    // the points are column vectors:
    // point_a1 - pointa2 = [point_b2-point_a2, point_b1-point_a2][t2;-t1]
    // Since it's 2x2 we simply write out the solution
    const Vector2f translated_edge_point =
        (edge_point_b - edge_point_a) + margin_sized_edge_hat_perp;
    const Vector2f translated_line_point = line_end - line_start;
    const float determinate =
        translated_edge_point[0] * translated_line_point[1] -
        translated_edge_point[1] * translated_line_point[0];

    const float edge_intersection =
        ((line_start[0] - edge_point_a[0]) * translated_line_point[1] -
         (line_start[1] - edge_point_a[1]) * translated_line_point[0]) /
        determinate;
    const float line_intersection =
        (-(-(line_start[0] - edge_point_a[0]) * translated_edge_point[1] +
           (line_start[1] - edge_point_a[1]) * translated_edge_point[0])) /
        determinate;

    if (edge_intersection <= 1.0f && edge_intersection >= 0.0f &&
        line_intersection <= 1.0f && line_intersection >= 0.0f) {
      return true;
    }
  }

  return false;
}

float RectangleObstacle::PointDistance(const Vector2f& point,
                                       float margin) const {
  // Check if the point is within the rectangle
  if (PointCollision(point, margin)) {
    // If there is a collision, the point is within the obstacle so the
    // distance is 0.
    return 0.0f;
  }
  // Otherwise, find the nearest point on the sides of the obstacle and
  // measure that distance.
  return (NearestFreePoint(point, margin) - point).norm();
}

float RectangleObstacle::LineDistance(const Vector2f& line_start,
                                      const Vector2f& line_end,
                                      float margin) const {
  if (LineCollision(line_start, line_end, margin)) {
    return 0.0f;
  }
  // TODO(slane): Check math
  float min_distance = std::numeric_limits<float>::max();
  for (int i = 0; i < 4; i++) {
    int index_a = i;
    int index_b = (i + 1) % 4;

    Vector2f edge_point_a = corner_coordinates_[index_a];
    Vector2f edge_point_b = corner_coordinates_[index_b];

    const Vector2f margin_sized_edge_hat_norm =
        (edge_point_a - edge_point_b).normalized() * margin;
    const Vector2f margin_sized_edge_hat_perp =
        Perp(margin_sized_edge_hat_norm);

    edge_point_a += margin_sized_edge_hat_norm;
    edge_point_a += margin_sized_edge_hat_perp;
    edge_point_b -= margin_sized_edge_hat_norm;
    edge_point_b += margin_sized_edge_hat_perp;

    // Get the intersection point as in LineCollision
    const float determinate =
        edge_point_a.x() * line_end.y() - edge_point_b.y() * line_end.x();
    // edge_point_a[0] * line_end[1] - edge_point_b[1] * line_end[0];

    float edge_intersection = (line_start[0] - edge_point_a[0]) * line_end[1] -
                              (line_start[1] - edge_point_a[1]) * line_end[0];
    float line_intersection =
        -(-(line_start[0] - edge_point_a[0]) * edge_point_b[1] +
          (line_start[1] - edge_point_a[1]) * edge_point_b[0]);

    edge_intersection /= determinate;
    line_intersection /= determinate;

    // Check if the lines intersect
    if (edge_intersection <= 1.0f && edge_intersection >= 0.0f &&
        line_intersection <= 1.0f && line_intersection >= 0.0f) {
      return 0.0f;
    }

    // Cap the collision point parameters so they lie on the line segments
    if (edge_intersection > 1.0f) {
      edge_intersection = 1.0f;
    } else if (edge_intersection < 0.0f) {
      edge_intersection = 0.0f;
    }

    if (line_intersection > 1.0f) {
      line_intersection = 1.0f;
    } else if (line_intersection < 0.0f) {
      line_intersection = 0.0f;
    }

    // Get the two corresponding points
    const Vector2f closest_line_point =
        line_start + line_intersection * line_end;
    const Vector2f closest_edge_point =
        edge_point_a + edge_intersection * edge_point_b;

    // Find their distance.
    float current_distance = (closest_edge_point - closest_line_point).norm();
    if (current_distance < min_distance) {
      min_distance = current_distance;
    }
  }
  return min_distance;
}

Vector2f RectangleObstacle::NearestFreePoint(const Vector2f& point,
                                             float margin) const {
  // Project the point onto the line segment, capping it at the two end points
  // TODO(slane): This is similar to point collision in terms of projection,
  // this shouln't need to redo that calculation

  if (!PointCollision(point, margin)) {
    return point;
  }

  Vector2f min_projection;
  float min_distance;

  const Vector2f direction_norm =
      (corner_coordinates_[0] - corner_coordinates_[1]).normalized() * margin;
  const Vector2f direction_perp = Perp(direction_norm);

  const Vector2f corner_inflated0 =
      (corner_coordinates_[0] + direction_perp + direction_norm);
  const Vector2f corner_inflated1 =
      (corner_coordinates_[1] + direction_perp - direction_norm);

  ProjectPointOntoLineSegment(point, corner_inflated0, corner_inflated1,
                              &min_projection, &min_distance);

  // For each remaining line segment in the rectangle
  for (int i = 1; i < 4; i++) {
    int index_a = i;
    int index_b = (i + 1) % 4;

    float current_distance;
    Vector2f current_projection;

    const Vector2f margin_sized_direction_norm =
        (corner_coordinates_[index_a] - corner_coordinates_[index_b])
            .normalized() *
        margin;
    const Vector2f margin_sized_direction_perp =
        Perp(margin_sized_direction_norm);

    const Vector2f corner_inflateda =
        (corner_coordinates_[index_a] + margin_sized_direction_perp +
         margin_sized_direction_norm);
    const Vector2f corner_inflatedb =
        (corner_coordinates_[index_b] + margin_sized_direction_perp -
         margin_sized_direction_norm);

    ProjectPointOntoLineSegment(point, corner_inflateda, corner_inflatedb,
                                &current_projection, &current_distance);

    // Update min projection
    if (current_distance < min_distance) {
      min_distance = current_distance;
      min_projection = current_projection;
    }
  }

  Vector2f delta;
  if (min_distance > kEpsilon) {
    delta =
        geometry::GetNormalizedOrZero(Eigen::Vector2f(min_projection - point)) *
        kEpsilonPad;
  } else {
    delta = geometry::GetNormalizedOrZero(
                Eigen::Vector2f(min_projection - pose_.translation)) *
            kEpsilonPad;
  }

  return min_projection + delta;
}

std::pair<Vector2f, Vector2f> InflateCornersByMargin(const Vector2f& c1,
                                                     const Vector2f& c2,
                                                     const float margin) {
  const Vector2f margin_norm =
      geometry::GetNormalizedOrZero(Vector2f(c1 - c2)) * margin;
  const Vector2f margin_perp = geometry::Perp(margin_norm);
  return {c1 + margin_norm + margin_perp, c2 - margin_norm + margin_perp};
}

bool RectangleObstacle::FurthestFreePointOnLine(const Vector2f& line_start,
                                                const Vector2f& line_end,
                                                Vector2f* free_point,
                                                float* distance_from_start,
                                                float margin) const {
  Vector2f min_point = line_end;
  float min_distance = (line_end - line_start).norm();
  bool is_colliding = false;
  // Check if end points are inside rectangle
  if (PointCollision(line_start, margin)) {
    *free_point = line_start;
    return true;
  }
  // For each side
  for (size_t i = 0; i < 4; i++) {
    const size_t index_a = i;
    const size_t index_b = (i + 1) % 4;
    const auto inflated_corners = InflateCornersByMargin(
        corner_coordinates_[index_a], corner_coordinates_[index_b], margin);
    const Vector2f& inflated_corner_a = inflated_corners.first;
    const Vector2f& inflated_corner_b = inflated_corners.second;

    const auto intersection_result = geometry::CheckLineLineIntersection(
        line_start, line_end, inflated_corner_a, inflated_corner_b);
    if (intersection_result.first) {
      is_colliding = true;
      const float intersect_distance =
          (intersection_result.second - line_start).norm();

      if (min_distance > intersect_distance) {
        min_distance = intersect_distance;
        min_point = intersection_result.second;
      }
    }
  }
  constexpr float kNumericInstabilityThreshold = 0.0015f;
  *distance_from_start = min_distance - kNumericInstabilityThreshold;
  *free_point =
      (min_point -
       geometry::GetNormalizedOrZero(Vector2f(min_point - line_start)) *
           kNumericInstabilityThreshold);
  if (!kProduction && PointCollision(*free_point, margin)) {
    LOG(ERROR) << "PointCollision; Free point: "
               << free_point->x() << ", " << free_point->y()
               << " Margin: " << margin << "Obstacle";
  }
  return is_colliding;
}

pose_2d::Pose2Df RectangleObstacle::GetPose() const { return pose_; }

void RectangleObstacle::UpdatePose(const pose_2d::Pose2Df& pose) {
  pose_ = pose;

  // Start by setting corners to their locations without translation or rotation
  corner_coordinates_[0] << -width_ / 2.0f, -height_ / 2.0f;
  corner_coordinates_[1] << width_ / 2.0f, -height_ / 2.0f;
  corner_coordinates_[2] << width_ / 2.0f, height_ / 2.0f;
  corner_coordinates_[3] << -width_ / 2.0f, height_ / 2.0f;

  // Apply rotation matrix
  Eigen::Rotation2D<float> rotation(pose_.angle);

  corner_coordinates_[0] = rotation * corner_coordinates_[0];
  corner_coordinates_[1] = rotation * corner_coordinates_[1];
  corner_coordinates_[2] = rotation * corner_coordinates_[2];
  corner_coordinates_[3] = rotation * corner_coordinates_[3];

  // Apply Translation
  corner_coordinates_[0] += pose_.translation;
  corner_coordinates_[1] += pose_.translation;
  corner_coordinates_[2] += pose_.translation;
  corner_coordinates_[3] += pose_.translation;

  int min_x = std::numeric_limits<int>::max();
  int min_y = std::numeric_limits<int>::max();
  int max_x = std::numeric_limits<int>::min();
  int max_y = std::numeric_limits<int>::min();
  for (size_t i = 0; i < 4; ++i) {
    min_x = std::min(static_cast<int>(
        std::floor(corner_coordinates_[i].x())), min_x);
    min_y = std::min(static_cast<int>(
        std::floor(corner_coordinates_[i].y())), min_y);
    max_x = std::max(static_cast<int>(
        std::ceil(corner_coordinates_[i].x())), max_x);
    max_y = std::max(static_cast<int>(
        std::ceil(corner_coordinates_[i].y())), max_y);
  }

  bounding_box_.x_ = (max_x + min_x) / 2;
  bounding_box_.y_ = (max_y + min_y) / 2;
  bounding_box_.half_width_ = (max_x - min_x + 1) / 2;
  bounding_box_.half_height_ = (max_y - min_y + 1) / 2;
  bounding_box_.width_ = max_x - min_x;
  bounding_box_.height_ = max_y - min_y;
}

bool RectangleObstacle::GetTangents(const Vector2f& desired_point,
                                    const float margin,
                                    Vector2f* right_tangent_point,
                                    Vector2f* left_tangent_point) const {
  if (PointCollision(desired_point, margin)) {
    return false;
  }

  Vector2f current_right_tangent_point = corner_coordinates_[0];
  Vector2f current_left_tangent_point = corner_coordinates_[0];

  // Find right tangent.
  for (int i = 0; i < 4; ++i) {
    const Vector2f& raw_corner = corner_coordinates_[i];
    const Vector2f& next_corner = corner_coordinates_[(i + 1) % 4];

    const Vector2f edge_norm = (next_corner - raw_corner).normalized();
    const Vector2f edge_perp = Perp(edge_norm);

    const Vector2f corner_inflated =
        (raw_corner - pose_.translation).normalized() * kEpsilon + raw_corner -
        edge_norm * margin - edge_perp * margin;
    const Vector2f proposed_tangent_ray = corner_inflated - desired_point;
    const Vector2f current_right_tangent_ray =
        current_right_tangent_point - desired_point;
    const Vector2f current_left_tangent_ray =
        current_left_tangent_point - desired_point;
    if (geometry::Cross(current_right_tangent_ray, proposed_tangent_ray) < 0) {
      current_right_tangent_point = corner_inflated;
    }
    if (geometry::Cross(current_left_tangent_ray, proposed_tangent_ray) > 0) {
      current_left_tangent_point = corner_inflated;
    }
  }

  *right_tangent_point = current_right_tangent_point;
  *left_tangent_point = current_left_tangent_point;
  return true;
}

void RectangleObstacle::SetHeightWidth(const float height, const float width) {
  height_ = height;
  width_ = width;
  // Start by setting corners to their locations without translation or rotation
  corner_coordinates_[0] << -width_ / 2.0f, -height_ / 2.0f;
  corner_coordinates_[1] << width_ / 2.0f, -height_ / 2.0f;
  corner_coordinates_[2] << width_ / 2.0f, height_ / 2.0f;
  corner_coordinates_[3] << -width_ / 2.0f, height_ / 2.0f;

  // Apply rotation matrix
  Eigen::Rotation2D<float> rotation(pose_.angle);

  corner_coordinates_[0] = rotation * corner_coordinates_[0];
  corner_coordinates_[1] = rotation * corner_coordinates_[1];
  corner_coordinates_[2] = rotation * corner_coordinates_[2];
  corner_coordinates_[3] = rotation * corner_coordinates_[3];

  // Apply Translation
  corner_coordinates_[0] += pose_.translation;
  corner_coordinates_[1] += pose_.translation;
  corner_coordinates_[2] += pose_.translation;
  corner_coordinates_[3] += pose_.translation;
}

const float RectangleObstacle::GetRadius() const {
  return std::max(width_, height_);
}

IntegerBoundingBox RectangleObstacle::GetInternalBoundingBox() const {
  const int& x = static_cast<int>(pose_.translation.x());
  const int& y = static_cast<int>(pose_.translation.y());

  int width_int = static_cast<int>(width_) - 1;
  int height_int = static_cast<int>(height_) - 1;

  width_int -= width_int % 2;
  height_int -= height_int % 2;

  return IntegerBoundingBox(x, y, width_int, height_int);
}

IntegerBoundingBox RectangleObstacle::GetExternalBoundingBox() const {
  return bounding_box_;
}

const float RectangleObstacle::GetHeight() const { return height_; }

const float RectangleObstacle::GetWidth() const { return width_; }

void RectangleObstacle::DrawObstacle(logger::Logger* local_logger) const {
  local_logger->AddLine(corner_coordinates_[0].x(), corner_coordinates_[0].y(),
                        corner_coordinates_[1].x(), corner_coordinates_[1].y(),
                        0, 0, 0, 1);

  local_logger->AddLine(corner_coordinates_[1].x(), corner_coordinates_[1].y(),
                        corner_coordinates_[2].x(), corner_coordinates_[2].y(),
                        0, 0, 0, 1);

  local_logger->AddLine(corner_coordinates_[2].x(), corner_coordinates_[2].y(),
                        corner_coordinates_[3].x(), corner_coordinates_[3].y(),
                        0, 0, 0, 1);

  local_logger->AddLine(corner_coordinates_[3].x(), corner_coordinates_[3].y(),
                        corner_coordinates_[0].x(), corner_coordinates_[0].y(),
                        0, 0, 0, 1);
}

}  // namespace obstacle
