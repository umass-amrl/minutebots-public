// Copyright 2017 - 2018 joydeepb@cs.umass.edu, kvedder@umass.edu
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
#include <glog/logging.h>
#include <cmath>
#include <iostream>

#include "eigen3/Eigen/Core"
#include "math/geometry.h"
#include "math/math_util.h"
#include "shared/common_includes.h"
#include "third_party/googletest/googletest-release-1.8.0/googletest/include/gtest/gtest.h"

using Eigen::Vector2f;
using Eigen::Vector2d;
using geometry::Angle;
using geometry::Cross;
using geometry::IsParallel;
using geometry::IsPerpendicular;
using geometry::FurthestFreePointCircle;
using geometry::GetTangentPoints;
using geometry::CheckLineLineCollision;
using geometry::LineLineIntersection;
using geometry::CheckLineLineIntersection;
using geometry::Perp;
using geometry::RayIntersect;
using geometry::ProjectPointOntoLineSegment;

TEST(GeometryTest, Perp) {
  ASSERT_EQ(Vector2f(1, 0), Perp(Vector2f(0, -1)));
  ASSERT_EQ(Vector2f(0, 1), Perp(Vector2f(1, 0)));
  ASSERT_EQ(Vector2f(0, 0), Perp(Vector2f(0, 0)));
}

TEST(GeometryTest, Cross) {
  ASSERT_FLOAT_EQ(0.0f, Cross(Vector2f(0, 1), Vector2f(0, 2)));
  ASSERT_FLOAT_EQ(2.0, Cross(Vector2f(1, 0), Vector2f(0, 2)));
  ASSERT_DOUBLE_EQ(-3.0, Cross(Vector2d(0, 3), Vector2d(1, 0)));
}

TEST(GeometryTest, IsParallel) {
  ASSERT_TRUE(IsParallel(Vector2f(1, 1), Vector2f(2, 2)));
  ASSERT_TRUE(IsParallel(Vector2f(1, 1), Vector2f(-1, -1)));
  ASSERT_FALSE(IsParallel(Vector2f(0, 1), Vector2f(1, 1.5)));
}

TEST(GeometryTest, IsPerpendicular) {
  ASSERT_TRUE(IsPerpendicular(Vector2f(1, 0), Vector2f(0, 1)));
  ASSERT_FALSE(IsPerpendicular(Vector2f(1, 0), Vector2f(0.5, 1)));
}

TEST(GeometryTest, CheckLineLineIntersection) {
  {
    const Vector2d a0(-1, 0);
    const Vector2d a1(1, 0);
    const Vector2d b0(0, -1);
    const Vector2d b1(0, 1);
    const Vector2d result(0, 0);
    const auto intr = CheckLineLineIntersection(a0, a1, b0, b1);
    ASSERT_TRUE(CheckLineLineCollision(a0, a1, b0, b1));
    ASSERT_TRUE(intr.first);
    EXPECT_DOUBLE_EQ(result.x(), intr.second.x());
    EXPECT_DOUBLE_EQ(result.y(), intr.second.y());
  }
  {
    const Vector2d a0(1, 0);
    const Vector2d a1(0, 1);
    const Vector2d b0(0, 0);
    const Vector2d b1(1, 1);
    const Vector2d result(0.5, 0.5);
    const auto intr = CheckLineLineIntersection(a0, a1, b0, b1);
    ASSERT_TRUE(CheckLineLineCollision(a0, a1, b0, b1));
    ASSERT_TRUE(intr.first);
    EXPECT_DOUBLE_EQ(result.x(), intr.second.x());
    EXPECT_DOUBLE_EQ(result.y(), intr.second.y());
  }
  {
    const Vector2d a0(1, 0);
    const Vector2d a1(0, 1);
    const Vector2d b0(0, 0);
    const Vector2d b1(0.1, 0.1);
    const auto intr = CheckLineLineIntersection(a0, a1, b0, b1);
    ASSERT_FALSE(CheckLineLineCollision(a0, a1, b0, b1));
    ASSERT_FALSE(intr.first);
  }
  {
    const Vector2d a0(1, 0);
    const Vector2d a1(0.8, 0.2);
    const Vector2d b0(0, 0);
    const Vector2d b1(1, 1);
    const auto intr = CheckLineLineIntersection(a0, a1, b0, b1);
    ASSERT_FALSE(CheckLineLineCollision(a0, a1, b0, b1));
    ASSERT_FALSE(intr.first);
  }
}

TEST(GeometryTest, LineLineIntersection) {
  {
    const Vector2d a0(-1, 0);
    const Vector2d a1(1, 0);
    const Vector2d b0(0, -1);
    const Vector2d b1(0, 1);
    const Vector2d result(0, 0);
    ASSERT_DOUBLE_EQ(result.x(), LineLineIntersection(a0, a1, b0, b1).x());
    ASSERT_DOUBLE_EQ(result.y(), LineLineIntersection(a0, a1, b0, b1).y());
  }
  {
    const Vector2d a0(1, 0);
    const Vector2d a1(0, 1);
    const Vector2d b0(0, 0);
    const Vector2d b1(1, 1);
    const Vector2d result(0.5, 0.5);
    ASSERT_DOUBLE_EQ(result.x(), LineLineIntersection(a0, a1, b0, b1).x());
    ASSERT_DOUBLE_EQ(result.y(), LineLineIntersection(a0, a1, b0, b1).y());
  }
  {
    const Vector2d a0(1, 0);
    const Vector2d a1(0, 1);
    const Vector2d b0(0, 0);
    const Vector2d b1(0.1, 0.1);
    const Vector2d result(0.5, 0.5);
    ASSERT_DOUBLE_EQ(result.x(), LineLineIntersection(a0, a1, b0, b1).x());
    ASSERT_DOUBLE_EQ(result.y(), LineLineIntersection(a0, a1, b0, b1).y());
  }
  {
    const Vector2d a0(1, 0);
    const Vector2d a1(0.8, 0.2);
    const Vector2d b0(0, 0);
    const Vector2d b1(1, 1);
    const Vector2d result(0.5, 0.5);
    ASSERT_DOUBLE_EQ(result.x(), LineLineIntersection(a0, a1, b0, b1).x());
    ASSERT_DOUBLE_EQ(result.y(), LineLineIntersection(a0, a1, b0, b1).y());
  }
  {
    const Vector2d a0(1, 1);
    const Vector2d a1(2, 3);
    const Vector2d b0(4, 5);
    const Vector2d b1(8, 9);
    const Vector2d result(2, 3);
    ASSERT_DOUBLE_EQ(result.x(), LineLineIntersection(a0, a1, b0, b1).x());
    ASSERT_DOUBLE_EQ(result.y(), LineLineIntersection(a0, a1, b0, b1).y());
  }
}

TEST(GeometryTest, GetTangentPoints) {
  {
    const Vector2d p(5, 0);
    const Vector2d c(0, 0);
    const double r = 3;
    const Vector2d t0_expected(1.8, 2.4);
    const Vector2d t1_expected(1.8, -2.4);
    Vector2d t0, t1;
    GetTangentPoints(p, c, r, &t0, &t1);
    ASSERT_DOUBLE_EQ(t0.x(), t0_expected.x());
    ASSERT_DOUBLE_EQ(t0.y(), t0_expected.y());
    ASSERT_DOUBLE_EQ(t1.x(), t1_expected.x());
    ASSERT_DOUBLE_EQ(t1.y(), t1_expected.y());
  }
  {
    const Vector2d p(0, 5);
    const Vector2d c(0, 0);
    const double r = 3;
    const Vector2d t0_expected(-2.4, 1.8);
    const Vector2d t1_expected(2.4, 1.8);
    Vector2d t0, t1;
    GetTangentPoints(p, c, r, &t0, &t1);
    ASSERT_DOUBLE_EQ(t0.x(), t0_expected.x());
    ASSERT_DOUBLE_EQ(t0.y(), t0_expected.y());
    ASSERT_DOUBLE_EQ(t1.x(), t1_expected.x());
    ASSERT_DOUBLE_EQ(t1.y(), t1_expected.y());
  }
}

TEST(GeometryTest, Angle) {
  ASSERT_DOUBLE_EQ(0.25 * M_PI, Angle(Vector2d(0.5, 0.5)));
  ASSERT_DOUBLE_EQ(0.5 * M_PI, Angle(Vector2d(0, 0.75)));
  ASSERT_DOUBLE_EQ(-0.5 * M_PI, Angle(Vector2d(0, -0.5)));
  ASSERT_DOUBLE_EQ(0.75 * M_PI, Angle(Vector2d(-0.5, 0.5)));
  ASSERT_DOUBLE_EQ(-0.75 * M_PI, Angle(Vector2d(-0.25, -0.25)));
}

TEST(GeometryTest, Heading) {
  {
    const float angle = 0.0;
    const Vector2f v = Heading(angle);
    ASSERT_FLOAT_EQ(1.0, v.x());
    ASSERT_FLOAT_EQ(0.0, v.y());
  }
  {
    const float angle = 0.25 * M_PI;
    const Vector2f v = Heading(angle);
    ASSERT_FLOAT_EQ(1.0 / sqrt(2.0), v.x());
    ASSERT_FLOAT_EQ(1.0 / sqrt(2.0), v.y());
  }
  {
    const float angle = 2.0 * M_PI / 3.0;
    const Vector2f v = Heading(angle);
    ASSERT_FLOAT_EQ(-0.5, v.x());
    ASSERT_FLOAT_EQ(0.5 * sqrt(3.0), v.y());
  }
  {
    const float angle = -M_PI / 6.0;
    const Vector2f v = Heading(angle);
    ASSERT_FLOAT_EQ(0.5 * sqrt(3.0), v.x());
    ASSERT_FLOAT_EQ(-0.5, v.y());
  }
}

TEST(GeometryTest, RayIntersect) {
  {
    // Does not intersect case
    const Vector2d r0(-1, 0);
    const Vector2d r1(-1, 0);
    const Vector2d p0(0, -1);
    const Vector2d p1(0, 1);
    const double true_distance = -1;
    double result_distance;
    Vector2d intersect_point;
    bool intersects =
        RayIntersect(r0, r1, p0, p1, &result_distance, &intersect_point);
    ASSERT_DOUBLE_EQ(result_distance, true_distance);
    ASSERT_FALSE(intersects);
  }
  {
    // Intersects case
    const Vector2d r0(-1, 0);
    const Vector2d r1(1, 0);
    const Vector2d p0(0, -1);
    const Vector2d p1(0, 1);
    const double true_distance = 1;
    double result_distance;
    Vector2d intersect_point;
    bool intersects =
        RayIntersect(r0, r1, p0, p1, &result_distance, &intersect_point);
    ASSERT_DOUBLE_EQ(result_distance, true_distance);
    ASSERT_TRUE(intersects);
  }
  {
    // ray start is on the line case
    const Vector2d r0(0, 0);
    const Vector2d r1(1, 0);
    const Vector2d p0(0, -1);
    const Vector2d p1(0, 1);
    const double true_distance = 0;
    double result_distance;
    Vector2d intersect_point;
    bool intersects =
        RayIntersect(r0, r1, p0, p1, &result_distance, &intersect_point);
    ASSERT_DOUBLE_EQ(result_distance, true_distance);
    ASSERT_TRUE(intersects);
  }
  //  {
  //  // Ray is colinear with line segment and closest to p0
  //  const Vector2d r0(0, -2);
  //  const Vector2d r1(0, 1);
  //  const Vector2d p0(0, -1);
  //  const Vector2d p1(0, 1);
  //  const double true_distance = 1;
  //  double result_distance;
  //  bool intersects = RayIntersect(r0, r1, p0, p1, &result_distance);
  //  ASSERT_DOUBLE_EQ(result_distance, true_distance);
  //  ASSERT_TRUE(intersects);
  //  }
  //  {
  //  // Ray is colinear with line segment and closest to p1
  //  const Vector2d r0(0, 2);
  //  const Vector2d r1(0, -1);
  //  const Vector2d p0(0, -1);
  //  const Vector2d p1(0, 1);
  //  const double true_distance = 1;
  //  double result_distance;
  //  bool intersects = RayIntersect(r0, r1, p0, p1, &result_distance);
  //  ASSERT_DOUBLE_EQ(result_distance, true_distance);
  //  ASSERT_TRUE(intersects);
  //  }
}

TEST(GeometryTest, ProjectPointOntoLineSegment) {
  {
    const Vector2f line_start(100, 0);
    const Vector2f line_end(200, 0);
    const Vector2f point(50, 0);

    Vector2f projected_point;
    float squared_distance;

    ProjectPointOntoLineSegment(point, line_start, line_end, &projected_point,
                                &squared_distance);

    ASSERT_FLOAT_EQ(squared_distance, 2500.0f);
    ASSERT_FLOAT_EQ(projected_point.x(), line_start.x());
    ASSERT_FLOAT_EQ(projected_point.y(), line_start.y());
  }
  {
    const Vector2f line_start(100, 0);
    const Vector2f line_end(200, 0);
    const Vector2f point(100, 50);

    Vector2f projected_point;
    float squared_distance;

    ProjectPointOntoLineSegment(point, line_start, line_end, &projected_point,
                                &squared_distance);

    ASSERT_FLOAT_EQ(squared_distance, 2500.0f);
    ASSERT_FLOAT_EQ(projected_point.x(), line_start.x());
    ASSERT_FLOAT_EQ(projected_point.y(), line_start.y());
  }
  {
    const Vector2f line_start(100, 0);
    const Vector2f line_end(200, 0);
    const Vector2f point(250, 0);

    Vector2f projected_point;
    float squared_distance;

    ProjectPointOntoLineSegment(point, line_start, line_end, &projected_point,
                                &squared_distance);

    ASSERT_FLOAT_EQ(squared_distance, 2500.0f);
    ASSERT_FLOAT_EQ(projected_point.x(), line_end.x());
    ASSERT_FLOAT_EQ(projected_point.y(), line_end.y());
  }
  {
    const Vector2f line_start(100, 0);
    const Vector2f line_end(200, 0);
    const Vector2f point(200, 50);

    Vector2f projected_point;
    float squared_distance;

    ProjectPointOntoLineSegment(point, line_start, line_end, &projected_point,
                                &squared_distance);

    ASSERT_FLOAT_EQ(squared_distance, 2500.0f);
    ASSERT_FLOAT_EQ(projected_point.x(), line_end.x());
    ASSERT_FLOAT_EQ(projected_point.y(), line_end.y());
  }
  {
    const Vector2f line_start(100, 0);
    const Vector2f line_end(200, 0);
    const Vector2f point(150, 50);

    Vector2f projected_point;
    float squared_distance;

    ProjectPointOntoLineSegment(point, line_start, line_end, &projected_point,
                                &squared_distance);

    ASSERT_FLOAT_EQ(squared_distance, 2500.0f);
    ASSERT_FLOAT_EQ(projected_point.x(), 150.0f);
    ASSERT_FLOAT_EQ(projected_point.y(), 0.0f);
  }
}

TEST(ObstaclePrimitiveTests, FurthestFreePointCircleTest) {
  {
    Vector2f circle_center(100, 10);
    float circle_radius = 100.0f;
    Vector2f line_start(-200, 0);
    Vector2f line_end(200, 0);

    Vector2f intersect_point;
    float squared_distance;
    ASSERT_TRUE(FurthestFreePointCircle(line_start, line_end, circle_center,
                                        circle_radius, &squared_distance,
                                        &intersect_point));
  }
  {
    Vector2f circle_center(100, 10);
    float circle_radius = 100.0f;
    Vector2f line_start(-200, 200);
    Vector2f line_end(200, 200);

    Vector2f intersect_point;
    float squared_distance;
    ASSERT_FALSE(FurthestFreePointCircle(line_start, line_end, circle_center,
                                         circle_radius, &squared_distance,
                                         &intersect_point));
  }
}
