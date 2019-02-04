// Copyright 2017 - 2018 joydeepb@cs.umass.edu
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
#include "evaluators/offense_evaluation.h"
#include "logging/logger.h"
#include "math/geometry.h"
#include "math/math_util.h"
#include "src/constants/constants.h"
#include "third_party/googletest/googletest-release-1.8.0/googletest/include/gtest/gtest.h"

using Eigen::Vector2f;
using Eigen::Vector2d;
using geometry::Angle;
using geometry::Cross;
using geometry::GetTangentPoints;
using geometry::LineLineIntersection;
using geometry::Perp;
using offense::AimOption;
using offense::CalculateAimOptions;
using std::cout;
using std::endl;
using std::vector;

TEST(OffenseTest, CalculateAimOptions) {
  static const float kEpsilon = 1e-4;
  {
    const float robot_radius = 90;
    const float ball_radius = 21;
    const Vector2f source(0, 0);
    const Vector2f target_l(3000, 500);
    const Vector2f target_r(3000, -500);
    vector<AimOption> aim_options;
    vector<Vector2f> obstacles{Vector2f(2900, 200), Vector2f(2900, -200)};
    CalculateAimOptions(source, target_l, target_r, ball_radius,
     robot_radius,
                        obstacles, &aim_options);
    ASSERT_EQ(aim_options.size(), 3u);

    for (const auto& aim : aim_options) {
      ASSERT_FLOAT_EQ(0, aim.source.x());
      ASSERT_FLOAT_EQ(0, aim.source.y());
    }
    ASSERT_FLOAT_EQ(3000, aim_options[0].target_center.x());
    ASSERT_FLOAT_EQ(-400.55975, aim_options[0].target_center.y());
    ASSERT_FLOAT_EQ(0.051192924, aim_options[0].angle_width);

    ASSERT_FLOAT_EQ(3000, aim_options[1].target_center.x());
    ASSERT_FLOAT_EQ(0, aim_options[1].target_center.y());
    ASSERT_FLOAT_EQ(3000, aim_options[1].target_center.norm());
    ASSERT_FLOAT_EQ(0.0613241, aim_options[1].angle_width);

    ASSERT_FLOAT_EQ(3000, aim_options[2].target_center.x());
    ASSERT_FLOAT_EQ(400.55975, aim_options[2].target_center.y());
    ASSERT_FLOAT_EQ(0.051192924, aim_options[2].angle_width);
  }
  {
    const float robot_radius = 90;
    const float ball_radius = 20;
    const Vector2f source(2000, 100);
    const Vector2f target_l(3000, 500);
    const Vector2f target_r(3000, -500);
    vector<AimOption> aim_options;
    vector<Vector2f> obstacles{};
    CalculateAimOptions(source, target_l, target_r, ball_radius,
    robot_radius,
                        obstacles, &aim_options);
    ASSERT_EQ(aim_options.size(), 1u);

    for (const auto& aim : aim_options) {
      ASSERT_FLOAT_EQ(2000, aim.source.x());
      ASSERT_FLOAT_EQ(100, aim.source.y());
    }
    Vector2f left_post_tangent(0, 0);
    geometry::ProjectPointOntoLine(target_l, source, aim_options[0].target_l,
                                   &left_post_tangent);
    ASSERT_LE(fabs(ball_radius - (left_post_tangent - target_l).norm()),
              kEpsilon);

    Vector2f right_post_tangent(0, 0);
    geometry::ProjectPointOntoLine(target_r, source, aim_options[0].target_r,
                                   &right_post_tangent);
    ASSERT_LE(fabs(ball_radius - (right_post_tangent - target_r).norm()),
              kEpsilon);
    ASSERT_FLOAT_EQ(3000, aim_options[0].target_center.x());
    ASSERT_FLOAT_EQ(3000, aim_options[0].target_l.x());
    ASSERT_FLOAT_EQ(3000, aim_options[0].target_r.x());
    const float angle_l = Angle<float>(aim_options[0].target_l - source);
    const float angle_r = Angle<float>(aim_options[0].target_r - source);
    ASSERT_FLOAT_EQ(0.5 * (angle_l + angle_r), aim_options[0].angle_center);
  }
}

TEST(OffenseTest, NoOpenAngleTests) {
  {
    const Vector2f source(0, 0);
    vector<AimOption> aim_options;
    vector<Vector2f> obstacles;
    Vector2f current = kTheirGoalL;
    current.x() -= 100;
    while (current.y() >= kTheirGoalR.y()) {
      obstacles.push_back(current);
      current.y() -= kRobotRadius * 2;
    }
    CalculateAimOptions(source, kTheirGoalL, kTheirGoalR, kBallRadius,
                        kRobotRadius, obstacles, &aim_options);
    ASSERT_EQ(aim_options.empty(), true);
  }
  {
    const Vector2f source(0, kFieldYMax / 2);
    vector<AimOption> aim_options;
    vector<Vector2f> obstacles;
    Vector2f current = kTheirGoalL;
    current.x() -= 50;
    while (current.y() >= kTheirGoalR.y()) {
      obstacles.push_back(current);
      current.y() -= kRobotRadius * 2;
    }
    CalculateAimOptions(source, kTheirGoalL, kTheirGoalR, kBallRadius,
                        kRobotRadius, obstacles, &aim_options);
    ASSERT_EQ(aim_options.empty(), true);
  }
  {
    const Vector2f source(-50, 0);
    vector<AimOption> aim_options;
    vector<Vector2f> obstacles;
    Vector2f current = kTheirGoalL;
    current.x() = 500;
    current.y() = kFieldYMax;
    while (current.y() >= -kFieldYMax) {
      obstacles.push_back(current);
      current.y() -= kRobotRadius * 2;
    }
    CalculateAimOptions(source, kTheirGoalL, kTheirGoalR, kBallRadius,
                        kRobotRadius, obstacles, &aim_options);
    //     ASSERT_EQ(aim_options.size(), 0);
  }
  {
    const Vector2f source(-50, kFieldYMax / 2);
    vector<AimOption> aim_options;
    vector<Vector2f> obstacles{
        Vector2f(500, kFieldYMax / 2),
        Vector2f(500, kFieldYMax / 2 - kRobotRadius * 2),
        Vector2f(500, kFieldYMax / 2 - kRobotRadius * 4),
        Vector2f(500, kFieldYMax / 2 - kRobotRadius * 2)};
    Vector2f current = kTheirGoalL;
    current.x() = 500;
    current.y() = kFieldYMax;
    CalculateAimOptions(source, kTheirGoalL, kTheirGoalR, kBallRadius,
                        kRobotRadius, obstacles, &aim_options);
    unsigned int expected_size = 0;
    ASSERT_EQ(aim_options.size(), expected_size);
  }
  {
    const Vector2f source(-50, kFieldYMax / 2);
    vector<AimOption> aim_options;
    vector<Vector2f> obstacles;
    Vector2f current = kTheirGoalL;
    current.x() = 500;
    current.y() = kFieldYMax;
    while (current.y() >= -kFieldYMax) {
      obstacles.push_back(current);
      current.y() -= kRobotRadius * 2;
    }
    CalculateAimOptions(source, kTheirGoalL, kTheirGoalR, kBallRadius,
                        kRobotRadius, obstacles, &aim_options);
    unsigned int expected_size = 0;
    ASSERT_EQ(aim_options.size(), expected_size);
  }
  {
    const Vector2f source(-50, -kFieldYMax / 2);
    vector<AimOption> aim_options;
    vector<Vector2f> obstacles;
    Vector2f current = kTheirGoalL;
    current.x() = 500;
    current.y() = kFieldYMax;
    while (current.y() >= -kFieldYMax) {
      obstacles.push_back(current);
      current.y() -= kRobotRadius * 2;
    }
    CalculateAimOptions(source, kTheirGoalL, kTheirGoalR, kBallRadius,
                        kRobotRadius, obstacles, &aim_options);
    unsigned int expected_size = 0;
    ASSERT_EQ(aim_options.size(), expected_size);
  }
}

TEST(OffenseTest, ObstacleSideSpecialCase) {
  {
    const Vector2f source(0, 0);
    vector<AimOption> aim_options;
    vector<Vector2f> obstacles{Vector2f(70, 0 + kRobotRadius)};
    Vector2f current = kTheirGoalL;
    current.x() = 500;
    current.y() = kFieldYMax;
    CalculateAimOptions(source, kTheirGoalL, kTheirGoalR, kBallRadius,
                        kRobotRadius, obstacles, &aim_options);
    ASSERT_EQ(aim_options.empty(), true);
  }
}
