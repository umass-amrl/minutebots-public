// Copyright 2018 slane@cs.umass.edu
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
#include "evaluators/defense_evaluation.h"
#include "logging/logger.h"
#include "math/geometry.h"
#include "math/math_util.h"
#include "src/constants/constants.h"
#include "third_party/googletest/googletest-release-1.8.0/googletest/include/gtest/gtest.h"

using defense::DefenseEvaluator;
using Eigen::Vector2f;
using Eigen::Vector2d;
using geometry::Angle;
using geometry::Cross;
using geometry::GetTangentPoints;
using geometry::LineLineIntersection;
using geometry::Perp;

using std::cout;
using std::endl;
using std::vector;

TEST(DefeseTest, PointOutsideBroadPhase) {
  {
    Vector2f point(-1500, -1500);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, true);
    ASSERT_FALSE(is_colliding);
  }
  {
    Vector2f point(-1500, -kFieldYMax);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, true);
    ASSERT_FALSE(is_colliding);
  }
  {
    Vector2f point(-1500, kFieldYMax);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, true);
    ASSERT_FALSE(is_colliding);
  }
  {
    Vector2f point(-kFieldXMax - 10, -1500);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, true);
    ASSERT_FALSE(is_colliding);
  }
  {
    Vector2f point(kFieldXMax, 2000);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, true);
    ASSERT_FALSE(is_colliding);
  }
}

TEST(DefeseTest, PointInsideLeftCircle) {
  {
    Vector2f point(-kFieldXMax + 100, -1000);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, true);
    ASSERT_TRUE(is_colliding);
  }
  {
    Vector2f point(-kFieldXMax + 100, -300);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, true);
    ASSERT_TRUE(is_colliding);
  }
  {
    Vector2f point(-kFieldXMax + 100, -400);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, true);
    ASSERT_TRUE(is_colliding);
  }
  {
    Vector2f point(-kFieldXMax + 100, -500);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, true);
    ASSERT_TRUE(is_colliding);
  }
  {
    Vector2f point(-kFieldXMax + 100, -600);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, true);
    ASSERT_TRUE(is_colliding);
  }
}

TEST(DefeseTest, PointInsideRightCircle) {
  {
    Vector2f point(-kFieldXMax + 100, 1000);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, true);
    ASSERT_TRUE(is_colliding);
  }
  {
    Vector2f point(-kFieldXMax + 100, 900);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, true);
    ASSERT_TRUE(is_colliding);
  }
  {
    Vector2f point(-kFieldXMax + 100, 800);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, true);
    ASSERT_TRUE(is_colliding);
  }
  {
    Vector2f point(-kFieldXMax + 100, 700);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, true);
    ASSERT_TRUE(is_colliding);
  }
  {
    Vector2f point(-kFieldXMax + 100, 600);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, true);
    ASSERT_TRUE(is_colliding);
  }
}

TEST(DefeseTest, PointInsideRectangle) {
  {
    Vector2f point(-kFieldXMax + 100, -250);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, true);
    ASSERT_TRUE(is_colliding);
  }
  {
    Vector2f point(-kFieldXMax + 100, 250);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, true);
    ASSERT_TRUE(is_colliding);
  }
  {
    Vector2f point(-kFieldXMax + 100, -100);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, true);
    ASSERT_TRUE(is_colliding);
  }
  {
    Vector2f point(-kFieldXMax + 100, 100);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, true);
    ASSERT_TRUE(is_colliding);
  }
  {
    Vector2f point(-kFieldXMax + 100, 0);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, true);
    ASSERT_TRUE(is_colliding);
  }
}

TEST(DefeseTest, PointOutsideBroadPhaseTheirs) {
  {
    Vector2f point(1500, 1500);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, false);
    ASSERT_FALSE(is_colliding);
  }
  {
    Vector2f point(1500, kFieldYMax);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, false);
    ASSERT_FALSE(is_colliding);
  }
  {
    Vector2f point(1500, -kFieldYMax);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, false);
    ASSERT_FALSE(is_colliding);
  }
  {
    Vector2f point(kFieldXMax + 10, 1500);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, false);
    ASSERT_FALSE(is_colliding);
  }
  {
    Vector2f point(-kFieldXMax, -2000);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, false);
    ASSERT_FALSE(is_colliding);
  }
}

TEST(DefeseTest, PointInsideLeftCircleTheirs) {
  {
    Vector2f point(kFieldXMax - 100, -1000);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, false);
    ASSERT_TRUE(is_colliding);
  }
  {
    Vector2f point(kFieldXMax - 100, -300);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, false);
    ASSERT_TRUE(is_colliding);
  }
  {
    Vector2f point(kFieldXMax - 100, -400);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, false);
    ASSERT_TRUE(is_colliding);
  }
  {
    Vector2f point(kFieldXMax - 100, -500);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, false);
    ASSERT_TRUE(is_colliding);
  }
  {
    Vector2f point(kFieldXMax - 100, -600);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, false);
    ASSERT_TRUE(is_colliding);
  }
}

TEST(DefeseTest, PointInsideRightCircleTheirs) {
  {
    Vector2f point(kFieldXMax - 100, 1000);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, false);
    ASSERT_TRUE(is_colliding);
  }
  {
    Vector2f point(kFieldXMax - 100, 900);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, false);
    ASSERT_TRUE(is_colliding);
  }
  {
    Vector2f point(kFieldXMax - 100, 800);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, false);
    ASSERT_TRUE(is_colliding);
  }
  {
    Vector2f point(kFieldXMax - 100, 700);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, false);
    ASSERT_TRUE(is_colliding);
  }
  {
    Vector2f point(kFieldXMax - 100, 600);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, false);
    ASSERT_TRUE(is_colliding);
  }
}

TEST(DefeseTest, PointInsideRectangleTheirs) {
  {
    Vector2f point(kFieldXMax - 100, -250);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, false);
    ASSERT_TRUE(is_colliding);
  }
  {
    Vector2f point(kFieldXMax - 100, 250);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, false);
    ASSERT_TRUE(is_colliding);
  }
  {
    Vector2f point(kFieldXMax - 100, -100);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, false);
    ASSERT_TRUE(is_colliding);
  }
  {
    Vector2f point(kFieldXMax - 100, 100);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, false);
    ASSERT_TRUE(is_colliding);
  }
  {
    Vector2f point(kFieldXMax - 100, 0);
    bool is_colliding = DefenseEvaluator::PointInDefenseArea(point, 0, false);
    ASSERT_TRUE(is_colliding);
  }
}

// TEST(DefenseTest, CalculateDistanceTests) {
//   {
//     Vector2f goal(0.0f, 0.0f);
//     Vector2f threat(1000.0f, 0.0f);
//     float ratio = 1.0f;
//
//     float distance = DefenseEvaluator::CalculateDistanceFromGoal(goal,
//                                                                  threat,
//                                                                  ratio);
//
//     ASSERT_FLOAT_EQ(500.0f, distance);
//   }
// }
