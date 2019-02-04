// Copyright 2016-2017 kvedder@umass.edu
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
#include <eigen3/Eigen/Core>
#include <cmath>
#include <iostream>
#include <vector>

#include "third_party/googletest/googletest-release-1.8.0/googletest/include/gtest/gtest.h"
#include "obstacles/circle_obstacle.h"
#include "obstacles/obstacle.h"
#include "math/math_util.h"
#include "math/poses_2d.h"
#include "util/timer.h"


using std::cout;
using std::endl;
using std::vector;
using pose_2d::Pose2Dd;
using Eigen::Vector2d;
using Eigen::Vector2f;
using Eigen::Vector3f;
using math_util::AngleMod;
using math_util::Cube;
using math_util::Ramp;
using math_util::Sq;
using math_util::SolveQuadratic;
using math_util::SolveCubic;

TEST(MathTest, Sq) {
  ASSERT_EQ(0, Sq(0));
  ASSERT_EQ(4.0, Sq(2.0));
  ASSERT_EQ(9.0, Sq(3.0));
  ASSERT_EQ(16.0, Sq(-4.0));
}

TEST(MathTest, Cube) {
  ASSERT_EQ(0, Cube(0));
  ASSERT_EQ(8.0, Cube(2.0));
  ASSERT_EQ(27.0, Cube(3.0));
  ASSERT_EQ(-64.0, Cube(-4.0));
}

TEST(MathTest, AngleMod) {
  ASSERT_EQ(0, AngleMod(0));
  ASSERT_EQ(static_cast<double>(M_PI), AngleMod(M_PI));
  ASSERT_EQ(static_cast<double>(0.25 * M_PI), AngleMod(2.25 * M_PI));
  ASSERT_EQ(static_cast<double>(-M_PI + 1e-10),
            AngleMod(-M_PI + 1e-10));
  ASSERT_EQ(static_cast<double>(-M_PI + 0.01),
            AngleMod(-3.0 * M_PI + 0.01));
}

TEST(MathTest, ApplyPose) {
  Pose2Dd base_pose(M_PI, Vector2d(0, 0));
  Pose2Dd rotate_pose(M_PI, Vector2d(0, 0));

  // Proves that fmod() does not strip the sign.
  ASSERT_FLOAT_EQ(fmod(-3, 2), -1);

  // Check that the simple setting of the angle is not somehow wrong.
  ASSERT_FLOAT_EQ(base_pose.angle, M_PI);

  base_pose.ApplyPose(rotate_pose);

  ASSERT_FLOAT_EQ(base_pose.angle, 0.0);
}

TEST(MathTest, Ramp) {
  {
    const float x_min = 0;
    const float x_max = 1;
    const float y_min = 0;
    const float y_max = 10;
    ASSERT_EQ(5, Ramp<float>(0.5, x_min, x_max, y_min, y_max));
    ASSERT_EQ(2, Ramp<float>(0.2, x_min, x_max, y_min, y_max));
    ASSERT_EQ(4, Ramp<float>(0.4, x_min, x_max, y_min, y_max));
    ASSERT_EQ(0, Ramp<float>(-0.01, x_min, x_max, y_min, y_max));
    ASSERT_EQ(10, Ramp<float>(10.01, x_min, x_max, y_min, y_max));
  }
}

TEST(MathTest, SolveQuadratic) {
  {
    const float a = 1.0;
    const float b = -7.0;
    const float c = 10.0;
    float root1 = 0;
    float root2 = 0;
    const int num_roots = SolveQuadratic(a, b, c, &root1, &root2);
    ASSERT_EQ(2, num_roots);
    ASSERT_FLOAT_EQ(2.0, root1);
    ASSERT_FLOAT_EQ(5.0, root2);
  }
  {
    const float a = 1.0;
    const float b = -1.5;
    const float c = -22.0;
    float root1 = 0;
    float root2 = 0;
    const int num_roots = SolveQuadratic(a, b, c, &root1, &root2);
    ASSERT_EQ(2, num_roots);
    ASSERT_FLOAT_EQ(-4.0, root1);
    ASSERT_FLOAT_EQ(5.5, root2);
  }
  {
    const float a = -1.0;
    const float b = 1.5;
    const float c = 22.0;
    float root1 = 0;
    float root2 = 0;
    const int num_roots = SolveQuadratic(a, b, c, &root1, &root2);
    ASSERT_EQ(2, num_roots);
    ASSERT_FLOAT_EQ(-4.0, root1);
    ASSERT_FLOAT_EQ(5.5, root2);
  }
  {
    const float a = 1.0;
    const float b = 6.0;
    const float c = 9.0;
    float root1 = 0;
    float root2 = 0;
    const int num_roots = SolveQuadratic(a, b, c, &root1, &root2);
    ASSERT_EQ(1, num_roots);
    ASSERT_FLOAT_EQ(-3.0, root1);
  }
  {
    const float a = 1.0;
    const float b = 1.0;
    const float c = 100.0;
    float root1 = 0;
    float root2 = 0;
    const int num_roots = SolveQuadratic(a, b, c, &root1, &root2);
    ASSERT_EQ(0, num_roots);
  }
}

TEST(MathTest, SolveCubic) {
  {
    const float a = 1.0;
    const float b = -9.0;
    const float c = 27.0;
    const float d = -27.0;
    float root0 = 0;
    float root1 = 0;
    float root2 = 0;

    const int num_roots = SolveCubic<float>(a, b, c, d, &root0, &root1, &root2);

    ASSERT_EQ(1, num_roots);
    ASSERT_FLOAT_EQ(root2, 3.0);
  }
  {
    const float a = 1.0;
    const float b = 9.0;
    const float c = 27.0;
    const float d = 27.0;
    float root0 = 0;
    float root1 = 0;
    float root2 = 0;

    const int num_roots = SolveCubic<float>(a, b, c, d, &root0, &root1, &root2);

    ASSERT_EQ(1, num_roots);
    ASSERT_FLOAT_EQ(root2, -3.0);
  }
  {
    const float a = 1.0;
    const float b = -5.0;
    const float c = 8.0;
    const float d = -4.0;
    float root0 = 0;
    float root1 = 0;
    float root2 = 0;

    const int num_roots = SolveCubic<float>(a, b, c, d, &root0, &root1, &root2);

    ASSERT_EQ(2, num_roots);
    ASSERT_FLOAT_EQ(root1, 1.0);
    ASSERT_FLOAT_EQ(root2, 2.0);
  }
  {
    const float a = 1.0;
    const float b = 5.0;
    const float c = 8.0;
    const float d = 4.0;
    float root0 = 0;
    float root1 = 0;
    float root2 = 0;

    const int num_roots = SolveCubic<float>(a, b, c, d, &root0, &root1, &root2);

    ASSERT_EQ(2, num_roots);
    ASSERT_FLOAT_EQ(root1, -2.0);
    ASSERT_FLOAT_EQ(root2, -1.0);
  }
  {
    const float a = 1.0;
    const float b = 0.0;
    const float c = 0.0;
    const float d = -8.0;
    float root0 = 0;
    float root1 = 0;
    float root2 = 0;

    const int num_roots = SolveCubic<float>(a, b, c, d, &root0, &root1, &root2);

    ASSERT_EQ(1, num_roots);
    ASSERT_FLOAT_EQ(root2, 2.0);
  }
  {
    const float a = 2.0;
    const float b = 0.0;
    const float c = 0.0;
    const float d = 16.0;
    float root0 = 0;
    float root1 = 0;
    float root2 = 0;

    const int num_roots = SolveCubic<float>(a, b, c, d, &root0, &root1, &root2);

    ASSERT_EQ(1, num_roots);
    ASSERT_FLOAT_EQ(root2, -2.0);
  }
  {
    const float a = 2.0;
    const float b = 2.0;
    const float c = 2.0;
    const float d = 0.0;
    float root0 = 0;
    float root1 = 0;
    float root2 = 0;

    const int num_roots = SolveCubic<float>(a, b, c, d, &root0, &root1, &root2);

    ASSERT_EQ(1, num_roots);
    ASSERT_FLOAT_EQ(root2, 0.0);
  }
  {
    const float a = 1.0;
    const float b = -2.0;
    const float c = -1.0;
    const float d = 2.0;
    float root0 = 0;
    float root1 = 0;
    float root2 = 0;

    const int num_roots = SolveCubic<float>(a, b, c, d, &root0, &root1, &root2);

    ASSERT_EQ(3, num_roots);
    ASSERT_FLOAT_EQ(root0, -1.0);
    ASSERT_FLOAT_EQ(root1, 1.0);
    ASSERT_FLOAT_EQ(root2, 2.0);
  }
}
