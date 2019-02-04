// Copyright 2017 kvedder@umass.edu
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
#include <fstream>
#include <iostream>
#include <vector>

#include "constants/constants.h"
#include "math/poses_2d.h"
#include "navigation/PRM_SSL.h"
#include "navigation/smoothing.h"
#include "obstacles/ball_obstacle.h"
#include "obstacles/circle_obstacle.h"
#include "obstacles/obstacle.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/obstacle_type.h"
#include "third_party/googletest/googletest-release-1.8.0/googletest/include/gtest/gtest.h"
#include "util/timer.h"

STANDARD_USINGS;
using obstacle::ObstacleFlag;
using obstacle::SafetyMargin;
using std::ofstream;

namespace smoothing {

TEST(SmoothingTests, IntegrationTest) {
  vector<Vector2f> basic_path;
  //   for (size_t i = 0; i < 10; ++i) {
  //     basic_path.push_back(Vector2f(i, 0));
  //   }
  //
  //   for (size_t i = 0; i < 10; ++i) {
  //     basic_path.push_back(Vector2f(i + 9, 5));
  //   }
  basic_path.push_back(Vector2f(0, 0));
  basic_path.push_back(Vector2f(1, 0));
  basic_path.push_back(Vector2f(1, 1));
  basic_path.push_back(Vector2f(2, 1));
  basic_path.push_back(Vector2f(1.5, 0));
  basic_path.push_back(Vector2f(0.5, 1));
  basic_path.push_back(Vector2f(2, 0));
  basic_path.push_back(Vector2f(0, 0));

  HypocycloidSmoothing smoothing;
  const vector<Vector2f> smoothed_plan = smoothing.SmoothPlan(basic_path);
  ofstream out_file;
  out_file.open("BASIC_PATH.path");
  for (const Vector2f& plan_position : smoothed_plan) {
//     LOG(INFO) << plan_position.x() << ", " << plan_position.y() << "\n";
    out_file << plan_position.x() << " " << plan_position.y() << "\n";
  }
  out_file.close();
}

// Tests to make sure that the diffusal of the points terminate at the midpoint
// of the segment, and project the virtual appoint appropriately.
TEST(SmoothingTests, DiffusePointsTest) {
  WorldState world_state;
  HypocycloidSmoothing smoothing;
  const auto& points =
      smoothing.DiffusePoints(Vector2f(0, 0), Vector2f(-1, 0), Vector2f(0, 1),
                              ObstacleFlag::GetAll(world_state));

//   LOG(INFO) << "Prior Point: " << points.prior_point.x() << ", "
//             << points.prior_point.y() << "\n";
//
//   LOG(INFO) << "After Point: " << points.after_point.x() << ", "
//             << points.after_point.y() << "\n";
//
//   LOG(INFO) << "Virtual Point: " << points.virtual_point.x() << ", "
//             << points.virtual_point.y() << "\n";

  ASSERT_LE((points.prior_point - Vector2f(-smoothing.GetTraversalAmount(), 0))
                .norm(),
            0.01f);
  ASSERT_LE(
      (points.after_point - Vector2f(0, smoothing.GetTraversalAmount())).norm(),
      0.01f);
  ASSERT_LE(
      (points.virtual_point - Vector2f(smoothing.GetTraversalAmount(), 0.0))
          .norm(),
      0.01f);
}

// Tests to make sure that the calculation of the diffusal points is correct.
TEST(SmoothingTests, CuspTest) {
  HypocycloidSmoothing smoothing;
  SafetyMargin default_margin;
  const Vector2f left(-1, 0);
  const Vector2f up(0, 1);
  const Vector2f right(1, 0);

  const HypocycloidSmoothing::DiffusionPoints points{left, up, right};
  const float angle = smoothing.GetDiffusionPointAngle(points, Vector2f(0, 0));

  ASSERT_LE(fabs(angle - M_PI / 2), 0.001);

  const float num_cusps = smoothing.GetNumCusps(angle);

  ASSERT_LE(fabs(num_cusps - 4), 0.001);
}

}  // namespace smoothing
