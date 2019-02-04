// Copyright 2016 - 2018 kvedder@umass.edu
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
#include <iostream>
#include <vector>

#include "constants/constants.h"
#include "graph/fastmultigraph.h"
#include "graph/graph_util.h"
#include "math/geometry.h"
#include "math/poses_2d.h"
#include "navigation/RRT.h"

#include "navigation/eight_grid.h"
#include "obstacles/ball_obstacle.h"
#include "obstacles/circle_obstacle.h"
#include "obstacles/obstacle.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/obstacle_type.h"
#include "obstacles/safety_margin.h"
#include "state/soccer_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "test/navigation/scenario_generator.h"
#include "third_party/googletest/googletest-release-1.8.0/googletest/include/gtest/gtest.h"
#include "util/timer.h"

using pose_2d::Pose2Df;
using Eigen::Vector2i;

namespace navigation {
namespace eight_grid {

TEST(EightGrid, FreeSpaceToGridVertex) {
  if (kUseEightGrid) {
    const obstacle::ObstacleFlag obstacles = obstacle::ObstacleFlag::GetBall();
    const obstacle::SafetyMargin safety_margin;
    const float distance_between_vertices = 50;
    EightGrid eg(obstacles, safety_margin, distance_between_vertices);
    EXPECT_EQ(Vector2i(0, 0), eg.FreeSpaceToGridVertex({0, 0}));

    EXPECT_EQ(Vector2i(0, 0), eg.FreeSpaceToGridVertex({24, 24}));
    EXPECT_EQ(Vector2i(0, 0), eg.FreeSpaceToGridVertex({-24, -24}));

    EXPECT_EQ(Vector2i(1, 1), eg.FreeSpaceToGridVertex({26, 26}));
    EXPECT_EQ(Vector2i(-1, -1), eg.FreeSpaceToGridVertex({-26, -26}));

    EXPECT_EQ(Vector2i(3, 1), eg.FreeSpaceToGridVertex({126, 26}));
    EXPECT_EQ(Vector2i(-3, -1), eg.FreeSpaceToGridVertex({-126, -26}));

    EXPECT_EQ(Vector2i(3, 7), eg.FreeSpaceToGridVertex({126, 326}));
    EXPECT_EQ(Vector2i(-3, -7), eg.FreeSpaceToGridVertex({-126, -326}));

    EXPECT_EQ(Vector2i(3, 6), eg.FreeSpaceToGridVertex({126, 324}));
    EXPECT_EQ(Vector2i(-3, -6), eg.FreeSpaceToGridVertex({-126, -324}));
  }
}

TEST(EightGrid, VertexToIndex) {
  if (kUseEightGrid) {
    const obstacle::ObstacleFlag obstacles = obstacle::ObstacleFlag::GetBall();
    const obstacle::SafetyMargin safety_margin;
    const float distance_between_vertices = 50;
    EightGrid eg(obstacles, safety_margin, distance_between_vertices);

    for (int y_shift = 0; y_shift < 20; ++y_shift) {
      for (int x_shift = 0; x_shift < 20; ++x_shift) {
        const Vector2i start_vector(
            static_cast<int>(-field_dimensions::kHalfFieldLength /
                             distance_between_vertices) +
                x_shift,
            static_cast<int>(-field_dimensions::kHalfFieldWidth /
                             distance_between_vertices) +
                y_shift);
        const int index = eg.VertexToIndex(start_vector);
        const int y = index / (static_cast<int>(field_dimensions::kFieldLength /
                                                distance_between_vertices));
        const int x = index % (static_cast<int>(field_dimensions::kFieldLength /
                                                distance_between_vertices));
        EXPECT_EQ(x, x_shift);
        EXPECT_EQ(y, y_shift);
        const int fx = x;
        const int fy = y;
        const Vector2i corner_vector(fx, fy);
        const Vector2i final_vector =
            corner_vector +
            Vector2i(
                -field_dimensions::kHalfFieldLength / distance_between_vertices,
                -field_dimensions::kHalfFieldWidth / distance_between_vertices);

        EXPECT_EQ(final_vector.x(), start_vector.x());
        EXPECT_EQ(final_vector.y(), start_vector.y());
      }
    }
  }
}

TEST(EightGrid, Wall50) {
  if (kUseEightGrid) {
    logger::Logger logger;
    auto scenerio = navigation::test::GenerateWall();
    EightGrid eg(scenerio.obstacle_flag, scenerio.margin, 50);

    static constexpr int kCount = 5;
    std::array<float, kCount> data;

    for (int i = 0; i < kCount; ++i) {
      const auto start_time = GetMonotonicTime();
      eg.Update(scenerio.obstacle_flag, scenerio.margin, scenerio.start,
                scenerio.goal, &logger);
      auto plan_result = eg.Plan(&logger);
      const auto end_time = GetMonotonicTime();
      data[i] = (end_time - start_time);
      if (i == 0) {
        graph::util::WritePathToProtoFile("EightGridPath.proto",
                                          plan_result.second);
      }
    }

    float average = 0;
    for (int i = 0; i < kCount; ++i) {
      average += data[i];
    }
    average /= static_cast<float>(kCount);

    LOG(INFO) << "Average time (millis): " << average * 1000;

    graph::util::WriteObstacleFlagToFile("WallObs.proto",
                                         scenerio.obstacle_flag);
  }
}

TEST(EightGrid, Wall100) {
  if (kUseEightGrid) {
    logger::Logger logger;
    auto scenerio = navigation::test::GenerateWall();
    EightGrid eg(scenerio.obstacle_flag, scenerio.margin, 100);

    static constexpr int kCount = 5;
    std::array<float, kCount> data;

    for (int i = 0; i < kCount; ++i) {
      const auto start_time = GetMonotonicTime();
      eg.Update(scenerio.obstacle_flag, scenerio.margin, scenerio.start,
                scenerio.goal, &logger);
      auto plan_result = eg.Plan(&logger);
      const auto end_time = GetMonotonicTime();
      data[i] = (end_time - start_time);
      if (i == 0) {
        graph::util::WritePathToProtoFile("EightGridPath.proto",
                                          plan_result.second);
      }
    }

    float average = 0;
    for (int i = 0; i < kCount; ++i) {
      average += data[i];
    }
    average /= static_cast<float>(kCount);

    LOG(INFO) << "Average time (millis): " << average * 1000;

    graph::util::WriteObstacleFlagToFile("WallObs.proto",
                                         scenerio.obstacle_flag);
  }
}

TEST(EightGrid, WallOld) {
  if (kUseEightGrid) {
    logger::Logger logger;
    auto scenerio = navigation::test::GenerateWall();
    EightGrid eg(scenerio.obstacle_flag, scenerio.margin, 100);

    const auto start_time = GetMonotonicTime();
    eg.Update(scenerio.obstacle_flag, scenerio.margin, scenerio.start,
              scenerio.goal, &logger);
    auto plan_result = eg.Plan(&logger);
    const auto end_time = GetMonotonicTime();
    LOG(INFO) << "Runtime: " << (end_time - start_time);

    graph::util::WritePathToProtoFile("EightGridPath.proto",
                                      plan_result.second);

    graph::util::WriteObstacleFlagToFile("WallObs.proto",
                                         scenerio.obstacle_flag);
  }
}

}  // namespace eight_grid
}  // namespace navigation
