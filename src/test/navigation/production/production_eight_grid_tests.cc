// Copyright 2018 kvedder@umass.edu
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

#include "graph/graph_util.h"
#include "test/navigation/scenario_generator.h"
#include "third_party/googletest/googletest-release-1.8.0/googletest/include/gtest/gtest.h"
#include "util/timer.h"

#include "navigation/production/production_eight_grid.h"

namespace navigation {
namespace production {
namespace eight_grid {

constexpr size_t kPerformanceIter = 100;
constexpr bool kRunPerfromanceTests = true;

TEST(ProductionEightGrid, ObstacleFreeTest) {
  if (!kRunPerfromanceTests) return;
  CollisionGrid cg_static(kEightGridSquareSize);
  CollisionGrid cg_dynamic(kEightGridSquareSize);
  cg_static.RebuildStatic(obstacle::SafetyMargin());
  cg_static.RebuildDynamic(obstacle::SafetyMargin());
  ProductionEightGrid peg(cg_static, cg_dynamic, kEightGridSquareSize);
  logger::Logger logger;
  peg.Update(obstacle::ObstacleFlag::GetEmpty(), obstacle::SafetyMargin(),
             {0, -1000}, {1000, 1000}, &logger);
  const auto result = peg.Plan(&logger);
  EXPECT_TRUE(result.first);
  for (const auto& e : result.second) {
    LOG(INFO) << e.x() << ", " << e.y();
  }
}

TEST(ProductionEightGrid, PerformanceTest) {
  if (!kRunPerfromanceTests) return;
  CollisionGrid cg_static(kEightGridSquareSize);
  CollisionGrid cg_dynamic(kEightGridSquareSize);
  cg_static.RebuildStatic(obstacle::SafetyMargin());
  cg_static.RebuildDynamic(obstacle::SafetyMargin());
  ProductionEightGrid peg(cg_static, cg_dynamic, kEightGridSquareSize);
  logger::Logger logger;
  double time = 0;
  for (size_t i = 0; i < kPerformanceIter; ++i) {
    const auto start = GetMonotonicTime();
    peg.Update(obstacle::ObstacleFlag::GetEmpty(), obstacle::SafetyMargin(),
               {-4000, -3000}, {4000, 3000}, &logger);
    const auto result = peg.Plan(&logger);
    const auto end = GetMonotonicTime();
    time += (end - start);
    EXPECT_TRUE(result.first);
  }
  time /= static_cast<float>(kPerformanceIter);
  LOG(INFO) << "Average time (ms): " << (time * 1000.0f);
}

TEST(ProductionEightGrid, Wall50Performance) {
  if (!kRunPerfromanceTests) return;
  logger::Logger logger;
  auto scenerio = navigation::test::GenerateWall();
  CollisionGrid cg_static(50);
  CollisionGrid cg_dynamic(50);
  cg_static.RebuildStatic(obstacle::SafetyMargin());
  cg_static.RebuildDynamic(obstacle::SafetyMargin());
  ProductionEightGrid eg(cg_static, cg_dynamic, 50);

  std::array<float, kPerformanceIter> data;

  for (size_t i = 0; i < kPerformanceIter; ++i) {
    const auto start_time = GetMonotonicTime();
    eg.Update(scenerio.obstacle_flag, scenerio.margin, scenerio.start,
              scenerio.goal, &logger);
    auto plan_result = eg.Plan(&logger);
    EXPECT_TRUE(plan_result.first);
    const auto end_time = GetMonotonicTime();
    data[i] = (end_time - start_time);
  }

  float average = 0;
  for (size_t i = 0; i < kPerformanceIter; ++i) {
    average += data[i];
  }
  average /= static_cast<float>(kPerformanceIter);

  LOG(INFO) << "Average time (ms): " << average * 1000;
}

TEST(ProductionEightGrid, Wall50) {
  logger::Logger logger;
  auto scenerio = navigation::test::GenerateWall();
  CollisionGrid cg_static(50);
  CollisionGrid cg_dynamic(50);
  cg_static.RebuildStatic(obstacle::SafetyMargin());
  cg_static.RebuildDynamic(obstacle::SafetyMargin());
  ProductionEightGrid eg(cg_static, cg_dynamic, 50);

  static constexpr int kCount = 5;
  std::array<float, kCount> data;

  for (int i = 0; i < kCount; ++i) {
    const auto start_time = GetMonotonicTime();
    eg.Update(scenerio.obstacle_flag, scenerio.margin, scenerio.start,
              scenerio.goal, &logger);
    auto plan_result = eg.Plan(&logger);
    EXPECT_TRUE(plan_result.first);
    const auto end_time = GetMonotonicTime();
    data[i] = (end_time - start_time);
    if (i == 0) {
      graph::util::WritePathToProtoFile("Wall50Path.proto", plan_result.second);
    }
  }

  float average = 0;
  for (int i = 0; i < kCount; ++i) {
    average += data[i];
  }
  average /= static_cast<float>(kCount);

  LOG(INFO) << "Average time (millis): " << average * 1000;

  graph::util::WriteObstacleFlagToFile("Wall50Obs.proto",
                                       scenerio.obstacle_flag);
}

TEST(ProductionEightGrid, Wall100) {
  logger::Logger logger;
  auto scenerio = navigation::test::GenerateWall();
  CollisionGrid cg_static(100);
  CollisionGrid cg_dynamic(100);
  cg_static.RebuildStatic(obstacle::SafetyMargin());
  cg_static.RebuildDynamic(obstacle::SafetyMargin());
  ProductionEightGrid eg(cg_static, cg_dynamic, 100);

  static constexpr int kCount = 5;
  std::array<float, kCount> data;

  for (int i = 0; i < kCount; ++i) {
    const auto start_time = GetMonotonicTime();
    eg.Update(scenerio.obstacle_flag, scenerio.margin, scenerio.start,
              scenerio.goal, &logger);
    auto plan_result = eg.Plan(&logger);
    EXPECT_TRUE(plan_result.first);
    const auto end_time = GetMonotonicTime();
    data[i] = (end_time - start_time);
    if (i == 0) {
      graph::util::WritePathToProtoFile("Wall100Path.proto",
                                        plan_result.second);
    }
  }

  float average = 0;
  for (int i = 0; i < kCount; ++i) {
    average += data[i];
  }
  average /= static_cast<float>(kCount);

  LOG(INFO) << "Average time (millis): " << average * 1000;

  graph::util::WriteObstacleFlagToFile("Wall100Obs.proto",
                                       scenerio.obstacle_flag);
}

TEST(ProductionEightGrid, Narrowing100) {
  logger::Logger logger;
  auto scenerio = navigation::test::GenerateNarrowing();
  CollisionGrid cg_static(100);
  CollisionGrid cg_dynamic(100);
  cg_static.RebuildStatic(obstacle::SafetyMargin());
  cg_static.RebuildDynamic(obstacle::SafetyMargin());
  ProductionEightGrid eg(cg_static, cg_dynamic, 100);

  static constexpr int kCount = 5;
  std::array<float, kCount> data;

  for (int i = 0; i < kCount; ++i) {
    const auto start_time = GetMonotonicTime();
    eg.Update(scenerio.obstacle_flag, scenerio.margin, scenerio.start,
              scenerio.goal, &logger);
    auto plan_result = eg.Plan(&logger);
    EXPECT_TRUE(plan_result.first);
    const auto end_time = GetMonotonicTime();
    data[i] = (end_time - start_time);
    if (i == 0) {
      graph::util::WritePathToProtoFile("Narrowing100Path.proto",
                                        plan_result.second);
    }
  }

  float average = 0;
  for (int i = 0; i < kCount; ++i) {
    average += data[i];
  }
  average /= static_cast<float>(kCount);

  LOG(INFO) << "Average time (millis): " << average * 1000;

  graph::util::WriteObstacleFlagToFile("Narrowing100Obs.proto",
                                       scenerio.obstacle_flag);
}

// TEST(ProductionEightGrid, RandomGap100) {
//   logger::Logger logger;
//     auto scenerio = navigation::test::GenerateRandomGap();
//     ProductionEightGrid eg(100);
//
//     static constexpr int kCount = 50;
//     std::array<float, kCount> data;
//
//     for (int i = 0; i < kCount; ++i) {
//       const auto start_time = GetMonotonicTime();
//       eg.Update(scenerio.obstacle_flag, scenerio.margin, scenerio.start,
//                 scenerio.goal, &logger);
//       auto plan_result = eg.Plan(&logger);
//       EXPECT_TRUE(plan_result.first);
//       const auto end_time = GetMonotonicTime();
//       data[i] = (end_time - start_time);
//       if (i == 0) {
//         graph::util::WritePathToProtoFile("RGap100Path.proto",
//                                           plan_result.second);
//       }
//     }
//
//     float average = 0;
//     for (int i = 0; i < kCount; ++i) {
//       average += data[i];
//     }
//     average /= static_cast<float>(kCount);
//
//     LOG(INFO) << "Average time (millis): " << average * 1000;
//
//     graph::util::WriteObstacleFlagToFile("RGap100Obs.proto",
//                                          scenerio.obstacle_flag);
// }

}  // namespace eight_grid
}  // namespace production
}  // namespace navigation
