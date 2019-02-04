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
#include <utility>
#include <vector>

#include "constants/constants.h"
#include "graph/graph_util.h"
#include "third_party/googletest/googletest-release-1.8.0/googletest/include/gtest/gtest.h"
#include "util/array_util.h"
#include "util/serialization.h"

#include "navigation/repair/astar_eight_grid_repairer.h"
#include "navigation/repair/expanding_astar_eight_grid_repairer.h"
#include "navigation/repair/vector_astar_eight_grid_repairer.h"

TEST(EightGridRepairerDemo, PlusCollision) {
  constexpr unsigned int kRobotCount = 2;

  // Test constructor compiles.
  const float distance_between_vertices = 200.0f;
  const size_t radius = 1;
  const size_t step_size = 1;
  ::navigation::repair::repairer::ExpandingAStarEightGridRepairer<kRobotCount>
      eight_grid_repairer(distance_between_vertices, radius, step_size);

  // Test Update() compiles.
  const std::array<obstacle::ObstacleFlag, kRobotCount> obstacles_array =
      array_util::MakeArray<kRobotCount>(obstacle::ObstacleFlag::GetEmpty());
  const std::array<obstacle::SafetyMargin, kRobotCount> safety_margin_array =
      array_util::MakeArray<kRobotCount>(obstacle::SafetyMargin());

  eight_grid_repairer.Update(obstacles_array, safety_margin_array);

  // Test VerifyAndRepairPlans() compiles.
  const std::vector<Eigen::Vector2i> path1 = {{0, 4},  {0, 3},  {0, 2},
                                              {0, 1},  {0, 0},  {0, -1},
                                              {0, -2}, {0, -3}, {0, -4}};

  const std::vector<float> path1_timings = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

  const std::vector<Eigen::Vector2i> path2 = {{0, -4}, {0, -3}, {0, -2},
                                              {0, -1}, {0, 0},  {0, 1},
                                              {0, 2},  {0, 3},  {0, 4}};

  const std::vector<float> path2_timings = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

  const std::vector<Eigen::Vector2i> wall = {};

  const std::array<std::vector<Eigen::Vector2i>, kRobotCount> individual_plans =
      {{path1, path2}};

  std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                     navigation::eight_grid::EightGrid::GridHasher>
      empty_map;
  const std::array<bool, 8> collision_array = array_util::MakeArray<8>(false);
  for (const auto& e : wall) {
    empty_map.insert({e, collision_array});
  }
  const std::array<
      std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                         navigation::eight_grid::EightGrid::GridHasher>,
      kRobotCount>
      static_invalid_vertices = array_util::MakeArray<kRobotCount>(empty_map);
  const std::array<
      std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                         navigation::eight_grid::EightGrid::GridHasher>,
      kRobotCount>
      dynamic_invalid_vertices = array_util::MakeArray<kRobotCount>(empty_map);

  const auto results = eight_grid_repairer.VerifyAndRepairPlans(
      individual_plans, static_invalid_vertices, dynamic_invalid_vertices);

  for (size_t i = 0; i < kRobotCount; ++i) {
    const auto& result = results[i];
    auto union_map = static_invalid_vertices[i];
    const auto& dynamic_map = dynamic_invalid_vertices[i];
    union_map.insert(dynamic_map.begin(), dynamic_map.end());
    std::vector<Eigen::Vector2f> float_result(result.size());
    for (size_t i = 0; i < result.size(); ++i) {
      float_result[i] = result[i].cast<float>() * distance_between_vertices;
    }
    graph::util::WritePathToProtoFile(
        "seperate_collision_test_path" + std::to_string(i) + ".proto",
        float_result);
    graph::util::WritePointObstacleToProtoFile(
        "seperate_collision_test_point_obstacles" + std::to_string(i) +
            ".proto",
        union_map, distance_between_vertices);
  }
  LOG(INFO) << "Folder name: " << util::serialization::GetFolderName();
}

TEST(EightGridRepairerDemo, UCollision) {
  constexpr unsigned int kRobotCount = 2;

  // Test constructor compiles.
  const float distance_between_vertices = 200.0f;
  const size_t radius = 1;
  const size_t step_size = 1;
  ::navigation::repair::repairer::ExpandingAStarEightGridRepairer<kRobotCount>
      eight_grid_repairer(distance_between_vertices, radius, step_size);

  // Test Update() compiles.
  const std::array<obstacle::ObstacleFlag, kRobotCount> obstacles_array =
      array_util::MakeArray<kRobotCount>(obstacle::ObstacleFlag::GetEmpty());
  const std::array<obstacle::SafetyMargin, kRobotCount> safety_margin_array =
      array_util::MakeArray<kRobotCount>(obstacle::SafetyMargin());

  eight_grid_repairer.Update(obstacles_array, safety_margin_array);

  // Test VerifyAndRepairPlans() compiles.
  const std::vector<Eigen::Vector2i> path1 = {
      {1, 4},  {1, 3},  {1, 2},  {1, 1},  {1, 0}, {0, -1},
      {-1, 0}, {-1, 1}, {-1, 2}, {-1, 3}, {-1, 4}};

  const std::vector<float> path1_timings = {0,
                                            1,
                                            2,
                                            3,
                                            4,
                                            4 + kSqrtTwo,
                                            4 + 2 * kSqrtTwo,
                                            5 + 2 * kSqrtTwo,
                                            6 + 2 * kSqrtTwo,
                                            7 + 2 * kSqrtTwo,
                                            8 + 2 * kSqrtTwo};

  const std::vector<Eigen::Vector2i> path2 = {
      {-1, 4}, {-1, 3}, {-1, 2}, {-1, 1}, {-1, 0}, {0, -1},
      {1, 0},  {1, 1},  {1, 2},  {1, 3},  {1, 4}};

  const std::vector<Eigen::Vector2i> path2_possible_fix = {
      {-1, 4}, {-1, 3}, {-1, 2}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -2},
      {1, -1}, {1, 0},  {1, 1},  {1, 2},  {1, 3},  {1, 4}};

  const std::vector<float> path2_possible_fix_timings = {0,
                                                         1,
                                                         2,
                                                         3,
                                                         4,
                                                         5,
                                                         5 + kSqrtTwo,
                                                         5 + 2 * kSqrtTwo,
                                                         6 + 2 * kSqrtTwo,
                                                         7 + 2 * kSqrtTwo,
                                                         8 + 2 * kSqrtTwo,
                                                         9 + 2 * kSqrtTwo,
                                                         10 + 2 * kSqrtTwo};

  const std::vector<Eigen::Vector2i> wall = {
      {0, 10}, {0, 9},   {0, 8},   {0, 7},   {0, 6},   {0, 5},  {0, 4},
      {0, 3},  {0, 2},   {0, 1},   {0, 0},   {2, 10},  {2, 9},  {2, 8},
      {2, 7},  {2, 6},   {2, 5},   {2, 4},   {2, 3},   {2, 2},  {2, 1},
      {2, 0},  {2, -1},  {2, -2},  {2, -3},  {-2, 10}, {-2, 9}, {-2, 8},
      {-2, 7}, {-2, 6},  {-2, 5},  {-2, 4},  {-2, 3},  {-2, 2}, {-2, 1},
      {-2, 0}, {-2, -1}, {-2, -2}, {-2, -3}, {-1, -3}, {0, -3}, {1, -3}};

  const std::array<std::vector<Eigen::Vector2i>, kRobotCount> individual_plans =
      {{path1, path2}};

  const std::array<std::vector<Eigen::Vector2i>, kRobotCount>
      possible_fix_plans = {{path1, path2_possible_fix}};

  const std::array<std::vector<float>, kRobotCount> possible_fix_times = {
      {path1_timings, path2_possible_fix_timings}};

  for (size_t i = 0; i < kRobotCount; ++i) {
    ASSERT_EQ(possible_fix_plans[i].size(), possible_fix_times[i].size());
  }

  //   const auto collision_results =
  //       ::navigation::repair::collision_checks::CollisionEvents<kRobotCount>(
  //           possible_fix_plans, possible_fix_times);
  //   for (const auto& result : collision_results) {
  //     for (const auto& e : result.path_data_array) {
  //       if (!e.enabled) {
  //         LOG(INFO) << "Not enabled!";
  //         continue;
  //       }
  //       LOG(INFO) << "Lower: " << e.lower_index << ", Upper: " <<
  //       e.upper_index;
  //     }
  //   }
  //   ASSERT_EQ(collision_results.size(), 0u);

  std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                     navigation::eight_grid::EightGrid::GridHasher>
      empty_map;
  const std::array<bool, 8> collision_array = array_util::MakeArray<8>(false);
  for (const auto& e : wall) {
    empty_map.insert({e, collision_array});
  }
  const std::array<
      std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                         navigation::eight_grid::EightGrid::GridHasher>,
      kRobotCount>
      static_invalid_vertices = array_util::MakeArray<kRobotCount>(empty_map);
  const std::array<
      std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                         navigation::eight_grid::EightGrid::GridHasher>,
      kRobotCount>
      dynamic_invalid_vertices = array_util::MakeArray<kRobotCount>(empty_map);

  const auto results = eight_grid_repairer.VerifyAndRepairPlans(
      individual_plans, static_invalid_vertices, dynamic_invalid_vertices);

  for (size_t i = 0; i < kRobotCount; ++i) {
    const auto& result = results[i];
    auto union_map = static_invalid_vertices[i];
    const auto& dynamic_map = dynamic_invalid_vertices[i];
    union_map.insert(dynamic_map.begin(), dynamic_map.end());
    std::vector<Eigen::Vector2f> float_result(result.size());
    for (size_t i = 0; i < result.size(); ++i) {
      float_result[i] = result[i].cast<float>() * distance_between_vertices;
    }
    graph::util::WritePathToProtoFile(
        "seperate_collision_test_path" + std::to_string(i) + ".proto",
        float_result);
    graph::util::WritePointObstacleToProtoFile(
        "seperate_collision_test_point_obstacles" + std::to_string(i) +
            ".proto",
        union_map, distance_between_vertices);
  }
  LOG(INFO) << "Folder name: " << util::serialization::GetFolderName();
}
