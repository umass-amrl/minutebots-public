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
#include "navigation/repair/identity_eight_grid_repairer.h"
#include "navigation/repair/vector_astar_eight_grid_repairer.h"

static constexpr bool kDumpOutput = false;

TEST(EightGridRepairer, BasicCollideTest) {
  constexpr unsigned int kRobotCount = 2;

  // Test constructor compiles.
  const float distance_between_vertices = 200.0f;
  const size_t radius = 1;
  const size_t step_size = 1;
  ::navigation::repair::repairer::AStarEightGridRepairer<kRobotCount>
      eight_grid_repairer(distance_between_vertices, radius, step_size);

  // Test Update() compiles.
  const std::array<obstacle::ObstacleFlag, kRobotCount> obstacles_array =
      array_util::MakeArray<kRobotCount>(obstacle::ObstacleFlag::GetEmpty());
  const std::array<obstacle::SafetyMargin, kRobotCount> safety_margin_array =
      array_util::MakeArray<kRobotCount>(obstacle::SafetyMargin());

  eight_grid_repairer.Update(obstacles_array, safety_margin_array);

  // Test VerifyAndRepairPlans() compiles.
  const std::vector<Eigen::Vector2i> base_plan = {{4, 0},  {3, 0},  {2, 0},
                                                  {1, 0},  {0, 0},  {-1, 0},
                                                  {-2, 0}, {-3, 0}, {-4, 0}};
  std::array<std::vector<Eigen::Vector2i>, kRobotCount> individual_plans;
  for (size_t i = 0; i < kRobotCount; ++i) {
    individual_plans[i] = base_plan;
    for (auto& vertex : individual_plans[i]) {
      // Rotates the integer positions by casting to float, doing the
      // rotation,
      // and then casting back to int.
      vertex =
          (Eigen::Rotation2Df(math_util::DegToRad<float>(360.0f / kRobotCount) *
                              static_cast<float>(i)) *
           vertex.cast<float>())
              .cast<int>();
    }
  }

  std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                     navigation::eight_grid::EightGrid::GridHasher>
      empty_map;
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
  const auto start_verify_and_replan = GetMonotonicTime();
  const auto results = eight_grid_repairer.VerifyAndRepairPlans(
      individual_plans, static_invalid_vertices, dynamic_invalid_vertices);
  const auto end_verify_and_replan = GetMonotonicTime();

  for (const auto& result : results) {
    LOG(INFO) << "======";
    for (const auto& e : result) {
      std::cout << "path_waypoint_list {\n  x: "
                << e.x() * distance_between_vertices
                << "\n  y: " << e.y() * distance_between_vertices << "\n}\n";
    }
  }
  LOG(INFO) << "Time delta (ms): "
            << (end_verify_and_replan - start_verify_and_replan) * 1000;
}

TEST(EightGridRepairer, SeperateCollisionsTest) {
  constexpr unsigned int kRobotCount = 3;

  // Test constructor compiles.
  const float distance_between_vertices = 200.0f;
  const size_t radius = 1;
  const size_t step_size = 1;
  ::navigation::repair::repairer::AStarEightGridRepairer<kRobotCount>
      eight_grid_repairer(distance_between_vertices, radius, step_size);

  // Test Update() compiles.
  const std::array<obstacle::ObstacleFlag, kRobotCount> obstacles_array =
      array_util::MakeArray<kRobotCount>(obstacle::ObstacleFlag::GetEmpty());
  const std::array<obstacle::SafetyMargin, kRobotCount> safety_margin_array =
      array_util::MakeArray<kRobotCount>(obstacle::SafetyMargin());

  eight_grid_repairer.Update(obstacles_array, safety_margin_array);

  // Test VerifyAndRepairPlans() compiles.
  const std::vector<Eigen::Vector2i> path1 = {{0, -4}, {0, -3}, {0, -2},
                                              {0, -1}, {0, 0},  {0, 1},
                                              {0, 2},  {0, 3},  {0, 4}};

  const std::vector<Eigen::Vector2i> path2 = {
      {2, 0},  {1, 0},  {0, 0},   {-1, 0},  {-2, 0},  {-3, 0},   {-4, 0},
      {-5, 0}, {-6, 0}, {-7, -1}, {-8, -2}, {-9, -3}, {-10, -4}, {-11, -5}};

  const std::vector<Eigen::Vector2i> path3 = {
      {-14, 0}, {-13, 0}, {-12, 0}, {-11, 0}, {-10, 0}, {-9, 0}, {-8, 0},
      {-7, 0},  {-6, 0},  {-5, 1},  {-4, 2},  {-3, 3},  {-2, 4}};

  std::array<std::vector<Eigen::Vector2i>, kRobotCount> individual_plans = {
      {path1, path2, path3}};

  std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                     navigation::eight_grid::EightGrid::GridHasher>
      empty_map;
  empty_map.insert({{0, 0}, array_util::MakeArray<8>(false)});
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

  auto results = eight_grid_repairer.VerifyAndRepairPlans(
      individual_plans, static_invalid_vertices, dynamic_invalid_vertices);

  constexpr int kIter = 0;
  double time_delta = 0;

  for (size_t i = 0; static_cast<int>(i) < kIter; ++i) {
    const auto start_verify_and_replan = GetMonotonicTime();
    results = eight_grid_repairer.VerifyAndRepairPlans(
        individual_plans, static_invalid_vertices, dynamic_invalid_vertices);
    const auto end_verify_and_replan = GetMonotonicTime();
    time_delta += (end_verify_and_replan - start_verify_and_replan);
  }

  for (size_t i = 0; i < kRobotCount; ++i) {
    const auto& result = results[i];
    auto union_map = static_invalid_vertices[i];
    const auto& dynamic_map = dynamic_invalid_vertices[i];
    union_map.insert(dynamic_map.begin(), dynamic_map.end());
    std::vector<Eigen::Vector2f> float_result(result.size());
    for (size_t i = 0; i < result.size(); ++i) {
      float_result[i] = result[i].cast<float>() * distance_between_vertices;
    }

    LOG(INFO) << "Skipping dump: " << ((!kDumpOutput) ? "TRUE" : "FALSE");
    if (kDumpOutput) {
      graph::util::WritePathToProtoFile(
          "seperate_collision_test_path" + std::to_string(i) + ".proto",
          float_result);
      graph::util::WritePointObstacleToProtoFile(
          "seperate_collision_test_point_obstacles" + std::to_string(i) +
              ".proto",
          union_map, distance_between_vertices);
    }
  }
  LOG(INFO) << "Time delta (ms): "
            << (time_delta / static_cast<double>(kIter)) * 1000;
  LOG(INFO) << "Folder name: " << util::serialization::GetFolderName();
}

TEST(EightGridRepairer, FullCountTest) {
  constexpr unsigned int kRobotCount = 6;

  // Test constructor compiles.
  const float distance_between_vertices = 200.0f;
  const size_t radius = 1;
  const size_t step_size = 1;
  ::navigation::repair::repairer::AStarEightGridRepairer<kRobotCount>
      eight_grid_repairer(distance_between_vertices, radius, step_size);

  // Test Update() compiles.
  const std::array<obstacle::ObstacleFlag, kRobotCount> obstacles_array =
      array_util::MakeArray<kRobotCount>(obstacle::ObstacleFlag::GetEmpty());
  const std::array<obstacle::SafetyMargin, kRobotCount> safety_margin_array =
      array_util::MakeArray<kRobotCount>(obstacle::SafetyMargin());

  eight_grid_repairer.Update(obstacles_array, safety_margin_array);

  // Test VerifyAndRepairPlans() compiles.
  const std::vector<Eigen::Vector2i> path1 = {
      {0, -2}, {0, -1}, {0, 0}, {0, 1}, {0, 2}};

  const std::vector<Eigen::Vector2i> path2 = {
      {2, 0},  {1, 0},  {0, 0},   {-1, 0},  {-2, 0},  {-3, 0},  {-4, 0},
      {-5, 0}, {-6, 0}, {-6, -1}, {-6, -2}, {-6, -3}, {-6, -4}, {-6, -5}};

  const std::vector<Eigen::Vector2i> path3 = {
      {-14, 0}, {-13, 0}, {-12, 0}, {-11, 0}, {-10, 0}, {-9, 0}, {-8, 0},
      {-7, 0},  {-6, 0},  {-6, 1},  {-6, 2},  {-6, 3},  {-6, 4}, {-6, 5}};

  const std::vector<Eigen::Vector2i> path4 = {
      {-5, -2}, {-5, -1}, {-5, 0}, {-5, 1}, {-5, 2}};

  const std::vector<Eigen::Vector2i> path5 = {
      {-7, -2}, {-7, -1}, {-7, 0}, {-7, 1}, {-7, 2}};

  const std::vector<Eigen::Vector2i> path6 = {
      {-9, -2}, {-9, -1}, {-9, 0}, {-9, 1}, {-9, 2}};

  std::array<std::vector<Eigen::Vector2i>, kRobotCount> individual_plans = {
      {path1, path2, path3, path4, path5, path6}};

  std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                     navigation::eight_grid::EightGrid::GridHasher>
      empty_map;
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
  const auto start_verify_and_replan = GetMonotonicTime();
  const auto results = eight_grid_repairer.VerifyAndRepairPlans(
      individual_plans, static_invalid_vertices, dynamic_invalid_vertices);
  const auto end_verify_and_replan = GetMonotonicTime();
  size_t i = 0;
  for (const auto& result : results) {
    LOG(INFO) << "====== " << i++;
    for (const auto& e : result) {
      std::cout << "path_waypoint_list {\n  x: "
                << e.x() * distance_between_vertices
                << "\n  y: " << e.y() * distance_between_vertices << "\n}\n";
    }
  }
  LOG(INFO) << "Time delta (ms): "
            << (end_verify_and_replan - start_verify_and_replan) * 1000;
}

TEST(EightGridRepairer, SingleCollideTest) {
  constexpr unsigned int kRobotCount = 1;

  // Test constructor compiles.
  const float distance_between_vertices = 200.0f;
  const size_t radius = 1;
  const size_t step_size = 1;
  ::navigation::repair::repairer::AStarEightGridRepairer<kRobotCount>
      eight_grid_repairer(distance_between_vertices, radius, step_size);

  // Test Update() compiles.
  const std::array<obstacle::ObstacleFlag, kRobotCount> obstacles_array =
      array_util::MakeArray<kRobotCount>(obstacle::ObstacleFlag::GetEmpty());
  const std::array<obstacle::SafetyMargin, kRobotCount> safety_margin_array =
      array_util::MakeArray<kRobotCount>(obstacle::SafetyMargin());

  eight_grid_repairer.Update(obstacles_array, safety_margin_array);

  // Test VerifyAndRepairPlans() compiles.
  const std::vector<Eigen::Vector2i> path1 = {
      {0, -2}, {0, -1}, {0, 0}, {0, 1}, {0, 2}};

  std::array<std::vector<Eigen::Vector2i>, kRobotCount> individual_plans = {
      {path1}};

  std::unordered_map<Eigen::Vector2i, std::array<bool, 8>,
                     navigation::eight_grid::EightGrid::GridHasher>
      empty_map;
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
  const auto start_verify_and_replan = GetMonotonicTime();
  const auto results = eight_grid_repairer.VerifyAndRepairPlans(
      individual_plans, static_invalid_vertices, dynamic_invalid_vertices);
  const auto end_verify_and_replan = GetMonotonicTime();
  size_t i = 0;
  for (const auto& result : results) {
    LOG(INFO) << "====== " << i++;
    for (const auto& e : result) {
      std::cout << "path_waypoint_list {\n  x: "
                << e.x() * distance_between_vertices
                << "\n  y: " << e.y() * distance_between_vertices << "\n}\n";
    }
  }
  LOG(INFO) << "Time delta (ms): "
            << (end_verify_and_replan - start_verify_and_replan) * 1000;
}
