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
#include <utility>
#include <vector>

#include "constants/constants.h"
#include "navigation/repair/multipath_collision_checks.h"
#include "third_party/googletest/googletest-release-1.8.0/googletest/include/gtest/gtest.h"

TEST(MultipathCollisionChecks, XCollideCheck) {
  const Eigen::Vector2i c1_start(0, 0);
  const Eigen::Vector2i c1_end(1, 1);
  const ::navigation::repair::repairer::Distance c1_start_time(0, 0);
  const ::navigation::repair::repairer::Distance c1_end_time(0, 1);
  const Eigen::Vector2i c2_start(1, 0);
  const Eigen::Vector2i c2_end(0, 1);
  const ::navigation::repair::repairer::Distance c2_start_time(0, 0);
  const ::navigation::repair::repairer::Distance c2_end_time(0, 1);
  EXPECT_TRUE(
      ::navigation::repair::collision_checks::PathCylinderSegmentCollides(
          c1_start, c1_end, c1_start_time, c1_end_time, c2_start, c2_end,
          c2_start_time, c2_end_time));
}

TEST(MultipathCollisionChecks, PathPathCollisionCheckNoCrossNoCollide) {
  const std::vector<Eigen::Vector2i> path1 = {{0, 0}, {0, 1}, {0, 2}};
  const std::vector<::navigation::repair::repairer::Distance> times1 = {
      {0, 0}, {1, 0}, {2, 0}};
  const std::vector<Eigen::Vector2i> path2 = {{1, 0}, {1, 1}, {1, 2}};
  const std::vector<::navigation::repair::repairer::Distance> times2 = {
      {0, 0}, {1, 0}, {2, 0}};

  const auto collisions =
      ::navigation::repair::collision_checks::PathPathCollisionIndices(
          1, path1, times1, 2, path2, times2);

  EXPECT_EQ(collisions.size(), 0ul);
}

TEST(MultipathCollisionChecks, PathPathCollisionCheckCrossNoCollide) {
  const std::vector<Eigen::Vector2i> path1 = {{0, 0}, {0, 1}, {0, 2}, {0, 3}};
  const std::vector<::navigation::repair::repairer::Distance> times1 = {
      {0, 0}, {1, 0}, {2, 0}, {3, 0}};
  const std::vector<Eigen::Vector2i> path2 = {{3, 0}, {2, 0}, {1, 0}, {0, 0}};
  const std::vector<::navigation::repair::repairer::Distance> times2 = {
      {0, 0}, {1, 0}, {2, 0}, {3, 0}};

  const auto collisions =
      ::navigation::repair::collision_checks::PathPathCollisionIndices(
          1, path1, times1, 2, path2, times2);

  ASSERT_EQ(collisions.size(), 0ul);
}

TEST(MultipathCollisionChecks, PathPathCollisionCheckSingleCollide) {
  // Paths crisscross at the center with an X.
  const std::vector<Eigen::Vector2i> path1 = {{0, 0}, {0, 1}, {1, 2}, {1, 3}};
  const std::vector<::navigation::repair::repairer::Distance> times1 = {
      {0, 0}, {1, 0}, {1, 1}, {2, 1}};
  const std::vector<Eigen::Vector2i> path2 = {{1, 0}, {1, 1}, {0, 2}, {0, 3}};
  const std::vector<::navigation::repair::repairer::Distance> times2 = {
      {0, 0}, {1, 0}, {1, 1}, {2, 1}};

  const auto collisions =
      ::navigation::repair::collision_checks::PathPathCollisionIndices(
          1, path1, times1, 2, path2, times2);
  ASSERT_EQ(collisions.size(), 1ul);
  const auto& single_collision = collisions[0];
  EXPECT_EQ(single_collision.path1_min, 1u);
  EXPECT_EQ(single_collision.path1_max, 2u);
  EXPECT_EQ(single_collision.path2_min, 1u);
  EXPECT_EQ(single_collision.path2_max, 2u);
}

TEST(MultipathCollisionChecks, PathPathHeadOnCollide) {
  const std::vector<Eigen::Vector2i> path1 = {
      {-2, 0}, {-1, 0}, {0, 0}, {1, 0}, {2, 0}};
  const std::vector<::navigation::repair::repairer::Distance> times1 = {
      {0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}};
  const std::vector<Eigen::Vector2i> path2 = {
      {2, 0}, {1, 0}, {0, 0}, {-1, 0}, {-2, 0}};
  const std::vector<::navigation::repair::repairer::Distance> times2 = {
      {0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}};

  const auto collisions =
      ::navigation::repair::collision_checks::PathPathCollisionIndices(
          1, path1, times1, 2, path2, times2);
  ASSERT_EQ(collisions.size(), 1ul);
  const auto& single_collision = collisions[0];
  EXPECT_EQ(single_collision.path1_min, 1u);
  EXPECT_EQ(single_collision.path1_max, 3u);
  EXPECT_EQ(single_collision.path2_min, 1u);
  EXPECT_EQ(single_collision.path2_max, 3u);
}

TEST(MultipathCollisionChecks, PathPathTBoneCollide) {
  const std::vector<Eigen::Vector2i> path1 = {
      {0, -2}, {0, -1}, {0, 0}, {0, 1}, {0, 2}};
  const std::vector<::navigation::repair::repairer::Distance> times1 = {
      {0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}};
  const std::vector<Eigen::Vector2i> path2 = {
      {2, 0}, {1, 0}, {0, 0}, {-1, 0}, {-2, 0}};
  const std::vector<::navigation::repair::repairer::Distance> times2 = {
      {0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}};

  const auto collisions =
      ::navigation::repair::collision_checks::PathPathCollisionIndices(
          1, path1, times1, 2, path2, times2);
  ASSERT_EQ(collisions.size(), 1ul);
  const auto& single_collision = collisions[0];
  EXPECT_EQ(single_collision.path1_min, 1u);
  EXPECT_EQ(single_collision.path1_max, 3u);
  EXPECT_EQ(single_collision.path2_min, 1u);
  EXPECT_EQ(single_collision.path2_max, 3u);
}

TEST(MultipathCollisionChecks, PathPathTBoneCollideEnd) {
  const std::vector<Eigen::Vector2i> path1 = {{2, 0},  {1, 0},  {0, 0},
                                              {-1, 0}, {-2, 0}, {-3, 0},
                                              {-4, 0}, {-5, 0}, {-6, 0}};
  const std::vector<::navigation::repair::repairer::Distance> times1 = {
      {0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}, {6, 0}, {7, 0}, {8, 0}};
  const std::vector<Eigen::Vector2i> path2 = {{-14, 0}, {-13, 0}, {-12, 0},
                                              {-11, 0}, {-10, 0}, {-9, 0},
                                              {-8, 0},  {-7, 0},  {-6, 0}};
  const std::vector<::navigation::repair::repairer::Distance> times2 = {
      {0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}, {6, 0}, {7, 0}, {8, 0}};

  const auto collisions =
      ::navigation::repair::collision_checks::PathPathCollisionIndices(
          1, path1, times1, 2, path2, times2);
  ASSERT_EQ(collisions.size(), 1ul);
  const auto& single_collision = collisions[0];
  EXPECT_EQ(single_collision.path1_min, 7u);
  EXPECT_EQ(single_collision.path1_max, 8u);
  EXPECT_EQ(single_collision.path2_min, 7u);
  EXPECT_EQ(single_collision.path2_max, 8u);
}

TEST(MultipathCollisionChecks, CollisionEventSeperation) {
  // Path 1 and 2 collide at {0, 0} at time 2; Path 2 and 3 collide at {-6, 0}
  // at time 8.
  constexpr unsigned int kRobotCount = 3;
  const std::vector<Eigen::Vector2i> path1 = {
      {0, -2}, {0, -1}, {0, 0}, {0, 1}, {0, 2}};
  const std::vector<::navigation::repair::repairer::Distance> times1 = {
      {0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}};
  const std::vector<Eigen::Vector2i> path2 = {{2, 0},  {1, 0},  {0, 0},
                                              {-1, 0}, {-2, 0}, {-3, 0},
                                              {-4, 0}, {-5, 0}, {-6, 0}};
  const std::vector<::navigation::repair::repairer::Distance> times2 = {
      {0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}, {6, 0}, {7, 0}, {8, 0}};
  const std::vector<Eigen::Vector2i> path3 = {{-14, 0}, {-13, 0}, {-12, 0},
                                              {-11, 0}, {-10, 0}, {-9, 0},
                                              {-8, 0},  {-7, 0},  {-6, 0}};
  const std::vector<::navigation::repair::repairer::Distance> times3 = {
      {0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}, {6, 0}, {7, 0}, {8, 0}};
  const std::array<std::vector<Eigen::Vector2i>, kRobotCount> paths = {
      {path1, path2, path3}};
  const std::array<std::vector<::navigation::repair::repairer::Distance>,
                   kRobotCount>
      times = {{times1, times2, times3}};

  const std::vector<
      ::navigation::repair::collision_checks::CollisionEvent<kRobotCount>>
      collision_events =
          ::navigation::repair::collision_checks::CollisionEvents<kRobotCount>(
              paths, times);
  ASSERT_EQ(collision_events.size(), 2lu);
}

TEST(MultipathCollisionChecks, PathPathCollisionCheckDualCollide) {
  // Paths crisscross twice, each with an X.
  const std::vector<Eigen::Vector2i> path1 = {
      {0, 0}, {0, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 5}, {1, 6}, {0, 7}, {0, 8}};
  const std::vector<::navigation::repair::repairer::Distance> times1 = {
      {0, 0}, {1, 0}, {1, 1}, {2, 1}, {3, 1}, {4, 1}, {5, 1}, {5, 2}, {6, 2}};
  const std::vector<Eigen::Vector2i> path2 = {
      {1, 0}, {1, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 5}, {0, 6}, {1, 7}, {1, 8}};
  const std::vector<::navigation::repair::repairer::Distance> times2 = {
      {0, 0}, {1, 0}, {1, 1}, {2, 1}, {3, 1}, {4, 1}, {5, 1}, {5, 2}, {6, 2}};

  const auto collisions =
      ::navigation::repair::collision_checks::PathPathCollisionIndices(
          1, path1, times1, 2, path2, times2);
  ASSERT_EQ(collisions.size(), 2ul);
  const auto& first_collision = collisions[0];
  EXPECT_EQ(first_collision.path1_min, 1u);
  EXPECT_EQ(first_collision.path1_max, 2u);
  EXPECT_EQ(first_collision.path2_min, 1u);
  EXPECT_EQ(first_collision.path2_max, 2u);

  const auto& second_collision = collisions[1];
  EXPECT_EQ(second_collision.path1_min, 6u);
  EXPECT_EQ(second_collision.path1_max, 7u);
  EXPECT_EQ(second_collision.path2_min, 6u);
  EXPECT_EQ(second_collision.path2_max, 7u);
}

TEST(MultipathCollisionChecks, PathPathCollisionCheckFourPath) {
  const std::vector<Eigen::Vector2i> path1 = {{4, 0},  {3, 0},  {2, 0},
                                              {1, 0},  {0, 0},  {-1, 0},
                                              {-2, 0}, {-3, 0}, {-4, 0}};
  const std::vector<::navigation::repair::repairer::Distance> times1 = {
      {0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}, {6, 0}, {7, 0}, {8, 0}};
  const std::vector<Eigen::Vector2i> path2 = {{-4, 0}, {-3, 0}, {-2, 0},
                                              {-1, 0}, {0, 0},  {1, 0},
                                              {2, 0},  {3, 0},  {4, 0}};
  const std::vector<::navigation::repair::repairer::Distance> times2 = {
      {0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}, {6, 0}, {7, 0}, {8, 0}};
  const std::vector<Eigen::Vector2i> path3 = {{0, -4}, {0, -3}, {0, -2},
                                              {0, -1}, {0, 0},  {0, 1},
                                              {0, 2},  {0, 3},  {0, 4}};
  const std::vector<::navigation::repair::repairer::Distance> times3 = {
      {0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}, {6, 0}, {7, 0}, {8, 0}};
  const std::vector<Eigen::Vector2i> path4 = {{0, 4},  {0, 3},  {0, 2},
                                              {0, 1},  {0, 0},  {0, -1},
                                              {0, -2}, {0, -3}, {0, -4}};
  const std::vector<::navigation::repair::repairer::Distance> times4 = {
      {0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}, {6, 0}, {7, 0}, {8, 0}};

  constexpr unsigned int kRobotCount = 4;

  const std::array<std::vector<Eigen::Vector2i>, kRobotCount> paths = {
      {path1, path2, path3, path4}};
  const std::array<std::vector<::navigation::repair::repairer::Distance>,
                   kRobotCount>
      times = {{times1, times2, times3, times4}};

  // ==========
  // P1
  // ==========

  const auto collisions_p1_p2 =
      ::navigation::repair::collision_checks::PathPathCollisionIndices(
          1, path1, times1, 2, path2, times2);
  ASSERT_EQ(collisions_p1_p2.size(), 1ul);
  const auto& first_collision_p1_p2 = collisions_p1_p2[0];
  EXPECT_EQ(first_collision_p1_p2.path1_min, 3u);
  EXPECT_EQ(first_collision_p1_p2.path1_max, 5u);
  EXPECT_EQ(first_collision_p1_p2.path2_min, 3u);
  EXPECT_EQ(first_collision_p1_p2.path2_max, 5u);

  const auto collisions_p1_p3 =
      ::navigation::repair::collision_checks::PathPathCollisionIndices(
          1, path1, times1, 3, path3, times3);
  ASSERT_EQ(collisions_p1_p3.size(), 1ul);
  const auto& first_collision_p1_p3 = collisions_p1_p3[0];
  EXPECT_EQ(first_collision_p1_p3.path1_min, 3u);
  EXPECT_EQ(first_collision_p1_p3.path1_max, 5u);
  EXPECT_EQ(first_collision_p1_p3.path2_min, 3u);
  EXPECT_EQ(first_collision_p1_p3.path2_max, 5u);

  const auto collisions_p1_p4 =
      ::navigation::repair::collision_checks::PathPathCollisionIndices(
          1, path1, times1, 4, path4, times4);
  ASSERT_EQ(collisions_p1_p4.size(), 1ul);
  const auto& first_collision_p1_p4 = collisions_p1_p4[0];
  EXPECT_EQ(first_collision_p1_p4.path1_min, 3u);
  EXPECT_EQ(first_collision_p1_p4.path1_max, 5u);
  EXPECT_EQ(first_collision_p1_p4.path2_min, 3u);
  EXPECT_EQ(first_collision_p1_p4.path2_max, 5u);

  // ==========
  // P2
  // ==========

  const auto collisions_p2_p1 =
      ::navigation::repair::collision_checks::PathPathCollisionIndices(
          2, path2, times2, 1, path1, times1);
  ASSERT_EQ(collisions_p2_p1.size(), 1ul);
  const auto& first_collision_p2_p1 = collisions_p2_p1[0];
  EXPECT_EQ(first_collision_p2_p1.path1_min, 3u);
  EXPECT_EQ(first_collision_p2_p1.path1_max, 5u);
  EXPECT_EQ(first_collision_p2_p1.path2_min, 3u);
  EXPECT_EQ(first_collision_p2_p1.path2_max, 5u);

  const auto collisions_p2_p3 =
      ::navigation::repair::collision_checks::PathPathCollisionIndices(
          2, path2, times2, 3, path3, times3);
  ASSERT_EQ(collisions_p2_p3.size(), 1ul);
  const auto& first_collision_p2_p3 = collisions_p2_p3[0];
  EXPECT_EQ(first_collision_p2_p3.path1_min, 3u);
  EXPECT_EQ(first_collision_p2_p3.path1_max, 5u);
  EXPECT_EQ(first_collision_p2_p3.path2_min, 3u);
  EXPECT_EQ(first_collision_p2_p3.path2_max, 5u);

  const auto collisions_p2_p4 =
      ::navigation::repair::collision_checks::PathPathCollisionIndices(
          2, path2, times2, 4, path4, times4);
  ASSERT_EQ(collisions_p2_p4.size(), 1ul);
  const auto& first_collision_p2_p4 = collisions_p2_p4[0];
  EXPECT_EQ(first_collision_p2_p4.path1_min, 3u);
  EXPECT_EQ(first_collision_p2_p4.path1_max, 5u);
  EXPECT_EQ(first_collision_p2_p4.path2_min, 3u);
  EXPECT_EQ(first_collision_p2_p4.path2_max, 5u);

  // ==========
  // P3
  // ==========

  const auto collisions_p3_p1 =
      ::navigation::repair::collision_checks::PathPathCollisionIndices(
          3, path3, times3, 1, path1, times1);
  ASSERT_EQ(collisions_p3_p1.size(), 1ul);
  const auto& first_collision_p3_p1 = collisions_p3_p1[0];
  EXPECT_EQ(first_collision_p3_p1.path1_min, 3u);
  EXPECT_EQ(first_collision_p3_p1.path1_max, 5u);
  EXPECT_EQ(first_collision_p3_p1.path2_min, 3u);
  EXPECT_EQ(first_collision_p3_p1.path2_max, 5u);

  const auto collisions_p3_p2 =
      ::navigation::repair::collision_checks::PathPathCollisionIndices(
          3, path3, times3, 2, path2, times2);
  ASSERT_EQ(collisions_p3_p2.size(), 1ul);
  const auto& first_collision_p3_p2 = collisions_p3_p2[0];
  EXPECT_EQ(first_collision_p3_p2.path1_min, 3u);
  EXPECT_EQ(first_collision_p3_p2.path1_max, 5u);
  EXPECT_EQ(first_collision_p3_p2.path2_min, 3u);
  EXPECT_EQ(first_collision_p3_p2.path2_max, 5u);

  const auto collisions_p3_p4 =
      ::navigation::repair::collision_checks::PathPathCollisionIndices(
          3, path3, times3, 4, path4, times4);
  ASSERT_EQ(collisions_p3_p4.size(), 1ul);
  const auto& first_collision_p3_p4 = collisions_p3_p4[0];
  EXPECT_EQ(first_collision_p3_p4.path1_min, 3u);
  EXPECT_EQ(first_collision_p3_p4.path1_max, 5u);
  EXPECT_EQ(first_collision_p3_p4.path2_min, 3u);
  EXPECT_EQ(first_collision_p3_p4.path2_max, 5u);

  // ==========
  // P4
  // ==========

  const auto collisions_p4_p1 =
      ::navigation::repair::collision_checks::PathPathCollisionIndices(
          4, path4, times4, 1, path1, times1);
  ASSERT_EQ(collisions_p4_p1.size(), 1ul);
  const auto& first_collision_p4_p1 = collisions_p4_p1[0];
  EXPECT_EQ(first_collision_p4_p1.path1_min, 3u);
  EXPECT_EQ(first_collision_p4_p1.path1_max, 5u);
  EXPECT_EQ(first_collision_p4_p1.path2_min, 3u);
  EXPECT_EQ(first_collision_p4_p1.path2_max, 5u);

  const auto collisions_p4_p2 =
      ::navigation::repair::collision_checks::PathPathCollisionIndices(
          4, path4, times4, 2, path2, times2);
  ASSERT_EQ(collisions_p4_p2.size(), 1ul);
  const auto& first_collision_p4_p2 = collisions_p4_p2[0];
  EXPECT_EQ(first_collision_p4_p2.path1_min, 3u);
  EXPECT_EQ(first_collision_p4_p2.path1_max, 5u);
  EXPECT_EQ(first_collision_p4_p2.path2_min, 3u);
  EXPECT_EQ(first_collision_p4_p2.path2_max, 5u);

  const auto collisions_p4_p3 =
      ::navigation::repair::collision_checks::PathPathCollisionIndices(
          4, path4, times4, 3, path3, times3);
  ASSERT_EQ(collisions_p4_p3.size(), 1ul);
  const auto& first_collision_p4_p3 = collisions_p4_p3[0];
  EXPECT_EQ(first_collision_p4_p3.path1_min, 3u);
  EXPECT_EQ(first_collision_p4_p3.path1_max, 5u);
  EXPECT_EQ(first_collision_p4_p3.path2_min, 3u);
  EXPECT_EQ(first_collision_p4_p3.path2_max, 5u);

  // ==========
  // Collision events
  // ==========

  const auto collision_events =
      ::navigation::repair::collision_checks::CollisionEvents<kRobotCount>(
          paths, times);
  ASSERT_EQ(collision_events.size(), 1ul);
  const auto& collision_event = collision_events[0];
  for (size_t i = 0; i < kRobotCount; ++i) {
    const auto& path_data = collision_event.path_data_array[i];
    EXPECT_TRUE(path_data.enabled) << "Path " << i
                                   << " should have registered a collision!";
    EXPECT_EQ(path_data.lower_index, 3u);
    EXPECT_EQ(path_data.upper_index, 5u);
  }
}
