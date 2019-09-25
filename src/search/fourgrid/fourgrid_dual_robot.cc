// Copyright 2018 kvedder@umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
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

#include <vector>

#include "constants/constants.h"
#include "search/fourgrid/dstar_fourgrid_solver.h"
#include "search/fourgrid/fourgrid.h"
#include "search/fourgrid/fourgrid_scenerio.h"
#include "search/fourgrid/fourgrid_solver.h"

static constexpr size_t kRobotCount = 2;
#define T 1
#define U 2
#define SQUARE 3
#define GRID 4

void Setup(char* name) {
  google::InitGoogleLogging(name);
  FLAGS_stderrthreshold = 0;
  FLAGS_colorlogtostderr = 1;
  FLAGS_logtostderr = true;
}

#define CONFIG U

::search::fourgrid::demo::FourgridScenerio<kRobotCount> SquareScenerio() {
  const ::search::fourgrid::FourGrid<kRobotCount> fourgrid_small(
      "fourgrid_2_robots_square_small_collide.txt");
  const ::search::fourgrid::FourGrid<kRobotCount> fourgrid_large(
      "fourgrid_2_robots_square_large_collide.txt");
  const std::vector<::search::fourgrid::FourGrid<kRobotCount>> four_grids = {
      fourgrid_small, fourgrid_large};
  const ::search::fourgrid::FourGrid<kRobotCount> fourgrid_no_collide(
      "fourgrid_2_robots_square_large_no_collide.txt");
  const std::vector<::search::fourgrid::JointPosition<kRobotCount>> starts = {
      {{{0, 0}, {1, 1}}}, {{{0, 0}, {2, 1}}}};
  const std::vector<::search::fourgrid::JointPosition<kRobotCount>> goals = {
      {{{1, 1}, {0, 0}}}, {{{2, 1}, {0, 0}}}};
  const ::search::fourgrid::CostPath<kRobotCount> first_path = {};
  const ::search::fourgrid::CostPath<kRobotCount> second_path = {
      {{{{0, 0}, {2, 1}}}, 0.0f}, {{{{0, 0}, {1, 1}}}, 1.0f}};
  const std::vector<::search::fourgrid::CostPath<kRobotCount>>
      start_connection_paths = {first_path, second_path};
  return {four_grids, start_connection_paths, starts, goals,
          fourgrid_no_collide};
}

::search::fourgrid::demo::FourgridScenerio<kRobotCount> TScenerio() {
  const ::search::fourgrid::FourGrid<kRobotCount> fourgrid_small(
      "fourgrid_2_robots_t_small_collide.txt");
  const ::search::fourgrid::FourGrid<kRobotCount> fourgrid_large(
      "fourgrid_2_robots_t_large_collide.txt");
  const std::vector<::search::fourgrid::FourGrid<kRobotCount>> four_grids = {
      fourgrid_small, fourgrid_large};
  const ::search::fourgrid::FourGrid<kRobotCount> fourgrid_no_collide(
      "fourgrid_2_robots_t_large_no_collide.txt");
  const std::vector<::search::fourgrid::JointPosition<kRobotCount>> starts = {
      {{{0, 0}, {2, 0}}}, {{{-1, 0}, {3, 0}}}};
  const std::vector<::search::fourgrid::JointPosition<kRobotCount>> goals = {
      {{{2, 0}, {0, 0}}}, {{{3, 0}, {-1, 0}}}};
  const ::search::fourgrid::CostPath<kRobotCount> first_path = {};
  const ::search::fourgrid::CostPath<kRobotCount> second_path = {
      {{{{-1, 0}, {3, 0}}}, 0.0f}, {{{{0, 0}, {2, 0}}}, 2.0f}};
  const std::vector<::search::fourgrid::CostPath<kRobotCount>>
      start_connection_paths = {first_path, second_path};
  return {four_grids, start_connection_paths, starts, goals,
          fourgrid_no_collide};
}

::search::fourgrid::demo::FourgridScenerio<kRobotCount> UScenerio() {
  const ::search::fourgrid::FourGrid<kRobotCount> fourgrid_small(
      "fourgrid_2_robots_u_small_collide.txt");
  const ::search::fourgrid::FourGrid<kRobotCount> fourgrid_large(
      "fourgrid_2_robots_u_large_collide.txt");
  const std::vector<::search::fourgrid::FourGrid<kRobotCount>> four_grids = {
      fourgrid_small, fourgrid_large};
  const ::search::fourgrid::FourGrid<kRobotCount> fourgrid_no_collide(
      "fourgrid_2_robots_u_large_no_collide.txt");
  const std::vector<::search::fourgrid::JointPosition<kRobotCount>> starts = {
      {{{0, 1}, {1, 1}}}, {{{0, 2}, {1, 2}}}};
  const std::vector<::search::fourgrid::JointPosition<kRobotCount>> goals = {
      {{{1, 1}, {0, 1}}}, {{{1, 2}, {0, 2}}}};
  const ::search::fourgrid::CostPath<kRobotCount> first_path = {};
  const ::search::fourgrid::CostPath<kRobotCount> second_path = {
      {{{{0, 2}, {1, 2}}}, 0.0f}, {{{{0, 1}, {1, 1}}}, 2.0f}};
  const std::vector<::search::fourgrid::CostPath<kRobotCount>>
      start_connection_paths = {first_path, second_path};
  return {four_grids, start_connection_paths, starts, goals,
          fourgrid_no_collide};
}

::search::fourgrid::demo::FourgridScenerio<kRobotCount> GridScenerio() {
  const ::search::fourgrid::FourGrid<kRobotCount> fourgrid_small(
      "fourgrid_2_robots_grid_small_collide.txt");
  const ::search::fourgrid::FourGrid<kRobotCount> fourgrid_large(
      "fourgrid_2_robots_grid_large_collide.txt");
  const std::vector<::search::fourgrid::FourGrid<kRobotCount>> four_grids = {
      fourgrid_small, fourgrid_large};
  const ::search::fourgrid::FourGrid<kRobotCount> fourgrid_no_collide(
      "fourgrid_2_robots_grid_large_no_collide.txt");
  const std::vector<::search::fourgrid::JointPosition<kRobotCount>> starts = {
      {{{-3, -3}, {3, 3}}}, {{{-4, -4}, {4, 4}}}};
  const std::vector<::search::fourgrid::JointPosition<kRobotCount>> goals = {
      {{{3, 3}, {-3, -3}}}, {{{4, 4}, {-4, -4}}}};
  const ::search::fourgrid::CostPath<kRobotCount> first_path = {};
  const ::search::fourgrid::CostPath<kRobotCount> second_path = {
      {{{{-4, -4}, {4, 4}}}, 0.0f},
      {{{{-4, -3}, {4, 3}}}, 2.0f},
      {{{{-3, -3}, {3, 3}}}, 4.0f}};
  const std::vector<::search::fourgrid::CostPath<kRobotCount>>
      start_connection_paths = {first_path, second_path};
  return {four_grids, start_connection_paths, starts, goals,
          fourgrid_no_collide};
}

int main(int argc, char** argv) {
  Setup(argv[0]);
#if CONFIG == SQUARE
  const auto scenerio = SquareScenerio();
#endif
#if CONFIG == T
  const auto scenerio = TScenerio();
#endif
#if CONFIG == U
  const auto scenerio = UScenerio();
#endif
#if CONFIG == GRID
  const auto scenerio = GridScenerio();
#endif
  scenerio.Verify();

  ::search::fourgrid::solver::expandingastar::FourGridSolver<kRobotCount>
      ea_solver;
  ::search::fourgrid::solver::dstar::DStarFourGridSolver<kRobotCount>
      dstar_solver;
  const auto astar_result = ea_solver.SolveAStar(
      scenerio.four_grids, scenerio.starts, scenerio.goals);
  LOG(INFO) << "A* Result:";
  for (const auto& joint_position : astar_result) {
    LOG(INFO) << search::fourgrid::util::JointPositionToString(
                     joint_position.first)
              << " cost: " << joint_position.second;
  }

  const auto eastar_result = ea_solver.SolveExpandingAStar(
      scenerio.four_grids, scenerio.starts, scenerio.goals,
      scenerio.start_connection_paths);
  LOG(INFO) << "EA* Result:";
  for (const auto& joint_position : eastar_result) {
    LOG(INFO) << search::fourgrid::util::JointPositionToString(
                     joint_position.first)
              << " cost: " << joint_position.second;
  }

  const auto final_start = scenerio.starts[scenerio.starts.size() - 1];
  const auto final_goal = scenerio.goals[scenerio.goals.size() - 1];
  const auto one_shot_dstar_result = dstar_solver.SolveDStarOneShot(
      final_start, final_goal, scenerio.fourgrid_individual,
      scenerio.GetLargest());
  LOG(INFO) << "One Shot D* Result:";
  for (const auto& joint_position : one_shot_dstar_result) {
    LOG(INFO) << search::fourgrid::util::JointPositionToString(
                     joint_position.first)
              << " cost: " << joint_position.second;
  }

  const auto iterative_dstar_result = dstar_solver.SolveDStarIterative(
      final_start, final_goal, scenerio.fourgrid_individual);
  LOG(INFO) << "Iterative D* Result:";
  for (const auto& joint_position : iterative_dstar_result) {
    LOG(INFO) << search::fourgrid::util::JointPositionToString(
                     joint_position.first)
              << " cost: " << joint_position.second;
  }
  return 0;
}
