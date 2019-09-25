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

#include "search/fourgrid/fourgrid.h"
#include "constants/constants.h"
#include "search/fourgrid/dstar_fourgrid_solver.h"
#include "search/fourgrid/fourgrid_solver.h"

using ::search::fourgrid::FourGrid;
using ::search::fourgrid::JointPosition;
using ::search::fourgrid::solver::expandingastar::FourGridSolver;
using ::search::fourgrid::solver::expandingastar::SolveInstrumentation;
using ::search::fourgrid::util::FileToJointPosition;
using ::search::fourgrid::util::CostPathCollides;
using ::search::fourgrid::CostPath;
using ::search::fourgrid::solver::dstar::DStarFourGridSolver;

static constexpr size_t kRobotCount = 1;

void Setup(char* name) {
  google::InitGoogleLogging(name);
  FLAGS_stderrthreshold = 0;
  FLAGS_colorlogtostderr = 1;
  FLAGS_logtostderr = true;
}

struct Scenario {
  static_assert(kRobotCount == 1, "kRobotCount must be 1!");
  FourGrid<kRobotCount> fourgrid_blocked;
  FourGrid<kRobotCount> fourgrid_unblocked;
  JointPosition<kRobotCount> fourgrid_start;
  JointPosition<kRobotCount> fourgrid_goal;
};

struct Expansions {
  std::vector<FourGrid<kRobotCount>> four_grids;
  std::vector<JointPosition<kRobotCount>> starts;
  std::vector<JointPosition<kRobotCount>> goals;
  std::vector<CostPath<kRobotCount>> path_between_starts;
  std::vector<CostPath<kRobotCount>> path_between_goals;

  void Verify() {
    NP_CHECK(four_grids.size() == starts.size());
    NP_CHECK(goals.size() == starts.size());

    NP_CHECK(path_between_starts.size() == starts.size());
    for (size_t i = 0; i < starts.size(); ++i) {
      NP_CHECK(!path_between_starts[i].empty() || i == 0);
      if (!path_between_starts[i].empty() && i > 0) {
        NP_CHECK(starts[i] == path_between_starts[i][0].first);
        NP_CHECK(
            starts[i - 1] ==
            path_between_starts[i][path_between_starts[i].size() - 1].first);
      }
    }

    NP_CHECK(path_between_goals.size() == goals.size());
    for (size_t i = 0; i < goals.size(); ++i) {
      if (i > 0 && !path_between_goals[i].empty() &&
          !path_between_goals[i - 1].empty()) {
        NP_CHECK(goals[i - 1] == path_between_goals[i][0].first);
        NP_CHECK(goals[i] ==
                 path_between_goals[i][path_between_goals[i].size() - 1].first);
      }
    }

    for (const auto& p : starts) {
      NP_CHECK(!::search::fourgrid::util::JointPositionCollides(p));
    }

    for (const auto& p : goals) {
      NP_CHECK(!::search::fourgrid::util::JointPositionCollides(p));
    }

    for (const auto& path : path_between_starts) {
      if (!path.empty() &&
          ::search::fourgrid::util::CostPathCollides(path).first) {
        LOG(ERROR) << "Colliding path between starts!";
        for (const auto& p : path) {
          LOG(ERROR) << ::search::fourgrid::util::JointPositionToString(p.first)
                     << " " << p.second;
        }
        LOG(FATAL) << "Exit";
      }
    }

    for (size_t i = 1; i < path_between_goals.size(); ++i) {
      const auto& path = path_between_goals[i];
      if (!path.empty() &&
          ::search::fourgrid::util::CostPathCollides(path).first) {
        const auto& start = starts[i];
        const auto& goal = goals[i];
        const auto& prev_start = starts[i - 1];
        const auto& prev_goal = goals[i - 1];

        LOG(INFO) << "Index: " << i;
        LOG(INFO) << "Start: "
                  << ::search::fourgrid::util::JointPositionToString(start);
        LOG(INFO) << "Goal: "
                  << ::search::fourgrid::util::JointPositionToString(goal);
        LOG(INFO) << "Prev Start: "
                  << ::search::fourgrid::util::JointPositionToString(
                         prev_start);
        LOG(INFO) << "Prev Goal: "
                  << ::search::fourgrid::util::JointPositionToString(prev_goal);

        LOG(ERROR) << "Colliding path between goals!";
        for (const auto& p : path) {
          LOG(ERROR) << ::search::fourgrid::util::JointPositionToString(p.first)
                     << " " << p.second;
        }
        LOG(FATAL) << "Exit";
      }
    }
  }
};

Scenario MakeSplitCorner() {
  return {FourGrid<kRobotCount>("fourgrid3_single_robot_split_blocked.txt"),
          FourGrid<kRobotCount>("fourgrid3_single_robot_split_unblocked.txt"),
          FileToJointPosition<1>(
              "fourgrid3_single_robot_split_individual_start.txt"),
          FileToJointPosition<1>(
              "fourgrid3_single_robot_split_individual_goal.txt")};
}

Scenario MakeDStarExample() {
  return {FourGrid<kRobotCount>("fourgrid3_single_robot_dstar_blocked.txt"),
          FourGrid<kRobotCount>("fourgrid3_single_robot_dstar_unblocked.txt"),
          FileToJointPosition<1>(
              "fourgrid3_single_robot_dstar_individual_start.txt"),
          FileToJointPosition<1>(
              "fourgrid3_single_robot_dstar_individual_goal.txt")};
}

Expansions ComputeExpansions(
    const Scenario& scenario,
    const CostPath<kRobotCount>& joint_unrepaired_plan,
    const JointPosition<kRobotCount>& collision_center) {
  Expansions expansions;

  for (size_t r = 5; r <= 10; ++r) {
    LOG(INFO) << "Window radius " << r;
    auto window_fourgrid =
        scenario.fourgrid_blocked.RestrictL1Norm(collision_center, r);

    const auto window_start = ::search::fourgrid::util::CostPathEntrance(
        joint_unrepaired_plan, window_fourgrid);
    NP_CHECK(window_start.first);
    LOG(INFO) << "Start: " << ::search::fourgrid::util::JointPositionToString(
                                  window_start.second);
    const auto window_goal = ::search::fourgrid::util::CostPathExit(
        joint_unrepaired_plan, window_fourgrid);
    NP_CHECK(window_goal.first);
    LOG(INFO) << "End: " << ::search::fourgrid::util::JointPositionToString(
                                window_goal.second);

    if (scenario.fourgrid_goal != window_goal.second) {
      window_fourgrid.AugmentPartialGoalsWithSelfLoops(window_goal.second);
    }

    const auto prev_window_start =
        (expansions.starts.empty()
             ? window_start.second
             : expansions.starts[expansions.starts.size() - 1]);
    const auto prev_window_goal =
        (expansions.starts.empty()
             ? window_goal.second
             : expansions.goals[expansions.goals.size() - 1]);

    const auto path_between_starts =
        ::search::fourgrid::util::PathBetweenStarts(
            joint_unrepaired_plan, window_start.second, prev_window_start);
    const auto path_between_goals = ::search::fourgrid::util::PathBetweenGoals(
        joint_unrepaired_plan, window_goal.second, prev_window_goal);

    expansions.four_grids.push_back(window_fourgrid);
    expansions.starts.push_back(window_start.second);
    expansions.goals.push_back(window_goal.second);
    expansions.path_between_starts.push_back(path_between_starts);
    expansions.path_between_goals.push_back(path_between_goals);
  }

  //   expansions.four_grids.push_back(scenario.fourgrid_joint);
  //   expansions.starts.push_back(scenario.fourgrid_joint_start);
  //   expansions.goals.push_back(scenario.fourgrid_joint_end);
  //   expansions.path_between_starts.push_back(
  //       ::search::fourgrid::util::PathBetweenStarts(
  //           joint_unrepaired_plan, scenario.fourgrid_joint_start,
  //           expansions.starts[expansions.starts.size() - 2]));
  //
  expansions.Verify();
  return expansions;
}

int main(int argc, char** argv) {
  Setup(argv[0]);

  const Scenario scenario = MakeDStarExample();

  DStarFourGridSolver<kRobotCount> dstar_solver;
  const CostPath<kRobotCount> result = dstar_solver.SolveDStarOneShot(
      scenario.fourgrid_start, scenario.fourgrid_goal,
      scenario.fourgrid_unblocked, scenario.fourgrid_blocked);

  FourGridSolver<kRobotCount> fgs;

  const auto individual_plans = fgs.SolveIndividualAStar(
      {{scenario.fourgrid_unblocked}}, {{scenario.fourgrid_start}},
      {{scenario.fourgrid_goal}});
  const auto joint_unrepaired_plan = fgs.ZipIndividualPlans(individual_plans);

  const JointPosition<kRobotCount> collision_center =
      ::search::fourgrid::util::GridChangeCollideCenter(
          joint_unrepaired_plan, scenario.fourgrid_unblocked,
          scenario.fourgrid_blocked);
  LOG(INFO) << "Collision center: "
            << ::search::fourgrid::util::JointPositionToString(
                   collision_center);

  const Expansions expansions =
      ComputeExpansions(scenario, joint_unrepaired_plan, collision_center);

  LOG(INFO) << "EXPANDING A*";
  const auto ea_start = GetMonotonicTime();
  const auto ea_star_plan_result = fgs.SolveExpandingAStar(
      expansions.four_grids, expansions.starts, expansions.goals,
      expansions.path_between_starts, expansions.path_between_goals);
  const auto ea_end = GetMonotonicTime();
  const auto ea_star_plan = ea_star_plan_result.first;
  //   DumpInstrumentation("ea_star.txt", ea_star_plan_result.second);
  LOG(INFO) << "EA* TIME ms: " << (ea_end - ea_start) * 1000;
  LOG(INFO) << "PLAN:";
  for (const auto& p : ea_star_plan) {
    LOG(INFO) << ::search::fourgrid::util::JointPositionToString(p.first)
              << " cost: " << p.second;
  }
  NP_CHECK_MSG(!CostPathCollides(ea_star_plan).first,
               "Collides at "
                   << ::search::fourgrid::util::JointPositionToString(
                          CostPathCollides(ea_star_plan).second.from)
                   << " => " << ::search::fourgrid::util::JointPositionToString(
                                    CostPathCollides(ea_star_plan).second.to));

  return 0;
}
