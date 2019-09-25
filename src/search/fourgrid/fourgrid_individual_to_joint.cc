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

#include <fstream>

#include "constants/constants.h"
#include "search/fourgrid/fourgrid.h"
#include "search/fourgrid/fourgrid_solver.h"

using ::search::fourgrid::FourGrid;
using ::search::fourgrid::JointPosition;
using ::search::fourgrid::solver::expandingastar::FourGridSolver;
using ::search::fourgrid::solver::expandingastar::SolveInstrumentation;
using ::search::fourgrid::util::FileToJointPosition;
using ::search::fourgrid::util::CostPathCollides;
using ::search::fourgrid::CostPath;

#define ROBOTCOUNT (2)

static constexpr size_t kRobotCount = ROBOTCOUNT;

void Setup(char* name) {
  google::InitGoogleLogging(name);
  FLAGS_stderrthreshold = 0;
  FLAGS_colorlogtostderr = 1;
  FLAGS_logtostderr = true;
}

struct Scenario {
  FourGrid<kRobotCount> fourgrid_joint;
  JointPosition<kRobotCount> fourgrid_joint_start;
  JointPosition<kRobotCount> fourgrid_joint_end;
  std::array<FourGrid<1>, kRobotCount> fourgrid_individuals;
  std::array<JointPosition<1>, kRobotCount> fourgrid_individual_starts;
  std::array<JointPosition<1>, kRobotCount> fourgrid_individual_goals;
};

#if (ROBOTCOUNT == 3)

Scenario MakeSplit3() {
  return Scenario{
      FourGrid<kRobotCount>("fourgrid3_split3_joint.txt"),
      FileToJointPosition<kRobotCount>("fourgrid3_split3_joint_start.txt"),
      FileToJointPosition<kRobotCount>("fourgrid3_split3_joint_goal.txt"),
      {{FourGrid<1>("fourgrid3_split3_individual_0.txt"),
        FourGrid<1>("fourgrid3_split3_individual_1.txt"),
        FourGrid<1>("fourgrid3_split3_individual_2.txt")}},
      {{FileToJointPosition<1>("fourgrid3_split3_individual_0_start.txt"),
        FileToJointPosition<1>("fourgrid3_split3_individual_1_start.txt"),
        FileToJointPosition<1>("fourgrid3_split3_individual_2_start.txt")}},
      {{FileToJointPosition<1>("fourgrid3_split3_individual_0_goal.txt"),
        FileToJointPosition<1>("fourgrid3_split3_individual_1_goal.txt"),
        FileToJointPosition<1>("fourgrid3_split3_individual_2_goal.txt")}}};
}
#endif

#if (ROBOTCOUNT == 2)
Scenario MakeSplit() {
  return Scenario{
      FourGrid<kRobotCount>("fourgrid3_split_joint.txt"),
      FileToJointPosition<kRobotCount>("fourgrid3_split_joint_start.txt"),
      FileToJointPosition<kRobotCount>("fourgrid3_split_joint_goal.txt"),
      {{FourGrid<1>("fourgrid3_split_individual_0.txt"),
        FourGrid<1>("fourgrid3_split_individual_1.txt")}},
      {{FileToJointPosition<1>("fourgrid3_split_individual_0_start.txt"),
        FileToJointPosition<1>("fourgrid3_split_individual_1_start.txt")}},
      {{FileToJointPosition<1>("fourgrid3_split_individual_0_goal.txt"),
        FileToJointPosition<1>("fourgrid3_split_individual_1_goal.txt")}}};
}

Scenario MakeSplitDual() {
  return Scenario{
      FourGrid<kRobotCount>("fourgrid3_split_dual_joint.txt"),
      FileToJointPosition<kRobotCount>("fourgrid3_split_dual_joint_start.txt"),
      FileToJointPosition<kRobotCount>("fourgrid3_split_dual_joint_goal.txt"),
      {{FourGrid<1>("fourgrid3_split_dual_individual_0.txt"),
        FourGrid<1>("fourgrid3_split_dual_individual_1.txt")}},
      {{FileToJointPosition<1>("fourgrid3_split_dual_individual_0_start.txt"),
        FileToJointPosition<1>("fourgrid3_split_dual_individual_1_start.txt")}},
      {{FileToJointPosition<1>("fourgrid3_split_dual_individual_0_goal.txt"),
        FileToJointPosition<1>("fourgrid3_split_dual_individual_1_goal.txt")}}};
}

Scenario MakeCorners() {
  return Scenario{
      FourGrid<kRobotCount>("fourgrid3_corners_joint.txt"),
      FileToJointPosition<kRobotCount>("fourgrid3_corners_joint_start.txt"),
      FileToJointPosition<kRobotCount>("fourgrid3_corners_joint_goal.txt"),
      {{FourGrid<1>("fourgrid3_corners_individual_0.txt"),
        FourGrid<1>("fourgrid3_corners_individual_1.txt")}},
      {{FileToJointPosition<1>("fourgrid3_corners_individual_0_start.txt"),
        FileToJointPosition<1>("fourgrid3_corners_individual_1_start.txt")}},
      {{FileToJointPosition<1>("fourgrid3_corners_individual_0_goal.txt"),
        FileToJointPosition<1>("fourgrid3_corners_individual_1_goal.txt")}}};
}

Scenario MakeStraight() {
  return Scenario{
      FourGrid<kRobotCount>("fourgrid3_straight_joint.txt"),
      FileToJointPosition<kRobotCount>("fourgrid3_straight_joint_start.txt"),
      FileToJointPosition<kRobotCount>("fourgrid3_straight_joint_goal.txt"),
      {{FourGrid<1>("fourgrid3_straight_individual_0.txt"),
        FourGrid<1>("fourgrid3_straight_individual_1.txt")}},
      {{FileToJointPosition<1>("fourgrid3_straight_individual_0_start.txt"),
        FileToJointPosition<1>("fourgrid3_straight_individual_1_start.txt")}},
      {{FileToJointPosition<1>("fourgrid3_straight_individual_0_goal.txt"),
        FileToJointPosition<1>("fourgrid3_straight_individual_1_goal.txt")}}};
}
#endif

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

Expansions ComputeExpansions(
    const Scenario& scenario,
    const CostPath<kRobotCount>& joint_unrepaired_plan,
    const JointPosition<kRobotCount>& collision_center) {
  Expansions expansions;

  for (size_t r = 5; r <= 12; ++r) {
    LOG(INFO) << "Window radius " << r;
    auto window_fourgrid =
        scenario.fourgrid_joint.RestrictL1Norm(collision_center, r);

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

    if (scenario.fourgrid_joint_end != window_goal.second) {
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

void DumpInstrumentation(const std::string& file_name,
                         const SolveInstrumentation& instrumentation) {
  NP_CHECK_EQ(instrumentation.queue_push_pops.size(),
              instrumentation.state_expansions.size());
  std::ofstream ofs(file_name);
  for (size_t i = 0; i < instrumentation.state_expansions.size(); ++i) {
    const auto& expansions = instrumentation.state_expansions[i];
    const auto& push_pops = instrumentation.queue_push_pops[i];
    const auto& reorders = instrumentation.queue_reorders[i];
    ofs << expansions << ", " << push_pops << ", " << reorders << '\n';
  }
  ofs.close();
  LOG(INFO) << "Dumped to: " << file_name;
}

int main(int argc, char** argv) {
  Setup(argv[0]);
  const Scenario scenario = MakeSplit();

  FourGridSolver<kRobotCount> fgs;

  const auto individual_plans = fgs.SolveIndividualAStar(
      scenario.fourgrid_individuals, scenario.fourgrid_individual_starts,
      scenario.fourgrid_individual_goals);
  const auto joint_unrepaired_plan = fgs.ZipIndividualPlans(individual_plans);

  LOG(INFO) << "Joint unrepaired plan:";
  for (const auto& p : joint_unrepaired_plan) {
    LOG(INFO) << ::search::fourgrid::util::JointPositionToString(p.first)
              << " cost: " << p.second;
  }

  const JointPosition<kRobotCount> collision_center =
      ::search::fourgrid::util::CostPathCenter(joint_unrepaired_plan);
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
  DumpInstrumentation("ea_star.txt", ea_star_plan_result.second);
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

  LOG(INFO) << "NAIVE WINDOW A*";
  const auto na_start = GetMonotonicTime();
  const auto window_a_star_plan_result = fgs.SolveAStar(
      expansions.four_grids, expansions.starts, expansions.goals);
  const auto na_end = GetMonotonicTime();
  const auto window_a_star_plan = window_a_star_plan_result.first;
  DumpInstrumentation("nwa_star.txt", window_a_star_plan_result.second);
  LOG(INFO) << "NWA* TIME ms: " << (na_end - na_start) * 1000;
  NP_CHECK(!CostPathCollides(window_a_star_plan).first);

  LOG(INFO) << "PLAN:";
  for (const auto& p : window_a_star_plan) {
    LOG(INFO) << ::search::fourgrid::util::JointPositionToString(p.first)
              << " cost: " << p.second;
  }

  LOG(INFO) << "JOINT PLANNING A*";
  const auto ja_start = GetMonotonicTime();
  const auto joint_a_star_plan_result =
      fgs.SolveAStar({scenario.fourgrid_joint}, {scenario.fourgrid_joint_start},
                     {scenario.fourgrid_joint_end});
  const auto ja_end = GetMonotonicTime();
  const auto joint_a_star_plan = joint_a_star_plan_result.first;
  DumpInstrumentation("joint.txt", joint_a_star_plan_result.second);
  LOG(INFO) << "JOINT TIME ms: " << (ja_end - ja_start) * 1000;
  NP_CHECK(!CostPathCollides(joint_a_star_plan).first);

  LOG(INFO) << "PLAN:";
  for (const auto& p : joint_a_star_plan) {
    LOG(INFO) << ::search::fourgrid::util::JointPositionToString(p.first)
              << " cost: " << p.second;
  }
  NP_CHECK(joint_a_star_plan == window_a_star_plan);
  NP_CHECK(joint_a_star_plan[joint_a_star_plan.size() - 1].second ==
           ea_star_plan[ea_star_plan.size() - 1].second)
}
