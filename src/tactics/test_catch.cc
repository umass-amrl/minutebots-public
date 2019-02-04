// Copyright 2017-2018 dbalaban@umass.edu
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

#include "tactics/test_catch.h"

#define _USE_MATH_DEFINES

#include <algorithm>
#include <cmath>

#include "constants/constants.h"
#include "datastructures/better_map.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "math/math_util.h"
#include "math/geometry.h"
#include "math/poses_2d.h"
#include "state/shared_state.h"
#include "state/soccer_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/safety_margin.h"
#include "navigation/navigation_util.h"
#include "motion_control/tsocs_old.h"
#include "motion_control/motion_control_structures.h"
#include "motion_control/ball_interception.h"
#include "tactics/navigate_to_catch.h"

STANDARD_USINGS;
using datastructures::OptionalValueMutable;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using Eigen::Vector2d;
using Eigen::Matrix2f;
using math_util::AngleMod;
using state::SharedRobotState;
using state::WorldState;
using state::WorldRobot;
using state::WorldBall;
using std::atan2;
using std::cos;
using std::endl;
using std::map;
using std::max;
using std::min;
using std::sin;
using std::abs;
using std::vector;
using std::unique_ptr;
using tactics::TacticIndex;
using state::SharedState;
using offense::AimOption;
using offense::CalculateAimOptions;
using offense::GetBestPassTarget;
using obstacle::ObstacleFlag;
using obstacle::SafetyMargin;
using navigation::CollisionFreePath;
using geometry::RayIntersect;

namespace tactics {
TestCatch::TestCatch(
    const WorldState& world_state, TacticArray* tactic_list,
    SharedState* shared_state, OurRobotIndex our_robot_index,
    state::SoccerState* soccer_state)
    : Tactic(world_state, tactic_list, shared_state, our_robot_index,
             soccer_state) {}

bool TestCatch::ShouldCatch() {
  Vector2f ball_to_robot_displace = current_robot_pose.translation
      - current_ball_pose;
  Vector2f projection = current_ball_pose
    + current_ball_velocity.normalized() * ball_to_robot_displace.norm();
  if ((projection - current_robot_pose.translation).norm() < kRobotRadius) {
    return true;
  }
  return false;
}

void TestCatch::Run() {
  the_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  current_robot_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  current_robot_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;
  current_ball_pose = world_state_.GetBallPosition().position;
  current_ball_velocity = world_state_.GetBallPosition().velocity;

  GetInterceptSolution(current_robot_pose.translation.cast<double>(),
                       current_velocity_world.cast<double>(),
                       current_ball_pose.cast<double>(),
                       current_ball_velocity.cast<double>(), kBallAcceleration,
                       kMaxRobotAcceleration, &intercept_solution);

  Vector2d final_robot_pos;
  Vector2d final_robot_vel;
  Vector2d final_ball_pos;
  Vector2d final_ball_vel;
  GetState(current_robot_pose.translation.cast<double>(),
           current_velocity_world.cast<double>(),
           current_ball_pose.cast<double>(),
           current_ball_velocity.cast<double>(), &final_robot_pos,
           &final_robot_vel, &final_ball_pos, &final_ball_vel,
           kMaxRobotAcceleration, intercept_solution.T, kBallAcceleration,
           intercept_solution);

  robot_interception_point = final_robot_pos.cast<float>();
  ball_intercept_point = final_ball_pos.cast<float>();
  velocity_at_intercept = final_robot_vel.cast<float>();


  if ((current_ball_pose - current_robot_pose.translation).norm() <
      kBallRadius / 100.0) {
    return;
  }

  if (current_ball_velocity.norm() < kBallVelocityThreshold) {
    return;
  }

  if (ShouldCatch()) {
    Tactic* controller = (*tactic_list_)[TacticIndex::CATCH].get();
    controller->Run();
  } else {
    NavigateToCatch* controller =
        static_cast<NavigateToCatch*>((*tactic_list_)
        [TacticIndex::NAVIGATE_TO_CATCH].get());
    controller->SetSolution(intercept_solution);
    controller->Run();
  }
}

void TestCatch::Init() {
  intercept_solution.isInitialized = false;
}

void TestCatch::Reset() {
  intercept_solution.isInitialized = false;
}

void TestCatch::SetGoal(const Pose2Df& pose) {}

}  // namespace tactics
