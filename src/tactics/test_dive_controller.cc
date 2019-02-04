// Copyright 2017 - 2018 jaholtz@cs.umass.edu
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

#include "tactics/test_dive_controller.h"

#define _USE_MATH_DEFINES

#include <algorithm>
#include <cmath>

#include "constants/constants.h"
#include "datastructures/better_map.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "math/math_util.h"
#include "math/poses_2d.h"
#include "math/geometry.h"
#include "state/shared_state.h"
#include "state/soccer_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "tactics/dive_controller.h"

STANDARD_USINGS;
using datastructures::OptionalValueMutable;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using Eigen::Matrix2f;
using math_util::AngleMod;
using geometry::EuclideanDistance;
using state::SharedRobotState;
using state::WorldState;
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
using geometry::RayIntersect;
using geometry::ProjectPointOntoLineSegment;
using geometry::Angle;

namespace tactics {

TestDiveController::TestDiveController(const WorldState& world_state,
                                 TacticArray* tactic_list,
                                 SharedState* shared_state,
                                 OurRobotIndex our_robot_index,
                                 state::SoccerState* soccer_state)
    : Tactic(world_state, tactic_list, shared_state, our_robot_index,
             soccer_state) {}

TestDiveController::~TestDiveController() {}

void TestDiveController::Run() {
  logger::Logger* the_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  the_logger->LogPrint("Test Dive Controller");
  the_logger->Push();
  Vector2f ball_pose = world_state_.GetBallPosition().position;
  Vector2f ball_velocity = world_state_.GetBallPosition().velocity;

  Vector2f ball_intersect_point;
  float ball_intersect_distance;
  bool ball_heading_towards_goal =
      RayIntersect(ball_pose, ball_velocity, kOurGoalL, kOurGoalR,
                  &ball_intersect_distance, &ball_intersect_point);

  const Vector2f current_ball_observation =
      world_state_.GetBallPosition().observed_pose;

  if (is_intercepting_) {
    if (current_ball_observation.x() < -kFieldXMax) {
      is_intercepting_ = false;
      interception_count_ = 0;
    }

    if (interception_count_ > kInterceptHysterisis) {
      is_intercepting_ = false;
      interception_count_ = 0;
    }
  }

  if ((ball_heading_towards_goal &&
       ball_velocity.squaredNorm() > kGoalieVelocityThreshold) ||
       is_intercepting_) {
    is_intercepting_ = true;

    if (ball_heading_towards_goal) {
      Vector2f target_point;
      float target_distance;

      bool intercepts_goalie_line =
          RayIntersect(ball_pose, ball_velocity, kGoalieLineL, kGoalieLineR,
                      &target_distance, &target_point);

      if (!intercepts_goalie_line) {
        ProjectPointOntoLineSegment(ball_intersect_point,
                                    kGoalieLineL,
                                    kGoalieLineR,
                                    &target_point,
                                    &target_distance);
      }

      time_to_goal_ = (target_distance / ball_velocity.norm());
      robot_goal_pose.translation = target_point;
      previous_intercept_point_ = target_point;
  } else {
    robot_goal_pose.translation = previous_intercept_point_;
    robot_goal_pose.angle = Angle<float>(ball_pose -
      previous_intercept_point_);
    time_to_goal_ = time_to_goal_ - kTransmitPeriodSeconds;
    interception_count_++;
  }
  the_logger->LogPrint("Diving");
  DiveController* controller =
  static_cast<DiveController*>(
    (*tactic_list_)[TacticIndex::DIVE_CONTROLLER].get());
  Pose2Df target;
  target.translation = robot_goal_pose.translation;
  target.angle = 0;
  controller->SetGoal(target);
  controller->SetTime(time_to_goal_);
  controller->Run();
  } else {
    the_logger->LogPrint("Halting");
    Tactic* controller =
      (*tactic_list_)[TacticIndex::HALT].get();
    controller->Run();
  }
}

void TestDiveController::Reset() {
  robot_goal_pose.translation = {0, 0};
  robot_goal_pose.angle = 0;
  goal_time_ = 0;
}

void TestDiveController::Init() {
  robot_goal_pose.translation = {0, 0};
  robot_goal_pose.angle = 0;
  goal_time_ = 0;
}

void TestDiveController::SetGoal(const pose_2d::Pose2Df& pose) {
  robot_goal_pose = pose;
}

void TestDiveController::SetTime(const double time) {
  goal_time_ = time;
}

}  // namespace tactics
