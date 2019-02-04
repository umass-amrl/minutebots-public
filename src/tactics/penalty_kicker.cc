// Copyright 2017 - 2018 slane@cs.umass.edu, jaholtz@cs.umass.edu
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

#include "tactics/penalty_kicker.h"

#include <algorithm>
#include <cmath>
#include "constants/constants.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "evaluators/offense_evaluation.h"
#include "math/geometry.h"
#include "math/math_util.h"
#include "math/poses_2d.h"
#include "motion_control/ntoc_2d.h"
#include "obstacles/obstacle_flag.h"
#include "obstacles/safety_margin.h"
#include "state/shared_state.h"
#include "state/soccer_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "tactics/eight_grid_navigation.h"

STANDARD_USINGS;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using geometry::Angle;
using state::WorldState;
using state::WorldRobot;
using state::SharedState;
using state::SharedRobotState;
using std::cos;
using std::sin;
using std::endl;
using std::map;
using std::unique_ptr;
using tactics::TacticIndex;
using offense::AimOption;
using offense::CalculateAimOptions;
using obstacle::SafetyMargin;
using obstacle::ObstacleType;
using ntoc::ControlPhase1D;
using ntoc::ControlPhase2D;
using ntoc::ControlSequence2D;
using motion::MotionModel;
using tactics::EightGridNavigation;

namespace tactics {
PenaltyKicker::PenaltyKicker(const WorldState& world_state,
                             TacticArray* tactic_list,
                             SharedState* shared_state,
                             OurRobotIndex our_robot_index,
                             state::SoccerState* soccer_state)
    : Tactic(world_state, tactic_list, shared_state, our_robot_index,
             soccer_state) {}

void PenaltyKicker::Init() {
  set_kick_goal = false;
  execution_state = PREPERATION;
  robot_goal_pose.Set(0.0f, Vector2f(1000, 1000));
  target_angle = 0.0f;
  target_position.x() = 4500.0f;
  target_position.y() = 0.0f;
  is_complete_ = false;
}

// Cost is currently the distance to the ball.
float PenaltyKicker::GetCost() {
  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  Pose2Df current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;
  Vector2f ball_pose = world_state_.GetBallPosition().position;

  MotionModel motion_model(kMaxRobotAcceleration, kMaxRobotVelocity);
  ControlSequence2D linear_control;

  Vector2f ntoc_position;
  Vector2f ntoc_velocity;

  ntoc::TransformCoordinatesForNTOC(current_pose, current_velocity.translation,
                                    ball_pose, &ntoc_position, &ntoc_velocity);

  ntoc_velocity.x() = 0;
  ntoc_velocity.y() = 0;

  return (NTOC2D(ntoc_position, ntoc_velocity, motion_model, &linear_control));
}

void PenaltyKicker::Run() {
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  robot_logger->LogPrint("Penalty Kicker");
  robot_logger->LogPrint("Execution State %d", execution_state);
  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  Vector2f ball_pose = world_state_.GetBallPosition().position;
  Vector2f goal_center = (kTheirGoalL + kTheirGoalR) / 2;
  Vector2f to_goal = ball_pose - goal_center;
  robot_logger->LogPrint("ToGoal x: %f, y: %f", to_goal.x(), to_goal.y());
  to_goal.normalize();
  Vector2f offset = 500 * to_goal;
  // Move to a position on the opposite side of the ball from the goal and wait.
  if (execution_state == PREPERATION) {
    is_complete_ = false;
    robot_goal_pose.translation.x() = ball_pose.x() + offset.x();
    robot_goal_pose.translation.y() = ball_pose.y() + offset.y();
    robot_goal_pose.angle = 0;
    // Move to a position for a kick.
  } else if (execution_state == PREKICK) {
    vector<Vector2f> world_robot_positions;
    for (const auto& robot : world_state_.GetOurRobots()) {
      if (world_state_.GetOurRobotIndex(robot.ssl_vision_id) !=
          static_cast<int>(our_robot_index_)) {
        world_robot_positions.push_back(robot.position.translation);
      }
    }
    for (const auto& robot : world_state_.GetTheirRobots()) {
      world_robot_positions.push_back(robot.position.translation);
    }
    is_complete_ = false;
    vector<AimOption> aim_options;

    CalculateAimOptions(ball_pose, kTheirGoalL, kTheirGoalR, kBallRadius,
                        kRobotRadius, world_robot_positions, &aim_options);

    if (!aim_options.empty()) {
      int max_width_index = -1;
      float max_width = -1;
      int current_index = 0;
      for (const AimOption& option : aim_options) {
        if (option.angle_width > max_width) {
          max_width = option.angle_width;
          max_width_index = current_index;
        }
        current_index++;
        LOG(WARNING) << option.angle_width << " " << option.angle_center;
      }
      target_angle = aim_options[max_width_index].angle_center;
    } else {
      LOG(WARNING) << "Found no options";
    }
    robot_goal_pose.translation.x() =
        ball_pose.x() -
        (kRotationRadius + kRotationRadiusMargin) * cos(target_angle);
    robot_goal_pose.translation.y() =
        ball_pose.y() -
        (kRotationRadius + kRotationRadiusMargin) * sin(target_angle);
    robot_goal_pose.angle = target_angle;
    EightGridNavigation* controller = static_cast<EightGridNavigation*>(
        (*tactic_list_)[TacticIndex::EIGHT_GRID].get());

    SafetyMargin margin;
    margin.SetMargin(ObstacleType::ROBOT, kNavigationRobotMargin);

    Vector2f ball_to_robot = current_pose.translation - ball_pose;

    float ball_to_robot_angle = Angle(ball_to_robot);

    Vector2f ball_to_target = target_position - ball_pose;

    float ball_to_target_angle = Angle(ball_to_target);

    if (fabs(AngleDiff(ball_to_robot_angle, ball_to_target_angle)) < M_PI) {
      margin.SetMargin(ObstacleType::BALL,
                       kNavigationBallMargin + kNavigationBallPad);
    } else {
      margin.SetMargin(ObstacleType::BALL, kNavigationBallMargin);
    }

    Pose2Df target_pose;
    target_pose.translation = target_position;
    target_pose.angle = target_angle;
    controller->SetGoal(target_pose);
    controller->SetAngularThreshold(kAngleThreshold);
    controller->SetLocationThreshold(kRotationRadiusMargin);

    controller->Run();
    // Kick
  } else if (execution_state == KICKING) {
    SharedRobotState* command = shared_state_->GetSharedState(our_robot_index_);
    command->flat_kick_set = true;
    command->flat_kick = 5.0;
    if (!set_kick_goal) {
      robot_goal_pose.translation.x() = ball_pose.x() + 250 * cos(target_angle);
      robot_goal_pose.translation.y() = ball_pose.y() + 250 * sin(target_angle);
      robot_goal_pose.angle = target_angle;
      set_kick_goal = true;
    }
    // Complete the action
  } else if (execution_state == POSTKICK) {
    is_complete_ = true;  // This could be handled better
    SharedRobotState* command = shared_state_->GetSharedState(our_robot_index_);
    command->flat_kick_set = true;
    command->flat_kick = 5.0;
    robot_goal_pose.translation.x() = 250.0f;
    robot_goal_pose.translation.y() = 0.0f;
    robot_goal_pose.angle = 0.0f;
    set_kick_goal = false;
  }

  // Switch states when goal is reached.
  if ((robot_goal_pose.translation - current_pose.translation).norm() < 20.0f &&
      std::abs(robot_goal_pose.angle - current_pose.angle) < DegToRad(5.0)) {
    switch (execution_state) {
      case PREKICK:
        execution_state = KICKING;
        break;
      case KICKING:
        execution_state = POSTKICK;
        break;
      case POSTKICK:
        execution_state = PREKICK;
        break;
      default:
        break;
    }
  }

  // Transition out of preparation when normal start is called.
  state::RefereeState ref_state = soccer_state_->GetRefereeState();
  if (ref_state.IsNormalStart()) {
    switch (execution_state) {
      case PREPERATION:
        execution_state = PREKICK;
        break;
      default:
        break;
    }
  }

  // Call the appropriate controller.
  if (execution_state == KICKING) {
    Tactic* controller = (*tactic_list_)[TacticIndex::THREE_KICK].get();
    Pose2Df temp_pose;
    temp_pose.translation = ball_pose;
    temp_pose.angle = target_angle;
    controller->SetGoal(temp_pose);
    controller->Run();
  }
}

void PenaltyKicker::Reset() {
  set_kick_goal = false;
  execution_state = PREPERATION;
  robot_goal_pose.Set(0.0f, Vector2f(1000, 1000));
  target_angle = 0.0f;
  target_position.x() = 4500.0f;
  target_position.y() = 0.0f;
  is_complete_ = false;
}

void PenaltyKicker::SetGoal(const Pose2Df& pose) {}

bool PenaltyKicker::IsComplete() { return is_complete_; }

}  // namespace tactics
