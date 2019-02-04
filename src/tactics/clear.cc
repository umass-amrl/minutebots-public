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

#include "tactics/clear.h"
#include <algorithm>
#include <cmath>
#include "constants/constants.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "evaluators/offense_evaluation.h"
#include "math/math_util.h"
#include "math/poses_2d.h"
#include "state/shared_state.h"
#include "state/soccer_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "tactics/prm_navigation.h"
#include "tactics/stox_navigation.h"
#include "obstacles/obstacle_flag.h"
#include "tactics/pivot2.h"
#include "tactics/kick.h"

STANDARD_USINGS;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
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
using offense::GetBestPassTarget;
using offense::GetBestChipTarget;

namespace tactics {
Clear::Clear(const WorldState& world_state,
                               TacticArray* tactic_list,
                               SharedState* shared_state,
                               OurRobotIndex our_robot_index,
                               state::SoccerState* soccer_state)
    : Tactic(world_state, tactic_list, shared_state, our_robot_index,
             soccer_state) {}

void Clear::Init() {
  set_kick_goal = false;
  execution_state = PREKICK;
  robot_goal_pose.Set(0.0f, Vector2f(1000, 1000));
  target_angle = 0.0f;
  target_position.x() = 4500.0f;
  target_position.y() = 0.0f;
  is_complete_ = false;
  should_pass_ = false;
  chip_ = false;
}

bool Clear::ShouldGoToBall(const Vector2f& ball,
                                      const Vector2f& robot) {
  bool dist_check = (ball - robot).norm()
      > kRotationRadius + kRotationRadiusMargin;

  return dist_check;
}

bool Clear::ShouldKick(const Vector2f& ball, const Pose2Df& robot,
                                    const Vector2f& current_ball_velocity,
                                    const Pose2Df& current_robot_velocity) {
  Eigen::Rotation2Df robot_to_world_rotation(robot.angle);
  Eigen::Rotation2Df world_to_target_rotation(-target_angle);
  Eigen::Rotation2Df robot_to_target_rotation =
      world_to_target_rotation * robot_to_world_rotation;

  Vector2f robot_prime_vel =
      robot_to_target_rotation * current_robot_velocity.translation;

  Vector2f robot_to_ball_displace =
        ball - robot.translation;
  const float robot_heading = robot.angle;
  const Vector2f y_dir(-sin(robot_heading), cos(robot_heading));

  float y_dist = y_dir.dot(robot_to_ball_displace);
  float robot_y_prime_vel = robot_prime_vel.y();
  float ball_y_prime_vel =
      (world_to_target_rotation * current_ball_velocity).y();
  float rel_y_prime_vel = robot_y_prime_vel - ball_y_prime_vel;

  Vector2f current_velocity_world =
      robot_to_world_rotation * current_robot_velocity.translation;

  Vector2f relative_velocity_vector =
      current_velocity_world - current_ball_velocity;

  const bool is_at_angle =
      fabs(AngleDiff(target_angle, robot.angle))
      < kAngleThreshold;
  const bool is_at_radial_dist =
      robot_to_ball_displace.norm()
      < kRotationRadius + kRotationRadiusMargin;
  const bool is_y_prime_at_relative_rest = fabs(rel_y_prime_vel)
      < kYPrimeVelocitythreshold;
  const bool is_at_relative_rest =
      (relative_velocity_vector).norm()
      < kRelativeBallVelocityThreshold;
  const bool is_in_alignment = fabs(y_dist) < kAlignmentThreshold;
  const bool is_rotation_at_rest = fabs(current_robot_velocity.angle)
      < kAngularVelocitythreshold;
  const bool should_kick =
      (is_at_angle && is_at_radial_dist
      && is_y_prime_at_relative_rest
      && is_at_relative_rest && is_in_alignment
      && is_rotation_at_rest);

  return should_kick;
}

bool Clear::ShouldPivot(const Vector2f& ball,
                                   const Pose2Df& robot) {
  Vector2f robot_to_ball_displace =
        ball - robot.translation;
  bool check_dist = robot_to_ball_displace.norm()
      < kNavigationBallMargin + kRotationRadius;

  return check_dist;
}

void Clear::Run() {
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  robot_logger->LogPrint("Clear");
  robot_logger->LogPrint("Execution State %d", execution_state);

  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  Vector2f ball_pose = world_state_.GetBallPosition().position;

  Pose2Df current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;

  Vector2f ball_vel = world_state_.GetBallPosition().velocity;

  // Switch states when goal is reached.
  if ((robot_goal_pose.translation - current_pose.translation).norm() <
        kNavigationBallMargin + kRotationRadius &&
     std::abs(robot_goal_pose.angle - current_pose.angle) < kAngleThreshold) {
    switch (execution_state) {
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

  if (execution_state == PREKICK) {
    if (ShouldKick(ball_pose, current_pose, ball_vel, current_velocity)) {
      execution_state = KICKING;
    }
  }

  // Find an aim option and use this as goal.
  if (execution_state == PREKICK) {
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
    bool should_pass_ = false;
    vector<AimOption> aim_options;
    CalculateAimOptions(ball_pose, kTheirGoalL, kTheirGoalR, kBallRadius,
                        kRobotRadius, world_robot_positions, &aim_options);


    robot_logger->LogPrint("Aim Options Size %d", aim_options.size());
    if (!aim_options.empty()) {
      int max_width_index = -1;
      float max_width = -1;
      int current_index = 0;

      for (const AimOption& option : aim_options) {
        robot_logger->AddLine(current_pose.translation, option.target_l, 1.0, 0,
                              0, 1.0);
        robot_logger->AddLine(current_pose.translation, option.target_r, 0, 0,
                              1.0, 1.0);
        if (option.angle_width > max_width) {
          max_width = option.angle_width;
          max_width_index = current_index;
        }
        current_index++;
      }
      target_angle = aim_options[max_width_index].angle_center;
      robot_logger->LogPrint("Max Width: %f", max_width);
    } else {
      should_pass_ = true;
    }
    // If no good aim option for shooting on the goal, pass the ball.
    if (should_pass_) {
      zone::FieldZone field_zone(zone::FULL_FIELD);
      OurRobotIndex receiving_robot;
      target_angle = GetBestPassTarget(
          world_state_, soccer_state_, our_robot_index_,
          world_state_.GetOurRobotPosition(our_robot_index_).ssl_vision_id,
          field_zone, &receiving_robot);
    }

    robot_goal_pose.translation.x() = ball_pose.x()
        - (kRotationRadius + kRotationRadiusMargin) * cos(target_angle);
    robot_goal_pose.translation.y() = ball_pose.y()
        - (kRotationRadius + kRotationRadiusMargin) * sin(target_angle);
    robot_goal_pose.angle = target_angle;
    StoxNavigation* controller = static_cast<StoxNavigation*>(
      (*tactic_list_)[TacticIndex::STOX_NAVIGATION].get());
    Pose2Df target_pose;
    target_pose.translation = robot_goal_pose.translation;
    target_pose.angle = target_angle;
    controller->SetGoal(target_pose);
    controller->Run();
    // Kick the ball
  } else if (execution_state == KICKING) {
    SharedRobotState* command = shared_state_->GetSharedState(our_robot_index_);
    command->flat_kick_set = true;
    // Kick the ball
  } else if (execution_state == KICKING) {
    SharedRobotState* command = shared_state_->GetSharedState(our_robot_index_);
    if (!should_pass_) {
      command->flat_kick = 5.0;
    } else {
      if (!chip_) {
        command->flat_kick_set = true;
        command->flat_kick = 2.0;
      } else {
          command->chip_kick_set = true;
          command->chip_kick = 5.0;
      }
    }
    if (!set_kick_goal) {
      robot_goal_pose.translation.x() = ball_pose.x() + 250 * cos(target_angle);
      robot_goal_pose.translation.y() = ball_pose.y() + 250 * sin(target_angle);
      robot_goal_pose.angle = target_angle;
      set_kick_goal = true;
    }
    // Set Complete Status
  } else if (execution_state == POSTKICK) {
    is_complete_ = true;  // This could be handled better
    set_kick_goal = false;
    execution_state = PREKICK;
    robot_goal_pose.Set(0.0f, Vector2f(1000, 1000));
    target_angle = 0.0f;
    target_position.x() = 4500.0f;
    target_position.y() = 0.0f;
    set_kick_goal = false;
  }
// Call Appropriate Controllers
  if (execution_state == KICKING) {
    Kick* controller =
          static_cast<Kick*>(
            (*tactic_list_)[TacticIndex::KICK].get());
    SharedRobotState* command = shared_state_->GetSharedState(our_robot_index_);
    if (!should_pass_) {
      command->flat_kick = 5.0;
    } else {
      command->flat_kick = 1.3;
    }

    if (command->chip_kick_set) {
      controller->SetKickSpeed(command->chip_kick, command->chip_kick_set);
    } else {
      controller->SetKickSpeed(command->flat_kick, command->chip_kick_set);
    }

  // Switch states when goal is reached.
if ((robot_goal_pose.translation - current_pose.translation).norm() < 10.0f &&
     std::abs(robot_goal_pose.angle - current_pose.angle) < 0.75) {
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

  // Call Appropriate Controllers
  if (execution_state == KICKING) {
    Tactic* controller =
        (*tactic_list_)[TacticIndex::KICK].get();
    Pose2Df temp_pose;
    temp_pose.translation = ball_pose;
    temp_pose.angle = target_angle;
    controller->SetGoal(temp_pose);
    controller->Run();
  } else {
    StoxNavigation* controller =
          static_cast<StoxNavigation*>(
            (*tactic_list_)[TacticIndex::STOX_NAVIGATION].get());
    obstacle::ObstacleFlag flags =
        obstacle::ObstacleFlag::GetAllExceptTeam(world_state_,
                                                 *soccer_state_,
                                                 our_robot_index_,
                                                 our_robot_index_);

    if (ball_pose.x() > current_pose.translation.x() - kRobotRadius) {
      obstacle::ObstacleFlag ball_flag = ball_flag.GetBall();
      flags = flags & ~ball_flag;
    }

    controller->SetObstacles(flags);
    controller->SetGoal(robot_goal_pose);
    controller->Run();
  }
}
}

void Clear::Reset() {
  set_kick_goal = false;
  execution_state = PREKICK;
  robot_goal_pose.Set(0.0f, Vector2f(1000, 1000));
  target_angle = 0.0f;
  target_position.x() = 4500.0f;
  target_position.y() = 0.0f;
  is_complete_ = false;
  should_pass_ = false;
  chip_ = false;
}

void Clear::SetGoal(const Pose2Df& pose) {}

bool Clear::IsComplete() { return is_complete_; }

}  // namespace tactics
