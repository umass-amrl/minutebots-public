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

#include "tactics/test_ball_interception.h"

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
#include "obstacles/obstacle_type.h"
#include "navigation/navigation_util.h"
#include "motion_control/tsocs_old.h"
#include "motion_control/motion_control_structures.h"
#include "motion_control/ball_interception.h"
#include "tactics/navigate_to_intercept.h"
#include "tactics/kick.h"
#include "tactics/ball_interception.h"

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
using obstacle::ObstacleType;
using navigation::CollisionFreePath;
using geometry::RayIntersect;

namespace tactics {
TestBallInterception::TestBallInterception(
    const WorldState& world_state, TacticArray* tactic_list,
    SharedState* shared_state, OurRobotIndex our_robot_index,
    state::SoccerState* soccer_state)
    : Tactic(world_state, tactic_list, shared_state, our_robot_index,
             soccer_state) {}

void TestBallInterception::GetTargetAngle(Vector2f ball_pose) {
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
  bool should_pass = false;
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
    }
    target_angle = aim_options[max_width_index].angle_center;
  } else {
    should_pass = true;
  }
  // If no good aim option for shooting on the goal, pass the ball.
  if (should_pass) {
    zone::FieldZone field_zone(zone::FULL_FIELD);
    OurRobotIndex receiving_index = 42;
    target_angle = GetBestPassTarget(
        world_state_, soccer_state_, our_robot_index_,
        world_state_.GetOurRobotPosition(our_robot_index_).ssl_vision_id,
        field_zone, &receiving_index);
  }
}

void TestBallInterception::GetSolutions() {
  Vector2f desired_norm(cos(target_angle), sin(target_angle));
  // the balls center should be at (kRobotRadius + kBallRadius) * desired_norm
  // relative to the robot's center at collision time
  Vector2f collision_point =
      current_robot_pose.translation + kInterceptionRadius * desired_norm;

  SolutionParameters save_intercept = intercept_solution;

  GetInterceptSolution(collision_point.cast<double>(),
                       current_velocity_world.cast<double>(),
                       current_ball_pose.cast<double>(),
                       current_ball_velocity.cast<double>(), kBallAcceleration,
                       kMaxRobotAcceleration, &intercept_solution);

  Vector2d final_robot_pos;
  Vector2d final_collision_pos;
  Vector2d final_robot_vel;
  Vector2d final_ball_pos;
  Vector2d final_ball_vel;
  GetState(collision_point.cast<double>(),
           current_velocity_world.cast<double>(),
           current_ball_pose.cast<double>(),
           current_ball_velocity.cast<double>(), &final_collision_pos,
           &final_robot_vel, &final_ball_pos, &final_ball_vel,
           kMaxRobotAcceleration, intercept_solution.T, kBallAcceleration,
           intercept_solution);

  if ((final_ball_pos - final_collision_pos).norm() > kBadSolutionthreshold) {
    intercept_solution.isInitialized = false;
    GetInterceptSolution(collision_point.cast<double>(),
                        current_velocity_world.cast<double>(),
                        current_ball_pose.cast<double>(),
                        current_ball_velocity.cast<double>(), kBallAcceleration,
                        kMaxRobotAcceleration, &intercept_solution);

    GetState(collision_point.cast<double>(),
            current_velocity_world.cast<double>(),
            current_ball_pose.cast<double>(),
            current_ball_velocity.cast<double>(), &final_collision_pos,
            &final_robot_vel, &final_ball_pos, &final_ball_vel,
            kMaxRobotAcceleration, intercept_solution.T, kBallAcceleration,
            intercept_solution);
  }

  if ((final_ball_pos - final_collision_pos).norm() > kBadSolutionthreshold) {
    intercept_solution = save_intercept;
  }

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
}


bool TestBallInterception::DoesPathCrossBallRay() {
  float dist;
  Vector2f point;
  return RayIntersect(current_ball_pose,
                      current_ball_velocity.normalized(),
                      current_robot_pose.translation,
                      robot_interception_point, &dist, &point);
}

bool TestBallInterception::ShouldKick() {
  const bool is_currenlty_kicking = execution_state == KICK;
  const bool is_timed_out = (world_state_.world_time_
      >= (kick_start_time + kKickTimeOut));

  the_logger->LogPrint("Chekcing Kick Conditions");
  the_logger->Push();
  const bool should_kick =
      Kick::ShouldKick(the_logger,
                       target_angle,
                       current_ball_pose,
                       current_ball_velocity,
                       current_robot_pose,
                       current_robot_velocity,
                       is_currenlty_kicking,
                       is_timed_out,
                       kDebug_);


  if (kDebug_) {
    if (is_timed_out && is_currenlty_kicking) {
      the_logger->LogPrint("Kicking Timed Out");
      the_logger->Push();
      the_logger->LogPrint("Kicking Start Time: %f", kick_start_time);
      the_logger->LogPrint("Kicking End Time: %f",
                          kick_start_time + kKickTimeOut);
      the_logger->LogPrint("World Time: %f", world_state_.world_time_);
      if (execution_state == KICK) {
        the_logger->LogPrint("Execution State was Previously Kicking");
      }
      the_logger->Pop();
    }
    if (should_kick && execution_state != KICK) {
      the_logger->LogPrint("Beginning Kick Phase");
    }
    if (should_kick && execution_state == KICK) {
      the_logger->LogPrint("Continuing Kick Phase");
    }
  }
  the_logger->Pop();
  return should_kick;
}


bool TestBallInterception::ShouldIntercept() {
  const float ball_speed = velocity_at_intercept.norm();
  bool is_safe_path = !DoesPathCrossBallRay();

  zone::FieldZone field_zone(zone::FULL_FIELD);
  bool is_valid_intercept =
      field_zone.IsInZone(robot_interception_point, kDefaultSafetyMargin);

  ObstacleFlag obstacles =
    ObstacleFlag::GetAllExceptTeam(world_state_,
                                   *soccer_state_,
                                   our_robot_index_,
                                   our_robot_index_);
  obstacles = obstacles & ~ObstacleFlag::GetBall();
  SafetyMargin safety_margin;

  bool is_collision_free = CollisionFreePath(obstacles, safety_margin,
                           current_robot_pose.translation,
                           robot_interception_point);

  the_logger->LogPrint("Testing Interception Conditions");
  the_logger->Push();
  if (kDebug_) {
    the_logger->LogPrint("Minimum Ball Speed Threshold: %f",
                         kBallVelocityThreshold);
    the_logger->LogPrint("Ball Speed At Intercept: %f",
                         ball_speed);
    the_logger->LogPrint("Upper Ball Speed Threshold: %f",
                         kCatchVelocityThreshold);

    the_logger->LogPrint("approx Intercept Position: (%f, %f)",
                         robot_interception_point.x(),
                         robot_interception_point.y());
    the_logger->AddCircle(ball_intercept_point, kBallRadius,
                          1.0, 1.0, 1.0, 1.0);
    the_logger->AddCircle(robot_interception_point, kRobotRadius,
                          1.0, 1.0, 1.0, 1.0);
    if (!is_valid_intercept) {
      the_logger->LogPrint("Would Intercept Off Field");
    }
    if (!is_collision_free) {
      the_logger->LogPrint("Collision Detected between Robot and Interception");
    }
    if (!is_safe_path) {
      the_logger->LogPrint("Robot Must Pass Through Ball's Trajectory");
    }
  }
  the_logger->Pop();

  return  is_valid_intercept
      && is_collision_free
      && is_safe_path;
}

void TestBallInterception::Run() {
  the_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  current_robot_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  current_robot_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;
  current_ball_pose = world_state_.GetBallPosition().position;
  current_ball_velocity = world_state_.GetBallPosition().velocity;

  if ((current_ball_pose - current_robot_pose.translation).norm() <
      kBallRadius / 100.0) {
    return;
  }

  Eigen::Rotation2Df robot_to_world_rotation(current_robot_pose.angle);

  current_velocity_world =
      robot_to_world_rotation * current_robot_velocity.translation;

  Vector2f desired_norm(cos(target_angle), sin(target_angle));
  Vector2f collision_point =
      current_robot_pose.translation + kInterceptionRadius * desired_norm;

  GetInterceptSolution(collision_point.cast<double>(),
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

  GetTargetAngle(current_ball_pose);
  GetSolutions();
  if (current_ball_velocity.norm() > kBallVelocityThreshold) {
    GetTargetAngle(ball_intercept_point);
    GetSolutions();
  }

  bool should_kick = ShouldKick();
  if (should_kick && execution_state != KICK) {
    execution_state = KICK;
    kick_start_time = world_state_.world_time_;
    kick_angle = target_angle;
    the_logger->LogPrint("Setting Target Kick Angle");
    the_logger->LogPrint("Should Kick");
  } else if (should_kick && execution_state == KICK) {
    execution_state = KICK;
    the_logger->LogPrint("Should Kick");
  } else {
    execution_state = INTERCEPT;
    the_logger->LogPrint("Should Intercept");
  }

  Pose2Df temp_pose;
  temp_pose.angle = target_angle;

  if (execution_state == INTERCEPT) {
    the_logger->LogPrint("Intercepting");
    InterceptionController* controller =
        static_cast<InterceptionController*>((*tactic_list_)
        [TacticIndex::BALL_INTERCEPTION].get());
    controller->SetAngle(target_angle);
    controller->Run();
  }

  if (execution_state == KICK) {
    the_logger->LogPrint("Kicking");

     Kick* controller =
          static_cast<Kick*>(
            (*tactic_list_)[TacticIndex::KICK].get());
    float kick_speed = 5;

    Pose2Df temp_pose;
    temp_pose.translation = current_ball_pose;
    temp_pose.angle = kick_angle;
    controller->SetKickSpeed(kick_speed, false);
    controller->SetGoal(temp_pose);
    controller->Run();

    if (soccer_state_->IsBallKicked())
      the_logger->LogPrint("Ball Kicked");
  }
}

void TestBallInterception::Init() {
  execution_state = INTERCEPT;
  intercept_solution.isInitialized = false;
  hysteresis_count = hysteresis;
}

void TestBallInterception::Reset() {
  execution_state = INTERCEPT;
  hysteresis_count = hysteresis;
}

void TestBallInterception::SetGoal(const Pose2Df& pose) {}
}  // namespace tactics
