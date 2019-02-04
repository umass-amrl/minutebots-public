// Copyright 2018 afischer@umass.edu
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

#include "tactics/test_ntoc.h"

#include <stdlib.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>


#include <map>
#include <memory>

#include "constants/constants.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "math/poses_2d.h"
#include "state/shared_state.h"
#include "state/soccer_state.h"
#include "state/world_state.h"
#include "tactics/ntoc_controller.h"

STANDARD_USINGS;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using state::WorldState;
using state::SharedState;
using std::map;
using std::unique_ptr;
using tactics::TacticIndex;

static unsigned int seed = 2;
static int num_problems = 0;
static int trials_per_problem = 1;
// ScopedFile ntoc_data_fid("NTOC-data.txt", "a");

#define TACTIC_VARIABLES \
  logger::Logger& log = \
      *(soccer_state_->GetMutableRobotLoggerByOurRobotIndex( \
          our_robot_index_)); \
  const Vector2f& wr_loc = \
      world_state_.GetOurRobotPosition(our_robot_index_).position.translation; \
  const float& wr_angle = \
      world_state_.GetOurRobotPosition(our_robot_index_).position.angle; \
  const Vector2f& wr_vel = \
      world_state_.GetOurRobotPosition(our_robot_index_).velocity.translation; \
  const float& wr_angvel = \
      world_state_.GetOurRobotPosition(our_robot_index_).velocity.angle; \
  do { \
    (void) log; \
    (void) wr_loc; \
    (void) wr_vel; \
    (void) wr_angle; \
    (void) wr_angvel; \
  } while (0)

namespace tactics {

TestNTOC::TestNTOC(const WorldState& world_state,
                   TacticArray* tactic_list,
                   SharedState* shared_state, OurRobotIndex our_robot_index,
                   state::SoccerState* soccer_state) :
    Tactic(world_state,
           tactic_list,
           shared_state,
           our_robot_index,
           soccer_state),
    execution_state(GOTO_WAIT),
    log_file(nullptr),
    log_number(0) { }

TestNTOC::~TestNTOC() {
  if (log_file) {
    fclose(log_file);
    log_file = nullptr;
  }
}

void TestNTOC::Execute() {
  TACTIC_VARIABLES;
  log.LogPrint("On problem %d", num_problems);
  NTOC_Controller* controller =
      static_cast<NTOC_Controller*>
      ((*tactic_list_)[TacticIndex::NTOC].get());
  controller->SetGoal(Pose2Df(0.0, goal_pos));
  controller->Run();

  if (kCollectTSOCSData) {
//     Pose2Df current_velocity =
//         world_state_.GetOurRobotPosition(our_robot_index_).velocity;
//     Pose2Df current_pose =
//         world_state_.GetOurRobotPosition(our_robot_index_).position;
//     Eigen::Rotation2Df robot_to_world_rotation(current_pose.angle);
//     Eigen::Rotation2Df world_to_robot_rotation(-current_pose.angle);
//     Vector2f current_velocity_world =
//         robot_to_world_rotation * current_velocity.translation;
//     Pose2Df observed_pose =
//         world_state_.GetOurRobotPosition(our_robot_index_).observed_pose;

//     fprintf(ntoc_data_fid, "\tCURRENT STATE: %f, %f, %f, %f\n",
//       current_pose.translation.cast<double>().x(),
//       current_pose.translation.cast<double>().y(),
//       current_velocity_world.cast<double>().x(),
//       current_velocity_world.cast<double>().y());
//     fprintf(ntoc_data_fid, "\tOBSERVED POSE: %f, %f\n",
//       observed_pose.translation.cast<double>().x(),
//       observed_pose.translation.cast<double>().y());
//     fprintf(ntoc_data_fid, "\tCURRENT TIME: %f\n", world_state_.world_time_);
  }

  log.AddPoint(wr_loc, 0, 0, 0, 1);
  log.AddLine(wr_loc, wr_loc + wr_vel, 0, 0, 0, 0.5);
  log.AddPoints(planned_trajectory, 1, 0, 0, 1);
  log.AddPath(planned_trajectory, 1, 0, 0, 0.8);

  /*
  if (controller->GetTotalTime() < 1.0 / kTransmitFrequency) {
    log.LogPrint("time to finish = %f, stopping", controller->GetTotalTime());
    execution_state = STOP;
    if (kCollectTSOCSData) {
      collect_ntoc_data = false;
    }
  }
  */
}

void TestNTOC::GotoWait() {
  TACTIC_VARIABLES;
  Tactic* ntoc = (*tactic_list_)[TacticIndex::NTOC].get();
  ntoc->SetGoal(Pose2Df(0.001, wait_pos));
  ntoc->Run();
}

void TestNTOC::Start() {
  TACTIC_VARIABLES;
  log.AddPoint(start_pos, 0, 1, 0, 1);

  // Accelerate in a straight line towards the start position, but do not
  // speed faster than the start speed.
  const Vector2f start_dir = (start_pos - wr_loc).normalized();

  const float speed = start_dir.dot(wr_vel);
  const float start_speed = start_vel.norm();
  const float next_speed = min(speed + kControlPeriodTranslation *
                                   kMaxRobotAcceleration,
                               start_speed);
  const Vector2f vel_next = next_speed * start_dir;
  log.LogPrint("CMD: %f, %f", vel_next.x(), vel_next.y());

  state::SharedRobotState* state =
      shared_state_->GetSharedState(our_robot_index_);
  const Vector2f robot_cmd = Eigen::Rotation2Df(-wr_angle) * vel_next;
  state->our_robot_index = our_robot_index_;
  state->ssl_vision_id =
      world_state_.GetOurRobotPosition(our_robot_index_).ssl_vision_id;
  state->velocity_x = robot_cmd.x();
  state->velocity_y = robot_cmd.y();

  // Compute angular velocity that will get us to 0 orientation
  Pose2Df current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;
  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  Pose2Df desired_velocity(current_pose);
  if (current_pose.angle > 0) {
    if (current_velocity.angle > 0) {
      // we are rotating away from the desired orientation, and we want to
      // rotate back
      desired_velocity.angle = current_velocity.angle - kMaxDeltaRotV;
    } else {
      // the angle we will be at if we decelerate to a screeching halt
      double t_halt = -current_velocity.angle / kMaxRobotRotAccel;
      double angle_halt = current_pose.angle + current_velocity.angle * t_halt
        + 0.5 * kMaxRobotRotAccel * t_halt * t_halt;
      if (angle_halt > 0) {
        // accelerate towards 0
        if (std::abs(current_velocity.angle - kMaxDeltaRotV)
          <= kMaxRobotRotVel) {
          desired_velocity.angle = current_velocity.angle - kMaxDeltaRotV;
        } else {
          desired_velocity.angle = current_velocity.angle;
        }
      } else {
        desired_velocity.angle = current_velocity.angle + kMaxDeltaRotV;
      }
    }
  } else if (current_pose.angle < 0) {
    if (current_velocity.angle < 0) {
      desired_velocity.angle = current_velocity.angle + kMaxDeltaRotV;
    } else {
      // the angle we will be at if we decelerate to a screeching halt
      double t_halt = current_velocity.angle / kMaxRobotRotAccel;
      double angle_halt = current_pose.angle + current_velocity.angle * t_halt
        + 0.5 * kMaxRobotRotAccel * t_halt * t_halt;
      if (angle_halt < 0) {
        // accelerate towards 0
        if (std::abs(current_velocity.angle - kMaxDeltaRotV)
          <= kMaxRobotRotVel) {
          desired_velocity.angle = current_velocity.angle + kMaxDeltaRotV;
        } else {
          desired_velocity.angle = current_velocity.angle;
        }
      } else {
        desired_velocity.angle = current_velocity.angle - kMaxDeltaRotV;
      }
    }
  }
  state->velocity_r = desired_velocity.angle;
}

void TestNTOC::Stop() {
  Tactic* stopped = (*tactic_list_)[TacticIndex::HALT].get();
  stopped->Run();
}

void TestNTOC::Wait() {
  Tactic* stopped = (*tactic_list_)[TacticIndex::HALT].get();
  stopped->Run();
}

void TestNTOC::NewLog() {
  TACTIC_VARIABLES;
  log.LogPrint("Resetting log: %d", log_number);
  if (log_file) {
    fclose(log_file);
  }
  const string log_file_name =
      StringPrintf("tsocs_logs/tsocs_log_%04d.txt", log_number);
  log_file = fopen(log_file_name.c_str(), "w");
  CHECK_NOTNULL(log_file);
  ++log_number;
}

void TestNTOC::GenerateNewProblem() {
  static int trials_this_problem = trials_per_problem;
  static const double max_speed = 2000;
  if (trials_this_problem == trials_per_problem) {
    static const bool special_problems = false;
    if (special_problems) {
      if (num_problems % 2) {
        start_pos = Eigen::Vector2f(2000, -1000);
        start_vel = Eigen::Vector2f(1200, 0);
        goal_pos = Eigen::Vector2f(2000, -2000);
        goal_vel = Eigen::Vector2f(0, 0);
      } else {
        start_pos = Eigen::Vector2f(2000, -1000);
        start_vel = Eigen::Vector2f(600, 0);
        goal_pos = Eigen::Vector2f(2000, -2000);
        goal_vel = Eigen::Vector2f(0, 0);
      }
    } else {
      // Run on the lower right quadrant because the cameras are better there
      start_pos = Eigen::Vector2f(2000, -1000);
      start_vel = Eigen::Vector2f(
        max_speed * static_cast<double>(rand_r(&seed)) / RAND_MAX,
        0);
      goal_pos = Eigen::Vector2f(
        1000 + 2000 * static_cast<double>(rand_r(&seed)) / RAND_MAX,
        -1000 - 1000 * static_cast<double>(rand_r(&seed)) / RAND_MAX);
      goal_vel = Eigen::Vector2f(0, 0);
    }
    trials_this_problem = 1;
    num_problems++;
  } else {
    trials_this_problem++;
  }
  if (kCollectTSOCSData) {
//     fprintf(ntoc_data_fid,
//       "GENERATED PROBLEM: %12f, %12f, %12f, %12f, %12f, %12f, %12f, %12f\n",
//        start_pos.x(), start_pos.y(), start_vel.x(), start_vel.y(),
//        goal_pos.x(), goal_pos.y(), goal_vel.x(), goal_vel.y());
//     fflush(ntoc_data_fid);
  }
}

void TestNTOC::Run() {
  TACTIC_VARIABLES;

  log.LogPrint("Testing NTOC");
  log.Push();


  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  Pose2Df current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;

  static double t_state_start = 0;
  const double time_in_state = world_state_.world_time_ - t_state_start;

  static double t_last_run = 0;
  if (world_state_.world_time_ - t_last_run > 0.1) {
    execution_state = GOTO_WAIT;
    log.LogPrint("Resetting state machine");
  }
  t_last_run = world_state_.world_time_;

  static const double kTimeOut = 5.0;
  // First, do state transitions.
  switch (execution_state) {
    case WAIT: {
      if (time_in_state > kWaitDuration_) {
        log.LogPrint("Generating new problem");
        GenerateNewProblem();
        execution_state = START;
        t_state_start = world_state_.world_time_;
      }
    } break;

    case START: {
      // Check if the robot has passed the start location.
      if ((current_pose.translation - start_pos).norm() < 20.0) {
        execution_state = EXECUTE;
        t_state_start = world_state_.world_time_;
        if (kCollectTSOCSData) {
          collect_ntoc_data = true;
        }
        // NewLog();
      }
    } break;

    case EXECUTE: {
      Eigen::Rotation2Df robot_to_world_rotation(current_pose.angle);
      Vector2f current_velocity_world =
        robot_to_world_rotation * current_velocity.translation;
      log.LogPrint("X distance: %f",
        (current_pose.translation - goal_pos).norm());
      log.LogPrint("V distance: %f",
        (current_velocity_world - goal_vel).norm());
      if (((current_pose.translation - goal_pos).norm() < 5.0 &&
          (current_velocity_world - goal_vel).norm() < 50.0) ||
          time_in_state > kTimeOut) {
        execution_state = STOP;
        t_state_start = world_state_.world_time_;
        if (kCollectTSOCSData) {
          collect_ntoc_data = false;
        }
      }
    } break;

    case STOP: {
      if (current_velocity.translation.norm() < 50.0) {
        execution_state = GOTO_WAIT;
        t_state_start = world_state_.world_time_;
      }
    } break;

    case GOTO_WAIT: {
      log.LogPrint("Dist: %f %f %f Speed: %f",
                    wait_pos.x(), wait_pos.y(),
                    (current_pose.translation - wait_pos).norm(),
                    current_velocity.translation.norm());
      if (((current_pose.translation - wait_pos).norm() < 5.0 &&
          current_velocity.translation.norm() < 50.0)) {
        execution_state = WAIT;
        t_state_start = world_state_.world_time_;
      }
    } break;
  }

  // Next, execute every state.
  switch (execution_state) {
    case WAIT: {
      log.LogPrint("WAIT");
      log.Push();
      Wait();
    } break;
    case START: {
      log.LogPrint("START");
      log.Push();
      Start();
    } break;
    case EXECUTE: {
      log.LogPrint("EXECUTE");
      log.Push();
      Execute();
    } break;
    case STOP: {
      log.LogPrint("STOP");
      log.Push();
      Stop();
    } break;
    case GOTO_WAIT: {
      log.LogPrint("GOTO_WAIT");
      log.Push();
      GotoWait();
    } break;
  }
  log.Pop();
}

void TestNTOC::Reset() {
  execution_state = START;
  previous_state = WAIT;
  current_goal_pos = start_pos;
  current_goal_vel = start_vel;
  isFirstRun = true;

  wait_start_time_ = world_state_.world_time_;
  if (wait_start_time_ < GetWallTime()) {
    wait_start_time_ = GetWallTime();
  }
}

void TestNTOC::Init() {
  execution_state = START;
  previous_state = WAIT;
  current_goal_pos = start_pos;
  current_goal_vel = start_vel;
  isFirstRun = true;

  wait_start_time_ = world_state_.world_time_;
  if (wait_start_time_ < GetWallTime()) {
    wait_start_time_ = GetWallTime();
  }
}

void TestNTOC::SetGoal(const Pose2Df& pose) { }
}  // namespace tactics
