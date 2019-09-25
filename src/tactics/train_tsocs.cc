// Copyright 2019 dbalaban@cs.umass.edu
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

#include "tactics/train_tsocs.h"

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
#include "tactics/tsocs_controller.h"
#include "debugging/tactic_exception.h"

STANDARD_USINGS;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using state::WorldState;
using state::SharedState;
using std::map;
using std::unique_ptr;
using tactics::TacticIndex;

extern ScopedFile tsocs_data_fid;

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

TrainTSOCS::TrainTSOCS(const WorldState& world_state,
                       TacticArray* tactic_list,
                       SharedState* shared_state,
                       OurRobotIndex our_robot_index,
                       state::SoccerState* soccer_state) :
    Tactic(world_state,
           tactic_list,
           shared_state,
           our_robot_index,
           soccer_state),
    execution_state(GOTO_WAIT),
    log_file(nullptr),
    log_number(0) { }

TrainTSOCS::~TrainTSOCS() {
  if (log_file) {
    fclose(log_file);
    log_file = nullptr;
  }
}

void TrainTSOCS::Execute() {
  TACTIC_VARIABLES;
  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  Pose2Df current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;
  Eigen::Rotation2Df robot_to_world_rotation(current_pose.angle);
  Vector2f current_velocity_world =
    robot_to_world_rotation * current_velocity.translation;
  log.LogPrint("X distance: %f",
    (current_pose.translation - prob_goal_pos).norm());
  log.LogPrint("V distance: %f",
    (current_velocity_world - prob_goal_vel).norm());
  log.LogPrint("On problem %d", problem_counter);
  log.LogPrint("On trial %d", trials_this_problem);
  TSOCSController* controller =
      static_cast<TSOCSController*>
      ((*tactic_list_)[TacticIndex::TSOCS].get());
  controller->SetGoal(Pose2Df(goal_orientation, prob_goal_pos),
                      Pose2Df(0.0, prob_goal_vel));
  controller->Run();
  if (controller->Finished()) {
    execution_state = GOTO_WAIT;
    log.LogPrint("Finished: setting execution state to stop");
  }
  log.AddPoint(wr_loc, 0, 0, 0, 1);
  log.AddLine(wr_loc, wr_loc + wr_vel, 0, 0, 0, 0.5);
  log.AddPoints(planned_trajectory, 1, 0, 0, 1);
  log.AddPath(planned_trajectory, 1, 0, 0, 0.8);
}

void TrainTSOCS::GotoWait() {
  TACTIC_VARIABLES;
  Tactic* ntoc = (*tactic_list_)[TacticIndex::NTOC].get();
  ntoc->SetGoal(Pose2Df(0.0, wait_pos));
  ntoc->Run();
}

void TrainTSOCS::Start() {
  TACTIC_VARIABLES;
  log.AddPoint(start_pos, 0, 1, 0, 1);

  const Vector2f start_dir = (start_pos - wr_loc).normalized();

  const float speed = start_dir.dot(wr_vel);
  const float start_speed = start_vel.norm();
  Pose2Df accel(0, 0, 0);
  if (speed < start_speed) {
    accel.translation = kDefaultRobotAcceleration
      * (Eigen::Rotation2Df(-wr_angle) * start_dir);
  } else if (speed > start_speed + kMaxDeltaV) {
    accel.translation = -kDefaultRobotAcceleration * wr_vel.normalized();
  } else {
    accel.translation = Eigen::Vector2f(0, 0);
  }

  // Compute angular velocity that will get us to 0 orientation
  Pose2Df current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;
  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  if (current_pose.angle > 0) {
    if (current_velocity.angle > 0) {
      // we are rotating away from the desired orientation, and we want to
      // rotate back
          accel.angle = -kMaxRobotRotAccel;
    } else {
      // the angle we will be at if we decelerate to a screeching halt
      double t_halt = -current_velocity.angle / kMaxRobotRotAccel;
      double angle_halt = current_pose.angle + current_velocity.angle * t_halt
        + 0.5 * kMaxRobotRotAccel * t_halt * t_halt;
      if (angle_halt > 0) {
        // accelerate towards 0
        if (std::abs(current_velocity.angle - kMaxDeltaRotV)
          <= kMaxRobotRotVel) {
          accel.angle = -kMaxRobotRotAccel;
        } else {
          accel.angle = 0;
        }
      } else {
          accel.angle = kMaxRobotRotAccel;
      }
    }
  } else if (current_pose.angle < 0) {
    if (current_velocity.angle < 0) {
          accel.angle = kMaxRobotRotAccel;
    } else {
      // the angle we will be at if we decelerate to a screeching halt
      double t_halt = current_velocity.angle / kMaxRobotRotAccel;
      double angle_halt = current_pose.angle + current_velocity.angle * t_halt
        + 0.5 * kMaxRobotRotAccel * t_halt * t_halt;
      if (angle_halt < 0) {
        // accelerate towards 0
        if (std::abs(current_velocity.angle - kMaxDeltaRotV)
          <= kMaxRobotRotVel) {
          accel.angle = kMaxRobotRotAccel;
        } else {
          accel.angle = -0;
        }
      } else {
          accel.angle = -kMaxRobotRotAccel;
      }
    }
  }
  state::SharedRobotState* state =
      shared_state_->GetSharedState(our_robot_index_);
  state->our_robot_index = our_robot_index_;
  state->ssl_vision_id =
      world_state_.GetOurRobotPosition(our_robot_index_).ssl_vision_id;
  state->acceleration_command = accel;
  log.LogPrint("acceleration command: %f, %f, %f",
               accel.translation.x(), accel.translation.y(), accel.angle);
}

void TrainTSOCS::Stop() {
  Tactic* halt = (*tactic_list_)[TacticIndex::HALT].get();
  halt->Run();
}

void TrainTSOCS::Wait() {
  Stop();
}

void TrainTSOCS::NewLog() {
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

void TrainTSOCS::GetProblem() {
  if (isFirstRun) {
    std::ifstream fs(problem_filename.c_str());
    std:: string str;
    while (std::getline(fs, str)) {
      if (str.size() > 0) {
        problems.push_back(str);
      }
    }
    trials_this_problem = trials_per_problem;
    problem_counter = -1;
    isFirstRun = false;
  }

  if (trials_this_problem == trials_per_problem) {
    problem_counter++;
    trials_this_problem = 0;

    if (problem_counter >= problems.size()) {
      throw TacticException("Training Complete");
    }

    std::stringstream prob(problems.at(problem_counter));
    std::vector<string> parsed;
    std::string segment;
    while (std::getline(prob, segment, ',')) {
      parsed.push_back(segment);
    }

    prob_goal_pos.x() = std::stof(parsed.at(0).c_str());
    prob_goal_pos.y() = std::stof(parsed.at(1).c_str());
    prob_goal_vel.x() = std::stof(parsed.at(2).c_str());
    prob_goal_vel.y() = std::stof(parsed.at(3).c_str());
  }

  trials_this_problem++;

  if (kCollectTSOCSData) {
    fprintf(tsocs_data_fid,
      "GENERATED PROBLEM: %12f, %12f, %12f, %12f, %12f, %12f, %12f, %12f\n",
       start_pos.x(), start_pos.y(), start_vel.x(), start_vel.y(),
       prob_goal_pos.x(), prob_goal_pos.y(),
       prob_goal_vel.x(), prob_goal_vel.y());
    fflush(tsocs_data_fid);
  }
}

void TrainTSOCS::Run() {
  TACTIC_VARIABLES;

  log.LogPrint("Training TSOCS");
  log.Push();

  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  Pose2Df current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;

  static double t_state_start = 0;
  const double time_in_state = world_state_.world_time_ - t_state_start;

  static const double kTimeOut = 20.0;
  // First, do state transitions.
  switch (execution_state) {
    case WAIT: {
      if (time_in_state > kWaitDuration_) {
        log.LogPrint("Generating new problem");
        GetProblem();
        execution_state = START;
        t_state_start = world_state_.world_time_;
      }
    } break;

    case START: {
      // Check if the robot has passed the start location.
      if (fabs((current_pose.translation - start_pos).x()) < 30.0) {
        execution_state = EXECUTE;
        TSOCSController* controller =
            static_cast<TSOCSController*>
            ((*tactic_list_)[TacticIndex::TSOCS].get());
        controller->Reset();
        controller->SetMotionModel(
          motion::MotionModel(kDefaultRobotAcceleration,
                              kDefaultRobotVelocity));
        t_state_start = world_state_.world_time_;
        // NewLog();
      }
    } break;

    case EXECUTE: {
      Eigen::Rotation2Df robot_to_world_rotation(current_pose.angle);
      Vector2f current_velocity_world =
        robot_to_world_rotation * current_velocity.translation;
      if (((current_pose.translation - prob_goal_pos).norm() < 5.0 &&
          (current_velocity_world - prob_goal_vel).norm() < 50.0) ||
          time_in_state > kTimeOut) {
        log.LogPrint("Timed out: setting state to GOTO_WAIT");
        execution_state = GOTO_WAIT;
      fflush(tsocs_data_fid);
        t_state_start = world_state_.world_time_;
      }
    } break;

    case STOP: {
      if (current_velocity.translation.norm() < 10.0) {
        execution_state = GOTO_WAIT;
        t_state_start = world_state_.world_time_;
      }
    } break;

    case GOTO_WAIT: {
      log.LogPrint("Dist: %f %f %f Speed: %f",
                    wait_pos.x(), wait_pos.y(),
                    (current_pose.translation - wait_pos).norm(),
                    current_velocity.translation.norm());
      if (((current_pose.translation - wait_pos).norm() < 10.0 &&
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

void TrainTSOCS::Reset() {
  execution_state = GOTO_WAIT;
  previous_state = GOTO_WAIT;
  isFirstRun = true;

  wait_start_time_ = world_state_.world_time_;
  if (wait_start_time_ < GetWallTime()) {
    wait_start_time_ = GetWallTime();
  }
}

void TrainTSOCS::Init() {
  execution_state = GOTO_WAIT;
  previous_state = GOTO_WAIT;
  isFirstRun = true;

  wait_start_time_ = world_state_.world_time_;
  if (wait_start_time_ < GetWallTime()) {
    wait_start_time_ = GetWallTime();
  }
}

void TrainTSOCS::SetGoal(const Pose2Df& pose) { }
}  // namespace tactics

