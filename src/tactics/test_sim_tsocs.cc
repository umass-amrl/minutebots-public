// Copyright 2018 - 2019 dbalaban@cs.umass.edu
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

#include "tactics/test_sim_tsocs.h"

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

STANDARD_USINGS;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using state::WorldState;
using state::SharedState;
using std::map;
using std::unique_ptr;
using tactics::TacticIndex;

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

extern ScopedFile tsocs_data_fid;

namespace tactics {

TestSimTSOCS::TestSimTSOCS(const WorldState& world_state,
                   TacticArray* tactic_list,
                   SharedState* shared_state,
                   OurRobotIndex our_robot_index,
                   state::SoccerState* soccer_state) :
    Tactic(world_state,
           tactic_list,
           shared_state,
           our_robot_index,
           soccer_state),
    goal_pos(configuration_reader::CONFIG_tsocs_xf,
             configuration_reader::CONFIG_tsocs_yf),
    goal_vel(configuration_reader::CONFIG_tsocs_vxf,
             configuration_reader::CONFIG_tsocs_vyf),
    log_file(nullptr),
    log_number(0) { }

TestSimTSOCS::~TestSimTSOCS() {
  if (log_file) {
    fclose(log_file);
    log_file = nullptr;
  }
}

void TestSimTSOCS::NewLog() {
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

void TestSimTSOCS::Run() {
  TACTIC_VARIABLES;

  log.LogPrint("Testing TSOCS");
  log.Push();


  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;
  Pose2Df current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;

  Eigen::Rotation2Df robot_to_world_rotation(current_pose.angle);
  Vector2f current_velocity_world =
    robot_to_world_rotation * current_velocity.translation;

  log.LogPrint("X distance: %f",
    (current_pose.translation - goal_pos).norm());
  log.LogPrint("V distance: %f",
    (current_velocity_world - goal_vel).norm());

  static double t_state_start = 0;
  const double time_in_state = world_state_.world_time_ - t_state_start;

  static const double kTimeOut = 20.0;
  // First, do state transitions.

  if (((current_pose.translation - goal_pos).norm() < 5.0 &&
      (current_velocity_world - goal_vel).norm() < 50.0)) {
    log.Pop();
    fflush(tsocs_data_fid);
    throw TacticException("Success");
  } else if (time_in_state > kTimeOut) {
    log.Pop();
    fflush(tsocs_data_fid);
    throw TacticException("Timed Out");
  }
  TSOCSController* controller =
      static_cast<TSOCSController*>
      ((*tactic_list_)[TacticIndex::TSOCS].get());
  controller->SetGoal(Pose2Df(goal_orientation, goal_pos),
                      Pose2Df(0.0, goal_vel));
  controller->Run();

  if (controller->Finished()) {
    log.Pop();
    fflush(tsocs_data_fid);
    throw TacticException("Finished TSOCS Execution");
  }
  log.AddPoint(wr_loc, 0, 0, 0, 1);
  log.AddLine(wr_loc, wr_loc + wr_vel, 0, 0, 0, 0.5);
  log.AddPoints(planned_trajectory, 1, 0, 0, 1);
  log.AddPath(planned_trajectory, 1, 0, 0, 0.8);

  log.Pop();
}

void TestSimTSOCS::Reset() { }

void TestSimTSOCS::Init() { }

void TestSimTSOCS::SetGoal(const Pose2Df& pose) { }
}  // namespace tactics
