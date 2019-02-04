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

#include "tactics/dive_controller.h"

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

namespace tactics {

DiveController::DiveController(const WorldState& world_state,
                                 TacticArray* tactic_list,
                                 SharedState* shared_state,
                                 OurRobotIndex our_robot_index,
                                 state::SoccerState* soccer_state)
    : Tactic(world_state, tactic_list, shared_state, our_robot_index,
             soccer_state) {}

DiveController::~DiveController() {}

void DiveController::Run() {
  logger::Logger* the_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);

  the_logger->LogPrint("Dive Controller");
  the_logger->Push();

  if (kDebug_) {
    the_logger->LogPrint("Current World Time: %f", world_state_.world_time_);
  }

  Pose2Df current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;
  Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).position;

  if (std::isnan(current_pose.angle)
      || std::isnan(current_pose.translation.x())
      || std::isnan(current_pose.translation.y())
      || std::isnan(current_velocity.angle)
      || std::isnan(current_velocity.translation.x())
      || std::isnan(current_velocity.translation.y())
      || std::isnan(goal_.translation.x())
      || std::isnan(goal_.translation.y())
      || world_state_.GetOurRobotPosition(our_robot_index_).confidence == 0) {
    LOG(WARNING) << "CONTROLLER PASSED NAN VALUES";

    SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);

    state->our_robot_index = our_robot_index_;
    state->ssl_vision_id =
        world_state_.GetOurRobotPosition(our_robot_index_).ssl_vision_id;
    state->velocity_x = 0.0;
    state->velocity_y = 0.0;
    state->velocity_r = 0.0;
    return;
  }

  Vector2f delta_x = goal_.translation - current_pose.translation;

  Eigen::Rotation2Df robot_to_world_rotation(current_pose.angle);

  Vector2f current_velocity_world =
      robot_to_world_rotation * current_velocity.translation;

  Vector2f accel_t =
      (2 * (delta_x - (current_velocity_world * goal_time_))) / Sq(goal_time_);


  SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);

  state->our_robot_index = our_robot_index_;
  state->ssl_vision_id =
      world_state_.GetOurRobotPosition(our_robot_index_).ssl_vision_id;
  state->acceleration_command.translation = accel_t;
  state->acceleration_command.angle = 0.0f;

  the_logger->Pop();
}

void DiveController::Reset() {
  goal_.translation = {0, 0};
  goal_.angle = 0;
  goal_time_ = 0;
}

void DiveController::Init() {
  goal_.translation = {0, 0};
  goal_.angle = 0;
  goal_time_ = 0;
}

void DiveController::SetGoal(const pose_2d::Pose2Df& pose) { goal_ = pose; }

void DiveController::SetTime(const float time) {
  goal_time_ = time;
}

}  // namespace tactics
