// Copyright 2017 - 2018 slane@cs.umass.edu
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

#include "tactics/joystick_controller.h"

#define _USE_MATH_DEFINES

#include <algorithm>
#include <cmath>

#include "constants/constants.h"
#include "datastructures/better_map.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "joystick/joystick.h"
#include "math/math_util.h"
#include "math/poses_2d.h"
#include "state/shared_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"

STANDARD_USINGS;
using datastructures::OptionalValueMutable;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using math_util::AngleMod;
using math_util::Sign;
using state::SharedRobotState;
using state::WorldRobot;
using state::WorldState;
using std::atan2;
using std::cos;
using std::endl;
using std::map;
using std::max;
using std::min;
using std::sin;
using std::abs;
using std::unique_ptr;
using tactics::TacticIndex;
using state::SharedState;

namespace tactics {

JoystickController::JoystickController(const WorldState& world_state,
                                       TacticArray* tactic_list,
                                       SharedState* shared_state,
                                       OurRobotIndex our_robot_index,
                                       state::SoccerState* soccer_state)
    : Tactic(world_state, tactic_list, shared_state, our_robot_index,
             soccer_state) {
  initialized = false;
}

JoystickController::~JoystickController() {
  if (initialized) {
    joystick_->Close();
    delete joystick_;
  }
}

void JoystickController::Run() {
  if (!initialized) {
    initialized = true;
    joystick_ = new joystick::Joystick();
    joystick_->Open(kJoystickID);
  }

  SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);

  Pose2Df commanded_velocity = joystick_->GetVelocity();

  Pose2Df current_velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;
  float current_angle =
      world_state_.GetOurRobotPosition(our_robot_index_).position.angle;

  current_velocity.translation =
      Eigen::Rotation2Df(current_angle) * current_velocity.translation;

  if (joystick_->axes[5] > kKickTriggerThreshold_) {
    // Issue a flat kick command.
    state->flat_kick_set = true;
    state->flat_kick = 5.0;
  } else if (joystick_->axes[2] > kKickTriggerThreshold_) {
    // Issue a chip kick command.
    state->chip_kick_set = true;
    state->chip_kick = 5.0;
  }

  if (commanded_velocity.translation.norm() > kMaxRobotVelocity) {
    commanded_velocity.translation =
        commanded_velocity.translation.normalized() * kMaxRobotVelocity;
  }

  if (fabs(commanded_velocity.angle) > kMaxRobotRotVel) {
    commanded_velocity.angle = Sign(commanded_velocity.angle) * kMaxRobotRotVel;
  }

  Pose2Df commanded_accel(0, 0, 0);

  float delta_max = kMaxRobotAcceleration/kTransmitPeriodSeconds;
  // Limit to set max acceleration of the robot.
  if ((commanded_velocity.translation -
       current_velocity.translation).norm() > delta_max) {
    commanded_accel.translation =
        (commanded_velocity.translation -
         current_velocity.translation).normalized() * kMaxRobotAcceleration;
  } else {
    commanded_accel.translation =
        (commanded_velocity.translation - current_velocity.translation)
        / kTransmitPeriodSeconds;
  }

  float delta_angle_max = kMaxRobotRotAccel/kTransmitPeriodSeconds;
  if (fabs(commanded_velocity.angle - current_velocity.angle) >
      delta_angle_max) {
    commanded_accel.angle = Sign(commanded_velocity.angle -
                                 current_velocity.angle) * kMaxRobotRotAccel;
  } else {
    commanded_accel.angle = (commanded_velocity.angle -
                             current_velocity.angle) /
                             kTransmitPeriodSeconds;
  }

  state->our_robot_index = our_robot_index_;
  state->ssl_vision_id =
      world_state_.GetOurRobotPosition(our_robot_index_).ssl_vision_id;
  state->acceleration_command = commanded_accel;
}

void JoystickController::Reset() {}

void JoystickController::Init() {}

void JoystickController::SetGoal(const pose_2d::Pose2Df& pose) {}
}  // namespace tactics
