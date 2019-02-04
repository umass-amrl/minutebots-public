// Copyright 2017-2018 kvedder@umass.edu
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
#include "tactics/halt.h"

#include "constants/constants.h"
#include "math/math_util.h"
#include "math/geometry.h"
#include "safety/dss2.h"

STANDARD_USINGS;
using state::WorldState;
using state::SoccerState;
using std::map;
using std::unique_ptr;
using tactics::TacticIndex;
using state::SharedState;
using state::SharedRobotState;
using pose_2d::Pose2Df;
using math_util::Sign;
using geometry::SafeVectorNorm;

namespace tactics {
Halt::Halt(const WorldState& world_state,
           TacticArray* tactic_list,
           SharedState* shared_state, OurRobotIndex our_robot_index,
           state::SoccerState* soccer_state)
    : Tactic(world_state, tactic_list, shared_state, our_robot_index,
             soccer_state) {
}

Halt::~Halt() {}

void Halt::Init() {
  translation_complete_ = false;
  angle_complete_ = false;
  started_ = false;
  started_velocity_.translation = {0, 0};
  started_velocity_.angle = 0;
}

void Halt::Run() {
  safety::DSS2::SetObstacleFlag(our_robot_index_,
                                obstacle::ObstacleFlag::GetEmpty());
  logger::Logger* logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  logger->LogPrint("Stopped");
  logger->Push();

  started_velocity_ =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity;
  float robot_angle =
      world_state_.GetOurRobotPosition(our_robot_index_).position.angle;

  started_velocity_.translation =
      Eigen::Rotation2Df(robot_angle) * started_velocity_.translation;

  logger->LogPrint("Started Velocity: %f,%f",
                   started_velocity_.translation.x(),
                   started_velocity_.translation.y());
  logger->Pop();
  if (SafeVectorNorm(started_velocity_.translation) < kEpsilon) {
    translation_complete_ = true;
  } else {
    translation_complete_ = false;
  }
  if (fabs(started_velocity_.angle) < kEpsilon) {
    angle_complete_ = true;
  } else {
    angle_complete_ = false;
  }

  Pose2Df desired_acceleration;
  desired_acceleration.translation = {0, 0};
  desired_acceleration.angle = 0;
  // Motion Profiling to avoid attempting to instantly stop.
  if (!translation_complete_) {
    if (started_velocity_.translation.norm()
        > kMaxRobotAcceleration * kTransmitPeriodSeconds) {
      desired_acceleration.translation =
          -kMaxRobotAcceleration * started_velocity_.translation.normalized();
    } else {
      desired_acceleration.translation =
          -started_velocity_.translation/kTransmitPeriodSeconds;
    }
  }
  if (!angle_complete_) {
    if (std::abs(started_velocity_.angle)
        > kMaxRobotAcceleration * kTransmitPeriodSeconds) {
      desired_acceleration.angle =
          -Sign(started_velocity_.angle) * kMaxRobotAcceleration;
    } else {
      desired_acceleration.angle =
          -started_velocity_.angle/kTransmitPeriodSeconds;
    }
  }

  // Attempt to reach the desired velocity
  SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);
  state->our_robot_index = our_robot_index_;
  state->ssl_vision_id =
      world_state_.GetOurRobotPosition(our_robot_index_).ssl_vision_id;
  state->acceleration_command = desired_acceleration;
  state->should_stop_linear = translation_complete_;
  state->should_stop_angular = angle_complete_;
}

void Halt::Reset() {
  translation_complete_ = false;
  angle_complete_ = false;
  started_ = false;
  started_velocity_.translation = {0, 0};
  started_velocity_.angle = 0;
}

void Halt::SetGoal(const pose_2d::Pose2Df& pose) {}

}  // namespace tactics
