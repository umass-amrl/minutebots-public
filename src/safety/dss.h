// Copyright 2017 - 2018 kvedder@umass.edu
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

#include <random>
#include <vector>

#include "math/poses_2d.h"
#include "motion_control/motion_model.h"
#include "state/soccer_state.h"
#include "state/world_state.h"
#include "util/random.h"

#ifndef SRC_SAFETY_DSS_H_
#define SRC_SAFETY_DSS_H_

namespace safety {
class DSS {
 public:
  DSS(const state::WorldState& world_state, state::SoccerState* soccer_state);
  ~DSS();

  void MakeSafe(const motion::MotionModel& our_motion_model,
                const motion::MotionModel& their_motion_model,
                const float control_period);

  bool PillBoxCollide(logger::Logger* current_robot_logger,
                      const Eigen::Vector2f& robot_1,
                      const Eigen::Vector2f& robot_1_current_velocity,
                      const Eigen::Vector2f& robot_1_command_velocity,
                      const motion::MotionModel& robot_1_motion_model,
                      const Eigen::Vector2f& robot_2,
                      const Eigen::Vector2f& robot_2_current_velocity,
                      const Eigen::Vector2f& robot_2_command_velocity,
                      const motion::MotionModel& robot_2_motion_model,
                      const float cap_radius, const float control_period,
                      const float activation_lag, const float margin) const;

  void UpdateState(state::SharedRobotState* current_robot,
                   logger::Logger* current_robot_logger,
                   const state::PositionVelocityState::RobotPositionVelocity&
                       current_world_robot,
                   const motion::MotionModel& current_motion_model,
                   const state::PositionVelocityState::RobotPositionVelocity&
                       other_world_robot,
                   const motion::MotionModel& other_motion_model,
                   const Eigen::Vector2f& other_robot_command_velocity,
                   const float control_period, const float margin);

  float Cost(const Eigen::Vector2f& desired, const Eigen::Vector2f& postulated);

  void GenerateNewVelocities(const Eigen::Vector2f& current_velocity,
                             std::vector<Eigen::Vector2f>* new_velocities,
                             const size_t num_velocities,
                             const float control_period,
                             const motion::MotionModel& motion_model);

 private:
  bool IsTryingToHalt(const Vector2f& robot_frame_desired_robot_velocity,
                      const SSLVisionId& vision_id) const;

  bool AvoidObstacle(state::SharedRobotState* current_robot,
                     const Vector2f& current_world_robot,
                     const Vector2f& other_world_robot,
                     const Eigen::Rotation2Df& world_to_current_robot_transform,
                     const Eigen::Rotation2Df& current_robot_to_world_transform,
                     const Vector2f& robot_frame_current_robot_velocity,
                     const float margin, const float control_period,
                     const motion::MotionModel& motion_model,
                     logger::Logger* logger) const;

  // Number of new velocities to generate.
  const unsigned int kNumNewVelocities = 100;
  // Max move out of obstacle velocity in mm/s
  const float kObstacleExitVelocity = kMaxRobotVelocity;
  // Max velocity the robot can be commanding before considered trying to move
  // in mm^2/s^2
  const float kMovingVelocityStopped = 900;
  util_random::Random random;
  const state::WorldState& world_state_;
  state::SoccerState* soccer_state_;
};
}  // namespace safety
#endif  // SRC_SAFETY_DSS_H_
