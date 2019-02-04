// Copyright 2017-2018 slane@cs.umass.edu, kvedder@umass.edu
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
#include <fstream>
#include <map>
#include <vector>

#ifndef SRC_STATE_SHARED_STATE_H_
#define SRC_STATE_SHARED_STATE_H_

#include "constants/constants.h"
#include "math/poses_2d.h"
#include "radio_protocol_wrapper.pb.h"
#include "state/position_velocity_state.h"

namespace logger {
class Logger;
}  // namespace logger

namespace state {

class WorldState;

struct SharedRobotState {
  SharedRobotState() = delete;
  SharedRobotState(const OurRobotIndex our_robot_index,
                   const SSLVisionId ssl_vision_id);

  pose_2d::Pose2Df GetDesiredVelocity() {
    pose_2d::Pose2Df velocity;
    velocity.translation.x() = velocity_x;
    velocity.translation.y() = velocity_y;
    velocity.angle = velocity_r;
    return velocity;
  }

  bool enabled = false;

  // Robot index for internal use.
  OurRobotIndex our_robot_index;

  // SSL Vision ID for commanding hardware.
  SSLVisionId ssl_vision_id;

  // Desired forward drive velocity in mm / second.
  float velocity_x;

  // Desired sideways left drive velocity in mm / second.
  float velocity_y;

  // Desired counter-clockwise angular velocity in radians / second.
  float velocity_r;

  // Acceleration commanded in world frame by controllers and modified by DSS.
  pose_2d::Pose2Df acceleration_command;

  // Maximum velocities over the entire control plan, world frame
  pose_2d::Pose2Df max_velocity;

  bool dss_changed_command;

  // Desired flat kick speed, in meters / second.
  // If this optional field is missing, then no flat kick command should be
  // sent.
  float flat_kick;

  bool flat_kick_set;

  // Desired chip kick distance, in meters.
  float chip_kick;

  bool chip_kick_set;

  // Desired dribbler spin, from -1 to +1, where -1 is the maximum reverse-spin
  // that can be imparted to the ball, and +1 is the maximum forward-spin
  // that can be imparted to the ball.
  float dribbler_spin;

  bool dribbler_set;

  // These should be set to true if a 0 velocity should be commanded this
  // timestep
  bool should_stop_linear;
  bool should_stop_angular;

  double cmd_time;

  // Sets the distance to chipkick in mm.
  void SetChipKickDistance(const float distance) {
    flat_kick_set = true;
    // TODO(kvedder): Replace with proper second order polynomial.
    flat_kick = distance;
  }

  // Sets the distance to flat kick in mm.
  void SetFlatKickDistance(const float distance) {
    chip_kick_set = true;
    // TODO(kvedder): Replace with proper second order polynomial.
    chip_kick = distance;
  }
};

class SharedState {
 public:
  SharedState();
  ~SharedState();
  RadioProtocolWrapper ConvertToRadioWrapper(const WorldState& world_state,
                                             logger::Logger* logger);

  void ConvertAccelerationsToCommand(pose_2d::Pose2Df acceleration);

  void Init();

  // Resets all commands to their natural "zero" state, e.g. zero velocity with
  // no kick commands set.
  void ResetCommands(const state::PositionVelocityState& pvs);

  // Returns a pointer to a robot's shared state given its index.
  SharedRobotState* GetSharedState(OurRobotIndex our_robot_index);

  // Returns a pointer to a robot's shared state given its ssl vision id.
  // Returns null if it is not found
  // Note: This is much slower than using the index and the other should
  // be used if at all possible. This should only be used when there is a chance
  // that the indices are different such as in the Kalman Update Thread.
  SharedRobotState* GetSharedStateByID(SSLVisionId ssl_vision_id);

  // Returns the number of robots currently in the shared state list
  int GetNumRobots();

  std::vector<SharedRobotState>* GetMutableSharedStates();
  const std::vector<SharedRobotState>& GetSharedStatesRef() const;

  void SetCommandTime(double time);
  double GetCommandTime() const;
  void SetPass(const OurRobotIndex& pass_target,
               const Eigen::Vector2f& pass_location);
  void SetPassShot();
  void ClearPass();
  bool IsPass();
  OurRobotIndex GetPassTarget();
  Eigen::Vector2f GetPassLocation();
  bool GetPassShot() const;

  // Returns the command for the given robot if it is present in the list
  // Returns 0s otherwise.
  pose_2d::Pose2Dd GetCommandByID(SSLVisionId ssl_vision_id) const;

 private:
  pose_2d::Pose2Df GetNextCommand(const WorldState& world_state,
                                  const pose_2d::Pose2Df& acceleration,
                                  const pose_2d::Pose2Df& max_velocity,
                                  OurRobotIndex our_robot_index,
                                  float delta_t_translation,
                                  float delta_t_rotation,
                                  logger::Logger* logger);

  void LogCommandedVelocityToFile(const WorldState& world_state,
                                  const SSLVisionId& ssl_vision_id,
                                  const pose_2d::Pose2Df& commanded_velocity);

  std::vector<SharedRobotState> shared_state_list;
  bool pass_set_;
  OurRobotIndex pass_target_;
  Eigen::Vector2f pass_location_;
  bool pass_shot_;
  double cmd_time_;
  static constexpr bool kLogCommandedVelocity = false;
};

}  // namespace state
#endif  // SRC_STATE_SHARED_STATE_H_
