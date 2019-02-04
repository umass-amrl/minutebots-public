// Copyright 2017 - 2018 slane@cs.umass.edu, kvedder@umass.edu
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

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "constants/constants.h"
#include "state/shared_state.h"
#include "state/world_state.h"
#include "tactics/tactic.h"
#include "tactics/tactic_index.h"

#include "logging/logger.h"
#include "radio_protocol_wrapper.pb.h"

#ifndef SRC_STATE_SOCCER_ROBOT_H_
#define SRC_STATE_SOCCER_ROBOT_H_

namespace state {
class SoccerState;

// This class houses the tactics as well as information for each robot. Once
// constructed, its robot information may be copied, but its actual tactics may
// not be. This is to ensure that the datastructure containing these robots
// always has slots that are valid to be written to, as if we std::move the
// tactics, the moved from object will not be valid to operate on any further,
// as per the C++ 11 spec and we will thus have to reinitialize, which includes
// reinitializing each tactic.
//
// Data that is not copied using assignment:
//  - Our Robot Index
//  - Tactic list
//
// Data that is not copied using copy constructor:
//  - Tactic list
class SoccerRobot {
 public:
  // No default, as we need a reference to various states.
  SoccerRobot() = delete;
  SoccerRobot(const WorldState& world_state,
              const tactics::TacticIndex default_tactic,
              SoccerState* soccer_state, SharedState* shared_state,
              const OurRobotIndex our_robot_index);
  // Copies only the data about the robot itself, not the underlying tactics
  // list.
  explicit SoccerRobot(const SoccerRobot& other);

  // No std::move allowed.
  explicit SoccerRobot(SoccerRobot&& other) = delete;

  // Copies only the data about the robot itself, not the underlying tactics
  // list.
  SoccerRobot& operator=(const SoccerRobot& other);

  // No std::move allowed.
  SoccerRobot& operator=(SoccerRobot&& other) = delete;

  ~SoccerRobot();

  bool operator<(const SoccerRobot& other);

  void SetRobotData(SSLVisionId ssl_vision_id, OurRobotIndex our_robot_index);

  void ClearRobotData();

  tactics::TacticIndex GetTacticByName(const std::string& name) const;

  // Convienience function to call set goal for the currently active tactic
  void SetGoal(pose_2d::Pose2Df goal_pose) const;

  const pose_2d::Pose2Df GetGoal() const;

  const float GetCost() const;

  const WorldState& world_state_;
  SSLVisionId ssl_vision_id_;
  tactics::TacticIndex current_tactic_;
  tactics::TacticIndex previous_tactic_;
  OurRobotIndex our_robot_index_;
  SoccerState* soccer_state_;
  SharedRobotState previous_command_;
  bool enabled_;
  tactics::TacticArray tactic_list_;
  SharedState* shared_state;
  logger::Logger bot_logger;

  void RunTactic();
  // Constructs the uniquely owned tactic vector, which will never be copied.
  //
  // Should only be called on startup. Should only be called once.
  void InitTactics();
};
}  // namespace state

#endif  // SRC_STATE_SOCCER_ROBOT_H_
