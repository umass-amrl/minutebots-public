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

#include <memory>
#include <vector>

#include "logging/logger.h"
#include "math/poses_2d.h"
#include "state/shared_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "tactics/tactic.h"

#ifndef SRC_TACTICS_TRIANGLE_H_
#define SRC_TACTICS_TRIANGLE_H_

namespace tactics {
class Triangle : public Tactic {
 public:
  Triangle(const state::WorldState& world_state,
           TacticArray* tactic_list,
           state::SharedState* shared_state, OurRobotIndex our_robot_index,
           state::SoccerState* soccer_state);

  const char* Name() const override { return "triangle"; }
  void Init() override;
  void Run() override;
  void SetGoal(const pose_2d::Pose2Df& pose) override;
  void Reset() override;

 private:
  enum TriangleState { WAIT, TURN, FORWARD };

  enum GoalPose { POSEA, POSEB, POSEC };

  TriangleState execution_state;
  GoalPose goal_pose;

  pose_2d::Pose2Df poseA;
  pose_2d::Pose2Df poseB;
  pose_2d::Pose2Df poseC;
  pose_2d::Pose2Df goal;

  double wait_start_time_;
  const double kWaitDuration_ = 5 / kTransmitFrequency;
};
}  // namespace tactics

#endif  // SRC_TACTICS_TRIANGLE_H_
