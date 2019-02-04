// Copyright 2017-2018 dbalaban@cs.umass.edu
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

#include "math/poses_2d.h"
#include "motion_control/ball_interception2.h"
#include "motion_control/motion_control_structures.h"
#include "motion_control/ntoc_2d.h"
#include "motion_control/optimal_control_1d.h"
#include "motion_control/tsocs_old.h"
#include "state/shared_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "tactics/tactic.h"

#ifndef SRC_TACTICS_BALL_INTERCEPTION_H_
#define SRC_TACTICS_BALL_INTERCEPTION_H_

namespace tactics {
class InterceptionController : public Tactic {
 public:
  InterceptionController(const state::WorldState& world_state,
                         TacticArray* tactic_list,
                         state::SharedState* shared_state,
                         OurRobotIndex our_robot_index,
                         state::SoccerState* soccer_state);

  const char* Name() const override { return "ball_interception"; }
  void Init() override;
  // just runs RunDontCommand() then Command()
  void Run() override;
  void Reset() override;
  void SetGoal(const pose_2d::Pose2Df& pose) override;
  void SetAngle(float angle);
  void SetMotionModel(motion::MotionModel motion_model);
  // runs ball interception without sending commands to the robots. This will
  // allow methods such as GetInterceptionPoint, GetInterceptionVelocity, and
  // GetInterceptionTime to be called.
  void RunDontCommand();
  // Commands the solution that was found from RunDontCommand. Run this after
  // RunDontCommand if you have decided that you want to use the solution that
  // ball interception found.
  void Command();
  double GetInterceptionTime();
  Vector2d GetInterceptionPoint();
  Vector2d GetInterceptionVelocity();
  SolutionParameters GetSolution();
  void SetSolution(SolutionParameters params);
  void SetOffset(Vector2f offset);
  void TurnOnAngleRelaxation();
  void TurnOffAngleRelaxation();

  Vector2d final_robot_pos_;
  Vector2d final_robot_vel_;
  Vector2d final_ball_pos_;
  Vector2d final_ball_vel_;
  bool match_velocity_ = true;

 private:
  void LogControlSequence(logger::Logger* the_log,
                          std::vector<ntoc::ControlPhase2D> control);

  float target_angle;
  SolutionParameters params_;
  motion::MotionModel motion_model_;
  Vector2f offset_;
  double interception_time_;
  bool relax_angle_;
  static const int kDebug = false;
};
}  // namespace tactics

#endif  // SRC_TACTICS_BALL_INTERCEPTION_H_
