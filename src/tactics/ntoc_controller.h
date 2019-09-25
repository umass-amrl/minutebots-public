// Copyright 2017 - 2019 dbalaban@cs.umass.edu, slane@cs.umass.edu
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

#include "configuration_reader/reader.h"
#include "math/poses_2d.h"
#include "motion_control/ntoc_2d.h"
#include "motion_control/optimal_control_1d.h"
#include "state/shared_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "tactics/tactic.h"

#ifndef SRC_TACTICS_NTOC_CONTROLLER_H_
#define SRC_TACTICS_NTOC_CONTROLLER_H_

namespace tactics {
class NTOC_Controller : public Tactic {
 public:
  NTOC_Controller(const state::WorldState& world_state,
                  TacticArray* tactic_list, state::SharedState* shared_state,
                  OurRobotIndex our_robot_index,
                  state::SoccerState* soccer_state);

  ~NTOC_Controller() = default;

  const char* Name() const override { return "ntoc_controller"; }
  void Init() override;
  void Run() override;
  void Reset() override;
  void SetGoal(const pose_2d::Pose2Df& pose) override;
  void SetMotionModel(motion::MotionModel motion_model);
  void TurnOnAngleRelaxation();
  void TurnOffAngleRelaxation();
  double GetTotalTime();
  void SetTranslationComplete();
  void SetRotationComplete();
  void TurnOffPDController();
  void TurnOnPDController();

 private:
  bool HasNans();

  void LogPositionTrajectoryToFile(
      const ntoc::ControlSequence2D& linear_control,
      const ntoc::ControlSequence1D& rotational_control,
      const pose_2d::Pose2Df& accel_command);

  pose_2d::Pose2Df goal_;

  const bool kDebug_ = false;
  const bool kDefaultUsePD_ = true;

  motion::MotionModel motion_model_;
  bool relax_angle_;

  bool translation_complete_;
  bool rotation_complete_;
  static constexpr bool kLogPositionTrajectory = false;
  std::fstream position_and_trajectory_file_;
  bool use_pd_;
};
}  // namespace tactics

#endif  // SRC_TACTICS_NTOC_CONTROLLER_H_
