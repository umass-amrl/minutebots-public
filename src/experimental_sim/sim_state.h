// Copyright 2016 - 2018 jaholtz@cs.umass.edu
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

#ifndef SRC_EXPERIMENTAL_SIM_SIM_STATE_H_
#define SRC_EXPERIMENTAL_SIM_SIM_STATE_H_

#include <glog/logging.h>
#include <vector>

#include "radio_protocol_wrapper.pb.h"

#include "experimental_sim/objects/ball.h"
#include "experimental_sim/objects/robot.h"

namespace experimental_simulator {

class SimState {
 public:
  SimState() = delete;
  SimState(const int& num_robots,
          const float& friction_coefficient,
          const float& time_slice);
  SimState(const std::vector<pose_2d::Pose2Df>& positions,
          const float& friction_coefficient,
          const float& time_slice);
  ~SimState() = default;

  void Update(const RadioProtocolWrapper& velocity_command,
              const float& step_time);

  const std::vector<Robot>& GetRobots() const;

  const Ball& GetBall() const;

  void SetBallPose(const Eigen::Vector2f& pose);

  void SetBallVelocity(const Eigen::Vector2f& velocity);

  const float& GetFriction() const;

  void SetBall(Ball new_ball);

  void SetBallAccel(const float& ball_accel);

 private:
  std::vector<Robot> robots;
  Ball ball;
  const float kFrictionCoefficient_;
};

}  // namespace experimental_simulator
#endif  // SRC_EXPERIMENTAL_SIM_SIM_STATE_H_
