// Copyright 2016 - 2018 kvedder@umass.edu
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

#ifndef SRC_SIM_WORLDSTATE_H_
#define SRC_SIM_WORLDSTATE_H_

#include <glog/logging.h>
#include <vector>

#include "radio_protocol_wrapper.pb.h"

#include "sim/objects/ball.h"
#include "sim/objects/robot.h"

namespace simulator {

class WorldState {
 public:
  WorldState() = delete;
  WorldState(const int num_robots, const float time_slice);
  WorldState(const std::vector<Eigen::Vector2f>& positions,
             const float time_slice);
  ~WorldState() = default;

  void Update(const RadioProtocolWrapper& velocity_command);

  const std::vector<Robot>& GetRobots() const;

  const Ball& GetBall() const;

 private:
  std::vector<Robot> robots;
  Ball ball;
};

}  // namespace simulator
#endif  // SRC_SIM_WORLDSTATE_H_
