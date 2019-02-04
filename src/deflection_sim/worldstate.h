// Copyright 2016 - 2017 kvedder@umass.edu
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

#include <glog/logging.h>
#include <vector>

#include "deflection_sim/objects/ball.h"
#include "deflection_sim/objects/robot.h"

#ifndef SRC_DEFLECTION_SIM_WORLDSTATE_H_
#define SRC_DEFLECTION_SIM_WORLDSTATE_H_

namespace simulator {

class WorldState {
 public:
  WorldState(std::vector<Robot> robots, Ball ball);
  ~WorldState();

  void Update(const RadioProtocolWrapper& velocity_command);

  const std::vector<Robot>& GetRobots() const;

  const Ball& GetBall() const;

 private:
  std::vector<Robot> robots;
  Ball ball;
};

}  // namespace simulator
#endif  // SRC_DEFLECTION_SIM_WORLDSTATE_H_"
