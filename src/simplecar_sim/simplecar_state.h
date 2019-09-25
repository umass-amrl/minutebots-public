// Copyright 2019 jaholtz@cs.umass.edu
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

#ifndef SRC_SIMPLECAR_SIM_SIMPLECAR_STATE_H_
#define SRC_SIMPLECAR_SIM_SIMPLECAR_STATE_H_

#include <glog/logging.h>
#include <vector>

#include "simplecar_sim/simplecar.h"

namespace simplecar {

struct SimplecarCommand {
  int car_id;
  float desired_speed;
  float desired_steer;
};

class SimplecarState {
 public:
  SimplecarState();

  explicit SimplecarState(const float& time_slice);

  SimplecarState(const std::vector<pose_2d::Pose2Df>& positions,
                 const std::vector<float>& speeds,
                 const float& time_slice);

  ~SimplecarState() = default;

  void Update();

  const std::vector<Simplecar>& GetCars() const;

  void SetTime(const float& time);
  void SetFrame(const int& frame);
  void SendCommand(const SimplecarCommand& command);

 private:
  std::vector<Simplecar> cars_;
  float time_;
  float frame_;
};


}  // namespace simplecar
#endif  // SRC_SIMPLECAR_SIM_SIMPLECAR_STATE_H_
