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

#include "simplecar_sim/simplecar_state.h"
#include <set>

using Eigen::Vector2f;
using pose_2d::Pose2Df;
using std::endl;
using std::vector;

namespace simplecar {

SimplecarState::SimplecarState() {}

SimplecarState::SimplecarState(const float& time_slice) {}

vector<Simplecar> CreateCars(const vector<Pose2Df>& positions,
                             const vector<float>& speeds,
                             const float& time_slice) {
  vector<Simplecar> cars;
  for (size_t i = 0; i < positions.size(); ++i) {
    cars.push_back(Simplecar(i, positions[i], speeds[i], time_slice));
  }
  return cars;
}

SimplecarState::SimplecarState(const vector<Pose2Df>& positions,
                               const vector<float>& speeds,
                               const float& time_slice)
    : cars_(CreateCars(positions, speeds, time_slice)) {}

void SimplecarState::Update() {
  for (Simplecar& car : cars_) {
    car.Update();
  }
}

void SimplecarState::SetFrame(const int& frame) {
  frame_ = frame;
}

void SimplecarState::SetTime(const float& time) {
  time_ = time;
}

void SimplecarState::SendCommand(const SimplecarCommand& command) {
  cars_[command.car_id].AccelToSpeed(command.desired_speed);
  cars_[command.car_id].SteerToAngle(command.desired_steer);
}

const vector<Simplecar>& SimplecarState::GetCars() const { return cars_; }
}  // namespace simplecar
