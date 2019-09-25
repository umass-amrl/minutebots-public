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

#include "simplecar_sim/simplecar_sim.h"

#include <glog/logging.h>
#include <condition_variable>
#include <iostream>
#include <queue>
#include <thread>
#include <vector>
#include <sstream>
#include <fstream>

using pose_2d::Pose2Df;
using std::vector;
using std::ifstream;

namespace simplecar {

void LoadConfigFile(const string& config_file,
                    vector<pose_2d::Pose2Df>* poses,
                    vector<float>* speeds) {
  std::ifstream infile(config_file);
  if (!infile) {
    LOG(FATAL) << "Cannot open file " << config_file;
  }

  std::string line;
  while (std::getline(infile, line)) {
    std::istringstream iss(line);
    float x = 0, y = 0, theta = 0, speed = 0;
    if (!(iss >> x >> y >> theta >> speed)) {
      break;
    }

    poses->push_back({DegToRad(theta), x, y});
    speeds->push_back(speed);
  }
}

SimplecarSim::SimplecarSim(const float& step_time, const string& config_file)
  : step_time_(step_time),
    current_time_(0.0),
    frame_index_(0),
    world_state_(step_time) {
  vector<pose_2d::Pose2Df> poses;
  vector<float> speeds;
  LoadConfigFile(config_file, &poses, &speeds);
  world_state_ = SimplecarState(poses, speeds, step_time);
}

SimplecarSim::SimplecarSim()
  : step_time_(1.0/60.0),
    current_time_(0.0),
    frame_index_(0),
    world_state_(1.0/60.0) {}

SimplecarSim::~SimplecarSim() {}

void SimplecarSim::SimulateStep() {
  world_state_.Update();
  current_time_ += step_time_;
  frame_index_++;
}

void SimplecarSim::SimulateStep(const SimplecarCommand& command) {
  world_state_.SendCommand(command);
  world_state_.Update();
  current_time_ += step_time_;
  frame_index_++;
}

SimplecarState SimplecarSim::GetWorldState() {
  world_state_.SetTime(current_time_);
  world_state_.SetFrame(frame_index_);
  return world_state_;
}

}  // namespace simplecar
