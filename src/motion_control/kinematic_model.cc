// Copyright 2018 srabiee@cs.umass.edu
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

#include <sstream>
#include <iomanip>
#include <fstream>
#include <string>

#include "motion_control/kinematic_model.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "math/poses_2d.h"

STANDARD_USINGS;
using Eigen::MatrixXf;
using Eigen::VectorXf;
using Eigen::Vector3f;
using Eigen::Vector3d;
using Eigen::Vector4f;
using pose_2d::Pose2Df;
using std::stringstream;
using std::ostringstream;
using std::string;
using std::vector;


namespace motion {

KinematicModel::KinematicModel() {
  LoadForwardModels();

  if (kUseFullLinearModel) {
    LoadInverseModels();
  } else {
    LoadScalarInverseModels();
  }
}

KinematicModel::~KinematicModel() {}

void KinematicModel::ForwardModel(const Eigen::Vector4d& wheel_vel,
                                  int robot_id,
                                  pose_2d::Pose2Df* velocity) {
  Vector3d robot_vel;
  robot_vel = forward_models_[robot_id] * wheel_vel;
  velocity->translation.x() = static_cast<float>(robot_vel(0));
  velocity->translation.y() = static_cast<float>(robot_vel(1));
  velocity->angle = static_cast<float>(robot_vel(2));
}


void KinematicModel::InverseModel(const pose_2d::Pose2Df& velocity,
                                  Eigen::Vector4d* wheel_vel,
                                  int robot_id) {
  Vector3d desired_vel(static_cast<double>(velocity.translation.x()),
                       static_cast<double>(velocity.translation.y()),
                       static_cast<double>(velocity.angle));

  // TODO(srabiee): Cap the wheel velocities to the maximum.
  if (kUseFullLinearModel) {
    *wheel_vel = inverse_models_[robot_id] * desired_vel;
  } else {
    *wheel_vel = scalar_inverse_models_[robot_id] * desired_vel;
  }
}

std::string KinematicModel::ZeroPadInt(const int input_num,
                                const int output_string_size) {
  ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(output_string_size)
      << input_num;
  return out.str();
}

void KinematicModel::LoadForwardModels() {
  forward_models_.resize(kRobotNum_);

  // The default forward model
  Matrix34d default_forward_model;


  default_forward_model << 0.0107,    0.0105,   -0.0105,   -0.0107,
                          -0.0119,    0.0120,    0.0116,   -0.0116,
                          -0.1092,   -0.0861,   -0.1027,   -0.0990;

  // Load the kinematic models
  for (int i = 0; i < kRobotNum_; i++) {
    // Read the trajectory file and load the waypoints
    const string fwd_model_file_name = kModelsDirectory_ +
                                  ZeroPadInt(i, 2) +
                                  kForwardModelNameSuffix_;
    std::ifstream fwd_model_file(fwd_model_file_name, std::ifstream::in);

    if (fwd_model_file.is_open()) {
      string line;

      int row_counter = 0;
      while (std::getline(fwd_model_file, line)) {
        stringstream linestream(line);
        string entry;

        if (row_counter >= kForwardModelSize_[0]) {
          LOG(INFO) << "Faulty forward model file was detected for robot# "
          << i;
          forward_models_[i] = default_forward_model;
        }

        std::getline(linestream, entry, ',');
        forward_models_[i](row_counter, 0) = std::stod(entry);

        std::getline(linestream, entry, ',');
        forward_models_[i](row_counter, 1) = std::stod(entry);

        std::getline(linestream, entry, ',');
        forward_models_[i](row_counter, 2) = std::stod(entry);

        std::getline(linestream, entry, ',');
        forward_models_[i](row_counter, 3) = std::stod(entry);

        row_counter++;
      }

      fwd_model_file.close();
    } else {
      // LOG(INFO) << "No forward model file was found for robot# " << i;
      forward_models_[i] = default_forward_model;
    }
  }
}

void KinematicModel::LoadInverseModels() {
  inverse_models_.resize(kRobotNum_);

  Matrix43d default_inverse_model;
  default_inverse_model << 24.6744,  -19.9809,  -2.5084,
                           22.4506,   22.4506,   -2.5084,
                          -22.4506,   22.4506,   -2.5084,
                          -24.6744,  -19.9809,   -2.5084;

  // Load the kinematic models
  for (int i = 0; i < kRobotNum_; i++) {
    // Read the trajectory file and load the waypoints
    const string inv_model_file_name = kModelsDirectory_ +
                                  ZeroPadInt(i, 2) +
                                  kInverseModelNameSuffix_;
    std::ifstream inv_model_file(inv_model_file_name, std::ifstream::in);

    if (inv_model_file.is_open()) {
      string line;

      int row_counter = 0;
      while (std::getline(inv_model_file, line)) {
        stringstream linestream(line);
        string entry;

        if (row_counter >= kInverseModelSize_[0]) {
          LOG(INFO) << "Faulty inverse model file was detected for robot# "
          << i;
          inverse_models_[i] = default_inverse_model;
        }

        std::getline(linestream, entry, ',');
        inverse_models_[i](row_counter, 0) = std::stod(entry);

        std::getline(linestream, entry, ',');
        inverse_models_[i](row_counter, 1) = std::stod(entry);

        std::getline(linestream, entry, ',');
        inverse_models_[i](row_counter, 2) = std::stod(entry);

        row_counter++;
      }

      inv_model_file.close();
    } else {
      LOG(INFO) << "No inverse model file was found for robot# " << i;
      inverse_models_[i] = default_inverse_model;
    }

//     std::cout << "Loaded inverse model for robot # " << i << std::endl;
//     std::cout << inverse_models_[i] << std::endl;
  }
}

void KinematicModel::LoadScalarInverseModels() {
  scalar_inverse_models_.resize(kRobotNum_);

  Matrix43d default_inverse_model;
  default_inverse_model << 24.6744,  -19.9809,  -2.5084,
                           22.4506,   22.4506,   -2.5084,
                          -22.4506,   22.4506,   -2.5084,
                          -24.6744,  -19.9809,   -2.5084;

  // Load the kinematic models
  for (int i = 0; i < kRobotNum_; i++) {
    // Read the trajectory file and load the waypoints
    const string inv_model_file_name = kModelsDirectory_ +
                                  ZeroPadInt(i, 2) +
                                  kScalarInverseModelNameSuffix_;
    std::ifstream inv_model_file(inv_model_file_name, std::ifstream::in);

    if (inv_model_file.is_open()) {
      string line;

      int row_counter = 0;
      while (std::getline(inv_model_file, line)) {
        stringstream linestream(line);
        string entry;

        if (row_counter >= kInverseModelSize_[0]) {
          LOG(INFO) << "Faulty scalar inverse model file was detected for "
              <<  "robot# " << i;
          scalar_inverse_models_[i] = default_inverse_model;
        }

        std::getline(linestream, entry, ',');
        scalar_inverse_models_[i](row_counter, 0) = std::stod(entry);

        std::getline(linestream, entry, ',');
        scalar_inverse_models_[i](row_counter, 1) = std::stod(entry);

        std::getline(linestream, entry, ',');
        scalar_inverse_models_[i](row_counter, 2) = std::stod(entry);

        row_counter++;
      }

      inv_model_file.close();
    } else {
      LOG(INFO) << "No scalar inverse model file was found for robot# " << i;
      scalar_inverse_models_[i] = default_inverse_model;
    }

//     std::cout << "Loaded inverse model for robot # " << i << std::endl;
//     std::cout << inverse_models_[i] << std::endl;
  }
}


}  // namespace motion
