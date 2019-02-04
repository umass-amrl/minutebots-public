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

#ifndef SRC_MOTION_CONTROL_KINEMATIC_MODEL_H_
#define SRC_MOTION_CONTROL_KINEMATIC_MODEL_H_

#include <string>
#include <vector>
#include "eigen3/Eigen/Core"
#include "math/poses_2d.h"

namespace motion {

typedef Eigen::Matrix<double, 4, 3> Matrix43d;
typedef Eigen::Matrix<double, 3, 4> Matrix34d;

class KinematicModel {
 public:
  KinematicModel();
  ~KinematicModel();

  // Receives wheel velocities as input and returns robot velocity in local
  // reference frame. The returned velocity is in m/s and rad/s.
  void ForwardModel(const Eigen::Vector4d& wheel_vel,
                    int robot_id,
                    pose_2d::Pose2Df* velocity);

  // Receives desired robot velocity in local reference frame and returns wheel
  // command velocities. The input velocity is in m/s and rad/s.
  void InverseModel(const pose_2d::Pose2Df& velocity,
               Eigen::Vector4d* wheel_vel,
               int robot_id);

  // Helper function for creating the file names to load models from
  std::string ZeroPadInt(const int input_num,
                  const int output_string_size);

  void LoadInverseModels();
  void LoadForwardModels();
  void LoadScalarInverseModels();

 private:
  const bool kUseFullLinearModel = false;
  const int kRobotNum_ = 10;
  const std::string kForwardModelNameSuffix_ = "_forward_model.csv";
  const std::string kInverseModelNameSuffix_ = "_inverse_model.csv";
  const std::string kScalarInverseModelNameSuffix_ =
                    "_scalar_inverse_model.csv";
  const std::string kModelsDirectory_ = "model_learning/learned_models/";
  const int kForwardModelSize_[2] = {3, 4};
  const int kInverseModelSize_[2] = {4, 3};

  std::vector<Matrix34d> forward_models_;
  std::vector<Matrix43d> inverse_models_;
  std::vector<Matrix43d> scalar_inverse_models_;
};
}  // namespace motion

#endif  // SRC_MOTION_CONTROL_KINEMATIC_MODEL_H_
