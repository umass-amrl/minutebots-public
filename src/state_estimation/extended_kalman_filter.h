// Copyright 2017-2018 slane@cs.umass.edu
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

#ifndef SRC_STATE_ESTIMATION_EXTENDED_KALMAN_FILTER_H_
#define SRC_STATE_ESTIMATION_EXTENDED_KALMAN_FILTER_H_

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <string>
#include <queue>
#include <vector>
#include "constants/typedefs.h"
#include "datastructures/bounded_queue.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "state_estimation/default_motion_model.h"
#include "state/shared_state.h"
#include "state/team.h"
#include "thread_safe/thread_safe_priority_queue.h"

namespace estimation {
class ExtendedKalmanFilter {
 public:
  ExtendedKalmanFilter(const pose_2d::Pose2Df& observed_pose,
                       const double& timestep,
                       unsigned int camera_id);

  ExtendedKalmanFilter();
  ExtendedKalmanFilter(const ExtendedKalmanFilter& other) = default;
  ExtendedKalmanFilter(ExtendedKalmanFilter&& other) = default;
  ~ExtendedKalmanFilter() = default;

  ExtendedKalmanFilter& operator=(const ExtendedKalmanFilter& other) = default;
  ExtendedKalmanFilter& operator=(ExtendedKalmanFilter&& other) = default;

  void ForwardPredict(const double& timestamp,
                      pose_2d::Pose2Df* output_pose,
                      pose_2d::Pose2Df* output_velocity,
                      logger::Logger* logger) const;

  void Predict(const double& timestamp,
               Vector6d* output_state,
               Matrix6d* output_covariance,
               logger::Logger* logger) const;

  void Update(const pose_2d::Pose2Df& observation,
              const double& timestamp,
              const unsigned int camera_id,
              const unsigned int lead_camera_id,
              logger::Logger* logger);

  void UpdateCmd(const state::SharedRobotState& command);

  void Reset();
  void Reset(const Eigen::Vector3d& observation,
             double delta_t);

  // void Reset(const pose_2d::Pose2Df& observation,
  //            const double& timestep);

  Matrix6d GetInitialCovariance() const;

  bool GetIsInitialized();

  void LogTrajectories(logger::Logger* logger);

  void DrawCovariance(logger::Logger* log) const;

  void GetCurrentState(pose_2d::Pose2Df* pose,
                       pose_2d::Pose2Df* velocity) const;
  pose_2d::Pose2Df GetCurrentPose() const;
  pose_2d::Pose2Df GetCurrentVelocityWorldFrame() const;
  pose_2d::Pose2Df GetCurrentVelocityRobotFrame() const;

  Matrix6d GetCurrentCovariance();

  double GetPreviousPredictTime();

  // Delete the existing motion model and replace it with this motion model
//   void SetMotionModel(motion_model::MotionModel* model);

  // Sets the SSL Vision ID in the associated Motion Model
  void SetSSLVisionID(SSLVisionId id, bool is_ours);

  pose_2d::Pose2Df GetObservedVelocity();

 private:
  void Init(pose_2d::Pose2Df init_pose,
            const double& timestep,
            const unsigned int camera_id);

  void CalculateOurProcessCovariance(const pose_2d::Pose2Dd& command,
                                     double delta_t,
                                     Eigen::Matrix3d* process_covariance) const;
  void CalculateTheirProcessCovariance(
      double delta_t,
      Matrix6d* process_covariance) const;

  // If the observation is outside of 3 Sigma of the predicted state, reset
  // Returns true if a reset happened.
  bool CheckAndReset(Eigen::Vector3d observation,

                     double delta_t,
                     logger::Logger* logger);

  // When you switch cameras, keep predicted velocity but reset your state to
  // the observation
  void SwitchCameras(const Eigen::Vector3d& observation,
                     logger::Logger* logger);

  void RemovePastCommands();

  pose_2d::Pose2Dd ConvertPose2Df(const pose_2d::Pose2Df& pose) const;

  // Dump the current state and covariance to CSV file
  void DumpToCSV() const;
  void DumpState(const Vector6d& state) const;
  void DumpCovariance(const Matrix6d& covariance) const;
  void DumpPose(const pose_2d::Pose2Df& pose) const;
  void DumpComma() const;

  void LogState(const Vector6d& state, logger::Logger* logger) const;
  void LogState(const Vector6d& state,
                logger::Logger* logger,
                std::string message) const;

  void LogCovariance(const Matrix6d& covariance, logger::Logger* logger) const;
  void LogCovariance(const Matrix6d& state,
                     logger::Logger* logger,
                     std::string message) const;

  void LogPose(const pose_2d::Pose2Dd& pose, logger::Logger* logger) const;
  void LogPose(const pose_2d::Pose2Dd& pose,
               logger::Logger* logger,
               std::string message) const;

  // Figures out the name for the next log file of observations and commands
  void FindNextFileName(std::string* file_name,
                        const string& directory,
                        const string& base_file_name);

  pose_2d::Pose2Df CalcObservedVelocity(Eigen::Vector3d observation,
                                        double delta_t);

  Vector6d current_state_;
  Matrix6d current_covariance_;

  double previous_predict_time_;
  double previous_update_time_;

  Matrix6x3d noise_jacobian_;                      // Should be const

  Matrix3x6d measurement_jacobian_;             // Should be const
  Eigen::Matrix3d measurement_noise_jacobian_;  // Should be const
  Eigen::Matrix3d measurement_covariance_;      // Should be const
  double reset_threshold_translation_;
  double reset_threshold_angle_;

  Matrix6d initial_covariance_;

  Matrix6d previous_predicted_covariance_;
  Vector6d previous_predicted_state_;
  double predict_reset_time_;

  bool is_initialized_;
  double time_start_;

  datastructures::BoundedQueue<pose_2d::Pose2Df> ssl_measurement_queue_;
  datastructures::BoundedQueue<pose_2d::Pose2Dd> predicted_pos_queue_;
  datastructures::BoundedQueue<pose_2d::Pose2Dd> update_pos_queue_;

  std::vector<state::SharedRobotState> commands_;
  pose_2d::Pose2Dd current_command_;
  bool uses_commands_;

  unsigned int previous_camera_id_;
  Eigen::Vector3d previous_observation_;
  pose_2d::Pose2Df observed_velocity_;

  // Name of the dump files for logging command and mixed command and
  // observation data for model learning (robot specific)
  static bool dump_file_opened_;
  static std::FILE* file_;

  static bool cmd_dump_file_opened_;
  static std::FILE* cmd_dump_file_;
  static std::FILE* cmd_obs_dump_file_;

  motion_model::DefaultMotionModel motion_model_;

  // Our team process covariance params
  constexpr static double kOurMinLinear = 100.0;
  constexpr static double kOurMinAngular = 500.0;
  constexpr static double kOurAlpha1 = 100.0;
  constexpr static double kOurAlpha2 = 250.0;
  constexpr static double kOurAlpha3 = 10000.0;
  constexpr static double kOurAlpha4 = 0.0001;
  constexpr static double kOurAlpha5 = 10.0;
  constexpr static double kOurAlpha6 = 10.0;
  constexpr static double kOurAlpha7 = 2.5;
  constexpr static double kOurAlpha8 = 2.5;

  constexpr static double kTheirMinLinearPos = 1.5625;
  constexpr static double kTheirMinAngularPos = 0.25;
  constexpr static double kTheirAlpha1 = 150.0;
  constexpr static double kTheirAlpha2 = 150.0;
  constexpr static double kTheirAlpha3 = 2.0;
  constexpr static double kTheirAlpha4 = 0.0001;
  constexpr static double kTheirAlpha5 = 0.0;
  constexpr static double kTheirAlpha6 = 0.0;

  constexpr static double kTheirMinLinearVel = 250.0;
  constexpr static double kTheirMinAngularVel = 40.0;
  constexpr static double kTheirAlpha7 = 200.0;
  constexpr static double kTheirAlpha8 = 200.0;
  constexpr static double kTheirAlpha9 = 250.0;
  constexpr static double kTheirAlpha10 = 0.0001;
  constexpr static double kTheirAlpha11 = 0.0;
  constexpr static double kTheirAlpha12 = 0.0;

  static const bool use_old_ = false;
  Eigen::Matrix3d our_team_process_covariance_;
};
}  // namespace estimation

#endif  //  SRC_STATE_ESTIMATION_EXTENDED_KALMAN_FILTER_H_
