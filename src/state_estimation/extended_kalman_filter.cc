// Copyright 2017 - 2019 slane@cs.umass.edu
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

// Model used is taken from Improbability Filtering for Rejecting False
//     Positives
//  c = cos(theta)
//  s = sin(theta)
//  dt = delta_t
//  vr = v_radial
//  vt = v_tangential
//
//   X(t) = transpose([x, y, theta, v_radial, v_tangential, omega])
//   B = [1 0 0 c*dt -s*dt 0,
//        0 1 0 s*dt c*dt 0,
//        0 0 1 0 0 dt,
//        0 0 0 0.5 0 0,
//        0 0 0 0 0.5 0,
//        0 0 0 0 0 0.5]
//   Update:  X(t+1) = B*X(t)
//
// Jacobian calculation:
//   A = [1 0 -(vr*s + vt*c)*dt c*dt -s*dt 0,
//        0 1 (vr*c - ct*s)*dt s*delta_t c*dt 0,
//        0 0 1 0 0 delta_t,
//        0 0 0 0.5 0 0,
//        0 0 0 0 0.5 0,
//        0 0 0 0 0 0.5]
// All others are 0

#include "state_estimation/extended_kalman_filter.h"

#include <dirent.h>
#include <stdio.h>
#include <glog/logging.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>

#include "constants/constants.h"
#include "datastructures/bounded_queue.h"
#include "math/math_util.h"
#include "shared/common_includes.h"
#include "state/shared_state.h"
#include "state/team.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "util/timer.h"

#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include "messages_robocup_ssl_wrapper.pb.h"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Eigenvalues"
#include "unsupported/Eigen/MatrixFunctions"
#include "math/poses_2d.h"
#include "state_estimation/default_motion_model.h"

STANDARD_USINGS;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Vector2d;
using geometry::SafeVectorNorm;
using logger::Logger;
using motion_model::DefaultMotionModel;
using pose_2d::Pose2Df;
using pose_2d::Pose2Dd;
using state::SharedRobotState;
using std::ofstream;
using std::sqrt;
using std::sin;
using std::cos;
using std::cout;
using std::endl;
using std::fclose;
using std::FILE;
using std::fopen;
using std::fprintf;

namespace estimation {
bool ExtendedKalmanFilter::dump_file_opened_ = false;
FILE* ExtendedKalmanFilter::file_ = NULL;
bool ExtendedKalmanFilter::cmd_dump_file_opened_ = false;
FILE* ExtendedKalmanFilter::cmd_dump_file_ = NULL;
FILE* ExtendedKalmanFilter::cmd_obs_dump_file_ = NULL;

ExtendedKalmanFilter::ExtendedKalmanFilter(const Pose2Df& observed_pose,
                                           const double& timestep,
                                           const unsigned int camera_id)
    : current_state_(Vector6d::Zero()),
      current_covariance_(Matrix6d::Zero()),
      previous_predict_time_(0),
      previous_update_time_(0),
      initial_covariance_(current_covariance_),
      predict_reset_time_(GetWallTime()),
      time_start_(0),
      ssl_measurement_queue_(kFrameQueueSize),
      predicted_pos_queue_(kFrameQueueSize),
      update_pos_queue_(kFrameQueueSize),
      current_command_(0, 0, 0),
      uses_commands_(false),
      previous_observation_(observed_pose.translation.x(),
                            observed_pose.translation.y(),
                            observed_pose.angle),
      observed_velocity_(0, 0, 0) {
  Pose2Df zero_velocity(0, 0, 0);
  Init(observed_pose, zero_velocity, timestep, camera_id);
}

ExtendedKalmanFilter::ExtendedKalmanFilter(const Pose2Df& observed_pose,
                                           const Pose2Df& observed_vel,
                                           const double& timestep,
                                           const unsigned int camera_id)
: current_state_(Vector6d::Zero()),
current_covariance_(Matrix6d::Zero()),
previous_predict_time_(0),
previous_update_time_(0),
initial_covariance_(current_covariance_),
predict_reset_time_(GetWallTime()),
time_start_(0),
ssl_measurement_queue_(kFrameQueueSize),
predicted_pos_queue_(kFrameQueueSize),
update_pos_queue_(kFrameQueueSize),
current_command_(observed_pose.translation.x(),
                 observed_pose.translation.y(),
                 observed_pose.angle),
uses_commands_(false),
previous_observation_(observed_pose.translation.x(),
                      observed_pose.translation.y(),
                      observed_pose.angle),
                      observed_velocity_(observed_vel) {
    Init(observed_pose, observed_vel, timestep, camera_id);
}

ExtendedKalmanFilter::ExtendedKalmanFilter()
    : current_state_(Vector6d::Zero()),
      current_covariance_(Matrix6d::Zero()),
      previous_predict_time_(0),
      previous_update_time_(0),
      initial_covariance_(current_covariance_),
      predict_reset_time_(GetWallTime()),
      time_start_(0),
      ssl_measurement_queue_(kFrameQueueSize),
      predicted_pos_queue_(kFrameQueueSize),
      update_pos_queue_(kFrameQueueSize),
      current_command_(0, 0, 0),
      uses_commands_(false),
      previous_observation_(0, 0, 0),
      observed_velocity_(0, 0, 0) {
  Pose2Df default_pose(0.0f, 0.0f, 0.0f);

  Init(default_pose, default_pose, GetWallTime(), 42);
  is_initialized_ = false;
}

// ExtendedKalmanFilter::~ExtendedKalmanFilter() {
// //   if (kDumpCmdData) {
// //     fclose(&cmd_dump_file_);
// //     fclose(&cmd_obs_dump_file_);
// //   }
// //
// //   if (kDumpKalmanData) {
// //     fclose(&file_);
// //   }
// }

double ConvertToReal(const std::complex<double>& complex) {
  return complex.real();
}

void ExtendedKalmanFilter::DrawCovariance(Logger* logger) const {
  const Eigen::Matrix2d xy_covariance =
      current_covariance_.block<2, 2>(0, 0);
  Eigen::EigenSolver<Eigen::Matrix2d> es(xy_covariance,
                                         /* Compute Eigen Vectors = */true);
  const auto& eigen_values = es.eigenvalues().unaryExpr(&ConvertToReal);
  const auto& eigen_vectors = es.eigenvectors().unaryExpr(&ConvertToReal);

  const double ellipse_angle =
      std::atan2(eigen_vectors.col(0).y(), eigen_vectors.col(0).x());
  const Vector2d current_loc = current_state_.head<2>();
  logger->AddEllipse(current_loc.cast<float>(),
                  static_cast<float>(eigen_values(0)),
                  static_cast<float>(eigen_values(1)),
                  static_cast<float>(ellipse_angle),
                  1,
                  1,
                  0,
                  1);
  logger->AddLine(current_loc.cast<float>(),
                  (current_loc + eigen_vectors.col(0) *
                                     eigen_values(0)).cast<float>(),
                  1,
                  1,
                  0,
                  1);
  logger->AddLine(current_loc.cast<float>(),
                  (current_loc + eigen_vectors.col(1) *
                                      eigen_values(1)).cast<float>(),
                  1,
                  1,
                  0,
                  1);
}

void ExtendedKalmanFilter::FindNextFileName(string* file_name,
                       const string& directory, const string& base_file_name) {
  char numbering[6];
  int prefix_length = 5;
  DIR* dirp = opendir(directory.c_str());
  struct dirent * dp;
  int max_prefix_number = 0;

  while ((dp = readdir(dirp)) != NULL) {
    // Ignore the '.' and ".." directories
    if (!strcmp(dp->d_name, ".") || !strcmp(dp->d_name, "..")) {
      continue;
    }

    for (int i = 0; i < prefix_length ; i++) {
        numbering[i] = dp->d_name[i];
    }

    int prefix_number = atoi(numbering);
    if (prefix_number > max_prefix_number) {
      max_prefix_number = prefix_number;
    }
  }
  (void)closedir(dirp);

  char cur_prefix_number[6];
  snprintf(cur_prefix_number, sizeof(cur_prefix_number), "%05d",
          max_prefix_number + 1);

  *file_name = directory.c_str();
  file_name->append(cur_prefix_number);


  char traj_type_prefix[4];
  snprintf(traj_type_prefix, sizeof(traj_type_prefix), "%03d", kTrajectoryNum);
  file_name->append("_");
  file_name->append(traj_type_prefix);

  // Get the robot's ssl-vision id
//   SSLVisionId this_robot_vision_id =
//                commands_[0].ssl_vision_id_;
  int this_robot_vision_id = kRobotIdUnderTest;
  char robot_id_prefix[3];
  snprintf(robot_id_prefix, sizeof(robot_id_prefix), "%02d",
           this_robot_vision_id);
  file_name->append("_");
  file_name->append(robot_id_prefix);

  file_name->append("_");
  file_name->append(base_file_name.c_str());
}

void ExtendedKalmanFilter::ForwardPredict(const double& timestamp,
                                          Pose2Df* output_pose,
                                          Pose2Df* output_velocity,
                                          logger::Logger* logger) const {
  static const bool kDebug = false;
  Vector6d temp_state(current_state_);
  Matrix6d temp_covariance(current_covariance_);

  Predict(timestamp, &temp_state, &temp_covariance, logger);

  if (kDebug) {
    logger->LogPrint("Current Velocity: %f, %f, %f",
                    temp_state(3),
                    temp_state(4),
                    temp_state(5));
  }

  output_pose->translation.x() = static_cast<float>(temp_state(0));
  output_pose->translation.y() = static_cast<float>(temp_state(1));
  output_pose->angle = static_cast<float>(temp_state(2));
  output_velocity->translation.x() = static_cast<float>(temp_state(3));
  output_velocity->translation.y() = static_cast<float>(temp_state(4));
  output_velocity->angle = static_cast<float>(temp_state(5));

  if (kDebug) {
    logger->LogPrint("Output Velocity: %f, %f, %f",
                    output_velocity->translation.x(),
                    output_velocity->translation.y(),
                    output_velocity->angle);
  }
}

void ExtendedKalmanFilter::Predict(const double& timestamp,
                                   Vector6d* output_state,
                                   Matrix6d* output_covariance,
                                   Logger* logger) const {
  static const bool kDebug = false;
  if (!is_initialized_) {
    logger->LogPrint("kalman Predict called while not",
                     "initialized, continuing \n");
  } else {
    double delta_t = (timestamp - previous_predict_time_);
    if (kDebug) {
      logger->LogPrint("delta timestamp: %f\n",
                       delta_t);
    }
    *output_state = current_state_;
    *output_covariance = current_covariance_;

    if (delta_t <= kEpsilon) {
      logger->LogPrint("Kalman Predict asked to predict into the past.",
                       "Skipping");
      return;
    }

    Vector6d temp_state(current_state_);
    Matrix6d temp_covariance(current_covariance_);
    Matrix6d jacobian;

    if (uses_commands_) {
      Pose2Dd current_command(current_command_);
      double previous_predict_time = previous_predict_time_;

      for (const SharedRobotState& command : commands_) {
        if (command.cmd_time > timestamp) {
          break;
        }

        delta_t = command.cmd_time - previous_predict_time;

        if (kDebug) {
          logger->LogPrint("Predicting from %f to %f\n",
                           previous_predict_time,
                           command.cmd_time);
          LogPose(current_command, logger);
        }
        motion_model_.Predict(delta_t,
                              (*output_state),
                              current_command,
                              &temp_state,
                              &jacobian,
                              logger);

        if (use_old_) {
          temp_covariance = noise_jacobian_ * our_team_process_covariance_ *
          noise_jacobian_.transpose() * delta_t;
          temp_covariance = temp_covariance.exp();
          temp_covariance += jacobian * (*output_covariance) *
                             jacobian.transpose();
        } else {
        Matrix3d our_team_process_covariance;
        CalculateOurProcessCovariance(current_command,
                                      delta_t,
                                      &our_team_process_covariance);
        temp_covariance = noise_jacobian_ * our_team_process_covariance *
                          noise_jacobian_.transpose();
        temp_covariance += jacobian * (*output_covariance) *
                           jacobian.transpose();
        }


        previous_predict_time = command.cmd_time;
        current_command =
            Pose2Dd(static_cast<double>(command.velocity_r),
                    static_cast<double>(command.velocity_x),
                    static_cast<double>(command.velocity_y));

        *output_state = temp_state;
        *output_covariance = temp_covariance;

        if (kDumpCmdData) {
          // Writing command data to file
          fprintf(cmd_dump_file_,
                  "%f, %f, %f, %f\n",
                  command.cmd_time,
                  command.velocity_x,
                  command.velocity_y,
                  command.velocity_r);
        }
      }
      delta_t = timestamp - previous_predict_time;
      if (kDebug) {
        logger->LogPrint("Predicting from %f to %f\n",
                         previous_predict_time,
                         timestamp);
        LogPose(current_command, logger);
      }

      motion_model_.Predict(delta_t,
                            (*output_state),
                            current_command,
                            &temp_state,
                            &jacobian,
                            logger);

      if (use_old_) {
        temp_covariance = noise_jacobian_ * our_team_process_covariance_ *
        noise_jacobian_.transpose() * delta_t;
        temp_covariance = temp_covariance.exp();
        temp_covariance += jacobian * (*output_covariance) *
                            jacobian.transpose();
      } else {
        Matrix3d our_team_process_covariance;
        CalculateOurProcessCovariance(current_command,
                                      delta_t,
                                      &our_team_process_covariance);
        temp_covariance = noise_jacobian_ * our_team_process_covariance *
                          noise_jacobian_.transpose();
        temp_covariance += jacobian * (*output_covariance) *
                            jacobian.transpose();
      }

      *output_state = temp_state;
      *output_covariance = temp_covariance;
    } else {
      motion_model_.Predict(delta_t,
                            *output_state,
                            &temp_state,
                            &jacobian,
                            logger);

      Matrix6d their_team_process_covariance;
      CalculateTheirProcessCovariance(delta_t,
                                      &their_team_process_covariance);

      temp_covariance =
        jacobian * (*output_covariance) * jacobian.transpose() +
        their_team_process_covariance;

      *output_covariance = temp_covariance;
      *output_state = temp_state;
    }
  }
}

void ExtendedKalmanFilter::Update(const Pose2Df& observed_pose,
                                  const double& timestamp,
                                  unsigned int camera_id,
                                  unsigned int lead_camera_id,
                                  Logger* logger) {
  static const bool kDebug = true;
  // If observations do not appear for over a set amount of time, simply reset
  // the filter; doing otherwise causes numeric instability.
  if ((timestamp - previous_update_time_)
      >= kExtendedKalmanFilterCovarianceTimeout) {
    Reset();
  }
  if (!is_initialized_) {
    Pose2Df default_pose(0.0f, 0.0f, 0.0f);
    Init(observed_pose, default_pose, timestamp, camera_id);
  }
  if (timestamp != previous_update_time_ && timestamp > previous_update_time_) {
    if (kDumpKalmanData) {
      if (uses_commands_ && !commands_.empty()) {
        fprintf(file_, "%f, %f, %f, ",
                previous_update_time_,
                commands_[0].cmd_time,
                timestamp);
      } else {
        fprintf(file_, "%f, %f, %f, ",
                previous_update_time_,
                timestamp,
                timestamp);
      }
      DumpToCSV();
      DumpComma();
    }

    if (kDumpCmdData && uses_commands_ && !commands_.empty()) {
      fprintf(cmd_obs_dump_file_,
              "%f, %f, %f, %f, %f, %f, %f, %f\n",
              commands_[0].cmd_time,
              commands_[0].velocity_x,
              commands_[0].velocity_y,
              commands_[0].velocity_r,
              timestamp,
              observed_pose.translation.x(),
              observed_pose.translation.y(),
              observed_pose.angle);
    }

    // Predict to the input timestep
    Vector6d temp_state(current_state_);
    Matrix6d temp_covariance(current_covariance_);
    if (kDebug) {
        logger->LogPrint("Current Time: %f", timestamp);
        LogState(current_state_, logger, "Pre-Predict State");
        logger->LogPrint("Command: %f, %f, %f",
                         current_command_.translation.x(),
                         current_command_.translation.y(),
                         current_command_.angle);
    }

    Predict(timestamp, &temp_state, &temp_covariance, logger);
    previous_predict_time_ = timestamp;
    current_state_ = temp_state;
    current_covariance_ = temp_covariance;

    if (uses_commands_) {
      RemovePastCommands();
    }

    if (kDumpKalmanData) {
      if (uses_commands_ && !commands_.empty()) {
        fprintf(file_,
                "%f, %f, %f, ",
                commands_[0].velocity_x,
                commands_[0].velocity_y,
                commands_[0].velocity_r);
      } else {
        fprintf(file_, "%f, %f, %f, ", 0.0, 0.0, 0.0);
      }
      DumpToCSV();
      DumpComma();
    }

    if (camera_id == lead_camera_id ||
        camera_id == previous_camera_id_ ||
        timestamp - previous_update_time_ > 2 / kTransmitFrequency) {
      Vector3d observation;
      observation(0) =
          static_cast<double>(observed_pose.translation.x());
      observation(1) =
          static_cast<double>(observed_pose.translation.y());
      observation(2) =
          static_cast<double>(AngleMod(observed_pose.angle));

//       if (CheckAndReset(observation,
//                         timestamp - previous_update_time_,
//                         logger)) {
//         previous_camera_id_ = camera_id;
//         previous_observation_ = observation;
//         previous_update_time_ = timestamp;
//         return;
//       }

      if (camera_id != previous_camera_id_) {
        SwitchCameras(observation, logger);
        observed_velocity_ =
            CalcObservedVelocity(observation, timestamp -
                                 previous_update_time_);
        previous_camera_id_ = camera_id;
        previous_observation_ = observation;
        previous_update_time_ = timestamp;
        return;
      }

      ssl_measurement_queue_.Add(observed_pose);
      if (kDebug) {
        LogState(current_state_, logger, "Predicted State");
      }
      predicted_pos_queue_.Add(
          Pose2Dd(current_state_(2), current_state_(0), current_state_(1)));

      if (kDebug) {
        LogPose(ConvertPose2Df(observed_pose), logger, "Observation");
      }

      Matrix6x3d kalman_gain;
      kalman_gain = current_covariance_ * measurement_jacobian_.transpose() *
                    ((measurement_jacobian_ * current_covariance_ *
                          measurement_jacobian_.transpose() +
                      measurement_noise_jacobian_ * measurement_covariance_ *
                          measurement_noise_jacobian_.transpose())
                         .inverse());

      Vector3d predicted_measurement(3);
      predicted_measurement(0) = current_state_(0);
      predicted_measurement(1) = current_state_(1);
      predicted_measurement(2) = current_state_(2);

      Vector3d observ_diff = observation - predicted_measurement;
      observ_diff(2) = AngleDiff(observation(2), predicted_measurement(2));

      if (kDebug) {
        logger->LogPrint("measurement difference: (%.5f, %.5f,%.5f)",
                         observ_diff(0), observ_diff(1), observ_diff(2));
      }

      Vector6d measurement_update = kalman_gain * (observ_diff);

      if (kDebug) {
        LogState(measurement_update, logger, "Measurement Update: ");
      }

      current_state_ += measurement_update;
      current_state_(2) = AngleMod(current_state_(2));

      if (kDebug) {
        LogState(current_state_, logger, "Updated State:");
      }

      Pose2Dd pos(current_state_(2), current_state_(0), current_state_(1));
      update_pos_queue_.Add(pos);

      current_covariance_ =
          (MatrixXd::Identity(6, 6) - kalman_gain * measurement_jacobian_) *
          current_covariance_;
      observed_velocity_ =
          CalcObservedVelocity(observation, timestamp - previous_update_time_);
      previous_camera_id_ = camera_id;
      previous_update_time_ = timestamp;
      previous_observation_ = observation;
    }

    if (kDumpKalmanData && (!uses_commands_ || !commands_.empty())) {
      DumpPose(observed_pose);
      DumpToCSV();
      fprintf(file_, "\n");
    }
  } else if (timestamp < previous_update_time_) {
    logger->LogPrint("Timestep precedes previous update.");
    logger->LogPrint("Ignoring");
  }

  our_team_process_covariance_ = Matrix3d::Zero();
  // Vx standard deviation of 10 mm/s.
  our_team_process_covariance_(0, 0) = Sq(20.0);
  // Vy standard deviation of 10 mm/s.
  our_team_process_covariance_(1, 1) = Sq(20.0);
  // V_theta standard deviation of 5 deg/s.
  our_team_process_covariance_(2, 2) = Sq(DegToRad(20.0));
  // covarance between V_y and V_theta 2.0 rad mm / s / s
  our_team_process_covariance_(2, 1) = DegToRad(20.0) * 6.0;
  our_team_process_covariance_(1, 2) = 6.0 * DegToRad(20.0);
}

void ExtendedKalmanFilter::UpdateCmd(const SharedRobotState& command) {
  if (!uses_commands_) {
    uses_commands_ = true;
    current_command_ = Pose2Dd(command.velocity_r,
                               command.velocity_x,
                               command.velocity_y);
  } else {
    commands_.push_back(command);
  }
}

void ExtendedKalmanFilter::Reset() { is_initialized_ = false; }

void ExtendedKalmanFilter::Reset(const Vector3d& observation,
                                 double delta_t) {
  current_state_.head(3) = observation;

  if (!uses_commands_) {
    current_state_.segment(3, 3) = (observation -
                                    previous_observation_)/delta_t;
  }
  current_state_(2) = 0.0;

  current_covariance_ = initial_covariance_;
}


Matrix6d ExtendedKalmanFilter::GetInitialCovariance() const {
  return initial_covariance_;
}

bool ExtendedKalmanFilter::GetIsInitialized() { return is_initialized_; }

void ExtendedKalmanFilter::LogTrajectories(logger::Logger* logger) {
  static const bool kDebug = false;
  if (ssl_measurement_queue_.Size() > 1 && predicted_pos_queue_.Size() > 1 &&
      update_pos_queue_.Size() > 1) {
    if (kDebug) {
      logger->AddLine(ssl_measurement_queue_[0].translation.x(),
                      ssl_measurement_queue_[0].translation.y(),
                      ssl_measurement_queue_[0].translation.x() +
                          kRobotRadius * cos(ssl_measurement_queue_[0].angle),
                      ssl_measurement_queue_[0].translation.y() +
                          kRobotRadius * sin(ssl_measurement_queue_[0].angle),
                      1, 0, 0, 1.0);

      logger->AddLine(predicted_pos_queue_[0].translation.x(),
                      predicted_pos_queue_[0].translation.y(),
                      predicted_pos_queue_[0].translation.x() +
                          kRobotRadius * cos(predicted_pos_queue_[0].angle),
                      predicted_pos_queue_[0].translation.y() +
                          kRobotRadius * sin(predicted_pos_queue_[0].angle),
                      0, 1, 0, 1.0);

      logger->AddLine(update_pos_queue_[0].translation.x(),
                      update_pos_queue_[0].translation.y(),
                      update_pos_queue_[0].translation.x() +
                          kRobotRadius * cos(update_pos_queue_[0].angle),
                      update_pos_queue_[0].translation.y() +
                          kRobotRadius * sin(update_pos_queue_[0].angle),
                      0, 0, 1, 1.0);
    }

    for (size_t i = 1; i < ssl_measurement_queue_.Size(); i++) {
      logger->AddLine(ssl_measurement_queue_[i - 1].translation,
                      ssl_measurement_queue_[i].translation,
                      1,
                      0,
                      0,
                      1.0);
      if (kDebug) {
        logger->AddLine(predicted_pos_queue_[i - 1].translation.cast<float>(),
                        predicted_pos_queue_[i].translation.cast<float>(),
                        0,
                        1,
                        0,
                        1);

        logger->AddLine(ssl_measurement_queue_[i].translation.x(),
                        ssl_measurement_queue_[i].translation.y(),
                        ssl_measurement_queue_[i].translation.x() +
                            kRobotRadius * cos(ssl_measurement_queue_[i].angle),
                        ssl_measurement_queue_[i].translation.y() +
                            kRobotRadius * sin(ssl_measurement_queue_[i].angle),
                        1,
                        0,
                        0,
                        1.0);
        logger->AddLine(
            static_cast<float>(predicted_pos_queue_[i].translation.x()),
            static_cast<float>(predicted_pos_queue_[i].translation.y()),
            static_cast<float>(predicted_pos_queue_[i].translation.x() +
                kRobotRadius * cos(predicted_pos_queue_[i].angle)),
            static_cast<float>(predicted_pos_queue_[i].translation.y() +
                kRobotRadius * sin(predicted_pos_queue_[i].angle)),
            0,
            1,
            0,
            1.0);

        logger->AddLine(
            static_cast<float>(update_pos_queue_[i].translation.x()),
            static_cast<float>(update_pos_queue_[i].translation.y()),
            static_cast<float>(update_pos_queue_[i].translation.x() +
                kRobotRadius * cos(update_pos_queue_[i].angle)),
            static_cast<float>(update_pos_queue_[i].translation.y() +
                kRobotRadius * sin(update_pos_queue_[i].angle)),
            0,
            0,
            1,
            1.0);
      }
      logger->AddLine(update_pos_queue_[i - 1].translation.cast<float>(),
                      update_pos_queue_[i].translation.cast<float>(),
                      0,
                      0,
                      1,
                      1);
    }
  }
}

void ExtendedKalmanFilter::Init(pose_2d::Pose2Df init_pose,
                                pose_2d::Pose2Df init_vel,
                                const double& timestep,
                                unsigned int camera_id) {
  previous_predict_time_ = timestep;
  previous_update_time_ = timestep;
  time_start_ = timestep;
  previous_camera_id_ = camera_id;
  previous_observation_.head(2) = init_pose.translation.cast<double>();
  previous_observation_(2) = static_cast<double>(init_pose.angle);

  current_state_(0) = static_cast<double>(init_pose.translation.x());
  current_state_(1) = static_cast<double>(init_pose.translation.y());
  current_state_(2) = static_cast<double>(init_pose.angle);
  current_state_(3) = static_cast<double>(init_vel.translation.x());
  current_state_(4) = static_cast<double>(init_vel.translation.y());
  current_state_(5) = static_cast<double>(init_vel.angle);

  current_covariance_ = MatrixXd::Zero(kRobotStateSize, kRobotStateSize);
  // Standard deviation of 25 mm
  current_covariance_(0, 0) = Sq(25.0f);
  // Standard deviation of 25 mm
  current_covariance_(1, 1) = Sq(25.0f);
  // Standard deviation of 10 deg
  current_covariance_(2, 2) = Sq(DegToRad(10.0f));
  // Standard deviation of 50 mm / s
  current_covariance_(3, 3) = Sq(50.0f);
  // Standard deviation of 50 mm / s
  current_covariance_(4, 4) = Sq(50.0f);
  // Standard deviation of 20 deg / s
  current_covariance_(5, 5) = Sq(DegToRad(20.0f));

  noise_jacobian_ = MatrixXd::Zero(6, 3);
  noise_jacobian_.topLeftCorner(3, 3) = MatrixXd::Zero(3, 3);
  noise_jacobian_.bottomRightCorner(3, 3) = MatrixXd::Identity(3, 3);

  measurement_jacobian_ = MatrixXd::Zero(3, 6);
  measurement_jacobian_.topLeftCorner(3, 3) = MatrixXd::Identity(3, 3);
  measurement_jacobian_.bottomRightCorner(3, 3) = MatrixXd::Zero(3, 3);

  measurement_noise_jacobian_ = MatrixXd::Identity(3, 3);

  measurement_covariance_ = MatrixXd::Zero(3, 3);
  // Standard deviation of 1.5 mm
  measurement_covariance_(0, 0) = Sq(1.5);
  // Standard deviation of 1.5 mm
  measurement_covariance_(1, 1) = Sq(1.5);
  // Standard deviation of 1 deg
  measurement_covariance_(2, 2) = Sq(DegToRad(1.0));

  // These thresholds are meant to be 3 sigma
  reset_threshold_translation_ = 3*1.5;
  reset_threshold_angle_ = 3*DegToRad(1.0);

  current_state_(2) = AngleMod(current_state_(2));

  previous_predicted_state_ = current_state_;
  previous_predicted_covariance_ = current_covariance_;

  is_initialized_ = true;
  predict_reset_time_ = timestep;

  if (kDumpCmdData && !cmd_dump_file_opened_) {
    string cmd_dir = kDumpCmdDirectory;
    string cmd_file_name = kDumpCmdFileName;
    string cmd_dump_file_name;
    string cmd_obs_dump_file_name;
    FindNextFileName(&cmd_dump_file_name, cmd_dir, cmd_file_name);

    cmd_dir = kDumpCmdObsDirectory;
    cmd_file_name = kDumpCmdObsFileName;
    FindNextFileName(&cmd_obs_dump_file_name, cmd_dir,
                     cmd_file_name);

    cmd_dump_file_ = fopen(cmd_dump_file_name.c_str(), "w");
    cmd_obs_dump_file_ = fopen(cmd_obs_dump_file_name.c_str(), "w");
    cmd_dump_file_opened_ = true;
  }

  if (kDumpKalmanData  && !cmd_dump_file_opened_) {
    file_ = fopen(kDumpFileName, "w");
    dump_file_opened_ = true;
  }
}

void ExtendedKalmanFilter::CalculateOurProcessCovariance(
    const Pose2Dd& command,
    double delta_t,
    Matrix3d* process_covariance) const {
  Matrix3d output_covariance = Matrix3d::Zero();

  double speed = command.translation.norm();
  double vx = std::abs(command.translation.x());
  double vy = std::abs(command.translation.y());
  double omega = std::abs(command.angle);


  output_covariance(0, 0) = kOurMinLinear + kOurAlpha1*speed*delta_t;
  output_covariance(1, 1) = kOurMinLinear + kOurAlpha2*speed*delta_t;
  output_covariance(2, 2) = kOurMinAngular + kOurAlpha3*omega*delta_t;
  output_covariance(0, 1) = kOurAlpha4 * speed * delta_t;
  output_covariance(1, 0) = kOurAlpha4 * speed * delta_t;
  output_covariance(1, 2) = kOurAlpha5 * omega * vy * delta_t +
                            kOurAlpha6 * vy * delta_t;
  output_covariance(2, 1) = kOurAlpha5 * omega * vy * delta_t +
                            kOurAlpha6 * vy * delta_t;
  output_covariance(0, 2) = kOurAlpha7 * omega * vx * delta_t +
                            kOurAlpha8 * vx * delta_t;
  output_covariance(2, 0) = kOurAlpha7 * omega * vx * delta_t +
                            kOurAlpha8 * vx * delta_t;

  *process_covariance = output_covariance;
}

void ExtendedKalmanFilter::CalculateTheirProcessCovariance(
  double delta_t,
  Matrix6d* process_covariance) const {
    Matrix6d output_covariance = Matrix6d::Zero();

    output_covariance(0, 0) = kTheirMinLinearPos + kTheirAlpha1*delta_t;
    output_covariance(1, 1) = kTheirMinLinearPos + kTheirAlpha2*delta_t;
    output_covariance(2, 2) = kTheirMinAngularPos + kTheirAlpha3*delta_t;
    output_covariance(0, 1) = kTheirAlpha4 * delta_t;
    output_covariance(1, 0) = kTheirAlpha4 * delta_t;
    output_covariance(1, 2) = kTheirAlpha5 * delta_t;
    output_covariance(2, 1) = kTheirAlpha5 * delta_t;
    output_covariance(0, 2) = kTheirAlpha6 * delta_t;
    output_covariance(2, 0) = kTheirAlpha6 * delta_t;

    output_covariance(3, 3) = kTheirMinLinearVel + kTheirAlpha7*delta_t;
    output_covariance(4, 4) = kTheirMinLinearVel + kTheirAlpha8*delta_t;
    output_covariance(5, 5) = kTheirMinAngularVel + kTheirAlpha9*delta_t;
    output_covariance(3, 4) = kTheirAlpha10 * delta_t;
    output_covariance(4, 3) = kTheirAlpha10 * delta_t;
    output_covariance(4, 5) = kTheirAlpha11 * delta_t;
    output_covariance(5, 4) = kTheirAlpha11 * delta_t;
    output_covariance(3, 5) = kTheirAlpha12 * delta_t;
    output_covariance(5, 3) = kTheirAlpha12 * delta_t;


    *process_covariance = output_covariance;
}

bool ExtendedKalmanFilter::CheckAndReset(Vector3d observation,
                                         double delta_t,
                                         Logger* logger) {
  Vector3d difference = observation - current_state_.head(3);
  difference(2) = AngleMod(difference(2));
  double distance = SafeVectorNorm(Vector2d(difference.head(2)));

  if (distance >= reset_threshold_translation_ ||
      difference(3) >= reset_threshold_angle_) {
    logger->LogPrint("Resetting");
    Reset(observation, delta_t);
    return true;
  }

  return false;
}

void ExtendedKalmanFilter::SwitchCameras(const Vector3d& observation,
                                         Logger* logger) {
  logger->LogPrint("Switching Cameras");
  current_state_.head(3) = observation;
}



void ExtendedKalmanFilter::RemovePastCommands() {
  if (!uses_commands_)
    return;

  while (!commands_.empty()) {
    if (commands_[0].cmd_time < previous_predict_time_) {
      current_command_ =
            Pose2Dd(static_cast<double>(commands_[0].velocity_r),
                    static_cast<double>(commands_[0].velocity_x),
                    static_cast<double>(commands_[0].velocity_y));
      commands_.erase(commands_.begin());
    } else {
      break;
    }
  }
}


void ExtendedKalmanFilter::DumpToCSV() const {
  if (!kDumpKalmanData)
    return;

  DumpState(current_state_);
  DumpCovariance(current_covariance_);
}

void ExtendedKalmanFilter::DumpPose(const Pose2Df& pose) const {
  if (!kDumpKalmanData)
    return;

  fprintf(file_,
          "%f, %f, %f, ",
          pose.translation.x(),
          pose.translation.y(),
          pose.angle);
}

void ExtendedKalmanFilter::DumpState(const Vector6d& state) const {
  if (!kDumpKalmanData)
    return;

  fprintf(file_,
          "%f, %f, %f, %f, %f, %f, ",
          state(0),
          state(1),
          state(2),
          state(3),
          state(4),
          state(5));
}

void ExtendedKalmanFilter::DumpCovariance(const Matrix6d& covariance) const {
  if (!kDumpKalmanData)
    return;

  for (unsigned int i = 0; i < 6; i++) {
    if (i > 0)
      DumpComma();

    fprintf(file_,
            "%f, %f, %f, %f, %f, %f",
            covariance(i, 0),
            covariance(i, 1),
            covariance(i, 2),
            covariance(i, 3),
            covariance(i, 4),
            covariance(i, 5));
  }
}

void ExtendedKalmanFilter::DumpComma() const {
  if (!kDumpKalmanData)
    return;
  fprintf(file_, ", ");
}


void ExtendedKalmanFilter::LogState(const Vector6d& state,
                                    Logger* logger) const {
  LogState(state, logger, "");
}

void ExtendedKalmanFilter::LogState(const Vector6d& state,
                                    Logger* logger,
                                    string message) const {
  logger->LogPrint(message + ": (%f, %f, %f, %f, %f, %f)",
                   state(0),
                   state(1),
                   state(2),
                   state(3),
                   state(4),
                   state(5));
}

void ExtendedKalmanFilter::LogCovariance(const Matrix6d& covariance,
                                         Logger* logger) const {
  LogCovariance(covariance, logger, "");
}

void ExtendedKalmanFilter::LogCovariance(const Matrix6d& covariance,
                                         Logger* logger,
                                         string message) const {
  logger->LogPrint(message);
  for (unsigned int i = 0; i < 6; i++) {
    logger->LogPrint(message + "%f, %f, %f, %f, %f, %f",
                     covariance(i, 0),
                     covariance(i, 1),
                     covariance(i, 2),
                     covariance(i, 3),
                     covariance(i, 4),
                     covariance(i, 5));
  }
}

void ExtendedKalmanFilter::LogPose(const Pose2Dd& pose, Logger* logger) const {
  LogPose(pose, logger, "");
}

void ExtendedKalmanFilter::LogPose(const Pose2Dd& pose,
                                   Logger* logger,
                                   string message) const {
  logger->LogPrint(message + ": (%f, %f, %f)",
                   pose.translation.x(),
                   pose.translation.y(),
                   pose.angle);
}


Pose2Dd ExtendedKalmanFilter::ConvertPose2Df(const Pose2Df& pose) const {
  return Pose2Dd(static_cast<double>(pose.angle),
                 static_cast<double>(pose.translation.x()),
                 static_cast<double>(pose.translation.y()));
}

void ExtendedKalmanFilter::GetCurrentState(Pose2Df* pose,
                                           Pose2Df* velocity) const {
  *pose = {static_cast<float>(current_state_(2)),
           static_cast<float>(current_state_(0)),
           static_cast<float>(current_state_(1))};
  *velocity = {static_cast<float>(current_state_(5)),
               static_cast<float>(current_state_(3)),
               static_cast<float>(current_state_(4))};
}

pose_2d::Pose2Df ExtendedKalmanFilter::GetCurrentPose() const {
  return {static_cast<float>(current_state_(2)),
          static_cast<float>(current_state_(0)),
          static_cast<float>(current_state_(1))};
}

pose_2d::Pose2Df ExtendedKalmanFilter::GetCurrentVelocityWorldFrame() const {
  const float theta = static_cast<float>(current_state_(2));
  return {static_cast<float>(current_state_(5)),
          Eigen::Rotation2Df(theta) *
              Vector2f(static_cast<float>(current_state_(3)),
                       static_cast<float>(current_state_(4)))};
}

pose_2d::Pose2Df ExtendedKalmanFilter::GetCurrentVelocityRobotFrame() const {
  return {static_cast<float>(current_state_(5)),
          static_cast<float>(current_state_(3)),
          static_cast<float>(current_state_(4))};
}

Matrix6d ExtendedKalmanFilter::GetCurrentCovariance() {
  return current_covariance_;
}

double ExtendedKalmanFilter::GetPreviousPredictTime() {
  return previous_predict_time_;
}

// void ExtendedKalmanFilter::SetMotionModel(motion_model::MotionModel* model) {
//   motion_model_ = model;
// }

void ExtendedKalmanFilter::SetSSLVisionID(SSLVisionId id, bool is_ours) {
  motion_model_.SetSSLVisionID(id, is_ours);
}

Pose2Df ExtendedKalmanFilter::GetObservedVelocity() {
  return observed_velocity_;
}


Pose2Df ExtendedKalmanFilter::CalcObservedVelocity(Vector3d observation,
                                                   double delta_t) {
  Pose2Df observed_velocity_;
  observed_velocity_.angle = static_cast<float>(observation(2))/delta_t;
  observed_velocity_.translation = ((observation.head(2) -
      previous_observation_.head(2))/delta_t).cast<float>();

  return observed_velocity_;
}


}  // namespace estimation
