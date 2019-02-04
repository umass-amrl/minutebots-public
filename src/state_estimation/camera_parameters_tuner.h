// Copyright 2017 srabiee@cs.umass.edu
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

#ifndef SRC_STATE_ESTIMATION_CAMERA_PARAMETERS_TUNER_H_
#define SRC_STATE_ESTIMATION_CAMERA_PARAMETERS_TUNER_H_

#include <utility>
#include <vector>

#include "state/world_state.h"
#include "state/direction.h"
#include "state_estimation/chip_kick_tester.h"
#include "state_estimation/chip_kick_detection.h"

#include "messages_robocup_ssl_wrapper.pb.h"
#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"


using SSLVisionProto::SSL_DetectionBall;
using SSLVisionProto::SSL_DetectionFrame;
using SSLVisionProto::SSL_DetectionRobot;
using SSLVisionProto::SSL_WrapperPacket;
using std::vector;

class CameraParametersTuner{
 public:
  CameraParametersTuner();
  ~CameraParametersTuner();

  // Performs chip kick estimation on the currently recorded ball detections
  // given the input camera parameters and output
  int EvaluateCameraParams(float* rmse,
                           const ChipKickTester::CameraParams& cam_params,
                           logger::NetLogger* logged_resutls);

  int Update(const SSL_DetectionFrame& detection_frame_latest,
             const state::WorldState& world_state);

  int ShotDetector(const vector<state::WorldState>& world_state_queue);

  // Iterates through various camera parameter values and runs
  // EvaluateCameraParams on them
  int ParamSearch(const Eigen::Vector4f& quat_step_size,
                  const Eigen::Vector3f& translation_step_size,
                  const int& quat_step_num,
                  const int translation_step_num,
                  const ChipKickTester::CameraParams current_camera,
                  float* min_rmse,
                  ChipKickTester::CameraParams* tuned_camera,
                  logger::NetLogger* min_rmse_log_local);

  Eigen::Vector2f ball_pose_2d;
  Eigen::Vector3f ball_pose_3d;
  Eigen::Vector3f impact_point_pos;
  Eigen::Vector3f impact_point_pos_partial;
  double latest_frame_time;
  // Defines the tuning mode: 1-partial optimization-based
  //                          2-full optimization-based
  uint tuning_mode = 1;


 private:
  const float ball_zero_height = 30.0;
  const float ball_zero_height_offset = 0.0;
//   const float ball_zero_height_offset = 150.0;
  const float confidece_thresh = 0.2f;
//   const double impact_time_thresh = 3.0 * 1.0/60.0;
  const double impact_time_thresh = 4.0 * 1.0/60.0;
  const uint chip_kick_min_frame_num = 14;  // 8
  bool chip_kick_running;
  Eigen::Vector3f gravity_acc;
  vector<ChipKickTester::CameraParams> cameras;
  vector<uint> camera_id_queue;
  vector<uint> frame_number_queue;
  vector<double> ball_time_queue;
  vector<float> ball_raw_pos_x_queue;
  vector<float> ball_raw_pos_y_queue;
  vector<float> ball_raw_pos_z_queue;

  // Variables for evaluation of chip kick estimation
  vector<float> ball_pose2d_x_queue;
  vector<float> ball_pose2d_y_queue;
  vector<float> ball_impact_point_x_queue;
  vector<float> ball_impact_point_y_queue;
  vector<double> ball_impact_time_queue;
  vector<float> ball_impact_point_x_queue_2nd;
  vector<float> ball_impact_point_y_queue_2nd;
  vector<double> ball_impact_time_queue_2nd;
  vector<float> chip_kick_rmse_queue;
  vector<double> computation_time_queue;
  vector<double> computation_time_mean_queue;
  // Handles storing more ball detections after the chip kick estimation is
  // finished (for visualization purposes).
  bool storing_further_detections = false;
  const uint extra_detections_number = 11;
  vector<float> ball_raw_pos_x_offline_queue;
  vector<float> ball_raw_pos_y_offline_queue;

  // Variables for shot detection
  vector<state::WorldState> world_state_queue;
  state::WorldState world_state_chip_kick_onset;
  uint shot_detected = 0;
  vector<Eigen::Vector2f> previous_ball_positions;
  vector<double> previous_ball_detections_time;
  double onset_time;
  double onset_time_candidate = 0;

  // Variables for camera parameters tuning
  bool camera_tuning_running = false;
  bool camera_tuning_done = false;
  ChipKickTester::CameraParams tuned_camera_param;
  ChipKickTester::CameraParams original_camera_param;
  float tuned_camera_rmse;
  logger::NetLogger min_rmse_log;


  // Thresholding the maximum chip kick duration
  double real_time_capturing_start_time = 0.0;
  const double max_chip_kick_duration = 10.0;
};

#endif  // SRC_STATE_ESTIMATION_CAMERA_PARAMETERS_TUNER_H_
