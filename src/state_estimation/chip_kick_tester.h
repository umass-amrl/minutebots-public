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
#ifndef SRC_STATE_ESTIMATION_CHIP_KICK_TESTER_H_
#define SRC_STATE_ESTIMATION_CHIP_KICK_TESTER_H_

#include <utility>
#include <vector>

#include "state/world_state.h"
#include "state/position_velocity_state.h"
#include "state/direction.h"

#include "messages_robocup_ssl_wrapper.pb.h"
#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"


using SSLVisionProto::SSL_DetectionBall;
using SSLVisionProto::SSL_DetectionFrame;
using SSLVisionProto::SSL_DetectionRobot;
using SSLVisionProto::SSL_WrapperPacket;
using std::vector;
using state::PositionVelocityState;

class ChipKickTester{
 public:
  ChipKickTester();
  ~ChipKickTester();

  // Analyzes the data in offline mode and then runs chip kick detection on
  // extracted segments of recorded data
  int OfflineAnalysis(logger::Logger* local_logger);

  int Update(const state::PositionVelocityState& position_velocity_state,
             state::SharedState* shared_state,
             logger::Logger* local_logger);

  int ShotDetector(
          const vector<state::PositionVelocityState>& pos_vel_queue,
          state::SharedState* shared_state);

  Eigen::Vector2f GetImpactPoint() const;
  Eigen::Vector3f GetInitialVel() const;
  double GetImpactTime() const;
  double GetChipShotTime() const;
  bool IsEstimationReady() const;

  // Returns true if a chip kick is being tracked with good confidence. Takes
  // some time steps after the chip kick onset for this to return true
  bool IsChipKickDetected() const;

  // Returns true as soon as a chip kick onset is detected. The state
  // and impact point might not yet be ready when it returns true.
  bool IsChipKickOnsetDetected() const;
  void Reset();

  void UpdateCameraParams(const SSLVisionProto::SSL_GeometryData&
        geometry_packet);

  Eigen::Vector2f ball_pose_2d;
  Eigen::Vector3f ball_pose_3d;
  Eigen::Vector3f impact_point_pos;
  Eigen::Vector3f impact_point_pos_partial;
  Eigen::Vector3f estimated_initial_velocity;
  double latest_frame_time;
  uint test_mode = 1;

  // Used to remove a robot from being considered an obstacle in the
  // shot detection method
  OurRobotIndex non_obstacle_robot_index;

  struct CameraParams{
    Eigen::Quaternionf quat;
    Eigen::Vector3f translation;
  };

  direction::Direction direction_;

 private:
  const float ball_zero_height = 30.0;
  const float ball_zero_height_offset = 0.0;
//   const float ball_zero_height_offset = 150.0;
//   const float confidece_thresh = 0.2f;
//   const double impact_time_thresh = 3.0 * 1.0/60.0;
  const double impact_time_thresh = 4.0 * 1.0/60.0;  // default: 4.0 * 1.0/60.0
  const uint chip_kick_min_frame_num = 15;  // 8 // default: 12
  bool chip_kick_running;
  bool in_the_game_mode;
  Eigen::Vector3f gravity_acc;
  vector<CameraParams> cameras;
  vector<uint> camera_id_queue;
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
  double chip_kick_rmse_;
  double flat_kick_rmse_;
  double kRMSEThresh_ = 170.0;
  // If chip kick rmse is less than this proportion of the flat kick rmse, it is
  // supposed to be a good estimate
//   double kRMSEProportionThresh = 0.5;

  // Variables for offline chip kick estimation
  vector<float> ball_raw_pos_x_offline_queue;
  vector<float> ball_raw_pos_y_offline_queue;
  vector<float> ball_raw_pos_z_offline_queue;
  vector<double> ball_time_offline_queue;
  vector<uint> camera_id_offline_queue;
  bool offline_data_capturing = false;
  double offline_capturing_start_time = 0.0;
  const double capturing_duration = 3.0;

  // Variables for shot detection
  // CHANGED
  vector<state::PositionVelocityState> pos_vel_state_queue;
  state::PositionVelocityState pos_vel_state_chip_kick_onset;
  enum Team { OUR, THEIR};


  uint shot_detected = 0;
  vector<Eigen::Vector2f> previous_ball_positions;
  vector<double> previous_ball_detections_time;
  double onset_time;
  double onset_time_candidate = 0.0;

  // Thresholding the maximum chip kick duration
  double real_time_capturing_start_time = 0.0;
  const double max_chip_kick_duration = 5.0;

  const bool kDetailedDebuggingInfo = false;
};

#endif  // SRC_STATE_ESTIMATION_CHIP_KICK_TESTER_H_
