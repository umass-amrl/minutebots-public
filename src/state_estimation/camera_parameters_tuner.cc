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

#include "state_estimation/camera_parameters_tuner.h"

#include <glog/logging.h>
#include <iomanip>

#include "state_estimation/chip_kick_detection.h"
#include "state/team.h"
#include "state/world_robot.h"
#include "state/world_state.h"
#include "datastructures/bounded_queue.h"
#include "constants/constants.h"
#include "util/timer.h"
#include "math/math_util.h"
#include "logging/logger.h"

#include "messages_robocup_ssl_wrapper.pb.h"
#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"

using direction::Direction;
using math_util::AngleMod;
using pose_2d::Pose2Df;
using SSLVisionProto::SSL_DetectionBall;
using SSLVisionProto::SSL_DetectionFrame;
using SSLVisionProto::SSL_DetectionRobot;
using SSLVisionProto::SSL_WrapperPacket;
using state::WorldRobot;
using state::WorldState;
using std::endl;
using std::vector;
using team::Team;


CameraParametersTuner::CameraParametersTuner()
  : min_rmse_log(logger::NetLogger(DATA_STREAM_DEBUG_IP
                                  , DATA_STREAM_DEBUG_PORT)) {
  gravity_acc << 0.0, 0.0, -9.8 * 1000;
  ball_pose_2d << 0.0f, 0.0f;
  ball_pose_3d << 0.0f, 0.0f, ball_zero_height;
  impact_point_pos << 0.0f, 0.0f, 0.0f;
  latest_frame_time = 0.0;
  chip_kick_running = false;

//   cameras.resize(4);
//   cameras[0].quat.w() = 0.037916;
//   cameras[0].quat.vec() = Vector3f(0.999156, -0.014069, 0.007252);
//   cameras[0].translation = Vector3f(-2049.59, 1792.99, 3994);
//
//   cameras[1].quat.w() = -0.003027;
//   cameras[1].quat.vec() = Vector3f(-0.007598, -0.999813, 0.017531);
//   cameras[1].translation = Vector3f(-2101.27, 1843.18, 4119);
//
//   cameras[2].quat.w() = 0.039124;
//   cameras[2].quat.vec() = Vector3f(0.008821, 0.998497, 0.037353);
//   cameras[2].translation = Vector3f(-2363.99, -1599.82, 4317);
//
//   cameras[3].quat.w() = 0.006441;
//   cameras[3].quat.vec() = Vector3f(-0.010074, -0.999012, 0.042813);
//   cameras[3].translation = Vector3f(2139.56, 1930.11, 4006);
//   // Tuned camera
//   cameras[3].quat.w() = 0.005941;
//   cameras[3].quat.vec() = Vector3f(-0.014074, -1.003012, 0.040813);
//   cameras[3].translation = Vector3f(2143.56, 1930.11, 4010);

  // Lab cameras' calibration parameters
  cameras.resize(4);
  cameras[0].quat.w() = 0.010508;
  cameras[0].quat.vec() = Vector3f(0.013591, 0.972322, -0.233010);
  cameras[0].translation = Vector3f(2082.451800, -1342.944266, 3940.000000);

  // No camera 1 actually exists
  cameras[1].quat.w() = 0.0;
  cameras[1].quat.vec() = Vector3f(0.7, -0.7, 0.0);
  cameras[1].translation = Vector3f(0.0, 1250.00, 3500.0);

  // No camera 2 actually exists
  cameras[2].quat.w() = 0.0;
  cameras[2].quat.vec() = Vector3f(0.7, -0.7, 0.0);
  cameras[2].translation = Vector3f(0.0, 1250.00, 3500.0);

  cameras[3].quat.w() = 0.221006;
  cameras[3].quat.vec() = Vector3f(0.974909, -0.014903, 0.022055);
  cameras[3].translation = Vector3f(-2387.453832, -1262.135682, 3628.000000);
}

CameraParametersTuner::~CameraParametersTuner() {
}

int CameraParametersTuner::ParamSearch(const Eigen::Vector4f& quat_step_size,
                  const Eigen::Vector3f& translation_step_size,
                  const int& quat_step_num,
                  const int translation_step_num,
                  const ChipKickTester::CameraParams current_camera,
                  float* min_rmse,
                  ChipKickTester::CameraParams* tuned_camera,
                  logger::NetLogger* min_rmse_log_local) {
  // Iterate through various camera parameter values around the
  // current camera parameters and evaluate the RMSE of chip kick
  // estimation
  ChipKickTester::CameraParams temp_camera;
  logger::NetLogger logged_resutls(DATA_STREAM_DEBUG_IP,
                                   DATA_STREAM_DEBUG_PORT);
  *min_rmse = -1.1;
  // Iterate through rotation parameters
  for (int k0 = 0; k0 < quat_step_num; k0++) {
    temp_camera.quat.x() = current_camera.quat.x()
                          - quat_step_size(0) * (quat_step_num - 1)/2
                          + k0 * quat_step_size(0);
    for (int k1 = 0; k1 < quat_step_num; k1++) {
      temp_camera.quat.y() = current_camera.quat.y()
                      - quat_step_size(1) * (quat_step_num - 1)/2
                      + k1 * quat_step_size(1);
      for (int k2 = 0; k2 < quat_step_num; k2++) {
        temp_camera.quat.z() = current_camera.quat.z()
                      - quat_step_size(2) * (quat_step_num - 1)/2
                      + k2 * quat_step_size(2);
        for (int k3 = 0; k3 < quat_step_num; k3++) {
          temp_camera.quat.w() = current_camera.quat.w() -
                          quat_step_size(3) * (quat_step_num - 1)/2
                          + k3 * quat_step_size(3);

          // Iterate through translation parameters
          for (int i0 = 0; i0 < translation_step_num; i0++) {
            temp_camera.translation(0) = current_camera.translation(0)
                     - translation_step_size(0) * (translation_step_num -1)/2
                     + i0 * translation_step_size(0);
            for (int i1 = 0; i1 < translation_step_num; i1++) {
              temp_camera.translation(1) = current_camera.translation(1)
                      - translation_step_size(1) * (translation_step_num -1)/2
                      + i1 * translation_step_size(1);
              for (int i2 = 0; i2 < translation_step_num; i2++) {
              temp_camera.translation(2) = current_camera.translation(2)
                        - translation_step_size(2) * (translation_step_num -1)/2
                        + i2 * translation_step_size(2);

                float rmse;
                logged_resutls.Clear();
                EvaluateCameraParams(&rmse, temp_camera, &logged_resutls);
                if (*min_rmse < 0.0) {
                  *min_rmse = rmse;
                  *tuned_camera = temp_camera;
                  min_rmse_log_local->Clear();
                  min_rmse_log_local->MergeLoggers(logged_resutls);
                } else if (rmse < *min_rmse) {
                  *min_rmse = rmse;
                  *tuned_camera = temp_camera;
                  min_rmse_log_local->Clear();
                  min_rmse_log_local->MergeLoggers(logged_resutls);
                }
              }
            }
          }
        }
      }
    }
  }
  return 0;
}


int CameraParametersTuner::ShotDetector(const vector<state::WorldState>&
                                          world_state_queue) {
  state::WorldState last_world_state = world_state_queue.back();
  state::WorldState previous_world_state = world_state_queue.front();
  team::Team their_team;
  their_team = (last_world_state.GetOurTeam() == team::Team::BLUE)?
                team::Team::YELLOW : team::Team::BLUE;

  float min_distance = 10000.0;
  float min_distance_new;
  team::Team closest_robot_team = their_team;
  SSLVisionId closest_robot_id = 0;
  uint shot_happened = 0;
  uint robot_found = 0;

  // Parameters for detecting ball contact with robots
  const float distance_threshold = static_cast<float>(kRobotRadius) +
              static_cast<float>(kBallRadius);
//   const float del_vel_threshold = 000.0f;

  Eigen::Vector2f ball_pos = previous_world_state.GetBall().GetPosition();
  Eigen::Vector2f ball_vel = previous_world_state.GetBall().GetBallVelocity();

  // Calculate the distance of the ball to all the robots in the previous world
  // state
  for (const state::WorldRobot& our_robot :
      previous_world_state.GetOurRobots()) {
    float del_x = ball_pos(0) - our_robot.GetPose().translation(0);
    float del_y = ball_pos(1) - our_robot.GetPose().translation(1);
    float distance_to_ball = sqrt(del_x * del_x + del_y * del_y);

    if (distance_to_ball < min_distance) {
      min_distance = distance_to_ball;
      closest_robot_id = our_robot.GetSSLVisionID();
      closest_robot_team = previous_world_state.GetOurTeam();
    }
    robot_found = 1;
  }

  for (const state::WorldRobot& their_robot :
        previous_world_state.GetTheirRobots())  {
    float del_x = ball_pos(0) - their_robot.GetPose().translation(0);
    float del_y = ball_pos(1) - their_robot.GetPose().translation(1);
    float distance_to_ball = sqrt(del_x * del_x + del_y * del_y);

    if (distance_to_ball < min_distance) {
      min_distance = distance_to_ball;
      closest_robot_id = their_robot.GetSSLVisionID();
      closest_robot_team = their_team;
    }
    robot_found = 1;
  }

  // If no robots are found, no shot could have happened
  if (!robot_found) {
    return 0;
  }

  // Check the distance of the ball to the closest robot in the latest world
  // state to check if it has moved away from it
  state::WorldRobot robot_of_interest;
  int robot_of_interest_index;
  if (closest_robot_team == their_team) {
    robot_of_interest_index = last_world_state.GetTheirRobotIndex(
                              closest_robot_id);
    robot_of_interest = last_world_state.GetTheirRobot(robot_of_interest_index);

  } else {
    robot_of_interest_index = last_world_state.GetOurRobotIndex(
                              closest_robot_id);
    robot_of_interest = last_world_state.GetOurRobot(robot_of_interest_index);
  }

  Eigen::Vector2f ball_pos_new = last_world_state.GetBall().GetPosition();
  Eigen::Vector2f ball_vel_new = last_world_state.GetBall().GetBallVelocity();
  float del_x = ball_pos_new(0) - robot_of_interest.GetPose().translation(0);
  float del_y = ball_pos_new(1) - robot_of_interest.GetPose().translation(1);

  min_distance_new = sqrt(del_x * del_x + del_y * del_y);

  // Calculate the change in velocity of the ball
  Eigen::Vector2f del_vel = ball_vel_new - ball_vel;
  float del_vel_abs = sqrt((del_vel(0) * del_vel(0)) + (del_vel(1) *
                      del_vel(1)));

  // Check if the robot has lost possesion of the ball
  if (min_distance < distance_threshold &&
    min_distance_new >= distance_threshold) {
//     && del_vel_abs > del_vel_threshold
      shot_happened = 1;
      std::cout << "Robot# " << static_cast<uint>(closest_robot_id) <<
      " lost possesion of ball"<< endl;
      std::cout << "Del_vel = " << del_vel_abs << endl;
      std::cout << "Initial ball position: " << ball_pos.transpose() << endl;
    }

  // Send out data to the viewer
//   logger::NetLogger the_log(DATA_STREAM_DEBUG_IP, DATA_STREAM_DEBUG_PORT);
//
//   MinuteBotsProto::RobotState::Team our_team_proto =
//     (last_world_state.GetOurTeam() == team::Team::YELLOW)
//         ? MinuteBotsProto::RobotState_Team_TEAM_YELLOW
//         : MinuteBotsProto::RobotState_Team_TEAM_BLUE;
//
//   // Visualize our robots
//   for (const state::WorldRobot& our_robot : last_world_state.GetOurRobots())
// {
//     the_log.AddRobot(static_cast<int>(our_robot.GetSSLVisionID()),
//                      our_team_proto,
//                      our_robot.GetPose().angle,
//                      our_robot.GetPose().translation(0),
//                      our_robot.GetPose().translation(1),
//                      1.0f);
//   }
//   the_log.AddBall(ball_pos_new(0), ball_pos_new(1));
//
//   std::cout << "Num of our robots: " <<
//              last_world_state.GetOurRobots().size() << endl;
//
//   the_log.SendData();

  return shot_happened;
}

int CameraParametersTuner::EvaluateCameraParams(float* rmse,
                          const ChipKickTester::CameraParams& cam_params,
                          logger::NetLogger* logged_resutls) {
  Vector3f initial_pos;
  Vector3f initial_vel;
  double time_init_optimized = 0.0;
  vector<ChipKickTester::CameraParams> temp_cams;
  temp_cams.push_back(cam_params);
  vector<uint> camera_id_queue_temp;
  for (size_t j = 0; j < camera_id_queue.size(); j++) {
    camera_id_queue_temp.push_back(0);
  }

  // Estimate the parameters of the projectile motion (full
  // optimization)
  if (tuning_mode == 1) {
    ChipKickDetection::FitParabola(&initial_pos,
                                   &initial_vel,
                                   ball_time_queue[0],
                                   ball_time_queue,
                                   ball_raw_pos_x_queue,
                                   ball_raw_pos_y_queue,
                                   ball_raw_pos_z_queue,
                                   camera_id_queue_temp,
                                   temp_cams,
                                   gravity_acc);
    time_init_optimized = ball_time_queue[0];

    // Partial optimization
  } else if (tuning_mode == 2) {
    Eigen::Vector3f initial_pos_known;
    initial_pos_known(0) =
    world_state_chip_kick_onset.GetBall().GetPosition()(0);
    initial_pos_known(1) =
    world_state_chip_kick_onset.GetBall().GetPosition()(1);
    initial_pos_known(2) = ball_zero_height;
    initial_pos = initial_pos_known;

    ChipKickDetection::FitParabolaPartialOptimization(&initial_vel,
                                                      &time_init_optimized,
                                                      initial_pos_known,
                                                      onset_time,
                                                      ball_time_queue,
                                                      ball_raw_pos_x_queue,
                                                      ball_raw_pos_y_queue,
                                                      ball_raw_pos_z_queue,
                                                      camera_id_queue_temp,
                                                      temp_cams,
                                                      gravity_acc);
  }

//   // Estimate the impact time and position
//   double impact_time;
//   Vector3f impact_point;
//   double impact_time_partial;
//   Vector3f impact_point_partial;
//   if (ChipKickDetection::PredictBouncing(initial_pos,
//     initial_vel,
//     time_init_optimized,
//     ball_zero_height + ball_zero_height_offset,
//     gravity_acc,
//     &impact_point,
//     &impact_time)) {
//       // No impact point was found
//       impact_time = 0.0;
//       impact_point << 0.0, 0.0, ball_zero_height;
//     }
//
//   //  Calculate the current ball position
//   Vector3f current_pos;
//   double delT = ball_time - time_init_optimized;
//   current_pos.head(2) = initial_pos.head(2) + initial_vel.head(2) *
//   delT;
//   current_pos(2) = initial_pos(2) + initial_vel(2) * delT +
//   0.5 * gravity_acc(2) * delT * delT;
//
//   ball_pose_3d = current_pos;
//
//   // Project the estimated impact point to the ground to
//   // account for the offset in the camera ball_zero_height
//   Vector2f projected_impact_point;
//   Vector2f projected_impact_point_partial;
//   ChipKickDetection::Project2GroundPlane(cameras[ball_camera_id],
//                                           impact_point,
//                                           ball_zero_height,
//                                           &projected_impact_point);
//   //         impact_point_pos = impact_point;
//   impact_point_pos.head(2) = projected_impact_point;
//   impact_point_pos(2) = ball_zero_height;




  // Recalculate the estimated position for all the points on the
  // current chip kick trajectory given the projectile
  // motion parameters estimated in the current time step. This is
  // used for evaluation purposes
  vector<Vector3f> ball_pose3d_batch_estimated;
  vector<float> ball_pose2d_x_batch_estimated;
  vector<float> ball_pose2d_y_batch_estimated;
  for (size_t k =0; k < ball_time_queue.size(); k++) {
    // Calculate the estimated 3d positions
    Vector3f current_pos;
    double delT = ball_time_queue[k] - time_init_optimized;
    current_pos.head(2) = initial_pos.head(2) + initial_vel.head(2) *
    delT;
    current_pos(2) = initial_pos(2) + initial_vel(2) * delT +
    0.5 * gravity_acc(2) * delT * delT;

    ball_pose3d_batch_estimated.push_back(current_pos);

    // Project the estimated 3d position to the ground plane
    Vector2f current_2d_pos;

    ChipKickDetection::Project2GroundPlane(cam_params,
                                            current_pos,
                                            ball_zero_height,
                                            &current_2d_pos);

    ball_pose2d_x_batch_estimated.push_back(current_2d_pos(0));
    ball_pose2d_y_batch_estimated.push_back(current_2d_pos(1));
  }

  // Calculates RMSE for chip kick estimation over the observed data
  // from the start of the latest chip kick. Use ball_pose2d_x_queue
  // instead of ball_pose2d_x_batch_estimated as argument to update
  // the RMSE by only taking into account the estimated value for the
  // current time step as opposed to re-estimating all the points on
  // the projectile.
  float rmse_chip_kick;
  ChipKickDetection::CalculateError(ball_raw_pos_x_queue,
                                    ball_raw_pos_y_queue,
                                    ball_pose2d_x_batch_estimated,
                                    ball_pose2d_y_batch_estimated,
                                    &rmse_chip_kick);
  *rmse = rmse_chip_kick;
  // Add drawings of the estimated 2d ball positions to the log
  for (size_t k = 0; k < ball_pose2d_x_batch_estimated.size() - 1; k++) {
    logged_resutls->AddLine(ball_pose2d_x_batch_estimated[k],
                    ball_pose2d_y_batch_estimated[k],
                    ball_pose2d_x_batch_estimated[k + 1],
                    ball_pose2d_y_batch_estimated[k + 1],
                    1,
                    1,
                    0,
                    0.8);
  }
  return 0;
}

int CameraParametersTuner::Update(
  const SSL_DetectionFrame& detection_frame_latest,
  const state::WorldState& world_state) {
  float confidence = 0.0f;
  Eigen::Vector2f ball_raw_pos(0.0f, 0.0f);
  double ball_time = 0;
  uint ball_camera_id = 0;
  uint ball_frame_number = 0;

  for (const SSL_DetectionBall& ball : detection_frame_latest.balls()) {
    if (ball.confidence() > confidence) {
      confidence = ball.confidence();
      ball_raw_pos = Eigen::Vector2f(ball.x(), ball.y());
      ball_time = detection_frame_latest.t_capture();
      ball_camera_id = detection_frame_latest.camera_id();
      ball_frame_number = detection_frame_latest.frame_number();
    }
  }
  // Do not update the state if the current detection has low conficence
  if (confidence < confidece_thresh) {
    // Non-confident reading
    if (detection_frame_latest.balls().size() > 0)
      std::cout << "Non_confident Reading..." << endl;
    return 0;
  }

  // Check if a shot has been performed
  if (world_state_queue.size() < 2) {
    world_state_queue.push_back(world_state);
  } else {
    world_state_queue[0] = world_state_queue[1];
    world_state_queue[1] = world_state;
  }

  if (world_state_queue.size() >= 2) {
    shot_detected = ShotDetector(world_state_queue);
  }


  // Store the latest n ball detections for onset detection purposes
  if (previous_ball_positions.size() > 10) {
    vector<double>::iterator index1 = previous_ball_detections_time.begin();
    vector<Vector2f>::iterator index2 = previous_ball_positions.begin();
    previous_ball_detections_time.erase(index1);
    previous_ball_positions.erase(index2);
  }
  previous_ball_positions.push_back(ball_raw_pos);
  previous_ball_detections_time.push_back(ball_time);

  if (previous_ball_detections_time.size() > 2) {
    vector<double>::size_type size;
    size = previous_ball_detections_time.size();
    onset_time_candidate = previous_ball_detections_time[size - 1 - 2];
  }


//   std::cout << "Latest ball positions and times: " << endl;
//   for (size_t k = 0; k < previous_ball_positions.size(); k++) {
//     std::cout << previous_ball_positions[k].transpose() << " :: " <<
//     std::setprecision(15) << previous_ball_detections_time[k] << endl;
//   }


  double current_time = GetMonotonicTime();
  if (shot_detected) {
    std::cout << "Started Capturing..." << endl;

    chip_kick_running = true;
    storing_further_detections = false;
    world_state_chip_kick_onset = world_state_queue.front();

    real_time_capturing_start_time = GetMonotonicTime();

//     chip_kick_running = true;
    ball_time_queue.clear();
    camera_id_queue.clear();
    frame_number_queue.clear();
    ball_raw_pos_x_queue.clear();
    ball_raw_pos_y_queue.clear();
    ball_raw_pos_z_queue.clear();
    ball_pose2d_x_queue.clear();
    ball_pose2d_y_queue.clear();
    ball_impact_point_x_queue.clear();
    ball_impact_point_y_queue.clear();
    ball_impact_time_queue.clear();
    ball_impact_point_x_queue_2nd.clear();
    ball_impact_point_y_queue_2nd.clear();
    ball_impact_time_queue_2nd.clear();
    chip_kick_rmse_queue.clear();
    computation_time_queue.clear();
    computation_time_mean_queue.clear();
    ball_pose_3d << 0.0f, 0.0f, ball_zero_height;
    impact_point_pos << 0.0f, 0.0f, 0.0f;
    std::cout << "Chip-kick detection is reset" << endl << endl;
    ball_raw_pos_x_offline_queue.clear();
    ball_raw_pos_y_offline_queue.clear();

    // Clear variables regarding onset detection
    previous_ball_positions.clear();
    previous_ball_detections_time.clear();
    onset_time = onset_time_candidate;

    // Clear varialbes regarding camera tuning
    camera_tuning_done = false;
  }
  latest_frame_time = current_time;
  if (!chip_kick_running) {
    std::cout << "ball detection not captured..." << endl;
  }


  if (chip_kick_running) {
    // If chip_kick estimation has already reached the impact point, just
    // store a few more ball detections for visualization purposes
    if (storing_further_detections) {
      ball_raw_pos_x_offline_queue.push_back(ball_raw_pos(0));
      ball_raw_pos_y_offline_queue.push_back(ball_raw_pos(1));

      // Visualize the results of the chip kick detection on the viewer
      logger::NetLogger the_log = ChipKickDetection::VisualizeResults(
                                        ball_pose_3d,
                                        impact_point_pos,
                                        ball_raw_pos_x_queue,
                                        ball_raw_pos_y_queue,
                                        ball_raw_pos_x_offline_queue,
                                        ball_raw_pos_y_offline_queue,
                                        ball_pose2d_x_queue,
                                        ball_pose2d_y_queue,
                                        camera_id_queue,
                                        ball_impact_point_x_queue,
                                        ball_impact_point_y_queue,
                                        ball_impact_time_queue,
                                        chip_kick_rmse_queue,
                                        ball_time_queue,
                                        computation_time_queue,
                                        computation_time_mean_queue);

      if (camera_tuning_done) {
        the_log.MergeLoggers(min_rmse_log);

        the_log.Pop();
        the_log.LogPrint("Tuned camera params vs. Original camera params");
        the_log.Push();
        the_log.LogPrint("q_w:  %2.5f,  %2.5f"
                , tuned_camera_param.quat.w(), original_camera_param.quat.w());
        the_log.LogPrint("q_x:  %2.5f,  %2.5f"
                , tuned_camera_param.quat.x(), original_camera_param.quat.x());
        the_log.LogPrint("q_y:  %2.5f,  %2.5f"
                , tuned_camera_param.quat.y(), original_camera_param.quat.y());
        the_log.LogPrint("q_z:  %2.5f,  %2.5f"
                , tuned_camera_param.quat.z(), original_camera_param.quat.z());
        the_log.LogPrint("t_x:  %6.5f,  %6.5f"
                        , tuned_camera_param.translation(0)
                        , original_camera_param.translation(0));
        the_log.LogPrint("t_y:  %6.5f,  %6.5f"
                        , tuned_camera_param.translation(1)
                        , original_camera_param.translation(1));
        the_log.LogPrint("t_z:  %6.5f,  %6.5f"
                        , tuned_camera_param.translation(2)
                        , original_camera_param.translation(2));
        the_log.Pop();
        the_log.LogPrint("Best RMSE");
        the_log.Push();
        the_log.LogPrint("%f", tuned_camera_rmse);
        the_log.SendData();
      }

      // Stop storing extra detections when the threshold is reached
      if ((ball_raw_pos_x_offline_queue.size() - ball_raw_pos_x_queue.size())
      >= extra_detections_number) {
        chip_kick_running = false;
        storing_further_detections = false;
      }
    } else {
        // Time the chip-kick estimation
        double computation_time = GetMonotonicTime();

        // TODO(srabiee) use bounded queue instead of the following queues
        // Update the current detection information queues
        ball_time_queue.push_back(ball_time);
        camera_id_queue.push_back(ball_camera_id);
        frame_number_queue.push_back(ball_frame_number);
        ball_raw_pos_x_queue.push_back(ball_raw_pos(0));
        ball_raw_pos_y_queue.push_back(ball_raw_pos(1));
        ball_raw_pos_z_queue.push_back(ball_zero_height);
        ball_raw_pos_x_offline_queue.push_back(ball_raw_pos(0));
        ball_raw_pos_y_offline_queue.push_back(ball_raw_pos(1));

        // Run chip-kick detection
        Vector3f initial_pos;
        Vector3f initial_vel;
        double time_init_optimized = 0.0;
        if (ball_time_queue.size() > chip_kick_min_frame_num &&
  chip_kick_running) {
          // Estimate the parameters of the projectile motion (full
          // optimization)
          if (tuning_mode == 1) {
            ChipKickDetection::FitParabola(&initial_pos,
                            &initial_vel,
                            ball_time_queue[0],
                            ball_time_queue,
                            ball_raw_pos_x_queue,
                            ball_raw_pos_y_queue,
                            ball_raw_pos_z_queue,
                            camera_id_queue,
                            cameras,
                            gravity_acc);
            time_init_optimized = ball_time_queue[0];

          // Partial optimization
          } else if (tuning_mode == 2) {
            Eigen::Vector3f initial_pos_known;
            initial_pos_known(0) =
                world_state_chip_kick_onset.GetBall().GetPosition()(0);
            initial_pos_known(1) =
                world_state_chip_kick_onset.GetBall().GetPosition()(1);
            initial_pos_known(2) = ball_zero_height;
            initial_pos = initial_pos_known;

            ChipKickDetection::FitParabolaPartialOptimization(&initial_vel,
              &time_init_optimized,
              initial_pos_known,
              onset_time,
              ball_time_queue,
              ball_raw_pos_x_queue,
              ball_raw_pos_y_queue,
              ball_raw_pos_z_queue,
              camera_id_queue,
              cameras,
              gravity_acc);
          }

          // Estimate the impact time and position
          double impact_time;
          Vector3f impact_point;
          if (ChipKickDetection::PredictBouncing(initial_pos,
          initial_vel,
          time_init_optimized,
          ball_zero_height + ball_zero_height_offset,
          gravity_acc,
          &impact_point,
          &impact_time)) {
            // No impact point was found
            impact_time = 0.0;
            impact_point << 0.0, 0.0, ball_zero_height;
          }

          //  Calculate the current ball position
          Vector3f current_pos;
          double delT = ball_time - time_init_optimized;
          current_pos.head(2) = initial_pos.head(2) + initial_vel.head(2) *
          delT;
          current_pos(2) = initial_pos(2) + initial_vel(2) * delT +
          0.5 * gravity_acc(2) * delT * delT;

          ball_pose_3d = current_pos;

          // Project the estimated impact point to the ground to
          // account for the offset in the camera ball_zero_height
          Vector2f projected_impact_point;
          Vector2f projected_impact_point_partial;
          ChipKickDetection::Project2GroundPlane(cameras[ball_camera_id],
                              impact_point,
                              ball_zero_height,
                              &projected_impact_point);
  //         impact_point_pos = impact_point;
          impact_point_pos.head(2) = projected_impact_point;
          impact_point_pos(2) = ball_zero_height;

          computation_time = GetMonotonicTime() - computation_time;
          computation_time_queue.push_back(computation_time);
          // Calculate the mean compuation time for chip-kick estimation
          double computation_time_mean = 0;
          for (size_t i = 0; i < computation_time_queue.size(); i++) {
            computation_time_mean += computation_time_queue[i];
          }
          computation_time_mean = computation_time_mean /
          static_cast<double>(computation_time_queue.size());
          computation_time_mean_queue.push_back(computation_time_mean);

          //  Stop updating the chip kick detection if
          //  the ball has reached the impact point
          if (fabs(impact_time - ball_time) < impact_time_thresh ||
            impact_time - ball_time < 0) {
            storing_further_detections = true;
            camera_tuning_running = true;
            std::cout << "Time of flight: " << impact_time -
                      time_init_optimized << endl << endl << endl << endl <<
                      endl << endl << endl << endl << endl << endl;
            std::cout << "Time init opt: " << time_init_optimized << endl;
            std::cout << "Time init: " << ball_time_queue[0] << endl;
          }

          Vector2f ball_pos_2d;
          // Project the estimated 3D ball position to the ground plane
          ChipKickDetection::Project2GroundPlane(cameras[ball_camera_id],
                              ball_pose_3d,
                              ball_zero_height,
                              &ball_pos_2d);

          ball_pose2d_x_queue.push_back(ball_pos_2d(0));
          ball_pose2d_y_queue.push_back(ball_pos_2d(1));
          ball_impact_point_x_queue.push_back(impact_point_pos(0));
          ball_impact_point_y_queue.push_back(impact_point_pos(1));
          ball_impact_time_queue.push_back(impact_time);

          // Recalculate the estimated position for all the points on the
          // current chip kick trajectory given the projectile
          // motion parameters estimated in the current time step. This is
          // used for evaluation purposes
          vector<Vector3f> ball_pose3d_batch_estimated;
          vector<float> ball_pose2d_x_batch_estimated;
          vector<float> ball_pose2d_y_batch_estimated;
          for (size_t k =0; k < ball_time_queue.size(); k++) {
            // Calculate the estimated 3d positions
            Vector3f current_pos;
            double delT = ball_time_queue[k] - time_init_optimized;
            current_pos.head(2) = initial_pos.head(2) + initial_vel.head(2) *
            delT;
            current_pos(2) = initial_pos(2) + initial_vel(2) * delT +
            0.5 * gravity_acc(2) * delT * delT;

            ball_pose3d_batch_estimated.push_back(current_pos);

            // Project the estimated 3d position to the ground plane
            Vector2f current_2d_pos;

ChipKickDetection::Project2GroundPlane(cameras[camera_id_queue[k]],
                                                  current_pos,
                                                  ball_zero_height,
                                                  &current_2d_pos);

            ball_pose2d_x_batch_estimated.push_back(current_2d_pos(0));
            ball_pose2d_y_batch_estimated.push_back(current_2d_pos(1));
          }

          // Calculates RMSE for chip kick estimation over the observed data
          // from the start of the latest chip kick. Use ball_pose2d_x_queue
          // instead of ball_pose2d_x_batch_estimated as argument to update
          // the RMSE by only taking into account the estimated value for the
          // current time step as opposed to re-estimating all the points on
          // the projectile.
          float rmse_chip_kick;
          ChipKickDetection::CalculateError(ball_raw_pos_x_queue,
                                            ball_raw_pos_y_queue,
                                            ball_pose2d_x_batch_estimated,
                                            ball_pose2d_y_batch_estimated,
                                            &rmse_chip_kick);

          chip_kick_rmse_queue.push_back(rmse_chip_kick);

        // Visualize the results of the chip kick detection on the viewer
        logger::NetLogger the_log2 = ChipKickDetection::VisualizeResults(
                                            ball_pose_3d,
                                            impact_point_pos,
                                            ball_raw_pos_x_queue,
                                            ball_raw_pos_y_queue,
                                            ball_raw_pos_x_queue,
                                            ball_raw_pos_y_queue,
                                            ball_pose2d_x_queue,
                                            ball_pose2d_y_queue,
                                            camera_id_queue,
                                            ball_impact_point_x_queue,
                                            ball_impact_point_y_queue,
                                            ball_impact_time_queue,
                                            chip_kick_rmse_queue,
                                            ball_time_queue,
                                            computation_time_queue,
                                            computation_time_mean_queue);

        // Limit the maximum duration of a chip kick in order to prevent
        // extra calculations when the impact point is not estimated correctly
        double current_time = GetMonotonicTime();
        if (fabs(current_time - real_time_capturing_start_time) >
            max_chip_kick_duration) {
          chip_kick_running = false;
          storing_further_detections = false;
        }

        // Run the camera tuner when data for one complete chip kick is recorded
        // Loop through different camera parameter values and anaylize the
        // effect on the estimation RMSE
        if (camera_tuning_running) {
          uint camera_under_test_id = camera_id_queue[0];
          // Check that all camera id's be the same
          for (size_t k = 0; k < camera_id_queue.size(); k++) {
            if (camera_id_queue[k] != camera_under_test_id) {
              " could be done on one camera at a time."<< endl << endl << endl;
              camera_tuning_running = false;
              return 0;
            }
          }

          ChipKickTester::CameraParams current_camera =
                                                  cameras[camera_under_test_id];
//           Eigen::Vector4f quat_step_size(0.02, 0.02, 0.02, 0.001);
          Eigen::Vector4f quat_step_size(0.002, 0.002, 0.002, 0.0005);
          Eigen::Vector3f translation_step_size(2, 2, 2);
          // The two step numbers should be odd
          int quat_step_num = 5;
          int translation_step_num = 5;

          float min_rmse;
          ChipKickTester::CameraParams tuned_camera;
          ParamSearch(quat_step_size,
                  translation_step_size,
                  quat_step_num,
                  translation_step_num,
                  current_camera,
                  &min_rmse,
                  &tuned_camera,
                  &min_rmse_log);
          tuned_camera_param = tuned_camera;
          original_camera_param = current_camera;
          tuned_camera_rmse = min_rmse;

          camera_tuning_running = false;
          camera_tuning_done = true;
        }

        // Log the result of the parameter tuner when finished
        if (camera_tuning_done) {
          the_log2.MergeLoggers(min_rmse_log);

          the_log2.Pop();
          the_log2.LogPrint("Tuned camera params vs. Original camera params");
          the_log2.Push();
          the_log2.LogPrint("q_w:  %2.5f,  %2.5f"
                  , tuned_camera_param.quat.w()
                  ,  original_camera_param.quat.w());
          the_log2.LogPrint("q_x:  %2.5f,  %2.5f"
                  , tuned_camera_param.quat.x()
                  , original_camera_param.quat.x());
          the_log2.LogPrint("q_y:  %2.5f,  %2.5f"
                  , tuned_camera_param.quat.y()
                  , original_camera_param.quat.y());
          the_log2.LogPrint("q_z:  %2.5f,  %2.5f"
                  , tuned_camera_param.quat.z()
                  , original_camera_param.quat.z());
          the_log2.LogPrint("t_x:  %6.5f,  %6.5f"
                          , tuned_camera_param.translation(0)
                          , original_camera_param.translation(0));
          the_log2.LogPrint("t_y:  %6.5f,  %6.5f"
                          , tuned_camera_param.translation(1)
                          , original_camera_param.translation(1));
          the_log2.LogPrint("t_z:  %6.5f,  %6.5f"
                          , tuned_camera_param.translation(2)
                          , original_camera_param.translation(2));
          the_log2.Pop();
          the_log2.LogPrint("Best RMSE");
          the_log2.Push();
          the_log2.LogPrint("%f", tuned_camera_rmse);
          the_log2.SendData();
        }
      }
    }
  }
  return 0;
}


