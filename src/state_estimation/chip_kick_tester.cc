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

#include "state_estimation/chip_kick_tester.h"

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

using math_util::AngleMod;
using pose_2d::Pose2Df;
using SSLVisionProto::SSL_DetectionBall;
using SSLVisionProto::SSL_DetectionFrame;
using SSLVisionProto::SSL_DetectionRobot;
using SSLVisionProto::SSL_WrapperPacket;
using state::WorldRobot;
using state::WorldState;
using direction::Direction;
//  using std::atomic_bool;
//  using std::condition_variable;
using std::endl;
using std::vector;
using team::Team;


ChipKickTester::ChipKickTester() {
  gravity_acc << 0.0, 0.0, -9.8 * 1000;
  ball_pose_2d << 0.0f, 0.0f;
  ball_pose_3d << 0.0f, 0.0f, ball_zero_height;
  impact_point_pos << 0.0f, 0.0f, 0.0f;
  latest_frame_time = 0.0;
  chip_kick_running = false;
  estimated_initial_velocity << 0.0, 0.0, 0.0;
  non_obstacle_robot_index = 0;
  chip_kick_rmse_ = 10000;
  flat_kick_rmse_ = 10000;

  // Lab cameras' calibration parameters
//   cameras.resize(4);
//   cameras[0].quat.w() = -0.0140573;
//   cameras[0].quat.vec() = Vector3f(0.7, -0.7, 0.0);
//   cameras[0].translation = Vector3f(0.0, 1250.00, 3500.0);
//
//   // No camera 1 actually exists
//   cameras[1].quat.w() = 0.0;
//   cameras[1].quat.vec() = Vector3f(0.7, -0.7, 0.0);
//   cameras[1].translation = Vector3f(0.0, 1250.00, 3500.0);
//
//   // No camera 2 actually exists
//   cameras[2].quat.w() = 0.0;
//   cameras[2].quat.vec() = Vector3f(0.7, -0.7, 0.0);
//   cameras[2].translation = Vector3f(0.0, 1250.00, 3500.0);
//
//   cameras[3].quat.w() = 0.200502;
//   cameras[3].quat.vec() =  Vector3f(0.7, -0.7, 0.0);
//   cameras[3].translation = Vector3f(0.0, 1250.00, 3500.0);

  // Lab cameras' calibration parameters
  cameras.resize(4);
  cameras[0].quat.w() = -0.0112175;
  cameras[0].quat.vec() = Vector3f(0.0071366, 0.972598, -0.232114);
  cameras[0].translation = Vector3f(2303.48, -1468.45, 3888.0);

  // No camera 1 actually exists
  cameras[1].quat.w() = 0.0;
  cameras[1].quat.vec() = Vector3f(0.7, -0.7, 0.0);
  cameras[1].translation = Vector3f(0.0, 1250.00, 3500.0);

  // No camera 2 actually exists
  cameras[2].quat.w() = 0.0;
  cameras[2].quat.vec() = Vector3f(0.7, -0.7, 0.0);
  cameras[2].translation = Vector3f(0.0, 1250.00, 3500.0);

  cameras[3].quat.w() = 0.212604;
  cameras[3].quat.vec() = Vector3f(0.976513, -0.0179741, 0.0299621);
  cameras[3].translation = Vector3f(-2460.69, -1547.55, 3667);
}

ChipKickTester::~ChipKickTester() {
}


int ChipKickTester::ShotDetector(
          const vector<state::PositionVelocityState>& pos_vel_queue,
          state::SharedState* shared_state) {
  state::PositionVelocityState previous_pos_vel_state =
          pos_vel_queue[0];
  state::PositionVelocityState current_pos_vel_state =
          pos_vel_queue[1];

  float min_distance = 10000.0;
  float min_distance_new;
  Team closest_robot_team = THEIR;
  SSLVisionId closest_robot_id = 0;
  uint shot_happened = 0;
  uint robot_found = 0;

  // Parameters for detecting ball contact with robots
  const float distance_threshold = static_cast<float>(kRobotRadius) +
              static_cast<float>(kBallRadius);
//   const float del_vel_threshold = 000.0f;

  Eigen::Vector2f ball_pos =
              pos_vel_queue[0].GetBallPositionVelocity().observed_pose;
  Eigen::Vector2f ball_vel =
              pos_vel_queue[0].GetBallPositionVelocity().velocity;

  // Calculate the distance of the ball to all the robots in the previous world
  // state
  int count = -1;
  for (const state::PositionVelocityState::RobotPositionVelocity& our_robot :
      previous_pos_vel_state.GetOurTeamRobots()) {
    count++;
    if (static_cast<uint>(count) == non_obstacle_robot_index) {
//       continue;
    }
    float del_x = ball_pos(0) - our_robot.observed_pose.translation(0);
    float del_y = ball_pos(1) - our_robot.observed_pose.translation(1);
    float distance_to_ball = sqrt(del_x * del_x + del_y * del_y);

    if (distance_to_ball < min_distance) {
      min_distance = distance_to_ball;
      closest_robot_id = our_robot.ssl_vision_id;
      closest_robot_team = OUR;
    }
    robot_found = 1;
  }

  for (const state::PositionVelocityState::RobotPositionVelocity& their_robot :
      previous_pos_vel_state.GetTheirTeamRobots())  {
    float del_x = ball_pos(0) - their_robot.observed_pose.translation(0);
    float del_y = ball_pos(1) - their_robot.observed_pose.translation(1);
    float distance_to_ball = sqrt(del_x * del_x + del_y * del_y);

    if (distance_to_ball < min_distance) {
      min_distance = distance_to_ball;
      closest_robot_id = their_robot.ssl_vision_id;
      closest_robot_team = THEIR;
    }
    robot_found = 1;
  }

  // If no robots are found, no shot could have happened
  if (!robot_found) {
    return 0;
  }

  // Check the distance of the ball to the closest robot in the latest world
  // state to check if it has moved away from it
  state::PositionVelocityState::RobotPositionVelocity robot_of_interest;
  bool previous_closest_robot_found = false;
  if (closest_robot_team == THEIR) {
    for (const state::PositionVelocityState::RobotPositionVelocity&
         their_robot : current_pos_vel_state.GetTheirTeamRobots())  {
        if (their_robot.ssl_vision_id == closest_robot_id) {
          robot_of_interest = their_robot;
          previous_closest_robot_found = true;
        }
      }
  } else {
    for (const state::PositionVelocityState::RobotPositionVelocity&
         our_robot : current_pos_vel_state.GetOurTeamRobots())  {
      if (our_robot.ssl_vision_id == closest_robot_id) {
        robot_of_interest = our_robot;
        previous_closest_robot_found = true;
      }
    }
  }

  if (!previous_closest_robot_found) {
    shot_happened = 0;
    return shot_happened;
  }


  Eigen::Vector2f ball_pos_new =
    pos_vel_queue[1].GetBallPositionVelocity().observed_pose;
  Eigen::Vector2f ball_vel_new =
    pos_vel_queue[1].GetBallPositionVelocity().velocity;
  float del_x = ball_pos_new(0) -
                              robot_of_interest.observed_pose.translation(0);
  float del_y = ball_pos_new(1) -
                              robot_of_interest.observed_pose.translation(1);

  min_distance_new = sqrt(del_x * del_x + del_y * del_y);

  // Calculate the change in velocity of the ball
  Eigen::Vector2f del_vel = ball_vel_new - ball_vel;
  float del_vel_abs = sqrt((del_vel(0) * del_vel(0)) + (del_vel(1) *
                      del_vel(1)));

//   std::cout << "previous ball dist: " << min_distance - distance_threshold <<
//       endl;
//
//   std::cout << "current ball dist: " << min_distance_new - distance_threshold
//       << endl;

  // Check if the robot has lost possesion of the ball
  if (min_distance < distance_threshold &&
    min_distance_new >= distance_threshold) {
//     && del_vel_abs > del_vel_threshold

      if (closest_robot_team == OUR) {
        state::SharedRobotState* robot_shared_state =
            shared_state->GetSharedStateByID(robot_of_interest.ssl_vision_id);
        if (robot_shared_state != NULL) {
          if (robot_shared_state->chip_kick_set) {
            shot_happened = 1;

            if (kDetailedDebuggingInfo) {
              std::cout << "Robot# " << static_cast<uint>(closest_robot_id) <<
              " lost possesion of ball"<< endl;
              std::cout << "Del_vel = " << del_vel_abs << endl;
              std::cout << "Initial ball position: " << ball_pos.transpose() <<
                  endl;
            }
          }
        }
      }
    }


  return shot_happened;
}

int ChipKickTester::OfflineAnalysis(logger::Logger* local_logger) {
  // Go through the captured ball detections and extract the
  // first projectile motion
  for (size_t n = 0; n < ball_time_offline_queue.size(); n++) {
    double ball_time = ball_time_offline_queue[n];
    uint ball_camera_id = camera_id_offline_queue[n];
    Eigen::Vector2f ball_raw_pos;
    ball_raw_pos(0) = ball_raw_pos_x_offline_queue[n];
    ball_raw_pos(1) = ball_raw_pos_y_offline_queue[n];

    // TODO(srabiee) use bounded queue instead of the following queues
    // Update the current detection information queues
    ball_time_queue.push_back(ball_time);
    camera_id_queue.push_back(ball_camera_id);
    ball_raw_pos_x_queue.push_back(ball_raw_pos(0));
    ball_raw_pos_y_queue.push_back(ball_raw_pos(1));
    ball_raw_pos_z_queue.push_back(ball_zero_height);

    // Run chip-kick detection
    Vector3f initial_pos;
    Vector3f initial_vel;
    if (ball_time_queue.size() > chip_kick_min_frame_num &&
chip_kick_running) {
      // Estimate the parameters of the projectile motion
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

      // Estimate the impact time and position
      double impact_time;
      Vector3f impact_point;
      if (ChipKickDetection::PredictBouncing(initial_pos,
      initial_vel,
      ball_time_queue[0],
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
      double delT = ball_time - ball_time_queue[0];
      current_pos.head(2) = initial_pos.head(2) + initial_vel.head(2) *
      delT;
      current_pos(2) = initial_pos(2) + initial_vel(2) * delT +
      0.5 * gravity_acc(2) * delT * delT;

      ball_pose_3d = current_pos;

      // Project the estimated impact point to the ground to
      // account for the offset in the camera ball_zero_height
      Vector2f projected_impact_point;
      ChipKickDetection::Project2GroundPlane(cameras[ball_camera_id],
                          impact_point,
                          ball_zero_height,
                          &projected_impact_point);
//         impact_point_pos = impact_point;
      impact_point_pos.head(2) = projected_impact_point;
      impact_point_pos(2) = ball_zero_height;

      //  Stop updating the chip kick detection if
      //  the ball has reached the impact point
      if (fabs(impact_time - ball_time) < impact_time_thresh ||
        impact_time - ball_time < 0) {
        chip_kick_running = false;
        if (kDetailedDebuggingInfo) {
          std::cout << "Time of flight: " << impact_time - ball_time_queue[0]
          << endl;
        }
        // Exit the loop and stop reading the captured SSL Vision data
        break;
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
      // current chip kick trajectory given the projectile motion parameters
      // estimated in the current time step. This is used for evaluation
      // purposes
      vector<Vector3f> ball_pose3d_batch_estimated;
      vector<float> ball_pose2d_x_batch_estimated;
      vector<float> ball_pose2d_y_batch_estimated;
      for (size_t k =0; k < ball_time_queue.size(); k++) {
        // Calculate the estimated 3d positions
        Vector3f current_pos;
        double delT = ball_time_queue[k] - ball_time_queue[0];
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
      // instead of ball_pose2d_x_batch_estimated as argument to update the
      // RMSE by only taking into account the estimated value for the
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
      ChipKickDetection::VisualizeResults(ball_pose_3d,
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
                                          computation_time_mean_queue,
                                          kDetailedDebuggingInfo,
                                          direction_,
                                          local_logger);
    }
    if (n == (ball_time_offline_queue.size() - 1)) {
      chip_kick_running = false;
      std::cout << "Reached end of captured queue: " << endl << endl;
      // Exit the loop and stop reading the captured SSL Vision data
      break;
    }
  }
  return 0;
}

int ChipKickTester::Update(
            const state::PositionVelocityState& position_velocity_state,
            state::SharedState* shared_state,
            logger::Logger* local_logger) {
  if (test_mode == 3 || test_mode == 4 || test_mode == 5) {
    in_the_game_mode = true;
  } else {
    in_the_game_mode = false;
  }

//   float confidence = 0.0f;
  Eigen::Vector2f ball_raw_pos(0.0f, 0.0f);
  double ball_time = 0;
  uint ball_camera_id = 0;

  // This part was commented after stopping the use of SSLVision detection
  // frames ********
//   for (const SSL_DetectionBall& ball : detection_frame_latest.balls()) {
//     if (ball.confidence() > confidence) {
//       confidence = ball.confidence();
//       ball_raw_pos = Eigen::Vector2f(ball.x(), ball.y());
//       ball_time = detection_frame_latest.t_capture();
//       ball_camera_id = detection_frame_latest.camera_id();
//       ball_frame_number = detection_frame_latest.frame_number();
//     }
//   }
//   // Do not update the state if the current detection has low conficence
//   if (confidence < confidece_thresh) {
//     // Non-confident reading
//     if (detection_frame_latest.balls().size() > 0)
//       std::cout << "Non_confident Reading..." << endl;
//     return 0;
//   }
  // ****************

  ball_raw_pos =
          position_velocity_state.GetBallPositionVelocity().observed_pose;
  if (direction_ != Direction::POSITIVE) {
    ball_raw_pos(0) = -ball_raw_pos(0);
    ball_raw_pos(1) = -ball_raw_pos(1);
  }
  ball_time = position_velocity_state.GetBallPositionVelocity().observed_time;
  ball_camera_id =
          position_velocity_state.GetBallPositionVelocity().last_camera_index;

  // Do not update the state if no new ball is detected
  if (!previous_ball_detections_time.empty()) {
    double delta_t = fabs(previous_ball_detections_time.back() - ball_time);
    if (delta_t <= kEpsilon) {
      return 0;
    }
  }

  // Check if a shot has been performed
  if (in_the_game_mode) {
    if (pos_vel_state_queue.size() < 2) {
      pos_vel_state_queue.push_back(position_velocity_state);
    } else {
      pos_vel_state_queue[0] = pos_vel_state_queue[1];
      pos_vel_state_queue[1] = position_velocity_state;
    }

    if (pos_vel_state_queue.size() >= 2) {
      shot_detected = ShotDetector(pos_vel_state_queue, shared_state);
    }
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

    // This line was CHANGED because raw detection frames are not being used
    // anymore.
//     onset_time_candidate = previous_ball_detections_time[size - 1 - 2];
    onset_time_candidate = previous_ball_detections_time[size - 1 - 1];
  }


//   std::cout << "Latest ball positions and times: " << endl;
//   for (size_t k = 0; k < previous_ball_positions.size(); k++) {
//     std::cout << previous_ball_positions[k].transpose() << " :: " <<
//     std::setprecision(15) << previous_ball_detections_time[k] << endl;
//   }

  // TODO(srabiee): replace the following reset procedure
  // by a method that takes into account the collisions of
  // ball with the robots as potential starting times for
  // a chip-kick
  double current_time = GetMonotonicTime();
  if ((in_the_game_mode && shot_detected) || (!in_the_game_mode &&
      fabs(current_time - latest_frame_time) > 3)) {
    if (kDetailedDebuggingInfo) {
      std::cout << "Started Capturing..." << endl;
    }

    if (test_mode == 1) {
      offline_data_capturing = true;
      chip_kick_running = false;
    } else if (test_mode == 2 || in_the_game_mode) {
      offline_data_capturing = false;
      chip_kick_running = true;
      storing_further_detections = false;
      if (in_the_game_mode) {
        pos_vel_state_chip_kick_onset = pos_vel_state_queue.front();
      }
    }
    offline_capturing_start_time = GetMonotonicTime();
    real_time_capturing_start_time = GetMonotonicTime();

//     chip_kick_running = true;
    ball_time_queue.clear();
    camera_id_queue.clear();
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
    chip_kick_rmse_ = 10000;
    flat_kick_rmse_ = 10000;
    ball_pose_3d << 0.0f, 0.0f, ball_zero_height;
    impact_point_pos << 0.0f, 0.0f, 0.0f;
    if (kDetailedDebuggingInfo) {
      std::cout << "Chip-kick detection is reset" << endl << endl;
    }

    // Clear vectors of offline capturing
    ball_time_offline_queue.clear();
    camera_id_offline_queue.clear();
    ball_raw_pos_x_offline_queue.clear();
    ball_raw_pos_y_offline_queue.clear();
    ball_raw_pos_z_offline_queue.clear();

    // Clear variables regarding onset detection
    previous_ball_positions.clear();
    previous_ball_detections_time.clear();
    onset_time = onset_time_candidate;
  }
  latest_frame_time = current_time;
  if (!offline_data_capturing && !chip_kick_running) {
//     std::cout << "ball detection not captured..." << endl;
  }

  // Capture some data for offline chip kick estimation
  if (offline_data_capturing) {
    double current_time = GetMonotonicTime();
    if ((current_time - offline_capturing_start_time) > capturing_duration) {
      chip_kick_running = true;
      offline_data_capturing = false;
      std::cout << "stopped capturing: " << ball_time_offline_queue.size()
      << " data points" << endl << endl;
    } else {
      ball_time_offline_queue.push_back(ball_time);
      camera_id_offline_queue.push_back(ball_camera_id);
      ball_raw_pos_x_offline_queue.push_back(ball_raw_pos(0));
      ball_raw_pos_y_offline_queue.push_back(ball_raw_pos(1));
      ball_raw_pos_z_offline_queue.push_back(ball_zero_height);
    }
  }

  if (chip_kick_running) {
    // If in offline test mode (test_mode == 1) go through the
    // captured data to extract the first projectile motion
    if (test_mode == 1) {
      OfflineAnalysis(local_logger);
    } else if (test_mode == 2 || in_the_game_mode) {
      // If chip_kick estimation has already reached the impact point, just
      // store a few more ball detections for visualization purposes
      if (storing_further_detections) {
        ball_raw_pos_x_offline_queue.push_back(ball_raw_pos(0));
        ball_raw_pos_y_offline_queue.push_back(ball_raw_pos(1));

        // Visualize the results of the chip kick detection on the viewer
        if (test_mode == 5) {
          ChipKickDetection::VisualizeResultsCompare(ball_pose_3d,
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
                        ball_impact_point_x_queue_2nd,
                        ball_impact_point_y_queue_2nd,
                        ball_impact_time_queue_2nd,
                        chip_kick_rmse_queue,
                        ball_time_queue,
                        computation_time_queue,
                        computation_time_mean_queue,
                        local_logger);
        } else {
          ChipKickDetection::VisualizeResults(ball_pose_3d,
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
                                            computation_time_mean_queue,
                                            kDetailedDebuggingInfo,
                                            direction_,
                                            local_logger);
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
          ball_raw_pos_x_queue.push_back(ball_raw_pos(0));
          ball_raw_pos_y_queue.push_back(ball_raw_pos(1));
          ball_raw_pos_z_queue.push_back(ball_zero_height);
          ball_raw_pos_x_offline_queue.push_back(ball_raw_pos(0));
          ball_raw_pos_y_offline_queue.push_back(ball_raw_pos(1));

          // Run chip-kick detection
          Vector3f initial_pos;
          Vector3f initial_vel;
          Vector3f initial_pos_partial;
          Vector3f initial_vel_partial;
          double time_init_optimized = 0.0;
          double time_init_optimized_partial;
          if (ball_time_queue.size() > chip_kick_min_frame_num &&
    chip_kick_running) {
            // Estimate the parameters of the projectile motion (full
            // optimization)
            if (test_mode == 2 || test_mode == 4) {
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

            } else if (test_mode == 3) {
              Eigen::Vector3f initial_pos_known;
              initial_pos_known(0) =
       pos_vel_state_chip_kick_onset.GetBallPositionVelocity().observed_pose(0);
              initial_pos_known(1) =
       pos_vel_state_chip_kick_onset.GetBallPositionVelocity().observed_pose(1);
              if (direction_ != Direction::POSITIVE) {
                initial_pos_known(0) = - initial_pos_known(0);
                initial_pos_known(1) = - initial_pos_known(1);
              }
              initial_pos_known(2) = ball_zero_height;
              initial_pos = initial_pos_known;

              ChipKickDetection::FitParabolaPartialOptimization(&initial_vel,
                &time_init_optimized,
                &chip_kick_rmse_,
                &flat_kick_rmse_,
                initial_pos_known,
                onset_time,
                ball_time_queue,
                ball_raw_pos_x_queue,
                ball_raw_pos_y_queue,
                ball_raw_pos_z_queue,
                camera_id_queue,
                cameras,
                gravity_acc);
              // Running and comparing both full and partial optimization
            } else if (test_mode == 5) {
              // Run full optimization
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

              // Run partial optimization
              Eigen::Vector3f initial_pos_known;
              initial_pos_known(0) =
       pos_vel_state_chip_kick_onset.GetBallPositionVelocity().observed_pose(0);
              initial_pos_known(1) =
       pos_vel_state_chip_kick_onset.GetBallPositionVelocity().observed_pose(1);
              if (direction_ != Direction::POSITIVE) {
                initial_pos_known(0) = - initial_pos_known(0);
                initial_pos_known(1) = - initial_pos_known(1);
              }
              initial_pos_known(2) = ball_zero_height;
              initial_pos_partial = initial_pos_known;


              ChipKickDetection::FitParabolaPartialOptimization(
                &initial_vel_partial,
                &time_init_optimized_partial,
                &chip_kick_rmse_,
                &flat_kick_rmse_,
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

            estimated_initial_velocity = initial_vel;
            // Estimate the impact time and position
            double impact_time;
            Vector3f impact_point;
            double impact_time_partial;
            Vector3f impact_point_partial;
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

            // Estimate the impact time and position for partial optimization
            // when comparing both methods
            if (test_mode == 5) {
              if (ChipKickDetection::PredictBouncing(initial_pos_partial,
                initial_vel_partial,
                time_init_optimized_partial,
                ball_zero_height + ball_zero_height_offset,
                gravity_acc,
                &impact_point_partial,
                &impact_time_partial)) {
                  // No impact point was found
                  impact_time_partial = 0.0;
                  impact_point_partial << 0.0, 0.0, ball_zero_height;
                }
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

            // Do the same projection for predicted impact point by parital
            // optimization
            if (test_mode == 5) {
              ChipKickDetection::Project2GroundPlane(cameras[ball_camera_id],
                                impact_point_partial,
                                ball_zero_height,
                                &projected_impact_point_partial);
              impact_point_pos_partial.head(2) = projected_impact_point_partial;
              impact_point_pos_partial(2) = ball_zero_height;

              float new_x_pos = impact_point_pos_partial(0);
              ball_impact_point_x_queue_2nd.push_back(new_x_pos);

              float new_y_pos = impact_point_pos_partial(1);
              ball_impact_point_y_queue_2nd.push_back(new_y_pos);
              ball_impact_time_queue_2nd.push_back(impact_time_partial);
            }

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
              if (kDetailedDebuggingInfo) {
                        std::cout << "Time of flight: " << impact_time -
                  time_init_optimized << endl;
              }
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
          if (test_mode != 5) {
            ChipKickDetection::VisualizeResults(ball_pose_3d,
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
                                                computation_time_mean_queue,
                                                kDetailedDebuggingInfo,
                                                direction_,
                                                local_logger);
          } else {
            ChipKickDetection::VisualizeResultsCompare(ball_pose_3d,
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
                                    ball_impact_point_x_queue_2nd,
                                    ball_impact_point_y_queue_2nd,
                                    ball_impact_time_queue_2nd,
                                    chip_kick_rmse_queue,
                                    ball_time_queue,
                                    computation_time_queue,
                                    computation_time_mean_queue,
                                    local_logger);
          }

          // Limit the maximum duration of a chip kick in order to prevent
          // extra calculations when the impact point is not estimated correctly
          double current_time = GetMonotonicTime();
          if (fabs(current_time - real_time_capturing_start_time) >
              max_chip_kick_duration) {
            chip_kick_running = false;
            storing_further_detections = false;
          }
        }
      }
    }
  }
  return 0;
}

Vector2f ChipKickTester::GetImpactPoint() const {
  Vector2f impact_point_2d;
  if (direction_ == Direction::POSITIVE) {
    impact_point_2d(0) = impact_point_pos(0);
    impact_point_2d(1) = impact_point_pos(1);
  } else {
    impact_point_2d(0) = -impact_point_pos(0);
    impact_point_2d(1) = -impact_point_pos(1);
  }
  return impact_point_2d;
}

Vector3f ChipKickTester::GetInitialVel() const {
  if (direction_ == Direction::POSITIVE) {
      return estimated_initial_velocity;
  } else {
      return -estimated_initial_velocity;
  }
}

double ChipKickTester::GetChipShotTime() const {
  return onset_time;
}


double ChipKickTester::GetImpactTime() const {
  if (!ball_impact_time_queue.empty()) {
    return ball_impact_time_queue.back();
  } else {
    return -1;
  }
}

bool ChipKickTester::IsEstimationReady() const {
  if (!chip_kick_rmse_queue.empty()) {
    if (chip_kick_running && chip_kick_rmse_ < kRMSEThresh_) {
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

bool ChipKickTester::IsChipKickDetected() const {
  return IsEstimationReady();
}

bool ChipKickTester::IsChipKickOnsetDetected() const {
  return chip_kick_running;
}

void ChipKickTester::UpdateCameraParams(const SSLVisionProto::SSL_GeometryData&
geometry_packet) {
  for (int k = 0; k < geometry_packet.calib_size(); k++) {
    uint32_t cam_id = geometry_packet.calib(k).camera_id();
    if (cam_id >= cameras.size()) {
      LOG(FATAL) << "Cam id index out of range!";
    }
    cameras[cam_id].quat.w() = geometry_packet.calib(k).q3();
    cameras[cam_id].translation(0) = geometry_packet.calib(k).tx();
    cameras[cam_id].translation(1) = geometry_packet.calib(k).ty();
    cameras[cam_id].translation(2) = geometry_packet.calib(k).tz();

    cameras[cam_id].quat.vec() = Vector3f(geometry_packet.calib(k).q0(),
                                          geometry_packet.calib(k).q1(),
                                          geometry_packet.calib(k).q2());
  }
}


void ChipKickTester::Reset() {
  ball_pose_2d << 0.0f, 0.0f;
  ball_pose_3d << 0.0f, 0.0f, ball_zero_height;
  impact_point_pos << 0.0f, 0.0f, 0.0f;
  chip_kick_running = false;
  storing_further_detections = false;
  estimated_initial_velocity << 0.0, 0.0, 0.0;
  latest_frame_time = 0.0;
  chip_kick_rmse_ = 10000;
  flat_kick_rmse_ = 10000;

  ball_time_queue.clear();
  camera_id_queue.clear();
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
  // Clear vectors of offline capturing
  ball_time_offline_queue.clear();
  camera_id_offline_queue.clear();
  ball_raw_pos_x_offline_queue.clear();
  ball_raw_pos_y_offline_queue.clear();
  ball_raw_pos_z_offline_queue.clear();

  // Clear variables regarding onset detection
  previous_ball_positions.clear();
  previous_ball_detections_time.clear();
}

