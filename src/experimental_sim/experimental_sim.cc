// Copyright 2018 - 2019 jaholtz@cs.umass.edu
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

#include "experimental_sim/experimental_sim.h"

#include <glog/logging.h>
#include <condition_variable>
#include <iostream>
#include <queue>
#include <thread>
#include <vector>

#include "constants/constants.h"
#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include "messages_robocup_ssl_wrapper.pb.h"
#include "util/colorize.h"
#include "util/timer.h"

STANDARD_USINGS;
using colorize::ColorGreen;
using net::UDPMulticastServer;
using pose_2d::Pose2Df;
using SSLVisionProto::SSL_DetectionBall;
using SSLVisionProto::SSL_DetectionFrame;
using SSLVisionProto::SSL_DetectionRobot;
using SSLVisionProto::SSL_FieldCicularArc;
using SSLVisionProto::SSL_FieldLineSegment;
using SSLVisionProto::SSL_GeometryData;
using SSLVisionProto::SSL_GeometryFieldSize;
using SSLVisionProto::SSL_WrapperPacket;
using state::PositionVelocityState;
using std::atomic_bool;
using std::cerr;
using std::condition_variable;
using std::cout;
using std::endl;
using std::lock_guard;
using std::mutex;
using std::queue;
using std::string;
using std::thread;
using std::vector;

namespace experimental_simulator {

ExperimentalSim::ExperimentalSim(double step_time,
                                   SimState* world_state,
                                   const unsigned int queue_size)
    : step_time_(step_time),
      ball_acceleration_(kBallAcceleration),
      localization_noise_stddev_(0.0015),
      angle_localization_noise_stddev_(0.0),
      current_time_(0.0),
      frame_index_(0),
      random_(),
      world_state_(world_state) {
        InitializeControlQueue(queue_size);
  }

ExperimentalSim::ExperimentalSim(double step_time,
                                   double current_time,
                                   int frame_index,
                                   SimState* world_state)
    : step_time_(step_time),
    ball_acceleration_(kBallAcceleration),
    localization_noise_stddev_(0),
    angle_localization_noise_stddev_(0.0),
    current_time_(current_time),
    frame_index_(frame_index),
    random_(),
    world_state_(world_state) {
      InitializeControlQueue(1);
  }

ExperimentalSim::ExperimentalSim()
    : step_time_(kTransmitPeriodSeconds),
      ball_acceleration_(kBallAcceleration),
      localization_noise_stddev_(0.0),
      angle_localization_noise_stddev_(0.0),
      current_time_(0.0),
      frame_index_(0),
      random_(),
      world_state_(nullptr) {}

Simulator* ExperimentalSim::Copy() {
  ExperimentalSim* to_copy = new ExperimentalSim(
      step_time_, current_time_, frame_index_, world_state_);
  return to_copy;
}

ExperimentalSim::~ExperimentalSim() {}

bool IsInCameraField(const Vector2f& position, const int camera_id) {
  bool in_frame = false;
  switch (camera_id) {
    case 0:
      in_frame = ((position.x() >= 0) && (position.y() >= 0));
      break;
    case 1:
      in_frame = ((position.x() <= 0) && (position.y() >= 0));
      break;
    case 2:
      in_frame = ((position.x() <= 0) && (position.y() <= 0));
      break;
    case 3:
      in_frame = ((position.x() >= 0) && (position.y() <= 0));
      break;
    default:
      in_frame = ((position.x() >= 0) && (position.y() <= 0));
  }
  return in_frame;
}

bool ExperimentalSim::ValidateVelocityCommandIntegrity(
    const RadioProtocolWrapper& velocity_command) {
    if (velocity_command.command_size() == 0) {
      LOG(ERROR) << "No velocity commands given to simulator!";
      return false;
    }
    return true;
}

void ExperimentalSim::InitializeControlQueue(const unsigned int queue_size) {
    for (unsigned int i = 0; i < queue_size; i++) {
      RadioProtocolWrapper rpw = GetEmptyRadioCommand();
      control_queue_.push(rpw);
    }
}

RadioProtocolWrapper ExperimentalSim::GetEmptyRadioCommand() {
    RadioProtocolWrapper rpw;

    for (const Robot& robot : world_state_->GetRobots()) {
      RadioProtocolCommand* command = rpw.add_command();

      command->set_velocity_x(robot.GetVelocity().x()/1000);
      command->set_velocity_y(robot.GetVelocity().y()/1000);
      command->set_velocity_r(robot.rotational_velocity_);

      command->set_robot_id(robot.GetId());

      command->set_dribbler_spin(0.0);
      command->set_flat_kick(0.0);
      command->set_chip_kick(0.0);
    }

    return rpw;
}

vector<SSL_WrapperPacket> ExperimentalSim::GetSSLWrapperPackets() {
    // Setup the messages for the four cameras with default constructed
    // SSL_WrapperPackets.
    vector<SSL_WrapperPacket> wrapper_packets(kNumCameras);
    for (size_t camera_id = 0;
         camera_id < wrapper_packets.size(); ++camera_id) {
      SSL_WrapperPacket& wrapper_packet = wrapper_packets[camera_id];
      SSL_DetectionFrame* detection = wrapper_packet.mutable_detection();
      // If we need to define field lines again, look at commit:
      // 5e130f22d8d0224df26ff84d8c1da86f3aa400b8

    detection->set_frame_number(frame_index_);
    // TODO(jaholtz): Add lag between time captured and time sent?
    auto time_capture = current_time_;
    detection->set_t_capture(time_capture);
    detection->set_t_sent(time_capture);
    detection->set_camera_id(camera_id);  // ID associated with the proto.

    // Check to see if ball is within this camera's frame.
    // Frames are numbered as per this chart:
    // https://raw.githubusercontent.com/RoboCup-SSL/ssl-vision/wiki/images/4cam_setup.png
    const Pose2Df& ball_position = world_state_->GetBall().GetPose2Df();

    if (IsInCameraField(ball_position.translation, camera_id)) {
      SSL_DetectionBall* ball = detection->add_balls();
      ball->set_confidence(1);  // Full confidence.
      ball->set_x(ball_position.translation.x());
      ball->set_y(ball_position.translation.y());
      ball->set_pixel_x(0);
      ball->set_pixel_y(0);
    }

    for (const Robot& robot : world_state_->GetRobots()) {
      if (random_.UniformRandom() > kSimulatorPacketLossPercent) {
        const Pose2Df& pose = robot.GetPose();
        const float velocity_norm = robot.GetVelocity().norm();

        SSL_DetectionRobot* ssl_robot = nullptr;
        if (IsInCameraField(pose.translation, camera_id)) {
          if (robot.GetTeam() == team::Team::BLUE) {
            ssl_robot = detection->add_robots_blue();
          } else {
            ssl_robot = detection->add_robots_yellow();
          }

          ssl_robot->set_robot_id(robot.GetId());
          ssl_robot->set_confidence(1);
          if (localization_noise_stddev_ > 0) {
            ssl_robot->set_x(
                random_.Gaussian(pose.translation.x(),
                                 localization_noise_stddev_ * velocity_norm));
            ssl_robot->set_y(
                random_.Gaussian(pose.translation.y(),
                                 localization_noise_stddev_ * velocity_norm));
            ssl_robot->set_orientation(
                random_.Gaussian(pose.angle, angle_localization_noise_stddev_));
          } else {
            ssl_robot->set_x(pose.translation.x());
            ssl_robot->set_y(pose.translation.y());
            ssl_robot->set_orientation(pose.angle);
          }
          ssl_robot->set_pixel_x(0);
          ssl_robot->set_pixel_y(0);
        }
      }
    }
  }
  return wrapper_packets;
}

PositionVelocityState ExperimentalSim::GetWorldState(const team::Team team) {
  PositionVelocityState world_pvs;
  Ball ball = world_state_->GetBall();
  // Initialize a ball (camera is chosen arbitrarily)
  PositionVelocityState::BallPositionVelocity ball_pvs(
      ball.GetPose2Df().translation,
      ball.GetVelocity(),
      ball.GetPose2Df().translation,
      current_time_,
      1);
  *(world_pvs.GetMutableBallPositionVelocity()) = ball_pvs;
  vector<Robot> robots = world_state_->GetRobots();
  for (Robot robot : robots) {
    pose_2d::Pose2Df velocity = {robot.rotational_velocity_,
                                 robot.GetVelocity()};
    state::PositionVelocityState::RobotPositionVelocity robot_pvs(
        robot.GetId(),
        robot.GetPose(),
        velocity,
        robot.GetPose(),
        velocity,
        current_time_,
        1);
    if (robot.GetTeam() == team) {
      world_pvs.GetMutableOurTeamRobots()->InsertBack(robot_pvs);
    } else {
      world_pvs.GetMutableTheirTeamRobots()->InsertBack(robot_pvs);
    }
  }
  return world_pvs;
}

void ExperimentalSim::SimulateStep(const RadioProtocolWrapper& new_command) {
    const RadioProtocolWrapper next_command = control_queue_.front();
    control_queue_.pop();
    control_queue_.push(new_command);
    if (ValidateVelocityCommandIntegrity(next_command)) {
      world_state_->Update(next_command, step_time_);
      world_state_->SetBallAccel(ball_acceleration_);
    }
    last_command_ = next_command;
    current_time_ += step_time_;
    frame_index_++;
}

MinuteBotsProto::FactorTuningData ExperimentalSim::GetFactorChanges() {
  return factor_data_;
}

void ExperimentalSim::SetFactors(MinuteBotsProto::FactorSettings settings) {
  settings.Clear();
  if (settings.has_ball_accel()) {
    ball_acceleration_ = settings.ball_accel();
    factor_data_.add_adjusted_factors(true);
    factor_data_.add_factor_adjustments(settings.ball_accel());
  } else {
    factor_data_.add_adjusted_factors(false);
  }
  if (settings.has_control_error()) {
    localization_noise_stddev_ = settings.control_error();
    factor_data_.add_adjusted_factors(true);
    factor_data_.add_factor_adjustments(settings.control_error());
  } else {
    factor_data_.add_adjusted_factors(false);
  }
  if (settings.has_step_time()) {
    step_time_ = settings.step_time();
    factor_data_.add_adjusted_factors(true);
    factor_data_.add_factor_adjustments(settings.step_time());
  } else {
    factor_data_.add_adjusted_factors(false);
  }
}

}  // namespace experimental_simulator
