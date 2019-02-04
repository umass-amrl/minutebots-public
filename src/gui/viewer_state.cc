// Copyright 2018 afischer@umass.edu
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

#include "gui/viewer_state.h"

#include <stdio.h>
#include <glog/logging.h>
#include <signal.h>

#include <string>
#include <vector>
#include <random>

#include "soccer/executor.h"
#include "soccer/kalmanupdate.h"
#include "state/direction.h"
#include "state/position_velocity_state.h"
#include "third_party/optionparser-1.4/optionparser.h"
#include "util/colorize.h"

#include "QApplication"
#include "QtGui"

#include "constants/constants.h"
#include "gui/drawing_helpers.h"
#include "gui/viewer.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "state/team.h"
#include "logging/logger.h"

using team::Team;

using drawing_helpers::DrawLine;
using drawing_helpers::DrawPoint;
using Eigen::Vector2f;
using gui::Viewer;
using opengl_helpers::Color4f;
using MinuteBotsProto::SoccerDebugMessage;
using MinuteBotsProto::TextTree;
using std::vector;
using std::default_random_engine;
using std::normal_distribution;
using logger::Logger;

using app::Executor;
using app::KalmanUpdate;
using colorize::ColorCyan;
using colorize::ColorGreen;
using direction::Direction;
using logger::Logger;
using SSLVisionProto::SSL_WrapperPacket;
using state::SharedState;
using state::SoccerState;
using state::WorldState;
using state::PositionVelocityState;
using std::atomic_bool;
using std::bitset;
using std::condition_variable;
using std::endl;
using std::mutex;
using pose_2d::Pose2Df;
using std::pair;
using std::string;
using tactics::Tactic;
using tactics::TacticIndex;
using std::unique_lock;
using std::vector;
using team::Team;
using option::Arg;
using option::OptionIndex;
using threadsafe::ThreadSafePriorityQueue;
using threadsafe::ThreadSafeQueue;
using threadsafe::ThreadSafeActor;
using estimation::ExtendedKalmanFilter;
using state::SharedRobotState;

static const double kDeltaT = 1.0 / kTransmitFrequency;
Viewer* view;
ExtendedKalmanFilter ekf;
Pose2Df pose;
Pose2Df vel;

static default_random_engine generator;
// additive error to velocity each time step
static normal_distribution<double> x_trans_distr(0.0, 2.0);
static normal_distribution<double> v_trans_distr(0.0, 10.0);
static normal_distribution<double> rot_distr(0.0, 0.01);

static double cur_time = 0.0;

// returns a control function in (mm/s^2, mm/s^2, s^-2)
Pose2Df StateVisualizer::Control() {
  logger_.LogPrint("time = %f", cur_time);
  double x_accel = kMaxRobotAcceleration * cos(cur_time * 2 * M_PI);
  Pose2Df result;
  result.translation = Vector2f(x_accel, 0);
  result.angle = 0;

  return result;
}

// simulates robot for one timestep and updates pose and vel
// command is the acceleration that the robot will experience
void StateVisualizer::Simulate(Pose2Df accel) {
  pose.translation += vel.translation * kDeltaT
    + 0.5 * accel.translation * kDeltaT * kDeltaT;
  pose.angle += vel.angle * kDeltaT
    + 0.5 * accel.angle * kDeltaT * kDeltaT;
  vel.translation += accel.translation * kDeltaT;
  vel.angle += accel.angle * kDeltaT;
}

void StateVisualizer::Noise() {
  double noise_x = x_trans_distr(generator), noise_y = x_trans_distr(generator);
  double noise_vx = v_trans_distr(generator),
    noise_vy = v_trans_distr(generator);
  logger_.LogPrint("Translation noise: %f, %f", noise_x, noise_y);
  logger_.LogPrint("Velocity noise: %f, %f", noise_vx, noise_vy);
  logger_.LogPrint("Rotational vel. noise (deg/s): %f", 0 * 180 / M_PI);
  pose.translation.x() += noise_x;
  pose.translation.y() += noise_y;
  vel.translation.x() += noise_vx;
  vel.translation.y() += noise_vy;
}

void StateVisualizer::LogRobotState() {
  logger_.LogPrint("Robot loc: %04.5f, %04.5f", pose.translation.x(),
    pose.translation.y());
  logger_.LogPrint("Robot angle (deg): %3.2f", pose.angle);
  logger_.LogPrint("Robot vel: %04.5f, %04.5f", vel.translation.x(),
    vel.translation.y());
  logger_.LogPrint("Robot anglular vel (deg): %3.2f", vel.angle);
}

StateVisualizer::StateVisualizer(gui::Viewer* viewer) {
  viewer_ = viewer;
  logger_ = Logger();
  logger_.SetMessage(message_);
}

void StateVisualizer::KeyPressEvent(QKeyEvent* event) {
  switch (event->key()) {
    case Qt::Key_A: {
      logger_.Clear();
      viewer_->Update(logger_.ReturnMessage());
    } break;
    case Qt::Key_S: {
      logger_.Clear();
      cur_time += kDeltaT;
      Pose2Df accel = Control();
      logger_.LogPrint("Acceleration: %f, %f", accel.translation.x(),
        accel.translation.y());
      logger_.LogPrint("Angular accel (deg/s^2): %f", accel.angle);
      Simulate(accel);

      // generate command for EKF
      SharedRobotState cmd(0, 0);
      cmd.velocity_x = vel.translation.x();
      cmd.velocity_y = vel.translation.y();
      cmd.velocity_r = vel.angle;
      cmd.cmd_time = cur_time;
      cmd.ssl_vision_id = 0;
      ekf.UpdateCmd(cmd);

      Noise();

      // get EKF prediction
      logger_.LogPrintPush("EKF");
      ekf.Update(pose, cur_time, 0, 0, &logger_);
      Pose2Df ekf_pose = ekf.GetCurrentPose();
      logger_.Pop();

      Noise();
      LogRobotState();
      logger_.AddPoint(Vector2f(0, 0), 1, 1, 1, 1);
      logger_.AddRobot(0, MinuteBotsProto::RobotState_Team_TEAM_BLUE,
        pose.angle, pose.translation.x(), pose.translation.y(), 0);
      logger_.AddRobot(0, MinuteBotsProto::RobotState_Team_TEAM_YELLOW,
        ekf_pose.angle, ekf_pose.translation.x(), ekf_pose.translation.y(), 0);

      ekf.DrawCovariance(&logger_);
      viewer_->Update(logger_.ReturnMessage());
    } break;
  }
}

int main(int argc, char** argv) {
  generator.seed(1);
  ekf = ExtendedKalmanFilter(pose, kDeltaT, 0);
  vel.translation = Vector2f(0, 200);
  QApplication app(argc, argv);
  logger::ReadLogger log("viewer_state.log");
  view = new Viewer(NULL, &log, false);
  StateVisualizer visualizer(view);
  view->show();
  StateVisualizer::connect(view, SIGNAL(KeypressSignal(QKeyEvent*)),
                            &visualizer, SLOT(KeyPressEvent(QKeyEvent*)));
  int return_value = app.exec();
  return return_value;
}
