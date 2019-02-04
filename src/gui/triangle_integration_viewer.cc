// Copyright 2011-2017 dbalaban@cs.umass.edu
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

#include <stdio.h>
#include <random>
#include <string>
#include <vector>

#include "QApplication"
#include "QObject"
#include "QtGui"
#include "logging/logger.h"

#include "constants/constants.h"
#include "gui/viewer.h"
#include "motion_control/ntoc_2d.h"
#include "motion_control/optimal_control_1d.h"
#include "motion_control/tsocs.h"
#include "net/netraw.h"
#include "shared/common_includes.h"

#include "state_estimation/extended_kalman_filter.h"

STANDARD_USINGS;
using gui::Viewer;
using MinuteBotsProto::SoccerDebugMessage;
using MinuteBotsProto::TextTree;
using logger::ReadWriteLogger;
using pose_2d::Pose2Df;
using estimation::ExtendedKalmanFilter;

std::random_device r;
std::default_random_engine generator(r());

Viewer* view;

enum TriangleState { TOWARDA, TOWARDB, TOWARDC };

TriangleState execution_state;

float time0;

double kTimeStep = 1. / 125.;

Pose2Df poseA;
Pose2Df poseB;
Pose2Df poseC;
Pose2Df goal;

Pose2Df true_pose;
Pose2Df true_velocity;
Pose2Df velocity_command;
Pose2Df measured_pose;

Pose2Df desired_pose;
Pose2Df desired_velocity;

unsigned int rand_seed_ = 1;

void RunTriangleStep(ExtendedKalmanFilter* belief, ReadWriteLogger* the_log);

class MyThread : public QThread {
 protected:
  void run() {
    static const double kMinDuration = 1.0 / 125.0;
    unsigned int step_count = 0;
    ExtendedKalmanFilter belief(true_pose, step_count * kTimeStep);
    while (true) {
      ReadWriteLogger the_log("triangle_logging2.txt");
      ++step_count;
      if (!view->IsLive()) {
        view->Playback();
      }
      RunTriangleStep(&belief, &the_log);
      Matrix2f R;
      R << cos(belief.GetCurrentPose().angle),
          -sin(belief.GetCurrentPose().angle),
          sin(belief.GetCurrentPose().angle),
          cos(belief.GetCurrentPose().angle);
      velocity_command.translation = R * velocity_command.translation;
      belief.Update(measured_pose, step_count * kTimeStep, &the_log);
      SoccerDebugMessage debug_message = the_log.ReturnMessage();
      struct timeval tval;
      gettimeofday(&tval, NULL);
      debug_message.set_time(((tval.tv_sec * 1000000) + tval.tv_usec));
      // belief.Update(measured_pose,
      //     step_count * kTimeStep, belief.GetCurrentVelocity());
      view->Update(debug_message);
      Sleep(kMinDuration);
    }
  }

 public:
  explicit MyThread(QObject* parent = 0) {}
  ~MyThread() {}
};

void DrawWorldState(ReadWriteLogger* the_log, ExtendedKalmanFilter* belief) {
  the_log->AddPoint(true_pose.translation.x(), true_pose.translation.y(), 0, 0,
                    1, 1);

  the_log->AddPoint(poseA.translation.x(), poseA.translation.y(), 1, 0, 0, 1);
  the_log->AddPoint(poseB.translation.x(), poseB.translation.y(), 1, 0, 0, 1);
  the_log->AddPoint(poseC.translation.x(), poseC.translation.y(), 1, 0, 0, 1);

  the_log->AddLine(
      true_pose.translation.x(), true_pose.translation.y(),
      true_pose.translation.x() + true_velocity.translation.x() / 10,
      true_pose.translation.y() + true_velocity.translation.y() / 10, 0, 0, 1,
      1);

  the_log->LogPrint("True Pose: (%.3f, %.3f, %.3f)\n",
                    true_pose.translation.x(), true_pose.translation.y(),
                    true_pose.angle);
  the_log->LogPrint("True Velocity: (%.3f, %.3f, %.3f)\n",
                    true_velocity.translation.x(),
                    true_velocity.translation.y(), true_velocity.angle);

  Pose2Df believed_pose = belief->GetCurrentPose();
  Pose2Df believed_velocity = belief->GetCurrentVelocityWorldFrame();

  the_log->AddLine(
      believed_pose.translation.x(), believed_pose.translation.y(),
      believed_pose.translation.x() + believed_velocity.translation.x() / 10,
      believed_pose.translation.y() + believed_velocity.translation.y() / 10, 1,
      1, 0, 1);

  the_log->LogPrint("Believed Pose: (%.3f, %.3f, %.3f)\n",
                    believed_pose.translation.x(),
                    believed_pose.translation.y(), believed_pose.angle);
  the_log->LogPrint("Believed Velocity: (%.3f, %.3f, %.3f)\n",
                    believed_velocity.translation.x(),
                    believed_velocity.translation.y(), believed_velocity.angle);

  the_log->AddLine(
      desired_pose.translation.x(), desired_pose.translation.y(),
      desired_pose.translation.x() + desired_velocity.translation.x() / 10,
      desired_pose.translation.y() + desired_velocity.translation.y() / 10, 1,
      0, 0, 1);

  the_log->LogPrint("Desired Pose: (%.3f, %.3f, %.3f)\n",
                    desired_pose.translation.x(), desired_pose.translation.y(),
                    desired_pose.angle);
  the_log->LogPrint("Desired Velocity: (%.3f, %.3f, %.3f)\n",
                    desired_velocity.translation.x(),
                    desired_velocity.translation.y(), desired_velocity.angle);
}

Vector2f GetAverageLinearAccel(vector<ControlPhase2D> controls) {
  double time_remain = kTimeStep;
  Vector2f weighted_total(0, 0);

  for (unsigned int i = 0; i < controls.size(); i++) {
    ControlPhase2D control = controls[i];
    const Vector2f accel = control.acceleration;
    const double duration = control.duration;
    if (duration < time_remain) {
      weighted_total += accel * duration;
      time_remain -= duration;
    } else {
      weighted_total += accel * time_remain;
      time_remain = 0;
      break;
    }
  }

  return weighted_total / kTimeStep;
}

double GetAverageRotationalAccel(vector<ControlPhase1D> controls) {
  double time_remain = kTimeStep;
  double weighted_total = 0;

  for (unsigned int i = 0; i < controls.size(); i++) {
    ControlPhase1D control = controls[i];
    double accel = control.acceleration;
    const double duration = control.duration;
    if (duration < time_remain) {
      weighted_total += accel * duration;
      time_remain -= duration;
    } else {
      weighted_total += accel * time_remain;
      time_remain = 0;
      break;
    }
  }

  return weighted_total / kTimeStep;
}

void ApplyControl(ReadWriteLogger* the_log,
                  vector<ControlPhase2D> linear_control,
                  vector<ControlPhase1D> rotational_control,
                  Pose2Df believed_velocity, Pose2Df believed_pose) {
  const Vector2f linear_accel = GetAverageLinearAccel(linear_control);
  const double rot_accel = GetAverageRotationalAccel(rotational_control);

  the_log->LogPrint("Commanding Acceleration: (%.3f, %.3f)\n", linear_accel.x(),
                    linear_accel.y());

  desired_pose.angle = believed_pose.angle +
                       believed_velocity.angle * kTimeStep +
                       0.5 * rot_accel * kTimeStep * kTimeStep;
  desired_velocity.angle = believed_velocity.angle + rot_accel * kTimeStep;

  desired_velocity.translation =
      believed_velocity.translation + linear_accel * kTimeStep;
  desired_pose.translation = believed_pose.translation +
                             believed_velocity.translation * kTimeStep +
                             0.5 * linear_accel * kTimeStep * kTimeStep;

  velocity_command.angle = believed_velocity.angle + rot_accel * kTimeStep;
  velocity_command.translation =
      believed_velocity.translation + linear_accel * kTimeStep;

  std::normal_distribution<double> gauss1(
      0, 0.01 * velocity_command.translation.x());
  std::normal_distribution<double> gauss2(
      0, 0.01 * velocity_command.translation.y());
  std::normal_distribution<double> gauss3(0, 0.01 * velocity_command.angle);
  std::normal_distribution<double> gauss4(0, 25);
  std::normal_distribution<double> gauss5(0, 0.173);

  const Vector2f actuation_error(gauss1(generator), gauss2(generator));
  const double actuation_angle_error = gauss3(generator);
  const Vector2f measurement_noise(gauss4(generator), gauss4(generator));
  const double measurement_angle_noise = gauss5(generator);

  const Vector2f current_velocity = true_velocity.translation;
  const double current_angle_velocity = true_velocity.angle;

  true_velocity.translation = velocity_command.translation + actuation_error;
  true_velocity.angle = velocity_command.angle + actuation_angle_error;

  const Vector2f accel_actual =
      (true_velocity.translation - current_velocity) / kTimeStep;
  const double angle_accel_actual =
      (true_velocity.angle - current_angle_velocity) / kTimeStep;

  true_pose.translation = true_pose.translation + current_velocity * kTimeStep +
                          0.5 * accel_actual * kTimeStep * kTimeStep;
  true_pose.angle = true_pose.angle + current_angle_velocity * kTimeStep +
                    0.5 * angle_accel_actual * kTimeStep * kTimeStep;

  measured_pose.translation = true_pose.translation + measurement_noise;
  measured_pose.angle = true_pose.angle + measurement_angle_noise;
}

void LogControlSequence(ReadWriteLogger* the_log,
                        vector<ControlPhase2D> control) {
  the_log->LogPrint("Planned Control Sequence \n");
  the_log->Push();
  for (unsigned int i = 0; i < control.size(); i++) {
    ControlPhase2D phase = control[i];
    the_log->LogPrint("Accel = (%.3f, %.3f), Duration = %.3f \n",
                      phase.acceleration.x(), phase.acceleration.y(),
                      phase.duration);
  }
}

void RunTriangleStep(ExtendedKalmanFilter* belief, ReadWriteLogger* the_log) {
  DrawWorldState(the_log, belief);

  if ((goal.translation - belief->GetCurrentPose().translation).norm() < 20) {
    switch (execution_state) {
      case TOWARDA:
        execution_state = TOWARDB;
        goal = poseB;
        break;
      case TOWARDB:
        execution_state = TOWARDC;
        goal = poseC;
        break;
      case TOWARDC:
        execution_state = TOWARDA;
        goal = poseA;
        break;
    }
  }

  MotionModel motion_model(kMaxRobotAcceleration, kMaxRobotVelocity);
  vector<ControlPhase2D> linear_control;
  Pose2Df believed_pose = belief->GetCurrentPose();
  Pose2Df believed_velocity = belief->GetCurrentVelocityWorldFrame();
  NTOC2D(believed_pose.translation - goal.translation,
         believed_velocity.translation, motion_model, &linear_control);
  LogControlSequence(the_log, linear_control);

  vector<ControlPhase1D> rotational_control;
  TimeOptimalControlAnyFinal1D(believed_pose.angle, believed_velocity.angle, 0.,
                               0., 0., kMaxRobotRotAccel, kMaxRobotRotVel,
                               &rotational_control);

  ApplyControl(the_log, linear_control, rotational_control, believed_velocity,
               believed_pose);
}

int main(int argc, char** argv) {
  execution_state = TOWARDA;
  poseA.Set(0.0f, Vector2f(1000, 1000));
  poseB.Set(0.0f, Vector2f(2500, 2000));
  poseC.Set(0.0f, Vector2f(3000, -2000));
  goal = poseA;
  true_pose.Set(0.0f, Vector2f(0.0f, 0.0f));
  true_velocity.Set(0.0f, Vector2f(0.0f, 0.0f));

  time_t t = time(0);
  struct timeval tval;
  gettimeofday(&tval, NULL);
  struct tm* now = new tm();
  localtime_r(&t, now);
  std::stringstream ss;
  ss << "soccerlog_";
  ss << std::to_string(now->tm_mon).c_str() << "-";
  ss << std::to_string(now->tm_mday).c_str() << "-"
     << std::to_string(1900 + now->tm_year).c_str() << "_";
  ss << std::to_string((tval.tv_sec * 1000000) + tval.tv_usec).c_str()
     << ".txt";
  string file_name = ss.str();

  QApplication app(argc, argv);
  view = new Viewer(nullptr, file_name);
  view->show();
  MyThread thread;
  thread.start();
  int return_value = app.exec();
  thread.terminate();
  thread.wait();

  return return_value;
}
