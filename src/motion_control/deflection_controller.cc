// Copyright 2017 - 2018 joydeepb@cs.umass.edu, dbalaban@cs.umass.edu
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

#include "motion_control/deflection_controller.h"
#include "math/math_util.h"
#include "motion_control/motion_model.h"

using ntoc::ControlPhase1D;
using ntoc::ControlSequence1D;
using motion::MotionModel;
using math_util::Sign;

const double kTimeStep = 1 / 60;
const double kMinimumRobotCollisionSpeed = 10;
const double kKickThreshold = 500;

const Eigen::Vector2d GetReflection(Vector2d velocity_init,
                                    const double robot_angle) {
  const Vector2d normal(cos(robot_angle), sin(robot_angle));
  const Vector2d tangent(-sin(robot_angle), cos(robot_angle));
  Eigen::Matrix2d R;
  R << normal, tangent;
  Eigen::Matrix2d Damping;
  // Damping << -kCollisionDamping, 0, 0, kCollisionDamping;
  Eigen::Vector2d boost(kKickBoost, 0);

  // return R * (Damping * R.inverse() * velocity_init + boost);
  return R * (kCollisionDamping * R.inverse() * velocity_init + boost);
}

const double GetRobotAngle(const Vector2d velocity_init,
                           const Vector2d velocity_final,
                           const double min_angle, const double max_angle,
                           const double angle) {
  const double kThreshold = 1e-12;
  const Eigen::Vector2d reflected = GetReflection(velocity_init, angle);
  const double desired_traject = atan2(velocity_final.y(), velocity_final.x());
  const double obtained_traject = atan2(reflected.y(), reflected.x());
  const double diff = desired_traject - obtained_traject;
  if (fabs(diff) < kThreshold) {
    return angle;
  } else if (diff > 0) {
    return GetRobotAngle(velocity_init, velocity_final, angle, max_angle,
                         (max_angle + angle) / 2);
  } else {
    return GetRobotAngle(velocity_init, velocity_final, min_angle, angle,
                         (min_angle + angle) / 2);
  }
}

const double GetBallTravelTime(const double ball_speed, const double dist) {
  double t1 = 0, t2 = 0;
  SolveQuadratic(-.5 * kBallAcceleration, ball_speed, -dist, &t1, &t2);
  if (t1 > 0) {
    return t1;
  } else {
    return t2;
  }
}

const double GetRobotTravelTime(const double robot_speed, const double dist,
                                const double accel) {
  double t1 = 0, t2 = 0;
  SolveQuadratic(-.5 * accel, robot_speed, -dist, &t1, &t2);
  if (t1 > 0) {
    return t1;
  } else {
    return t2;
  }
}

bool WillBallHitGoal(Vector2d ball_pose, Vector2d goal,
                     Vector2d ball_velocity) {
  const double dist1 = (ball_pose - goal).norm();
  const double time = GetBallTravelTime(ball_velocity.norm(), dist1);
  const Vector2d new_pos =
      ball_pose + ball_velocity * time -
      .5 * ball_velocity.normalized() * kBallAcceleration * time * time;
  const double dist2 = (new_pos - goal).norm();
  return dist2 < kBallRadius;
}

pose_2d::Pose2Dd MoveToRest(Pose2Dd robot_velocity) {
  Vector2d new_linear_vel;
  double new_angular_vel;

  const double speed_dec = -kMaxRobotAcceleration * kTimeStep;
  if (robot_velocity.translation.norm() < fabs(speed_dec)) {
    new_linear_vel << 0, 0;
  } else {
    new_linear_vel = robot_velocity.translation -
                     robot_velocity.translation.normalized() *
                         kMaxRobotAcceleration * kTimeStep;
  }

  const double rot_speed_dec = -kMaxRobotRotAccel * kTimeStep;
  if (robot_velocity.angle < fabs(rot_speed_dec)) {
    new_angular_vel = 0;
  } else {
    new_angular_vel = robot_velocity.angle + rot_speed_dec;
  }

  Pose2Dd new_velocity;
  new_velocity.translation = new_linear_vel;
  new_velocity.angle = new_angular_vel;
  return new_velocity;
}

pose_2d::Pose2Dd DeflectControl(const Pose2Dd& robot_pose,
                                const Vector2d& ball_pose,
                                const Pose2Dd& robot_velocity,
                                const Vector2d& ball_velocity,
                                const Vector2d& goal) {
  const Vector2d rest_velocity(0, 0);
  const Vector2d robot_to_goal = goal - robot_pose.translation;
  const Vector2d goal_dir = robot_to_goal.normalized();
  const Vector2d ball_dir = ball_velocity.normalized();
  const Vector2d ball_accel = -kBallAcceleration * ball_dir;

  if (WillBallHitGoal(ball_pose, goal, ball_velocity)) {
    return MoveToRest(robot_velocity);
  }

  // solve for the interception point
  // sineal_sol gives distances between robot / ball and collision point
  Eigen::Matrix2d linear_eqs;
  linear_eqs << goal_dir, -ball_dir;
  const Vector2d linear_sol =
      linear_eqs.inverse() * (ball_pose - robot_pose.translation);

  if (linear_sol[1] < 0) {
    LOG(WARNING) << "BALL IS MOVING AWAY FROM COLLISION POINT, "
                 << "SOLUTION INFEASIBLE, RETURNING DO NOTHING CONTROL";
    return robot_velocity;
  }
  if (linear_sol[0] < 0) {
    Pose2Dd new_velocity = robot_velocity;
    new_velocity.translation = robot_velocity.translation -
                               kMaxRobotAcceleration * goal_dir * kTimeStep;
    return new_velocity;
  }

  // get robot and ball positions at interception
  // const Vector2d ball_collision_pos = ball_pose + ball_dir * linear_sol[1];
  const Vector2d robot_collision_pos =
      (linear_sol[0] - kRobotRadius - kBallRadius) * goal_dir +
      robot_pose.translation;

  const double ball_speed = ball_velocity.norm();
  const double ball_range = Sq(ball_speed) / (2 * kBallAcceleration);
  if (ball_range < linear_sol[1]) {
    LOG(WARNING) << "BALL IS NOT MOVING FAST ENOUGH TO REACH COLLISION POINT, "
                 << "SUGGEST USING BALL INTERCEPT OR KICK AT TARGET INSTEAD, "
                 << "RETURNING DO NOTHING CONTROL";
    Pose2Dd new_velocity = robot_velocity;
    new_velocity.translation = robot_velocity.translation -
                               kMaxRobotAcceleration * goal_dir * kTimeStep;
    return new_velocity;
  }

  const double time_to_collision = GetBallTravelTime(ball_speed, linear_sol[1]);
  const Vector2d intcpt_ball_vel =
      ball_velocity + ball_accel * time_to_collision;

  const double desired_robot_angle =
      GetRobotAngle(intcpt_ball_vel, goal_dir, -M_PI, M_PI, 0);

  const Vector2d normal(cos(desired_robot_angle), sin(desired_robot_angle));
  const double speed_gap = kKickThreshold + intcpt_ball_vel.dot(normal);
  double target_robot_speed;
  if (speed_gap < kMinimumRobotCollisionSpeed) {
    target_robot_speed = kMinimumRobotCollisionSpeed;
  } else {
    target_robot_speed = speed_gap / goal_dir.dot(normal);
  }

  ControlSequence1D linear_control;
  ControlSequence1D angular_control;
  const double time_optimal = TimeOptimalControlAnyFinal1D(
      robot_pose.translation, robot_velocity.translation, robot_collision_pos,
      target_robot_speed * goal_dir, 0, kMaxRobotAcceleration,
      kMaxRobotVelocity, &linear_control);
  TimeOptimalControlAnyFinal1D(robot_pose.angle, robot_velocity.angle,
                               desired_robot_angle, 0, 0, kMaxRobotRotAccel,
                               kMaxRobotRotVel, &angular_control);
  if (time_optimal > time_to_collision) {
    LOG(WARNING) << "NOT ENOUGH TIME TO REACH COLLISION";
  }

  pose_2d::Pose2Dd new_velocity;

  if (fabs(robot_pose.angle - desired_robot_angle) <
          .5 * kMaxRobotRotAccel * Sq(kTimeStep) &&
      fabs(robot_velocity.angle) < kMaxRobotRotAccel * kTimeStep) {
    new_velocity.angle = 0;
  } else if (angular_control.phases[0].duration < kTimeStep) {
    const double time_remain = kTimeStep - angular_control.phases[0].duration;
    const double accel_weighted =
        (angular_control.phases[0].acceleration *
             angular_control.phases[0].duration +
         angular_control.phases[1].acceleration * time_remain) /
        kTimeStep;
    new_velocity.angle = robot_velocity.angle + accel_weighted * kTimeStep;
  } else {
    new_velocity.angle = robot_velocity.angle +
                         angular_control.phases[0].acceleration * kTimeStep;
  }

  const double robot_vel_1d =
      robot_velocity.translation.norm() *
      Sign(robot_velocity.translation.normalized().dot(goal_dir));
  const double time_direct_accel_to_target_pos = GetRobotTravelTime(
      robot_vel_1d, (robot_collision_pos - robot_pose.translation).norm(),
      kMaxRobotAcceleration);

  // if (fabs(time_direct_accel_to_target_pos - time_to_collision) < kTimeStep
  //    && target_robot_speed >=  model.lin_accel_max * kTimeStep)
  if (abs(static_cast<int>(time_direct_accel_to_target_pos / kTimeStep) -
          static_cast<int>(time_to_collision / kTimeStep)) < 1 &&
      target_robot_speed >= kMaxRobotAcceleration * kTimeStep) {
    new_velocity.translation = robot_velocity.translation +
                               kMaxRobotAcceleration * goal_dir * kTimeStep;
    return new_velocity;
  } else if (fabs(time_direct_accel_to_target_pos - time_to_collision) <
                 kTimeStep &&
             fabs(target_robot_speed - robot_velocity.translation.norm()) <
                 kMaxRobotAcceleration * kTimeStep) {
    new_velocity.translation = target_robot_speed * goal_dir;
  }

  if (fabs(time_optimal - time_to_collision) < kTimeStep) {
    new_velocity.translation =
        robot_velocity.translation +
        linear_control.phases[0].acceleration * goal_dir * kTimeStep;
    return new_velocity;
  }

  const double time_accel_rest_target_speed =
      (target_robot_speed - robot_vel_1d) / kMaxRobotAcceleration;
  double dist_rest_target_speed =
      .5 * kMaxRobotAcceleration * Sq(time_accel_rest_target_speed);
  if (dist_rest_target_speed < kRobotRadius) {
    dist_rest_target_speed = 1 * kRobotRadius;
  }
  const Vector2d ideal_rest_pos =
      robot_collision_pos - dist_rest_target_speed * goal_dir;

  ControlSequence1D linear_control_rest;
  const double time_to_rest = TimeOptimalControlAnyFinal1D(
      robot_pose.translation, robot_velocity.translation, ideal_rest_pos,
      rest_velocity, 0, kMaxRobotAcceleration, kMaxRobotVelocity,
      &linear_control_rest);

  if (time_to_rest + time_accel_rest_target_speed < time_to_collision) {
    if ((robot_pose.translation - ideal_rest_pos).norm() < kEpsilon &&
        robot_velocity.translation.norm() < kMaxRobotAcceleration * kTimeStep) {
      new_velocity.translation = rest_velocity;
      return new_velocity;
    }
    const double accel =
        linear_control_rest.phases[0].acceleration *
        Sign(goal_dir.dot(ideal_rest_pos - robot_pose.translation));
    new_velocity.translation =
        robot_velocity.translation + accel * goal_dir * kTimeStep;
    return new_velocity;
  }
  return MoveToRest(robot_velocity);
}
