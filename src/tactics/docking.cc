// Copyright 2018 - 2019 slane@cs.umass.edu
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

#include "tactics/docking.h"

#include <algorithm>
#include <cmath>
#include "constants/constants.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "evaluators/defense_evaluation.h"
#include "evaluators/offense_evaluation.h"
#include "math/geometry.h"
#include "math/math_util.h"
#include "math/poses_2d.h"
#include "motion_control/ntoc_2d.h"
#include "radio_protocol_wrapper.pb.h"
#include "state/shared_state.h"
#include "state/soccer_state.h"
#include "state/world_robot.h"
#include "state/world_state.h"

#define _USE_MATH_DEFINES

STANDARD_USINGS;
using defense::DefenseEvaluator;
using Eigen::Vector2f;
using Eigen::Matrix2f;
using pose_2d::Pose2Df;
using state::WorldState;
using state::WorldRobot;
using state::SoccerRobot;
using state::SharedState;
using state::SharedRobotState;
using std::atan2;
using std::endl;
using std::map;
using std::tan;
using std::min;
using std::max;
using std::unique_ptr;
using tactics::TacticIndex;
using geometry::Angle;
using geometry::Perp;
using geometry::Heading;
using geometry::LineLineIntersection;
using geometry::ProjectPointOntoLineSegment;
using geometry::ProjectPointOntoLine;
using geometry::RayIntersect;
using math_util::AngleDiff;
using math_util::Sign;

namespace tactics {

  Docking::Docking(const string& machine_name,
                   const WorldState& world_state,
                   TacticArray* tactic_list,
                   SharedState* shared_state,
                   OurRobotIndex our_robot_index,
                   state::SoccerState* soccer_state)
    : StateMachineTactic(machine_name,
                         world_state,
                         tactic_list,
                         shared_state,
                         our_robot_index,
                         soccer_state),
      start_(std::bind(&Docking::Start, this), "Start"),
      s1_left_(std::bind(&Docking::RotateLeft, this), "S1_RotateLeft"),
      s1_right_(std::bind(&Docking::RotateRight, this), "S1_RotateRight"),
      s1_forward_(std::bind(&Docking::Forward, this), "S1_Forward"),
      s2_left_(std::bind(&Docking::RotateLeft, this), "S2_RotateLeft"),
      s2_right_(std::bind(&Docking::RotateRight, this), "S2_RotateRight"),
      s2_forward_(std::bind(&Docking::Forward, this), "S2_Forward"),
      backward_(std::bind(&Docking::Backward, this), "Backward"),
      docked_(std::bind(&Docking::Docked, this), "Docked"),
      s1_align_(100.0, "s1_align", this),
      s1_right_angle_(100.0, "s1_right", this),
      s1_left_angle_(100.0, "s1_left", this),
      perp_y_(100.0, "perp_y", this),
      s2_right_angle_(100.0, "s2_right", this),
      s2_left_angle_(100.0, "s2_left", this),
      perp_point(-1000, -(static_cast<float>(s1_align_))),
      s1_align_lower_((static_cast<float>(s1_align_) / 2) - 5) {
    state_ = start_;
  }

void Docking::Init() {}

void Docking::SendVelocity() {
  SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);
  const Vector2f desired = {state->velocity_x, state->velocity_y};
  const float desired_r = state->velocity_r;
  const Vector2f velocity =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity.translation;
  const float r =
      world_state_.GetOurRobotPosition(our_robot_index_).velocity.angle;
  const float angle =
      world_state_.GetOurRobotPosition(our_robot_index_).position.angle;
  const float r_diff = desired_r - r;

  Eigen::Rotation2Df robot_frame(-angle);
  const Vector2f v_diff = desired - (robot_frame * velocity);

  if (r_diff > 0.1) {
    state->acceleration_command.angle = kMaxRobotRotAccel;
  } else if (r_diff < 0.1) {
    state->acceleration_command.angle = -kMaxRobotRotAccel;
  } else {
    state->acceleration_command.angle = 0.0;
  }

  if (v_diff.x() > 0) {
    state->acceleration_command.translation.x() = kMaxRobotAcceleration / 2;
  } else if (v_diff.x() < 0) {
    state->acceleration_command.translation.x() = -kMaxRobotAcceleration / 2;
  } else {
    state->acceleration_command.translation.x() = 0;
  }

  if (v_diff.y() > 0) {
    state->acceleration_command.translation.y() = kMaxRobotAcceleration / 2;
  } else if (v_diff.y() < 0) {
    state->acceleration_command.translation.y() = -kMaxRobotAcceleration / 2;
  } else {
    state->acceleration_command.translation.y() = 0;
  }
}

void Docking::Start() {
  SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);
  state->velocity_x = 0;
  state->velocity_y = 0;
  state->velocity_r = 0;
  SendVelocity();
}

void Docking::RotateLeft() {
  SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);
  state->velocity_x = 0;
  state->velocity_y = 0;
  state->velocity_r = 1;
  SendVelocity();
}

void Docking::RotateRight() {
  SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);
  state->velocity_x = 0;
  state->velocity_y = 0;
  state->velocity_r = -1;
  SendVelocity();
}

void Docking::Forward() {
  SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);
  // Non-Forward movement simulates control drift.
  state->velocity_x = 500;
  state->velocity_y = 2;
  state->velocity_r = .08;
  SendVelocity();
}

void Docking::Backward() {
  SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);
  state->velocity_x = -500;
  state->velocity_y = 0.0;
  state->velocity_r = 0.0;
  SendVelocity();
}

void Docking::Docked() {
  SharedRobotState* state = shared_state_->GetSharedState(our_robot_index_);
  state->velocity_x = 0;
  state->velocity_y = 0;
  state->velocity_r = 0;
  SendVelocity();
}

bool Docking::ShouldStage1Forward() {
  const Pose2Df current_pose =
  world_state_.GetOurRobotPosition(our_robot_index_).observed_pose;

  const float robot_heading = current_pose.angle;

  // Dock is facing the opposite direction from the desired
  // docking angle for the robot.
  const Eigen::Vector2f dock_direction = Heading(-dock_angle);


  const Vector2f y_dir(-sin(robot_heading), cos(robot_heading));

  Eigen::Vector2f perpendicular_point;
  Eigen::Vector2f dock_point2 = dock_location + dock_direction;
  ProjectPointOntoLine(current_pose.translation,
                       dock_location,
                       dock_point2,
                       &perpendicular_point);

  const Vector2f robot_to_perp_displace =
      perpendicular_point - current_pose.translation;

  perpendicular_point.y() = perpendicular_point.y()
      - s1_align_lower_;

  const Vector2f robot_to_final_displace =
  perpendicular_point - current_pose.translation;

  float perp_x_dist = robot_to_perp_displace.y();

  const Vector2f robot_to_perp_point =
      perp_point - current_pose.translation;

  float perp_y_dist = robot_to_perp_point.x();

  const float perp_point_angle = Angle(robot_to_perp_point);

  const float perp_point_angle_diff = RadToDeg(AngleDiff(robot_heading,
                                                         perp_point_angle));

  const float robot_to_perp_angle = Angle(robot_to_final_displace);

  float perp_rot_diff = AngleDiff(robot_heading,
                                  robot_to_perp_angle);
  perp_rot_diff = RadToDeg(perp_rot_diff);


  // SET POTENTIAL TRANSITION
  potential_state_ = "STAGE1_GOFORWARD";
  // ADD A BLOCK OF CLAUSES
  AddBlock(true);
  // SET AND FOR CLAUSES
  and_clause_ = true;

  const bool perp_unaligned = perp_x_dist > s1_align_lower_;

  // SET AND FOR CLAUSES (next clause(s) are ORs)
  and_clause_ = false;

  const bool perp_unaligned_lower = perp_x_dist < s1_align_lower_;

  // ADD ANOTHER BLOCK(TRUE) THIS IS:
  //    (CLAUSES PREVIOUS TO THIS) && (UPCOMING CLAUSES)
  AddBlock(true);

  // SET AND FOR CLAUSES (next clause(s) are ANDs)
  and_clause_ = true;

  const bool y_unaligned = fabs(perp_y_dist) > perp_y_;

  const bool perp_rotate_right =
      perp_point_angle_diff < s1_right_angle_;

  const bool perp_rotate_left =
      -perp_point_angle_diff < s1_left_angle_;

  const bool stage1_forward =
      (perp_unaligned || perp_unaligned_lower) &&
      (y_unaligned && perp_rotate_left && perp_rotate_right);

  return stage1_forward;
}

bool Docking::ShouldStage2Forward() {
  const Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).observed_pose;

  const float robot_heading = current_pose.angle;

  // Dock is facing the opposite direction from the desired
  // docking angle for the robot.
  const Eigen::Vector2f dock_direction = Heading(-dock_angle);

  const Vector2f y_dir(-sin(robot_heading), cos(robot_heading));

  Eigen::Vector2f perpendicular_point;
  Eigen::Vector2f dock_point2 = dock_location + dock_direction;
  ProjectPointOntoLine(current_pose.translation,
                       dock_location,
                       dock_point2,
                       &perpendicular_point);

  const Vector2f robot_to_perp_displace =
      perpendicular_point - current_pose.translation;

  perpendicular_point.y() = perpendicular_point.y() - s1_align_lower_;

  const Vector2f robot_to_final_displace =
      perpendicular_point - current_pose.translation;

  float perp_x_dist = robot_to_perp_displace.y();

  const Vector2f robot_to_perp_point =
      perp_point - current_pose.translation;

  float perp_y_dist = robot_to_perp_point.x();

  const float perp_point_angle = Angle(robot_to_perp_point);

  const float perp_point_angle_diff = RadToDeg(AngleDiff(robot_heading,
                                                         perp_point_angle));

  const float robot_to_perp_angle = Angle(robot_to_final_displace);

  float perp_rot_diff = AngleDiff(robot_heading,
                                  robot_to_perp_angle);
  perp_rot_diff = RadToDeg(perp_rot_diff);


  // SET POTENTIAL TRANSITION
  potential_state_ = "STAGE2_GOFORWARD";
  // ADD A BLOCK OF CLAUSES
  AddBlock(true);
  // SET AND FOR CLAUSES
  and_clause_ = true;

  const bool perp_unaligned = perp_x_dist > s1_align_lower_;

  // SET AND FOR CLAUSES (next clause(s) are ORs)
  and_clause_ = false;

  const bool perp_unaligned_lower = perp_x_dist < s1_align_lower_;

  // ADD ANOTHER BLOCK(TRUE) THIS IS:
  //    (CLAUSES PREVIOUS TO THIS) && (UPCOMING CLAUSES)
  AddBlock(true);

  // SET AND FOR CLAUSES (next clause(s) are ANDs)
  and_clause_ = true;

  const bool y_unaligned = fabs(perp_y_dist) > perp_y_;

  const bool perp_rotate_right =
      perp_point_angle_diff < s2_right_angle_;

  const bool perp_rotate_left =
      -perp_point_angle_diff < s2_left_angle_;

  const bool stage2_forward =
      (perp_unaligned || perp_unaligned_lower) &&
      (y_unaligned && perp_rotate_left && perp_rotate_right);

  return stage2_forward;
}

bool Docking::ShouldStage1Right() {
  const Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).observed_pose;

  const float robot_heading = current_pose.angle;

  // Dock is facing the opposite direction from the desired
  // docking angle for the robot.
  const Eigen::Vector2f dock_direction = Heading(-dock_angle);

  const Vector2f y_dir(-sin(robot_heading), cos(robot_heading));

  Eigen::Vector2f perpendicular_point;
  Eigen::Vector2f dock_point2 = dock_location + dock_direction;
  ProjectPointOntoLine(current_pose.translation,
                       dock_location,
                       dock_point2,
                       &perpendicular_point);

  const Vector2f robot_to_perp_displace =
      perpendicular_point - current_pose.translation;

  perpendicular_point.y() = perpendicular_point.y()
      - s1_align_lower_;

  const Vector2f robot_to_final_displace =
      perpendicular_point - current_pose.translation;

  float perp_x_dist = robot_to_perp_displace.y();

  const Vector2f robot_to_perp_point =
      perp_point - current_pose.translation;

  float perp_y_dist = robot_to_perp_point.x();

  const float perp_point_angle = Angle(robot_to_perp_point);

  const float perp_point_angle_diff = RadToDeg(AngleDiff(robot_heading,
                                                         perp_point_angle));

  const float robot_to_perp_angle = Angle(robot_to_final_displace);

  float perp_rot_diff = AngleDiff(robot_heading,
                                  robot_to_perp_angle);
  perp_rot_diff = RadToDeg(perp_rot_diff);


  // SET POTENTIAL TRANSITION
  potential_state_ = "STAGE1_ROTATERIGHT";
  // ADD A BLOCK OF CLAUSES
  AddBlock(true);
  // SET AND FOR CLAUSES
  and_clause_ = true;
  const bool perp_unaligned = perp_x_dist > s1_align_;

  // SET AND FOR CLAUSES (next clause(s) are ORs)
  and_clause_ = false;

  const bool perp_unaligned_lower = perp_x_dist < s1_align_lower_;


  const bool y_unaligned = fabs(perp_y_dist) > perp_y_;

  AddBlock(true);

  and_clause_ = true;

  const bool perp_rotate_right =
      perp_point_angle_diff > s1_right_angle_;

  const bool stage1_rotate_right =
      (perp_unaligned || perp_unaligned_lower || y_unaligned)
      && perp_rotate_right;

  return stage1_rotate_right;
}

bool Docking::ShouldStage2Right() {
  const Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).observed_pose;

  const float robot_heading = current_pose.angle;

  // Dock is facing the opposite direction from the desired
  // docking angle for the robot.
  const Eigen::Vector2f dock_direction = Heading(-dock_angle);

  const Vector2f y_dir(-sin(robot_heading), cos(robot_heading));

  Eigen::Vector2f perpendicular_point;
  Eigen::Vector2f dock_point2 = dock_location + dock_direction;
  ProjectPointOntoLine(current_pose.translation,
                       dock_location,
                       dock_point2,
                       &perpendicular_point);

  const Vector2f robot_to_perp_displace =
      perpendicular_point - current_pose.translation;

  perpendicular_point.y() = perpendicular_point.y()
      - s1_align_lower_;

  const Vector2f robot_to_final_displace =
      perpendicular_point - current_pose.translation;

  float perp_x_dist = robot_to_perp_displace.y();

  const Vector2f robot_to_perp_point =
      perp_point - current_pose.translation;

  float perp_y_dist = robot_to_perp_point.x();

  const float perp_point_angle = Angle(robot_to_perp_point);

  const float perp_point_angle_diff = RadToDeg(AngleDiff(robot_heading,
                                                         perp_point_angle));

  const float robot_to_perp_angle = Angle(robot_to_final_displace);

  float perp_rot_diff = AngleDiff(robot_heading,
                                  robot_to_perp_angle);
  perp_rot_diff = RadToDeg(perp_rot_diff);


  // SET POTENTIAL TRANSITION
  potential_state_ = "STAGE2_ROTATERIGHT";
  // ADD A BLOCK OF CLAUSES
  AddBlock(true);
  // SET AND FOR CLAUSES
  and_clause_ = true;
  const bool perp_unaligned = perp_x_dist > s1_align_;

  // SET AND FOR CLAUSES (next clause(s) are ORs)
  and_clause_ = false;

  const bool perp_unaligned_lower = perp_x_dist < s1_align_lower_;


  const bool y_unaligned = fabs(perp_y_dist) > perp_y_;

  AddBlock(true);

  and_clause_ = true;

  const bool perp_rotate_right =
      perp_point_angle_diff > s2_right_angle_;

  const bool stage2_rotate_right =
      (perp_unaligned || perp_unaligned_lower || y_unaligned)
      && perp_rotate_right;

  return stage2_rotate_right;
}

bool Docking::ShouldStage1Left() {
  const Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).observed_pose;

  const float robot_heading = current_pose.angle;

  // Dock is facing the opposite direction from the desired
  // docking angle for the robot.
  const Eigen::Vector2f dock_direction = Heading(-dock_angle);

  const Vector2f y_dir(-sin(robot_heading), cos(robot_heading));

  Eigen::Vector2f perpendicular_point;
  Eigen::Vector2f dock_point2 = dock_location + dock_direction;
  ProjectPointOntoLine(current_pose.translation,
                       dock_location,
                       dock_point2,
                       &perpendicular_point);

  const Vector2f robot_to_perp_displace =
      perpendicular_point - current_pose.translation;

  perpendicular_point.y() = perpendicular_point.y()
      - s1_align_lower_;

  const Vector2f robot_to_final_displace =
      perpendicular_point - current_pose.translation;

  float perp_x_dist = robot_to_perp_displace.y();

  const Vector2f robot_to_perp_point =
      perp_point - current_pose.translation;

  float perp_y_dist = robot_to_perp_point.x();

  const float perp_point_angle = Angle(robot_to_perp_point);

  const float perp_point_angle_diff = RadToDeg(AngleDiff(robot_heading,
                                                         perp_point_angle));

  const float robot_to_perp_angle = Angle(robot_to_final_displace);

  float perp_rot_diff = AngleDiff(robot_heading,
                                  robot_to_perp_angle);
  perp_rot_diff = RadToDeg(perp_rot_diff);


  // SET POTENTIAL TRANSITION
  potential_state_ = "STAGE1_ROTATELEFT";
  // ADD A BLOCK OF CLAUSES
  AddBlock(true);
  // SET AND FOR CLAUSES
  and_clause_ = true;
  const bool perp_unaligned = perp_x_dist > s1_align_;

  // SET AND FOR CLAUSES (next clause(s) are ORs)
  and_clause_ = false;

  const bool perp_unaligned_lower = perp_x_dist < s1_align_lower_;


  const bool y_unaligned = fabs(perp_y_dist) > perp_y_;

  AddBlock(true);

  and_clause_ = true;

  const bool perp_rotate_left =
      -perp_point_angle_diff > s1_left_angle_;

  const bool stage1_rotate_left =
      (perp_unaligned || perp_unaligned_lower || y_unaligned)
      && perp_rotate_left;

  return stage1_rotate_left;
}

bool Docking::ShouldStage2Left() {
  const Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).observed_pose;

  const float robot_heading = current_pose.angle;

  // Dock is facing the opposite direction from the desired
  // docking angle for the robot.
  const Eigen::Vector2f dock_direction = Heading(-dock_angle);

  const Vector2f y_dir(-sin(robot_heading), cos(robot_heading));

  Eigen::Vector2f perpendicular_point;
  Eigen::Vector2f dock_point2 = dock_location + dock_direction;
  ProjectPointOntoLine(current_pose.translation,
                       dock_location,
                       dock_point2,
                       &perpendicular_point);

  const Vector2f robot_to_perp_displace =
      perpendicular_point - current_pose.translation;

  perpendicular_point.y() = perpendicular_point.y()
      - s1_align_lower_;

  const Vector2f robot_to_final_displace =
      perpendicular_point - current_pose.translation;

  float perp_x_dist = robot_to_perp_displace.y();

  const Vector2f robot_to_perp_point =
      perp_point - current_pose.translation;

  float perp_y_dist = robot_to_perp_point.x();

  const float perp_point_angle = Angle(robot_to_perp_point);

  const float perp_point_angle_diff = RadToDeg(AngleDiff(robot_heading,
                                                         perp_point_angle));

  const float robot_to_perp_angle = Angle(robot_to_final_displace);

  float perp_rot_diff = AngleDiff(robot_heading,
                                  robot_to_perp_angle);
  perp_rot_diff = RadToDeg(perp_rot_diff);


  // SET POTENTIAL TRANSITION
  potential_state_ = "STAGE2_ROTATELEFT";
  // ADD A BLOCK OF CLAUSES
  AddBlock(true);
  // SET AND FOR CLAUSES
  and_clause_ = true;
  const bool perp_unaligned = perp_x_dist > s1_align_;

  // SET AND FOR CLAUSES (next clause(s) are ORs)
  and_clause_ = false;

  const bool perp_unaligned_lower = perp_x_dist < s1_align_lower_;


  const bool y_unaligned = fabs(perp_y_dist) > perp_y_;

  AddBlock(true);

  and_clause_ = true;

  const bool perp_rotate_left =
      -perp_point_angle_diff > s2_left_angle_;

  const bool stage2_rotate_left =
      (perp_unaligned || perp_unaligned_lower || y_unaligned)
      && perp_rotate_left;

  return stage2_rotate_left;
}

void Docking::Transition() {
  // Basic Logging
  logger::Logger* robot_logger =
      soccer_state_->GetMutableRobotLoggerByOurRobotIndex(our_robot_index_);
  robot_logger->LogPrint("Docking Machine");
  robot_logger->LogPrint("Execution State %s", state_.name_);
  robot_logger->AddArc(dock_location, 90, -1.5708, 1.5708, 1.0, 0.0, 0.0, 1.0);

  const Pose2Df current_pose =
      world_state_.GetOurRobotPosition(our_robot_index_).observed_pose;

  const float robot_heading = current_pose.angle;

  const Eigen::Vector2f robot_direction = Heading(robot_heading);

  // Dock is facing the opposite direction from the desired
  // docking angle for the robot.
  //   const Eigen::Vector2f dock_direction = Heading(-dock_angle);
  const float rotation_difference = RadToDeg(AngleDiff(robot_heading,
                                                       dock_angle));

  const Vector2f robot_to_dock_displace =
      dock_location - current_pose.translation;

  const Vector2f y_dir(-sin(robot_heading), cos(robot_heading));

  float y_dist = y_dir.dot(robot_to_dock_displace);

  float x_dist = robot_direction.dot(robot_to_dock_displace);

  const bool docked = fabs(y_dist) < kDockedYThreshold &&
                      fabs(x_dist) < kDockOffsetThreshold_ &&
                      fabs(rotation_difference) < kDockRotationThreshold;

  if (state_ == start_) {
    // IF NOT DOCKED
    if (!docked) {
      const bool stage1_forward = ShouldStage1Forward();
      // Drive towards perpendicular
      if (stage1_forward) {
        SetTransition(stage1_forward);
        state_ = s1_forward_;
      }

      const bool stage1_rotate_right = ShouldStage1Right();
      if (stage1_rotate_right) {
        SetTransition(stage1_rotate_right);
        state_ = s1_right_;
      }

      const bool stage1_rotate_left = ShouldStage1Left();
      if (stage1_rotate_left) {
        SetTransition(stage1_rotate_left);
        state_ = s1_left_;
      }

      // Drive Towards Dock
      const bool stage2_forward = ShouldStage2Forward();
      if (stage2_forward) {
        SetTransition(stage2_forward);
        state_ = s1_forward_;
      }

      const bool stage2_rotate_right = ShouldStage2Right();
      if (stage2_rotate_right) {
        SetTransition(stage2_rotate_right);
        state_ = s2_left_;
      }

      const bool stage2_rotate_left = ShouldStage2Left();
      if (stage2_rotate_left) {
        SetTransition(stage2_rotate_left);
        state_ = s2_left_;
      }

    } else {
      state_ = docked_;
    }
  }

  // STAGE1 transitions
  if (state_ == s1_left_ ||
      state_ == s1_right_ ||
      state_ == s1_forward_) {
    // IF NOT DOCKED
    if (!docked) {
      // Drive towards perpendicular
      const bool stage1_forward = ShouldStage1Forward();
      // Drive towards perpendicular
      if (stage1_forward) {
        SetTransition(stage1_forward);
        state_ = s1_forward_;
      }

      const bool stage1_rotate_right = ShouldStage1Right();
      if (stage1_rotate_right) {
        SetTransition(stage1_rotate_right);
        state_ = s1_right_;
      }

      const bool stage1_rotate_left = ShouldStage1Left();
      if (stage1_rotate_left) {
        SetTransition(stage1_rotate_left);
        state_ = s1_left_;
      }

      const bool stage2_rotate_right = ShouldStage2Right();
      if (stage2_rotate_right) {
        SetTransition(stage2_rotate_right);
        state_ = s2_right_;
      }

      const bool stage2_rotate_left = ShouldStage2Left();
      if (stage2_rotate_left) {
        SetTransition(stage2_rotate_left);
        state_ = s2_left_;
      }
    }
  }

  // STAGE2 transitions
  if (state_ == s2_left_ ||
      state_ == s2_right_ ||
      state_ == s2_forward_) {
    if (!docked) {
      // Drive Towards Dock
      const bool stage2_forward = ShouldStage2Forward();
      if (stage2_forward) {
        SetTransition(stage2_forward);
        state_ = s2_forward_;
      }

      const bool stage2_rotate_right = ShouldStage2Right();
      if (stage2_rotate_right) {
        SetTransition(stage2_rotate_right);
        state_ = s2_right_;
      }

      const bool stage2_rotate_left = ShouldStage2Left();
      if (stage2_rotate_left) {
        SetTransition(stage2_rotate_left);
        state_ = s2_left_;
      }
    } else {
      state_ = docked_;
    }
  }
}

void Docking::Reset() {}

void Docking::SetGoal(const Pose2Df& pose) {
  dock_location = pose.translation;
}

float Docking::GetCost() {
  return 0.0;
}

}  // namespace tactics
