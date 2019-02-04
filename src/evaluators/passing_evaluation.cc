// Copyright 2018 ikhatri@umass.edu
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
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
// ========================================================================

#include "evaluators/passing_evaluation.h"

STANDARD_USINGS;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using Eigen::Vector2d;
using state::WorldState;
using geometry::EuclideanDistance;
using offense::AimOption;
using offense::CalculateAimOptions;

using field_dimensions::kFieldWidth;
using field_dimensions::kFieldLength;
using field_dimensions::kGoalDepth;
using field_dimensions::kGoalWidth;
using geometry::Angle;

namespace passing_evaluation {
// A function that normalizes all squared distances on the field between 0 and 1
// The maximum squared distance possible in the field is the field diagonal
// squared
// The minimum squared distance possible is the robot radius squared
float NormalizeDistance(float old_dist) {
  float field_diag = geometry::EuclideanDistance(
      Vector2f(0, 0), Vector2f(kFieldWidth, kFieldLength));
  return old_dist / field_diag;
}

float NormalizeOpenAngle(float old_angle) { return old_angle / M_PI; }

// Returns true if i is contained within the list, false otherwise
bool Contains(vector<unsigned int> list, unsigned int i) {
  for (auto& t : list) {
    if (t == i) return true;
  }
  return false;
}

// Pass Ahead Objective Function
// ARGS: the ball pose, the pass ahead pose to be evaluated and a world state
// RETURNS: a score for the given pass ahead pose (between 0.5 and 1)

// Function format: 1/(1-x) where x = [(1-x_1)^(n_1) * (1-x_2)^(n_2)]^1/(n_1 +
// n_2)
// Where x_1 represents the first factor (normalized between 0 and 1)
// and n_1 represents its weight (larger n's decrease the impact of x)

// For now uses the following factors:
// The largest open angles to the goal from the target position
// The squared distance between the path to the target position and
// the position of the closest opponent (change this to the open angle to the
// recieving position)
// The smallest squared distance between the target position and the position
// of any teammate
float PassAheadObjective(Vector2f ball_pose,
                         vector<unsigned int> robots_to_ignore,
                         Vector2f pass_ahead_pose,
                         const WorldState& world_state) {
  float objective = 0;
  const float kScoringWeight = 10;  // The weight for the component of the score
                                    // that calculates how good the target
                                    // position is for scoring a goal
  const float kInterceptWeight = 1;  // The weight for the component of the
                                     // score that calculates how likely the
                                     // ball is to be intercepted
  const float kRecieveWeight = 10;  // The weight for the component of the score
                                    // that calculates how likely the ball is to
                                    // be recieved by a teammate

  // Calculate the number of aim options to to the goal from the proposed pass
  // ahead pose
  // Add all robots except this one (main attacker) to the obstacles list
  vector<Vector2f> world_robot_positions;
  for (const auto& robot : world_state.GetOurRobots()) {
    if (!Contains(robots_to_ignore, robot.ssl_vision_id)) {
      world_robot_positions.push_back(robot.position.translation);
    }
  }
  for (const auto& robot : world_state.GetTheirRobots()) {
    world_robot_positions.push_back(robot.position.translation);
  }

  // Create an aim options vector & fill it with options
  vector<AimOption> aim_options;
  CalculateAimOptions(pass_ahead_pose, kTheirGoalL, kTheirGoalR,
                      2 * kBallRadius, kRobotRadius, world_robot_positions,
                      &aim_options);

  float max_width = -1;
  if (!aim_options.empty()) {
    int current_index = 0;
    for (const AimOption& option : aim_options) {
      if (option.angle_width > max_width) {
        max_width = option.angle_width;
      }
      current_index++;
    }
  }
  if (max_width == -1) max_width = 0;

  objective = pow(1.0 - NormalizeOpenAngle(max_width), kScoringWeight);
  // std::cout << "Max width: " << objective << std::endl;

  // Find the smallest distance between the path and the closest
  // opponent
  // The smallest interception distance is initialized to the length of the
  // diagonal of the field (squared)
  float smallest_interception_distance =
      kFieldWidth * kFieldWidth + kFieldLength * kFieldLength;

  for (const auto& robot : world_state.GetTheirRobots()) {
    Eigen::Matrix<float, 2, 1> projected_point =
        Eigen::Matrix<float, 2, 1>::Identity();

    float squared_distance = 0;
    geometry::ProjectPointOntoLineSegment(robot.position.translation, ball_pose,
                                          pass_ahead_pose, &projected_point,
                                          &squared_distance);
    smallest_interception_distance =
        (smallest_interception_distance <= squared_distance)
            ? smallest_interception_distance
            : squared_distance;
  }
  smallest_interception_distance = pow(smallest_interception_distance, 0.5);
  objective *= pow(1.0 - NormalizeDistance(smallest_interception_distance),
                   kInterceptWeight);
  // std::cout << "Smallest Intercept : "
  //           << NormalizeDistance(smallest_interception_distance)
  //           << std::endl;
  // std::cout << "Score: " << objective << std::endl;

  // Find the smallest distance between the target pass position and the
  // closest teammate
  // The smallest recieve distance is initialized to the length of the diagonal
  // of the field
  float smallest_recieve_distance =
      pow(kFieldWidth * kFieldWidth + kFieldLength * kFieldLength, 0.5);

  for (const auto& robot : world_state.GetOurRobots()) {
    if (!Contains(robots_to_ignore, robot.ssl_vision_id)) {
      float euclidean_distance = geometry::EuclideanDistance(
          pass_ahead_pose, robot.position.translation);

      smallest_recieve_distance =
          (smallest_recieve_distance <= euclidean_distance)
              ? smallest_recieve_distance
              : euclidean_distance;
    }
  }

  objective *=
      pow(NormalizeDistance(smallest_recieve_distance), kRecieveWeight);

  objective = pow(objective,
                  1.0 / (kScoringWeight + kInterceptWeight + kRecieveWeight));

  // std::cout << "Final score: " << 1 / (1 + objective) << std::endl;
  return 1 / (1 + objective);
}

// Sadegh's Cost function from support attacker
float GetPoseCost(Vector2f ball,
                  vector<unsigned int> robots_to_ignore,
                  Vector2f position,
                  const WorldState& world_state) {
  // Used to determine whether the support attacker should look at the goal
  // or the ball
  const float min_goal_open_angle = (1.0 / 180.0) * M_PI;

  // The maximum passing clearance angles used for saturating the angle
  const float passing_clearance_max = (20.0 / 180.0) * M_PI;

  // Used to prevent rapid switching when there exists no open angle
  // towards the goal.
  const float goal_open_angle_bias = (1.0 / 180.0) * M_PI;

  // Used to prevent the support attackers from camping close
  // to the primary offensive role
  //   const float kAttackerPadding = 15.0 * kRobotRadius;


  const float deflection_angle_max_threshold = (80.0 / 180.0) * M_PI;
  const float deflection_angle_min_threshold = (10.0 / 180.0) * M_PI;
  const float deflection_penalty_rate = 0.001 / ((1.0 / 180.0) * M_PI);

  const unsigned int attacker_index = robots_to_ignore[0];
  float cost = 1.0;
  float goal_angle_width = 0.0;
  float goal_angle = 0.0;
  const Vector2f ball_pos = world_state.GetBallPosition().position;
  const Pose2Df attacker_pose =
  world_state.GetOurRobotPosition(attacker_index).position;
//   const Vector2f displacement = attacker_pose.translation - position;
//   const Vector2f ball_displacement = ball_pos - position;

  size_t our_robots_num = world_state.GetOurRobots().GetElementCount();
//   // Keep distance from the ball and the attacker
//   if ((displacement.norm() < kAttackerPadding &&
//     attacker_index != our_robot_index) ||
//     ball_displacement.norm() < kAttackerPadding) {
//     return 1.0;
//   }

  // Points cannot be in the defense areas.
  obstacle::ObstacleFlag flags = obstacle::ObstacleFlag::GetDefenseAreas();
  for (auto obstacle : flags) {
    if (obstacle->PointCollision(position, 2 * kRobotRadius)) {
      return 1.0;
    }
  }

  // Calculate the open angle from the given pose to the goal

  float support_ball_angle = Angle<float>(ball_pos - position);

  vector<Vector2f> obstacle_robot_positions;
  for (size_t k = 0; k < our_robots_num; k++) {
    if (k == attacker_index || std::find(robots_to_ignore.begin(),
                                         robots_to_ignore.end(),
                                          k) != robots_to_ignore.end()) {
      continue;
    } else {
      obstacle_robot_positions.push_back(
        world_state.GetOurRobotPosition(k).position.translation);
    }
  }
  for (const auto& robot : world_state.GetTheirRobots()) {
    obstacle_robot_positions.push_back(robot.position.translation);
  }


  // Get all the aiming options toward the goal
  vector<AimOption> aim_options;
  CalculateAimOptions(position, kTheirGoalL, kTheirGoalR, kBallRadius,
                      kRobotRadius, obstacle_robot_positions, &aim_options);

  // Take the aim option with largest open angle
  if (!aim_options.empty()) {
    int max_width_index = -1;
    float max_width = -1;
    int current_index = 0;
    for (const AimOption& option : aim_options) {
      if (option.angle_width > max_width) {
        max_width = option.angle_width;
        max_width_index = current_index;
      }
      current_index++;
    }
    goal_angle_width = max_width;
    goal_angle = aim_options[max_width_index].angle_center;
  } else {
    goal_angle_width = 0.0;
    goal_angle = 0.0;
  }

  // TODO(srabiee): Modify the distance of the ball and the robot so that the
  // ball touches the robot exactly on the kicker

  pose_2d::Pose2Df proposed_pose;
  // Returning the proposed pose of the robot given it is going to
  // receive the ball at the given position.
  if (goal_angle_width > min_goal_open_angle) {
    proposed_pose.angle = goal_angle;
  } else {
    proposed_pose.angle = support_ball_angle;
  }
  //   proposed_pose->translation.x() =
  //       position(0) - (kRobotRadius + kBallRadius) * cos(goal_angle);
  //   proposed_pose->translation.y() =
  //       position(1) - (kRobotRadius + kBallRadius) * sin(goal_angle);
  proposed_pose.translation.x() = position(0);
  proposed_pose.translation.y() = position(1);

  // ****
  // Calculate the angle clearance for receiving a ball at the given position
  float clearance_angle = 0.0;
  SSLVisionId attacker_id =
      world_state.GetOurRobotPosition(attacker_index).ssl_vision_id;
  float pass_open_angle_center;
  clearance_angle =
  offense::PassEvaluatorUnlimited(world_state,
                                  attacker_id,
                                  attacker_id,
                                  position,
                                  &pass_open_angle_center);

  // Saturate the passing clearance angle
  if (fabs(clearance_angle) > fabs(passing_clearance_max)) {
    clearance_angle = passing_clearance_max;
  }

  // Calculate the deflection angle (angle between the line connecting
  // the ball and the support attacker and the heading of the support
  // attacker towards the goal). Deflection angles more and less than a
  // threshold will be penalized to prevent the support attacker from
  // positioning in front of or behind the attacker

//     float deflection_angle = fabs(AngleDiff(goal_angle, support_ball_angle));

  // TODO(srabiee): Penalize the distance of the proposed position from the
  // robot's current position

  // Consider the deflection angle from the current ball position.
  const Vector2f robot_to_ball = ball_pos - position;
  const Vector2f robot_to_goal = Heading(goal_angle);
  float deflection_angle = acos(robot_to_goal.dot(robot_to_ball)
  / (robot_to_goal.norm() * robot_to_ball.norm()));
  deflection_angle = AngleMod(deflection_angle);
  deflection_angle = fabs(deflection_angle);
//   float deflection_adjustment = 1;
//   if (goal_angle_width > min_goal_open_angle) {
//     deflection_angle = fabs(deflection_angle - DegToRad(45));
//     deflection_adjustment = 1 - (deflection_angle / M_PI);
//   }
  // Cost calculation (Option#1): Multiplication of the pass and goal angles
  goal_angle_width = goal_angle_width + goal_open_angle_bias;
//   float goal_angle_width_normalized = goal_angle_width / M_PI;
  float clearance_angle_normalized = clearance_angle / M_PI;

  float ang_multiplication =
      goal_angle_width *
      clearance_angle_normalized;
  cost = 1.0 / (1.0 + ang_multiplication);

  // Cost calculation (Option#2): linear combination of the pass and goal angles
  //   float cost_goal_view = 1.0 / (1.0 + goal_angle_width);
  //   float cost_attacker_view = 1.0/ (1.0 + clearance_angle);
  //   cost = goal_view_importance * cost_goal_view +
  //          (1 - goal_view_importance) * cost_attacker_view;

  // TODO(jaholtz) determine if this is desirable?
  // Penalize very large or very small deflection angles:
    if (deflection_angle > deflection_angle_max_threshold) {
      float delta_high = deflection_angle - deflection_angle_max_threshold;
      cost = cost + delta_high * deflection_penalty_rate;
    } else if (deflection_angle < deflection_angle_min_threshold) {
      float delta_low = deflection_angle_min_threshold - deflection_angle;
      cost = cost + delta_low * deflection_penalty_rate;
    }

  if (cost > 1.0) {
    cost = 1.0;
  }

  return cost;
}

void save_pass_ahead_heatmap(Vector2f ball_pose,
                             vector<unsigned int> robots_to_ignore,
                             Vector2f pass_ahead_pose,
                             const state::WorldState& world_state) {
  // Create image
  cimg_library::CImg<unsigned char> image;

  // Color the image according to the score
  field_visualizer::render_score(&image,
                                 ball_pose,
                                 robots_to_ignore,
                                 pass_ahead_pose,
                                 world_state,
                                 GetPoseCost);

  // Draw the field lines
  field_visualizer::draw_field_lines(&image);

  // Draw our robots
  field_visualizer::draw_teammates(&image, world_state, 0);

  // Draw their robots
  field_visualizer::draw_opponents(&image, world_state);

  // Draw the ball
  field_visualizer::draw_ball(&image, ball_pose);

  image.save_png("test.bmp");
}

}  // namespace passing_evaluation
