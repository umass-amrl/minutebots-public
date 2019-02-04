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
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
// ========================================================================

#include "evaluators/field_visualizer.h"
#include <iostream>
#include <fstream>

STANDARD_USINGS;
using Eigen::Vector2f;
using state::WorldState;
using field_dimensions::kFieldWidth;
using field_dimensions::kFieldLength;
using field_dimensions::kGoalDepth;
using field_dimensions::kGoalWidth;

namespace field_visualizer {

cimg_library::CImg<unsigned char> create_field() {
  cimg_library::CImg<unsigned char> field(
      kFieldLength, kFieldWidth, 1, kNumberOfColorChannels, kInitialValue);
  return field;
}
// A function to draw the lines of the field on the image
void draw_field_lines(cimg_library::CImg<unsigned char>* field) {
  field->draw_circle(kFieldLength / 2, kFieldWidth / 2, kGoalWidth / 2, white,
                     1, 1);
  field->draw_rectangle(0, kFieldWidth / 2 + kGoalWidth / 2, kGoalDepth,
                        kFieldWidth / 2 - kGoalWidth / 2, white, 1);
  field->draw_rectangle(kFieldLength, kFieldWidth / 2 + kGoalWidth / 2,
                        kFieldLength - kGoalDepth,
                        kFieldWidth / 2 - kGoalWidth / 2, white, 1);
  field->draw_line(kFieldLength / 2, 0, kFieldLength / 2, kFieldWidth, white);
  return;
}

Vector2f convert_coordinates(float field_x, float field_y) {
  Vector2f r = Vector2f::Zero();
  r[0] = field_x + kFieldLength / 2;
  r[1] = -field_y + kFieldWidth / 2;
  return r;
}

void draw_ball(cimg_library::CImg<unsigned char>* field, Vector2f ball_pose) {
  unsigned char color[] = {255, 165, 0};
  Vector2f t = convert_coordinates(ball_pose.x(), ball_pose.y());
  field->draw_circle(static_cast<int>(t[0]), static_cast<int>(t[1]),
                     kBallRadius, color);
}

// Draws our team as blue and the current robot as a lighter blue
void draw_teammates(cimg_library::CImg<unsigned char>* field,
                    const WorldState& world_state, int this_robot) {
  for (const auto& robot : world_state.GetOurRobots()) {
    Vector2f t = convert_coordinates(robot.position.translation.x(),
                                     robot.position.translation.y());

    field->draw_circle(static_cast<int>(t[0]), static_cast<int>(t[1]),
                       kRobotRadius, blue);

    if (world_state.GetOurRobotIndex(robot.ssl_vision_id) == this_robot) {
      unsigned char color[] = {0, 150, 255};
      field->draw_circle(static_cast<int>(t[0]), static_cast<int>(t[1]),
                         kRobotRadius - 15, color);
    }
  }
  return;
}

// Draws the opponents as yellow
void draw_opponents(cimg_library::CImg<unsigned char>* field,
                    const WorldState& world_state) {
  for (const auto& robot : world_state.GetTheirRobots()) {
    Vector2f t = convert_coordinates(robot.position.translation.x(),
                                     robot.position.translation.y());
    field->draw_circle(static_cast<int>(t[0]), static_cast<int>(t[1]),
                       kRobotRadius, yellow);
  }
  return;
}

// This function will color the pixels of the field accordingly with their
// scoring/passing potential
// The end result should be an image of the field that is a heatmap of the
// score_function defined above
void render_score(cimg_library::CImg<unsigned char>* field,
                  Vector2f ball_pose,
                  vector<unsigned int> robots_to_ignore,
                  Vector2f pos_to_eval,
                  const WorldState& world_state,
                  float (*score_func)(Vector2f ball_pose,
                                      vector<unsigned int> robots_to_ignore,
                                      Vector2f pos_to_eval,
                                      const WorldState& world_state)) {
  FunctionTimer function_timer("Field Scoring Time");
//   const float kScoreMax = 1;
//   const float kScoreMin = 0.5;
//   const float kColorMax = 255;
//   const float kColorMin = 0;
//
//   float scoreMax = 0;
//   float scoreMin = 1000000000;
//   std::ofstream file;
//   file.open("pass_scores.csv");
//   file << "X,Y,Score\n";
  Vector2f eval_me = Vector2f::Zero();
  const float search_offset_left = 2000;
  const float search_offset_right = 4000;
  for (float x = ball_pose.x() - search_offset_left;
       (x < kFieldXMax && x < ball_pose.x() + search_offset_right);
        x += kRobotRadius * 4) {
    for (float y = -(kFieldWidth / 2);
         y < kFieldWidth / 2; y += kRobotRadius * 4) {
//       unsigned char color[] = {0, 0, 0};
      eval_me[0] = x;
      eval_me[1] = y;
      if (fabs(x - ball_pose.x()) < 500 ||
          fabs(y - ball_pose.y()) < 500) {
//         file << x << "," << y << "," << 1 << "\n";
      }
      float your_score =
          score_func(ball_pose, robots_to_ignore, eval_me, world_state);
      your_score++;
//       file << x << "," << y << "," << your_score << "\n";
    }
  }
}

}  // namespace field_visualizer
