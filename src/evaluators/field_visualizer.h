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

#ifndef SRC_EVALUATORS_FIELD_VISUALIZER_H_
#define SRC_EVALUATORS_FIELD_VISUALIZER_H_
#include <iostream>
#include <vector>
#include "constants/constants.h"
#include "state/world_state.h"
// // resorting to ImageMagick
// #define cimg_use_jpeg  // Do this if you want CImg to process JPEGs itself
// // without resorting to ImageMagick
// #define cimg_use_tiff  // Do this if you want CImg to process TIFFs itself
// // without resorting to ImageMagick
#include "CImg.h"

// X.h (included from CImg) defines a bunch of things that conflict with a bunch
// of other things
#ifdef CursorShape
#undef CursorShape
#endif
#ifdef Status
#undef Status
#endif
#ifdef Bool
#undef Bool
#endif
#ifdef None
#undef None
#endif
#ifdef Success
#undef Success
#endif
#ifdef KeyPress
#undef KeyPress
#endif
#ifdef KeyRelease
#undef KeyRelease
#endif
#ifdef FocusIn
#undef FocusIn
#endif
#ifdef FocusOut
#undef FocusOut
#endif
#ifdef FontChange
#undef FontChange
#endif
#ifdef Unsorted
#undef Unsorted
#endif
#ifdef GrayScale
#undef GrayScale
#endif

#include "eigen3/Eigen/Dense"

namespace field_visualizer {


// const int kFieldLength = 900;
// const int kFieldWidth = 600;
// const int kGoalDepth = 20;
// const int kGoalWidth = 100;
// const float kRobotRadius = 3;
const int kNumberOfColorChannels = 3;
const unsigned char kInitialValue = 0;
const unsigned char red[] = {255, 0, 0}, green[] = {0, 255, 0},
                    yellow[] = {255, 255, 0}, blue[] = {0, 0, 255},
                    white[] = {255, 255, 255};

cimg_library::CImg<unsigned char> create_field();

// A function to draw the lines of the field on the image
void draw_field_lines(cimg_library::CImg<unsigned char> *field);

Eigen::Vector2f convert_coordinates(float field_x, float field_y);

void draw_ball(cimg_library::CImg<unsigned char> *field,
               Eigen::Vector2f ball_pose);

// A function to draw our team's robots
void draw_teammates(cimg_library::CImg<unsigned char> *field,
                    const state::WorldState &world_state, int this_robot);

// A function to draw the defending team's robots
void draw_opponents(cimg_library::CImg<unsigned char> *field,
                    const state::WorldState &world_state);

// This function will color the pixels of the field accordingly with their
// scoring/passing potential
// The end result should be an image of the field that is a heatmap of the
// score_function defined above
void render_score(
    cimg_library::CImg<unsigned char> *field, Eigen::Vector2f ball_pose,
    std::vector<unsigned int> robots_to_ignore, Eigen::Vector2f pos_to_eval,
    const state::WorldState &world_state,
    float (*score_func)(Eigen::Vector2f ball_pose,
                        std::vector<unsigned int> robots_to_ignore,
                        Eigen::Vector2f pos_to_eval,
                        const state::WorldState &world_state));

}  // namespace field_visualizer

#endif  // SRC_EVALUATORS_FIELD_VISUALIZER_H_
