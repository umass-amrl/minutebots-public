// Copyright 2017 joydeepb@cs.umass.edu
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

#include "eigen3/Eigen/Dense"

#ifndef SRC_GUI_OPENGL_HELPERS_H_
#define SRC_GUI_OPENGL_HELPERS_H_

namespace opengl_helpers {

// RGBA color with each channel scaling from 0.0 to 1.0.
struct Color4f {
  float r;
  float g;
  float b;
  float a;
  Color4f() {}
  Color4f(float r, float g, float b, float a) : r(r), g(g), b(b), a(a) {}

  static const Color4f kRed;
  static const Color4f kGreen;
  static const Color4f kBlue;
  static const Color4f kWhite;
  static const Color4f kBlack;
  static const Color4f kYellow;
  static const Color4f kCyan;
  static const Color4f kMagenta;
};

// Set the specified color as the OpenGL drawing color.
void SetColor(const Color4f& color);

// Draw an arbitrary rectangle.
void DrawRectangle(const Eigen::Vector2f& p0,
                   const Eigen::Vector2f& p1,
                   const Eigen::Vector2f& p2,
                   const Eigen::Vector2f& p3,
                   float z,
                   const Color4f& color);

// Draw an axis-aligned rectangle with the specified width and height, and with
// the specified origin p0 (bottom left corner of rectangle).
void DrawAlignedRectangle(const Eigen::Vector2f& p0,
                          float width,
                          float height,
                          float z,
                          const Color4f& color);

// Draw a line from p0 to p1, with specified width and color.
void DrawLine(const Eigen::Vector2f& p0,
              const Eigen::Vector2f& p1,
              float width,
              float z,
              const Color4f& color);

// Draw a circular arc centered at loc, from radius r1 to r2, from angle
// theta1 to theta2, on the specified z plane.
void DrawArc(const Eigen::Vector2f& loc,
             float r1,
             float r2,
             float theta1,
             float theta2,
             float z,
             const Color4f& color);

void DrawEllipse(const Eigen::Vector2f& loc,
                 float r1,
                 float r2,
                 float angle,
                 float z,
                 const Color4f& color);

}  // namespace opengl_helpers

#endif  // SRC_GUI_OPENGL_HELPERS_H_
