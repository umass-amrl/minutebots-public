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

#include <algorithm>

#include "GL/gl.h"
#include "GL/glu.h"
#include "eigen3/Eigen/Geometry"
#include "gui/opengl_helpers.h"
#include "shared/common_includes.h"

namespace opengl_helpers {

const Color4f Color4f::kRed(1.0, 0.0, 0.0, 1.0);
const Color4f Color4f::kGreen(0.0, 1.0, 0.0, 1.0);
const Color4f Color4f::kBlue(0.0, 0.0, 1.0, 1.0);
const Color4f Color4f::kWhite(1.0, 1.0, 1.0, 1.0);
const Color4f Color4f::kBlack(0.0, 0.0, 0.0, 1.0);
const Color4f Color4f::kYellow(1.0, 1.0, 0.0, 1.0);
const Color4f Color4f::kCyan(0.0, 1.0, 1.0, 1.0);
const Color4f Color4f::kMagenta(1.0, 0.0, 1.0, 1.0);

void SetColor(const Color4f& color) {
  glColor4f(color.r, color.g, color.b, color.a);
}

void DrawRectangle(const Vector2f& p0, const Vector2f& p1, const Vector2f& p2,
                   const Vector2f& p3, float z, const Color4f& color) {
  SetColor(color);
  glBegin(GL_QUADS);
  glVertex3f(p0.x(), p0.y(), z);
  glVertex3f(p1.x(), p1.y(), z);
  glVertex3f(p2.x(), p2.y(), z);
  glVertex3f(p3.x(), p3.y(), z);
  glEnd();
}

void DrawAlignedRectangle(const Vector2f& p0, float width, float height,
                          float z, const Color4f& color) {
  SetColor(color);
  glBegin(GL_QUADS);
  glVertex3f(p0.x(), p0.y(), z);
  glVertex3f(p0.x() + width, p0.y(), z);
  glVertex3f(p0.x() + width, p0.y() + height, z);
  glVertex3f(p0.x(), p0.y() + height, z);
  glEnd();
}

void DrawLine(const Vector2f& p0, const Vector2f& p1, float width, float z,
              const Color4f& color) {
  const Vector2f line_dir = (p1 - p0).normalized();
  const Vector2f line_perp = Perp(line_dir);
  const Vector2f v0 = p0 + width * (-line_dir - line_perp);
  const Vector2f v1 = p1 + width * (line_dir - line_perp);
  const Vector2f v2 = p1 + width * (line_dir + line_perp);
  const Vector2f v3 = p0 + width * (-line_dir + line_perp);
  DrawRectangle(v0, v1, v2, v3, z, color);
}

void DrawArc(const Vector2f& loc, float r1, float r2, float theta1,
             float theta2, float z, const Color4f& color) {
  if (r1 < 0 || r2 < 0) {
    printf("WARNING: Asked to draw arc with -ve radii: %f to %f\n", r1, r2);
    return;
  }
  SetColor(color);
  const float d_theta = min<float>(20.0 / r2, DegToRad(10.0));
  glBegin(GL_QUAD_STRIP);
  for (float theta = theta1; theta < theta2; theta += d_theta) {
    float c1 = cos(theta), s1 = sin(theta);
    glVertex3f(r2 * c1 + loc.x(), r2 * s1 + loc.y(), z);
    glVertex3f(r1 * c1 + loc.x(), r1 * s1 + loc.y(), z);
  }
  float c1 = cos(theta2), s1 = sin(theta2);
  glVertex3f(r2 * c1 + loc.x(), r2 * s1 + loc.y(), z);
  glVertex3f(r1 * c1 + loc.x(), r1 * s1 + loc.y(), z);
  glEnd();
}

void DrawEllipse(const Vector2f& loc, float r1, float r2, float angle, float z,
                 const Color4f& color) {
  SetColor(color);
  glBegin(GL_QUAD_STRIP);

  const Eigen::Rotation2Df rot(angle);

  constexpr float delta = 20.0f;
  for (float deg = 0; deg <= 360; deg += delta) {
    const float ang1 = DegToRad(deg);
    const float ang2 = DegToRad(deg + delta);

    Vector2f xy1(r1 * cos(ang1), r2 * sin(ang1));
    Vector2f xy2((r1 + 0.5f) * cos(ang2), (r2 + 0.5f) * sin(ang2));
    xy1 = rot * xy1 + loc;
    xy2 = rot * xy2 + loc;
    glVertex3f(xy1.x(), xy1.y(), z);
    glVertex3f(xy2.x(), xy2.y(), z);
  }
  //     float c1 = cos(0), s1 = sin(360);
  //     glVertex3f(r2 * c1 + loc.x(), r2 * s1 + loc.y(), z);
  //     glVertex3f(r1 * c1 + loc.x(), r1 * s1 + loc.y(), z);
  glEnd();
}

}  // namespace opengl_helpers
