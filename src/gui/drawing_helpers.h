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

#include "gui/opengl_helpers.h"
#include "soccer_logging.pb.h"

#ifndef SRC_GUI_DRAWING_HELPERS_H_
#define SRC_GUI_DRAWING_HELPERS_H_

namespace drawing_helpers {

template <typename T>
void DrawPoint(const Eigen::Matrix<T, 2, 1>& p,
               const opengl_helpers::Color4f& color,
               MinuteBotsProto::DebugDrawings* drawings) {
  MinuteBotsProto::ColoredPoint& point = *(drawings->add_points());
  point.mutable_value()->set_x(p.x());
  point.mutable_value()->set_y(p.y());
  point.mutable_color()->set_r(color.r);
  point.mutable_color()->set_g(color.g);
  point.mutable_color()->set_b(color.b);
  point.mutable_color()->set_a(color.a);
}

template <typename T>
void DrawLine(const Eigen::Matrix<T, 2, 1>& p0,
              const Eigen::Matrix<T, 2, 1>& p1,
              const opengl_helpers::Color4f& color,
              MinuteBotsProto::DebugDrawings* drawings) {
  MinuteBotsProto::ColoredLine& line = *(drawings->add_lines());
  line.mutable_value()->mutable_v0()->set_x(p0.x());
  line.mutable_value()->mutable_v0()->set_y(p0.y());
  line.mutable_value()->mutable_v1()->set_x(p1.x());
  line.mutable_value()->mutable_v1()->set_y(p1.y());
  line.mutable_color()->set_r(color.r);
  line.mutable_color()->set_g(color.g);
  line.mutable_color()->set_b(color.b);
  line.mutable_color()->set_a(color.a);
}

}  // namespace drawing_helpers

#endif  // SRC_GUI_DRAWING_HELPERS_H_
