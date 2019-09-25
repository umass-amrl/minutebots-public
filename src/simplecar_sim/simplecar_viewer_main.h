// Copyright 2018 - 2019 joydeepb@cs.umass.edu
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

#include <string>
#include "QObject"
#include "gui/soccerview.h"
#include "net/netraw.h"
#include "shared/common_includes.h"

#ifndef SRC_SIMPLECAR_SIM_SIMPLECAR_VIEWER_MAIN_H_
#define SRC_SIMPLECAR_SIM_SIMPLECAR_VIEWER_MAIN_H_

namespace gui {

class SoccerViewCallback : public QObject {
  Q_OBJECT

 public:
  void Initialize(const std::string& address, int port);

 public slots:  // NOLINT(whitespace/indent)
  void MouseDown(Eigen::Vector2f loc,
                 Qt::KeyboardModifiers modifiers,
                 Qt::MouseButtons button,
                 gui::SelectionType selection,
                 uint32_t selected_robot_id,
                 team::Team selected_robot_team);
  void MouseUp(Eigen::Vector2f loc,
               Qt::KeyboardModifiers modifiers,
               Qt::MouseButtons button,
               gui::SelectionType selection,
               uint32_t selected_robot_id,
               team::Team selected_robot_team);
  void MouseMove(Eigen::Vector2f loc,
                 Qt::KeyboardModifiers modifiers,
                 Qt::MouseButtons button,
                 gui::SelectionType selection,
                 uint32_t selected_robot_id,
                 team::Team selected_robot_team);

 private:
  net::UDPMulticastServer udp_server_;
};

}  // namespace gui

#endif  // SRC_SIMPLECAR_SIM_SIMPLECAR_VIEWER_MAIN_H_
