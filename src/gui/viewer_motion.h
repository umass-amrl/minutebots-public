// Copyright 2011-2018 joydeepb@cs.umass.edu, dbalaban@cs.umass.edu
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

#include "QObject"

#include "constants/constants.h"
#include "gui/viewer.h"
#include "motion_control/ntoc_2d.h"
#include "motion_control/optimal_control_1d.h"
#include "motion_control/tsocs_old.h"
#include "net/netraw.h"
#include "shared/common_includes.h"

#ifndef SRC_GUI_VIEWER_MOTION_H_
#define SRC_GUI_VIEWER_MOTION_H_

class MotionVisualizer : public QObject {
  Q_OBJECT
 public:
  explicit MotionVisualizer(gui::Viewer* viewer) : viewer_(viewer) {}
 public slots:  // NOLINT(whitespace/indent)
  void GenerateNewProblem(QKeyEvent* event);

 private:
  MinuteBotsProto::SoccerDebugMessage message_;
  gui::Viewer* viewer_;
};

#endif  // SRC_GUI_VIEWER_MOTION_H_
