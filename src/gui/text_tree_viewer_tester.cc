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

#include <QApplication>
#include <QDesktopWidget>

#include "soccer_logging.pb.h"
#include "gui/text_tree_viewer.h"
#include "shared/common_includes.h"

using gui::TextTreeViewer;
using MinuteBotsProto::TextTree;

TextTree NewTextTree(const string& str) {
  TextTree tree;
  tree.set_text(str);
  return tree;
}

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);
  TextTreeViewer viewer;
  viewer.resize(viewer.sizeHint());
  viewer.setWindowTitle("OpenGL Text Tree Viewer");
  viewer.show();
  TextTree sub_sub_log, sub_log, log;
  sub_log.set_text("Sub-Log");
  for (auto i = 0; i < 3; ++i) {
    *sub_log.add_sub_tree() = NewTextTree(StringPrintf(
        "%d Hello World %.3f",
        i,
        fmod(GetMonotonicTime(), 100.0)));
  }
  sub_sub_log = sub_log;
  *sub_sub_log.mutable_text() = "Sub-Sub-Log";
  *sub_log.add_sub_tree() = sub_sub_log;
  log.set_text("Log");
  for (int i = 0; i < 5; ++i) {
    *log.add_sub_tree() = sub_log;
    log.mutable_sub_tree(i)->set_text(
        log.sub_tree(i).text() + StringPrintf(" %d", i));
  }
  MinuteBotsProto::SoccerDebugMessage debug_msg;
  *debug_msg.mutable_text_log() = log;
  viewer.Update(debug_msg);
  return app.exec();
}
