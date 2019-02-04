// Copyright 2011-2017 joydeepb@cs.umass.edu
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

#include "QtGui"
#include "QApplication"

#include "gui/soccerview.h"
#include "shared/common_includes.h"

using gui::GLSoccerView;
using SSLVisionProto::SSL_WrapperPacket;

GLSoccerView *view;

bool runApp = true;

class MyThread : public QThread {
 protected:
  void run() {
    static const double kMinDuration = 0.01;
    SSL_WrapperPacket packet;
    while (runApp) {
      Sleep(kMinDuration);
    }
  }

 public:
  explicit MyThread(QObject* parent = 0) {}
  ~MyThread() {}
};

int main(int argc, char **argv) {
  QApplication app(argc, argv);
  view = new GLSoccerView();
  view->show();
  MyThread thread;
  thread.start();
  int retVal = app.exec();
  runApp = false;
  thread.wait();
  return retVal;
}

