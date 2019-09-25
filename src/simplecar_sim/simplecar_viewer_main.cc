// Copyright 2011-2019 joydeepb@cs.umass.edu, dbalaban@cs.umass.edu
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
#include <sys/time.h>
#include <sys/stat.h>
#include <ctime>
#include "QApplication"
#include "QtGui"
#include "logging/logger.h"
#include "net/netraw.h"
#include "shared/common_includes.h"
#include "simplecar_sim/simplecar_viewer_main.h"
#include "simplecar_sim/simplecar_viewer.h"
#include "referee.pb.h"
#include "viewer_callback.pb.h"
#include "X11/Xlib.h"


using gui::SoccerViewCallback;
using gui::Viewer;
using MinuteBotsProto::SoccerDebugMessage;
using MinuteBotsProto::TextTree;
using MinuteBotsProto::LogWrapper;
using MinuteBotsProto::SSL_Referee;
using net::UDPMulticastServer;

Viewer* view = nullptr;
bool test_mode_ = false;

bool test_logger_mode_ = false;

bool test_logger_mode_file_ = false;

bool test_logger_mode_net_ = false;

bool kick_tuning_mode = false;

bool team_yellow = true;

unsigned int rand_seed_ = 1;

float time0;
net::UDPMulticastServer server_;

bool is_game_halted_ = false;
int referee_port_ = DATA_STREAM_REF_PORT;

Eigen::Vector2d x0;
Eigen::Vector2d v0;
Eigen::Vector2d xf;
Eigen::Vector2d vf;
int received = 0;
int dropped = 0;
int last_frame = -1;

logger::WriteLogger write_logger("viewer_logs/test.txt");
logger::ReadLogger read_logger("viewer_logs/test.txt");

namespace gui {
void SoccerViewCallback::Initialize(const string& address, int port) {
  if (!udp_server_.Open(address, port, false)) {
    LOG(ERROR) << "Error opening UDP port for viewer feedback, exiting.";
  } else {
  }
  CHECK(udp_server_.IsOpen());
}

void SetEventMessage(const Vector2f& loc,
                     const Qt::KeyboardModifiers& modifiers,
                     const Qt::MouseButtons& button,
                     const gui::SelectionType& selection,
                     const uint32_t& selected_robot_id,
                     const team::Team& selected_robot_team,
                     MinuteBotsProto::MouseEvent* msg_ptr) {
  MinuteBotsProto::MouseEvent& msg = * msg_ptr;
  msg.set_x(loc.x());
  msg.set_y(loc.y());
  msg.set_alt(modifiers.testFlag(Qt::AltModifier));
  msg.set_ctrl(modifiers.testFlag(Qt::ControlModifier));
  msg.set_shift(modifiers.testFlag(Qt::ShiftModifier));
  if (button.testFlag(Qt::LeftButton)) {
    msg.set_button(0);
  } else if (button.testFlag(Qt::RightButton)) {
    msg.set_button(1);
  } else if (button.testFlag(Qt::MidButton)) {
    msg.set_button(2);
  } else {
    msg.set_button(-1);
  }

  msg.set_selection(MinuteBotsProto::MouseEvent_SelectionType_NONE_SELECTED);
  if (selection == gui::SelectionType::kBallSelected) {
    msg.set_selection(MinuteBotsProto::MouseEvent_SelectionType_BALL_SELECTED);
  } else if (selection == gui::SelectionType::kRobotSelected) {
    msg.set_selection(MinuteBotsProto::MouseEvent_SelectionType_ROBOT_SELECTED);
    msg.mutable_selected_robot()->set_id(selected_robot_id);
    if (selected_robot_team == team::Team::BLUE) {
      msg.mutable_selected_robot()->set_team(
          MinuteBotsProto::SelectedRobot_Team_TEAM_BLUE);
    } else if (selected_robot_team == team::Team::YELLOW) {
      msg.mutable_selected_robot()->set_team(
        MinuteBotsProto::SelectedRobot_Team_TEAM_YELLOW);
    } else {
      // Should never happen: a robot is selected, with an invalid team!
      CHECK(false);
    }
  }
}

void SoccerViewCallback::MouseDown(
    Eigen::Vector2f loc,
    Qt::KeyboardModifiers modifiers,
    Qt::MouseButtons button,
    gui::SelectionType selection,
    uint32_t selected_robot_id,
    team::Team selected_robot_team) {
  CHECK(udp_server_.IsOpen());
  MinuteBotsProto::MouseCallback msg;
  SetEventMessage(loc,
                  modifiers,
                  button,
                  selection,
                  selected_robot_id,
                  selected_robot_team,
                  msg.mutable_mouse_down());
  udp_server_.SendProtobuf(msg);
}

void SoccerViewCallback::MouseUp(
    Eigen::Vector2f loc,
    Qt::KeyboardModifiers modifiers,
    Qt::MouseButtons button,
    gui::SelectionType selection,
    uint32_t selected_robot_id,
    team::Team selected_robot_team) {
  CHECK(udp_server_.IsOpen());
  MinuteBotsProto::MouseCallback msg;
  SetEventMessage(loc,
                  modifiers,
                  button,
                  selection,
                  selected_robot_id,
                  selected_robot_team,
                  msg.mutable_mouse_up());
  udp_server_.SendProtobuf(msg);
}

void SoccerViewCallback::MouseMove(
    Eigen::Vector2f loc,
    Qt::KeyboardModifiers modifiers,
    Qt::MouseButtons button,
    gui::SelectionType selection,
    uint32_t selected_robot_id,
    team::Team selected_robot_team) {
  CHECK(udp_server_.IsOpen());
  MinuteBotsProto::MouseCallback msg;
  SetEventMessage(loc,
                  modifiers,
                  button,
                  selection,
                  selected_robot_id,
                  selected_robot_team,
                  msg.mutable_mouse_move());
  udp_server_.SendProtobuf(msg);
}

}  // namespace gui

pthread_mutex_t viewer_mutex_;

class WriteThread : public QThread {
 protected:
  void run() {
    static const double kMinDuration = 1.0 / 125.0;
    LogWrapper message;
    time0 = GetMonotonicTime();
    while (true) {
      SoccerDebugMessage soccer_debug;
      if (server_.TryReceiveProtobuf(&soccer_debug)) {
        ScopedLock lock(&viewer_mutex_);
        received++;
        // Count dropped messages
        if (last_frame != -1 && last_frame != soccer_debug.frame_number() - 1) {
          dropped++;
        }
        last_frame = soccer_debug.frame_number();
        bool kDebug = false;
        if (true) {
          write_logger.SetMessage(soccer_debug);
          if (kDebug) {
            write_logger.LogRobotDataEntry(kGraphRobot);
            write_logger.LogMessageStats(received, dropped);
          }
          int file_size = write_logger.GetFileSize();
          write_logger.WriteData();
          view->AddIndexEntry(file_size);
          if (received == 2) {
            view->UpdateSliderSize();
          }
          if (view->IsLive()) {
//             std::cout << "Updating viewer" << std::endl;
            view->Update(soccer_debug);
            read_logger.Live();
          }
        }
      }
      Sleep(kMinDuration);
    }
  }

 public:
  explicit WriteThread(QObject* parent = 0) {}
  ~WriteThread() {}
};

class ReadThread : public QThread {
 protected:
  void run() {
    LogWrapper message;
    time0 = GetMonotonicTime();
    if (kick_tuning_mode) {
      read_logger.BuildTuningIndex();
      std::cout << "Built tuning index" << std::endl;
    }
    while (true) {
      static const double kMinDuration = 1.0 / 125.0;
      {
        ScopedTryLock lock(&viewer_mutex_);
        if (lock.Locked()) {
          if (!view->IsLive()) {
            view->Playback();
          }
        }
      }
      Sleep(kMinDuration);
    }
  }

 public:
  explicit ReadThread(QObject* parent = 0) {}
  ~ReadThread() { if (kick_tuning_mode) read_logger.WriteKickTuningData(); }
};

class RefereeThread : public QThread {
 protected:
  void run() {
    static const double kMinDuration = 1.0 / 125.0;
    UDPMulticastServer referee_server;
    if (!referee_server.Open(DATA_STREAM_REF_IP, referee_port_, true)) {
      LOG(FATAL) << "Error opening UDP for referee "
                 << "HandleExecution thread exiting.";
    }
    CHECK(referee_server.IsOpen());
    SSL_Referee referee_message;
    while (true) {
      if (referee_server.TryReceiveProtobuf(&referee_message)) {
        if (referee_message.command() ==
            MinuteBotsProto::SSL_Referee_Command_HALT) {
          printf("Halt called!\n");
          is_game_halted_ = true;
        } else {
          is_game_halted_ = false;
        }
      }
      Sleep(kMinDuration);
    }
  }

 public:
  explicit RefereeThread(QObject* parent = 0) {}
  ~RefereeThread() {}
};

TextTree NewTextTree(const string& str) {
  TextTree tree;
  tree.set_text(str);
  return tree;
}

int main(int argc, char** argv) {
  XInitThreads();
  string read_log_name = "temp.txt";
  bool team_set = false;
  for (int i = 0; i < argc; ++i) {
    if (string(argv[i]) == "-t") {
      test_mode_ = true;
    } else if (string(argv[i]) == "-l") {
      test_logger_mode_ = true;
    } else if (string(argv[i]) == "-f") {
      test_logger_mode_file_ = true;
      read_log_name = argv[i + 1];
    } else if (string(argv[i]) == "-n") {
      test_logger_mode_net_ = true;
    } else if (string(argv[i]) == "-r" && i + 1 < argc) {
      ++i;
      referee_port_ = atoi(argv[i]);
    } else if (string(argv[i]) == "-k") {
      kick_tuning_mode = true;
      read_log_name = argv[i + 1];
    } else if (string(argv[i]) == "-ty") {
      team_yellow = true;
      team_set = true;
    } else if (string(argv[i]) == "-tb") {
      team_yellow = false;
      team_set = true;
    }
  }
//   if (!team_set && !test_logger_mode_file_) {
//     std::cout << "Team flag required: -ty for yellow team, " <<
//     "-tb for blue team. " << std::endl;
//     return -1;
//   }
  if (test_logger_mode_file_) {
    std::cout << "Read Log: " << read_log_name << std::endl;
  }
  server_.Open(DATA_STREAM_DEBUG_IP, 10108, true);
  server_.SetSplitLarge(kNetworkSplitLarge);
  CHECK(server_.IsOpen());
  time_t t = time(0);  // get current time
  struct timeval tval;
  gettimeofday(&tval, NULL);
  struct tm* now = new tm();
  localtime_r(&t, now);
  std::stringstream ss;
  ss << "car_viewer_logs/";
  ss << "carlog_";
  ss << std::to_string(now->tm_mon).c_str() << "-";
  ss << std::to_string(now->tm_mday).c_str() << "-"
     << std::to_string(1900 + now->tm_year).c_str() << "_";
  ss << std::to_string((tval.tv_sec * 1000000) + tval.tv_usec).c_str()
     << ".txt";
  string file_name = ss.str();
  ss << ".comment";
  string comment_file = ss.str();
  struct stat sb;

  if (stat("car_viewer_logs/", &sb) == 0) {
    QApplication app(argc, argv);
    if (!test_logger_mode_file_ && !kick_tuning_mode) {
      write_logger = logger::WriteLogger(file_name);
      read_logger = logger::ReadLogger(file_name);
      view = new Viewer(nullptr, &read_logger, kick_tuning_mode);
      view->SetCommentFile(comment_file);
    } else {
      read_logger = logger::ReadLogger(read_log_name);
      view = new Viewer(nullptr, &read_logger, kick_tuning_mode);
      const string read_comment_file = read_log_name + ".comment";
      comment_file = read_comment_file;
      view->SetCommentFile(read_comment_file);
      view->SetLive(false);
    }
    SoccerViewCallback callback;
    callback.Initialize(DATA_STREAM_VIEWER_IP, DATA_STREAM_VIEWER_PORT);
    view->AddSoccerCallback(&callback);
    view->show();
    WriteThread thread;
    if (!test_logger_mode_file_ && !kick_tuning_mode) {
      thread.start();
    } else {
      read_logger.BuildIndex();
      view->UpdateSliderSize();
    }
    ReadThread read_thread;
    read_thread.start();
    int return_value = app.exec();
    view->WriteCommentFile(comment_file);
    thread.terminate();
    read_thread.terminate();
    thread.wait();
    read_thread.wait();
    return return_value;
  } else {
    std::cout << "car_viewer_logs directory does not exist" << std::endl;
  }
  return 1;
}
