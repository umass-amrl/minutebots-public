// Copyright 2017 - 2018 joydeepb@cs.umass.edu
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

#ifndef SRC_GUI_VIEWER_H_
#define SRC_GUI_VIEWER_H_

#include <string>
#include <mutex>
#include <map>
#include "QSplitter"
#include "QWidget"
#include "QSlider"
#include "QComboBox"
#include "QLineEdit"

#include "gui/graph.h"
#include "gui/soccerview.h"
#include "gui/text_tree_viewer.h"
#include "logging/logger.h"
#include "soccer_logging.pb.h"

namespace gui {
class Viewer : public QWidget {
  Q_OBJECT

 public:
  Viewer(QWidget* parent,
         logger::ReadLogger*,
         const bool& tuning);
  // Update log contents.
  void Update(const MinuteBotsProto::SoccerDebugMessage& msg);
  // Handles the playback loop when reading from file or in
  // playback state.
  void Playback();
  bool IsLive();
  void SetLive(bool value);
  bool pause_status = false;
  int pause_count_;
  bool just_paused_;
  void UpdateSliderSize();
  void AdvanceSlider();
  void AddIndexEntry(const int& index);
  void SetCommentFile(string filename);
  void WriteCommentFile(string filename);

  template <class T>
  void AddSoccerCallback(const T* callback) {
    connect(&field_,
            SIGNAL(MouseDownCallback(Eigen::Vector2f,
                                     Qt::KeyboardModifiers,
                                     Qt::MouseButtons,
                                     gui::SelectionType,
                                     uint32_t,
                                     team::Team)),
            callback,
            SLOT(MouseDown(Eigen::Vector2f,
                           Qt::KeyboardModifiers,
                           Qt::MouseButtons,
                           gui::SelectionType,
                           uint32_t,
                           team::Team)));
    connect(&field_,
            SIGNAL(MouseUpCallback(Eigen::Vector2f,
                                   Qt::KeyboardModifiers,
                                   Qt::MouseButtons,
                                   gui::SelectionType,
                                   uint32_t,
                                   team::Team)),
            callback,
            SLOT(MouseUp(Eigen::Vector2f,
                         Qt::KeyboardModifiers,
                         Qt::MouseButtons,
                         gui::SelectionType,
                         uint32_t,
                         team::Team)));
    connect(&field_,
            SIGNAL(MouseMoveCallback(Eigen::Vector2f,
                                     Qt::KeyboardModifiers,
                                     Qt::MouseButtons,
                                     gui::SelectionType,
                                     uint32_t,
                                     team::Team)),
            callback,
            SLOT(MouseMove(Eigen::Vector2f,
                           Qt::KeyboardModifiers,
                           Qt::MouseButtons,
                           gui::SelectionType,
                           uint32_t,
                           team::Team)));
  }

 signals:
  // Used to register callbacks for keyboard events.
  void KeypressSignal(QKeyEvent* event);
 public slots: // NOLINT
  void HandleSlider();
  void HandleCommentBox();

 protected:
  // Keyboard keypress event handler.
  void keyPressEvent(QKeyEvent* event);
  // Window resizing event handler.
  void resizeEvent(QResizeEvent* event);
  // Event trigerred after the window is shown.
  void showEvent(QShowEvent* event);

 private:
  std::mutex slider_mutex;
  // Soccer field viewer.
  GLSoccerView field_;
  // Text tree log viewer.
  TextTreeViewer text_log_;
  // Graph visuzlizer.
  GLGraph graph_;
  // Horizontal divider.
  QSplitter horizontal_splitter_;
  // Vertical divider.
  QSplitter vertical_splitter_;
  // Slider for controlling the view.
  QSlider slider_;
  QComboBox tuning_box_;
  QLineEdit comment_box_;
  std::map<double, string> comment_map_;
  MinuteBotsProto::SoccerDebugMessage current_message_;
  std::fstream comment_file_;
  const bool tuning_mode_;
  logger::ReadLogger* file_logger_;
  bool log_status_ = false;
  bool slider_changed_ = false;
  int last_slider_value_ = 0;
  double last_time_ = 0;
  // Send the halt referee command.
  void RefHalt();
  // Send the go (force_start) referee command.
  void RefForceStart();
  // Reset the layout of the window.
  void Reset();
  // Pause the viewer
  void Pause();
  // Send the viewer live
  void Live();
  void SetTransition();
  void UnsetTransition();

  // Replay a message without saving to file
  void Replay(const MinuteBotsProto::SoccerDebugMessage& msg);
  // Move forward in history
  void Next();
  // Plays forward through the history at timed rate.
  void NextLive();
  // Move back in history
  void Previous();
  // Send a data entry to the graph.
  void UpdateGraph(const MinuteBotsProto::SoccerDebugMessage& msg);
  void UpdateComment();
  void TestSelection();

  DataEntry GenBallData(const MinuteBotsProto::SoccerDebugMessage& msg);
};

}  // namespace gui

#endif  // SRC_GUI_VIEWER_H_
