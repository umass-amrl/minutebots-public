// Copyright 2018 joydeepb@cs.umass.edu
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

#ifndef SRC_GUI_TEXT_TREE_VIEWER_H_
#define SRC_GUI_TEXT_TREE_VIEWER_H_

#include <pthread.h>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "GL/glu.h"
#include "QFont"
#include "QGLWidget"
#include "eigen3/Eigen/Dense"

#include "gui/gltext.h"
#include "soccer_logging.pb.h"

namespace gui {

class TextTreeViewer : public QGLWidget {
  friend class Viewer;
  Q_OBJECT

 public:
  explicit TextTreeViewer(QWidget* parent = nullptr);

  // Preferred size for the QT widget.
  QSize sizeHint() const { return QSize(580, 1000); }
  QSize minimumSizeHint() const { return QSize(0, 0); }
  // Text-Space Coordinates:
  // Origin is in the top left, X increasing left to right, Y increasing top to
  // bottom. Each line of text is unit height. Each character is unit width,
  // since the font is monospace.

  // Find a canonical scale of text height to window pixel height.
  void SetCanonicalScale();

  // Convert text-space coordinates to window coordinates.
  Eigen::Vector2f TextToWindowCoords(const Eigen::Vector2f& x);

  // Convert window coordinates to text-space coordinates.
  Eigen::Vector2f WindowToTextCoords(const Eigen::Vector2f& x);

  // Update log contents.
  void Update(const MinuteBotsProto::SoccerDebugMessage& msg);

 public slots:  // NOLINT(whitespace/indent)
  // Accept signals to update display.
  void RedrawSlot();

 signals:
  // Thread-safe way of scheduling display updates.
  void RedrawSignal();

 protected:
  // Event handler to accept paint requests from QT.
  void paintEvent(QPaintEvent* event);

  // Event handler to accept mouse wheel events from QT.
  void wheelEvent(QWheelEvent* event);

  // Event handler to accept mouse move events from QT.
  void mouseMoveEvent(QMouseEvent* event);

  // Event handler to accept events from QT.
  void keyPressEvent(QKeyEvent* event);

  // Event handler to accept events from QT.
  void mousePressEvent(QMouseEvent* event);

  // Event handler to accept events from QT.
  void mouseReleaseEvent(QMouseEvent* event);

  // Event handler to accept events from QT.
  void resizeEvent(QResizeEvent* event);

  // Event handler to accept events from QT.
  void initializeGL();

  // Event handler to accept events from QT.
  void resizeGL(int width, int height);

  // Indicates that the widget lost focus.
  void leaveEvent(QEvent* event);

 private:
  // Vector OpenGL text renderer.
  GLText gltext_;
  // Indicates that the user is drag-scrolling the text.
  bool drag_scroll_;
  // Canonical scale for text, in terms of pixel height of text.
  float canonical_scale_;
  // Text size magnification.
  float scale_;
  // Window-space origin in text-space coordinates.
  Eigen::Vector2f text_origin_;
  // Location in window space coordinates that the user pressed the mouse
  // button.
  Eigen::Vector2f mouse_down_loc_;
  // Previous value of text origin before the start of mouse drag operation.
  Eigen::Vector2f previous_text_origin_;
  // Enable line highlighting.
  bool highlight_line_;
  // Which line to highlight.
  float line_to_highlight_;
  // Log contents.
  MinuteBotsProto::TextTree log_;
  // Mutex for coordinating access to log.
  pthread_mutex_t log_mutex_;
  // Width of a single character.
  float char_width_;
  // List of depths and strings at collapsible nodes.
  std::vector<std::pair<std::uint_fast32_t, std::string>> collapsible_nodes_;
  // List of collapsed nodes.
  std::vector<std::string> collapsed_nodes_;

  // Set up drawing window, scaling, pan.
  void SetupViewport();
  // Returns true iff a line at the specified y-coordinate will be visible
  // on screen.
  bool IsLineVisible(float y);
  // Render the tree log text.
  float DrawTreeLog(const MinuteBotsProto::TextTree& log, float x, float y);
  // Draw a sub-tree marker.
  void DrawSubTreeMarker(float x, float y, bool collapsed);
};

}  // namespace gui

#endif  // SRC_GUI_TEXT_TREE_VIEWER_H_
