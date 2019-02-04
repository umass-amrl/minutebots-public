// Copyright 2017-2018 joydeepb@cs.umass.edu
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

#include <pthread.h>
#include <string>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "QGLWidget"

#include "gui/soccerview.h"
#include "gui/text_tree_viewer.h"
#include "soccer_logging.pb.h"

#ifndef SRC_GUI_GRAPH_H_
#define SRC_GUI_GRAPH_H_

namespace gui {
// A single data point in the graph.
struct DataEntry {
  // Timestamp.
  double t;
  // 2D location.
  Eigen::Vector2f loc;
  // Orientation in radians.
  float angle;
  // 2D Translation speed.
  float speed;
  // 2D velocity.
  Eigen::Vector2f vel;
  // Angular velocity.
  float omega;

  // Default constructor: do nothing.
  DataEntry() : t(0), loc(0, 0), angle(0), speed(0), vel(0, 0), omega(0) {}

  // Initialization constructor.
  DataEntry(double t,
            const Eigen::Vector2f loc,
            float angle,
            float speed,
            const Eigen::Vector2f vel,
            float omega) :
      t(t), loc(loc), angle(angle), speed(speed), vel(vel), omega(omega) {}
};

enum class VisualizationType : int {
  kSpeedLinear = 0,
  kSpeedAngular,
  kVelocityX,
  kVelocityY,
  kNumTypes
};

// An OpenGL graph visualizer.
class GLGraph : public QGLWidget {
  friend class Viewer;
  Q_OBJECT

 public:
  explicit GLGraph(QWidget* parent = nullptr);
  // Update drawing data.
  void Update();

  // Add a message to the front of the data array.
  void PushFront(const DataEntry& entry);

  // Add a message to the back of the data array.
  void PushBack(const DataEntry& entry);

  // Clear all graph data.
  void Clear();

 public slots:  // NOLINT(whitespace/indent)
  // Accept signals to update display.
  void RedrawSlot();

 signals:
  // Thread-safe way of scheduling display updates.
  void RedrawSignal();

 protected:
  // Event handler to accept paint requests from QT.
  void paintEvent(QPaintEvent* event);

  // Event handler to accept mouse move events from QT.
  void mouseMoveEvent(QMouseEvent* event);

  // Event handler to accept events from QT.
  void keyPressEvent(QKeyEvent* event);

  // Event handler to accept events from QT.
  void mousePressEvent(QMouseEvent* event);

  // Event handler to accept events from QT.
  void mouseReleaseEvent(QMouseEvent* event);

  // Return preferred widget size.
  QSize sizeHint() const;

  // Return minimum acceptable widget size.
  QSize minimumSizeHint() const;

 private:
  // Draw graph grid based on provided scale.
  void DrawGrid(float min_value, float max_value);

  // Get selected channel's data from a data entry.
  float GetValue(const DataEntry& entry);

 private:
  // Vector OpenGL text renderer.
  GLText gltext_;

  // Graph data.
  std::vector<DataEntry> data_;

  const size_t kMaxDataPoints = 15000;

  // Indicates that the user is selecting a time window.
  bool selecting_;

  int selection_start_;
  int selection_end_;
  int cursor_column_;

  // Data visualization type.
  VisualizationType type_;

  // Drawing mutex.
  pthread_mutex_t drawing_mutex_;
};

}  // namespace gui

#endif  // SRC_GUI_GRAPH_H_
