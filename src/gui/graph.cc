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

#include <pthread.h>
#include <utility>

#include "QMouseEvent"
#include "QWidget"

#include "gui/graph.h"
#include "gui/opengl_helpers.h"
#include "shared/common_includes.h"
#include "util/pthread_utils.h"

namespace {
const float kTextZ = 99;
const float kCursorZ = 98;
const float kSelectionZ = 97;
const float kGraphZ = 0;
// Epsilon time in ms.
const double kEpsilonTime = 0.1;
// Text height for graph stats.
const float kTextHeight = 15;
}  // namespace

namespace gui {
GLGraph::GLGraph(QWidget* parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers | QGL::DirectRendering)),
      gltext_(QFont("Monospace")),
      selecting_(false),
      selection_start_(0),
      selection_end_(0),
      cursor_column_(0),
      type_(VisualizationType::kSpeedLinear),
      drawing_mutex_(PTHREAD_MUTEX_INITIALIZER) {
  static const bool kGenerateTestData = false;
  connect(this, SIGNAL(RedrawSignal()), this, SLOT(RedrawSlot()));
  setMouseTracking(true);
  gltext_.SetRenderDepth(kTextZ);
  if (kGenerateTestData) {
    const Vector2f zero(0, 0);
    for (double t = 0; t < 15000.0; t += kTransmitPeriod) {
      data_.push_back(DataEntry(t, zero, sin(2.0 * M_PI * 0.001 * t),
                                5000.0 * (1.0 + sin(2.0 * M_PI * 0.0002 * t)),
                                zero, sin(2.0 * M_PI * 0.003 * t)));
    }
  }
}

QSize GLGraph::sizeHint() const { return QSize(1024, 200); }

QSize GLGraph::minimumSizeHint() const { return QSize(1024, 0); }

void PrintEntry(const DataEntry& entry) {
  printf("Time: %f Position: %f,%f Velocity: %f,%f Speed: %f \n", entry.t,
         entry.loc.x(), entry.loc.y(), entry.vel.x(), entry.vel.y(),
         entry.speed);
}

const char* VisualizationTypeString(const VisualizationType t) {
  switch (t) {
    case VisualizationType::kSpeedLinear: {
      return "Linear Speed";
    } break;
    case VisualizationType::kSpeedAngular: {
      return "Angular Speed";
    } break;
    case VisualizationType::kVelocityX: {
      return "Velocity.X";
    } break;
    case VisualizationType::kVelocityY: {
      return "Velocity.Y";
    } break;
    default: { return "[Unknown]"; } break;
  }
}

void GLGraph::paintEvent(QPaintEvent* event) {
  ScopedLock lock(&drawing_mutex_);
  makeCurrent();
  glClearColor(1, 1, 1, 1.0);

  glShadeModel(GL_SMOOTH);
  glDisable(GL_LIGHTING);
  glDisable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_MULTISAMPLE);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);

  // If there's no data, no need to draw.
  if (data_.size() < 3) {
    swapBuffers();
    return;
  }
  // Perform coordinate transform to account for scrolling.
  glLoadIdentity();

  // TODO(joydeepb): Change the color based on the channel being drawn, and
  // what has been selected.
  const opengl_helpers::Color4f kDrawColor(0.3, 0.3, 0.3, 1.0);
  const opengl_helpers::Color4f kGridColor(0.8, 0.8, 0.8, 1.0);
  const opengl_helpers::Color4f kTextColor(0.0, 0.0, 0.0, 1.0);

  double min_value = 0.0;
  double max_value = 7000.0;
  double grid_y_increment = 1000.0;
  if (type_ == VisualizationType::kSpeedAngular) {
    min_value = -2.0 * M_PI;
    max_value = 2.0 * M_PI;
    grid_y_increment = M_PI;
  } else if (type_ == VisualizationType::kVelocityX ||
             type_ == VisualizationType::kVelocityY) {
    min_value = -7000.0;
    max_value = 7000.0;
    grid_y_increment = 2000.0;
  }

  // Compute t_min and t_max from window width.
  const int window_width = width();
  const double t_max = data_.back().t;
  const double kPixelsPerFrame = 2;
  const double time_length =
      static_cast<double>(window_width) / kPixelsPerFrame * kTransmitPeriod;
  const double t_min = t_max - 0.001 * time_length;

  // Set OpenGL view port to t_min, t_max, y_min, y_max
  const int window_height = height();
  glViewport(0, 0, window_width, window_height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, time_length, min_value, max_value, -100, 100);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  {
    const double kGridInterval = 30.0 * kTransmitPeriod / 1000.0;
    const double tick_min = floor(t_min / kGridInterval) * kGridInterval;
    opengl_helpers::SetColor(kGridColor);
    glBegin(GL_LINES);
    for (double t = 1000.0 * (tick_min - t_min); t < 1000.0 * (t_max - t_min);
         t += 1000.0 * kGridInterval) {
      glVertex3f(t, min_value, kGraphZ);
      glVertex3f(t, max_value, kGraphZ);
    }
    const float grid_y_min =
        floor(min_value / grid_y_increment) * grid_y_increment;
    for (float y = grid_y_min; y < max_value; y += grid_y_increment) {
      glVertex3f(0, y, kGraphZ);
      glVertex3f(1000.0 * (t_max - t_min), y, kGraphZ);
    }
    glEnd();
    opengl_helpers::SetColor(opengl_helpers::Color4f::kBlack);
    glBegin(GL_LINES);
    if (type_ == VisualizationType::kSpeedLinear) {
      // Draw a line for the ball speed limit.
      glVertex3f(0, 6500, kGraphZ + 0.01);
      glVertex3f(1000.0 * (t_max - t_min), 6500, kGraphZ);
    }
    glVertex3f(0, 0, kGraphZ + 0.01);
    glVertex3f(1000.0 * (t_max - t_min), 0, kGraphZ);
    glEnd();
  }

  // Compute selection time.
  float selection_t_start =
      selection_start_ / kPixelsPerFrame * kTransmitPeriod;
  float selection_t_end = selection_end_ / kPixelsPerFrame * kTransmitPeriod;
  if (selection_t_end < selection_t_start) {
    std::swap(selection_t_end, selection_t_start);
  }
  const float cursor_time = cursor_column_ / kPixelsPerFrame * kTransmitPeriod;

  // Draw the actual data, and compute statistics.
  double mean_y = 0;
  double mean_x = 0;
  double mean_xy = 0;
  double mean_xx = 0;
  double num_selected_values = 0;
  // Data point under the cursor.
  DataEntry cursor_data;
  bool cursor_data_found = false;
  for (size_t i = 0; i < data_.size(); ++i) {
    const double x = 1000.0 * (data_[i].t - t_min);
    if (x < 0) {
      continue;
    }
    const float y = GetValue(data_[i]);
    if (x >= cursor_time && x <= cursor_time + kTransmitPeriod + kEpsilonTime) {
      cursor_data = data_[i];
      cursor_data_found = true;
    }
    if (x >= selection_t_start && x < selection_t_end) {
      mean_y += y;
      mean_x += x;
      mean_xy += x * y;
      mean_xx += x * x;
      ++num_selected_values;
    }
    opengl_helpers::DrawAlignedRectangle(Vector2f(x, 0), kTransmitPeriod, y,
                                         kGraphZ, kDrawColor);
  }
  mean_y /= num_selected_values;
  mean_x /= num_selected_values;
  mean_xy /= num_selected_values;
  mean_xx /= num_selected_values;
  const double acc =
      1000.0 * (mean_xy - mean_x * mean_y) / (mean_xx - Sq(mean_x));

  // Draw cursor.
  opengl_helpers::DrawAlignedRectangle(
      Vector2f(cursor_column_ / kPixelsPerFrame * kTransmitPeriod, min_value),
      kTransmitPeriod, max_value - min_value, kCursorZ,
      opengl_helpers::Color4f(0.8, 0.8, 0.8, 0.75));

  // Draw selection window.
  if (selection_end_ != selection_start_) {
    opengl_helpers::DrawAlignedRectangle(
        Vector2f(selection_t_start, min_value),
        selection_t_end - selection_t_start, max_value - min_value, kSelectionZ,
        opengl_helpers::Color4f(0.8, 0.8, 1, 0.5));
  }

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(0, window_width, 0, window_height, -100, 100);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  opengl_helpers::SetColor(kTextColor);

  string stats_string;
  if (cursor_data_found) {
    stats_string = StringPrintf(
        "t:%f\nloc:%.1f,%.1f\nvel:%.1f,%.1f (%.1f)\n"
        "angle:%.2fdeg\nang_vel:%.1fdeg/s\n",
        cursor_data.t, cursor_data.loc.x(), cursor_data.loc.y(),
        cursor_data.vel.x(), cursor_data.vel.y(), cursor_data.speed,
        RadToDeg(cursor_data.angle), RadToDeg(cursor_data.omega));
  }
  if (num_selected_values > 2) {
    stats_string +=
        StringPrintf("mean:%.3f\nacc:%.3f\nduration:%.2fms\nn:%d", mean_y, acc,
                     selection_t_end - selection_t_start,
                     static_cast<int>(num_selected_values));
  }
  gltext_.DrawString(Vector2f(kTextHeight, window_height - kTextHeight), 0,
                     kTextHeight,
                     string("Mode[v]: ") + VisualizationTypeString(type_));
  if (stats_string.length() > 0) {
    gltext_.DrawString(Vector2f(kTextHeight, window_height - 2 * kTextHeight),
                       0, kTextHeight, stats_string);
  }
  swapBuffers();
}

float GLGraph::GetValue(const DataEntry& entry) {
  // TODO(joydeepb): Return the entry channel based on what has been selected.
  switch (type_) {
    case VisualizationType::kSpeedLinear: {
      return (entry.speed);
    } break;
    case VisualizationType::kSpeedAngular: {
      return (entry.omega);
    } break;
    case VisualizationType::kVelocityX: {
      return (entry.vel.x());
    } break;
    case VisualizationType::kVelocityY: {
      return (entry.vel.y());
    } break;
    default: { return 0; } break;
  }
}

void GLGraph::Update() {
  // TODO(joydeepb): update data from log.
}

void GLGraph::Clear() {
  {
    ScopedLock lock(&drawing_mutex_);
    data_.clear();
  }
  RedrawSignal();
}

void GLGraph::keyPressEvent(QKeyEvent* event) {
  switch (event->key()) {
    case Qt::Key_V: {
      const int N = static_cast<int>(VisualizationType::kNumTypes);
      const int i = static_cast<int>(type_);
      type_ = static_cast<VisualizationType>((i + 1) % N);
    } break;
  }
  RedrawSignal();
}

void GLGraph::mouseMoveEvent(QMouseEvent* event) {
  cursor_column_ = event->x();
  if (selecting_) {
    selection_end_ = event->x();
  }
  RedrawSignal();
}

void GLGraph::mousePressEvent(QMouseEvent* event) {
  selecting_ = true;
  selection_start_ = event->x();
  selection_end_ = event->x();
  RedrawSignal();
}

void GLGraph::mouseReleaseEvent(QMouseEvent* event) {
  selecting_ = false;
  selection_end_ = event->x();
  RedrawSignal();
}

void GLGraph::RedrawSlot() { update(); }

void GLGraph::PushBack(const DataEntry& entry) {
  data_.push_back(entry);
  if (data_.size() > kMaxDataPoints) {
    data_.erase(data_.begin());
  }
  RedrawSignal();
}

void GLGraph::PushFront(const DataEntry& entry) {
  data_.insert(data_.begin(), entry);
  if (data_.size() > kMaxDataPoints) {
    data_.pop_back();
  }
}

}  // namespace gui
