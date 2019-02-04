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

#include <algorithm>
#include <utility>

#include "GL/gl.h"
#include "GL/glu.h"
#include "QApplication"
#include "QDesktopWidget"
#include "QGLWidget"
#include "QMouseEvent"
#include "QScreen"

#include "soccer_logging.pb.h"
#include "gui/gltext.h"
#include "gui/opengl_helpers.h"
#include "gui/text_tree_viewer.h"
#include "shared/common_includes.h"

using opengl_helpers::Color4f;
using opengl_helpers::SetColor;
using opengl_helpers::DrawAlignedRectangle;
using opengl_helpers::DrawLine;
using std::binary_search;
using std::make_pair;
using std::uint_fast32_t;
using MinuteBotsProto::TextTree;

namespace {
const Color4f kLineColor(0.6, 0.6, 0.6, 1.0);
const Color4f kBackgroundColor(0.0, 0.0, 0.0, 1.0);
const Color4f kTextColor(1.0, 1.0, 1.0, 1.0);
const Color4f kHighlightColor(0.1, 0.1, 0.2, 1.0);
}  // namespace

namespace gui {

TextTreeViewer::TextTreeViewer(QWidget* parent) :
    QGLWidget(QGLFormat(QGL::SampleBuffers | QGL::DirectRendering)),
    gltext_(QFont("Monospace")),
    drag_scroll_(false),
    scale_(1),
    text_origin_(0, 0),
    highlight_line_(false),
    char_width_(1) {
  char_width_ = gltext_.GetWidth("A");
  SetCanonicalScale();
  connect(this, SIGNAL(RedrawSignal()), this, SLOT(RedrawSlot()));
  setMouseTracking(true);
  CHECK_EQ(pthread_mutex_init(&log_mutex_, nullptr), 0);
}

void TextTreeViewer::Update(const MinuteBotsProto::SoccerDebugMessage& msg) {
  ScopedLock lock(&log_mutex_);
  if (msg.has_text_log()) {
    log_ = msg.text_log();
    RedrawSignal();
  }
}

void TextTreeViewer::SetCanonicalScale() {
  // TODO(joydeepb): Derive these from screen DPI.
  canonical_scale_ = 18;
}

Vector2f TextTreeViewer::TextToWindowCoords(const Vector2f& x) {
  Vector2f y(x.x(), -x.y());
  y = (y - text_origin_) * scale_ * canonical_scale_;
  return y;
}

Vector2f TextTreeViewer::WindowToTextCoords(const Vector2f& x) {
  Vector2f y(x.x(), -x.y());
  y = y / (scale_ * canonical_scale_) + text_origin_;
  return y;
}

void TextTreeViewer::initializeGL() {
}

void TextTreeViewer::keyPressEvent(QKeyEvent* event) {
  if (event->key() == Qt::Key_Escape) {
    exit(0);
  }
}

void TextTreeViewer::leaveEvent(QEvent* event) {
  highlight_line_ = false;
  RedrawSignal();
}

void TextTreeViewer::mouseMoveEvent(QMouseEvent* event) {
  // If mouse down is true, drag scroll the text.
  if (drag_scroll_) {
    const Vector2f loc(event->x(), event->y());
    Vector2f delta(loc - mouse_down_loc_);
    text_origin_ = delta / (scale_ * canonical_scale_) + previous_text_origin_;
    text_origin_.x() = min(0.0f, text_origin_.x());
    text_origin_.y() = min(0.0f, text_origin_.y());
    SetupViewport();
    RedrawSignal();
  } else {
    highlight_line_ = true;
    line_to_highlight_ =
        ceil(WindowToTextCoords(Vector2f(event->x(), event->y())).y());
    RedrawSignal();
  }
}

void TextTreeViewer::mousePressEvent(QMouseEvent* event) {
  if (event->modifiers() == Qt::NoModifier &&
      event->button() == Qt::LeftButton) {
    drag_scroll_ = true;
    mouse_down_loc_ = Vector2f(event->x(), event->y());
    previous_text_origin_ = text_origin_;
  }
}

void TextTreeViewer::mouseReleaseEvent(QMouseEvent* event) {
  if (event->modifiers() == Qt::NoModifier) {
    // Collapse / un-collapse text.
    const Vector2f window_coords(event->x(), event->y());
    const Vector2f text_coords = WindowToTextCoords(window_coords);
    const uint_fast32_t node_height =
        static_cast<uint_fast32_t>(-floor(text_coords.y())) - 1;
    for (const auto& node : collapsible_nodes_) {
      if (node.first == node_height) {
        const string& node_string = node.second;
        const auto clicked_node = std::find(collapsed_nodes_.begin(),
                                            collapsed_nodes_.end(),
                                            node_string);
        if (clicked_node == collapsed_nodes_.end()) {
          collapsed_nodes_.push_back(node_string);
        } else {
          collapsed_nodes_.erase(clicked_node);
        }
      }
    }
  }
  drag_scroll_ = false;
  RedrawSignal();
}

void TextTreeViewer::SetupViewport() {
  makeCurrent();
  const int window_width = width();
  const int window_height = height();
  const float draw_scale = 1.0 / (scale_ * canonical_scale_);
  glViewport(0, 0, window_width, window_height);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-text_origin_.x(),
          window_width * draw_scale - text_origin_.x(),
          -window_height * draw_scale + text_origin_.y(),
          text_origin_.y(),
          -100,
          100);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}


void TextTreeViewer::paintEvent(QPaintEvent* event) {
  makeCurrent();
  glClearColor(0, 0, 0, 1.0);
  glShadeModel(GL_SMOOTH);
  glDisable(GL_LIGHTING);
  glDisable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Perform coordinate transform to account for scrolling.
  glLoadIdentity();

  {
    // Clear the list of collapsible nodes, it will be re-evaluated by
    // DrawTreeLog.
    collapsible_nodes_.clear();
    // Do the actual text rendering here.
    ScopedLock lock(&log_mutex_);
    // Top level label is not shown if it is empty.
    if (log_.text().length() == 0) {
      float y = 0;
      for (int i = 0; i < log_.sub_tree_size(); ++i) {
        y = DrawTreeLog(log_.sub_tree(i), char_width_, y);
      }
    } else {
      DrawTreeLog(log_, 0, 0);
    }
  }

  if (highlight_line_) {
    static const float kZ = -1e-8;
    const float line_width =
         width() / (scale_ * canonical_scale_) - text_origin_.x();
    DrawAlignedRectangle(Vector2f(0, line_to_highlight_ - 1),
                         line_width,
                         1,
                         kZ,
                         kHighlightColor);
  }
  swapBuffers();
}

void TextTreeViewer::DrawSubTreeMarker(float x, float y, bool collapsed) {
  const Vector2f p0(x - char_width_, y);
  SetColor(kLineColor);
  if (collapsed) {
    gltext_.DrawString(p0, 0, 1, "+", GLText::LeftAligned, GLText::TopAligned);
  } else {
    gltext_.DrawString(p0, 0, 1, "-", GLText::LeftAligned, GLText::TopAligned);
  }
}

bool TextTreeViewer::IsLineVisible(float y) {
  const float first_line = y - text_origin_.y();
  return (first_line * scale_ * canonical_scale_ > -height() &&
          first_line < 1.0f);
}

float TextTreeViewer::DrawTreeLog(const TextTree& log,
                                  float x,
                                  float y) {
  static const float kIndent = 2.0 * char_width_;
  const bool is_collapsed =
      (std::find(collapsed_nodes_.begin(), collapsed_nodes_.end(), log.text())
      != collapsed_nodes_.end());
  SetColor(kTextColor);
  if (IsLineVisible(y)) {
    gltext_.DrawString(Vector2f(x, y),
                      0,
                      1,
                      log.text(),
                      GLText::LeftAligned,
                      GLText::TopAligned);
  }
  if (log.sub_tree_size() > 0) {
    collapsible_nodes_.push_back(make_pair(-y, string(log.text())));
    DrawSubTreeMarker(x, y, is_collapsed);
  }
  --y;
  const float sub_tree_begin = y;
  if (!is_collapsed) {
    for (int i = 0; i < log.sub_tree_size(); ++i) {
      y = DrawTreeLog(log.sub_tree(i), x + kIndent, y);
    }
    if (log.sub_tree_size() > 0) {
      DrawLine(Vector2f(x + 0.25 * kIndent, sub_tree_begin),
               Vector2f(x + 0.25 * kIndent, y),
               0.01,
               0,
               kLineColor);
    }
  }
  return y;
}

void TextTreeViewer::RedrawSlot() {
  update();
}

void TextTreeViewer::resizeGL(int width, int height) {
  SetupViewport();
}

void TextTreeViewer::resizeEvent(QResizeEvent* event) {
  QGLWidget::resizeEvent(event);
  RedrawSignal();
}

void TextTreeViewer::wheelEvent(QWheelEvent* event) {
  if (event->modifiers() == Qt::NoModifier) {
    // Scroll text.
  } else if (event->modifiers() == Qt::ControlModifier) {
    // Zoom in or out.
  }
}

}  // namespace gui

