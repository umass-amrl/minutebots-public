// Copyright 2011-2019 joydeepb@cs.umass.edu
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
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <string>

#include "GL/glu.h"
#include "QFileDialog"
#include "QGLWidget"
#include "QMouseEvent"
#include "QMutex"

#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include "messages_robocup_ssl_wrapper.pb.h"

#include "gui/gltext.h"
#include "gui/opengl_helpers.h"
#include "simplecar_sim/simplecar_view.h"
#include "shared/common_includes.h"
#include "state/team.h"

using gui::GLText;
using opengl_helpers::Color4f;
using opengl_helpers::DrawAlignedRectangle;
using opengl_helpers::DrawArc;
using opengl_helpers::DrawLine;
using opengl_helpers::SetColor;
using std::fprintf;
using team::Team;

namespace {
const float kMinZValue = -10;
const float kMaxZValue = 10;

// Z-plane to draw field markings.
const float kFieldZ = -2.0;

// Z-plane to draw robots.
const float kRobotZ = -1.0;

// Z-plane to draw ball(s).
const float kBallZ = 1.0;

// Z-plane to draw debug drawings.
const float kDebugDrawingsZ = 2.0;

// Color of field markings.
const Color4f kFieldLinesColor(1.0, 1.0, 1.0, 1.0);

const Color4f kBallColor(1.0, 0.5059, 0.0, 1.0);
const Color4f kBallOutlineColor(0.8706, 0.3490, 0.0, 1.0);

const Color4f kBlueRobotColor(0.1961, 0.3922, 0.8118, 1.0);
const Color4f kYellowRobotColor(1.0, 0.9529, 0.2431, 1.0);

const Color4f kBlueRobotOutlineColor(0.0706, 0.2314, 0.6275, 1.0);
const Color4f kYellowRobotOutlineColor(0.8, 0.6157, 0.0, 1.0);

// Thickness of debug drawing lines and arcs, in pixels.
const float kDebugLineThickness = 8.0;

// Helper function to translate team enum from protobuf to internal enum.
team::Team ProtoTeamToEnum(const MinuteBotsProto::RobotState::Team& t) {
  switch (t) {
    case MinuteBotsProto::RobotState_Team_TEAM_BLUE: {
      return team::Team::BLUE;
    } break;
    case MinuteBotsProto::RobotState_Team_TEAM_YELLOW: {
      return team::Team::YELLOW;
    } break;
    default: {
      // This should never happen.
      CHECK(false);
    }
  }
}

// Helper function to convert from MinuteBotsProto::Vector2f to Eigen::Vector2f.
Vector2f ProtoVectorToEigen(const MinuteBotsProto::Vector2f& v) {
  return (Vector2f(v.x(), v.y()));
}

// Helper function to convert proto Color to opengl_helpers::Color4f.
Color4f ProtoColorToOpenGL(const MinuteBotsProto::Color& c) {
  return (Color4f(c.r(), c.g(), c.b(), c.a()));
}

}  // namespace

namespace gui {
const int GLCarView::kPreferedWidth = 1024;
const int GLCarView::kPreferedHeight = 768;

QSize GLCarView::sizeHint() const {
  return QSize(kPreferedWidth, kPreferedHeight);
}

GLCarView::GLCarView(QWidget* parent)
    : QGLWidget(
          QGLFormat(QGL::DoubleBuffer | QGL::DepthBuffer | QGL::SampleBuffers),
          parent),
      blue_robot_gl_list_(GL_INVALID_VALUE),
      yellow_robot_gl_list_(GL_INVALID_VALUE),
      field_gl_list_(GL_INVALID_VALUE),
      t_last_redraw_(0),
      selection_(GuiSelectionType::kNoneSelected),
      selected_robot_id_(-1),
      selected_robot_team_(Team::BLUE),
      selected_ball_index_(-1),
      view_interaction_(false) {
  view_scale_ = (kFieldLength + 2.0 * (kGoalDepth + kFieldLineWidth)) /
                sizeHint().width();
  view_scale_ = max<float>(
      view_scale_, (kFieldWidth + kFieldBoundary) / sizeHint().height());
  x_offset_ = y_offset_ = 0.0;
  setAutoFillBackground(false);
  connect(this, SIGNAL(RedrawSignal()), this, SLOT(Redraw()));
  QFont robot_id_font = this->font();
  robot_id_font.setWeight(QFont::Bold);
  robot_id_font.setPointSize(80);
  gl_text_ = gui::GLText(robot_id_font);
}

int ButtonNumber(const QMouseEvent& event) {
  if (event.buttons().testFlag(Qt::LeftButton)) {
    return 0;
  } else if (event.buttons().testFlag(Qt::RightButton)) {
    return 1;
  } else if (event.buttons().testFlag(Qt::MidButton)) {
    return 2;
  }
  return -1;
}

void GLCarView::Redraw() {
  // Maximum display rate, to limit OpenGL drawing rate.
  const float kMaxDisplayRate = 61.0;
  const double t = GetMonotonicTime();
  if (kMaxDisplayRate * (t - t_last_redraw_) < 1.0) return;
  graphics_mutex_.lock();
  update();
  graphics_mutex_.unlock();
  t_last_redraw_ = t;
}

void GLCarView::mousePressEvent(QMouseEvent* event) {
  const bool alt = event->modifiers().testFlag(Qt::AltModifier);
  const bool ctrl = event->modifiers().testFlag(Qt::ControlModifier);
  const bool shift = event->modifiers().testFlag(Qt::ShiftModifier);
  const int button_num = ButtonNumber(*event);

  view_interaction_ = false;
  if (!alt && !ctrl && !shift && (button_num == 0 || button_num == 2)) {
    // Start Pan / Zoom
    mouse_start_x_ = event->x();
    mouse_start_y_ = event->y();
    RedrawSignal();
  }
  const Vector2f p_screen(event->x(), event->y());
  const Vector2f p_world = ScreenToWorld(p_screen);
  MouseDownCallback(p_world,
                    event->modifiers(),
                    event->buttons(),
                    selection_,
                    selected_robot_id_,
                    selected_robot_team_);
}

void GLCarView::DrawSelection() {
  if (selection_ == GuiSelectionType::kBallSelected) {
    if (selected_ball_index_ < debug_msg_.world_state().balls_size()) {
      const auto& ball = debug_msg_.world_state().balls(selected_ball_index_);
      const Vector2f loc(ball.x(), ball.y());
      DrawArc(loc,
              kBallRadius + 5.0,
              kBallRadius + 8.0,
              0,
              M_2PI,
              kBallZ,
              Color4f::kRed);
    }
  } else if (selection_ == GuiSelectionType::kRobotSelected) {
    for (const auto& robot : debug_msg_.world_state().robots()) {
      if (robot.robot_id() == selected_robot_id_ &&
          ProtoTeamToEnum(robot.team()) == selected_robot_team_) {
        const Vector2f robot_loc(robot.pose().loc().x(),
                                 robot.pose().loc().y());
        DrawArc(robot_loc,
                kRobotRadius + 5.0,
                kRobotRadius + 8.0,
                0,
                M_2PI,
                kBallZ,
                Color4f::kRed);
      }
    }
  }
}

void GLCarView::UpdateSelection(const Vector2f& loc) {
  graphics_mutex_.lock();
  float sq_min_dist = kRobotRadius * kRobotRadius;
  selected_robot_id_ = -1;
  selection_ = GuiSelectionType::kNoneSelected;
  selected_ball_index_ = -1;
  for (const auto& robot : debug_msg_.world_state().robots()) {
    const Vector2f robot_loc(robot.pose().loc().x(), robot.pose().loc().y());
    const float sq_dist = (loc - robot_loc).squaredNorm();
    if (sq_dist < sq_min_dist) {
      selected_robot_id_ = robot.robot_id();
      selected_robot_team_ = ProtoTeamToEnum(robot.team());
      sq_min_dist = sq_dist;
      selection_ = GuiSelectionType::kRobotSelected;
    }
  }
  // Only select the ball if no robot has been found.
  if (selection_ == GuiSelectionType::kNoneSelected) {
    for (int i = 0; i < debug_msg_.world_state().balls_size(); ++i) {
      const auto& ball = debug_msg_.world_state().balls(i);
      const Vector2f ball_loc(ball.x(), ball.y());
      const float sq_dist = (loc - ball_loc).squaredNorm();
      if (sq_dist < sq_min_dist) {
        selection_ = GuiSelectionType::kBallSelected;
        selected_ball_index_ = i;
      }
    }
  }
  graphics_mutex_.unlock();
}

void GLCarView::mouseReleaseEvent(QMouseEvent* event) {
  setCursor(Qt::ArrowCursor);
  const Vector2f p_screen(event->x(), event->y());
  const Vector2f p_world = ScreenToWorld(p_screen);
  if (!view_interaction_ && !event->modifiers()) {
    UpdateSelection(p_world);
    RedrawSignal();
  }
  if (!view_interaction_) {
    MouseUpCallback(p_world,
                    event->modifiers(),
                    event->buttons(),
                    selection_,
                    selected_robot_id_,
                    selected_robot_team_);
  }
}

void GLCarView::mouseMoveEvent(QMouseEvent* event) {
  const bool alt = event->modifiers().testFlag(Qt::AltModifier);
  const bool ctrl = event->modifiers().testFlag(Qt::ControlModifier);
  const bool shift = event->modifiers().testFlag(Qt::ShiftModifier);
  const int button_num = ButtonNumber(*event);

  if (!alt && !ctrl && !shift) {
    view_interaction_ = true;
  }
  if (view_interaction_ && button_num == 0) setCursor(Qt::ClosedHandCursor);
  if (view_interaction_ && button_num == 2) setCursor(Qt::SizeVerCursor);
  if (view_interaction_ && button_num == 0) {
    // Pan
    x_offset_ -= view_scale_ * static_cast<float>(event->x() - mouse_start_x_);
    y_offset_ += view_scale_ * static_cast<float>(event->y() - mouse_start_y_);
    mouse_start_x_ = event->x();
    mouse_start_y_ = event->y();
    RecomputeProjection();
    RedrawSignal();
  } else if (view_interaction_ && button_num == 2) {
    // Zoom
    float zoomRatio = static_cast<float>(event->y() - mouse_start_y_) / 500.0;
    view_scale_ = view_scale_ * (1.0 + zoomRatio);
    RecomputeProjection();
    mouse_start_x_ = event->x();
    mouse_start_y_ = event->y();
    RedrawSignal();
  }
  const Vector2f p_screen(event->x(), event->y());
  const Vector2f p_world = ScreenToWorld(p_screen);
  MouseMoveCallback(p_world,
                    event->modifiers(),
                    event->buttons(),
                    selection_,
                    selected_robot_id_,
                    selected_robot_team_);
}

void GLCarView::wheelEvent(QWheelEvent* event) {
  float zoomRatio = -static_cast<float>(event->delta()) / 1000.0;
  view_scale_ = view_scale_ * (1.0 + zoomRatio);
  RecomputeProjection();
  RedrawSignal();
}

void GLCarView::SaveImage() {
  const QString filename =
      QFileDialog::getSaveFileName(this, QString("Save Image"), QString(),
                                   QString("Image Files (*.png *.jpg *.bmp)"));
  if (filename.length() == 0) return;
  Redraw();
  paintGL();
  QImage image = this->grabFrameBuffer();
  image.save(filename);
}

void GLCarView::keyPressEvent(QKeyEvent* event) {
  static const bool debug = false;
  bool ctrl_key = event->modifiers().testFlag(Qt::ControlModifier);
  if (debug) printf("KeyPress: 0x%08X\n", event->key());
  switch (event->key()) {
    case Qt::Key_Space: {
      ResetView();
    } break;
    case Qt::Key_P: {
      if (ctrl_key) {
        SaveImage();
      }
    } break;
    case Qt::Key_B: {
      if (debug_msg_.world_state().balls_size() > 0) {
        selection_ = GuiSelectionType::kBallSelected;
        selected_ball_index_ = 0;
        RedrawSignal();
      }
    }
    default: {
      // Do nothing.
    }
  }
}

void GLCarView::ResetView() {
  view_scale_ = (kFieldLength + 2.0 * (kGoalDepth + kFieldLineWidth)) / width();
  view_scale_ =
      max<float>(view_scale_, (kFieldWidth + kFieldBoundary) / height());
  x_offset_ = y_offset_ = 0.0;
  RecomputeProjection();
  RedrawSignal();
}

void GLCarView::resizeEvent(QResizeEvent* event) {
  QGLWidget::resizeEvent(event);
  Redraw();
}

void GLCarView::RecomputeProjection() {
  makeCurrent();
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-0.5 * view_scale_ * width() + x_offset_,
          0.5 * view_scale_ * width() + x_offset_,
          -0.5 * view_scale_ * height() + y_offset_,
          0.5 * view_scale_ * height() + y_offset_, kMinZValue, kMaxZValue);
  glMatrixMode(GL_MODELVIEW);
}

Vector2f GLCarView::ScreenToWorld(const Vector2f& p_screen) const {
  const Vector2f p_world(
      (p_screen.x() -0.5 * width()) * view_scale_  + x_offset_,
      (0.5 * height() - p_screen.y()) * view_scale_ + y_offset_);
  return p_world;
}


Vector2f GLCarView::WorldToScreen(const Vector2f& p_world) const {
  const Vector2f p_screen(
      (p_world.x() - x_offset_) / view_scale_ + 0.5 * width(),
      -((p_world.y() - y_offset_) / view_scale_ - 0.5 * height()));
  return p_screen;
}

void GLCarView::resizeGL(int width, int height) {
  makeCurrent();
  glViewport(0, 0, width, height);
  RecomputeProjection();
}

void GLCarView::initializeGL() {
  makeCurrent();
  field_gl_list_ = glGenLists(1);
  CHECK_NE(field_gl_list_, GL_INVALID_VALUE);
  glNewList(field_gl_list_, GL_COMPILE);
  DrawFieldLines();
  glEndList();

  blue_robot_gl_list_ = glGenLists(1);
  CHECK_NE(blue_robot_gl_list_, GL_INVALID_VALUE);
  glNewList(blue_robot_gl_list_, GL_COMPILE);
  CompileRobotDisplayList(Team::BLUE);
  glEndList();

  yellow_robot_gl_list_ = glGenLists(1);
  CHECK_NE(yellow_robot_gl_list_, GL_INVALID_VALUE);
  glNewList(yellow_robot_gl_list_, GL_COMPILE);
  CompileRobotDisplayList(Team::YELLOW);
  glEndList();
}

void GLCarView::DrawRobots() {
  for (int i = 0; i < debug_msg_.world_state().robots_size(); ++i) {
    auto robot = debug_msg_.world_state().robots(i);
    DrawRobot(Vector2f(robot.pose().loc().x(), robot.pose().loc().y()),
              robot.pose().angle(), robot.confidence(), robot.robot_id(),
              ProtoTeamToEnum(robot.team()));
    if (robot.has_pose_raw()) {
      const Vector2f raw_loc(robot.pose_raw().loc().x(),
                            robot.pose_raw().loc().y());
      const float raw_angle(robot.pose_raw().angle());
      DrawRobotOutline(raw_loc,
                       raw_angle,
                       robot.robot_id(),
                       ProtoTeamToEnum(robot.team()));
    }
  }
}

void GLCarView::DrawBalls() {
  for (int i = 0; i < debug_msg_.world_state().balls_size(); ++i) {
    DrawBall(ProtoVectorToEigen(debug_msg_.world_state().balls(i)));
  }
}

void GLCarView::DrawLines() {
  const float line_width = 0.5 * kDebugLineThickness * view_scale_;
  // An epsilon to add to the Z-ordering so that alpha blending is performed
  // correctly.
  const float epsilon =
      0.1 / static_cast<float>(debug_msg_.drawings().lines_size());
  for (int i = 0; i < debug_msg_.drawings().lines_size(); ++i) {
    auto line = debug_msg_.drawings().lines(i);
    DrawLine(ProtoVectorToEigen(line.value().v0()),
             ProtoVectorToEigen(line.value().v1()), line_width,
             kFieldZ + static_cast<float>(i) * epsilon,
             ProtoColorToOpenGL(line.color()));
  }
}

void GLCarView::DrawPoints() {
  const float point_width = 0.5 * kDebugLineThickness * view_scale_;
  // An epsilon to add to the Z-ordering so that alpha blending is performed
  // correctly.
  const float epsilon =
      0.1 / static_cast<float>(debug_msg_.drawings().points_size());
  for (int i = 0; i < debug_msg_.drawings().points_size(); ++i) {
    auto point = debug_msg_.drawings().points(i);
    DrawAlignedRectangle(Vector2f(point.value().x() - point_width,
                                  point.value().y() - point_width),
                         2.0 * point_width, 2.0 * point_width,
                         kDebugDrawingsZ + static_cast<float>(i) * epsilon,
                         ProtoColorToOpenGL(point.color()));
  }
}

void GLCarView::DrawArcs() {
  const float line_width = 0.5 * kDebugLineThickness * view_scale_;
  // An epsilon to add to the Z-ordering so that alpha blending is performed
  // correctly.
  const float epsilon =
      0.1 / static_cast<float>(debug_msg_.drawings().arcs_size());
  for (int i = 0; i < debug_msg_.drawings().arcs_size(); ++i) {
    auto arc = debug_msg_.drawings().arcs(i);
    const float r0 = max(0.0f, arc.radius() - line_width);
    const float r1 = arc.radius() + line_width;
    DrawArc(ProtoVectorToEigen(arc.center()),
            r0,
            r1,
            arc.angle_start(),
            arc.angle_end(),
            kDebugDrawingsZ + static_cast<float>(i) * epsilon,
            ProtoColorToOpenGL(arc.color()));
  }
}

void GLCarView::DrawEllipses() {
  const float line_width = 0.5 * kDebugLineThickness * view_scale_;
  // An epsilon to add to the Z-ordering so that alpha blending is performed
  // correctly.
  const float epsilon =
      0.1 / static_cast<float>(debug_msg_.drawings().ellipses_size());
  for (int i = 0; i < debug_msg_.drawings().ellipses().size(); ++i) {
    auto ellipse = debug_msg_.drawings().ellipses(i);
    DrawEllipse(ProtoVectorToEigen(ellipse.center()),
                ellipse.radius_1() - line_width,
                ellipse.radius_2() + line_width, ellipse.angle(),
                kDebugDrawingsZ + static_cast<float>(i) * epsilon,
                ProtoColorToOpenGL(ellipse.color()));
  }
}

void GLCarView::DrawCircles() {
  const float line_width = 0.5 * kDebugLineThickness * view_scale_;
  // An epsilon to add to the Z-ordering so that alpha blending is performed
  // correctly.
  const float epsilon =
      0.1 / static_cast<float>(debug_msg_.drawings().circles_size());
  for (int i = 0; i < debug_msg_.drawings().circles_size(); ++i) {
    auto circle = debug_msg_.drawings().circles(i);
    const float r0 = max(0.0f, circle.radius() - line_width);
    const float r1 = circle.radius() + line_width;
    DrawArc(ProtoVectorToEigen(circle.center()),
            r0,
            r1,
            -M_PI,
            M_PI,
            kDebugDrawingsZ + static_cast<float>(i) * epsilon,
            ProtoColorToOpenGL(circle.color()));
  }
}

void GLCarView::paintEvent(QPaintEvent* event) {
  graphics_mutex_.lock();
  makeCurrent();
  glClearColor(0.0, 0.5686, 0.0980, 1);
  glShadeModel(GL_SMOOTH);
  glDisable(GL_LIGHTING);
  glDisable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_MULTISAMPLE);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glCallList(field_gl_list_);

  DrawRobots();
  DrawBalls();
  DrawLines();
  DrawPoints();
  DrawArcs();
  DrawCircles();
  DrawEllipses();
  DrawSelection();

  glPopMatrix();
  swapBuffers();
  graphics_mutex_.unlock();
}

void GLCarView::DrawQuad(const Vector2f& loc1, const Vector2f& loc2,
                            float z) {
  glBegin(GL_QUADS);
  glVertex3f(loc1.x(), loc1.y(), z);
  glVertex3f(loc2.x(), loc1.y(), z);
  glVertex3f(loc2.x(), loc2.y(), z);
  glVertex3f(loc1.x(), loc2.y(), z);
  glEnd();
}

void GLCarView::CallRobotDisplayList(Team team) {
  switch (team) {
    case Team::BLUE: {
      glCallList(blue_robot_gl_list_);
    } break;
    case Team::YELLOW: {
      glCallList(yellow_robot_gl_list_);
    } break;
    default: {
      // This should never happen.
      CHECK(false);
    }
  }
}

void GLCarView::CompileRobotDisplayList(Team team) {
  Color4f robot_color(0, 0, 0, 1);
  Color4f outline_color(0, 0, 0, 1);
  switch (team) {
    case Team::BLUE: {
      robot_color = kBlueRobotColor;
      outline_color = kBlueRobotOutlineColor;
    } break;
    case Team::YELLOW: {
      robot_color = kYellowRobotColor;
      outline_color = kYellowRobotOutlineColor;
    } break;
    default: {
      // This should never happen.
      CHECK(false);
    }
  }
  float theta1 = DegToRad(40.0);
  float theta2 = M_2PI - theta1;
  DrawArc(Vector2f(0, 0), 0, 90, theta1, theta2, kRobotZ, robot_color);
  glBegin(GL_TRIANGLES);
  glVertex3f(0, 0, kRobotZ);
  glVertex3f(90.0 * cos(theta1), 90.0 * sin(theta1), kRobotZ);
  glVertex3f(90.0 * cos(theta2), 90.0 * sin(theta2), kRobotZ);
  glEnd();

  DrawArc(Vector2f(0, 0), 80, 90, theta1, theta2, kRobotZ + 0.01,
          outline_color);
  DrawQuad(Vector2f(90.0 * cos(theta1) - 10, 90.0 * sin(theta1)),
           Vector2f(90.0 * cos(theta2), 90.0 * sin(theta2)), kRobotZ + 0.01);
}

void GLCarView::DrawRobot(const Vector2f& loc, float theta, float conf,
                             int robotID, Team team) {
  glPushMatrix();
  glLoadIdentity();
  glTranslated(loc.x(), loc.y(), 0);
  Color4f robot_color, robot_outline_color;
  switch (team) {
    case Team::BLUE: {
      robot_color = kBlueRobotColor;
      robot_outline_color = kBlueRobotOutlineColor;
      break;
    }
    case Team::YELLOW: {
      robot_color = kYellowRobotColor;
      robot_outline_color = kYellowRobotOutlineColor;
      break;
    }
    default: {
      // This should never happen.
      CHECK(false);
    }
  }
  SetColor(Color4f::kBlack);
  const string team_string = StringPrintf("%X", robotID);
  gl_text_.DrawString(loc, 0, 90, team_string, GLText::CenterAligned,
                      GLText::MiddleAligned);

  glRotatef(RadToDeg(theta), 0, 0, 1.0);
  CallRobotDisplayList(team);
  glPopMatrix();
}

void GLCarView::DrawRobotOutline(
    const Vector2f& loc,
    float theta,
    int robot_id,
    Team team) {
  glPushMatrix();
  glLoadIdentity();
  glTranslated(loc.x(), loc.y(), 0);
  Color4f robot_outline_color;
  switch (team) {
    case Team::BLUE: {
      robot_outline_color = kBlueRobotOutlineColor;
      break;
    }
    case Team::YELLOW: {
      robot_outline_color = kYellowRobotOutlineColor;
      break;
    }
    default: {
      // This should never happen.
      CHECK(false);
    }
  }
  robot_outline_color.a = 0.5;
  SetColor(robot_outline_color);
  glRotatef(RadToDeg(theta), 0, 0, 1.0);

  float theta1 = DegToRad(40.0);
  float theta2 = M_2PI - theta1;
  const string team_string = StringPrintf("%X", robot_id);
  gl_text_.DrawString(loc, 0, 90, team_string, GLText::CenterAligned,
                      GLText::MiddleAligned);
  DrawArc(Vector2f(0, 0), 80, 90, theta1, theta2, kRobotZ - 0.01,
          robot_outline_color);
  DrawQuad(Vector2f(90.0 * cos(theta1) - 10, 90.0 * sin(theta1)),
           Vector2f(90.0 * cos(theta2), 90.0 * sin(theta2)), kRobotZ - 0.01);
  glPopMatrix();
}

void GLCarView::DrawFieldLines() {
}

void GLCarView::DrawBall(const Vector2f& loc) {
  DrawArc(loc, 0, 16, -M_PI, M_PI, kBallZ, kBallColor);
  DrawArc(loc, 15, 21, -M_PI, M_PI, kBallZ, kFieldLinesColor);
}

void GLCarView::Update(const MinuteBotsProto::SoccerDebugMessage& msg) {
  graphics_mutex_.lock();
  debug_msg_ = msg;
  graphics_mutex_.unlock();
  RedrawSignal();
}

}  // namespace gui
