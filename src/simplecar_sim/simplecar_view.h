// Copyright 2017 - 2019 joydeepb@cs.umass.edu
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

#include "QGLWidget"
#include "QMutex"

#include "gui/gltext.h"

#include "messages_robocup_ssl_wrapper.pb.h"
#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include "shared/common_includes.h"
#include "state/team.h"
#include "soccer_logging.pb.h"

#ifndef SRC_SIMPLECAR_SIM_SIMPLECAR_VIEW_H_
#define SRC_SIMPLECAR_SIM_SIMPLECAR_VIEW_H_

namespace gui {
enum class GuiSelectionType {
  kNoneSelected = 0,
  kBallSelected = 1,
  kRobotSelected = 2,
};

class GLCarView : public QGLWidget {
  friend class Viewer;
  Q_OBJECT
  static const int kPreferedWidth;
  static const int kPreferedHeight;

 public:
  // Mutex to permit thread-safe read and write of the debug message.
  QMutex graphics_mutex_;

  // OpenGL vector text renderer.
  gui::GLText gl_text_;

  // OpenGL display list for blue robot.
  GLuint blue_robot_gl_list_;

  // OpenGL display list for yellow robot.
  GLuint yellow_robot_gl_list_;

  // OpenGL display list for field markings.
  GLuint field_gl_list_;

  // Ratio of screen space to world space coordinates.
  float view_scale_;

  // X-coordinate in world space of the point at the center of the window.
  float x_offset_;
  // Y-coordinate in world space of the point at the center of the window.
  float y_offset_;

  // Window-space x coordinate where the mouse button down event occured.
  int mouse_start_x_;
  // Window-space y coordinate where the mouse button down event occured.
  int mouse_start_y_;

  // Last time that a display refresh was performed, used to limit display rate.
  double t_last_redraw_;

  // Indicates what is selected.
  GuiSelectionType selection_;

  // The ID of the selected robot.
  int selected_robot_id_;

  // The team of the selected robot.
  team::Team selected_robot_team_;

  // Selected ball index.
  int selected_ball_index_;

  // Indicates that a click-drag, or click-zoom event is under way.
  bool view_interaction_;

  // Debug drawing message.
  MinuteBotsProto::SoccerDebugMessage debug_msg_;

  // Redraw event handler.
  void paintEvent(QPaintEvent* event);
  // Mouse wheel event handler.
  void wheelEvent(QWheelEvent* event);
  // Mouse move event handler.
  void mouseMoveEvent(QMouseEvent* event);
  // Mouse button down event handler.
  void mousePressEvent(QMouseEvent* event);
  // Mouse button up event handler.
  void mouseReleaseEvent(QMouseEvent* event);
  // Keyboard keypress event handler.
  void keyPressEvent(QKeyEvent* event);
  // Window resizing event handler.
  void resizeEvent(QResizeEvent* event);
  // OpenGL system initialization event handler.
  void initializeGL();
  // Resize OpenGL viewport event handler.
  void resizeGL(int width, int height);
  // Return preferred widget size.
  QSize sizeHint() const;
  // Reset the display zoom and pan to show the entire field to fit the window.
  void ResetView();
  // Update which robot or ball is selected, using the specified selection
  // location.
  void UpdateSelection(const Eigen::Vector2f& loc);
  // Convert a screen coordinate to world coordinates.
  Eigen::Vector2f ScreenToWorld(const Eigen::Vector2f& p_screen) const;
  // Convert a world coordinate to screen coordinates.
  Eigen::Vector2f WorldToScreen(const Eigen::Vector2f& p_world) const;

 private:
  // Draw field markings.
  void DrawFieldLines();
  // Draw all known robots.
  void DrawRobots();
  // Draw all known balls.
  void DrawBalls();
  // Draw debugging lines.
  void DrawLines();
  // Draw debugging points.
  void DrawPoints();
  // Draw debugging arcs.
  void DrawArcs();
  // Draw debugging circles.
  void DrawCircles();
  // Draw debugging ellipses.
  void DrawEllipses();
  // Draw selection circle.
  void DrawSelection();
  // Draw an axis-aligned rectangle with the loc1 as the bottom left corner,
  // and loc2 as the top right corner, and on the specified z plane.
  void DrawQuad(const Eigen::Vector2f& loc1,
                const Eigen::Vector2f& loc2,
                float z);
  // Recompute OpenGL projection based on new window size.
  void RecomputeProjection();
  // Draw a robot with the specified pose.
  void DrawRobot(const Eigen::Vector2f& loc,
                 float theta,
                 float conf,
                 int robotID,
                 team::Team team);
  // Draw a robot outline at the specified location.
  void DrawRobotOutline(const Eigen::Vector2f& loc,
                        float theta,
                        int robot_id,
                        team::Team team);
  // Call a robot display list.
  void CallRobotDisplayList(team::Team team);
  // Compile a robot display list.
  void CompileRobotDisplayList(team::Team team);
  // Draw a ball at the specified location.
  void DrawBall(const Eigen::Vector2f& loc);
  // Save an image of the current view.
  void SaveImage();

 public:
  explicit GLCarView(QWidget *parent = nullptr);
  // Update drawing data.
  void Update(const MinuteBotsProto::SoccerDebugMessage& msg);

 private slots:  // NOLINT(whitespace/indent)
  // Handle a redraw signal in a thread-safe manner.
  void Redraw();

 signals:
  // Raise a redraw signal in a thread-safe manner.
  void RedrawSignal();
  // Used for GUI interaction mouse-down callback.
  void MouseDownCallback(Eigen::Vector2f loc,
                         Qt::KeyboardModifiers modifiers,
                         Qt::MouseButtons button,
                         gui::GuiSelectionType selection,
                         uint32_t selected_robot_id,
                         team::Team selected_robot_team);
  // Used for GUI interaction mouse-up callback.
  void MouseUpCallback(Eigen::Vector2f loc,
                       Qt::KeyboardModifiers modifiers,
                       Qt::MouseButtons button,
                       gui::GuiSelectionType selection,
                       uint32_t selected_robot_id,
                       team::Team selected_robot_team);
  // Used for GUI interaction mouse-move callback.
  void MouseMoveCallback(Eigen::Vector2f loc,
                         Qt::KeyboardModifiers modifiers,
                         Qt::MouseButtons button,
                         gui::GuiSelectionType selection,
                         uint32_t selected_robot_id,
                         team::Team selected_robot_team);
};
}  // namespace gui
#endif  // SRC_SIMPLECAR_SIM_SIMPLECAR_VIEW_H_
