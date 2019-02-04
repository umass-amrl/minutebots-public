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

#include <string>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "GL/glu.h"
#include "QChar"
#include "QFont"
#include "QFontMetricsF"

#ifndef SRC_GUI_GLTEXT_H_
#define SRC_GUI_GLTEXT_H_

namespace gui {

class GLText{
  struct Glyph{
    bool compiled;
    GLuint display_list_id;
    float width;
    float height;
    float ascent;
    float descent;
    Glyph() : compiled(false) {}
  };

  std::vector<Glyph> glyphs;

  float character_spacing;
  QFont font;

 public:
  typedef enum {
    LeftAligned,
    RightAligned,
    CenterAligned
  } HAlignOptions;

  typedef enum {
    TopAligned,
    BottomAligned,
    MedianAligned,
    MiddleAligned
  } VAlignOptions;

  QFontMetricsF font_metrics;

  GLText();
  explicit GLText(QFont font);
  ~GLText();
  void DrawString(const Eigen::Vector2f& loc,
                  float angle,
                  float size,
                  const std::string& str,
                  GLText::HAlignOptions hAlign = LeftAligned,
                  GLText::VAlignOptions vAlign = MiddleAligned);
  void DrawGlyph(const QChar& glyph);
  void InitializeGlyph(const QChar& ch);

  float GetWidth(const QChar& ch);
  float GetHeight(const QChar& ch);
  float GetAscent(const QChar& ch);
  float GetDescent(const QChar& ch);

  float GetWidth(const QString&  str);
  float GetHeight();
  float GetAscent();
  float GetDescent();

  void SetRenderDepth(float depth);

 private:
  void Initialize();

  static const char* GetPrimitiveType(GLenum type);
  static void tessBeginCB(GLenum which);
  static void tessEndCB();
  static void tessVertexCB(const GLvoid *data);
  static void tessErrorCB(GLenum errorCode);

  float font_height;
  float font_ascent;
  float font_descent;
  static const float kFontRenderSize;
  static const bool kDebugTesselation;
  float render_depth;
};

}  // namespace gui

#endif  // SRC_GUI_GLTEXT_H_
