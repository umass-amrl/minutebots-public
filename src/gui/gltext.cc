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

#include <float.h>
#include <algorithm>
#include <cmath>

#include "QtGui"

#include "gui/gltext.h"
#include "shared/common_includes.h"

using std::max;
using std::min;

namespace gui {
const bool GLText::kDebugTesselation = false;
const float GLText::kFontRenderSize = 1000.0;

void GLText::Initialize() {
  font.setPixelSize(kFontRenderSize);
  character_spacing = 0.1;
  font_metrics = QFontMetricsF(font);
  font_height = font_metrics.height() / kFontRenderSize;
  font_ascent = font_metrics.ascent() / kFontRenderSize;
  font_descent = font_metrics.descent() / kFontRenderSize;
}

GLText::GLText() : font(), font_metrics(font) {
  Initialize();
}

GLText::GLText(QFont font) :
    font(font),
    font_metrics(font),
    render_depth(0) {
  Initialize();
}

GLText::~GLText() {}

void GLText::DrawGlyph(const QChar& glyph) {
  InitializeGlyph(glyph);
  glCallList(glyphs[glyph.unicode()].display_list_id);
}

float GLText::GetWidth(const QChar& ch) {
  InitializeGlyph(ch);
  return glyphs[ch.unicode()].width;
}

float GLText::GetHeight(const QChar& ch) {
  InitializeGlyph(ch);
  return glyphs[ch.unicode()].height;
}

float GLText::GetAscent(const QChar& ch) {
  InitializeGlyph(ch);
  return glyphs[ch.unicode()].ascent;
}

float GLText::GetDescent(const QChar& ch) {
  InitializeGlyph(ch);
  return glyphs[ch.unicode()].descent;
}

float GLText::GetWidth(const QString& str) {
  return (font_metrics.width(str) / kFontRenderSize);
}

float GLText::GetHeight() {
  return font_height;
}

float GLText::GetAscent() {
  return font_ascent;
}

float GLText::GetDescent() {
  return font_descent;
}

void GLText::DrawString(const Vector2f& loc,
                        float angle,
                        float size,
                        const string& std_str,
                        gui::GLText::HAlignOptions hAlign,
                        gui::GLText::VAlignOptions vAlign) {
  const QString str(std_str.c_str());
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glTranslatef(loc.x(), loc.y(), render_depth);
  glScalef(size / GetHeight(), size / GetHeight(), 1.0);
  glRotatef(angle, 0, 0, 1);

  switch (hAlign) {
    case LeftAligned: {
      // Normal rendering will achieve this.
      break;
    }
    case RightAligned: {
      glTranslatef(-GetWidth(str), 0, 0);
      break;
    }
    case CenterAligned: {
      glTranslatef(-0.5 * GetWidth(str), 0, 0);
      break;
    }
  }
  switch (vAlign) {
    case BottomAligned: {
      glTranslatef(0.0, GetDescent(), 0.0);
      break;
    }
    case TopAligned: {
      glTranslatef(0.0, -GetAscent(), 0.0);
      break;
    }
    case MedianAligned: {
      // Normal rendering will achieve this!
      break;
    }
    case MiddleAligned: {
      glTranslatef(0.0, -0.5 * GetHeight(), 0.0);
      break;
    }
  }
  static const QChar kLineFeed(static_cast<char>('\n'));
  static const QChar kCarriageReturn(static_cast<char>('\r'));
  static const QChar kTabStop(static_cast<char>('\t'));
  static const QChar kSpace(static_cast<char>(' '));
  const float tab_width = 2.0 * GetWidth(kSpace);
  float line_length = 0.0;
  for (int i = 0; i < str.length(); ++i) {
    const QChar& letter = str[i];
    if (letter == kLineFeed || letter == kCarriageReturn) {
      glTranslatef(-line_length, -GetHeight(), 0.0);
      line_length = 0.0;
      continue;
    } else if (letter == kTabStop) {
      glTranslatef(tab_width, 0.0, 0.0);
      line_length += tab_width;
      continue;
    }
    DrawGlyph(letter);
    const float d = GetWidth(letter);
    line_length += d;
    glTranslatef(d, 0.0, 0.0);
  }
  glPopMatrix();
}

void GLText::InitializeGlyph(const QChar& ch) {
  const ushort& unicode_value = ch.unicode();
  if (glyphs.size() < static_cast<size_t>(unicode_value + 1)) {
    glyphs.resize(unicode_value + 1);
  }
  if (glyphs[unicode_value].compiled) {
    return;
  }
  Glyph glyph;
  const QString q_string(ch);

  QPainterPath path;
  path.addText(0, 0, font, q_string);
  QList<QPolygonF> polygons = path.toSubpathPolygons();
  glyph.ascent = font_metrics.ascent() / kFontRenderSize;
  glyph.descent = font_metrics.descent() / kFontRenderSize;

  glyph.height = font_metrics.height() / kFontRenderSize;
  glyph.width = font_metrics.width(q_string) / kFontRenderSize;

  int numVertices = 0;
  for (int i = 0; i < polygons.size(); ++i) {
    numVertices += polygons[i].size();
  }
  GLdouble vertices[numVertices][3];
  int j = 0;
  for (int i = 0; i < polygons.size(); ++i) {
    for (int k = 0; k < polygons[i].size(); ++k) {
      vertices[j][0] = polygons[i][k].x()/kFontRenderSize;
      vertices[j][1] = -polygons[i][k].y()/kFontRenderSize;
      vertices[j][2] = 0;
      j++;
    }
  }

  if (kDebugTesselation) {
    printf("Glyph for QChar 0x%04hX, '%c' Width %f, %d polygons, %d vertices\n",
           ch.unicode(),
           ch.toAscii(),
           glyph.width,
           polygons.size(),
           numVertices);
  }
  GLUtesselator* tess = gluNewTess();
  gluTessCallback(tess, GLU_TESS_BEGIN, (_GLUfuncptr) tessBeginCB);
  gluTessCallback(tess, GLU_TESS_END, (_GLUfuncptr) tessEndCB);
  gluTessCallback(tess, GLU_TESS_ERROR, (_GLUfuncptr) tessErrorCB);
  gluTessCallback(tess, GLU_TESS_VERTEX, (_GLUfuncptr) tessVertexCB);

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();

  if (kDebugTesselation) {
    printf("\nBegin tesselation of character %c\n", ch.toAscii());
  }
  glyph.display_list_id = glGenLists(1);
  if (glyph.display_list_id == GL_INVALID_VALUE) {
    printf("Unable to create display list!\n");
    exit(1);
  }
  glNewList(glyph.display_list_id, GL_COMPILE);
  gluTessBeginPolygon(tess, 0);
  j = 0;
  for (int i = 0; i < polygons.size(); ++i) {
    gluTessBeginContour(tess);
    for (int k = 0; k < polygons[i].size(); ++k) {
      gluTessVertex(tess, vertices[j], vertices[j]);
      ++j;
    }
    gluTessEndContour(tess);
  }
  gluTessEndPolygon(tess);
  gluDeleteTess(tess);
  glEndList();
  glPopMatrix();
  glyph.compiled = true;
  glyphs[ch.unicode()] = glyph;
}

const char* GLText::GetPrimitiveType(GLenum type) {
  switch (type) {
    case 0x0000:
      return "GL_POINTS";
      break;
    case 0x0001:
      return "GL_LINES";
      break;
    case 0x0002:
      return "GL_LINE_LOOP";
      break;
    case 0x0003:
      return "GL_LINE_STRIP";
      break;
    case 0x0004:
      return "GL_TRIANGLES";
      break;
    case 0x0005:
      return "GL_TRIANGLE_STRIP";
      break;
    case 0x0006:
      return "GL_TRIANGLE_FAN";
      break;
    case 0x0007:
      return "GL_QUADS";
      break;
    case 0x0008:
      return "GL_QUAD_STRIP";
      break;
    case 0x0009:
      return "GL_POLYGON";
      break;
  }
  return "UNKNOWN";
}

void GLText::tessBeginCB(GLenum which) {
  glBegin(which);
  if (kDebugTesselation) {
    printf("glBegin(%s);\n", GetPrimitiveType(which));
  }
}

void GLText::tessEndCB() {
  glEnd();
  if (kDebugTesselation) printf("glEnd();\n");
}

void GLText::tessVertexCB(const GLvoid *data) {
  const GLdouble *ptr = reinterpret_cast<const GLdouble*>(data);
  if (kDebugTesselation) {
    printf("glVertex3d(%f,%f,%f);\n",
           ptr[0],
           ptr[1],
           ptr[2]);
  }
  glVertex3dv(ptr);
}

void GLText::tessErrorCB(GLenum errorCode) {
  const GLubyte *errorStr;
  errorStr = gluErrorString(errorCode);
  printf("[ERROR]: %s\n", errorStr);
}

void GLText::SetRenderDepth(float depth) {
  render_depth = depth;
}

}  // namespace gui
