// Copyright 2011-2018 joydeepb@cs.umass.edu, dbalaban@cs.umass.edu
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
#include <stdio.h>

#include <CImg.h>
#include <algorithm>
#include <cfloat>  // DBL_MAX, DBL_MIN
#include <chrono>
#include <ctime>  // to produce unique filenames for saving images
#include <random>
#include <string>
#include <vector>
// X.h (included from CImg) defines a bunch of things that conflict with a bunch
// of other things
#ifdef CursorShape
#undef CursorShape
#endif
#ifdef Status
#undef Status
#endif
#ifdef Bool
#undef Bool
#endif
#ifdef None
#undef None
#endif
#ifdef Success
#undef Success
#endif
#ifdef KeyPress
#undef KeyPress
#endif
#ifdef KeyRelease
#undef KeyRelease
#endif
#ifdef FocusIn
#undef FocusIn
#endif
#ifdef FocusOut
#undef FocusOut
#endif
#ifdef FontChange
#undef FontChange
#endif
#ifdef Unsorted
#undef Unsorted
#endif
#ifdef GrayScale
#undef GrayScale
#endif

#include "QApplication"
#include "QtGui"

#include "constants/constants.h"
#include "gui/drawing_helpers.h"
#include "gui/opengl_helpers.h"
#include "gui/viewer.h"
#include "gui/viewer_motion.h"
#include "logging/logger.h"
#include "motion_control/ball_interception.h"
#include "motion_control/ball_interception2.h"
#include "motion_control/ntoc_2d.h"
#include "motion_control/optimal_control_1d.h"
#include "motion_control/tsocs_old.h"
#include "motion_control/tsocs_alt.h"
#include "net/netraw.h"
#include "shared/common_includes.h"
#include "soccer_logging.pb.h"
#include "src/gui/view_function.h"
#include "src/motion_control/tsocs.h"

using drawing_helpers::DrawLine;
using drawing_helpers::DrawPoint;
using Eigen::Vector2d;
using gui::Viewer;
using opengl_helpers::Color4f;
using MinuteBotsProto::SoccerDebugMessage;
using MinuteBotsProto::TextTree;
using ntoc::ControlPhase2D;
using ntoc::ControlSequence2D;
using motion::MotionModel;
using ntoc::GetAverageAccel;
using ntoc::NTOC2D;
using ntoc::NTOCFinished;
using tsocs::Tsocs;
using tsocs::BallInterception;
using tsocs::kUsePolarParams;
using tsocs::CostFunctionPolar;
using tsocs::CostFunctionDoublePolar;
using tsocs::kUseDoublePolarParams;
using tsocs::kTSOCSThreshold;
using tsocs::kUsePerturbationsFix;
using tsocs::TSOCSProblem;
using tsocs::kMaxPerturbations;
using tsocs::GetState;
using tsocs::GetGuess;
using tsocs::GetSolution;
using std::vector;
using std::default_random_engine;
using std::normal_distribution;
using cimg_library::CImg;
using cimg_library::CImgDisplay;
using cimg_library::CImgList;
using view_function::FunctionViewer;
using std::chrono::system_clock;
using std::chrono::duration;

Viewer* view;

unsigned int rand_seed_ = 1;
// true if the last problem visualized was ball interception, false if regular
bool ball_problem = false;
bool new_interception = true;

void VisualizeNTOC(SoccerDebugMessage* message);
bool VisualizeTSOCS(SoccerDebugMessage* message);
bool VisualizeTSOCS_alt(SoccerDebugMessage* message);
bool VisualizeInterception(SoccerDebugMessage* message);
bool VisualizeIterativeTSOCS(SoccerDebugMessage* message, ScopedFile* fid);
bool VisualizeIterativeNTOC(SoccerDebugMessage* message, ScopedFile* fid);
void DefineProblemConditions(SoccerDebugMessage* message);
void DefineProblemConditionsFinalV(SoccerDebugMessage* message);
void DefineInterceptionConditions(SoccerDebugMessage* message);
void DrawBoundaryConditions(MinuteBotsProto::DebugDrawings* drawings);
void DrawNewProblem(SoccerDebugMessage* message);
void DrawNewProblemFinalV(SoccerDebugMessage* message);
void DrawInterception(SoccerDebugMessage* message);
void DrawIterativeTSOCS(SoccerDebugMessage* message);
void DrawTSOCSPath(const SolutionParameters& params, const Color4f& col,
                   MinuteBotsProto::DebugDrawings* drawings);
void DrawTSOCSPath(const SolutionParameters& params, const Color4f& col,
                   MinuteBotsProto::DebugDrawings* drawings, string filename);
void DrawInterceptionPath(SolutionParameters intercept_params, Color4f color,
                          MinuteBotsProto::DebugDrawings* drawings);
void RandomTest(SoccerDebugMessage* message, Viewer* viewer);
void IterativeTest(SoccerDebugMessage* message, Viewer* viewer);

MinuteBotsProto::SoccerDebugMessage* message_global = NULL;
gui::Viewer* viewer_global = NULL;
const int time_steps = 200;
static const int kMinSamples = 1000;
int* perturbations_histogram = NULL;
int num_panels = 7;
int panel_size = 101;
int panel_spacing = 30;
int left_spacing = 150;
static bool center_stage1;
// in a panel, vary the parameters so they range from param_lower_range * (the
// param) to param_upper_range * (the param)
double param_lower_range = 0.1, param_upper_range = 1.9;
double time_lower_range = 0.01, time_upper_range = 1.99;
static const bool kNewTSOCS = true;

static const Color4f kSuccessColor(0, 0, 1, 1);
static const Color4f kFailureColor(1, 0, 1, 1);
static const Color4f kLineColor(1, 1, 0, 1);
static const Color4f kGuessColor(1, 0, 0, 1);
static const Color4f kStage1Color(1, 0.647, 0, 1);

Eigen::Vector2d x0;
Eigen::Vector2d v0;
Eigen::Vector2d xf;
Eigen::Vector2d vf;

Eigen::Vector2d ball_pos;
Eigen::Vector2d ball_vel;
// const double ball_accel = 30;
const double ball_accel = .01 * kDefaultRobotAcceleration;
bool use_alt_tsocs = false;
SolutionParameters params, params_stage1, ball_params;
SolutionParameters new_params_guess;
static tsocs::Tsocs tsocs_instance(Vector2d(0, 0),
                                   Vector2d(0, 0),
                                   Vector2d(0, 0),
                                   Vector2d(0, 0),
                                   kDefaultRobotAcceleration);
static BallInterception intercept;

default_random_engine generator;

TextTree NewTextTree(const string& str) {
  TextTree tree;
  tree.set_text(str);
  return tree;
}

void CostLeftClick(double x1, double x2, double x3, double x4) {
  SolutionParameters cur_params = center_stage1 ? params_stage1 : params;
  if (kUsePolarParams) {
    double a = x1, b = x2, theta = x3, T = x4;
    printf("a=%f,b=%f,theta=%f,T=%f\n", a, b, theta, T);
    printf("log10(cost) = %f\n", log10(CostFunctionPolar(a, b, theta, T)));
  } else if (kUseDoublePolarParams) {
    double theta1 = x1, theta2 = x2, theta3 = x3, T = x4;
    printf("theta1=%f,theta2=%f,theta3=%f,T=%f\n", theta1, theta2, theta3, T);
    printf("log10(cost) = %f\n",
           log10(CostFunctionDoublePolar(theta1, theta2, theta3, T)));
  } else {
    double a = x1, b = x2, c = cur_params.c, d = x3, T = x4;
    printf("a=%f,b=%f,c=%f,d=%f,T=%f\n", a, b, c, d, T);
    printf("log10(cost) = %f\n", log10(tsocs_instance.CostFunction(a,
                                                                   b,
                                                                   c,
                                                                   d,
                                                                   T)));
  }
}

void CostRightClick(double x1, double x2, double x3, double x4) {
  MinuteBotsProto::DebugDrawings& drawings =
      *message_global->mutable_drawings();
  if (ball_problem) {
    double a = x1, b = x2, c = ball_params.c, d = x3, T = x4;
    new_params_guess = SolutionParameters(a, b, c, d, T, 0);
    printf("a=%f,b=%f,c=%f,d=%f,T=%f\n", a, b, c, d, T);
    DrawInterceptionPath(new_params_guess, kGuessColor, &drawings);
  } else {
    SolutionParameters cur_params = center_stage1 ? params_stage1 : params;
    if (kUsePolarParams) {
      // TODO(afischer)
    } else if (kUseDoublePolarParams) {
      new_params_guess = SolutionParameters(x1, x2, x3, 0, x4, 0);
      printf("theta1=%f\ntheta2=%f\ntheta3=%f\nT=%f\n", x1, x2, x3, x4);
    } else {
      double a = x1, b = x2, c = cur_params.c, d = x3, T = x4;
      new_params_guess = SolutionParameters(a, b, c, d, T, 0);
      printf("a=%f,b=%f,c=%f,d=%f,T=%f\n", a, b, c, d, T);
    }
    DrawTSOCSPath(new_params_guess, Color4f(1, 1, 1, 1), &drawings);
  }
  viewer_global->Update(*message_global);
}

void CostKey(CImgDisplay display) {
  SolutionParameters cur_params = center_stage1 ? params_stage1 : params;
  MinuteBotsProto::DebugDrawings& drawings =
      *message_global->mutable_drawings();
  drawings.clear_lines();
  drawings.clear_arcs();
  drawings.clear_points();
  bool foundSolution;
  SolutionParameters new_params = new_params_guess;
  if (ball_problem) {
    if (new_interception) {
      foundSolution = intercept.GetInterceptSolution(&new_params);
    } else {
      foundSolution =
          GetInterceptSolution(x0, v0, ball_pos, ball_vel, ball_accel,
                               kDefaultRobotAcceleration, &new_params);
    }
    DrawInterceptionPath(
        new_params, foundSolution ? kSuccessColor : kFailureColor, &drawings);
    printf("ball interception\n");
  } else {
    DrawBoundaryConditions(&drawings);
    // draw original solution path
    Color4f solution_color =
        cur_params.cost < kTSOCSThreshold ? kSuccessColor : kFailureColor;
    DrawTSOCSPath(cur_params, solution_color, &drawings);

    // run ceres with new parameters
    printf("a=%f,b=%f,c=%f,d=%f,T=%f\n", new_params.a, new_params.b,
           new_params.c, new_params.d, new_params.T);
    foundSolution = tsocs_instance.GetSolutionNoStages(&new_params);
    solution_color = foundSolution ? kSuccessColor : kFailureColor;
    DrawTSOCSPath(new_params, solution_color, &drawings);
  }
  if (kUsePolarParams) {
    // TODO(afischer)
  } else if (kUseDoublePolarParams) {
    if (foundSolution) {
      printf("New guess succeeded: theta1=%f, theta2=%f, theta3=%fT=%f\n",
             new_params.a, new_params.b, new_params.c, new_params.T);
    } else {
      printf("New guess failed: theta1=%f, theta2=%f, theta3=%fT=%f\n",
             new_params.a, new_params.b, new_params.c, new_params.T);
    }
  } else {
    if (foundSolution) {
      printf("New guess succeeded: a=%f, b=%f, c=%f, d=%f, T=%f\n",
             new_params.a, new_params.b, new_params.c, new_params.d,
             new_params.T);
    } else {
      printf("New guess failed: a=%f, b=%f, c=%f, d=%f, T=%f\n", new_params.a,
             new_params.b, new_params.c, new_params.d, new_params.T);
    }
  }
  printf("log_10 cost=%f\n", log10(new_params.cost));
  viewer_global->Update(*message_global);
}

void* VisualizeCostThread(void* thread_id) {
  FunctionViewer cost_viewer;
  if (ball_problem) {
    auto anon_cost = [](double a, double b, double d, double T) {
      return log10(
          max(BallCostFunction(a, b, ball_params.c, d, T), kTSOCSThreshold));
    };
    cost_viewer.Display(
        anon_cost, "cost visualization: press s to save", "a",
        param_lower_range * ball_params.a, param_upper_range * ball_params.a,
        "b", param_lower_range * ball_params.b,
        param_upper_range * ball_params.b, "d",
        param_lower_range * ball_params.d, param_upper_range * ball_params.d,
        "T", time_lower_range * ball_params.T, time_upper_range * ball_params.T,
        CostLeftClick, CostRightClick, CostKey);
  } else {
    SolutionParameters cur_params = center_stage1 ? params_stage1 : params;
    if (kUsePolarParams) {
      cost_viewer.Display(
          CostFunctionPolar, "cost visualization: press s to save", "a",
          param_lower_range * cur_params.a, param_upper_range * cur_params.a,
          "b", param_lower_range * cur_params.b,
          param_upper_range * cur_params.b, "theta", -M_PI, M_PI, "T",
          time_lower_range * params.T, time_upper_range * params.T,
          CostLeftClick, CostRightClick, CostKey);
    } else if (kUseDoublePolarParams) {
      auto anon_cost = [](double theta1, double theta2, double theta3,
                          double T) {
        return min(10.0, CostFunctionDoublePolar(theta1, theta2, theta3, T));
      };
      double theta1 = cur_params.a, theta2 = cur_params.b,
             theta3 = cur_params.c;
      printf("centering at params:\n%f\n%f\n%f\n%f\n", theta1, theta2, theta3,
             cur_params.T);
      cost_viewer.Display(
          anon_cost, "cost visualization: press s to save", "theta1",
          theta1 - M_PI, theta1 + M_PI, "theta2", theta2 - M_PI / 2,
          theta2 + M_PI / 2, "theta3", theta3 - M_PI / 2, theta3 + M_PI / 2,
          "T", time_lower_range * cur_params.T, time_upper_range * cur_params.T,
          CostLeftClick, CostRightClick, CostKey);
    } else {
      auto anon_cost = [](double a, double b, double d, double T) {
        return log10(
            max(tsocs_instance.CostFunction(a, b, params.c, d, T),
                kTSOCSThreshold));
      };
      cost_viewer.Display(
          anon_cost, "cost visualization: press s to save", "a",
          param_lower_range * cur_params.a, param_upper_range * cur_params.a,
          "b", param_lower_range * cur_params.b,
          param_upper_range * cur_params.b, "d",
          param_lower_range * cur_params.d, param_upper_range * cur_params.d,
          "T", time_lower_range * params.T, time_upper_range * params.T,
          CostLeftClick, CostRightClick, CostKey);
    }
  }
  return NULL;
}

void DrawNewProblem(SoccerDebugMessage* message) {
  DefineProblemConditions(message);
  VisualizeNTOC(message);
  VisualizeTSOCS(message);
}

void DrawNewProblemFinalV(SoccerDebugMessage* message) {
  DefineProblemConditionsFinalV(message);
  VisualizeTSOCS(message);
}

void DrawInterception(SoccerDebugMessage* message) {
  DefineInterceptionConditions(message);
  VisualizeInterception(message);
}

void DrawIterativeTSOCS(SoccerDebugMessage* message) {
  DefineProblemConditionsFinalV(message);
  ScopedFile fid("/dev/null", "a");
  VisualizeIterativeTSOCS(message, &fid);
}

void DrawIterativeNTOC(SoccerDebugMessage* message) {
  DefineProblemConditions(message);
  ScopedFile fid("/dev/null", "a");
  VisualizeIterativeNTOC(message, &fid);
}

void RandomTest(SoccerDebugMessage* message, Viewer* viewer) {
  ScopedFile fid("TSOCS-failures.txt", "a");
  int n = 0;
  int failures = 0;
  bool succesful = true;
  fprintf(fid, "x0.x, x0.y, v0.x, v0.y, xf.x, xf.y, vf.x, vf.y\n");
  do {
    ++n;
    printf("\rTested %d cases, %d failures", n, failures);
    fflush(stdout);
    DefineProblemConditions(message);
    VisualizeNTOC(message);
    if (use_alt_tsocs) {
      succesful = VisualizeTSOCS_alt(message);
    } else {
      succesful = VisualizeTSOCS(message);
    }
    viewer->Update(*message);
    viewer->update();
    if (!succesful) {
      ++failures;
      fprintf(fid, "%12f, %12f, %12f, %12f, %12f, %12f, %12f, %12f\n", x0.x(),
              x0.y(), v0.x(), v0.y(), xf.x(), xf.y(), vf.x(), vf.y());
    }
  } while (n < kMinSamples || succesful);
  printf("\nFailures: %d/%d (%.3f)\n", failures, n,
         100.0 * static_cast<float>(failures) / static_cast<float>(n));
}

void RandomTestFinalV(SoccerDebugMessage* message, Viewer* viewer) {
  auto start = system_clock::now();
  ScopedFile fid("TSOCS-failures-finalV.txt", "a");
  int n = 0;
  int failures = 0;
  bool successful = true;
  if (kUsePerturbationsFix) {
    perturbations_histogram = new int[kMaxPerturbations + 1];
    for (int i = 0; i < kMaxPerturbations + 1; i++) {
      perturbations_histogram[i] = 0;
    }
  }
  do {
    ++n;
    printf("\rTested %d cases, %d failures", n, failures);
    fflush(stdout);
    DefineProblemConditionsFinalV(message);
    if (use_alt_tsocs) {
      successful = VisualizeTSOCS_alt(message);
    } else {
      successful = VisualizeTSOCS(message);
    }
    viewer->Update(*message);
    viewer->update();
    if (!successful) {
      ++failures;
      fprintf(fid, "%12f, %12f, %12f, %12f, %12f, %12f, %12f, %12f\n", x0.x(),
              x0.y(), v0.x(), v0.y(), xf.x(), xf.y(), vf.x(), vf.y());
    }
  } while (n < kMinSamples || successful);
  printf("\nFailures: %d/%d (%.3f)\n", failures, n,
         100.0 * static_cast<float>(failures) / static_cast<float>(n));
  auto end = system_clock::now();
  duration<double> elapsed_time = end - start;
  cout << "Took " << elapsed_time.count() << " seconds" << endl;
  if (kUsePerturbationsFix) {
    printf("Perturbations histogram:\n");
    printf("immediate success: %d\n", perturbations_histogram[0]);
    for (int i = 1; i <= kMaxPerturbations; i++) {
      if (perturbations_histogram[i] > 0) {
        printf("%d: %d\n", i, perturbations_histogram[i]);
      }
    }
    printf("did not succeed: %d\n", failures);
    printf("total: %d\n", n);
    delete[] perturbations_histogram;
  }
}

void TestBallInterception(SoccerDebugMessage* message, Viewer* viewer) {
  ScopedFile fid("TSOCS-failures-interception.txt", "a");
  int n = 0;
  int failures = 0;
  bool successful = true;
  do {
    ++n;
    printf("\rTested %d cases, %d failures", n, failures);
    fflush(stdout);
    DefineInterceptionConditions(message);
    successful = VisualizeInterception(message);
    viewer->Update(*message);
    viewer->update();
    if (!successful) {
      ++failures;
      fprintf(fid, "%12f, %12f, %12f, %12f, %12f, %12f, %12f, %12f\n", x0.x(),
              x0.y(), v0.x(), v0.y(), xf.x(), xf.y(), vf.x(), vf.y());
    }
  } while (n < kMinSamples || successful);
  printf("\nFailures: %d/%d (%.3f)\n", failures, n,
         100.0 * static_cast<float>(failures) / static_cast<float>(n));
}

void IterativeTest(SoccerDebugMessage* message, Viewer* viewer) {
  string table_file = "Time-Data-TSOCS-FinalV-0noise.csv";
  ScopedFile csvid(table_file, "a");
  ScopedFile fid("TSOCS-iterative-failures-finalV.txt", "a");

  fprintf(csvid, "optimal, elapsed, remaining\n");

  int n = 0;
  int failures = 0;
  bool succesful = true;
  do {
    ++n;
    printf("\rTested %d cases, %d failures", n, failures);
    fflush(stdout);
    DefineProblemConditionsFinalV(message);
    // DefineProblemConditions(message);
    succesful = VisualizeIterativeTSOCS(message, &csvid);
    // succesful = VisualizeIterativeNTOC(message, &csvid);
    viewer->Update(*message);
    viewer->update();
    if (!succesful) {
      ++failures;
      fprintf(fid, "%12f, %12f, %12f, %12f, %12f, %12f, %12f, %12f\n", x0.x(),
              x0.y(), v0.x(), v0.y(), xf.x(), xf.y(), vf.x(), vf.y());
    }
  } while (n < kMinSamples || succesful);
  printf("\nFailures: %d/%d (%.3f)\n", failures, n,
         100.0 * static_cast<float>(failures) / static_cast<float>(n));
}

void DrawBoundaryConditions(MinuteBotsProto::DebugDrawings* drawings) {
  DrawLine<double>(x0, x0 + v0, Color4f::kBlack, drawings);
  DrawLine<double>(xf, xf + vf, Color4f::kBlack, drawings);
}

void DefineProblemConditions(SoccerDebugMessage* message) {
  TextTree sub_log, log;

  MinuteBotsProto::DebugDrawings& drawings = *message->mutable_drawings();
  drawings.clear_lines();
  drawings.clear_arcs();
  drawings.clear_points();

  float x1 = -kHalfFieldLength +
             2 * kHalfFieldLength * static_cast<float>(rand_r(&rand_seed_)) /
                 static_cast<float>(RAND_MAX);
  float x2 = -kHalfFieldWidth +
             2 * kHalfFieldWidth * static_cast<float>(rand_r(&rand_seed_)) /
                 static_cast<float>(RAND_MAX);

  float v1 = kMaxRobotVelocity * static_cast<float>(rand_r(&rand_seed_)) /
             static_cast<float>(RAND_MAX);
  float v2 = 0.0;

  //   v1 = v1 * sqrt(2) / 2;
  //   v2 = v2 * sqrt(2) / 2;

  x0 << x1, x2;
  v0 << v1, v2;
  xf << 0, 0;
  vf << 0, 0;
  tsocs_instance = Tsocs(x0, v0, xf, vf, kDefaultRobotAcceleration);

  sub_log.set_text(StringPrintf("Initial Conditions"));
  *sub_log.add_sub_tree() =
      NewTextTree(StringPrintf("pos = (%.3f, %.3f)", x0.x(), x0.y()));
  *sub_log.add_sub_tree() =
      NewTextTree(StringPrintf("vel = (%.3f, %.3f)", v0.x(), v0.y()));
  *sub_log.add_sub_tree() =
      NewTextTree(StringPrintf("max_vel = %f", kMaxRobotVelocity));
  *sub_log.add_sub_tree() =
      NewTextTree(StringPrintf("max_accel = %f", kDefaultRobotAcceleration));

  *log.add_sub_tree() = sub_log;
  log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
  sub_log.Clear();

  DrawBoundaryConditions(&drawings);

  *(message->mutable_text_log()) = log;
}

void DefineProblemConditionsFinalV(SoccerDebugMessage* message) {
  TextTree sub_log, log;

  MinuteBotsProto::DebugDrawings& drawings = *message->mutable_drawings();
  drawings.clear_lines();
  drawings.clear_arcs();
  drawings.clear_points();

  float x1 = -kHalfFieldLength +
             2 * kHalfFieldLength * static_cast<float>(rand_r(&rand_seed_)) /
                 static_cast<float>(RAND_MAX);
  float x2 = -kHalfFieldWidth +
             2 * kHalfFieldWidth * static_cast<float>(rand_r(&rand_seed_)) /
                 static_cast<float>(RAND_MAX);

  float v1 = -kMaxRobotVelocity +
             2 * kMaxRobotVelocity * static_cast<float>(rand_r(&rand_seed_)) /
                 static_cast<float>(RAND_MAX);
  float v2 = -kMaxRobotVelocity +
             2 * kMaxRobotVelocity * static_cast<float>(rand_r(&rand_seed_)) /
                 static_cast<float>(RAND_MAX);

  float v1f = kMaxRobotVelocity * static_cast<float>(rand_r(&rand_seed_)) /
              static_cast<float>(RAND_MAX);
  float v2f = 0;

  v1 = v1 * sqrt(2) / 2;
  v2 = v2 * sqrt(2) / 2;
  //   v1f = v1f * sqrt(2) / 2;
  //   v2f = v2f * sqrt(2) / 2;

  x0 << x1, x2;
  v0 << v1, v2;
  xf << 0, 0;
  vf << v1f, v2f;
  tsocs_instance = Tsocs(x0, v0, xf, vf, kDefaultRobotAcceleration);

  sub_log.set_text(StringPrintf("Initial Conditions"));
  *sub_log.add_sub_tree() =
      NewTextTree(StringPrintf("pos = (%.3f, %.3f)", x0.x(), x0.y()));
  *sub_log.add_sub_tree() =
      NewTextTree(StringPrintf("vel = (%.3f, %.3f)", v0.x(), v0.y()));
  *sub_log.add_sub_tree() =
      NewTextTree(StringPrintf("final vel = (%.3f, %.3f)", vf.x(), vf.y()));
  *sub_log.add_sub_tree() =
      NewTextTree(StringPrintf("max_vel = %f", kMaxRobotVelocity));
  *sub_log.add_sub_tree() =
      NewTextTree(StringPrintf("max_accel = %f", kDefaultRobotAcceleration));

  *log.add_sub_tree() = sub_log;
  log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
  sub_log.Clear();

  DrawBoundaryConditions(&drawings);

  *(message->mutable_text_log()) = log;
}

void LoadTSOCSProblems(const string& file, vector<TSOCSProblem>* problems) {
  ScopedFile fid(file.c_str(), "r");
  if (fid() == NULL) {
    printf("ERROR: Unable to load TSOCS failures file '%s'.\n", file.c_str());
    return;
  }
  Eigen::Vector2d x0;
  Eigen::Vector2d v0;
  Eigen::Vector2d xf;
  Eigen::Vector2d vf;
  problems->clear();
  while (fscanf(fid(), "%lf, %lf, %lf, %lf, %lf, %lf, %lf, %lf", &(x0.x()),
                &(x0.y()), &(v0.x()), &(v0.y()), &(xf.x()), &(xf.y()),
                &(vf.x()), &(vf.y())) == 8) {
    problems->push_back(TSOCSProblem(x0, v0, xf, vf));
  }
  printf("Loaded %d problems from '%s'\n", static_cast<int>(problems->size()),
         file.c_str());
}

void LoadFailureCasesFinalV(SoccerDebugMessage* message,
                            const string filename) {
  static bool loaded = false;
  static vector<TSOCSProblem> problems;
  static int i = 0;
  if (!loaded) {
    LoadTSOCSProblems(filename, &problems);
    loaded = true;
  }
  if (problems.size() == 0) {
    printf("No TSOCS problems loaded\n");
    return;
  }
  i = (i % problems.size());

  TextTree sub_log, log;

  MinuteBotsProto::DebugDrawings& drawings = *message->mutable_drawings();
  drawings.clear_lines();
  drawings.clear_arcs();
  drawings.clear_points();

  x0 = problems[i].x0;
  v0 = problems[i].v0;
  xf = problems[i].xf;
  vf = problems[i].vf;
  tsocs_instance = Tsocs(x0, v0, xf, vf, kDefaultRobotAcceleration);
  ++i;

  sub_log.set_text(StringPrintf("Initial Conditions"));
  *sub_log.add_sub_tree() =
      NewTextTree(StringPrintf("pos = (%.3f, %.3f)", x0.x(), x0.y()));
  *sub_log.add_sub_tree() =
      NewTextTree(StringPrintf("vel = (%.3f, %.3f)", v0.x(), v0.y()));
  *sub_log.add_sub_tree() =
      NewTextTree(StringPrintf("final vel = (%.3f, %.3f)", vf.x(), vf.y()));
  *sub_log.add_sub_tree() =
      NewTextTree(StringPrintf("max_vel = %f", kMaxRobotVelocity));
  *sub_log.add_sub_tree() =
      NewTextTree(StringPrintf("max_accel = %f", kDefaultRobotAcceleration));

  *log.add_sub_tree() = sub_log;
  log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
  sub_log.Clear();

  DrawLine<double>(x0, x0 + v0, Color4f::kBlack, &drawings);
  DrawLine<double>(xf, xf + vf, Color4f::kBlack, &drawings);

  *(message->mutable_text_log()) = log;
}

void LoadInterceptCase(SoccerDebugMessage* message, const string filename) {
  ball_problem = true;
  static bool loaded = false;
  static vector<TSOCSProblem> problems;
  static int i = 0;
  if (!loaded) {
    LoadTSOCSProblems(filename, &problems);
    loaded = true;
  }
  if (problems.size() == 0) {
    printf("No TSOCS problems loaded\n");
    return;
  }
  i = (i % problems.size());

  TextTree sub_log, log;

  MinuteBotsProto::DebugDrawings& drawings = *message->mutable_drawings();
  drawings.clear_lines();
  drawings.clear_arcs();
  drawings.clear_points();

  x0 = problems[i].x0;
  v0 = problems[i].v0;
  ball_pos = problems[i].xf;
  ball_vel = problems[i].vf;
  intercept = BallInterception(x0, v0, ball_pos, ball_vel,
    kDefaultRobotAcceleration, kBallAcceleration);
  ++i;

  sub_log.set_text(StringPrintf("Initial Conditions"));
  *sub_log.add_sub_tree() =
      NewTextTree(StringPrintf("pos = (%.3f, %.3f)", x0.x(), x0.y()));
  *sub_log.add_sub_tree() =
      NewTextTree(StringPrintf("vel = (%.3f, %.3f)", v0.x(), v0.y()));
  *sub_log.add_sub_tree() =
      NewTextTree(StringPrintf("max_vel = %f", kMaxRobotVelocity));
  *sub_log.add_sub_tree() =
      NewTextTree(StringPrintf("max_accel = %f", kDefaultRobotAcceleration));

  *log.add_sub_tree() = sub_log;
  log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
  sub_log.Clear();

  DrawLine<double>(x0, x0 + v0, Color4f::kBlack, &drawings);
  DrawLine<double>(xf, xf + vf, Color4f::kBlack, &drawings);

  *(message->mutable_text_log()) = log;
}

void TestFailureCase(SoccerDebugMessage* message) {
  LoadFailureCasesFinalV(message, "TSOCS-failures-finalV.txt");
  VisualizeTSOCS(message);
}

void TestIterativeFailureCase(SoccerDebugMessage* message) {
  LoadFailureCasesFinalV(message, "TSOCS-iterative-failures-finalV.txt");
  ScopedFile fid("/dev/null", "a");
  VisualizeIterativeTSOCS(message, &fid);
}

void TestSimulatedFailureCase(SoccerDebugMessage* message) {
  MinuteBotsProto::DebugDrawings& drawings = *message->mutable_drawings();
  TextTree sub_sub_log, sub_log, log;
  // TextTree log = *(message->mutable_text_log());
  static bool success = true;
  if (success) {
    LoadFailureCasesFinalV(message, "TSOCS-simulated-iterative-failures.txt");
    VisualizeTSOCS(message);
  } else {
    DrawBoundaryConditions(&drawings);
    LoadFailureCasesFinalV(message, "TSOCS-simulated-iterative-failures.txt");
    bool foundSolution = tsocs_instance.GetSolution(&params);
    const Color4f solution_color =
        params.cost < kTSOCSThreshold ? kSuccessColor : kFailureColor;
    DrawBoundaryConditions(&drawings);
    DrawTSOCSPath(params, solution_color, &drawings);

    sub_log.set_text(
        StringPrintf("Stage 2 log10(cost) is %f", log10(params.cost)));
    *log.add_sub_tree() = sub_log;
    log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
    sub_log.Clear();
    if (foundSolution) {
      sub_log.set_text(StringPrintf("TSOCS Travel Time: %.3fs", params.T));
      *log.add_sub_tree() = sub_log;
      log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
      sub_log.Clear();

      sub_log.set_text(
          StringPrintf("TSOCS Solution: a = %.3f, b = %.3f, c = %.3f, d = %.3f",
                       params.a, params.b, params.c, params.d));
      *log.add_sub_tree() = sub_log;
      log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
      sub_log.Clear();
    } else {
      sub_log.set_text(StringPrintf("TSOCS was Unsuccessful"));
      *log.add_sub_tree() = sub_log;
      log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
      sub_log.Clear();

      sub_log.set_text(StringPrintf(
          "final params: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", params.a,
          params.b, params.c, params.d, params.T, params.cost));
      *log.add_sub_tree() = sub_log;
      log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
      sub_log.Clear();
    }
    *(message->mutable_text_log()) = log;
  }
  success = !success;
}

/**/
void DefineInterceptionConditions(SoccerDebugMessage* message) {
  TextTree sub_log, log;
  MinuteBotsProto::DebugDrawings& drawings = *message->mutable_drawings();
  drawings.clear_lines();
  drawings.clear_arcs();
  drawings.clear_points();
  const double kMaxBallVelocity = 5000;

  float x1 = -kHalfFieldLength +
             2 * kHalfFieldLength * static_cast<float>(rand_r(&rand_seed_)) /
                 static_cast<float>(RAND_MAX);
  float x2 = -kHalfFieldWidth +
             2 * kHalfFieldWidth * static_cast<float>(rand_r(&rand_seed_)) /
                 static_cast<float>(RAND_MAX);

  float v1 = -kMaxRobotVelocity +
             2 * kMaxRobotVelocity * static_cast<float>(rand_r(&rand_seed_)) /
                 static_cast<float>(RAND_MAX);
  float v2 = -kMaxRobotVelocity +
             2 * kMaxRobotVelocity * static_cast<float>(rand_r(&rand_seed_)) /
                 static_cast<float>(RAND_MAX);

  v1 = v1 * sqrt(2) / 2;
  v2 = v2 * sqrt(2) / 2;

  x0 << x1, x2;
  v0 << v1, v2;

  float ball_v1 = kMaxBallVelocity * static_cast<float>(rand_r(&rand_seed_)) /
                  static_cast<float>(RAND_MAX);

  ball_pos << 0, 0;
  ball_vel << ball_v1, 0;

  intercept = BallInterception(x0, v0, ball_pos, ball_vel,
    kDefaultRobotAcceleration, kBallAcceleration);

  const double halt_time = v0.norm() / kDefaultRobotAcceleration;
  MinuteBotsProto::ColoredLine& line = *drawings.add_lines();
  line.mutable_value()->mutable_v0()->set_x(x0.x());
  line.mutable_value()->mutable_v0()->set_y(x0.y());

  line.mutable_value()->mutable_v1()->set_x(x0.x() + v0.x() * halt_time -
                                            .5 * halt_time * halt_time *
                                                kDefaultRobotAcceleration *
                                                v0.x() / v0.norm());

  line.mutable_value()->mutable_v1()->set_y(x0.y() + v0.y() * halt_time -
                                            .5 * halt_time * halt_time *
                                                kDefaultRobotAcceleration *
                                                v0.y() / v0.norm());
  line.mutable_color()->set_r(0);
  line.mutable_color()->set_g(0);
  line.mutable_color()->set_b(0);
  line.mutable_color()->set_a(1);

  sub_log.set_text(StringPrintf("Initial Conditions"));
  *sub_log.add_sub_tree() =
      NewTextTree(StringPrintf("robot pos = (%.3f, %.3f)", x0.x(), x0.y()));
  *sub_log.add_sub_tree() =
      NewTextTree(StringPrintf("robot vel = (%.3f, %.3f)", v0.x(), v0.y()));
  *sub_log.add_sub_tree() = NewTextTree(
      StringPrintf("ball pos = (%.3f, %.3f)", ball_pos[0], ball_pos[1]));
  *sub_log.add_sub_tree() = NewTextTree(
      StringPrintf("ball vel = (%.3f, %.3f)", ball_vel.x(), ball_vel.y()));
  *sub_log.add_sub_tree() =
      NewTextTree(StringPrintf("max_accel = %f", kDefaultRobotAcceleration));

  *log.add_sub_tree() = sub_log;
  log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
  sub_log.Clear();

  *(message->mutable_text_log()) = log;
}

void VisualizeNTOC(SoccerDebugMessage* message) {
  TextTree sub_sub_log, sub_log;
  TextTree log = *(message->mutable_text_log());
  log.set_text("");

  Eigen::Vector2f x = x0.cast<float>();
  Eigen::Vector2f v = v0.cast<float>();

  // MotionModel model(kMaxAcceleration, kMaxVelocity);
  MotionModel model(kDefaultRobotAcceleration, INFINITY);
  ControlSequence2D controls;
  float total_time = NTOC2D(x, v, model, &controls);

  Eigen::Vector2f x_prev;
  Eigen::Vector2f v_prev;

  MinuteBotsProto::DebugDrawings& drawings = *message->mutable_drawings();

  int phase_count = 0;
  for (const auto& phase : controls.phases) {
    ++phase_count;
    const Eigen::Vector2f a = phase.acceleration;
    const float phase_time = phase.duration;
    const int phase_steps = 1 + time_steps * phase_time / total_time;
    const float step_duration = phase_time / phase_steps;

    x_prev = x;
    v_prev = v;

    for (int i = 0; i < phase_steps; i++) {
      MinuteBotsProto::ColoredPoint& point = *drawings.add_points();
      point.mutable_value()->set_x(x[0]);
      point.mutable_value()->set_y(x[1]);
      if (i > 0) {
        point.mutable_color()->set_r(0);
        point.mutable_color()->set_g(0);
        point.mutable_color()->set_b(1);
        point.mutable_color()->set_a(1);
      } else {
        point.mutable_color()->set_r(1);
        point.mutable_color()->set_g(0);
        point.mutable_color()->set_b(0);
        point.mutable_color()->set_a(1);
      }
      x = x_prev + v_prev * step_duration * i +
          .5 * a * step_duration * step_duration * i * i;
      v = v_prev + a * step_duration * i;
    }

    MinuteBotsProto::ColoredPoint& point = *drawings.add_points();
    point.mutable_value()->set_x(x[0]);
    point.mutable_value()->set_y(x[1]);
    point.mutable_color()->set_r(0);
    point.mutable_color()->set_g(0);
    point.mutable_color()->set_b(1);
    point.mutable_color()->set_a(1);

    x = x_prev + v_prev * phase_time + .5 * a * phase_time * phase_time;
    v = v_prev + a * phase_time;

    sub_log.set_text(StringPrintf("NTOC Phase %d End", phase_count));
    *sub_log.add_sub_tree() =
        NewTextTree(StringPrintf("pos = (%.3f, %.3f)", x[0], x[1]));
    *sub_log.add_sub_tree() =
        NewTextTree(StringPrintf("vel = (%.3f, %.3f)", v[0], v[1]));
    *sub_log.add_sub_tree() =
        NewTextTree(StringPrintf("accel = (%.3f, %.3f)", a[0], a[1]));
    *sub_log.add_sub_tree() =
        NewTextTree(StringPrintf("duration = %.3f", phase_time));

    *log.add_sub_tree() = sub_log;
    log.mutable_sub_tree(phase_count)
        ->set_text(log.sub_tree(phase_count).text());
    sub_log.Clear();
  }
  sub_log.set_text(StringPrintf("NTOC Travel Time: %.3fs", total_time));
  *log.add_sub_tree() = sub_log;
  log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
  sub_log.Clear();

  *(message->mutable_text_log()) = log;
}

void DrawTSOCSPath(const SolutionParameters& params, const Color4f& col,
                   MinuteBotsProto::DebugDrawings* drawings) {
  const double dt = 0.001 * kTransmitPeriod;
  for (double t = 0; t < params.T; t += dt) {
    Eigen::Vector2d xt;
    Eigen::Vector2d vt;
    if (kNewTSOCS) {
      tsocs_instance.GetState(&xt, &vt, t, params);
    } else {
      GetState(x0, v0, &xt, &vt, kDefaultRobotAcceleration, t, params);
    }
    DrawPoint(xt, col, drawings);
  }
}

void DrawTSOCSPath(const SolutionParameters& params, const Color4f& col,
                   MinuteBotsProto::DebugDrawings* drawings,
                   const char* filename) {
  const double dt = 0.001 * kTransmitPeriod;
  std::FILE* fid = std::fopen(filename, "w");
  fprintf(fid, "t, x1, x2, v1, v2\n");
  for (double t = 0; t < params.T; t += dt) {
    Eigen::Vector2d xt;
    Eigen::Vector2d vt;
    if (kNewTSOCS) {
      tsocs_instance.GetState(&xt, &vt, t, params);
    } else {
      GetState(x0, v0, &xt, &vt, kDefaultRobotAcceleration, t, params);
    }
    DrawPoint(xt, col, drawings);
    fprintf(fid, "%12f, %12f, %12f, %12f, %12f\n", t, xt.x(), xt.y(), vt.x(),
            vt.y());
  }
  std::fclose(fid);
}

void DrawTSOCSPath(const SolutionParameters_alt& params, const Color4f& col,
                   MinuteBotsProto::DebugDrawings* drawings) {
  //     const double dt = 0.001 * kTransmitPeriod;
  //     for (double t = 0; t < params.T; t += dt) {
  //       Eigen::Vector2d xt;
  //       Eigen::Vector2d vt;
  //       temp::GetState_alt(x0, v0, &xt, &vt,
  //                          kDefaultRobotAcceleration, t, params);
  //       DrawPoint(xt, col, drawings);
  //     }
}

bool VisualizeTSOCS(SoccerDebugMessage* message) {
  ball_problem = false;
  MinuteBotsProto::DebugDrawings& drawings = *message->mutable_drawings();

  TextTree sub_sub_log, sub_log;
  TextTree log = *(message->mutable_text_log());
  log.set_text("");

  // Draw three-part solution.
  Vector2d x1 = x0;
  Vector2d x2 = xf;
  if (v0.squaredNorm() > kEpsilon) {
    const float stopping_distance =
        v0.squaredNorm() / (2.0 * kDefaultRobotAcceleration);
    x1 += stopping_distance * v0.normalized();
  }
  if (vf.squaredNorm() > kEpsilon) {
    const float stopping_distance =
        vf.squaredNorm() / (2.0 * kDefaultRobotAcceleration);
    x2 -= stopping_distance * vf.normalized();
  }
  DrawLine(x0, x1, kLineColor, &drawings);
  DrawLine(x1, x2, kLineColor, &drawings);
  DrawLine(x2, xf, kLineColor, &drawings);

  std::vector<SolutionParameters> params_list;
  bool foundSolution;
  if (kUsePerturbationsFix) {
    int perturbations;
    foundSolution = tsocs_instance.GetSolutionSet(&params_list, &perturbations);
    if (foundSolution) {
      if (perturbations_histogram != NULL) {
        perturbations_histogram[perturbations]++;
      }
      sub_log.set_text(StringPrintf("TSOCS succeeded after %d perturbations",
                                    perturbations));
      *log.add_sub_tree() = sub_log;
      log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
      sub_log.Clear();

      sub_log.set_text(StringPrintf("Stage 2 log10(cost) is %f",
                                    log10(params_list.at(2).cost)));
      *log.add_sub_tree() = sub_log;
      log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
      sub_log.Clear();
    }
  } else {
    foundSolution = tsocs_instance.GetSolutionSet(&params_list);
  }
  const SolutionParameters params_guess = params_list.at(0);
  params_stage1 = params_list.at(1);
  params = params_list.at(2);
  const double total_time = params.T;
  if (foundSolution) {
    sub_log.set_text(StringPrintf("TSOCS Travel Time: %.5fs", total_time));
    *log.add_sub_tree() = sub_log;
    log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
    sub_log.Clear();

    sub_log.set_text(
        StringPrintf("TSOCS Stage 1 Time: %.3fs", params_stage1.T));
    *log.add_sub_tree() = sub_log;
    log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
    sub_log.Clear();

    if (kUsePolarParams) {
      // TODO(afischer)
    } else if (kUseDoublePolarParams) {
      sub_log.set_text(
          StringPrintf("Double Polar Params: theta1 = %f, theta2 = %f, theta3 "
                       "= %f, T = %.3f",
                       params.a, params.b, params.c, params.T));
      *log.add_sub_tree() = sub_log;
      log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
      sub_log.Clear();
    } else {
      sub_log.set_text(
          StringPrintf("TSOCS Solution: a = %.3f, b = %.3f, c = %.3f, d = %.3f",
                       params.a, params.b, params.c, params.d));
      *log.add_sub_tree() = sub_log;
      log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
      sub_log.Clear();
    }
  } else {
    sub_log.set_text(StringPrintf("TSOCS was Unsuccessful"));
    *log.add_sub_tree() = sub_log;
    log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
    sub_log.Clear();

    if (kUsePolarParams) {
      // TODO(afischer)
    } else if (kUseDoublePolarParams) {
      sub_log.set_text(
          StringPrintf("Double Polar Params: theta1 = %f, theta2 = %f, theta3 "
                       "= %f, T = %.3f",
                       params.a, params.b, params.c, params.T));
      *log.add_sub_tree() = sub_log;
      log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
      sub_log.Clear();
    } else {
      sub_log.set_text(StringPrintf(
          "guessed params: %.3f, %.3f, %.3f, %.3f, %.3f", params_guess.a,
          params_guess.b, params_guess.c, params_guess.d, params_guess.T));
      *log.add_sub_tree() = sub_log;
      log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
      sub_log.Clear();

      sub_log.set_text(StringPrintf(
          "stage 1 solution params: %.3f, %.3f, %.3f, %.3f, %.3f, %f",
          params_stage1.a, params_stage1.b, params_stage1.c, params_stage1.d,
          params_stage1.T, params_stage1.cost));
      *log.add_sub_tree() = sub_log;
      log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
      sub_log.Clear();

      sub_log.set_text(StringPrintf(
          "final params: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", params.a,
          params.b, params.c, params.d, params.T, params.cost));
      *log.add_sub_tree() = sub_log;
      log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
      sub_log.Clear();
    }
  }
  sub_log.set_text(StringPrintf("log10(cost) = %3.2f", log10(params.cost)));
  *log.add_sub_tree() = sub_log;
  log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
  sub_log.Clear();

  const Color4f solution_color =
      params.cost < kTSOCSThreshold ? kSuccessColor : kFailureColor;
  const Color4f solution_color_faded =
      (foundSolution ? Color4f(1, 1, 0, 0.5) : Color4f(1, 0, 1, 0.5));
  DrawTSOCSPath(params_stage1, kStage1Color, &drawings);
  DrawTSOCSPath(params_guess, kGuessColor, &drawings);
  DrawTSOCSPath(params, solution_color, &drawings);

  Vector2d stage1_xf;
  Vector2d stage1_vf;
  GetState(x0, v0, &stage1_xf, &stage1_vf, kDefaultRobotAcceleration,
           params_stage1.T, params_stage1);
  DrawLine<double>(stage1_xf, stage1_xf + stage1_vf, Color4f(1, 0.647, 0, 0.5),
                   &drawings);

  Vector2d stage2_xf;
  Vector2d stage2_vf;
  GetState(x0, v0, &stage2_xf, &stage2_vf, kDefaultRobotAcceleration, params.T,
           params);
  DrawLine<double>(stage2_xf, stage2_xf + stage2_vf, solution_color_faded,
                   &drawings);

  *(message->mutable_text_log()) = log;
  return foundSolution;
}

bool VisualizeTSOCS_alt(SoccerDebugMessage* message) {
  MinuteBotsProto::DebugDrawings& drawings = *message->mutable_drawings();

  TextTree sub_sub_log, sub_log;
  TextTree log = *(message->mutable_text_log());
  log.set_text("");

  std::vector<SolutionParameters_alt> params_list;
  const bool foundSolution = temp::GetSolutionSet_alt(
      x0, v0, xf, vf, kDefaultRobotAcceleration, &params_list);
  const SolutionParameters_alt params_guess = params_list.at(0);
  const SolutionParameters_alt params_stage1 = params_list.at(1);
  const SolutionParameters_alt params = params_list.at(2);
  const double total_time = params.T;
  if (foundSolution) {
    sub_log.set_text(StringPrintf("TSOCS Travel Time: %.3fs", total_time));
    *log.add_sub_tree() = sub_log;
    log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
    sub_log.Clear();

    sub_log.set_text(
        StringPrintf("TSOCS Estimated Time: %.3fs", params_stage1.T));
    *log.add_sub_tree() = sub_log;
    log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
    sub_log.Clear();
  } else {
    sub_log.set_text(StringPrintf("TSOCS was Unsuccessful"));
    *log.add_sub_tree() = sub_log;
    log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
    sub_log.Clear();

    sub_log.set_text(StringPrintf("params: %.3f, %.3f, %.3f, %.3f",
                                  params.theta_0, params.theta_T, params.r,
                                  params.T));
    *log.add_sub_tree() = sub_log;
    log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
    sub_log.Clear();
  }

  const Color4f solution_color = foundSolution ? kSuccessColor : kFailureColor;
  DrawTSOCSPath(params, solution_color, &drawings);
  DrawTSOCSPath(params_stage1, kStage1Color, &drawings);
  DrawTSOCSPath(params_guess, kGuessColor, &drawings);

  *(message->mutable_text_log()) = log;
  return foundSolution;
}

bool VisualizeInterception(SoccerDebugMessage* message) {
  ball_problem = true;
  TextTree sub_sub_log, sub_log;
  TextTree log = *(message->mutable_text_log());
  log.set_text("");

  MinuteBotsProto::DebugDrawings& drawings = *message->mutable_drawings();
  bool foundSolution;
  ball_params.isInitialized = false;
  if (new_interception) {
    foundSolution = intercept.GetInterceptSolution(&ball_params);
  } else {
    foundSolution =
        GetInterceptSolution(x0, v0, ball_pos, ball_vel, ball_accel,
                             kDefaultRobotAcceleration, &ball_params);
  }
  const double rest_time = ball_vel.norm() / ball_accel;
  const double total_time = ball_params.T;
  if (foundSolution) {
    sub_log.set_text(StringPrintf("Time to Intercept: %.3fs", total_time));
    *log.add_sub_tree() = sub_log;
    log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
    sub_log.Clear();

    sub_log.set_text(StringPrintf("Rest Time: %.3fs", rest_time));
    *log.add_sub_tree() = sub_log;
    log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
    sub_log.Clear();

    Eigen::Vector2d intercept_pos =
        ball_pos + ball_vel * total_time +
        .5 * ball_accel * total_time * total_time * ball_vel / ball_vel.norm();
    Eigen::Vector2d rest_pos =
        ball_pos + ball_vel * rest_time +
        .5 * ball_accel * rest_time * rest_time * ball_vel / ball_vel.norm();

    sub_log.set_text(StringPrintf("Intercept Point: (%.3f, %.3f)",
                                  intercept_pos.x(), intercept_pos.y()));
    *log.add_sub_tree() = sub_log;
    log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
    sub_log.Clear();

    sub_log.set_text(
        StringPrintf("Rest Point: (%.3f, %.3f)", rest_pos.x(), rest_pos.y()));
    *log.add_sub_tree() = sub_log;
    log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
    sub_log.Clear();
  } else {
    sub_log.set_text(StringPrintf("Interception was Unsuccessful"));
    *log.add_sub_tree() = sub_log;
    log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
    sub_log.Clear();

    sub_log.set_text(StringPrintf("params: %.3f, %.3f, %.3f, %.3f, %.3f",
                                  ball_params.a, ball_params.b, ball_params.c,
                                  ball_params.d, ball_params.T));
    *log.add_sub_tree() = sub_log;
    log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
    sub_log.Clear();
  }
  sub_log.set_text(StringPrintf("log10(cost) = %f", log10(ball_params.cost)));
  *log.add_sub_tree() = sub_log;
  log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
  sub_log.Clear();

  DrawInterceptionPath(
      ball_params, foundSolution ? kSuccessColor : kFailureColor, &drawings);

  *(message->mutable_text_log()) = log;
  return foundSolution;
}

void DrawInterceptionPath(SolutionParameters intercept_params, Color4f color,
                          MinuteBotsProto::DebugDrawings* drawings) {
  for (double time = 0; time < intercept_params.T;
       time += 1 / kTransmitFrequency) {
    Eigen::Vector2d robot_xt;
    Eigen::Vector2d robot_vt;
    Eigen::Vector2d ball_xt;
    Eigen::Vector2d ball_vt;
    if (new_interception) {
      intercept.GetState(&robot_xt, &robot_vt, &ball_xt, &ball_vt, time,
                         intercept_params);
    } else {
      GetState(x0, v0, ball_pos, ball_vel, &robot_xt, &robot_vt, &ball_xt,
               &ball_vt, kDefaultRobotAcceleration, time, ball_accel,
               intercept_params);
    }

    MinuteBotsProto::ColoredPoint& robot_point = *drawings->add_points();
    robot_point.mutable_value()->set_x(robot_xt.x());
    robot_point.mutable_value()->set_y(robot_xt.y());
    robot_point.mutable_color()->set_r(color.r);
    robot_point.mutable_color()->set_g(color.g);
    robot_point.mutable_color()->set_b(color.b);
    robot_point.mutable_color()->set_a(color.a);

    MinuteBotsProto::ColoredPoint& ball_point = *drawings->add_points();
    ball_point.mutable_value()->set_x(ball_xt.x());
    ball_point.mutable_value()->set_y(ball_xt.y());
    ball_point.mutable_color()->set_r(0);
    ball_point.mutable_color()->set_g(0);
    ball_point.mutable_color()->set_b(1);
    ball_point.mutable_color()->set_a(1);
  }
}

bool VisualizeIterativeNTOC(SoccerDebugMessage* message, ScopedFile* fid) {
  MinuteBotsProto::DebugDrawings& drawings = *message->mutable_drawings();

  VisualizeNTOC(message);

  TextTree sub_sub_log, sub_log;
  TextTree log = *(message->mutable_text_log());
  log.set_text("");

  // Draw three-part solution.
  Vector2d x1 = x0;
  Vector2d x2 = xf;
  if (v0.squaredNorm() > kEpsilon) {
    const float stopping_distance =
        v0.squaredNorm() / (2.0 * kDefaultRobotAcceleration);
    x1 += stopping_distance * v0.normalized();
  }
  if (vf.squaredNorm() > kEpsilon) {
    const float stopping_distance =
        vf.squaredNorm() / (2.0 * kDefaultRobotAcceleration);
    x2 -= stopping_distance * vf.normalized();
  }
  DrawLine(x0, x1, kLineColor, &drawings);
  DrawLine(x1, x2, kLineColor, &drawings);
  DrawLine(x2, xf, kLineColor, &drawings);

  std::vector<SolutionParameters> params_list;
  bool foundSolution = tsocs_instance.GetSolutionSet(&params_list);
  if (!foundSolution) {
    return false;
  }

  const double dt = 0.001 * kTransmitPeriod;
  const double noise_level = 0.05;

  normal_distribution<double> distribution(1.0, noise_level);

  Vector2d xt = x0;
  Vector2d vt = v0;

  double time_elapsed = 0.0;

  const MotionModel motion_model(kDefaultRobotAcceleration, INFINITY);
  ControlSequence2D control;

  NTOC2D(x0.cast<float>(), v0.cast<float>(), motion_model, &control);

  const Color4f col = Color4f(1, 1, 0, 1);
  bool finished = NTOCFinished(xt, vt, xf);
  while (!finished) {
    Eigen::Vector2d xdt;
    Eigen::Vector2d vdt;

    Eigen::Vector2d u_commanded = GetAverageAccel(control, dt).cast<double>();
    vdt = (vt + u_commanded * dt) * distribution(generator);
    Eigen::Vector2d u_actual = (vdt - vt) / dt;
    xdt = xt + vt * dt + 0.5 * u_actual * dt * dt;

    control.Reset();
    NTOC2D(xdt.cast<float>(), vdt.cast<float>(), motion_model, &control);

    DrawPoint(xt, col, &drawings);

    xt = xdt;
    vt = vdt;
    time_elapsed += dt;
    finished = NTOCFinished(xt, vt, xf);
  }
  *(message->mutable_text_log()) = log;
  return true;
}

bool VisualizeIterativeTSOCS(SoccerDebugMessage* message, ScopedFile* fid) {
  MinuteBotsProto::DebugDrawings& drawings = *message->mutable_drawings();
  const bool isOpenLoop = false;

  TextTree sub_sub_log, sub_log;
  TextTree log = *(message->mutable_text_log());
  log.set_text("");

  // Draw three-part solution.
  Vector2d x1 = x0;
  Vector2d x2 = xf;
  if (v0.squaredNorm() > kEpsilon) {
    const float stopping_distance =
        v0.squaredNorm() / (2.0 * kDefaultRobotAcceleration);
    x1 += stopping_distance * v0.normalized();
  }
  if (vf.squaredNorm() > kEpsilon) {
    const float stopping_distance =
        vf.squaredNorm() / (2.0 * kDefaultRobotAcceleration);
    x2 -= stopping_distance * vf.normalized();
  }
  DrawLine(x0, x1, kLineColor, &drawings);
  DrawLine(x1, x2, kLineColor, &drawings);
  DrawLine(x2, xf, kLineColor, &drawings);

  std::vector<SolutionParameters> params_list;
  bool foundSolution = tsocs_instance.GetSolutionSet(&params_list);
  SolutionParameters params = params_list.at(2);

  const Color4f original_sol_color = Color4f(0, 0, 1, 1);
  DrawTSOCSPath(params, original_sol_color, &drawings);

  const double dt = 0.001 * kTransmitPeriod;
  const double noise_level = 0.0;

  normal_distribution<double> distribution(1.0, noise_level);

  Vector2d xt = x0;
  Vector2d vt = v0;
  const Color4f solution_color = kSuccessColor;
  const Color4f open_loop_color = Color4f(1, 0, 0, 1);

  double time_elapsed = 0.0;

  bool failed = !foundSolution;
  bool finished = tsocs_instance.Finished(params);
  while (!finished) {
    if (foundSolution) {
      DrawPoint(xt, solution_color, &drawings);
    } else {
      DrawPoint(xt, open_loop_color, &drawings);
    }
    Eigen::Vector2d xdt;
    Eigen::Vector2d vdt;
    GetState(xt, vt, &xdt, &vdt, kDefaultRobotAcceleration, dt, params);

    vdt.x() = vdt.x() * distribution(generator);
    vdt.y() = vdt.y() * distribution(generator);

    Vector2d adt = (vdt - vt) / dt;
    xdt = xt + vt * dt + 0.5 * adt * dt * dt;

    time_elapsed += dt;
    SolutionParameters params_save(params);
    if (!isOpenLoop) {
      tsocs_instance = Tsocs(xdt, vdt, xf, vf, kDefaultRobotAcceleration);
      foundSolution = tsocs_instance.GetSolution(&params);
    } else {
      foundSolution = false;
    }
    if (!foundSolution) {
      params = params_save;
      failed = true;
    }
    xt = xdt;
    vt = vdt;

    finished = tsocs_instance.Finished(params);
  }

  sub_log.set_text(StringPrintf("Final Conditions"));
  *sub_log.add_sub_tree() =
      NewTextTree(StringPrintf("pos = (%.3f, %.3f)", xt.x(), xt.y()));
  *sub_log.add_sub_tree() =
      NewTextTree(StringPrintf("vel = (%.3f, %.3f)", vt.x(), vt.y()));
  *log.add_sub_tree() = sub_log;
  log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
  sub_log.Clear();

  if (failed) {
    sub_log.set_text(StringPrintf("TSOCS was Unsuccessful"));
    *log.add_sub_tree() = sub_log;
    log.mutable_sub_tree(0)->set_text(log.sub_tree(0).text());
    sub_log.Clear();
  }

  *(message->mutable_text_log()) = log;
  return !failed;
}

void GenerateFigure(SoccerDebugMessage* message, Viewer* viewer) {
  MinuteBotsProto::DebugDrawings& drawings = *message->mutable_drawings();
  drawings.clear_lines();
  drawings.clear_arcs();
  drawings.clear_points();

  x0 << -kHalfFieldLength * static_cast<double>(rand_r(&rand_seed_)) /
            static_cast<double>(RAND_MAX),
      0.0;
  v0 << 0.0, 0.0;
  xf << 0.0, 0.0;
  double v1 = -kMaxRobotVelocity +
              2 * kMaxRobotVelocity * static_cast<double>(rand_r(&rand_seed_)) /
                  static_cast<double>(RAND_MAX);
  double v2 = -kMaxRobotVelocity +
              2 * kMaxRobotVelocity * static_cast<double>(rand_r(&rand_seed_)) /
                  static_cast<double>(RAND_MAX);

  vf << v1, v2;

  DrawBoundaryConditions(&drawings);

  SolutionParameters bad_guess = GetGuess();
  bad_guess.b = 2;
  bad_guess.T = 10;
  bad_guess.isInitialized = true;

  SolutionParameters naive_solution(bad_guess);
  tsocs_instance.GetSolution(&naive_solution);

  vector<SolutionParameters> solution_set;
  tsocs_instance.GetSolutionSet(&solution_set);

  SolutionParameters stage1_params = solution_set.at(1);
  SolutionParameters stage2_params = solution_set.at(2);

  const Color4f red_color = Color4f(1, 0, 0, 1);
  const Color4f orange_color = Color4f(1, 0.647, 0, 1);
  const Color4f purple_color = Color4f(0.627, 0.125, 0.941, 1);
  const Color4f yellow_color = Color4f(1, 1, 0, 1);

  printf("naive_params: %.3f, %.3f, %.3f, %.3f, %.3f\n", naive_solution.a,
         naive_solution.b, naive_solution.c, naive_solution.d,
         naive_solution.T);

  DrawTSOCSPath(bad_guess, orange_color, &drawings, "bad_guess.csv");
  DrawTSOCSPath(naive_solution, red_color, &drawings, "naive_solution.csv");
  DrawTSOCSPath(stage1_params, purple_color, &drawings, "stage1.csv");
  DrawTSOCSPath(stage2_params, yellow_color, &drawings, "stage2.csv");
}

void VisualizeArc(SoccerDebugMessage* message, Viewer* viewer) {
  static double angle = 0.0;
  angle += 0.1;
  angle = fmod(angle, 2 * M_PI);
  printf("Angle = %f deg\n", angle * 180 / M_PI);

  TextTree log;
  *(message->mutable_text_log()) = log;
  MinuteBotsProto::DebugDrawings& drawings = *message->mutable_drawings();
  drawings.clear_lines();
  drawings.clear_arcs();
  drawings.clear_points();
  x0 = Eigen::Vector2d(kDefaultRobotAcceleration, 0);
  v0 = Eigen::Vector2d(0, kDefaultRobotAcceleration);
  xf = Eigen::Vector2d(kDefaultRobotAcceleration * cos(angle),
                       kDefaultRobotAcceleration * sin(angle));
  vf = Eigen::Vector2d(kDefaultRobotAcceleration * cos(angle + M_PI / 2),
                       kDefaultRobotAcceleration * sin(angle + M_PI / 2));
  VisualizeTSOCS(message);
  // max velocity
  double max_vel = 0.0, argmax_vel = 0.0;
  for (double i = 0.0; i <= 1.0; i += 0.01) {
    double t = params.T * i;
    Eigen::Vector2d xt;
    Eigen::Vector2d vt;
    GetState(x0, v0, &xt, &vt, kDefaultRobotAcceleration, t, params);
    if (vt.norm() > max_vel) {
      max_vel = vt.norm();
      argmax_vel = i;
    }
  }
  printf("Maximum velocity is %f at i = %f\n", max_vel, argmax_vel);

  // draw the circle
  for (double i = 0.0; i < angle; i += 0.05) {
    Eigen::Vector2d loc(kDefaultRobotAcceleration * cos(i),
                        kDefaultRobotAcceleration * sin(i));
    DrawPoint(loc, Color4f(1, 1, 1, 1), &drawings);
  }
  viewer->Update(*message);
}

void MotionVisualizer::GenerateNewProblem(QKeyEvent* event) {
  switch (event->key()) {
    case Qt::Key_C: {
      // Draw a cost figure centered at the solution parameters
      center_stage1 = false;
      message_global = &message_;
      viewer_global = viewer_;
      pthread_t thread;
      int rc = pthread_create(&thread, NULL, VisualizeCostThread, NULL);
      if (rc) {
        printf("Error: unable to create thread\n");
      }
    } break;
    case Qt::Key_D: {
      // Draw a cost figure centered at the stage 1 parameters
      center_stage1 = true;
      message_global = &message_;
      viewer_global = viewer_;
      pthread_t thread;
      int rc = pthread_create(&thread, NULL, VisualizeCostThread, NULL);
      if (rc) {
        printf("Error: unable to create thread\n");
      }
    } break;
    case Qt::Key_N: {
      DrawNewProblem(&message_);
    } break;
    case Qt::Key_B: {
      DrawInterception(&message_);
    } break;
    case Qt::Key_E: {
      TestBallInterception(&message_, viewer_);
    } break;
    case Qt::Key_L: {
      LoadInterceptCase(&message_, "TSOCS-intercept-cases.txt");
      VisualizeInterception(&message_);
    } break;
    case Qt::Key_V: {
      DrawNewProblemFinalV(&message_);
    } break;
    case Qt::Key_T: {
      RandomTest(&message_, viewer_);
    } break;
    case Qt::Key_R: {
      RandomTestFinalV(&message_, viewer_);
    } break;
    case Qt::Key_F: {
      TestFailureCase(&message_);
    } break;
    case Qt::Key_G: {
      TestIterativeFailureCase(&message_);
    } break;
    case Qt::Key_S: {
      TestSimulatedFailureCase(&message_);
    } break;
    case Qt::Key_I: {
      DrawIterativeTSOCS(&message_);
    } break;
    case Qt::Key_U: {
      IterativeTest(&message_, viewer_);
    } break;
    case Qt::Key_P: {
      GenerateFigure(&message_, viewer_);
    } break;
    case Qt::Key_A: {
      VisualizeArc(&message_, viewer_);
    } break;
  }
  viewer_->Update(message_);
}

int main(int argc, char** argv) {
  QApplication app(argc, argv);
  logger::ReadLogger log("viewer_motion.log");
  generator.seed(rand_seed_);
  view = new Viewer(NULL, &log, false);
  MotionVisualizer visualizer(view);
  view->show();
  MotionVisualizer::connect(view, SIGNAL(KeypressSignal(QKeyEvent*)),
                            &visualizer, SLOT(GenerateNewProblem(QKeyEvent*)));
  int return_value = app.exec();
  return return_value;
}
