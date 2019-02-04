// Copyright 2017-2018 dbalaban@cs.umass.edu
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

#include <ceres/ceres.h>
#include <ceres/types.h>
#include <math.h>
#include <eigen3/Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "logging/logger.h"
#include "motion_control/motion_control_structures.h"
#include "motion_control/optimal_control_1d.h"
#include "motion_control/sub_optimal_controls.h"
#include "shared/common_includes.h"

#ifndef SRC_MOTION_CONTROL_TSOCS_OLD_H_
#define SRC_MOTION_CONTROL_TSOCS_OLD_H_

namespace tsocs {

extern const bool kUsePerturbationsFix;
extern const bool kUsePolarParams;
extern const bool kUseDoublePolarParams;
extern const int kMaxPerturbations;
extern const double kTSOCSThreshold;

// static ScopedFile tsocs_data_fid("TSOCS-data.txt", "a");

struct TSOCSProblem {
  TSOCSProblem(Eigen::Vector2d x0, Eigen::Vector2d v0, Eigen::Vector2d xf,
               Eigen::Vector2d vf)
      : x0(x0), v0(v0), xf(xf), vf(vf) {}

  TSOCSProblem(double x01, double x02, double v01, double v02, double xf1,
               double xf2, double vf1, double vf2)
      : x0(x01, x02), v0(v01, v02), xf(xf1, xf2), vf(vf1, vf2) {}

  Eigen::Vector2d x0;
  Eigen::Vector2d v0;
  Eigen::Vector2d xf;
  Eigen::Vector2d vf;
};

// guesses a solution parameter set
SolutionParameters GetGuess();

// finds a set of parameters which were used to find optimal solution
// given starting / ending position and velocity
// uses given initial guess
// bounded acceleration, unbounded velocity
// returns true if correct solution was found
bool GetSolutionSetFromGuess(Eigen::Vector2d x0, Eigen::Vector2d v0,
                             Eigen::Vector2d xf, Eigen::Vector2d vf,
                             const double a_max,
                             std::vector<SolutionParameters>* params,
                             const SolutionParameters guess);

// finds a set of parameters which were used to find optimal solution
// given starting / ending position and velocity
// uses given initial guess
// bounded acceleration, unbounded velocity
// returns true if correct solution was found
bool GetSolutionSetFromGuess(Eigen::Vector2d x0, Eigen::Vector2d v0,
                             Eigen::Vector2d xf, Eigen::Vector2d vf,
                             const double a_max,
                             std::vector<SolutionParameters>* params,
                             const SolutionParameters guess);

// finds a set of parameters which were used to find optimal solution
// given starting / ending position and velocity
// bounded acceleration, unbounded velocity
// returns true if correct solution was found
// perturbations stores how many perturbations it took to solve it
// if perturbations is null it doesn't store how many it took
bool GetSolutionSet(Eigen::Vector2d x0, Eigen::Vector2d v0, Eigen::Vector2d xf,
                    Eigen::Vector2d vf, const double a_max,
                    std::vector<SolutionParameters>* params,
                    int* perturbations = NULL);

// finds a single parameter set
// returns true if correct solution was found
// bool GetSolution(Eigen::Vector2d x0, Eigen::Vector2d v0,
//                     Eigen::Vector2d xf, Eigen::Vector2d vf,
//                     const double a_max,
//                     SolutionParameters* params);

bool GetSolution(Eigen::Vector2d x0, Eigen::Vector2d v0, Eigen::Vector2d xf,
                 Eigen::Vector2d vf, const double a_max,
                 SolutionParameters* params, logger::Logger* the_log = NULL);

// finds the expected position and velocity after time t
// given starting position and velocity and an optimal solution
// bounded acceleration, unbounded velocity
void GetState(Eigen::Vector2d x0, Eigen::Vector2d v0, Eigen::Vector2d* xt,
              Eigen::Vector2d* vt, const double a_max, const double t,
              SolutionParameters params);

void GetState(Eigen::Vector2d x0, Eigen::Vector2d v0, Eigen::Vector2d* xt,
              Eigen::Vector2d* vt, const double a_max, const double t,
              SolutionParameters params, logger::Logger* the_log);

std::vector<Eigen::Vector2f> GetPath(Eigen::Vector2d robot_pos_init,
                                     Eigen::Vector2d robot_vel_init,
                                     const double a_max,
                                     const unsigned int npts,
                                     SolutionParameters params);

// Returns the cost of the current problem given the parameters. The problem
// must have been previously set by a call to GetSolution or GetSolutionSet.
double CostFunction(double a, double b, double c, double d, double T);

double CostFunctionPolar(double a, double b, double theta, double T);

double CostFunctionDoublePolar(double theta1, double theta2, double theta3,
                               double T);

SolutionParameters DoublePolarToStandard(double theta1, double theta2,
                                         double theta3, double T);

double RunStage2();

bool GetSolutionNoStages(Eigen::Vector2d x0, Eigen::Vector2d v0,
                         Eigen::Vector2d xf, Eigen::Vector2d vf,
                         const double a_max, SolutionParameters* params);

bool TSOCSFinished(Eigen::Vector2d x0, Eigen::Vector2d v0, Eigen::Vector2d xf,
                   Eigen::Vector2d vf, const double a_max,
                   SolutionParameters* params);

void StandardToDoublePolar(SolutionParameters params, double* theta1,
                           double* theta2, double* theta3);

}  // namespace tsocs

#endif  // SRC_MOTION_CONTROL_TSOCS_OLD_H_
