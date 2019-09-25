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

#include "configuration_reader/reader.h"
#include "eigen3/Eigen/Dense"
#include "logging/logger.h"
#include "motion_control/motion_control_structures.h"
#include "motion_control/optimal_control_1d.h"
#include "motion_control/sub_optimal_controls.h"
#include "motion_control/tsocs_templates.h"
#include "shared/common_includes.h"

#ifndef SRC_MOTION_CONTROL_TSOCS_H_
#define SRC_MOTION_CONTROL_TSOCS_H_

namespace tsocs {

extern const double kTSOCSThreshold;
extern const double kMaxRegularization;
using Eigen::Vector2d;

class Tsocs {
 public:
  Tsocs(Vector2d x0, Vector2d v0, Vector2d xf, Vector2d vf, double a_max);
  Tsocs();

  // guesses a solution parameter set
  SolutionParameters GetGuess();

  bool GetSolutionSetFromGuess(std::vector<SolutionParameters>* params,
                               SolutionParameters guess);

  // finds a set of parameters which were used to find optimal solution
  // given starting / ending position and velocity
  // bounded acceleration, unbounded velocity
  // first params are initial guess including time upper bound
  // second params are result of stage 1
  // third params are result of stage 2 - final params
  // returns true if correct solution was found
  // perturbations stores how many perturbations it took to solve it
  // if perturbations is null it doesn't store how many it took
  bool GetSolutionSet(std::vector<SolutionParameters>* params,
                      int* perturbations = NULL);

  // runs stage 2 starting from the given parameters
  // returns true if it succeeded
  bool GetSolutionNoStages(SolutionParameters* params);

  // finds a single parameter set
  // returns true if correct solution was found
  bool GetSolution(SolutionParameters* params, logger::Logger* the_log = NULL);

  // finds the expected position and velocity after time t
  // given starting position and velocity and an optimal solution
  // bounded acceleration, unbounded velocity
  void GetState(Eigen::Vector2d* xt, Eigen::Vector2d* vt, const double t,
                SolutionParameters params);

  Eigen::Vector2d GetAccelVector(const double t,
                      SolutionParameters params);

  std::vector<Eigen::Vector2f> GetPath(const unsigned int npts,
                                       SolutionParameters params);

  // Returns the cost of the given problem with the given parameters
  double CostFunction(double a, double b, double c, double d, double T);

  double RunStage2(SolutionParameters* params);
  double RunStage1(SolutionParameters* params);

  bool Finished(SolutionParameters params);

 protected:
  double x_cost_coef = 1.0, v_cost_coef = 1.0;
  // all of these vectors are divided by a_max
  Vector2d x0, v0, xf, vf, delta_pose, delta_vel;

  double a_max;
  double t_reg_ratio;
  double k1;
  double k2;
  double t_expected;
  bool use_t_regularization = false;
  struct Xdist {
    Tsocs* tsocs;
    explicit Xdist(Tsocs* tsocs) : tsocs(tsocs) {}

    template <typename T>
    bool operator()(const T* const a, const T* const b, const T* const c,
                    const T* const d, const T* const t, T* residual) const {
      if (*t < T(0)) {
        return false;
      }

      T x1 = X(*a, *b, *c, *d, *t) + T(tsocs->v0.x()) * t[0];
      T x2 = X(*b, *a, *d, *c, *t) + T(tsocs->v0.y()) * t[0];

      if (ceres::IsNaN(x1) || ceres::IsNaN(x2)) {
        printf("In Xdist:\n");
        cout << "x1 = " << x1 << endl;
        cout << "x2 = " << x2 << endl;
        cout << "a: " << *a << endl;
        cout << "b: " << *b << endl;
        cout << "c: " << *c << endl;
        cout << "d: " << *d << endl;
        cout << "T: " << *t << endl;
      }

      residual[0] =
          T(sqrt(tsocs->x_cost_coef)) * (x1 - T(tsocs->delta_pose.x()));
      residual[1] =
          T(sqrt(tsocs->x_cost_coef)) * (x2 - T(tsocs->delta_pose.y()));

      return true;
    }
  };

  struct Vdist {
    Tsocs* tsocs;
    explicit Vdist(Tsocs* tsocs) : tsocs(tsocs) {}
    template <typename T>
    bool operator()(const T* const a, const T* const b, const T* const c,
                    const T* const d, const T* const t, T* residual) const {
      if (*t < T(0)) {
        return false;
      }

      T v1 = V(*a, *b, *c, *d, *t);
      T v2 = V(*b, *a, *d, *c, *t);

      if (ceres::IsNaN(v1) || ceres::IsNaN(v2)) {
        printf("In Vdist:\n");
        cout << "v1 = " << v1 << endl;
        cout << "v2 = " << v2 << endl;
        cout << "a: " << *a << endl;
        cout << "b: " << *b << endl;
        cout << "c: " << *c << endl;
        cout << "d: " << *d << endl;
        cout << "T: " << *t << endl;
      }

      residual[0] =
          T(sqrt(tsocs->v_cost_coef)) * (v1 - T(tsocs->delta_vel.x()));
      residual[1] =
          T(sqrt(tsocs->v_cost_coef)) * (v2 - T(tsocs->delta_vel.y()));

      return true;
    }
  };

  struct Reg_T {
    Tsocs* tsocs;
    explicit Reg_T(Tsocs* tsocs) : tsocs(tsocs) {}
    template <typename T>
    bool operator()(const T* const t, T* residual) const {
      residual[0] =
          tsocs->k1 *
          ceres::exp(T(tsocs->k2) * (*t / T(tsocs->t_expected + kEpsilon)
          - tsocs->t_reg_ratio));
      residual[0] = min(residual[0], T(kMaxRegularization));
      return true;
    }
  };

  SolutionParameters GetNonProjectionGuess();
  SolutionParameters GetProjectionGuess();

  void UpdateParameters(SolutionParameters* params);
  double GetTimeBound();
};

}  // namespace tsocs

#endif  // SRC_MOTION_CONTROL_TSOCS_H_
