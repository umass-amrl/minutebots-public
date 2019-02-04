// Copyright 2017 dbalaban@cs.umass.edu
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

#ifndef SRC_MOTION_CONTROL_MOTION_CONTROL_STRUCTURES_H_
#define SRC_MOTION_CONTROL_MOTION_CONTROL_STRUCTURES_H_

// parameters describing the shape of a robot's trajectory
// and the total time of traversal
struct SolutionParameters {
  double a, b, c, d, T, cost;
  bool isInitialized;

  SolutionParameters() {isInitialized = false;}

  SolutionParameters(const double a, const double b,
                     const double c, const double d,
                     const double T) : a(a), b(b),
                     c(c), d(d), T(T) {
                       isInitialized = true;
                       cost = 0.0;
                    }

  SolutionParameters(const double a, const double b,
                     const double c, const double d,
                     const double T, const double cost) : a(a), b(b),
                     c(c), d(d), T(T), cost(cost) {
                       isInitialized = true;
                    }

  SolutionParameters(const SolutionParameters &sol) :
                     a(sol.a), b(sol.b), c(sol.c), d(sol.d),
                     T(sol.T), cost(sol.cost),
                     isInitialized(sol.isInitialized) {}


  SolutionParameters& operator= (const SolutionParameters &sol) {
    a = sol.a;
    b = sol.b;
    c = sol.c;
    d = sol.d;
    T = sol.T;
    cost = sol.cost;
    isInitialized = sol.isInitialized;

    return *this;
  }
};

struct SolutionParameters_alt {
  double theta_0, theta_T, r, T;
  bool isInitialized;

  SolutionParameters_alt() {isInitialized = false;}

  SolutionParameters_alt(const double theta_0, const double theta_T,
                     const double r,
                     const double T) : theta_0(theta_0), theta_T(theta_T),
                     r(r), T(T) {isInitialized = true;}

  SolutionParameters_alt(const SolutionParameters_alt &sol) :
                     theta_0(sol.theta_0), theta_T(sol.theta_T), r(sol.r),
                     T(sol.T), isInitialized(sol.isInitialized) {}


  SolutionParameters_alt& operator= (const SolutionParameters_alt &sol) {
    theta_0 = sol.theta_0;
    theta_T = sol.theta_T;
    r = sol.r;
    T = sol.T;
    isInitialized = sol.isInitialized;

    return *this;
  }
};

#endif  // SRC_MOTION_CONTROL_MOTION_CONTROL_STRUCTURES_H_
