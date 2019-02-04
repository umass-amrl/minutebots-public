// Copyright 2018 afischer@umass.edu
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

// template functions for TSOCS/ball interception

#include <stdio.h>
#include <cmath>
#include <iostream>
#include <sstream>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "shared/common_includes.h"

#ifndef SRC_MOTION_CONTROL_TSOCS_TEMPLATES_H_
#define SRC_MOTION_CONTROL_TSOCS_TEMPLATES_H_

using std::isnan;
using ntoc::ControlSequence1D;
using math_util::AngleMod;

static const double kEpsilonSq = kEpsilon * kEpsilon;

template <typename T>
inline T F1(const T a, const T b) {
  return (sqrt(a * a + b * b));
}

template <typename T>
T F2(const T a, const T b, const T c, const T d) {
  return a * c + b * d;
}

template <typename T>
T F3(const T a, const T b, const T c, const T d) {
  return b * c - a * d;
}

template <typename T>
T G1(const T f1, const T f2, const T f4, const T t) {
  T g1 = sqrt(t * t * f1 * f1 + T(2) * t * f2 + f4 * f4);
  return g1;
}

template <typename T>
T LogRatio(const T g1, const T f1, const T f2, const T f4) {
  T f5 = f1 * f4 + f2;
  T g2 = f1 * g1 + f1 * f1 + f2;
  T ratio = (g2 + T(kEpsilonSq)) / (f5 + T(kEpsilonSq));
  return ratio;
}

template <typename T>
T TimeDependentLogRatio(const T g1, const T f1, const T f2, const T f4,
                        const T t) {
  T f5 = f1 * f4 + f2;
  T g2 = f1 * g1 + t * f1 * f1 + f2;
  T ratio = (g2 + T(kEpsilonSq)) / (f5 + T(kEpsilonSq));
  return ratio;
}

template <typename T, const int N>
T PowN(const T& x) {
  T result = T(1.0);
  for (int i = 0; i < N; ++i) {
    result = result * x;
  }
  return result;
}

template <typename T>
T X(const T a, const T b, const T c, const T d, const T t) {
  const T f1 = F1(a, b);
  const T f2 = F2(a, b, c, d);
  const T f3 = F3(a, b, c, d);
  const T f4 = F1(c, d);
  const T g1 = G1(f1, f2, f4, t);
  const T ln = TimeDependentLogRatio(g1, f1, f2, f4, t);
  if (ceres::IsNaN(f1)) {
    cout << "f1 = " << f1 << endl;
  }
  if (ceres::IsNaN(f2)) {
    cout << "f2 = " << f2 << endl;
  }
  if (ceres::IsNaN(f3)) {
    cout << "f3 = " << f3 << endl;
  }
  if (ceres::IsNaN(f4)) {
    cout << "f4 = " << f4 << endl;
  }
  if (ceres::IsNaN(g1)) {
    cout << "g1 = " << g1 << endl;
  }
  if (ceres::IsNaN(ln)) {
    cout << "ln = " << ln << endl;
  }

  const T apart = a * (g1 * (f1 * f2 + t * PowN<T, 3>(f1)) + f3 * f3 * log(ln) -
                       f4 * (f1 * f2 + T(2) * t * PowN<T, 3>(f1))) /
                  (T(2) * PowN<T, 5>(f1));
  const T bpart =
      b * f3 * (log(ln) * (f1 * f1 * t + f2) - f1 * (g1 - f4)) / PowN<T, 5>(f1);

  const T sum = apart + bpart;
  if (ceres::IsNaN(apart)) {
    printf("apart = ");
    cout << apart << endl;
  }
  if (ceres::IsNaN(bpart)) {
    printf("bpart = ");
    cout << bpart << endl;
  }
  if (ceres::IsNaN(sum)) {
    printf("sum is nan in X\n");
    cout << "a: " << a << endl;
    cout << "b: " << b << endl;
    cout << "c: " << c << endl;
    cout << "d: " << d << endl;
    cout << "T: " << t << endl;
  }
  return sum;
}

template <typename T>
T V(const T a, const T b, const T c, const T d, const T t) {
  const T f1 = F1(a, b);
  const T f2 = F2(a, b, c, d);
  const T f3 = F3(a, b, c, d);
  const T f4 = F1(c, d);
  const T g1 = G1(f1, f2, f4, t);
  const T ln = TimeDependentLogRatio(g1, f1, f2, f4, t);
  return a * (g1 - f4) / (f1 * f1) + b * f3 * log(ln) / (f1 * f1 * f1);
}

template <typename T>
T X_end(const T a, const T b, const T c, const T d, const T t) {
  const T f1 = F1(a, b);
  const T f2 = F2(a, b, c, d);
  const T f3 = F3(a, b, c, d);
  const T f4 = F1(c, d);
  const T g1 = sqrt(f1 * f1 + T(2) * f2 + f4 * f4);
  const T ln = LogRatio(g1, f1, f2, f4);

  T sum;
  const T a_part = a * (g1 * (f1 * f2 + PowN<T, 3>(f1)) + f3 * f3 * log(ln) -
                        f4 * (f1 * f2 + T(2) * PowN<T, 3>(f1))) /
                   (T(2) * PowN<T, 5>(f1));

  const T b_part =
      b * f3 * (log(ln) * (f1 * f1 + f2) - f1 * (g1 - f4)) / PowN<T, 5>(f1);

  sum = t * t * (a_part + b_part);
  return sum;
}

#endif  // SRC_MOTION_CONTROL_TSOCS_TEMPLATES_H_
