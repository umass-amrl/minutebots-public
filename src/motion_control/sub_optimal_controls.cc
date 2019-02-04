// Copyright 2017 - 2018 dbalaban@cs.umass.edu
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

#include "motion_control/sub_optimal_controls.h"
#include "math/math_util.h"
#include "motion_control/motion_model.h"

using ntoc::ControlPhase1D;
using ntoc::ControlSequence1D;
using ntoc::ControlPhase2D;
using ntoc::ControlSequence2D;
using motion::MotionModel;
using math_util::Sign;

double GetMinimumUpperTimeBound(Eigen::Vector2d x0, Eigen::Vector2d v0,
                                Eigen::Vector2d xf, Eigen::Vector2d vf,
                                const double a_max, const double v_max) {
  const double kThreshold = 1e-7;
  const double t_method1 = ThreeStageLinear(x0, v0, xf, vf, a_max, v_max);
  std::vector<double> times;
  times.push_back(t_method1);
  if (v0.norm() > kThreshold) {
    if (vf.norm() > kThreshold) {
      RotateTowardsGoal(x0, v0, xf, vf, a_max, v_max, &times);
    }
    RotateTowardsRest(x0, v0, xf, vf, a_max, v_max, &times);
  }
  if (vf.norm() < kThreshold) {
    MotionModel motion_model(a_max, v_max);
    ControlSequence2D control;
    const double t_ntoc =
        NTOC2D(x0.cast<float>(), v0.cast<float>(), motion_model, &control);
    times.push_back(t_ntoc);
  }
  double t_min = times.at(0);
  for (unsigned int i = 1; i < times.size(); i++) {
    const double t_tmp = times.at(i);
    if (t_min > t_tmp) {
      t_min = t_tmp;
    }
  }
  return t_min;
}

double ThreeStageLinear(Eigen::Vector2d x0, Eigen::Vector2d v0,
                        Eigen::Vector2d xf, Eigen::Vector2d vf,
                        const double a_max, double v_max) {
  x0 = x0 / a_max;
  v0 = v0 / a_max;
  xf = xf / a_max;
  vf = vf / a_max;
  v_max = v_max / a_max;

  // time to accelerate to rest
  const double t0 = v0.norm();
  // position after accelerating to rest, first rest point
  const Eigen::Vector2d x_rest1 = v0 * v0.norm() / 2 + x0;
  // position to accelerate to goal from
  const Eigen::Vector2d x_rest2 = xf - .5 * vf * vf.norm();

  // time of direct acceleration to end velocity
  const double T_l2 = vf.norm();

  // dist between two rest points
  const double x_tilda = (x_rest2 - x_rest1).norm();
  double T1 = 2 * sqrt(x_tilda);  // travel time between rest points
  // total time for 1D problems and acceleration to rest
  const double t_max = T_l2 + T1 + t0;
  return t_max;
}

bool RotateTowardsGoal(Eigen::Vector2d x0, Eigen::Vector2d v0,
                       Eigen::Vector2d xf, Eigen::Vector2d vf,
                       const double a_max, double v_max,
                       std::vector<double>* times) {
  x0 = x0 / a_max;
  v0 = v0 / a_max;
  xf = xf / a_max;
  vf = vf / a_max;
  v_max = v_max / a_max;

  xf = xf - x0;
  x0 -= x0;

  const Eigen::Vector2d v0_norm = v0.normalized();
  const Eigen::Vector2d vf_norm = vf.normalized();

  const double cos_angle_diff =
      cos(atan2(v0.y(), v0.x()) - atan2(vf.y(), vf.x()));
  const Eigen::Vector2d displ = xf - x0;

  const double r1 = (displ.x() - displ.y()) / (cos_angle_diff + 1);
  if (r1 >= v0.norm() * v0.norm()) {
    Eigen::Vector2d circle_center;
    circle_center << r1 * v0_norm.x(), -r1 * v0_norm.y();
    Eigen::Vector2d x1;
    x1 << r1 * (v0_norm.y() + vf_norm.y()), -r1 * (v0_norm.x() + vf_norm.x());
    const double ang_vel =
        (circle_center.y() * v0.x() - circle_center.x() * v0.y()) / (r1 * r1);
    double angle0 = atan2(-circle_center.y(), -circle_center.x());
    if (angle0 < 0) {
      angle0 += 2 * M_PI;
    }
    double anglef =
        atan2(x1.y() - circle_center.y(), x1.x() - circle_center.x());
    if (anglef < 0) {
      anglef += 2 * M_PI;
    }
    double angle_diff = anglef - angle0;
    if (Sign(angle_diff) != Sign(ang_vel)) {
      angle_diff = 2 * M_PI - fabs(angle_diff);
    }
    anglef = angle0 + angle_diff * Sign(ang_vel * angle_diff);
    Eigen::Vector2d v1;
    v1 << -v0.norm() * sin(anglef) * Sign(ang_vel),
        v0.norm() * cos(anglef) * Sign(ang_vel);
    const double t0 = fabs(angle_diff / ang_vel);

    const Eigen::Vector2d displ2 = xf - x1;
    ControlSequence1D control;
    const double v1_1d = v1.norm() * displ2.normalized().dot(v1.normalized());
    const double vf_1d = vf.norm() * displ2.normalized().dot(vf_norm);
    const double t1 = TimeOptimalControlAnyFinal1D(
        0, v1_1d, displ2.norm(), vf_1d, 0, 1, v_max, &control);
    times->push_back(t1 + t0);
  }

  const double r2 = (displ.x() - displ.y()) / (-cos_angle_diff + 1);
  if (r2 >= v0.norm() * v0.norm()) {
    Eigen::Vector2d circle_center;
    circle_center << -r1 * v0_norm.x(), r1 * v0_norm.y();
    Eigen::Vector2d x1;
    x1 << r2 * (-v0_norm.y() + vf_norm.y()), r2 * (-v0_norm.x() + vf_norm.x());
    const double ang_vel =
        (circle_center.y() * v0.x() - circle_center.x() * v0.y()) / (r1 * r1);
    double angle0 = atan2(-circle_center.y(), -circle_center.x());
    if (angle0 < 0) {
      angle0 += 2 * M_PI;
    }
    double anglef =
        atan2(x1.y() - circle_center.y(), x1.x() - circle_center.x());
    if (anglef < 0) {
      anglef += 2 * M_PI;
    }
    double angle_diff = anglef - angle0;
    if (Sign(angle_diff) != Sign(ang_vel)) {
      angle_diff = 2 * M_PI - fabs(angle_diff);
    }
    anglef = angle0 + angle_diff * Sign(ang_vel * angle_diff);
    Eigen::Vector2d v1;
    v1 << -v0.norm() * sin(anglef) * Sign(ang_vel),
        v0.norm() * cos(anglef) * Sign(ang_vel);
    const double t0 = fabs(angle_diff / ang_vel);

    const Eigen::Vector2d displ2 = xf - x1;
    ControlSequence1D control;
    const double v1_1d = v1.norm() * displ2.normalized().dot(v1.normalized());
    const double vf_1d = vf.norm() * displ2.normalized().dot(vf_norm);
    const double t1 = TimeOptimalControlAnyFinal1D(
        0, v1_1d, displ2.norm(), vf_1d, 0, 1, v_max, &control);
    times->push_back(t1 + t0);
  }

  if (times->size() > 0) {
    return true;
  }
  return false;
}

double GetTravelTime(Eigen::Vector2d r, Eigen::Vector2d v0, Eigen::Vector2d vf,
                     Eigen::Vector2d x_rest, const double tan_angle,
                     const double ang_vel, const double v_max) {
  const Eigen::Vector2d rest_to_center = r - x_rest;
  const double dist_to_tangent =
      sqrt(rest_to_center.norm() * rest_to_center.norm() + r.norm() * r.norm());

  const Eigen::Vector2d tangent_vect(cos(tan_angle), sin(tan_angle));
  const Eigen::Vector2d tangent_point = x_rest + dist_to_tangent * tangent_vect;

  double angle0 = atan2(-r.y(), -r.x());
  if (angle0 < 0) {
    angle0 += 2 * M_PI;
  }
  const Eigen::Vector2d center_to_tan = tangent_point - r;
  double angle_from_center = atan2(center_to_tan.y(), center_to_tan.x());
  if (angle_from_center < 0) {
    angle_from_center += 2 * M_PI;
  }
  double angle_diff = angle_from_center - angle0;
  if (Sign(angle_diff) != Sign(ang_vel)) {
    angle_diff = 2 * M_PI - fabs(angle_diff);
  }
  const double t0 = fabs(angle_diff / ang_vel);
  const Eigen::Vector2d v1_at_tangent(
      -v0.norm() * sin(angle_from_center) * Sign(ang_vel),
      v0.norm() * cos(angle_from_center) * Sign(ang_vel));
  const Eigen::Vector2d displ_from_tangent = x_rest - tangent_point;
  const double v1_tangent_1d =
      v1_at_tangent.norm() *
      displ_from_tangent.normalized().dot(v1_at_tangent.normalized());
  ControlSequence1D control;
  const double t1 = TimeOptimalControlAnyFinal1D(
      0, v1_tangent_1d, displ_from_tangent.norm(), 0, 0, 1, v_max, &control);
  return t0 + t1 + vf.norm();
}

bool RotateTowardsRest(Eigen::Vector2d x0, Eigen::Vector2d v0,
                       Eigen::Vector2d xf, Eigen::Vector2d vf,
                       const double a_max, double v_max,
                       std::vector<double>* times) {
  // const double kThreshold = 1e-7;
  x0 = x0 / a_max;
  v0 = v0 / a_max;
  xf = xf / a_max;
  vf = vf / a_max;
  v_max = v_max / a_max;

  xf = xf - x0;
  x0 -= x0;

  const Eigen::Vector2d v0_norm = v0.normalized();
  // position of rest point
  const Eigen::Vector2d x_rest = xf - .5 * vf * vf.norm();
  const double radius_mag = v0.norm() * v0.norm();
  const Eigen::Vector2d r1(radius_mag * -v0_norm.y(), radius_mag * v0_norm.x());
  const Eigen::Vector2d r2(radius_mag * v0_norm.y(), -radius_mag * v0_norm.x());

  if ((x_rest - r1).norm() > radius_mag) {
    const double ang_vel =
        (r1.y() * v0.x() - r1.x() * v0.y()) / (r1.norm() * r1.norm());
    const Eigen::Vector2d rest_to_center = r1 - x_rest;
    const double angle_to_center =
        atan2(rest_to_center.y(), rest_to_center.x());
    const double angle_tanget_plus =
        angle_to_center + asin(radius_mag / rest_to_center.norm());
    const double angle_tanget_minus =
        angle_to_center - asin(radius_mag / rest_to_center.norm());

    const double t1 =
        GetTravelTime(r1, v0, vf, x_rest, angle_tanget_plus, ang_vel, v_max);
    const double t2 =
        GetTravelTime(r1, v0, vf, x_rest, angle_tanget_minus, ang_vel, v_max);

    times->push_back(t1);
    times->push_back(t2);
  }

  if ((x_rest - r2).norm() > radius_mag) {
    const double ang_vel =
        (r2.y() * v0.x() - r2.x() * v0.y()) / (r2.norm() * r2.norm());
    const Eigen::Vector2d rest_to_center = r2 - x_rest;
    const double angle_to_center =
        atan2(rest_to_center.y(), rest_to_center.x());
    const double angle_tanget_plus =
        angle_to_center + asin(radius_mag / rest_to_center.norm());
    const double angle_tanget_minus =
        angle_to_center - asin(radius_mag / rest_to_center.norm());

    const double t1 =
        GetTravelTime(r2, v0, vf, x_rest, angle_tanget_plus, ang_vel, v_max);
    const double t2 =
        GetTravelTime(r2, v0, vf, x_rest, angle_tanget_minus, ang_vel, v_max);

    times->push_back(t1);
    times->push_back(t2);
  }
  if (times->size() > 0) {
    return true;
  }
  return false;
}
