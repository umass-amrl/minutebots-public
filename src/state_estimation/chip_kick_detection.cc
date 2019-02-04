// Copyright 2017 srabiee@cs.umass.edu
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

#include "state_estimation/chip_kick_detection.h"
#include <glog/logging.h>
#include <algorithm>
#include <iomanip>

#include "state/team.h"
#include "state/world_robot.h"
#include "state/world_state.h"

#include "datastructures/bounded_queue.h"
#include "constants/constants.h"
#include "util/timer.h"
#include "math/math_util.h"


using std::endl;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Vector4f;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::MatrixXf;
using Eigen::MatrixXd;
using Eigen::Matrix4f;
using direction::Direction;

ChipKickDetection::ChipKickDetection() {}
ChipKickDetection::~ChipKickDetection() {}

int ChipKickDetection::Worl2Cam(const ChipKickTester::CameraParams& camera,
              const Vector3f& pos_world,
              Vector3f* pos_cam) {
  MatrixXf transform_mat(3, 4);
  transform_mat.block<3, 3>(0, 0) = camera.quat.toRotationMatrix();
  transform_mat.block<3, 1>(0, 3) = camera.translation;

  Vector4f homogeneous_coordinate_world;
  Vector4f homogeneous_coordinate_cam;
  homogeneous_coordinate_world << pos_world, 1.0f;

  homogeneous_coordinate_cam = transform_mat * homogeneous_coordinate_world;
  *pos_cam = homogeneous_coordinate_cam.head(3);
  return 0;
}

int ChipKickDetection::FitParabola(Vector3f* initial_pos,
                      Vector3f* initial_vel,
                      const double& time_init,
                      const vector<double>& time_input,
                      const vector<float>& ball_pos_x,
                      const vector<float>& ball_pos_y,
                      const vector<float>& ball_pos_z,
                      const vector<uint>& camera_id_input,
                      const vector<ChipKickTester::CameraParams>& cameras,
                      const Vector3f& g) {
  MatrixXd A(time_input.size() * 2, 6);
  VectorXd b(time_input.size() * 2);
  VectorXd A_current_row1(6);
  VectorXd A_current_row2(6);
  Vector3f ball_pos_cam;
  double alpha;
  double beta;
  double b_init1;
  double b_init2;

  // Gravity acceleration in the cameras' reference frame
  MatrixXd g_cam(4, 3);
  for (size_t j = 0; j < cameras.size(); j++) {
    g_cam.row(j) = cameras[j].quat.toRotationMatrix().cast<double>()
    * g.cast <double>();
  }

  for (size_t i = 0; i < time_input.size(); i++) {
    uint current_cam_id = camera_id_input[i];
    double delT = time_input[i] - time_init;
    Worl2Cam(cameras[current_cam_id],
              Vector3f(ball_pos_x[i], ball_pos_y[i], ball_pos_z[i]),
              &ball_pos_cam);
    alpha = static_cast<double>(ball_pos_cam(0))/
    static_cast<double>(ball_pos_cam(2));

    beta = static_cast<double>(ball_pos_cam(1))/
    static_cast<double>(ball_pos_cam(2));

    b_init1 = -(0.5 * g_cam(current_cam_id, 2) * alpha * delT * delT) +
              (0.5 * g_cam(current_cam_id, 0) * delT * delT);

    b_init2 = -(0.5 * g_cam(current_cam_id, 2) * beta * delT * delT) +
              (0.5 * g_cam(current_cam_id, 1) * delT * delT);

    A_current_row1 << -1 , 0, alpha, -delT, 0, alpha * delT;
    A_current_row2 << 0, -1, beta, 0, -delT, beta * delT;

//     Original linear system of equations:  A * X = b where,
//     X = [x0, y0, z0, vx, vy, vz]'
//     With a change of variables X' = R^(-1) * ( X - T ) => X = R X' + T
//     X' is the same as X but in the world coordinate system
//
//     A' = A * R
//     b' = b - A * T
    MatrixXd R(6, 6);
    R << cameras[current_cam_id].quat.toRotationMatrix().cast <double>(),
          MatrixXd::Zero(3, 3),
          MatrixXd::Zero(3, 3),
          cameras[current_cam_id].quat.toRotationMatrix().cast <double>();

    A.row(2 * i) = A_current_row1.transpose() * R;
    A.row(2 * i + 1) = A_current_row2.transpose() * R;

    b(2 * i) = b_init1 -
        A_current_row1.head(3).dot(cameras[current_cam_id].
        translation.cast <double>());
    b(2 * i + 1) = b_init2 -
        A_current_row2.head(3).dot(cameras[current_cam_id].
        translation.cast <double>());
  }
  // Find the least square solution for the system of equations
//   VectorXd initial_values = A.jacobiSvd(0x08 | 0x20).solve(b);
  //   VectorXd initial_values =
  //   A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b);
  VectorXd initial_values = (A.transpose() * A).ldlt().solve(A.transpose() * b);

  *initial_pos = initial_values.head(3).cast <float>();
  *initial_vel = initial_values.tail(3).cast <float>();

  return 0;
}

int ChipKickDetection::FitParabolaPartialOptimization(Vector3f* initial_vel,
                      double* time_init_optimized,
                      double* chip_rmse,
                      double* flat_kick_rmse,
                      const Vector3f& initial_pos,
                      const double& time_init,
                      const vector<double>& time_input,
                      const vector<float>& ball_pos_x,
                      const vector<float>& ball_pos_y,
                      const vector<float>& ball_pos_z,
                      const vector<uint>& camera_id_input,
                      const vector<ChipKickTester::CameraParams>& cameras,
                      const Vector3f& g) {
  MatrixXd A(time_input.size() * 2, 6);
  MatrixXd A_partial(time_input.size() * 2, 3);
  MatrixXd A_flat_kick(time_input.size() * 2, 2);
  VectorXd b(time_input.size() * 2);
  VectorXd b_partial(time_input.size() * 2);
  VectorXd b_flat_kick(time_input.size() * 2);
  Vector3d initial_pos_known;
  VectorXd A_current_row1(6);
  VectorXd A_current_row2(6);
  VectorXd A_flat_kick_current_row1(3);
  VectorXd A_flat_kick_current_row2(3);
  Vector3f ball_pos_cam;
  double alpha;
  double beta;
  double b_init1;
  double b_init2;
//   double time_init_corrected = time_init - 0.016 * 2.5;
  double time_init_corrected = time_init - 0.016 * 0.2;


  // Variables handling looping over multiple initial time values
  double RMSE_min = 1000000000.0;
  MatrixXd cov_mat_min;
  int time_query_num = 40;
  double time_search_interval = 0.016;

  initial_pos_known(0) = static_cast<double>(initial_pos(0));
  initial_pos_known(1) = static_cast<double>(initial_pos(1));
  initial_pos_known(2) = static_cast<double>(initial_pos(2));

  // Gravity acceleration in the cameras' reference frame
  MatrixXd g_cam(4, 3);
  for (size_t j = 0; j < cameras.size(); j++) {
    g_cam.row(j) = cameras[j].quat.toRotationMatrix().cast<double>()
    * g.cast <double>();
  }

  for (int j = 0; j < time_query_num; j++) {
    time_init_corrected += time_search_interval * (static_cast<double>(j) /
                          static_cast<double>(time_query_num));
//     std::cout << "Time Offset: " << time_search_interval *
//              (static_cast<double>(j) / static_cast<double>(time_query_num))
//               << endl;
    for (size_t i = 0; i < time_input.size(); i++) {
      uint current_cam_id = camera_id_input[i];
      double delT = time_input[i] - time_init_corrected;
      Worl2Cam(cameras[current_cam_id],
                Vector3f(ball_pos_x[i], ball_pos_y[i], ball_pos_z[i]),
                &ball_pos_cam);
      alpha = static_cast<double>(ball_pos_cam(0))/
      static_cast<double>(ball_pos_cam(2));

      beta = static_cast<double>(ball_pos_cam(1))/
      static_cast<double>(ball_pos_cam(2));

      b_init1 = -(0.5 * g_cam(current_cam_id, 2) * alpha * delT * delT) +
                (0.5 * g_cam(current_cam_id, 0) * delT * delT);

      b_init2 = -(0.5 * g_cam(current_cam_id, 2) * beta * delT * delT) +
                (0.5 * g_cam(current_cam_id, 1) * delT * delT);

      A_current_row1 << -1 , 0, alpha, -delT, 0, alpha * delT;
      A_current_row2 << 0, -1, beta, 0, -delT, beta * delT;

  //     Original linear system of equations:  A * X = b where,
  //     X = [x0, y0, z0, vx, vy, vz]'
  //     With a change of variables X' = R^(-1) * ( X - T ) => X = R X' + T
  //     X' is the same as X but in the world coordinate system
  //
  //     A' = A * R
  //     b' = b - A * T
      MatrixXd R(6, 6);
      R << cameras[current_cam_id].quat.toRotationMatrix().cast <double>(),
            MatrixXd::Zero(3, 3),
            MatrixXd::Zero(3, 3),
            cameras[current_cam_id].quat.toRotationMatrix().cast <double>();

      A.row(2 * i) = A_current_row1.transpose() * R;
      A.row(2 * i + 1) = A_current_row2.transpose() * R;

      b(2 * i) = b_init1 -
          A_current_row1.head(3).dot(cameras[current_cam_id].
          translation.cast <double>());
      b(2 * i + 1) = b_init2 -
          A_current_row2.head(3).dot(cameras[current_cam_id].
          translation.cast <double>());
    }

    // Change the problem to a partial optimization by assuming that the initial
    // position is known and reducing the number of unknowns to 3 (initial
    // velocity)
    A_partial = A.block(0, 3, 2 * time_input.size(), 3);
    b_partial = b - (A.block(0, 0, 2 * time_input.size(), 3) *
                initial_pos_known);

    // Find the least square solution for the system of equations
    //  VectorXd initial_values =
    //  A_partial.jacobiSvd(0x08 | 0x20).solve(b_partial);
    //   VectorXd initial_values =
    //   A_partial.jacobiSvd(ComputeThinU | ComputeThinV).solve(b_partial);
    VectorXd initial_values = (A_partial.transpose() *
              A_partial).ldlt().solve(A_partial.transpose() * b_partial);

    // Calculate the estimation RMSE
    VectorXd b_partial_estimated(time_input.size() * 2);
    VectorXd residual(time_input.size() * 2);
    b_partial_estimated = A_partial * initial_values;

    residual = b_partial - b_partial_estimated;
    int degree_of_freedom = residual.size() - initial_values.size();
    double residual_variance = residual.transpose().dot(residual) /
                            degree_of_freedom;
    MatrixXd cov_mat = residual_variance * (A_partial.transpose() *
          A_partial).inverse();

    double squared_err = 0.0;
    for (size_t k = 0; k < time_input.size() * 2; k++) {
      double error = b_partial(k) - b_partial_estimated(k);
      squared_err += error * error;
    }
    double mean_squared_err = squared_err /
                              static_cast<double>(time_input.size() * 2);
    double RMSE = sqrt(mean_squared_err);
//     std::cout << "RMSE value: " << RMSE << endl;

    if (j == 0) {
      RMSE_min = RMSE;
      cov_mat_min = cov_mat;
      *time_init_optimized = time_init_corrected;
      *initial_vel = initial_values.tail(3).cast <float>();
    }

    if (RMSE < RMSE_min) {
      RMSE_min = RMSE;
      cov_mat_min = cov_mat;
      *time_init_optimized = time_init_corrected;
      *initial_vel = initial_values.tail(3).cast <float>();
//       std::cout << "TMP Optimized Initial Time: " << time_init_corrected <<
//                   endl;
    }
  }

  // Fit a flat kick to the same data segment and  calculate the rmse for
  // comparison purposes
  for (size_t i = 0; i < time_input.size(); i++) {
    double delT = time_input[i] - *time_init_optimized;

    A_flat_kick_current_row1 << delT, 0;
    A_flat_kick_current_row2 << 0, delT;

    A_flat_kick.row(2 * i) = A_flat_kick_current_row1.transpose();
    A_flat_kick.row(2 * i + 1) = A_flat_kick_current_row2.transpose();

    b_flat_kick(2 * i) = ball_pos_x[i] - initial_pos.x();
    b_flat_kick(2 * i + 1) = ball_pos_y[i] - initial_pos.y();
  }

  VectorXd initial_values_flat_kick = (A_flat_kick.transpose() *
          A_flat_kick).ldlt().solve(A_flat_kick.transpose() * b_flat_kick);

  // Calculate the estimation RMSE for a hypothetical flat kick
  VectorXd b_flat_kick_estimated(time_input.size() * 2);
  VectorXd residual_flat_kick(time_input.size() * 2);
  b_flat_kick_estimated = A_flat_kick * initial_values_flat_kick;

  residual_flat_kick = b_flat_kick - b_flat_kick_estimated;
  double RMSE_flat_kick =
      sqrt(residual_flat_kick.transpose().dot(residual_flat_kick) /
                          static_cast<double>(time_input.size() * 2));

//  std::cout << "Input Initial Position: " << initial_pos.transpose() << endl;
//  std::cout << "Optimized Initial Time: " << std::setprecision(15)
//             << *time_init_optimized << endl;
//  std::cout << "Optimized Initial Vel_z: " << initial_vel->coeffRef(2) <<endl;
//  std::cout << "b_partial: ********" << "\n" << b_partial << endl << endl;
//  std::cout << "A_partial: ********" << "\n" << A_partial << endl << endl;
//  std::cout << "Initial_Values: ********" << "\n" << *initial_vel << endl <<
//               endl;
//  std::cout << "A: ********" << "\n" << A << endl << endl;
//   std::cout << "cov_mat_min: " << cov_mat_min << endl;

//   std::cout << "Min RMSE chip: " << RMSE_min << endl;
//   std::cout << "RMSE flat: " << RMSE_flat_kick << endl;

  *flat_kick_rmse = RMSE_flat_kick;
  *chip_rmse = RMSE_min;

  return 0;
}


int ChipKickDetection::PredictBouncing(const Vector3f& initial_pos,
     const Vector3f& initial_vel,
     const double& time_init,
     const float& ball_zero_height,
     const Vector3f& g,
     Vector3f* impact_point,
     double* impact_time) {
  // Solve a quadratic equation to find the impact point
  // 0.5 * gz * t^2 + v0z * t + z0 - ball_zero_height = 0
  // a t^2 + b t + c = 0

  double a = 0.5 * static_cast<double>(g(2));
  double b = static_cast<double>(initial_vel(2));
  double c = static_cast<double>(initial_pos(2)) -
             static_cast<double>(ball_zero_height);

  double delta = b * b - 4 * a * c;
  if (delta < 0) {
    // No real solution
    return 1;
  } else {
    *impact_time = (-b - sqrt(delta)) / (2 * a);
    double delT = *impact_time;
    *impact_time += time_init;

    impact_point->head(2) = initial_pos.head(2) + initial_vel.head(2) * delT;
    impact_point->coeffRef(2) = initial_pos(2) + initial_vel(2) * delT +
        0.5 * g(2) * delT * delT;
  }

  return 0;
}

int ChipKickDetection::Project2GroundPlane(
                                const ChipKickTester::CameraParams& camera,
                                const Vector3f& pos3d,
                                const float& ball_zero_height,
                                Vector2f* pos2d) {
  Vector3f camera_pos = -camera.quat.toRotationMatrix().inverse()
                        * camera.translation;
  Vector3f cam_to_ball_vec = pos3d - camera_pos;
  float cam_to_ball_vec_norm = sqrt(cam_to_ball_vec(0) * cam_to_ball_vec(0)
                                  + cam_to_ball_vec(1) * cam_to_ball_vec(1)
                                  + cam_to_ball_vec(2) * cam_to_ball_vec(2));
  Vector3f unit_vec = cam_to_ball_vec / cam_to_ball_vec_norm;
  float alpha = (ball_zero_height - pos3d(2)) / unit_vec(2);
  Vector3f projected_point = pos3d + unit_vec * alpha;
  *pos2d = projected_point.head(2);

  return 0;
}

// Visualize the results of the chip kick estimation in the viewer
logger::Logger ChipKickDetection::VisualizeResults(const Vector3f& ball_pos_3d,
                            const Vector3f& ball_impact_point,
                            const vector<float>& ball_raw_pos_x_queue,
                            const vector<float>& ball_raw_pos_y_queue,
                            const vector<float>& ball_raw_pos_x_offline_queue,
                            const vector<float>& ball_raw_pos_y_offline_queue,
                            const vector<float>& ball_pose2d_x_queue,
                            const vector<float>& ball_pose2d_y_queue,
                            const vector<uint>& camera_id_queue,
                            const vector<float>& ball_impact_point_x_queue,
                            const vector<float>& ball_impact_point_y_queue,
                            const vector<double>& ball_impact_time_queue,
                            const vector<float>& chip_kick_rmse_queue,
                            const vector<double>& ball_time_queue,
                            const vector<double>& computation_time_queue,
                            const vector<double>& computation_time_mean_queue,
                            const bool& detailed_visualization,
                            const direction::Direction& game_direction,
                            logger::Logger* the_log) {
  //     std::cout << ball_raw_pos << endl;
  //     std::cout << "Current position:" << endl;
  //     std::cout << current_pos << endl;
  //     std::cout << "Initial position:" << endl;
  //     std::cout << initial_pos.transpose() << endl;
  //     std::cout << "impact point: " << endl;
  //     std::cout << impact_point << endl;


//   std::cout << "Current 3d Position: " << endl;
//   std::cout << ball_pos_3d.transpose() << endl;
//   std::cout << "Impact Point: " << endl;
//   std::cout << ball_impact_point.transpose() << endl;


  // Send out data to the viewer
//   logger::NetLogger the_log(DATA_STREAM_DEBUG_IP, DATA_STREAM_DEBUG_PORT);

  if (detailed_visualization) {
    // Visualize the SSL_Vision detected ball trajectory
    for (size_t k = 0; k < ball_raw_pos_x_queue.size() - 1; k++) {
      if (camera_id_queue[k + 1] == 0 || camera_id_queue[k + 1] == 1) {
        the_log->AddLine(ball_raw_pos_x_queue[k],
                        ball_raw_pos_y_queue[k],
                        ball_raw_pos_x_queue[k + 1],
                        ball_raw_pos_y_queue[k + 1],
                        1,
                        0,
                        0,
                        1);
      } else {
        the_log->AddLine(ball_raw_pos_x_queue[k],
                        ball_raw_pos_y_queue[k],
                        ball_raw_pos_x_queue[k + 1],
                        ball_raw_pos_y_queue[k + 1],
                        0,
                        0,
                        1,
                        1);
      }

      the_log->AddBall(ball_raw_pos_x_queue[k], ball_raw_pos_y_queue[k]);
      the_log->AddBall(ball_raw_pos_x_queue[k + 1],
                      ball_raw_pos_y_queue[k + 1]);
    }

    // Visualize the estimated 3d position of the ball projected to the
    // ground plane
    for (size_t k = 0; k < ball_pose2d_x_queue.size() - 1; k++) {
      if (camera_id_queue[k + 1] == 0 || camera_id_queue[k + 1] == 1) {
        the_log->AddLine(ball_pose2d_x_queue[k],
                        ball_pose2d_y_queue[k],
                        ball_pose2d_x_queue[k + 1],
                        ball_pose2d_y_queue[k + 1],
                        0,
                        1,
                        1,
                        0.8);
      } else {
        the_log->AddLine(ball_pose2d_x_queue[k],
                        ball_pose2d_y_queue[k],
                        ball_pose2d_x_queue[k + 1],
                        ball_pose2d_y_queue[k + 1],
                        0,
                        1,
                        1,
                        0.8);
      }

      the_log->AddPoint(ball_pose2d_x_queue[k],
                      ball_pose2d_y_queue[k],
                      1,
                      1,
                      0,
                      1);
      the_log->AddPoint(ball_pose2d_x_queue[k + 1],
                      ball_pose2d_y_queue[k + 1],
                      1,
                      1,
                      0,
                      1);
    }

    // Visualize a few detected balls into the future
    uint future_horizon_max = 12;
    uint size_difference = ball_raw_pos_x_offline_queue.size() -
    ball_raw_pos_x_queue.size();
    uint future_horizon = min(future_horizon_max, size_difference);

    for (size_t k = ball_raw_pos_x_queue.size();
        k < ball_raw_pos_x_queue.size() + future_horizon - 1; k++) {
          the_log->AddLine(ball_raw_pos_x_offline_queue[k],
                ball_raw_pos_y_offline_queue[k],
                ball_raw_pos_x_offline_queue[k + 1],
                ball_raw_pos_y_offline_queue[k + 1],
                1,
                1,
                0,
                1);

          the_log->AddBall(ball_raw_pos_x_offline_queue[k],
                          ball_raw_pos_y_offline_queue[k]);
      }
  }

  // Visualize the predicted impact points of the ball
  if (detailed_visualization) {
    the_log->LogPrint("Impact Point_x, Impact Point_y, Impact time");
    the_log->Push();
  }

  for (size_t k = 0; k < ball_impact_point_x_queue.size(); k++) {
    if (game_direction == Direction::POSITIVE) {
      the_log->AddPoint(ball_impact_point_x_queue[k],
                      ball_impact_point_y_queue[k],
                      1,
                      0,
                      0,
                      1);
      // Draw circles as well as points for better visibility
      the_log->AddCircle(Vector2f(ball_impact_point_x_queue[k],
                                  ball_impact_point_y_queue[k]),
                          30,
                          1,
                          0,
                          0,
                          1);
    } else {
      the_log->AddPoint(-ball_impact_point_x_queue[k],
                      -ball_impact_point_y_queue[k],
                      1,
                      0,
                      0,
                      1);
      // Draw circles as well as points for better visibility
      the_log->AddCircle(Vector2f(-ball_impact_point_x_queue[k],
                                  -ball_impact_point_y_queue[k]),
                          30,
                          1,
                          0,
                          0,
                          1);
    }


    if (detailed_visualization) {
      the_log->LogPrint("%f, %f, %3.5f",
                      ball_impact_point_x_queue[k],
                      ball_impact_point_y_queue[k],
                      ball_impact_time_queue[k] - ball_time_queue[0]);
    }
  }

  if (detailed_visualization) {
    the_log->Pop();
  }

  if (detailed_visualization) {
  if (computation_time_queue.empty()) {
      the_log->LogPrint("Observation#, Time, RMSE");
      the_log->Push();
      int size_offset = ball_time_queue.size() - chip_kick_rmse_queue.size();
      for (size_t k = 0; k < chip_kick_rmse_queue.size(); k++) {
        the_log->LogPrint("%4d, %2.5fs, %fmm",
                        static_cast<int>(k + 1 + size_offset),
                        ball_time_queue[k + size_offset] - ball_time_queue[0],
                        chip_kick_rmse_queue[k]);
      }
      the_log->Pop();
    } else {
      the_log->LogPrint("Observation#, Time, RMSE, Comp_Time, Mean_Comp_Time");
      the_log->Push();
      int size_offset = ball_time_queue.size() - chip_kick_rmse_queue.size();
      for (size_t k = 0; k < chip_kick_rmse_queue.size(); k++) {
        the_log->LogPrint("%4d, %2.5fs, %fmm, %2.8fs, %2.8fs",
                        static_cast<int>(k + 1 + size_offset),
                        ball_time_queue[k + size_offset] - ball_time_queue[0],
                        chip_kick_rmse_queue[k], computation_time_queue[k],
                        computation_time_mean_queue[k]);
      }
      the_log->Pop();
    }
  }
  return *the_log;
}


int ChipKickDetection::VisualizeResultsCompare(const Vector3f& ball_pos_3d,
                        const Vector3f& ball_impact_point,
                        const vector<float>& ball_raw_pos_x_queue,
                        const vector<float>& ball_raw_pos_y_queue,
                        const vector<float>& ball_raw_pos_x_offline_queue,
                        const vector<float>& ball_raw_pos_y_offline_queue,
                        const vector<float>& ball_pose2d_x_queue,
                        const vector<float>& ball_pose2d_y_queue,
                        const vector<uint>& camera_id_queue,
                        const vector<float>& ball_impact_point_x_queue,
                        const vector<float>& ball_impact_point_y_queue,
                        const vector<double>& ball_impact_time_queue,
                        const vector<float>& ball_impact_point_x_queue_2nd,
                        const vector<float>& ball_impact_point_y_queue_2nd,
                        const vector<double>& ball_impact_time_queue_2nd,
                        const vector<float>& chip_kick_rmse_queue,
                        const vector<double>& ball_time_queue,
                        const vector<double>& computation_time_queue,
                        const vector<double>& computation_time_mean_queue,
                        logger::Logger* the_log) {
//   std::cout << "Current 3d Position: " << endl;
//   std::cout << ball_pos_3d.transpose() << endl;
//   std::cout << "Impact Point: " << endl;
//   std::cout << ball_impact_point.transpose() << endl;

  // Send out data to the viewer
//   logger::NetLogger the_log(DATA_STREAM_DEBUG_IP, DATA_STREAM_DEBUG_PORT);

  // Visualize the SSL_Vision detected ball trajectory
  for (size_t k = 0; k < ball_raw_pos_x_queue.size() - 1; k++) {
    if (camera_id_queue[k + 1] == 0 || camera_id_queue[k + 1] == 1) {
      the_log->AddLine(ball_raw_pos_x_queue[k],
                      ball_raw_pos_y_queue[k],
                      ball_raw_pos_x_queue[k + 1],
                      ball_raw_pos_y_queue[k + 1],
                      1,
                      0,
                      0,
                      1);
    } else {
      the_log->AddLine(ball_raw_pos_x_queue[k],
                      ball_raw_pos_y_queue[k],
                      ball_raw_pos_x_queue[k + 1],
                      ball_raw_pos_y_queue[k + 1],
                      0,
                      0,
                      1,
                      1);
    }

    the_log->AddBall(ball_raw_pos_x_queue[k], ball_raw_pos_y_queue[k]);
    the_log->AddBall(ball_raw_pos_x_queue[k + 1],
                    ball_raw_pos_y_queue[k + 1]);
  }

  // Visualize the estimated 3d position of the ball projected to the
  // ground plane
  for (size_t k = 0; k < ball_pose2d_x_queue.size() - 1; k++) {
    if (camera_id_queue[k + 1] == 0 || camera_id_queue[k + 1] == 1) {
      the_log->AddLine(ball_pose2d_x_queue[k],
                      ball_pose2d_y_queue[k],
                      ball_pose2d_x_queue[k + 1],
                      ball_pose2d_y_queue[k + 1],
                      0,
                      1,
                      1,
                      0.8);
    } else {
      the_log->AddLine(ball_pose2d_x_queue[k],
                      ball_pose2d_y_queue[k],
                      ball_pose2d_x_queue[k + 1],
                      ball_pose2d_y_queue[k + 1],
                      0,
                      1,
                      1,
                      0.8);
    }

    the_log->AddPoint(ball_pose2d_x_queue[k],
                     ball_pose2d_y_queue[k],
                     1,
                     1,
                     0,
                     1);
    the_log->AddPoint(ball_pose2d_x_queue[k + 1],
                     ball_pose2d_y_queue[k + 1],
                     1,
                     1,
                     0,
                     1);
  }

  // Visualize a few detected balls into the future
  uint future_horizon_max = 12;
  uint size_difference = ball_raw_pos_x_offline_queue.size() -
  ball_raw_pos_x_queue.size();
  uint future_horizon = min(future_horizon_max, size_difference);

  for (size_t k = ball_raw_pos_x_queue.size();
      k < ball_raw_pos_x_queue.size() + future_horizon - 1; k++) {
        the_log->AddLine(ball_raw_pos_x_offline_queue[k],
              ball_raw_pos_y_offline_queue[k],
              ball_raw_pos_x_offline_queue[k + 1],
              ball_raw_pos_y_offline_queue[k + 1],
              1,
              1,
              0,
              1);

        the_log->AddBall(ball_raw_pos_x_offline_queue[k],
                        ball_raw_pos_y_offline_queue[k]);
    }

  // Visualize the predicted impact points of the ball by both optimization
  // methods and connect the corresponding impact points
  the_log->LogPrint("Impact Point_x, Impact Point_y, Impact time");
  the_log->Push();
  for (size_t k = 0; k < ball_impact_point_x_queue.size(); k++) {
    float middle_point_x = (ball_impact_point_x_queue[k] +
                            ball_impact_point_x_queue_2nd[k]) / 2.0f;
    float middle_point_y = (ball_impact_point_y_queue[k] +
                            ball_impact_point_y_queue_2nd[k]) / 2.0f;

    the_log->AddLine(ball_impact_point_x_queue[k],
              ball_impact_point_y_queue[k],
              middle_point_x,
              middle_point_y,
              1,
              0,
              0,
              1);

    the_log->AddLine(middle_point_x,
          middle_point_y,
          ball_impact_point_x_queue_2nd[k],
          ball_impact_point_y_queue_2nd[k],
          0,
          0,
          1,
          1);

//     the_log.AddPoint(ball_impact_point_x_queue[k],
//                      ball_impact_point_y_queue[k],
//                      1,
//                      0,
//                      0,
//                      1);
//
//     the_log.AddPoint(ball_impact_point_x_queue_2nd[k],
//                      ball_impact_point_y_queue_2nd[k],
//                      0,
//                      0,
//                      1,
//                      1);

    the_log->LogPrint("%f, %f, %3.5f",
                     ball_impact_point_x_queue[k],
                     ball_impact_point_y_queue[k],
                     ball_impact_time_queue[k] - ball_time_queue[0]);
  }

  the_log->Pop();
  if (computation_time_queue.empty()) {
    the_log->LogPrint("Observation#, Time, RMSE");
    the_log->Push();
    int size_offset = ball_time_queue.size() - chip_kick_rmse_queue.size();
    for (size_t k = 0; k < chip_kick_rmse_queue.size(); k++) {
      the_log->LogPrint("%4d, %2.5fs, %fmm",
                      static_cast<int>(k + 1 + size_offset),
                      ball_time_queue[k + size_offset] - ball_time_queue[0],
                      chip_kick_rmse_queue[k]);
    }
  } else {
    the_log->LogPrint("Observation#, Time, RMSE, Comp_Time, Mean_Comp_Time");
    the_log->Push();
    int size_offset = ball_time_queue.size() - chip_kick_rmse_queue.size();
    for (size_t k = 0; k < chip_kick_rmse_queue.size(); k++) {
      the_log->LogPrint("%4d, %2.5fs, %fmm, %2.8fs, %2.8fs",
                      static_cast<int>(k + 1 + size_offset),
                      ball_time_queue[k + size_offset] - ball_time_queue[0],
                      chip_kick_rmse_queue[k], computation_time_queue[k],
                      computation_time_mean_queue[k]);
    }
  }

//   the_log.SendData();
  return 0;
}

// Calculates RMSE for chip kick estimation over the observed data from
// the start of the latest chip kick.
int ChipKickDetection::CalculateError(const vector<float>& ball_raw_pos_x_queue,
                          const vector<float>& ball_raw_pos_y_queue,
                          const vector<float>& ball_pose2d_x_queue,
                          const vector<float>& ball_pose2d_y_queue,
                          float* rmse) {
  float mean_squared_err = 0.0;

  int size_offset = ball_raw_pos_x_queue.size() - ball_pose2d_x_queue.size();
  for (size_t k = 0; k < ball_pose2d_x_queue.size(); k++) {
    float x_err = ball_raw_pos_x_queue[k + size_offset] -
                  ball_pose2d_x_queue[k];
    float y_err = ball_raw_pos_y_queue[k + size_offset] -
                  ball_pose2d_y_queue[k];
    mean_squared_err += x_err * x_err + y_err * y_err;
  }

  mean_squared_err = mean_squared_err / ball_pose2d_x_queue.size();
  *rmse = sqrt(mean_squared_err);

  return 0;
}
