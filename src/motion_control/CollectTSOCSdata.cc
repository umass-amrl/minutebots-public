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

#include <stdio.h>
#include <fstream>

#include <string>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "motion_control/tsocs_old.h"

int main(int argc, char **argv) {
  const char* read_from = argv[1];
  const char* write_to = argv[2];

  std::ofstream out;

  out.open(write_to);
  out << "success, T \n";

  std::ifstream read_file(read_from);
  std::string line;
  while (std::getline(read_file, line)) {
    std::istringstream s(line);
    string field;
    double params[5];
    int i = 0;
    while (getline(s, field, ',')) {
      params[i] = stod(field);
      ++i;
    }

    Eigen::Vector2d x0(0, 0);
    Eigen::Vector2d xf(params[0], 0);
    Eigen::Vector2d v0(params[1], params[2]);
    Eigen::Vector2d vf(params[3], params[4]);
    SolutionParameters tsocs_params;

    bool success = tsocs::GetSolution(x0, xf, v0, vf, 1., &tsocs_params);
    double time = tsocs_params.T;
    if (success) {
      out << "1, " << time << "\n";
    } else {
      out <<  "0, NA\n";
    }
  }
  out.close();
}
