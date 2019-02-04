// Copyright 2016-2017 joydeepb@cs.umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
//========================================================================
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
//========================================================================

#include <signal.h>
#include <stdio.h>

#include "util/helpers.h"
#include "util/timer.h"

// Switch to gracefully terminate the program on signals.
bool run_ = true;

// Cumulative timer for SlowFunction().
CumulativeFunctionTimer slow_function_timer_("SlowFunction");

void SigIntHandler(int) {
  run_ = false;
  // Send a new line to separate the "^C" on stdout.
  printf("\n");
}

void SlowFuntion() {
  CumulativeFunctionTimer::Invocation invoke(&slow_function_timer_);
  Sleep(0.0001);
}

void TestBackTrace(int n) {
  if (n == 0) {
    PrintStackTrace();
  } else {
    TestBackTrace(n - 1);
  }
}

int main(int num_arguments, char** arguments) {
  signal(SIGINT, SigIntHandler);

  TestBackTrace(10);
  fflush(stdout);
  while (run_) {
    SlowFuntion();
    Sleep(1.0 / 60.0);
  }

  return 0;
}
