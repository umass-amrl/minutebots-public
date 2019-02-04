// Copyright 2016 - 2017 kvedder@umass.edu
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
#include <glog/logging.h>

#include "third_party/googletest/googletest-release-1.8.0/googletest/include/gtest/gtest.h"

TEST(ExampleTestCase, GTestIsWorking) {
  EXPECT_EQ(2, 2);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = 0;  // INFO level logging.
    FLAGS_colorlogtostderr = 1;  // Colored logging.
    FLAGS_logtostderr = true;  // Don't log to disk.

    LOG(INFO) << "Logging is setup!";

    int returnValue;

    // Do whatever setup here you will need for your tests here
    //
    //

    returnValue =  RUN_ALL_TESTS();

    // Do Your teardown here if required
    //
    //

    return returnValue;
}
