// Copyright 2017-2018 kvedder@umass.edu
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
#include "navigation/navigation_util.h"

#include "constants/constants.h"
#include "third_party/googletest/googletest-release-1.8.0/googletest/include/gtest/gtest.h"

STANDARD_USINGS;
using logger::Logger;

namespace navigation {

// TEST(NavigationUtils, ProjectToSafety) {
//   const Vector2f start_position(field_dimensions::kHalfFieldLength + 300, 0);
//   const Vector2f end_position(0, 0);
//   const float margin = 50;
//   Logger logger;
//   const Vector2f safe =
//       ProjectToSafety(start_position, end_position, margin, &logger);
//   LOG(INFO) << "X: " << safe.x();
//   EXPECT_LE(safe.x() - (field_dimensions::kHalfFieldLength + margin), 0.01);
// }

TEST(NavigationUtils, ProjectToSafetyNoCollide) {
  const Vector2f start_position(300, 0);
  const Vector2f end_position(0, 0);
  const float margin = 50;
  Logger logger;
  const Vector2f safe =
      ProjectToSafety(start_position, end_position, margin, &logger);
  EXPECT_EQ(start_position, safe);
}

}  // namespace navigation
