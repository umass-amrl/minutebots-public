// Copyright 2019 jaholtz@cs.umass.edu
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

#include <exception>
#include <iostream>
#include <string>

#ifndef SRC_DEBUGGING_TACTIC_EXCEPTION_H_
#define SRC_DEBUGGING_TACTIC_EXCEPTION_H_

class TacticException : public std::exception {
 public:
    explicit TacticException(const std::string& message)
      : message_(message) {}

    const char* what() const throw() {
        return message_.c_str();
    }
 private:
  std::string message_;
};

#endif  // SRC_DEBUGGING_TACTIC_EXCEPTION_H_"

