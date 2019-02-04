// Copyright 2017 slane@cs.umass.edu
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

#ifndef SRC_ALGORITHMS_BRANCH_AND_BOUND_H_
#define SRC_ALGORITHMS_BRANCH_AND_BOUND_H_

#include <queue>
#include <vector>
#include <iostream>

using std::priority_queue;
using std::vector;
using std::cout;
using std::endl;

namespace algorithms {

// PartialAssignment type must supply a default constructor that produces the
// empty assignment
template <typename Cost, typename Heuristic, typename PartialAssignment>
class BranchAndBound {
 public:
  BranchAndBound() {
  }
  bool Search(Cost* optimal_cost,
              PartialAssignment* optimal_assignment) {
    priority_queue<PriorityQueueVertex,
                   vector<PriorityQueueVertex>> pq;

    PartialAssignment empty_assignment;
    Heuristic empty_heuristic =
    CalculateHeuristic(empty_assignment);
    pq.push(PriorityQueueVertex(empty_heuristic,
                                empty_assignment));

    Cost upper_bound = CalculateUpperBound(empty_assignment);
    *optimal_cost = CalculateUpperBound(empty_assignment);


    bool found_solution = false;

    while (!pq.empty()) {
      const PartialAssignment candidate = pq.top().assignment;
      pq.pop();

      Cost lower_bound = CalculateLowerBound(candidate);

      // If lower bound is greater than our current upper bound (optimal_cost)
      // skip the rest of the calculation for this branch and continue the loop
      if (lower_bound > upper_bound) {
        continue;
      }

      Cost candidate_cost;
      bool is_solution = Optimize(candidate, &candidate_cost);

      if (is_solution) {
        // If the partial assignment is a solution, check and update our
        // candidate solution
        found_solution = true;

        if (candidate_cost < *optimal_cost) {
          *optimal_assignment = candidate;
          *optimal_cost = candidate_cost;
        }
        if (candidate_cost < upper_bound) {
          upper_bound = candidate_cost;
        }
      } else {
        // This is not yet a solution, update the upper bound and then branch
        Cost candidate_upper_bound = CalculateUpperBound(candidate);
        if (candidate_upper_bound < upper_bound) {
          upper_bound = candidate_upper_bound;
        }

        std::vector<PartialAssignment> branches;
        Branch(candidate, &branches);

        for (const auto& branch : branches) {
          Cost branch_heuristic = CalculateHeuristic(branch);
          pq.push(PriorityQueueVertex(branch_heuristic, branch));
        }
      }
    }

    return found_solution;
  }

 protected:
  virtual Heuristic CalculateHeuristic(const PartialAssignment& assignment) = 0;
  virtual Cost CalculateLowerBound(const PartialAssignment& assignment) = 0;
  virtual Cost CalculateUpperBound(const PartialAssignment& assignment) = 0;
  virtual bool Optimize(const PartialAssignment& assignment,
                        Cost* optimal_cost) = 0;
  virtual void Branch(const PartialAssignment& assignment,
                      std::vector<PartialAssignment>* branches) = 0;

 private:
  struct PriorityQueueVertex {
    Heuristic heuristic_value;
    PartialAssignment assignment;
    PriorityQueueVertex(Heuristic heuristic_value,
                        const PartialAssignment& assignment)
        : heuristic_value(heuristic_value),
          assignment(assignment) {}
    ~PriorityQueueVertex() {}

    bool operator<(const PriorityQueueVertex& other) const {
      return assignment > other.assignment;
    }

    bool operator==(const PriorityQueueVertex& other) const {
      return assignment == other.assignment;
    }
  };

  bool CompareVertices(PriorityQueueVertex vertex_a,
                       PriorityQueueVertex vertex_b) {
    return vertex_a.heuristic_value > vertex_b.heuristic_value;
  }

  std::priority_queue<PriorityQueueVertex,
                      std::vector<PriorityQueueVertex>> priority_queue_;
};
}  // namespace algorithms
#endif  // SRC_ALGORITHMS_BRANCH_AND_BOUND_H_
