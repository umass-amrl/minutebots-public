// Copyright 2018 jaholtz@cs.umass.edu
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

#include <functional>
#include <memory>
#include <string>
#include <vector>

// #include "constants/constants.h"
// #include "math/math_util.h"
// #include "math/poses_2d.h"
// #include "state/shared_state.h"
// #include "state/world_state.h"
// #include "tactics/tactic_index.h"
// #include "tactics/tactic.h"
//
// #include "util/random.h"
// #include "logging/logger.h"

#include "tuning_data.pb.h"
using std::string;

#ifndef SRC_SRTR_STATE_MACHINE_H_
#define SRC_SRTR_STATE_MACHINE_H_

namespace srtr {

class StateMachine{
 public:
  explicit StateMachine(const string& machine_name);
  // Overrides the Tactic Run method for some additional functionality.
  void Run();
  // Does preliminary setup of a Protobuff component used for storing
  // transition function execution traces.
  void SetupMessage();
  // Should handle state transitions, must be implemented for each
  // state machine.
  virtual void Transition() = 0;

  // RepairableParam Class is used to handle the logging of transition function
  // execution traces. Parameters which can be repaired with SRTR should be
  // stored as RepairableParams.
  class RepairableParam {
   public:
    RepairableParam(const float& value,
                    const string& name,
                    StateMachine* parent);
    // Trace Logging happens in the comparison operators.
    bool operator>(const float& x);
    bool operator<(const float& x);
    explicit operator float();
    // Allows setting of the repaired params actual value.
    void SetValue(const float& x);

    string name_;

   private:
    float value_;
    StateMachine* parent_;
  };

  // State class represents an individual state for a StateMachine
  // States will be members of classes which inherit from StateMachineTactic.
  class State {
    // std::function pointer type
    using FuncType = std::function<void()>;

   public:
    // Default Constructor (no associated function, named Start)
    State() : name_("Start") {}
    // Constructor which generates a useable StateType
    // function Should be generated with
    // std::bind(STATEMACHINE MEMBER FUNCTION, INSTANCE OF STATE MACHINE)
    // name is a unique identifier for the state.
    State(FuncType function, string name) : name_(name),
                                            state_function_(function) {}
    // Runs the stored function when the State is called.
    void operator()() { state_function_(); }
    // Compare two States using their unique names
    bool operator== (const State& state) {
      return name_.compare(state.name_) == 0;
    }
    bool operator!= (const State& state) {
      return name_.compare(state.name_) != 0;
    }
    string name_;

   private:
    // This is the function that corresponds to the state behavior
    FuncType state_function_;
  };

 protected:
  MinuteBotsProto::StateMachineData log_message_;
  string machine_name_;
  State state_;
  string potential_state_;
  bool and_clause_;
  bool and_block_;
  int block_index_;
  bool should_transition_;
  // Adds a transition block for the current start->potential pair
  // bool is_and : designates whether this block should be &&(true)
  // with the block before it, or ||(false).
  void AddBlock(const bool& is_and);
  // Sets the transition in the execution trace.
  void SetTransition(const bool& should_transition);
};
// Overloaded comparisons to float for RepairableParams.
bool operator>(const float& x, StateMachine::RepairableParam y);

bool operator<(const float& x, StateMachine::RepairableParam y);

}  // namespace srtr

#endif  // SRC_SRTR_STATE_MACHINE_H_
