# SRTR

SMT-based Robot Transition Repair (SRTR), is a method for reparing robot state machine behavior transitions using
human provided corrections. Instead of manually adjusting parameters SRTR allows a user to identify instances of failing transitions and the desired alternative transition, and then formulats a MaxSMT problem from the program trace that is solved with Z3 to yield correcting parameter adjustments. This repository contains libraries which can be used to write state machines that record the execution traces necessary for repair, and the functions for reading and repairing from execution traces with corrections.

Authors: Jarrett Holtz (jaholtz@cs.umass.edu), Arjun Guha, Joydeep Biswas

### COMPILATION:

If dependences are not installed run the InstallPackages script, otherwise simply run make.
The make install command will place the libraries and h files into folders in the install and include paths respectively.

```bash
./InstallPackages

make
sudo make install
```
### USAGE:
To repair a state machine with SRTR an execution trace of the behavior containing one or more human provided corrections is necessary. The *srtr.h* file contains a small set of functions for reading these traces from Google Protobuffs in a file, and repairing using traces with corrections. The *state_machine.h* file contains the StateMachine and RepairableParam classes that can be used to write state machines which will automatically record the necessary execution traces. The process for using SRTR is as follows: 

1. Implement a behavior in the StateMachine class with parameters to adjust written as RepairableParams utilized within the `Transition()` function. 
2. Run the behavior, which will record an execution trace as part of `Transition()`. This will yield an execution trace as a series of protobuffs written to a file, and should include failing transitions  to repair.
3. Provide corrections. The PossibleTransition protobuffs contains a field for signaling if an element of the trace is a human correction. In order to provide corrections, the human_constraint field of the transition to adjust should be set to true, and the should_transition bool should be set to true if this is a transition that should have happened, and false otherwise. This library does not provide a standardized interface for providing corrections beyond the necessary protobuffs, as we do not visualize or replay the traces for arbitrary robots. 
4. Given a trace which contains human constraints, this can be repaired using the `void TuneFromTraceFile(const string& machine_name)` to read in the trace and solve, or by providing the necessary information from the trace to `SolveWithBlocks(...)` without using a file. 

### Publications
Interactive Robot Transition Repair with SMT
Jarrett Holtz, Arjun Guha, and Joydeep Biswas. 
To appear in Proceedings of IJCAI-ECAI-2018, the International Joint Conference on Artificial Intelligence
