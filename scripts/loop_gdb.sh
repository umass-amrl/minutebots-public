#/bin/bash
gdb --batch --command=scripts/test.gdb --args ./bin/soccer -tb -dp --ids=0,1,2,3,4,5 --tactic 1,fb
while [ -n "$?" ]; do
  gdb --batch --command=scripts/test.gdb --args ./bin/soccer -tb -dp --ids=0,1,2,3,4,5 --tactic 1,fb
  sleep 0.01
done
