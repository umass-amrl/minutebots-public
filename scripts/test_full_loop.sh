#!/bin/bash
(./bin/simulator &) && timeout --preserve-status --signal=SIGINT 5 ./bin/soccer -dn -i=0,1,2,3,4,5 --tactic 1,fb
RETURN_VAL=$?
killall simulator
echo -e "\033[0;36mReturn value: $RETURN_VAL \033[0;0m"
echo -e "\033[0;36mIf the above return value is zero, full loop passes \033[0;32m:)\033[0;36m, otherwise, it does not \033[0;31m:'(\033[0;0m"
exit $RETURN_VAL
