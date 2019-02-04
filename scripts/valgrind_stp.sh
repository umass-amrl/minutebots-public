#!/bin/bash
(./bin/simulator &) && timeout --preserve-status --signal=SIGINT 5 valgrind --error-exitcode=1 --track-origins=yes --soname-synonyms='somalloc=*tcmalloc*' --suppressions=scripts/valgrind/unittests.supp --leak-check=full ./bin/soccer -tb -dp --ids=0,1,2,3,4,5 -py
RETURN_VAL=$?
killall simulator
exit $RETURN_VAL
