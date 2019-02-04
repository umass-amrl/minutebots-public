#!/bin/bash
echo "Debug gcc full stack"
(./bin/simulator &) && timeout --preserve-status --signal=SIGINT 5 valgrind --error-exitcode=1 --track-origins=yes --soname-synonyms='somalloc=*tcmalloc*' --suppressions=scripts/valgrind/unittests.supp --leak-check=full ./bin/soccer -dn --tactic 1,fb
RETURN_VAL=$?
(killall simulator || true)
exit $RETURN_VAL
