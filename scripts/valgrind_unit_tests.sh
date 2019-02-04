#!/bin/bash
valgrind --error-exitcode=1 --track-origins=yes --soname-synonyms='somalloc=*tcmalloc*' --suppressions=scripts/valgrind/unittests.supp --leak-check=full ./bin/run_unit_tests
