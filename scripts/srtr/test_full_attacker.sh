#!/bin/bash
# The individual test cases, hard coded and the scoring


score=$(./bin/attacker_test $3 $2 $1)
ret_value=$?

echo $4, $ret_value