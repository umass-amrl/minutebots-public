#!/bin/bash
# The individual test cases, hard coded and the scoring
full_score=0
count=0
for i in 0 800 1600 2400 3200 4000
  do
  for j in 2500 1500 500 -1500 -2500
    do
      for a in 0 72 144 216 288 360
        do
        ./bin/soccer -tb -dp -a$1 -v $2 -m $3 &
        bg_pid=$!
        ./bin/attacker_test $3 $2 $1 $i $j $a
        ret_value=$?
        echo RETURN VALUE: $ret_value
        if (( $ret_value != 2 ))
          then
            count=$[$count +1]
            full_score=$[$full_score + $ret_value]
          fi
        echo SCORE: $full_score
        echo COUNT: $count
        kill -9 $bg_pid
      done
    done
  done
#full_score=$(expr $temp \* 100)
echo $4, $full_score, $count