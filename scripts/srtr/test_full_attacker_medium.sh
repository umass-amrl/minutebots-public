#!/bin/bash
# The individual test cases, hard coded and the scoring
full_score=0
for (( i=0; i<=4000; i+=400 ))
  do
    for (( j=2500; j>=-2500; j-=500 ))
    do
      for (( a=0; a<=360; a+=36 ))
        do
          ./bin/soccer -tb -dp -v $1 -m $2 &
          bg_pid=$!
          ./bin/attacker_test $2 $1 $i $j $a
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
  temp=$(expr $full_score / $count)
  full_score=$(expr $temp \* 100)
  echo $4, $full_score