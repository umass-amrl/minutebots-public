#!/bin/bash
# The individual test cases, hard coded and the scoring
SCORE=0
SCORE1=0
SCORE2=0
SCORE3=0
SCORE4=0
SCORE5=0
SCORE6=0
SCORE7=0
SCORE8=0
SCORE9=0
SCORE10=0
SCORE11=0
SCORE12=0
SCORE13=0

./bin/soccer -tb -dp -a$1 -v $2 -m $3 &
bg_pid=$!
./bin/deflection_sim 0 0 0 1500 -1500 -1 .95 $3 $2
if [ $? = 0 ]; then
  SCORE=$((SCORE+1))
  SCORE1=1
fi
kill -9 $bg_pid
./bin/soccer -tb -dp -a$1 -v $2 -m $3 &
bg_pid=$!
./bin/deflection_sim 0 0 0 2000 -1500 -0 1.1 $3 $2
if [ $? = 0 ]; then
  SCORE=$((SCORE+1))
  SCORE2=1
fi
kill -9 $bg_pid
./bin/soccer -tb -dp -a$1 -v $2 -m $3 &
bg_pid=$!
./bin/deflection_sim 0 0 0 500 0 1 0 $3 $2
if [ $? = 0 ]; then
  SCORE=$((SCORE+1))
  SCORE3=1
fi
kill -9 $bg_pid
./bin/soccer -tb -dp -a$1 -v $2 -m $3 &
bg_pid=$!
./bin/deflection_sim 0 0 0 1000 0 -1 0 $3 $2
if [ $? = 0 ]; then
  SCORE=$((SCORE+1))
  SCORE4=1
fi
kill -9 $bg_pid
./bin/soccer -tb -dp -a$1 -v $2 -m $3 &
bg_pid=$!
./bin/deflection_sim 0 0 0 -480 500 1 .95 $3 $2
if [ $? = 0 ]; then
  SCORE=$((SCORE+1))
  SCORE5=1
fi
kill -9 $bg_pid
./bin/soccer -tb -dp -a$1 -v $2 -m $3 &
bg_pid=$!
./bin/deflection_sim 0 0 0 -480 -500 1 -.95 $3 $2
if [ $? = 0 ]; then
  SCORE=$((SCORE+1))
  SCORE6=1
fi
kill -9 $bg_pid
./bin/soccer -tb -dp -a$1 -v $2 -m $3 &
bg_pid=$!
./bin/deflection_sim 0 0 0 0 -1000 0 1 $3 $2
if [ $? = 0 ]; then
  SCORE=$((SCORE+1))
  SCORE7=1
fi 
kill -9 $bg_pid
./bin/soccer -tb -dp -a$1 -v $2 -m $3 &
bg_pid=$!
./bin/deflection_sim 0 0 0 500 500 1 .95 $3 $2
if [ $? = 0 ]; then
  SCORE=$((SCORE+1))
  SCORE8=1
fi
kill -9 $bg_pid
# Stopped ball cases
./bin/soccer -tb -dp -a$1 -v $2 -m $3 &
bg_pid=$!
./bin/deflection_sim 0 0 0 -480 500 0 0 $3 $2
if [ $? = 0 ]; then
  SCORE=$((SCORE+1))
  SCORE9=1
fi
kill -9 $bg_pid
./bin/soccer -tb -dp -a$1 -v $2 -m $3 &
bg_pid=$!
./bin/deflection_sim 0 0 0 -480 -500 0 0 $3 $2
if [ $? = 0 ]; then
  SCORE=$((SCORE+1))
  SCORE10=1
fi
kill -9 $bg_pid
./bin/soccer -tb -dp -a$1 -v $2 -m $3 &
bg_pid=$!
./bin/deflection_sim 0 0 0 1500 -1500 0 0 $3 $2
if [ $? = 0 ]; then
  SCORE=$((SCORE+1))
  SCORE11=1
fi
kill -9 $bg_pid
./bin/soccer -tb -dp -a$1 -v $2 -m $3 &
bg_pid=$!
./bin/deflection_sim 0 0 0 2000 1500 -0 0 $3 $2
if [ $? = 0 ]; then
  SCORE=$((SCORE+1))
  SCORE12=1
fi
kill -9 $bg_pid
./bin/soccer -tb -dp -a$1 -v $2 -m $3 &
bg_pid=$!
./bin/deflection_sim 0 0 0 0 500 -0 0 $3 $2
if [ $? = 0 ]; then
  SCORE=$((SCORE+1))
  SCORE13=1
fi
# End Stopped ball cases
kill -9 $bg_pid
echo $1, $SCORE,$SCORE1,$SCORE2,$SCORE3,$SCORE4,$SCORE5,$SCORE6,$SCORE7,$SCORE8,$SCORE9,$SCORE10,$SCORE11,$SCORE12,$SCORE13