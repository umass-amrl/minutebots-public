#!/bin/bash

#The individual test cases, hard coded and the scoring
SCORE=0
./bin/soccer -tb -dp -a1.0,156,9,200,10.0,300,40 -v 10010  -m 10011 &
./bin/deflection_sim 0 0 0 1500 -1500 -1 .95 10011 10010
if [ $? = 0 ]; then
  SCORE=$((SCORE+1))
fi
pkill -kill soccer
./bin/soccer -tb -dp -a1.0,156,9,200,10.0,300,40 -v 10010  -m 10011 &
./bin/deflection_sim 0 0 0 2000 -1500 -0 1.1 10011 10010
if [ $? = 0 ]; then
  SCORE=$((SCORE+1))
fi
pkill -kill soccer
./bin/soccer -tb -dp -a1.0,156,9,200,10.0,300,40 -v 10010  -m 10011 &
./bin/deflection_sim 0 0 0 500 0 1 0 10011 10010
if [ $? = 0 ]; then
  SCORE=$((SCORE+1))
fi
pkill -kill soccer
./bin/soccer -tb -dp -a1.0,156,9,200,10.0,300,40 -v 10010  -m 10011 &
./bin/deflection_sim 0 0 0 1000 0 -1 0 10011 10010
if [ $? = 0 ]; then
  SCORE=$((SCORE+1))
fi
./bin/soccer -tb -dp -a1.0,156,9,200,10.0,300,40 -v 10010  -m 10011 &
./bin/deflection_sim 0 0 0 -500 500 1 .95 10011 10010
if [ $? = 0 ]; then
  SCORE=$((SCORE+1))
fi
pkill -kill soccer
./bin/soccer -tb -dp -a1.0,156,9,200,10.0,300,40 -v 10010  -m 10011 &
./bin/deflection_sim 0 0 0 -500 -500 1 -.95 10011 10010
if [ $? = 0 ]; then
  SCORE=$((SCORE+1))
fi
./bin/soccer -tb -dp -a1.0,156,9,200,10.0,300,40 -v 10010  -m 10011 &
./bin/deflection_sim 0 0 0 -500 500 1 .85 10011 10010
if [ $? = 0 ]; then
  SCORE=$((SCORE+1))
fi
pkill -kill soccer
./bin/soccer -tb -dp -a1.0,156,9,200,10.0,300,40 -v 10010  -m 10011 &
./bin/deflection_sim 0 0 0 -500 -500 1 -.85 10011 10010
if [ $? = 0 ]; then
  SCORE=$((SCORE+1))
fi
pkill -kill soccer
./bin/soccer -tb -dp -a1.0,156,9,200,10.0,300,40 -v 10010  -m 10011 &
./bin/deflection_sim 0 0 0 0 -1000 0 1 10011 10010
if [ $? = 0 ]; then
  SCORE=$((SCORE+1))
fi 
pkill -kill soccer
./bin/soccer -tb -dp -a1.0,156,9,200,10.0,300,40 -v 10010  -m 10011 &
./bin/deflection_sim 0 0 0 500 500 1 .95 10011 10010
if [ $? = 0 ]; then
  SCORE=$((SCORE+1))
fi
pkill -kill soccer
echo $1 $SCORE