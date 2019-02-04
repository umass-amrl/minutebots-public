#!/bin/bash

#The individual test cases, hard coded and the scoring
SCORE=0
./bin/soccer -tb -py &
./bin/deflection_sim -4500 375 0 -1000 400 -3 0 10007 10006
pkill -kill soccer

./bin/soccer -tb -py &
./bin/deflection_sim -4500 375 0 -1000 0 -3 0 10007 10006
pkill -kill soccer

./bin/soccer -tb -py &
./bin/deflection_sim -4500 375 0 -1000 400 -5 0 10007 10006
pkill -kill soccer

./bin/soccer -tb -py &
./bin/deflection_sim -4500 375 0 -1000 400 -4 0 10007 10006
pkill -kill soccer

./bin/soccer -tb -py &
./bin/deflection_sim -4500 375 0 -1000 0 -4 0 10007 10006
pkill -kill soccer

./bin/soccer -tb -py &
./bin/deflection_sim -4500 375 0 -1000 -225 -3 0 10007 10006
pkill -kill soccer

./bin/soccer -tb -py &
./bin/deflection_sim -4500 375 0 -1000 -100 -3 0 10007 10006
pkill -kill soccer

./bin/soccer -tb -py &
./bin/deflection_sim -4500 375 0 -1000 300 -2.5 0 10007 10006
pkill -kill soccer


./bin/soccer -tb -py &
./bin/deflection_sim -4500 375 0 -1000 500 -2.5 -.5 10007 10006
pkill -kill soccer

./bin/soccer -tb -py &
./bin/deflection_sim -4500 375 0 -1000 300 -2.5 -.5 10007 10006
pkill -kill soccer

./bin/soccer -tb -py &
./bin/deflection_sim -4500 375 0 -1000 0 -2.5 0 10007 10006
pkill -kill soccer


./bin/soccer -tb -py &
./bin/deflection_sim -4500 375 0 -1000 0 -2.5 -.25 10007 10006
pkill -kill soccer

./bin/soccer -tb -py &
./bin/deflection_sim -4500 375 0 -1000 0 -2.5 .25 10007 10006
pkill -kill soccer

./bin/soccer -tb -py &
./bin/deflection_sim -4500 375 0 -1000 -300 -2.75 .5 10007 10006
pkill -kill soccer

./bin/soccer -tb -py &
./bin/deflection_sim -4500 375 0 -1000 -500 -3.0 .5 10007 10006
pkill -kill soccer

./bin/soccer -tb -py &
./bin/deflection_sim -4500 375 0 -1000 -500 -3.0 .25 10007 10006
pkill -kill soccer