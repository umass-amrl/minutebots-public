#!/bin/bash

#The individual test cases, hard coded and the scoring
SCORE=0

for i in {1..500}
do
  ballx=$(awk -v "seed=$[(RANDOM & 32767) + 32768 * (RANDOM & 32767)]" \
  'BEGIN { srand(seed); test= (0 + rand() * 1) * 2000.0; printf(test) }')
  bally=$(awk -v "seed=$[(RANDOM & 32767) + 32768 * (RANDOM & 32767)]" \
  'BEGIN { srand(seed); test= (-1 + rand() * 2) * 2000.0; printf(test) }')
  velx=$(awk -v "seed=$[(RANDOM & 32767) + 32768 * (RANDOM & 32767)]" \
  'BEGIN { srand(seed); test= (0 + rand() * 1) * -2.0; printf(test) }')
  vely=$(awk -v "seed=$[(RANDOM & 32767) + 32768 * (RANDOM & 32767)]" \
  'BEGIN { srand(seed); test= (-1 + rand() * 2) * 2.0; printf(test) }')

echo $ballx
echo $bally
echo $velx
echo $vely
echo " "

./bin/soccer -tb -dp -a10.0,120,1,300,30.0,300,40 -v 10006  -m 10007 &
./bin/deflection_sim 0 0 0 $ballx $bally $velx $vely 10007 10006
pkill -kill soccer
done