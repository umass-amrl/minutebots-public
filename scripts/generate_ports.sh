#!/bin/bash
for a in $(seq 10032 2 14032) 
do
	b=$(( a + 1 ))
	echo $a $b >> ports.txt
done