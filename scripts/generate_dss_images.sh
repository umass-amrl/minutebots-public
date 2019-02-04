#!/bin/bash

echo "Starting"

for i in `seq 0 5`;
do
    echo $i
    echo $1
    ./scripts/python/HistogramDSS.py $1 $i
done
./scripts/python/HistogramDSS.py $1
