#!/usr/bin/env python3
import sys
import numpy as np

f = open(sys.argv[1], "r")
lst = sorted([int(l.split(sys.argv[2])[1]) for l in f.readlines() if sys.argv[2] in l])
for l in lst:
    print(l)
print("Mean: {}".format(np.mean(lst)))
print("Min: {} Max: {}".format(min(lst), max(lst)))
