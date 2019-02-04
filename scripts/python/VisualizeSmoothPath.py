#!/usr/bin/python3

import math
import numpy as np
import matplotlib.pyplot as plt
import sys

f = open(sys.argv[1], 'r')

xs = []
ys = []
circs = []

xline = []
yline = []

for line in f:
    ind = line.find(' ')
    x = float(line[0:ind])
    y = float(line[ind+1:])
    xs = xs + [x]
    ys = ys + [y]

for i in range(0, len(xs) - 1):
    xline = xline + [xs[i],xs[i+1]]
    yline = yline + [ys[i],ys[i+1]]

plt.plot(xline, yline, 'k-', lw=2)
plt.plot(xs, ys, 'ro')

f = open(sys.argv[2], 'r')

for line in f:
    ind = line.find(' ')
    x = float(line[0:ind])
    line = line[ind+1:]
    ind = line.find(' ')
    y = float(line[0:ind])
    r = float(line[ind+1:])
    print("X: " + str(x) + " Y: " + str(y) + " R: " + str(r))
    circs = circs + [plt.Circle((x,y), radius=r, color='r', fill=True)]



fig1 = plt.figure(1)

ax_plt1 = fig1.add_subplot(111)
for c in circs:
    ax_plt1.add_patch(c)

# for c in circles:
#     ax_plt1.add_patch(c[0])
#     plt.plot([c[1][0], c[2][0]], [c[1][1], c[2][1]], 'k-', lw=2)

#plt.subplot(311)
#plt.plot(xs, ys, 'ro')
# ax_plt1.set_ylabel('q_1')
# ax_plt1.set_xlabel('Radians of center circle')

plt.show()
