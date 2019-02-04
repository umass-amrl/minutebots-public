#! /usr/bin/env python3
import math
import numpy as np
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
import matplotlib
import sys

font = {'family' : 'cmr10',
        'weight' : 'bold',
        'size'   : 50}

matplotlib.rc('font', **font)

def flatten(lstlst):
    flatlst = []
    for lst in lstlst:
        for e in lst:
            flatlst.append(e)
    return flatlst

content = []
title = sys.argv[1]
for file_name in sys.argv[2:]:
    with open(file_name, 'r') as f:
        lines = f.readlines()
        if len(lines) > 0:
            content.append(lines)

def merge_times(content):
    assert(len(content) > 0)
    times = [float(n) for n in content[0]]
    for ts in content[1:]:
        times = [float(tsum) + float(t) for tsum, t in zip(times, ts)]
    return times

def max_times(content):
    assert(len(content) > 0)
    times = [float(n) for n in content[0]]
    for ts in content[1:]:
        times = [max(float(tcurr), float(t)) for tcurr, t in zip(times, ts)]
    return times

sum_times = merge_times(content)
max_times = max_times(content)
x = sum_times

n_bins = 200
fig, ax = plt.subplots(figsize=(8, 4))

# plot the cumulative histogram
n, bins, patches = ax.hist(x, n_bins, histtype='step', normed=True,
                           cumulative=True)

# tidy up the figure
ax.grid(True)
ax.set_title(title)
ax.set_xlabel('Time (millis)')
ax.set_ylabel('Likelihood of occurrence')
ax.set_ylim(0, 1)

plt.show()


# plt.subplot(111)

# # the histogram of the data
# n, bins, patches = plt.hist(sum_times,
#                             1000, edgecolor="red", color="orange",
#                             histtype='step', cumulative=True)
# fig = plt.gcf()
# l = plt.plot(bins)

# plt.xlabel('$Time\ taken\ in\ millisecconds$')
# plt.ylabel('$Occurences)$')

# plt.title("Histograms of the sum of the runtimes of {} robots.".format(len(content)))

# #plt.axis([0, max(max(sum_times), max(max_times)) * 1.01, 0.9, 3000])
# plt.grid(True)
# plt.show()

# # plt.subplot(212)
# # # the histogram of the data
# # n, bins, patches = plt.hist(max_times, 1000, edgecolor="green", color="blue", log=True)
# # fig = plt.gcf()
# # l = plt.plot(bins)

# # plt.xlabel('$Time\ taken\ in\ millisecconds$')
# # plt.ylabel('$\log_{10}(Number\ of\ occurrences)$')

# # plt.title("Histograms of the max of the runtimes of {} robots.".format(len(content)))

# # plt.axis([0, max(max(sum_times), max(max_times)) * 1.01, 0.9, 3000])
# # plt.grid(True)
# print("Saving image. Please wait.")
# fig.set_size_inches(10, 10, forward=False)
# plt.savefig('eight_grid_times_plot.png',
#             dpi=100)
# print("Image save done!")
# #plt.show()
