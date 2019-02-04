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

if len(sys.argv) == 3:
    robot_index = int(sys.argv[2])
else:
    robot_index = ''

with open(sys.argv[1], 'r') as f:
    content = f.readlines()
unfiltered_content = [x.strip() for x in content if 'Make safe for index' in x]
content = [x.strip() for x in content if 'Make safe for index' in x
           and 'index {}'.format(robot_index) in x]

unfiltered_times = [float(x[x.find("time:") + 6:]) for x in unfiltered_content]
times = [float(x[x.find("time:") + 6:]) for x in content]

# the histogram of the data
n, bins, patches = plt.hist(times, 1000, edgecolor="red", color="orange", log=True)

fig = plt.gcf()
l = plt.plot(bins)

plt.xlabel('$Time\ taken\ in\ seconds$')
plt.ylabel('$\log_{10}(Number\ of\ occurrences)$')
if robot_index is '':
    plt.title("Histograms of DSS Runtimes")
else:
    plt.title("Histograms of DSS Runtimes for robot {}".format(robot_index))
plt.axis([0, max(unfiltered_times) * 1.01, 0.9, 3000])
plt.grid(True)
print("Saving image. Please wait.")
fig.set_size_inches(45, 12, forward=False)
plt.savefig('dss_plot_{}.png'.format(str(robot_index) if robot_index is not '' else 'all_bots'),
            dpi=300)
print("Image save done!")
#plt.show()
