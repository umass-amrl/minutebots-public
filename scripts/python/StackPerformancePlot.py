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

with open(sys.argv[1], 'r') as f:
    content = f.readlines()

stp_content = [x.strip() for x in content if 'STP TIME' in x]
tactics_content = [x.strip() for x in content if 'TACTICS TIME' in x]
stox_content = [x.strip() for x in content if 'STOX TIME' in x]
ntoc_content = [x.strip() for x in content if 'NTOC TIME' in x]
dss_content = [x.strip() for x in content if 'DSS TIME' in x]
remerge_content = [x.strip() for x in content if 'REGMERGE TIME' in x]
postmerge_content = [x.strip() for x in content if 'POSTMERGE TIME' in x]
total_content = [x.strip() for x in content if 'TOTAL TIME' in x]

stp_times = [ float(line[line.find('TIME:') + 6:]) for line in stp_content]
tactics_times = [ float(line[line.find('TIME:') + 6:]) for line in tactics_content]
stox_times = [ float(line[line.find('TIME:') + 6:]) for line in stox_content]
ntoc_times = [ float(line[line.find('TIME:') + 6:]) for line in ntoc_content]
dss_times = [ float(line[line.find('TIME:') + 6:]) for line in dss_content]
remerge_times = [ float(line[line.find('TIME:') + 6:]) for line in remerge_content]
postmerge_times = [ float(line[line.find('TIME:') + 6:]) for line in postmerge_content]
total_times = [ float(line[line.find('TIME:') + 6:]) for line in total_content]


data_list = [(stp_times, "stp_times.png", "STP") ,(tactics_times, "tactics_times.png", "Tactics"), (stox_times, "stox_times.png", "Individual STOx"),(ntoc_times, "ntoc_times.png", "Individual NTOC"), (dss_times, "dss_times.png", "DSS"), (remerge_times, "merge_times.png", "Merge"), (postmerge_times, "postmerge_times.png", "Postmerge"), (total_times, "total_times.png", "Total")]


def draw(data_tuple):
    print(data_tuple[2])
    plt.cla()
    plt.clf()
    n, bins, patches = plt.hist(data_tuple[0], 1000, edgecolor="red", color="orange", log=True)

    fig = plt.gcf()
    l = plt.plot(bins)

    plt.xlabel('$Time\ taken\ in\ seconds$')
    plt.ylabel('$\log_{10}(Number\ of\ occurrences)$')
    plt.title("Histograms of {} Runtimes".format(data_tuple[2]))
    plt.axis([0, max(data_tuple[0]) * 1.01, 0.9, 3000])
    plt.grid(True)
    print("Saving image. Please wait.")
    fig.set_size_inches(45, 12, forward=False)
    plt.savefig(data_tuple[1], dpi=300)
    print("Image save done!")
    # plt.show()

for data_tuple in data_list:
    draw(data_tuple)
