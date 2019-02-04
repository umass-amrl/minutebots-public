#!/usr/bin/env python3
import os
import sys
import glob
import shutil
import numpy as np
import matplotlib.pyplot as plt

print(os.getcwd())

for filename in glob.glob(os.path.join("build/", '*.py')):
    shutil.copy(filename, 'scripts/python/')

import graph_pb2
import path_pb2
import point_obstacle_pb2
import multigraph_pb2
import pathable_multigraph_pb2
import obstacle_pb2
from google.protobuf import text_format

if len(sys.argv) < 2:
    print("Usage: <proto folder>")
    exit(-1)

files = sorted([os.path.join(sys.argv[1], f) for f in os.listdir(sys.argv[1])])
path_files = [f for f in files if "path" in f]
point_files = [f for f in files if "point" in f]

print("Path files:", path_files)
print("Point files:", point_files)

smooth_xs = []
smooth_ys = []

path_points = []
circles = []

fig = plt.gcf()
ax = fig.gca()

def get_color(idx):
    colors = ['#ff3300', '#33cc33', '#0066ff', '#ffff66', '#9966ff']
    return colors[idx % len(colors)]

for idx, path_file in enumerate(path_files):
    f = open(path_file, 'rb')
    message = text_format.Parse(f.read(), path_pb2.Path())
    xs = []
    ys = []
    for p in message.path_waypoint_list:
        xs.append(float(p.x))
        ys.append(float(p.y))
        cr = plt.Circle((p.x, p.y), 15, color=get_color(idx), alpha=0.5)
        ax.add_artist(cr)
    plt.plot(xs, ys, color=get_color(idx))

for idx, point_file in enumerate(point_files):
    f = open(point_file, 'rb')
    message = text_format.Parse(f.read(), point_obstacle_pb2.PointObstacle())
    xs = []
    ys = []
    for p in message.point_list:
        xs.append(float(p.x))
        ys.append(float(p.y))
        print("x:", p.x, "y:", p.y)
        cr = plt.Circle((p.x, p.y), 10, color=get_color(idx), alpha=0.2)
        ax.add_artist(cr)

plt.xlim([-4500,4500])
plt.ylim([-3000,3000])
plt.axes().set_aspect('equal', 'datalim')
plt.tight_layout()
# fig.set_size_inches(30, 20)
# fig.savefig('locked.png')
plt.show()

