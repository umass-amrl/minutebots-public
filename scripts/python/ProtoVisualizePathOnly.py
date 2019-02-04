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
import multigraph_pb2
import pathable_multigraph_pb2
import obstacle_pb2
from google.protobuf import text_format

if len(sys.argv) < 4:
    print("Usage: <multi graph file> <obstacles file> <draw edges>")
    exit(-1)

xs = []
ys = []
smooth_xs = []
smooth_ys = []

path_points = []
circles = []

fig = plt.gcf()
ax = fig.gca()

f = open(sys.argv[1], 'rb') 
message = text_format.Parse(f.read(), path_pb2.Path())
for p in message.path_waypoint_list:
    xs.append(float(p.x))
    ys.append(float(p.y))
    path_points.append((float(p.x), float(p.y)))

def get_color(idx):
    return ['brown', 'green', 'orange', 'purple', 'magenta', 'pink', 'teal'][idx % 7 ]

def get_vertex_color(idx):
    return ['teal', 'orange', 'purple', 'magenta', 'pink', 'brown', 'green'][idx % 7]

f = open(sys.argv[2], 'rb') 
message = text_format.Parse(f.read(), obstacle_pb2.Obstacles())
print(message)
for p in message.circle_list:
    circles.append((float(p.x), float(p.y), float(p.radius)))

for p in path_points:
    cr = plt.Circle((p[0], p[1]), 5, color='r')
    ax.add_artist(cr)

plt.plot(xs, ys, 'r-')

for c in circles:
    cr = plt.Circle((c[0], c[1]), c[2], color='r')
    ax.add_artist(cr)

plt.xlim([-4500,4500])
plt.ylim([-3000,3000])
plt.axes().set_aspect('equal', 'datalim')
plt.tight_layout()
fig.set_size_inches(30, 20)
fig.savefig('locked.png')
plt.show()

