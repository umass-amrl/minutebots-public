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
message = text_format.Parse(f.read(), pathable_multigraph_pb2.PathableMultiGraphProto())
for p in message.path_waypoint_list:
    xs.append(float(p.x))
    ys.append(float(p.y))
    path_points.append((float(p.x), float(p.y)))

for p in message.smoothed_path_waypoint_list:
    smooth_xs.append(float(p.x))
    smooth_ys.append(float(p.y))

def get_color(idx):
    return ['brown', 'green', 'orange', 'purple', 'magenta', 'pink', 'teal'][idx % 7 ]

def get_vertex_color(idx):
    return ['teal', 'orange', 'purple', 'magenta', 'pink', 'brown', 'green'][idx % 7]

for idx, g in enumerate(message.multi_graph.graph_list):
    if sys.argv[3].lower() == 't' or sys.argv[3].lower() == 'true':
        for e in g.edge_list:
            v1 = g.vertex_list[e.vertex_1_index]
            v2 = g.vertex_list[e.vertex_2_index]
            pxs = [ v1.x, v2.x ]
            pys = [ v1.y, v2.y ]
            alpha_val = 0.1 if idx == 0 else 0.7
            plt.plot(pxs, pys, color='black', alpha=alpha_val)


for e in message.multi_graph.intra_graph_edge_list:
    if sys.argv[3].lower() == 't' or sys.argv[3].lower() == 'true':
        print("{}, {} to {}, {}".format(e.graph_1_index, e.vertex_1_index, e.graph_2_index, e.vertex_2_index))
        g1 = message.multi_graph.graph_list[e.graph_1_index]
        v1 = g1.vertex_list[e.vertex_1_index]
        g2 = message.multi_graph.graph_list[e.graph_2_index]
        v2 = g2.vertex_list[e.vertex_2_index]
        pxs = [ v1.x, v2.x ]
        pys = [ v1.y, v2.y ]
        plt.plot(pxs, pys, color='black', alpha=0.1)

for idx, g in enumerate(message.multi_graph.graph_list):
    for v in g.vertex_list:
        cr = plt.Circle((v.x, v.y), 25, color=get_vertex_color(idx))
        ax.add_artist(cr)

f = open(sys.argv[2], 'rb') 
message = text_format.Parse(f.read(), obstacle_pb2.Obstacles())
print(message)
for p in message.circle_list:
    circles.append((float(p.x), float(p.y), float(p.radius)))

for p in path_points:
    cr = plt.Circle((p[0], p[1]), 5, color='r')
    ax.add_artist(cr)

l = plt.plot(xs, ys, '--')
plt.setp(l, linewidth=3, color='black')
#plt.plot(smooth_xs, smooth_ys, 'b--')

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

