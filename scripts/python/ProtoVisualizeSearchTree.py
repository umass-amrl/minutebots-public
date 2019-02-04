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
import search_pb2
import point_obstacle_pb2
import multigraph_pb2
import pathable_multigraph_pb2
import obstacle_pb2
from google.protobuf import text_format

if len(sys.argv) < 2:
    print("Usage: <proto folder>")
    exit(-1)

def get_key(s):
    numdotproto = s.split("_")[-1]
    intval = numdotproto.split(".")[0]
    return int(intval)

def get_color(idx):
    colors = ['#ff3300', '#33cc33', '#0066ff', '#ffff66', '#9966ff']
    return colors[idx % len(colors)]

files = sorted([os.path.join(sys.argv[1], f) for f in os.listdir(sys.argv[1])])
search_tree_files = sorted([f for f in files if "search_tree" in f], key=get_key)

def show():
    plt.xlim([-5,5])
    plt.ylim([-2,8])
    plt.axes().set_aspect('equal', 'datalim')
    plt.tight_layout()
    # fig.set_size_inches(30, 20)
    # fig.savefig('locked.png')
    plt.show()


for idx, search_tree_file in enumerate(search_tree_files):
    fig = plt.gcf()
    ax = fig.gca()
    f = open(search_tree_file, 'rb')
    message = text_format.Parse(f.read(), search_pb2.SearchTree())
    current_xs = [e.x for e in message.current_edge.current_positions if e.x != 2147483647]
    current_ys = [e.y for e in message.current_edge.current_positions if e.y != 2147483647]
    prev_xs = [e.x for e in message.current_edge.prev_positions if e.x != 2147483647]
    prev_ys = [e.y for e in message.current_edge.prev_positions if e.y != 2147483647]

    xs_list = [list(t) for t in zip(current_xs, prev_xs)]
    ys_list = [list(t) for t in zip(current_ys, prev_ys)]

    print(xs_list)

    for xs, ys in zip(xs_list, ys_list):
        for x, y in zip(xs, ys):
            print(x, y)
            ax.add_artist(plt.Circle((x, y), 0.15, color='blue', alpha=0.5))
            plt.plot([x], [y], color='blue')

    for other_edge in message.other_edge_list:
        current_xs = [e.x for e in other_edge.current_positions if e.x != 2147483647]
        current_ys = [e.y for e in other_edge.current_positions if e.y != 2147483647]
        prev_xs = [e.x for e in other_edge.prev_positions if e.x != 2147483647]
        prev_ys = [e.y for e in other_edge.prev_positions if e.y != 2147483647]
        xs_list = [list(t) for t in zip(current_xs, prev_xs)]
        ys_list = [list(t) for t in zip(current_ys, prev_ys)]

        for idx, xs_ys in enumerate(zip(xs_list, ys_list)):
            for inner_idx, x_y in enumerate(zip(xs_ys[0], xs_ys[1])):
                x, y = x_y
                ax.add_artist(plt.Circle((x, y), 0.15, color=get_color(inner_idx), alpha=0.5))
                plt.plot([x], [y], color=get_color(inner_idx))
    show()


path_points = []
circles = []

fig = plt.gcf()
ax = fig.gca()

for idx, file_tuple in enumerate(zip(robot_0_search_tree_files, robot_1_search_tree_files)):
    print("Search tree index:", idx, "Files:", file_tuple)
    fig = plt.gcf()
    ax = fig.gca()
    for inner_idx, search_tree_file in enumerate([file_tuple[0], file_tuple[1]]):
        f = open(search_tree_file, 'rb')
        message = text_format.Parse(f.read(), search_pb2.SearchTree())
        if 2147483648.0 == message.current_edge.x_start:
            continue
        xs = []
        ys = []
        xs.append(float(message.current_edge.x_start))
        ys.append(float(message.current_edge.y_start))
        xs.append(float(message.current_edge.x_end))
        ys.append(float(message.current_edge.y_end))
        print(inner_idx, "from", message.current_edge.x_start, ",", message.current_edge.y_start, "to",message.current_edge.x_end,",",message.current_edge.x_end)
        cr = plt.Circle((xs[1], ys[1]), 0.1, color='blue', alpha=0.5)
        ax.add_artist(cr)
        plt.plot(xs, ys, color='blue')

        for p in message.other_edge_list:
            xs = []
            ys = []
            if 2147483648.0 == p.x_start:
                continue
            xs.append(float(p.x_start))
            ys.append(float(p.y_start))
            xs.append(float(p.x_end))
            ys.append(float(p.y_end))
            cr = plt.Circle((xs[1], ys[1]), 0.15, color=get_color(idx), alpha=0.5)
            ax.add_artist(cr)
            plt.plot(xs, ys, color=get_color(inner_idx))
    show()
