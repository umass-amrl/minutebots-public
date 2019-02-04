#!/usr/bin/env python3
import os
import sys
import glob
import shutil
import numpy as np
import matplotlib.pyplot as plt
from time import sleep

dest = os.path.join(os.getcwd(), 'scripts/python/repair/')
assert(os.path.isdir(dest))
for filename in glob.glob(os.path.join("build/", '*.py')):
    shutil.copy2(filename, dest)

import replan_scenerio_pb2
import search_pb2
from google.protobuf import text_format

if len(sys.argv) < 2:
    print("Usage: <proto folder>")
    exit(-1)

def get_key(s):
    numdotproto = s.split("_")[-1]
    intval = numdotproto.split(".")[0]
    return int(intval)

def get_path_key(s):
    numdotproto = s.split("_")[-2]
    intval = numdotproto.split(".")[0]
    return int(intval)

def get_color(idx):
    colors = ['#ff3300', '#33cc33', '#0066ff', '#ffff66', '#9966ff']
    return colors[idx % len(colors)]

def invalid_position_list(lst):
    for p in lst:
        if p.x == 2147483647 or p.y == 2147483647:
            return True
    return False

files = sorted([os.path.join(sys.argv[1], f) for f in os.listdir(sys.argv[1])])
scenerio_files = sorted([f for f in files if "replan_scenerio" in f], key=get_key)
parentmap_files_list = [(get_key(f), []) for f in scenerio_files]
parentmap_files_list = [sorted([f for f in files if "parentmap_{}_step".format(pair[0]) in f], key=get_path_key) for pair in parentmap_files_list]

print(parentmap_files_list)

def show():
    plt.xlim([-5,5])
    plt.ylim([-5,5])
    plt.axes().set_aspect('equal', 'datalim')
    plt.tight_layout()
    # fig.set_size_inches(30, 20)
    # fig.savefig('locked.png')
    plt.show()


def draw_point_obstacles(p_list):
    for p in p_list:
        ax.add_artist(plt.Circle((p.x, p.y), 0.1, color='grey', alpha=0.2))

def draw_replan_box(center_x, center_y, radius):
    ax.add_artist(plt.Rectangle((center_x - radius, center_y - radius),
                                2 * radius,
                                2 * radius,
                                color='black', linestyle='-', linewidth=1,
                                fill=False,
                                alpha=1))

def draw_start_end(starts, ends):
    for p in starts:
        ax.add_artist(plt.Circle((p.x, p.y), 0.075, color='green', alpha=0.2))
    for p in ends:
        ax.add_artist(plt.Circle((p.x, p.y), 0.1, color='red', alpha=0.2))

def draw_closed_list(closed):
    for a in closed:
        for p in a.position:
            ax.add_artist(plt.Circle((p.x, p.y), 0.05, color='black', alpha=0.2))

def draw_parent_map(edges):
    for e in edges:
        for idx, tpl in enumerate(zip(e.current_positions, e.prev_positions)):
            current, prev = tpl
            offset = 0.1 * idx

            x = current.x + offset
            y = current.y
            dx = prev.x - current.x
            dy = prev.y - current.y
            # point towards prev
            ax.arrow(x, y, dx, dy,
                     head_width=0.15, head_length=0.15, color=get_color(idx), alpha=0.1, head_starts_at_zero=False)

def draw_current_edge(current_edge):
    assert(not invalid_position_list(current_edge.search_tree_edge.current_positions))
    print("Heuristic:", current_edge.heuristic)

    kSqrtTwo = 1.414213562

    plt.text(0, -4.6, "G: " + str(sum([d.straight for d  in current_edge.distances])) + ", " + str(sum([d.angled for d in current_edge.distances])))
    plt.text(0, -4.3, "H: " + str(current_edge.heuristic.straight) + ", " + str(current_edge.heuristic.angled))
    plt.text(0, -4, "F: " +  str(current_edge.heuristic.straight + kSqrtTwo * current_edge.heuristic.angled + sum([d.straight for d in current_edge.distances]) + sum([d.angled for d in current_edge.distances]) * kSqrtTwo))
    print("Distances:", current_edge.distances)
    print("Times:", current_edge.times)

    # for t, d in zip(current_edge.distances, current_edge.times):
    #     if (d != 0):
    #         print(t / d)
    #     else:
    #         print("d=0")

    for idx, pair in enumerate(zip(current_edge.search_tree_edge.current_positions, current_edge.distances)):
        p, d = pair
        ax.add_artist(plt.Circle((p.x, p.y), 0.05, color='blue', alpha=1))
        plt.text(p.x, p.y, str(d), color=get_color(idx))
    if not invalid_position_list(current_edge.search_tree_edge.prev_positions):
        assert(len(current_edge.search_tree_edge.prev_positions) == len(current_edge.search_tree_edge.current_positions))
        for current, prev in zip(current_edge.search_tree_edge.current_positions, current_edge.search_tree_edge.prev_positions):
            plt.plot([current.x, prev.x], [current.y, prev.y], color='blue', alpha=1)

def draw_prev_edges(prev_edges):
    for prev_edge in prev_edges:
        assert(not invalid_position_list(prev_edge.current_positions))
        for idx, p in enumerate(prev_edge.current_positions):
            ax.add_artist(plt.Circle((p.x, p.y), 0.05, color=get_color(idx), alpha=0.3))
        if not invalid_position_list(prev_edge.prev_positions):
            assert(len(prev_edge.prev_positions) == len(prev_edge.current_positions))
            for idx, pair in enumerate(zip(prev_edge.current_positions, prev_edge.prev_positions)):
                current, prev = pair
                #plt.plot([current.x, prev.x], [current.y, prev.y], color=get_color(idx), alpha=0.3)
                if prev.x != current.x or prev.y != current.y:
                    ax.arrow(prev.x, prev.y, (current.x - prev.x), (current.y - prev.y), head_width=0.05, head_length=0.1, fc='k', ec='k')


def press(event):
    global search_tree_files_index
    global scenerio_files_index
    print('press', event.key)
    sys.stdout.flush()
    if event.key == 'right':
        ax.clear()
        search_tree_files_index += 1
    elif event.key == 'left':
        ax.clear()
        search_tree_files_index -= 1
    elif event.key == 'home':
        ax.clear()
        search_tree_files_index += 20
    elif event.key == 'end':
        ax.clear()
        search_tree_files_index -= 20
    elif event.key == 'up':
        ax.clear()
        scenerio_files_index += 1
        update_scenerio_message()
    elif event.key == 'down':
        ax.clear()
        scenerio_files_index -= 1
        update_scenerio_message()
    else:
        return
    print("Scenerio {}, Index {}".format(scenerio_files_index, search_tree_files_index))
    plot_parent_map(scenerio_message, parentmap_files_list[scenerio_files_index][search_tree_files_index])
    plt.xlim([-5,5])
    plt.ylim([-5,5])
    plt.axes().set_aspect('equal', 'datalim')
    plt.tight_layout()
    fig.canvas.draw()

# fig = plt.gcf()
# ax = fig.gca()
# fig.canvas.mpl_connect('key_press_event', press)
# ax.plot(np.random.rand(12), np.random.rand(12), 'go')
# xl = ax.set_xlabel('easy come, easy go')
# ax.set_title('Press a key')
# plt.show()

def update_scenerio_message():
    global scenerio_message
    f = open(scenerio_files[scenerio_files_index], 'rb')
    scenerio_message = text_format.Parse(f.read(), replan_scenerio_pb2.ReplanScenerio())
    assert(len(scenerio_message.start.positions) == len(scenerio_message.goal.positions))


def plot_parent_map(scenerio_message, parent_map_file):
    print("Parent map file:", parent_map_file)
    draw_point_obstacles(scenerio_message.point_obstacle.point_list)
    draw_replan_box(scenerio_message.center.x, scenerio_message.center.y, scenerio_message.radius)
    draw_start_end(scenerio_message.start.positions, scenerio_message.goal.positions)
    f = open(parent_map_file, 'rb')
    parent_map_message = text_format.Parse(f.read(), search_pb2.ParentMap())
    draw_parent_map(parent_map_message.edges)



scenerio_files_index = 0
search_tree_files_index = 0

update_scenerio_message()

fig = plt.gcf()
ax = fig.gca()
fig.canvas.mpl_connect('key_press_event', press)
plot_parent_map(scenerio_message, parentmap_files_list[scenerio_files_index][search_tree_files_index])
show()
