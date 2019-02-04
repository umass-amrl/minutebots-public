#!/usr/bin/env python3
import os
import sys
import glob
import shutil
import numpy as np
import matplotlib.pyplot as plt
from time import sleep
import networkx as nx
import itertools

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

fig = plt.gcf()
ax = fig.gca()

# plt.subplot(122)
# nx.draw_shell(G, nlist=[range(5, 10), range(5)], with_labels=True, font_weight='bold')

# fig = plt.gcf()
# ax = fig.gca()

# for x in range(8):
#     for y in range(8):
#         ax.add_artist(plt.Circle((x, y), 0.1, color='grey', alpha=0.2))

# plt.xlim([-1,8])
# plt.ylim([-1,8])


#plt.show()

#exit()


def get_key(s):
    s = s.split("/")[2]
    key = int(s[3]) * 100

    if "11." in s:
        key += 10
    elif "12." in s:
        key += 20
    else:
        key += 30
    if "post" in s:
        key += 1

    return key

def get_key(s):
    s = s.split("/")[2]
    s = int(s.split("sst")[1].split("result")[0])
    return s


files = sorted([os.path.join(sys.argv[1], f) for f in os.listdir(sys.argv[1])])
search_tree_files = sorted([f for f in files if "sst" in f], key=get_key)
print(search_tree_files)

states = [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (2, 1), (2, 2)]
def position_to_string(p):
    idx1 = states.index((p.p1.x, p.p1.y))
    idx2 = states.index((p.p2.x, p.p2.y))
    return idx1 + 8 * idx2
    print()
    return ((p.p1.x, p.p1.y), (p.p2.x, p.p2.y))
    # return "({}, {}), ({}, {}) ".format(p.p1.x, p.p1.y, p.p2.x, p.p2.y)

def plot_states():
    for x in range(len(states)):
        for y in range(len(states)):
            ax.add_artist(plt.Circle((x, y), 0.1, color='grey', alpha=0.2))
            ax.text(x, y, '{} {}'.format(states[x], states[y]),
                    verticalalignment='center', horizontalalignment='center',
                    color='black', fontsize=10)

def plot_search_tree(search_tree_file, idx):
    print("File:", search_tree_file)
    f = open(search_tree_file, 'rb')
    search_tree_message = text_format.Parse(f.read(), search_pb2.SimpleSearchTree())
    for c in search_tree_message.closed_list:
        p1idx = states.index((c.position.p1.x, c.position.p1.y))
        p2idx = states.index((c.position.p2.x, c.position.p2.y))
        ax.add_artist(plt.Circle((p1idx, p2idx), 0.07, color='red', alpha=0.2))

    if len(search_tree_message.open_list) == 0:
        return
    n = search_tree_message.open_list[idx]
    prev_p1idx = None
    prev_p2idx = None
    print("Sum: {} Weight: {} Heuristic: {}".format((n.weight + n.heuristic), n.weight, n.heuristic))
    for idx, p in enumerate(n.path):
        p1idx = states.index((p.p1.x, p.p1.y))
        p2idx = states.index((p.p2.x, p.p2.y))
        alpha = 0.5 if idx != len(n.path) - 1 else 1
        color = "blue" if idx != len(n.path) - 1 else "green"
        ax.add_artist(plt.Circle((p1idx, p2idx), 0.05, color=color, alpha=alpha))
        if prev_p1idx is not None and prev_p2idx is not None:
            print("{}, {} => {}, {}".format(states[prev_p1idx], states[prev_p2idx], states[p1idx], states[p2idx]))
            try:
                if prev_p1idx != p1idx or prev_p2idx != p2idx:
                    ax.arrow(prev_p1idx, prev_p2idx, p1idx - prev_p1idx, p2idx - prev_p2idx,
                             head_width=0.1, length_includes_head=True)
            except ValueError:
                print("Stupid value error")
        prev_p1idx = p1idx
        prev_p2idx = p2idx



def press_openlist(event):
    global openlist_idx
    global search_tree_idx
    print('press', event.key)
    sys.stdout.flush()
    if event.key == 'right':
        ax.clear()
        openlist_idx += 1
    elif event.key == 'left':
        ax.clear()
        openlist_idx -= 1

    elif event.key == 'up':
        ax.clear()
        search_tree_idx += 1
    elif event.key == 'down':
        ax.clear()
        search_tree_idx -= 1
    else:
        return
    plot_states()
    print("Openlist: {}, Search Tree: {}".format(openlist_idx, search_tree_idx))
    plot_search_tree(search_tree_files[search_tree_idx], openlist_idx)
    plt.xlim([-1,8])
    plt.ylim([-1,8])
    fig.canvas.draw()

openlist_idx = 0
search_tree_idx = 0

fig = plt.gcf()
ax = fig.gca()
fig.canvas.mpl_connect('key_press_event', press_openlist)
print(search_tree_files[search_tree_idx].split("/")[2])
plot_states()
plot_search_tree(search_tree_files[search_tree_idx], openlist_idx)
plt.xlim([-1,8])
plt.ylim([-1,8])
plt.show()

