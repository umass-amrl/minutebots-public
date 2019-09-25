#! /usr/bin/env python3
import itertools
import sys
import matplotlib.pyplot as plt
import numpy as np

def generate_grid(min_x, max_x, min_y, max_y, is_valid):
    kGridEdgeCost = 1
    grid_vertices = []
    grid_edges = []
    for x in range(min_x, max_x + 1):
        for y in range(min_y, max_y + 1):
            current_position = (x, y)
            if not is_valid(current_position):
                continue
            grid_vertices.append(current_position)

            right_position = (x + 1, y)
            if is_valid(right_position) and right_position[0] <= max_x:
                grid_edges.append((current_position, right_position, kGridEdgeCost))

            up_position = (x, y + 1)
            if is_valid(up_position) and up_position[1] <= max_y:
                grid_edges.append((current_position, up_position, kGridEdgeCost))
    return (grid_vertices, grid_edges)

def make_predicate(disallowed_list):
    return (lambda x: not x in disallowed_list)

def in_l1(radius, center, point):
    assert(type(center) == type(point))
    assert(type(center) == list)
    assert(len(center) == len(point))
    for i in range(len(center)):
        return np.linalg.norm(np.array([center[i][0] - point[i][0],
                                        center[i][1] - point[i][1]]),
                              ord=1) <= radius

def in_linf(radius, center, point):
    assert(type(center) == type(point))
    assert(type(center) == list)
    assert(len(center) == len(point))
    for i in range(len(center)):
        return np.linalg.norm(np.array([center[i][0] - point[i][0],
                                        center[i][1] - point[i][1]]),
                              ord=np.inf) <= radius

def get_entrance_exit(path, center, radius, in_fn):
    start = path[0]
    end = path[0]
    for p in path:
        if in_fn(radius, center, p):
            start = p
            break

    for p in reversed(path):
        if in_fn(radius, center, p):
            end = p
            break
    return (start, end)

def plot_grid(vertices, edges, plot_window=False, start=None, goal=None):
    xs = [v[0] for v in vertices]
    ys = [v[1] for v in vertices]
    min_x = min(xs)
    max_x = max(xs)
    delta_x = (max_x - min_x)
    min_y = min(ys)
    max_y = max(ys)
    delta_y = (max_y - min_y)

    import matplotlib.pyplot as plt
    from matplotlib import colors
    import numpy as np

    aa = np.zeros((delta_x + 1, delta_y + 1))
    for x, y in vertices:
        aa[(x - min_x), (y - min_y)] = 1

    if plot_window:
        for x in range(delta_x + 1):
            for y in range(delta_y + 1):
                if in_linf(3, [[(min_x + max_x) / 2, (min_y + max_y) / 2]], [[x, y]]) and aa[(x - min_x), (y - min_y)] != 0:
                    aa[(x - min_x), (y - min_y)] = 0.5

    if start is not None:
        assert(type(start) == list)
        assert(len(start) == 1)
        assert(type(start[0]) == tuple)
        s = start[0]
        aa[s[0], s[1]] = 0.5

    if goal is not None:
        assert(type(goal) == list)
        assert(len(goal) == 1)
        assert(type(goal[0]) == tuple)
        g = goal[0]
        aa[g[0], g[1]] = 0.5

    # Display matrix
    fig = plt.figure()
    ax = plt.Axes(fig, [0.0, 0.0, 1.0, 1.0])
    ax.set_axis_off()
    fig.add_axes(ax)
    ax.matshow(aa.T, cmap=plt.cm.gray)
    plt.show()
