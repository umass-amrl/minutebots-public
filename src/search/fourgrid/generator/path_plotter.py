#! /usr/bin/env python3
import itertools
import sys
import matplotlib.pyplot as plt
import numpy as np
import grid_generator as gg

# Individual


p1_individual = [ (0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (4, 1), (4, 2), (4, 3), (4, 4), (4, 5), (5, 5), (6, 5), (7, 5), (8, 5), (9, 5), (10, 5), (10, 6), (10, 7), (10, 8), (10, 9), (10, 10),]

p2_individual = [ (10, 10),  (9, 10),  (8, 10),  (7, 10),  (6, 10),  (6, 9),  (6, 8),  (6, 7),  (6, 6),  (6, 5),  (5, 5),  (4, 5),  (3, 5),  (2, 5),  (1, 5),  (0, 5),  (0, 4),  (0, 3),  (0, 2),  (0, 1),  (0, 0), ]

# Joint

p1_joint =[ (0, 0),  (1, 0),  (2, 0),  (3, 0),  (4, 0),  (4, 1),  (4, 2),  (4, 3),  (4, 4),  (4, 5),  (4, 5),  (4, 6),  (4, 5),  (5, 5),  (6, 5),  (7, 5),  (8, 5),  (9, 5), (10, 5),  (10, 6),  (10, 7),  (10, 8),  (10, 9),  (10, 10),]

p2_joint =[(10, 10),  (9, 10),  (8, 10),  (7, 10),  (6, 10),  (6, 9),  (6, 8),  (6, 7),  (6, 6),  (6, 5),  (5, 5),  (4, 5),  (3, 5),  (2, 5), (1, 5),  (0, 5),  (0, 4),  (0, 3),  (0, 2),  (0, 1),  (0, 0), ]




def plot_grid(vertices, edges, plot_window=False):
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


    path = p2_joint

    color_step = 1/(len(path) + 1)

    for i, p in enumerate(path):
        x, y = p
        aa[x, 10-y] = (1-(i+1)) * color_step

    # Display matrix
    fig = plt.figure()
    ax = plt.Axes(fig, [0.0, 0.0, 1.0, 1.0])
    ax.set_axis_off()
    fig.add_axes(ax)
    ax.matshow(aa.T, cmap=plt.cm.Reds)
    plt.show()

invalid_positions = [(5, y) for y in range(11) if y != 5 ]
verticies, edges = gg.generate_grid(0, 10, 0, 10, gg.make_predicate(invalid_positions))
starts = [(0, 0), (10, 10)]
goals = [(10, 10), (0, 0)]
plot_grid(verticies, edges)
