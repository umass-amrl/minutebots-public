#!/usr/bin/env python3

import matplotlib.pyplot as plt
from enum import Enum
fig, (ax1, ax2) = plt.subplots(1, 2)
generator_args_state = {}

kNorth = 0
kEast = 1
kSouth = 2
kWest = 3

class NextDirection(Enum):
    CLOCKWISE = 1
    COUNTERCLOCKWISE = 2
    OPPOSITE = 3

def GetOpposingRotation(direction):
    if direction == NextDirection.CLOCKWISE:
        return NextDirection.COUNTERCLOCKWISE
    elif direction == NextDirection.COUNTERCLOCKWISE:
        return NextDirection.CLOCKWISE
    else:
        print("ERROR: not a rotation:", direction)

def GetPosition(first_position):
    if first_position == kNorth:
        return 0, 1
    elif first_position == kSouth:
        return 0, -1
    elif first_position == kEast:
        return 1, 0
    elif first_position == kWest:
        return -1, 0

def GetNextPosition(current_position, operation):
    if operation == NextDirection.OPPOSITE:
        current_position -= 2
        if current_position < 0:
            current_position += 4
        return current_position
    elif operation == NextDirection.CLOCKWISE:
        current_position += 1
        if current_position > 3:
            current_position -= 4
        return current_position
    else:
        current_position -= 1
        if current_position < 0:
            current_position += 4
        return current_position

def GetNeighborGenerator(first_pos, next_direction):
    current_pos = first_pos
    yield GetPosition(current_pos)
    for operation in [next_direction, NextDirection.OPPOSITE, GetOpposingRotation(next_direction)]:
        current_pos = GetNextPosition(current_pos, operation)
        yield GetPosition(current_pos)

lookup_table = {}
lookup_table[0] = (kWest, NextDirection.COUNTERCLOCKWISE)
lookup_table[1] = (kWest, NextDirection.CLOCKWISE)
lookup_table[2] = (kSouth, NextDirection.CLOCKWISE)
lookup_table[3] = (kNorth, NextDirection.COUNTERCLOCKWISE)
lookup_table[4] = (kSouth, NextDirection.COUNTERCLOCKWISE)
lookup_table[5] = (kNorth, NextDirection.CLOCKWISE)
lookup_table[6] = (kEast, NextDirection.CLOCKWISE)
lookup_table[7] = (kEast, NextDirection.COUNTERCLOCKWISE)

def draw_plus(ax):
    ax.plot([-1, 1], [0, 0], "-", color='blue')
    ax.plot([0, 0], [-1, 1], "-", color='blue')
    ax.set_ylim((-3, 3))
    ax.set_xlim((-3, 3))
    

def draw_quadrants(ax):
    ax.plot([-3, 3], [3, -3], color='gray', alpha=0.1)
    ax.plot([3, -3], [3, -3], color='gray', alpha=0.1)
    ax.plot([0, 0], [3, -3], color='gray', alpha=0.1)
    ax.plot([3, -3], [0, 0], color='gray', alpha=0.1)
    i = 0
    for x in [-2.4, -1, 1, 2.4]:
        for y in [-1.5, 1.5]:
            if x == 0 and y == 0:
                continue
            ax.text(x , y, str(i))
            i += 1

def get_click_quadrant(x, y):
    if y >=0:
        if x >=0:
            if x < y:
                return 5
            else:
                return 7
        else:
            if abs(x) < y:
                return 3
            else:
                return 1
    else:
        if x >=0:
            if x < abs(y):
                return 4
            else:
                return 6
        else:
            if abs(x) < abs(y):
                return 2
            else:
                return 0


def mark_postion(x, y, ax):
    ax.plot([x - 0.1, x + 0.1], [y + 0.1,y - 0.1], '-', color="black")
    ax.plot([x - 0.1, x + 0.1], [y - 0.1,y + 0.1], '-', color="black")

def label_ordering(generator, ax):
    for idx, pos in enumerate(generator):
        ax.text(pos[0], pos[1], str(idx))

def draw_base_images(axis=[ax1, ax2]):
    for i, ax in enumerate(axis):
        draw_plus(ax)
        draw_quadrants(ax)

def get_proper_axis(event):
    bbox = fig.get_window_extent().transformed(fig.dpi_scale_trans.inverted())
    width, height = bbox.width*fig.dpi, bbox.height*fig.dpi
    return (ax1, 1) if event.x < width/2 else (ax2, 2)

def fmt(n):
    return "{:0.2f}".format(n)

def fmtall(l):
    return [fmt(e) for e in l]

def compute_heuristics(results):
    lst = []
    for (pos_x, pos_y), (click_x, click_y) in results:
        lst.append(abs(pos_x - click_x) + abs(pos_y - click_y))
    return (lst, sum(lst))

def print_selected_neighbors_rec(prefix, generator_args_state_idx, following_idxs):
    generator_args, (click_x, click_y), quadrant_index = generator_args_state[generator_args_state_idx]
    generator = GetNeighborGenerator(generator_args[0], generator_args[1])
    for p in generator:
        if len(following_idxs) <= 0:
            result = prefix + [(p, (click_x, click_y))]
            heuristics = compute_heuristics(result)
            heuristics_pp = (fmtall(heuristics[0]), fmt(heuristics[1]))
            print([(p, (fmt(c_x), fmt(c_y))) for (p, (c_x, c_y)) in result], heuristics_pp)
        else:
            print_selected_neighbors_rec(prefix + [(p, (click_x, click_y))], following_idxs[0], following_idxs[1:])

def print_selected_neighbors():
    if len(generator_args_state) < 2:
        return
    generator_idxs = [1, 2]
    generators_args_lst = [generator_args_state[i] for i in generator_idxs]
    generators_lst = [GetNeighborGenerator(generator_args[0], generator_args[1]) for generator_args, c in generators_args_lst]
    generators_values_lst = [next(g) for g in generators_lst]
    print_selected_neighbors_rec([], generator_idxs[0], generator_idxs[1:])

def onclick(event):
    if event.xdata is None or event.ydata is None:
        return
    # print('button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
    #       (event.button, event.x, event.y, event.xdata, event.ydata))
    proper_axis, idx = get_proper_axis(event)
    proper_axis.clear()
    draw_base_images(axis=[proper_axis])
    mark_postion(event.xdata, event.ydata, proper_axis)
    quadrant_index = get_click_quadrant(event.xdata, event.ydata)
    proper_axis.text(-2.5, -2.5, "Click Section: {}".format(quadrant_index))
    generator_args = lookup_table[quadrant_index]
    generator_args_state[idx] = (generator_args, (event.xdata, event.ydata), quadrant_index)
    generator = GetNeighborGenerator(generator_args[0], generator_args[1])
    label_ordering(generator, proper_axis)
    fig.canvas.draw()
    print("Neighbors:")
    print_selected_neighbors()

cid = fig.canvas.mpl_connect('button_press_event', onclick)
draw_base_images()
plt.show()
