#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import sys
import pdb

_current_file = 0;

def parse_file(file_path):
    print "Parsing " + file_path
    f = open(file_path, 'r')

    line = f.readline()

    x = 0
    y = 0

    fig = plt.gcf()
    ax = fig.gca()

    yellow_robots = []
    line = f.readline()
    while (line != "[BLUE ROBOTS]\n"):
        # Parse Vertex
        # Format x, y
        ind = line.find(',')
        x_str = line[0:ind]
        x = float(x_str)
        y_str = line[ind+1:-1];
        y = float(y_str)
        yellow_robots.append((x,y));
        line = f.readline()

    print "Parsed, found " + str(len(yellow_robots)) + " yellow robots"

    blue_robots = []
    line = f.readline()
    while (line != "[BALL LIST]\n"):
        # Parse Vertex
        # Format x, y
        ind = line.find(',')
        x_str = line[0:ind]
        x = float(x_str)
        y_str = line[ind+1:-1];
        y = float(y_str)
        blue_robots.append((x,y));
        line = f.readline()

    print "Parsed, found " + str(len(blue_robots)) + " blue robots"

    f.close()

    return (yellow_robots, blue_robots)

def draw(lists):
    fig = plt.gcf()
    ax = fig.gca()
    ax.clear()
    fig.canvas.set_window_title('RoboCup SSL Visualizer')
    plt.title('Field Plot Step ' + str(_current_file))

    for t in lists[0]:  # Yellow robots
        plt.scatter(t[0], t[1], color='yellow')
        ax.add_artist(plt.Circle(t, 200, color='yellow'))

    for t in lists[1]:  # Blue robots
        plt.scatter(t[0], t[1], color='blue')
        ax.add_artist(plt.Circle(t, 200, color='blue'))

    plt.ylim([-3000,3000])
    plt.xlim([-4500,4500])
    fig.canvas.draw()


def press(event):
    global _current_file
    print('press', event.key)
    sys.stdout.flush()
    if event.key == 'right':
       print "right"
       _current_file += 1
       print "Current file: " + str(_current_file)
       lists = parse_file(sys.argv[1] + str(_current_file))
       draw(lists);
    if event.key == 'left':
       print "left"
       _current_file -= 1
       print "Current file: " + str(_current_file)
       lists = parse_file(sys.argv[1] + str(_current_file))
       draw(lists);


def main():
    fig, ax = plt.subplots()
    fig.canvas.mpl_connect('key_press_event', press)

    lists = parse_file(sys.argv[1] + str(_current_file))

    draw(lists)
    plt.show()
if __name__ == "__main__":
    main()
