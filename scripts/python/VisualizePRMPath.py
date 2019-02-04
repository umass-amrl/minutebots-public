#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import sys
import pdb

if __name__ == "__main__":
    #edges = np.array()
    #path = np.array()
    #obstacle_centers = np.array()
    #obstacle_areas = np.array()
    f = open(sys.argv[1], 'r')

    junk = f.readline()
    line = f.readline()

    ind = line.find(',')
    x = float(line[0:ind])
    y = float(line[ind+2:-1])

    vertices = np.array([x,y])
    line = f.readline()
    
    while (line != "[EDGE LIST]\n"):
        # Parse Vertex
        # Format x, y
        ind = line.find(',')
        x = float(line[0:ind])
        y = float(line[ind+2:-1])
        vertices = np.vstack((vertices, np.array([x,y])))
        line = f.readline()

    print "Parsed graph vertices"
    plt.scatter(vertices[:,0], vertices[:,1], color='black')

    #line = f.readline()
    #ind = line.find('>')
    #str1 = line[0:ind-2]
    #str2 = line[ind+2:-1]
    #ind = str1.find(',')
    #x1 = float(str1[0:ind])
    #y1 = float(str1[ind+2:])
    #ind = str2.find(',')
    #x2 = float(str2[0:ind])
    #y2 = float(str2[ind+2:])
    #edges = np.array([[[x1,y1],[x2,y2]]])
    line=f.readline()

    while (line != "[FINAL PATH]\n"):
        # Process Edge
        # Format x1, y1 -> x2, y2
        #ind = line.find('>')
        #str1 = line[0:ind-2]
        #str2 = line[ind+2:-1]
        #ind = str1.find(',')
        #x1 = float(str1[0:ind])
        #y1 = float(str1[ind+2:])
        #ind = str2.find(',')
        #x2 = float(str2[0:ind])
        #y2 = float(str2[ind+2:])
        #plt.plot([x1,x2],[y1,y2], linestyle='dotted', color='black')
        line = f.readline()

    print "Parsed edges"
    #plt.plot(edges)

    line = f.readline()

    ind = line.find(',')
    x = float(line[0:ind])
    y = float(line[ind+2:-1])

    path = np.array([x,y])
    line = f.readline()

    while (line != "[DYNAMIC OBSTACLES]\n"):
        # Process Path vertex
        # Format x, y
        ind = line.find(',')
        x = float(line[0:ind])
        y = float(line[ind+2:-1])
        path = np.vstack((path, np.array([x,y])))
        line = f.readline()

    print "Parsed path"

    #plt.plot(path[:,0], path[:,1], linestyle='dotted', color='blue')

    fig = plt.gcf()
    ax = fig.gca()


    line = f.readline()
    ind = line.find(',')
    x = float(line[0:ind])
    y = float(line[ind+2:-1])
    ax.add_artist(plt.Circle((x,y), 90, color='g', fill=False))
    line = f.readline()

    #plot the path
    path = np.vstack((np.array([x,y]), path))
    plt.plot(path[:,0], path[:,1], color='blue', lw=3)

    while (line != "[FINAL PATH LENGTH]\n"): 
        # Process variable center
        # Format x, y
        print(line)
        split_line = line.split(", ")
        print(split_line)
        ind = line.find(',')
        x = float(split_line[0])
        y = float(split_line[1])
        type = split_line[2]
        if type == "rect":
            # Rectangle
            width = float(split_line[3])
            height = float(split_line[4])
            print("Width: " + str(width))
            print("Height: " + str(height))
            ax.add_artist(plt.Rectangle((x - width/2,y-height/2), width, height, color='r'))
        elif type == "circ" or type == "robo":
            # Circle
            radius = float(split_line[3])
            print("Radius: " + str(radius))
            ax.add_artist(plt.Circle((x,y), radius, color='r', fill=False))
        line = f.readline()

    print "Parsed obstacle positions"

    line = f.readline()
    print line

    f.close()

    plt.show()
