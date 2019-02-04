#!/usr/bin/env python3
import numpy as np
import sys
import matplotlib
import matplotlib.pyplot as plt

font = {'family' : 'cmr10',
        'weight' : 'bold',
        'size'   : 22}

matplotlib.rc('font', **font)


print("Starting allocation")
has_collision = np.empty(shape=(0,0))
num_vertices = np.empty(shape=(0,0))
standard_init_time = np.empty(shape=(0,0))
standard_update_time = np.empty(shape=(0,0))
standard_setup_plan_time = np.empty(shape=(0,0))
standard_plan_time = np.empty(shape=(0,0))
standard_plan_steps = np.empty(shape=(0,0))
standard_plan_length = np.empty(shape=(0,0))
standard_num_edges = np.empty(shape=(0,0))
scaffold_init_time = np.empty(shape=(0,0))
scaffold_update_time = np.empty(shape=(0,0))
scaffold_setup_plan_time = np.empty(shape=(0,0))
scaffold_plan_time = np.empty(shape=(0,0))
scaffold_plan_steps = np.empty(shape=(0,0))
scaffold_plan_length = np.empty(shape=(0,0))
scaffold_num_verticies = np.empty(shape=(0,0))
scaffold_num_edges = np.empty(shape=(0,0))
print("Done allocation")

fig = plt.gcf()
ax = fig.gca()

graph_num_verts = np.empty(shape=(0,0))
standard_path_dropped = np.empty(shape=(0,0))
scaffold_path_dropped = np.empty(shape=(0,0))

def init_storage():
    # Globals from allocation above.
    global has_collision
    global num_vertices
    global standard_init_time
    global standard_update_time
    global standard_setup_plan_time
    global standard_plan_time
    global standard_plan_steps
    global standard_plan_length
    global standard_num_edges
    global scaffold_init_time
    global scaffold_update_time
    global scaffold_setup_plan_time
    global scaffold_plan_time
    global scaffold_plan_steps
    global scaffold_plan_length
    global scaffold_num_verticies
    global scaffold_num_edges
    has_collision = np.empty(shape=(0,0))
    num_vertices = np.empty(shape=(0,0))
    standard_init_time = np.empty(shape=(0,0))
    standard_update_time = np.empty(shape=(0,0))
    standard_setup_plan_time = np.empty(shape=(0,0))
    standard_plan_time = np.empty(shape=(0,0))
    standard_plan_steps = np.empty(shape=(0,0))
    standard_plan_length = np.empty(shape=(0,0))
    standard_num_edges = np.empty(shape=(0,0))
    scaffold_init_time = np.empty(shape=(0,0))
    scaffold_update_time = np.empty(shape=(0,0))
    scaffold_setup_plan_time = np.empty(shape=(0,0))
    scaffold_plan_time = np.empty(shape=(0,0))
    scaffold_plan_steps = np.empty(shape=(0,0))
    scaffold_plan_length = np.empty(shape=(0,0))
    scaffold_num_verticies = np.empty(shape=(0,0))
    scaffold_num_edges = np.empty(shape=(0,0))

def parse(line):
    #globals from allocation above.
    global has_collision
    global num_vertices
    global standard_init_time
    global standard_update_time
    global standard_setup_plan_time
    global standard_plan_time
    global standard_plan_steps
    global standard_plan_length
    global standard_num_edges
    global scaffold_init_time
    global scaffold_update_time
    global scaffold_setup_plan_time
    global scaffold_plan_time
    global scaffold_plan_steps
    global scaffold_plan_length
    global scaffold_num_verticies
    global scaffold_num_edges

    split_line = line.split("\t")
    has_collision = np.append(has_collision, (split_line[0] == "true"))
    num_vertices = np.append(num_vertices, int(split_line[1]))

    standard_init_time = np.append(standard_init_time, float(split_line[2]))
    standard_update_time = np.append(standard_update_time, float(split_line[3]))
    standard_setup_plan_time = np.append(standard_setup_plan_time, float(split_line[4]))
    standard_plan_time = np.append(standard_plan_time, float(split_line[5]))
    standard_plan_steps = np.append(standard_plan_steps, int(split_line[6]))
    standard_plan_length = np.append(standard_plan_length, float(split_line[7]))
    standard_num_edges = np.append(standard_num_edges, int(split_line[8]))

    scaffold_init_time = np.append(scaffold_init_time, float(split_line[9]))
    scaffold_update_time = np.append(scaffold_update_time, float(split_line[10]))
    scaffold_setup_plan_time = np.append(scaffold_setup_plan_time, float(split_line[11]))
    scaffold_plan_time = np.append(scaffold_plan_time, float(split_line[12]))
    scaffold_plan_steps = np.append(scaffold_plan_steps, int(split_line[13]))
    scaffold_plan_length = np.append(scaffold_plan_length, float(split_line[14]))
    scaffold_num_verticies = np.append(scaffold_num_verticies, int(split_line[15]))
    scaffold_num_edges = np.append(scaffold_num_edges, int(split_line[16]))

def zeros(arr):
    num_dropped = 0
    for i in range(0, len(arr)):
        if arr[i] == 0.0:
            num_dropped = num_dropped + 1
    return num_dropped

def analyze():
    global graph_num_verts
    global scaffold_path_dropped
    global standard_path_dropped
    #print(len(standard_num_edges))
    num_verts = int(num_vertices[0])
    standard_dropped = zeros(standard_plan_length)
    scaffold_dropped = zeros(scaffold_plan_length)
    graph_num_verts = np.append(graph_num_verts, num_verts)
    scaffold_path_dropped = np.append(scaffold_path_dropped, scaffold_dropped)
    standard_path_dropped = np.append(standard_path_dropped, standard_dropped)

def plot():
    plt.xlabel('Number of Vertices')
    plt.ylabel('Path length (mm)')
    plt.plot(graph_num_verts, standard_path_dropped, color='black',
             label='Standard Number of Failures')
    plt.plot(graph_num_verts, scaffold_path_dropped, color='red',
             label='Scaffold Number of Failures')
    # configure legend
    plt.legend(loc=0)
    leg = plt.gca().get_legend()
    ltext = leg.get_texts()
    plt.setp(ltext, fontsize=30)

def handle_file(file_path):
    f = open(file_path, 'r')
    init_storage()

    # Skip first line as it contains just human readable information
    line = f.readline()
    for line in f:
        # print(line, end='')
        parse(line)
    analyze()

def main():
    print("Reading from file")
    file_paths = sys.argv[1:]
    for file_path in file_paths:
        print("File: " + file_path)
        handle_file(file_path)
    plot()
    plt.ylim([0,100])
    plt.xlim([500,2500])
    fig.set_size_inches(10.5, 10.5)
    fig.savefig('/home/kyle/test2png.png', dpi=100)
    plt.show()


if __name__ == "__main__":
    main()
