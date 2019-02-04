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

data_list = {}

graph_num_verts = np.empty(shape=(0,0))
global_means = np.empty(shape=(0,0))
global_std = np.empty(shape=(0,0))

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

def non_zero(arr, arr2):
    new_arr = np.empty(shape=(0,0))
    new_arr2 = np.empty(shape=(0,0))
    for i in range(0, len(arr)):
        if arr[i] != 0.0 and arr2[i] != 0.0:
            new_arr = np.append(new_arr, arr[i])
            new_arr2 = np.append(new_arr2, arr2[i])
    return (new_arr, new_arr2)

def analyze():
    global graph_num_verts
    global global_means
    global global_std
    #print(len(standard_num_edges))
    num_verts = int(num_vertices[0])
    print("Standard Num Verts: " + str(int(num_vertices[0])))
    print("Scaffold Num Verts: " + str(int(scaffold_num_verticies[0])))
    non_zero_standard_plan_length = non_zero(standard_plan_length, scaffold_plan_length)[0]
    non_zero_scaffold_plan_length = non_zero(standard_plan_length, scaffold_plan_length)[1]
    mean_non_zero_standard_plan_length = np.mean(non_zero_standard_plan_length)
    mean_non_zero_scaffold_plan_length = np.mean(non_zero_scaffold_plan_length)
    non_zero_ratios = non_zero_scaffold_plan_length / non_zero_standard_plan_length
    non_zero_ratio_std = np.std(non_zero_ratios)

    mean_ratio = mean_non_zero_scaffold_plan_length / mean_non_zero_standard_plan_length

    global_means = np.append(global_means, mean_ratio)
    global_std = np.append(global_std, non_zero_ratio_std)
    graph_num_verts = np.append(graph_num_verts, num_verts)


def plot():
    plt.xlabel('Number of Vertices')
    plt.ylabel('Scaffold / Standard Path Length Ratio')
    global_means_lower_bound = global_means - global_std * 2.03 / 10
    global_means_upper_bound = global_means + global_std * 2.03 / 10
    plt.plot(graph_num_verts, global_means_upper_bound, 'b--', color='black')
    plt.plot(graph_num_verts, global_means, color='black',
             label='Scaffold / Standard Path Length')
    plt.plot(graph_num_verts, global_means_lower_bound, 'b--', color='black',)
    plt.fill_between(graph_num_verts, global_means_lower_bound, global_means_upper_bound,
                     where=global_means_lower_bound <= global_means_upper_bound,
                     facecolor=(0.8, 0.8, 0.8, 1), interpolate=True, linewidth=0.0)
    # configure legend
    plt.legend(loc=0)
    leg = plt.gca().get_legend()
    ltext = leg.get_texts()
    plt.setp(ltext, fontsize=30)

def handle_file(file_path):
    f = open(file_path, 'r')
    init_storage()

    for line in f:
        # print(line, end='')
        if not line.startswith("has_collision"):
            parse(line)
    analyze()

def main():
    print("Reading from file")
    file_paths = sys.argv[1:]
    for file_path in file_paths:
        print("File: " + file_path)
        handle_file(file_path)
    plot()
    # plt.ylim([800,1400])
    # plt.xlim([500,2500])
    fig.set_size_inches(10.5, 10.5)
    fig.savefig('/home/kyle/test2png.png', dpi=100)
    plt.show()


if __name__ == "__main__":
    main()
