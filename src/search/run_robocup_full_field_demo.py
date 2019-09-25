#!/usr/bin/env python3
import subprocess
import numpy as np
import matplotlib.pyplot as plt
import plot_helper
import sys
import time
from functools import reduce

modes = {"robocup" : ("ROBOCUP_OBSTACLES", (-5000.0, 1000.0), (-4000.0, 1000.0)), 
         "circle" : ("SINGLE_LARGE", (-2000.0, 2000.0), (-2000.0, 2000.0)) }

if len(sys.argv) < 4:
    print("Usage:", sys.argv[0], "[heuristic inflation] [num_robots] [mode] [starting seed]")
    exit(1)
    
inflation = float(sys.argv[1])
kNumRobots = int(sys.argv[2])
mode_info = modes[sys.argv[3]]
starter_seed = int(sys.argv[4])

def extract_position_from_line(l):
    f, s = l.split()[-2:]
    return (float(f[:-1]), float(s))

def to_tuple(string):
    split = string.split(",")
    return (float(split[0]) * 103, float(split[1]) * 103)
    

def extract_individual_paths(path_line):
    path_line = path_line.replace("Path: ", "")
    path_line_list = [e.split(") (") for e in path_line.split(";")]
    path_line_list = [ [e.replace("(", "").replace(")", "") for e in s] for s in path_line_list]
    path_line_list = [ [to_tuple(e) for e in s] for s in path_line_list[:-1]]
    individual_path_list = zip(*path_line_list)
    individual_path_list = [list(e) for e in individual_path_list]
    return individual_path_list

def run_xstar(num_robots, seed):
    collisions = False
    result = subprocess.run([str(e) for e in ['./bin/robocup_eastar', num_robots, scenario_name, inflation, starting_radius, seed ]], stdout=subprocess.PIPE)
    result_lines = result.stdout.decode('UTF-8').split("\n")
    total_plan_time = float([l for l in result_lines if "Total runtime" in l][0].split(' ')[-1])
    if len([l for l in result_lines if "No collision events" in l]) <=  0:
        print("Collisions!")
        collisions = True
        first_plan_time = float([l for l in result_lines if "iteration 1" in l][0].split(' ')[-1])
    else:
        print("No collisions!")
        collisions = False
        first_plan_time = total_plan_time
        
    our_team_positions = [extract_position_from_line(l) for l in result_lines if "Our team:" in l]
    our_team_goals = [extract_position_from_line(l) for l in result_lines if "Our team goals:" in l]
    their_team_positions = [extract_position_from_line(l) for l in result_lines if "Their team:" in l]
    
    
    individual_path_list = []
    for e in [extract_individual_paths(l) for l in result_lines if "Path:" in l]:
        individual_path_list += e


    return collisions, first_plan_time, total_plan_time, our_team_positions, our_team_goals, their_team_positions, individual_path_list

experiment_name = "xstar_full_field"
scenario_name = mode_info[0]
starting_radius = 3

def get_color(idx):
    return ['blue', 'peachpuff', 'olive', 'lawngreen', 'c', 'dodgerblue'][idx]

def run_example(seed):
    xlim = mode_info[1]
    ylim = mode_info[2]
    plt.gca().clear()
    print("Running with seed: {}".format(seed))
    collisions, xstar_first, xstar_total, our_team_positions, our_team_goals, their_team_positions, individual_path_list = run_xstar(kNumRobots, seed)
    plot_helper.add_dots_to_robocup_field(our_team_positions, 90, 'red', xlim, ylim)
    plot_helper.add_dots_to_robocup_field(our_team_goals, 70, 'black', xlim, ylim)
    plot_helper.add_dots_to_robocup_field(their_team_positions, 90, 'lightgreen', xlim, ylim)
    for idx, p in enumerate(individual_path_list):
        plot_helper.add_dots_to_robocup_field(p, 20, get_color(idx), xlim, ylim)
    plt.title("Seed: {}  ".format(seed) + "Inflation: {}".format(inflation) + "  {}".format("Collisions" if collisions else "No Collisions") + "  First solution (ms): {}".format(xstar_first) + "  Full solution (ms): {}".format(xstar_total))

seed_list = []
seed_list_counter = 0

def press(event):
    global seed_list
    global seed_list_counter
    if event.key == 'right':
        if seed_list_counter == 0:
            seed = int(time.time() * 100000) % 10000
            seed_list.append(seed)
            run_example(seed)
        else:
            seed_list_counter += 1
            run_example(seed_list[seed_list_counter])
    elif event.key == 'left':
        if seed_list_counter == 0:
            seed_list_counter -= 1
        seed_list_counter = max(-len(seed_list), seed_list_counter - 1)
        run_example(seed_list[seed_list_counter])
    elif event.key == 'escape':
        exit(0)
    else:
        print(event.key)
    print(seed_list)
    print(seed_list_counter)

plt.gcf().canvas.mpl_connect('key_press_event', press)
run_example(starter_seed)
seed_list.append(starter_seed)
plt.show()
