#!/usr/bin/env python3
import subprocess
import numpy as np
import matplotlib.pyplot as plt
import plot_helper
import sys


def run_xstar(num_robots, radius):
    result = subprocess.run([str(e) for e in ['./bin/robocup_eastar', num_robots, scenario_name, inflation, radius, seed ]], stdout=subprocess.PIPE)
    result_lines = result.stdout.decode('UTF-8').split("\n")
        
    if len([l for l in result_lines if "No collision events" in l]) <=  0:
        phase1exp = [int(l.split(' ')[-1]) for l in result_lines if "Phase 1.1" in l]
        phase2exp = [int(l.split(' ')[-1]) for l in result_lines if "Phase 1.2" in l]
        phase3exp = [int(l.split(' ')[-1]) for l in result_lines if "Phase 2" in l]
        assert(len(phase1exp) == len(phase2exp))
        assert(len(phase1exp) == len(phase3exp))
        iteration_exps = [sum(e) for e in zip(phase1exp, phase2exp, phase3exp)]
    
        first_plan_exps = iteration_exps[0]
        total_plan_exps = sum(iteration_exps)
    
    else:
        first_plan_exps = 0
        total_plan_exps = 0

    return first_plan_exps, total_plan_exps

def save_data_to_file(filename, data):
    f = open(filename, 'w')
    f.write(str(data))
    f.close()

def read_saved_data_from_file(filename):
    f = open(filename, 'r')
    data = eval(f.read())
    f.close()
    return data

experiment_name = "eastar_sweep_radius"
scenario_name = "SINGLE_LARGE"
num_robots = 4
inflation = 1.0
seed = 0
iterations = 1
kRunNewTrials = (sys.argv[1].lower() == 'true')

kRadiusValues = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]

xstar_firsts_lst = []
xstar_totals_lst = []

if kRunNewTrials:
    print("===Running new trials")
    for radius in kRadiusValues:
        seed = int(radius * 100 * 53 + 17)
        print("radius:", radius)
        print("X*")
        xstar_first_exps, xstar_total_exps = run_xstar(num_robots, radius)
        xstar_firsts_lst.append(xstar_first_exps)
        xstar_totals_lst.append(xstar_total_exps)

    save_data_to_file("xstar_firsts_lst_{}_{}e{}.txt".format(experiment_name, scenario_name, inflation), xstar_firsts_lst)
    save_data_to_file("xstar_totals_lst_{}_{}e{}.txt".format(experiment_name, scenario_name, inflation), xstar_totals_lst)
else:
    print("===Using saved trials")
    xstar_firsts_lst = read_saved_data_from_file("xstar_firsts_lst_{}_{}e{}.txt".format(experiment_name, scenario_name, inflation))
    xstar_totals_lst = read_saved_data_from_file("xstar_totals_lst_{}_{}e{}.txt".format(experiment_name, scenario_name, inflation))

plot_helper.plot(kRadiusValues, xstar_firsts_lst, 2, "X* First Solution")
plot_helper.plot(kRadiusValues, xstar_totals_lst, 2, "X* Opt. Solution")
plt.ylim(bottom=0)
plt.xticks(kRadiusValues)
plt.xlabel("Radius Values")
plt.ylabel("State Expansions")
#plt.yscale('log')
plot_helper.legend(loc=4)
plot_helper.save_fig("xstar_radius_sweep")
plt.show()
