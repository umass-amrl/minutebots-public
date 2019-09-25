#!/usr/bin/env python3
import subprocess
import numpy as np
import matplotlib.pyplot as plt
import plot_helper
import sys


def run_xstar(num_robots):
    result = subprocess.run([str(e) for e in ['./bin/robocup_eastar', num_robots, scenario_name, inflation, starting_radius, seed ]], stdout=subprocess.PIPE)
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
  
def run_nwastar(num_robots):
    result = subprocess.run([str(e) for e in ['./bin/robocup_nwastar', num_robots, scenario_name, inflation, starting_radius, seed ]], stdout=subprocess.PIPE)
    result_lines = result.stdout.decode('UTF-8').split("\n")
        
    if len([l for l in result_lines if "No collision events" in l]) <=  0:
        phase3exp = [int(l.split(' ')[-1]) for l in result_lines if "Phase 2" in l]
        iteration_exps = phase3exp
    
        first_plan_exps = iteration_exps[0]
        total_plan_exps = sum(iteration_exps)
    
    else:
        first_plan_exps = 0
        total_plan_exps = 0

    return first_plan_exps, total_plan_exps
  
def run_astar(num_robots):
    result = subprocess.run([str(e) for e in ['./bin/robocup_eastar', num_robots, scenario_name, inflation, 1000, seed ]], stdout=subprocess.PIPE)
    result_lines = result.stdout.decode('UTF-8').split("\n")
        
    if len([l for l in result_lines if "No collision events" in l]) <=  0:
        phase1exp = [int(l.split(' ')[-1]) for l in result_lines if "Phase 1.1" in l]
        phase2exp = [int(l.split(' ')[-1]) for l in result_lines if "Phase 1.2" in l]
        phase3exp = [int(l.split(' ')[-1]) for l in result_lines if "Phase 2" in l]
        assert(len(phase1exp) == len(phase2exp))
        assert(len(phase1exp) == len(phase3exp))
        iteration_exps = [sum(e) for e in zip(phase1exp, phase2exp, phase3exp)]
    
        total_plan_exps = sum(iteration_exps)
    
    else:
        total_plan_exps = 0

    return total_plan_exps

def save_data_to_file(filename, data):
    f = open(filename, 'w')
    f.write(str(data))
    f.close()

def read_saved_data_from_file(filename):
    f = open(filename, 'r')
    data = eval(f.read())
    f.close()
    return data

experiment_name = "xstar_vs_nwastar_vs_astar"
scenario_name = "SINGLE_LARGE"
num_robots = 3
starting_radius = 2
seed = 0
iterations = 20
kRunNewTrials = (sys.argv[1].lower() == 'true')
inflation = 1.0
kAgentCount = [2,3,4,5]

xstar_firsts_lst = []
xstar_totals_lst = []
nwastar_firsts_lst = []
nwastar_totals_lst = []
astar_totals_lst = []

if kRunNewTrials:
    print("===Running new trials")
    for num_robots in kAgentCount:
        seed = int(num_robots * 100 * 53 + 17)
        print("Agent Count:", num_robots)
        print("X*")
        xstar_first_exps, xstar_total_exps = run_xstar(num_robots)
        nwastar_first_exps, nwastar_total_exps = run_nwastar(num_robots)
        astar_total_exps = run_astar(num_robots)
        xstar_firsts_lst.append(xstar_first_exps)
        xstar_totals_lst.append(xstar_total_exps)
        nwastar_firsts_lst.append(nwastar_first_exps)
        nwastar_totals_lst.append(nwastar_total_exps)
        astar_totals_lst.append(astar_total_exps)

    save_data_to_file("xstar_firsts_lst_{}_{}e{}.txt".format(experiment_name, scenario_name, inflation), xstar_firsts_lst)
    save_data_to_file("xstar_totals_lst_{}_{}e{}.txt".format(experiment_name, scenario_name, inflation), xstar_totals_lst)
    save_data_to_file("nwastar_firsts_lst_{}_{}e{}.txt".format(experiment_name, scenario_name, inflation), nwastar_firsts_lst)
    save_data_to_file("nwastar_totals_lst_{}_{}e{}.txt".format(experiment_name, scenario_name, inflation), nwastar_totals_lst)
    save_data_to_file("astar_totals_lst_{}_{}e{}.txt".format(experiment_name, scenario_name, inflation), astar_totals_lst)
else:
    print("===Using saved trials")
    xstar_firsts_lst = read_saved_data_from_file("xstar_firsts_lst_{}_{}e{}.txt".format(experiment_name, scenario_name, inflation))
    xstar_totals_lst = read_saved_data_from_file("xstar_totals_lst_{}_{}e{}.txt".format(experiment_name, scenario_name, inflation))
    nwastar_firsts_lst = read_saved_data_from_file("nwastar_firsts_lst_{}_{}e{}.txt".format(experiment_name, scenario_name, inflation))
    nwastar_totals_lst = read_saved_data_from_file("nwastar_totals_lst_{}_{}e{}.txt".format(experiment_name, scenario_name, inflation))
    astar_totals_lst = read_saved_data_from_file("astar_totals_lst_{}_{}e{}.txt".format(experiment_name, scenario_name, inflation))

plot_helper.plot(kAgentCount, xstar_firsts_lst, 4, "X*/NWA* First Solution")
plot_helper.plot(kAgentCount, xstar_totals_lst, 4, "X* Opt. Solution")
plot_helper.plot(kAgentCount, nwastar_totals_lst, 4, "NWA* Opt. Solution")
plot_helper.plot(kAgentCount, astar_totals_lst, 4, "A* Solution")
print("A* values:", astar_totals_lst)
print("X* values:", xstar_totals_lst)
print("X* First values:", xstar_firsts_lst)
print("NWA* values:", nwastar_totals_lst)
plt.ylim(bottom=5)
plt.xticks(kAgentCount)
plt.xlabel("Agent Count")
plt.ylabel("Number of Expansions")
plot_helper.legend(2)
plt.yscale('log')
print("Saving")
plot_helper.save_fig(experiment_name+"_agents_vs_expansions")
print("Saved!")
plt.show()
