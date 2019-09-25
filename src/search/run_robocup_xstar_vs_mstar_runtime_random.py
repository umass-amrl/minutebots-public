#!/usr/bin/env python3
import subprocess
import numpy as np
import matplotlib.pyplot as plt
import plot_helper
import sys


def run_xstar(num_robots, iterations, starter_seed):
    num_collisions = 0
    xstar_firsts = []
    xstar_totals = []
    for i in range(iterations):
        seed = (starter_seed * i) % 10000
        result = subprocess.run([str(e) for e in ['./bin/robocup_eastar', num_robots, scenario_name, inflation, starting_radius, seed ]], stdout=subprocess.PIPE)
        result_lines = result.stdout.decode('UTF-8').split("\n")
        total_plan_time = float([l for l in result_lines if "Total runtime" in l][0].split(' ')[-1])
        if len([l for l in result_lines if "No collision events" in l]) <=  0:
            first_plan_time = float([l for l in result_lines if "iteration 1" in l][0].split(' ')[-1])
            num_collisions += 1
        else:
            first_plan_time = total_plan_time
        xstar_firsts.append(first_plan_time)
        xstar_totals.append(total_plan_time)

    print("Mean, std first time (ms)", np.mean(xstar_firsts), np.std(xstar_firsts))
    print("Mean, std total time (ms)", np.mean(xstar_totals), np.std(xstar_totals))
    print(num_collisions)

    return xstar_firsts, xstar_totals

def run_mstar(num_robots, iterations, starter_seed):
    mstar_totals = []
    for i in range(iterations):
        seed = (starter_seed * i) % 10000
        result = subprocess.run([str(e) for e in ['./bin/robocup_mstar', num_robots, scenario_name, inflation, starting_radius, seed ]], stdout=subprocess.PIPE)
        result_lines = result.stdout.decode('UTF-8').split("\n")
        total_plan_time = float([l for l in result_lines if "Planning time" in l][0].split(' ')[-1])
        mstar_totals.append(total_plan_time)

    print("Mean, std total time (ms)", np.mean(mstar_totals), np.std(mstar_totals))

    return mstar_totals

def get_CI(lst):
    assert(type(lst) == list)
    lst.sort()
    lower = lst[int(len(lst) * 0.05)]
    mid = lst[int(len(lst) * 0.5)]
    upper = lst[int(len(lst) * 0.95)]
    return lower, mid, upper

def save_data_to_file(filename, data):
    f = open(filename, 'w')
    f.write(str(data))
    f.close()

def read_saved_data_from_file(filename):
    f = open(filename, 'r')
    data = eval(f.read())
    f.close()
    return data

experiment_name = "mstar_vs_eastar_runtime_random_field"
scenario_name = "ROBOCUP_OBSTACLES"
inflation = 1.0
starting_radius = 3
seed = 0
iterations = 100
kRunNewTrials = (sys.argv[1].lower() == 'true')

kNumRobotsList = [2, 3, 4, 5, 6, 7, 8]

xstar_firsts_lst = []
xstar_totals_lst = []

mstar_totals_lst = []

if kRunNewTrials:
    print("===Running new trials")
    for n in kNumRobotsList:
        seed = 53
        print("Num robots:", n)
        print("X*")
        xstar_firsts, xstar_totals = run_xstar(n, iterations, seed)
        xstar_firsts_lst.append(sorted(xstar_firsts))
        xstar_totals_lst.append(sorted(xstar_totals))

        print("M*")
        mstar_totals = run_mstar(n, iterations, seed)
        mstar_totals_lst.append(sorted(mstar_totals))

    save_data_to_file("xstar_firsts_lst_{}_{}e{}.txt".format(experiment_name, scenario_name, inflation), xstar_firsts_lst)
    save_data_to_file("xstar_totals_lst_{}_{}e{}.txt".format(experiment_name, scenario_name, inflation), xstar_totals_lst)
    save_data_to_file("mstar_totals_lst_{}_{}e{}.txt".format(experiment_name, scenario_name, inflation), mstar_totals_lst)

    xstar_firsts_lst = [get_CI(e) for e in xstar_firsts_lst]
    xstar_totals_lst = [get_CI(e) for e in xstar_totals_lst]
    mstar_totals_lst = [get_CI(e) for e in mstar_totals_lst]

    save_data_to_file("xstar_firsts_lst_ci_{}_{}e{}.txt".format(experiment_name, scenario_name, inflation), xstar_firsts_lst)
    save_data_to_file("xstar_totals_lst_ci_{}_{}e{}.txt".format(experiment_name, scenario_name, inflation), xstar_totals_lst)
    save_data_to_file("mstar_totals_lst_ci_{}_{}e{}.txt".format(experiment_name, scenario_name, inflation), mstar_totals_lst)
else:
    print("===Using saved trials")
    xstar_firsts_lst = read_saved_data_from_file("xstar_firsts_lst_ci_{}_{}e{}.txt".format(experiment_name, scenario_name, inflation))
    xstar_totals_lst = read_saved_data_from_file("xstar_totals_lst_ci_{}_{}e{}.txt".format(experiment_name, scenario_name, inflation))
    mstar_totals_lst = read_saved_data_from_file("mstar_totals_lst_ci_{}_{}e{}.txt".format(experiment_name, scenario_name, inflation))

plot_helper.plot_ci(kNumRobotsList, xstar_totals_lst, 3, "X* Opt. Solution")
plot_helper.plot_ci(kNumRobotsList, xstar_firsts_lst, 3, "X* First Solution")
plot_helper.plot_ci(kNumRobotsList, mstar_totals_lst, 3, "M* Solution")
plt.xticks(kNumRobotsList)
plt.xlabel("Agent Count")
plt.ylabel("Runtime (ms)")
plt.yscale('log')
plot_helper.legend(loc=2)
plot_helper.save_fig("xstar_vs_mstar_agents_vs_runtime_random_field")
plt.show()
