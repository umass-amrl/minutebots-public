#!/usr/bin/env python3
import sys
import subprocess
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import plot_helper
from matplotlib.ticker import FuncFormatter

def run_xstar(num_robots, iterations):
    num_collisions = 0
    xstar_firsts = []
    xstar_totals = []
    for i in range(iterations):
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
  
def run_eager_xstar(num_robots, iterations):
    num_collisions = 0
    xstar_firsts = []
    xstar_totals = []
    for i in range(iterations):
        result = subprocess.run([str(e) for e in ['./bin/robocup_eager_eastar', num_robots, scenario_name, inflation, starting_radius, seed ]], stdout=subprocess.PIPE)
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

def run_mstar(num_robots, iterations):
    mstar_totals = []
    for i in range(iterations):
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
  
def get_percent_different(ci1, ci2):
    assert(len(ci1) == len(ci2))
    m1 = [e[1] for e in ci1]
    m2 = [e[1] for e in ci2]
    return [e1 / e2 for e1, e2 in zip(m1, m2)]

def save_data_to_file(filename, data):
    f = open(filename, 'w')
    f.write(str(data))
    f.close()

def read_saved_data_from_file(filename):
    f = open(filename, 'r')
    data = eval(f.read())
    f.close()
    return data

experiment_name = "lazy_vs_eager"
scenario_name = "SINGLE_LARGE"
inflation = 1.0
starting_radius = 3
seed = 0
iterations = 20
kRunNewTrials = (sys.argv[1].lower() == 'true')

kNumRobotsList = [2, 3, 4, 5]

xstar_firsts_lst = []
xstar_totals_lst = []
eager_xstar_firsts_lst = []
eager_xstar_totals_lst = []

if kRunNewTrials:
    print("===Running new trials")

    for n in kNumRobotsList:
        seed = n * 53 + 17
        print("Num robots:", n)
        print("X*")
        xstar_firsts, xstar_totals = run_xstar(n, iterations)
        xstar_firsts_lst.append(sorted(xstar_firsts))
        xstar_totals_lst.append(sorted(xstar_totals))
        
        print("Eager X*")
        eager_xstar_firsts, eager_xstar_totals = run_eager_xstar(n, iterations)
        eager_xstar_firsts_lst.append(sorted(eager_xstar_firsts))
        eager_xstar_totals_lst.append(sorted(eager_xstar_totals))
        
    save_data_to_file("xstar_firsts_lst_{}e{}.txt".format(scenario_name, inflation), xstar_firsts_lst)
    save_data_to_file("xstar_totals_lst_{}e{}.txt".format(scenario_name, inflation), xstar_totals_lst)
    save_data_to_file("eager_xstar_firsts_lst_{}e{}.txt".format(scenario_name, inflation), eager_xstar_firsts_lst)
    save_data_to_file("eager_xstar_totals_lst_{}e{}.txt".format(scenario_name, inflation), eager_xstar_totals_lst)
    save_data_to_file("mstar_totals_lst_{}e{}.txt".format(scenario_name, inflation), mstar_totals_lst)

    xstar_firsts_lst = [get_CI(e) for e in xstar_firsts_lst]
    xstar_totals_lst = [get_CI(e) for e in xstar_totals_lst]
    eager_xstar_firsts_lst = [get_CI(e) for e in eager_xstar_firsts_lst]
    eager_xstar_totals_lst = [get_CI(e) for e in eager_xstar_totals_lst]

    save_data_to_file("xstar_firsts_lst_ci_{}e{}.txt".format(scenario_name, inflation), xstar_firsts_lst)
    save_data_to_file("xstar_totals_lst_ci_{}e{}.txt".format(scenario_name, inflation), xstar_totals_lst)
    save_data_to_file("eager_xstar_firsts_lst_ci_{}e{}.txt".format(scenario_name, inflation), eager_xstar_firsts_lst)
    save_data_to_file("eager_xstar_totals_lst_ci_{}e{}.txt".format(scenario_name, inflation), eager_xstar_totals_lst)
    
else:
    print("===Using saved trials")

    xstar_firsts_lst = read_saved_data_from_file("xstar_firsts_lst_ci_{}e{}.txt".format(scenario_name, inflation))
    xstar_totals_lst = read_saved_data_from_file("xstar_totals_lst_ci_{}e{}.txt".format(scenario_name, inflation))
    eager_xstar_firsts_lst = read_saved_data_from_file("eager_xstar_firsts_lst_ci_{}e{}.txt".format(scenario_name, inflation))
    eager_xstar_totals_lst = read_saved_data_from_file("eager_xstar_totals_lst_ci_{}e{}.txt".format(scenario_name, inflation))


#plot_helper.plot_ci(kNumRobotsList, xstar_firsts_lst, 3, "X* First Search")
#plot_helper.plot_ci(kNumRobotsList, xstar_totals_lst, 3, "X* Full Search")
#plot_helper.plot_ci(kNumRobotsList, mstar_totals_lst, 3, "M* First/Full Search")
plot_helper.plot(kNumRobotsList, get_percent_different(eager_xstar_firsts_lst, xstar_firsts_lst), 2, "First Solution Improvement")
plot_helper.plot(kNumRobotsList, get_percent_different(eager_xstar_totals_lst, xstar_totals_lst), 2, "Opt. Solution Improvement")


def to_percent(y, position):
    # Ignore the passed in position. This has the effect of scaling the default
    # tick locations.
    s = str(100 * y)

    # The percent symbol needs escaping in latex
    if matplotlib.rcParams['text.usetex'] is True:
        return s + r'$\%$'
    else:
        return s + '%'

plt.ylim(bottom=0)
      
formatter = FuncFormatter(to_percent)

# Set the formatter
plt.gca().yaxis.set_major_formatter(formatter)

plt.xticks(kNumRobotsList)
plt.xlabel("Agent Count")
plt.ylabel("Lazy vs Eager X* Mean Runtime Improvement")
#plt.yscale('log')
plot_helper.legend(loc=3)
plot_helper.save_fig("xstar_eager_vs_lazy")
plt.show()
