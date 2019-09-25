#!/usr/bin/env python3
import subprocess
import numpy as np
import matplotlib.pyplot as plt
import plot_helper
import sys
import matplotlib
from matplotlib.ticker import FuncFormatter


def run_xstar(num_robots):
    print(['./bin/robocup_eastar', num_robots, scenario_name, inflation, starting_radius, seed ])
    result = subprocess.run([str(e) for e in ['./bin/robocup_eastar', num_robots, scenario_name, inflation, starting_radius, seed ]], stdout=subprocess.PIPE)
    result_lines = result.stdout.decode('UTF-8').split("\n")
        
    if len([l for l in result_lines if "No collision events" in l]) <=  0:
        individual_global_costs = [int(l.split(' ')[-1]) for l in result_lines if "Individual plan costs" in l]
        individual_solution_costs = [int(l.split(' ')[-1]) for l in result_lines if "Individual solution" in l]
        joint_solution_costs = [int(l.split(' ')[-1]) for l in result_lines if "Joint solution" in l]
        
        iteration_times = [float(l.split(' ')[-1]) for l in result_lines if "Time X* for iteration" in l]
        iteration_percentages = [e / iteration_times[-1] for e in iteration_times]
        print(iteration_percentages)
        
        total_solution_costs = [e1 - e2 + e3 for e1, e2, e3 in zip(individual_global_costs, individual_solution_costs, joint_solution_costs)]
        best_solution_cost = total_solution_costs[-1]
        percentage_total_solution_costs = [e / best_solution_cost for e in total_solution_costs]
        print(percentage_total_solution_costs)
        
        for e in percentage_total_solution_costs:
          print(individual_global_costs)
          print(individual_solution_costs)
          print(joint_solution_costs)
          assert(e >= 1.0)
        
        assert(len(individual_global_costs) == len(individual_solution_costs))
        assert(len(individual_global_costs) == len(joint_solution_costs))
    else:
        print("CANNOT FIND SOLUTION COST!")
        percentage_total_solution_costs = None
        iteration_percentages = None

    return percentage_total_solution_costs, iteration_percentages

def save_data_to_file(filename, data):
    f = open(filename, 'w')
    f.write(str(data))
    f.close()

def read_saved_data_from_file(filename):
    f = open(filename, 'r')
    data = eval(f.read())
    f.close()
    return data

experiment_name = "eastar_quality_vs_iteration"
scenario_name = "SINGLE_LARGE"
num_robots = 5
starting_radius = 2
seed = 0
iterations = 1
kRunNewTrials = (sys.argv[1].lower() == 'true')
inflation = 1.0
kNumIterations = 20

xstar_firsts_lst = []
xstar_totals_lst = []

if kRunNewTrials:
    print("===Running new trials")
    percentage_solutions = []
    percentage_iterations = []
    for iteration in range(kNumIterations):
        seed = int(iteration * 100 * 53 + 17)
        print("X*")
        percentage_total_solution_costs, iteration_percentages = run_xstar(num_robots)
        if percentage_total_solution_costs is not None:
          percentage_solutions.append(percentage_total_solution_costs)
          percentage_iterations.append(iteration_percentages)
        
    final_precentage_solutions = [sum(e) / len(e) for e in zip(*percentage_solutions)]
    final_percentage_iterations = [sum(e) / len(e) for e in zip(*percentage_iterations)]
    print("Final percentage iterations:", final_percentage_iterations)
    
    save_data_to_file("xstar_soltion_quality_{}.txt".format(experiment_name, scenario_name, inflation), final_precentage_solutions)
    save_data_to_file("final_percentage_iterations_{}.txt".format(experiment_name, scenario_name, inflation), final_percentage_iterations)
else:
    print("===Using saved trials")
    final_precentage_solutions = read_saved_data_from_file("xstar_soltion_quality_{}.txt".format(experiment_name, scenario_name, inflation))
    final_percentage_iterations = read_saved_data_from_file("final_percentage_iterations_{}.txt".format(experiment_name, scenario_name, inflation))

xs = list(range(1, len(final_precentage_solutions) + 1))

plot_helper.plot(xs, final_percentage_iterations, 2, None)

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

#plot_helper.plot(kAgentCount, xstar_totals_lst, 2, "X* Opt. Solution")
plt.xticks(xs)
plt.xlabel("Iteration Count")
plt.ylabel("Percentage of Total Computation Time")
#plt.yscale('log')
#plot_helper.legend(loc=2)
plot_helper.save_fig("xstar_computation_time_vs_iteration")
plt.show()
