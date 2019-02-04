#!/usr/bin/env python3
import os
import sys
import glob
import shutil
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib
from scipy.stats import t

font = {'family' : 'cmr10',
        'weight' : 'bold',
        'size'   : 32}

matplotlib.rc('font', **font)


for filename in glob.glob(os.path.join("build/", '*.py')):
    shutil.copy(filename, 'scripts/python/')

import performance_pb2
from google.protobuf import text_format

if len(sys.argv) < 3:
    print("Usage:<test type> <performance file(s)>")
    exit(-1)

type = sys.argv[1]

for f_name in sys.argv[2:]:
    f = open(f_name, 'rb') 
    scenario = text_format.Parse(f.read(), performance_pb2.Scenerio())
    print("Scenario: {}".format(scenario.scenerio_name))

    # RRT Stuff
    scaffold_bias = []

    scaffold_runtime = []
    scaffold_std_dev = []
    no_scaffold_runtime = []
    no_scaffold_std_dev = []

    scaffold_samples_distance, scaffold_samples_distance_lower, scaffold_samples_distance_upper = [], [], []
    no_scaffold_samples_distance, no_scaffold_samples_distance_lower, no_scaffold_samples_distance_upper = [], [], []

    scaffold_path_length = []
    scaffold_path_length_std_dev = []
    scaffold_path_length_max = []
    scaffold_path_length_min = []
    no_scaffold_path_length = []
    no_scaffold_path_length_std_dev = []
    no_scaffold_path_length_max = []
    no_scaffold_path_length_min = []

    for configuration in scenario.configuration_list:
        runtimes = []
        runtimes_scaffold = []
        path_lengths = []
        path_lengths_scaffold = []
        samples_to_distances = []
        samples_to_distances_scaffold = []
        
        for trial in configuration.trial_list:
            if not trial.scaffold:
                lruntimes = [t for t in trial.trial_times_list]
                runtimes.append(np.mean(lruntimes))
                samples_to_distances.append(trial.samples)# / trial.path_distance)
                path_lengths.append(trial.path_length)
            else:
                lruntimes = [t for t in trial.trial_times_list]
                runtimes_scaffold.append(np.mean(lruntimes))
                samples_to_distances_scaffold.append(trial.samples )#/ trial.path_distance)
                path_lengths_scaffold.append(trial.path_length)


        samples_to_distances_scaffold.sort()
        samples_to_distances.sort()

        runtimes_scaffold.sort()
        runtimes.sort()

        path_lengths.sort()
        path_lengths_scaffold.sort()

        def lower_upper_diff(lst):
            lst.sort()
            return lst[int(len(lst) * 0.95)] - lst[int(len(lst) * 0.05)]

        print("No Scaffold Samples mean: {} ci: {}".format(np.mean(samples_to_distances), lower_upper_diff(samples_to_distances)))
        print("Scaffold Samples mean: {} ci: {}".format(np.mean(samples_to_distances_scaffold), lower_upper_diff(samples_to_distances_scaffold)))

        print("No Scaffold Runtime (millis) mean: {} ci: {}".format(np.mean([x * 1000 for x in runtimes]), lower_upper_diff([x * 1000 for x in runtimes])))
        print("Scaffold Runtime (millis) mean: {} ci: {}".format(np.mean([x * 1000 for x in runtimes_scaffold]), lower_upper_diff([x * 1000 for x in runtimes_scaffold])))

        print("No scaffold path length mean {} ci {}".format(np.mean(path_lengths), lower_upper_diff(path_lengths)))
        print("Scaffold path length mean {} ci {}".format(np.mean(path_lengths_scaffold), lower_upper_diff(path_lengths_scaffold)))

        scaffold_bias.append(configuration.rrt_config.scaffold_bias)

        scaffold_runtime.append(np.mean(runtimes_scaffold))
        scaffold_std_dev.append(np.std(runtimes_scaffold))
        no_scaffold_runtime.append(np.mean(runtimes))
        no_scaffold_std_dev.append(np.std(runtimes))

        samples_to_distances_scaffold.sort()
        samples_to_distances.sort()

        scaffold_samples_distance.append(np.mean(samples_to_distances_scaffold))
        scaffold_samples_distance_lower.append(samples_to_distances_scaffold[int(len(samples_to_distances_scaffold) * 0.05)])
        scaffold_samples_distance_upper.append(samples_to_distances_scaffold[int(len(samples_to_distances_scaffold) * 0.95)])
        no_scaffold_samples_distance.append(np.mean(samples_to_distances))
        no_scaffold_samples_distance_lower.append(samples_to_distances[int(len(samples_to_distances) * 0.05)])
        no_scaffold_samples_distance_upper.append(samples_to_distances[int(len(samples_to_distances) * 0.95)])

        scaffold_path_length.append(np.mean(path_lengths_scaffold))
        scaffold_path_length_std_dev.append(np.mean(path_lengths_scaffold))
        path_lengths_scaffold.sort()
        path_lengths_scaffold_max = path_lengths_scaffold[int(len(path_lengths_scaffold) * 0.95)]
        path_lengths_scaffold_min = path_lengths_scaffold[int(len(path_lengths_scaffold) * 0.05)]
        scaffold_path_length_max.append(path_lengths_scaffold_max)
        scaffold_path_length_min.append(path_lengths_scaffold_min)
        no_scaffold_path_length.append(np.mean(path_lengths))
        no_scaffold_path_length_std_dev.append(np.std(path_lengths))
        path_lengths.sort()
        path_lengths_max = path_lengths[int(len(path_lengths) * 0.95)]
        path_lengths_min = path_lengths[int(len(path_lengths) * 0.05)]
        no_scaffold_path_length_max.append(path_lengths_max)
        no_scaffold_path_length_min.append(path_lengths_min)

        # print("No scaffold Runtime mean: {} std: {}".format(np.mean(runtimes), np.std(runtimes)))
        # print("No scaffold samples/distance: {} std: {}".format(np.mean(samples_to_distances), np.std(samples_to_distances)))
        # print("No scaffold Path length: {} std: {}".format(np.mean(path_lengths), np.std(path_lengths)))
        
        # print("Scaffold Runtime mean: {} std: {}".format(np.mean(runtimes_scaffold), np.std(runtimes_scaffold)))
        # print("Scaffold samples/distance: {} std: {}".format(np.mean(samples_to_distances_scaffold), np.std(samples_to_distances_scaffold)))
        # print("Scaffold Path length: {} std: {}".format(np.mean(path_lengths_scaffold), np.std(path_lengths_scaffold)))


    if type == "number samples":
        lower_bound = no_scaffold_samples_distance_lower
        upper_bound = no_scaffold_samples_distance_upper
        plt.plot(scaffold_bias, lower_bound, color='black')
        plt.plot(scaffold_bias, upper_bound, color='black')
        plt.fill_between(scaffold_bias, lower_bound, upper_bound, facecolor=(0.8, 0.8, 0.8, 1))
        plt.plot(scaffold_bias, no_scaffold_samples_distance, color='black')
    
        lower_bound = scaffold_samples_distance_lower
        upper_bound = scaffold_samples_distance_upper
    
        plt.plot(scaffold_bias, lower_bound, color='red')
        plt.plot(scaffold_bias, upper_bound, color='red')
        plt.fill_between(scaffold_bias, lower_bound, upper_bound, facecolor=(1, 0.8, 0.8, 1))
                         # where=lower_bound <= upper_bound,
                         # facecolor=(1, 0.8, 0.8, 1), interpolate=True, linewidth=0.0, hatch="X",
                         # edgecolor=(1, 0.6, 0.6, 1))
        plt.plot(scaffold_bias, scaffold_samples_distance, color='red')
        plt.ylabel('Number of samples (95% confidence)')

    elif type == "path length":
        lower_bound = no_scaffold_path_length_min
        upper_bound = no_scaffold_path_length_max
        plt.plot(scaffold_bias, lower_bound, color='black')
        plt.plot(scaffold_bias, upper_bound, color='black')
        plt.fill_between(scaffold_bias, lower_bound, upper_bound, facecolor=(0.8, 0.8, 0.8, 1))
        plt.plot(scaffold_bias, no_scaffold_path_length, color='black')

        lower_bound = scaffold_path_length_min
        upper_bound = scaffold_path_length_max
    
        plt.plot(scaffold_bias, lower_bound, color='red')
        plt.plot(scaffold_bias, upper_bound, color='red')
        plt.fill_between(scaffold_bias, lower_bound, upper_bound, facecolor=(1, 0.8, 0.8, 1))
                         # where=lower_bound <= upper_bound,
                         # facecolor=(1, 0.8, 0.8, 1), interpolate=True, linewidth=0.0, hatch="X",
                         # edgecolor=(1, 0.6, 0.6, 1))
        plt.plot(scaffold_bias, scaffold_path_length, color='red')

        plt.ylabel('Path length (95% confidence)')

#    plt.title("Scenario: {}".format(scenario.scenerio_name))

    plt.xlabel('Scaffold bias')
    red_patch = patches.Patch(color='red', label='Scaffolded')
    black_patch = patches.Patch(color='black', label='Unscaffolded')
#    plt.legend(handles=[red_patch, black_patch])
    fig = plt.gcf()
    ax = fig.gca()
    fig.set_size_inches(15, 15)
    fig.savefig('graphrrt{}{}.png'.format(type, scenario.scenerio_name))
    #plt.ylim([0.8,1.2])
    plt.show()

