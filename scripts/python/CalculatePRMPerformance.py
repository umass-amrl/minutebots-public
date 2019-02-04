#!/usr/bin/env python3
import os
import sys
import glob
import shutil
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib
from collections import defaultdict
from scipy.stats import t

font = {'family' : 'cmr10',
        'weight' : 'bold',
        'size'   : 32}

matplotlib.rc('font', **font)

for filename in glob.glob(os.path.join("build/", '*.py')):
    shutil.copy(filename, 'scripts/python/')

import performance_pb2
from google.protobuf import text_format

if len(sys.argv) < 2:
    print("Usage:<test type> <performance file(s)>")
    exit(-1)

type = sys.argv[1]

def flatten(l):
    return [item for sublist in l for item in sublist]

for f_name in sys.argv[2:]:
    f = open(f_name, 'rb') 
    scenario = text_format.Parse(f.read(), performance_pb2.Scenerio())
    print("Scenario: {}".format(scenario.scenerio_name))

    
    total_verticies_to_runtimes_no_scaffold = defaultdict(list)
    total_verticies_to_runtimes_scaffold = defaultdict(list)

    total_verticies_to_quality_scaffold = defaultdict(list)
    total_verticies_to_quality_no_scaffold = defaultdict(list)


    total_verticies_to_percent_failure_scaffold = defaultdict(list)
    total_verticies_to_percent_failure_no_scaffold = defaultdict(list)

    connect_radius_to_quality_scaffold = defaultdict(list)
    connect_radius_to_quality_no_scaffold = defaultdict(list)

    connect_radius_to_runtimes_scaffold = defaultdict(list)
    connect_radius_to_runtimes_no_scaffold = defaultdict(list)

    obstacle_count_to_runtimes_no_scaffold = defaultdict(list)
    obstacle_count_to_runtimes_scaffold = defaultdict(list)

    for configuration in scenario.configuration_list:
        runtimes_no_scaffold = []
        runtimes_scaffold = []
        path_lengths_no_scaffold = []
        path_lengths_scaffold = []

        failed_seed_list = []
        scaffold_failures = 0
        no_scaffold_failures = 0

        for trial in configuration.trial_list:

            if trial.path_found is False:
                failed_seed_list.append(trial.seed)
                if trial.scaffold:
                    scaffold_failures += 1
                else:
                    no_scaffold_failures += 1

        for trial in configuration.trial_list:
            if not trial.scaffold:
                lruntimes = [t for t in trial.update_times_list]
                runtimes_no_scaffold.append(np.mean(lruntimes))
                if trial.seed not in failed_seed_list:
                    path_lengths_no_scaffold.append(trial.path_length)
            else:
                lruntimes = [t for t in trial.update_times_list]
                runtimes_scaffold.append(np.mean(lruntimes))
                if trial.seed not in failed_seed_list:
                    path_lengths_scaffold.append(trial.path_length)

        def lower_upper_diff(lst):
            lst.sort()
            return lst[int(len(lst) * 0.95)] - lst[int(len(lst) * 0.05)]

        print("No scaffold path len {}, ci {}".format(np.mean(path_lengths_no_scaffold), lower_upper_diff(path_lengths_no_scaffold)))
        print("Scaffold path len {}, ci {}".format(np.mean(path_lengths_scaffold), lower_upper_diff(path_lengths_scaffold)))

        runtimes_no_scaffold.sort()
        runtimes_scaffold.sort()

        print("No scaffold runtimes mean {} ci {}".format(np.mean([x * 1000 for x in runtimes_no_scaffold]), lower_upper_diff([x * 1000 for x in runtimes_no_scaffold])))
        print("Scaffold runtimes mean {} ci {}".format(np.mean([x * 1000 for x in runtimes_scaffold]), lower_upper_diff([x * 1000 for x in runtimes_scaffold])))

        # print("No scaffold runtimes {},  +/-{}  {} <> {} ".format(np.mean(runtimes_no_scaffold), np.std(runtimes_no_scaffold) * 1.96,
        #                                       runtimes_no_scaffold[int(len(runtimes_no_scaffold) * 0.95)],
        #                                       runtimes_no_scaffold[int(len(runtimes_no_scaffold) * 0.05)]))
        # print("Scaffold runtimes {},  +/-{}  {} <> {}".format(np.mean(runtimes_scaffold), np.std(runtimes_scaffold) * 1.96,
        #                                      runtimes_scaffold[int(len(runtimes_scaffold) * 0.95)],
        #                                      runtimes_scaffold[int(len(runtimes_scaffold) * 0.05)]
        # ))h

        total_verticies_to_runtimes_scaffold[configuration.prm_config.total_verticies].append(runtimes_scaffold)
        total_verticies_to_runtimes_no_scaffold[configuration.prm_config.total_verticies].append(runtimes_no_scaffold)

        obstacle_count_to_runtimes_scaffold[configuration.prm_config.num_obstacles].append(runtimes_scaffold)
        obstacle_count_to_runtimes_no_scaffold[configuration.prm_config.num_obstacles].append(runtimes_no_scaffold)

        connect_radius_to_runtimes_scaffold[configuration.prm_config.connect_radius].append(runtimes_scaffold)
        connect_radius_to_runtimes_no_scaffold[configuration.prm_config.connect_radius].append(runtimes_no_scaffold)

        total_verticies_to_quality_scaffold[configuration.prm_config.total_verticies].append(path_lengths_scaffold)
        total_verticies_to_quality_no_scaffold[configuration.prm_config.total_verticies].append(path_lengths_no_scaffold)

        total_verticies_to_percent_failure_scaffold[configuration.prm_config.total_verticies].append((scaffold_failures, len(configuration.trial_list)))
        total_verticies_to_percent_failure_no_scaffold[configuration.prm_config.total_verticies].append((no_scaffold_failures, len(configuration.trial_list)))

        connect_radius_to_quality_scaffold[configuration.prm_config.connect_radius].append(path_lengths_scaffold)
        connect_radius_to_quality_no_scaffold[configuration.prm_config.connect_radius].append(path_lengths_no_scaffold)

    print(type)
    if type == "runtimes":
        keys = sorted([k for k in total_verticies_to_runtimes_scaffold])
        scaffold_mean, scaffold_lower, scaffold_higher  = [], [], []
        no_scaffold_mean, no_scaffold_lower, no_scaffold_higher = [], [], []

        for k in keys:
            scaffold_runtimes = sorted(flatten(total_verticies_to_runtimes_scaffold[k]))
            no_scaffold_runtimes = sorted(flatten(total_verticies_to_runtimes_no_scaffold[k]))

            scaffold_mean.append(np.mean(scaffold_runtimes))
            scaffold_lower.append(scaffold_runtimes[int(len(scaffold_runtimes) * 0.05)])
            scaffold_higher.append(scaffold_runtimes[int(len(scaffold_runtimes) * 0.95)])

            no_scaffold_mean.append(np.mean(no_scaffold_runtimes))
            no_scaffold_lower.append(no_scaffold_runtimes[int(len(no_scaffold_runtimes) * 0.05)])
            no_scaffold_higher.append(no_scaffold_runtimes[int(len(no_scaffold_runtimes) * 0.95)])

        lower_bound = no_scaffold_lower
        upper_bound = no_scaffold_higher

        plt.plot(keys, lower_bound, color='black')
        plt.plot(keys, upper_bound, color='black')
        plt.fill_between(keys, lower_bound, upper_bound, facecolor=(0.8, 0.8, 0.8, 1))
        plt.plot(keys, no_scaffold_mean, color='black')

        lower_bound = scaffold_lower
        upper_bound = scaffold_higher

        plt.plot(keys, lower_bound, color='red')
        plt.plot(keys, upper_bound, color='red')
        plt.fill_between(keys, lower_bound, upper_bound, facecolor=(1, 0.8, 0.8, 1))
        plt.plot(keys, scaffold_mean, color='red')

        plt.ylabel('PRM Path Plan Runtime in Seconds (95% confidence)')
        plt.xlabel('Total verticies')

    elif type == "radiusruntimes":
        keys = sorted([k for k in connect_radius_to_runtimes_scaffold])
        scaffold_mean, scaffold_lower, scaffold_higher  = [], [], []
        no_scaffold_mean, no_scaffold_lower, no_scaffold_higher = [], [], []

        for k in keys:
            scaffold_runtimes = sorted(flatten(connect_radius_to_runtimes_scaffold[k]))
            no_scaffold_runtimes = sorted(flatten(connect_radius_to_runtimes_no_scaffold[k]))

            scaffold_mean.append(np.mean(scaffold_runtimes))
            scaffold_lower.append(scaffold_runtimes[int(len(scaffold_runtimes) * 0.05)])
            scaffold_higher.append(scaffold_runtimes[int(len(scaffold_runtimes) * 0.95)])

            no_scaffold_mean.append(np.mean(no_scaffold_runtimes))
            no_scaffold_lower.append(no_scaffold_runtimes[int(len(no_scaffold_runtimes) * 0.05)])
            no_scaffold_higher.append(no_scaffold_runtimes[int(len(no_scaffold_runtimes) * 0.95)])

        lower_bound = [np.mean(no_scaffold_lower)] * len(no_scaffold_lower)
        upper_bound = [np.mean(no_scaffold_higher)] * len(no_scaffold_higher)
        no_scaffold_mean = [np.mean(no_scaffold_mean)] * len(no_scaffold_mean)

        plt.plot(keys, lower_bound, color='black')
        plt.plot(keys, upper_bound, color='black')
        plt.fill_between(keys, lower_bound, upper_bound, facecolor=(0.8, 0.8, 0.8, 1))
        plt.plot(keys, no_scaffold_mean, color='black')

        lower_bound = scaffold_lower
        upper_bound = scaffold_higher

        plt.plot(keys, lower_bound, color='red')
        plt.plot(keys, upper_bound, color='red')
        plt.fill_between(keys, lower_bound, upper_bound, facecolor=(1, 0.8, 0.8, 1))
        plt.plot(keys, scaffold_mean, color='red')

        plt.ylabel('PRM Path Plan Runtime in Seconds (95% confidence)')
        plt.xlabel('Connect Radius (mm)')

    elif type == "obstaclecount":
        keys = sorted([k for k in obstacle_count_to_runtimes_scaffold])
        scaffold_mean, scaffold_lower, scaffold_higher  = [], [], []
        no_scaffold_mean, no_scaffold_lower, no_scaffold_higher = [], [], []

        for k in keys:
            scaffold_runtimes = sorted(flatten(obstacle_count_to_runtimes_scaffold[k]))
            no_scaffold_runtimes = sorted(flatten(obstacle_count_to_runtimes_no_scaffold[k]))

            scaffold_mean.append(np.mean(scaffold_runtimes))
            scaffold_lower.append(scaffold_runtimes[int(len(scaffold_runtimes) * 0.05)])
            scaffold_higher.append(scaffold_runtimes[int(len(scaffold_runtimes) * 0.95)])

            no_scaffold_mean.append(np.mean(no_scaffold_runtimes))
            no_scaffold_lower.append(no_scaffold_runtimes[int(len(no_scaffold_runtimes) * 0.05)])
            no_scaffold_higher.append(no_scaffold_runtimes[int(len(no_scaffold_runtimes) * 0.95)])

        lower_bound = no_scaffold_lower
        upper_bound = no_scaffold_higher

        plt.plot(keys, lower_bound, color='black')
        plt.plot(keys, upper_bound, color='black')
        plt.fill_between(keys, lower_bound, upper_bound, facecolor=(0.8, 0.8, 0.8, 1))
        plt.plot(keys, no_scaffold_mean, color='black')

        lower_bound = scaffold_lower
        upper_bound = scaffold_higher

        plt.plot(keys, lower_bound, color='red')
        plt.plot(keys, upper_bound, color='red')
        plt.fill_between(keys, lower_bound, upper_bound, facecolor=(1, 0.8, 0.8, 1))
        plt.plot(keys, scaffold_mean, color='red')

        plt.ylabel('PRM Path Plan Runtime in Seconds (95% confidence)')
        plt.xlabel('Number of Obstacles')

    elif type == "quality":
        keys = sorted([k for k in total_verticies_to_quality_no_scaffold])
        scaffold_mean, scaffold_lower, scaffold_higher  = [], [], []
        no_scaffold_mean, no_scaffold_lower, no_scaffold_higher = [], [], []

        to_remove = []

        for idx, k in enumerate(keys):
            scaffold_quality = sorted(flatten(total_verticies_to_quality_scaffold[k]))
            no_scaffold_quality = sorted(flatten(total_verticies_to_quality_no_scaffold[k]))

            if len(scaffold_quality) is 0 or len(no_scaffold_quality) is 0:
                print("no data")
                to_remove.append(k)
            else:
                scaffold_mean.append(np.mean(scaffold_quality))
                scaffold_lower.append(scaffold_quality[int(len(scaffold_quality) * 0.05)])
                scaffold_higher.append(scaffold_quality[int(len(scaffold_quality) * 0.95)])
                no_scaffold_mean.append(np.mean(no_scaffold_quality))
                no_scaffold_lower.append(no_scaffold_quality[int(len(no_scaffold_quality) * 0.05)])
                no_scaffold_higher.append(no_scaffold_quality[int(len(no_scaffold_quality) * 0.95)])

        for k in to_remove:
            keys.remove(k)

        lower_bound = scaffold_lower
        upper_bound = scaffold_higher
        
        plt.plot(keys, lower_bound, color='red')
        plt.plot(keys, upper_bound, color='red')
        plt.fill_between(keys, lower_bound, upper_bound, facecolor=(1, 0.8, 0.8, 1))
        plt.plot(keys, scaffold_mean, color='red')

        lower_bound = no_scaffold_lower
        upper_bound = no_scaffold_higher

        plt.plot(keys, lower_bound, color='black')
        plt.plot(keys, upper_bound, color='black')
        plt.fill_between(keys, lower_bound, upper_bound, facecolor=(0.8, 0.8, 0.8, 1))
        plt.plot(keys, no_scaffold_mean, color='black')

        plt.ylabel('PRM Path Plan Length (95% confidence)')
        plt.xlabel('Total verticies')


    elif type == "radius":
        keys = sorted([k for k in connect_radius_to_quality_scaffold])
        print(keys)
        scaffold_mean, scaffold_lower, scaffold_higher  = [], [], []
        no_scaffold_mean, no_scaffold_lower, no_scaffold_higher = [], [], []

        to_remove = []

        for idx, k in enumerate(keys):
            scaffold_quality = sorted(flatten(connect_radius_to_quality_scaffold[k]))
            no_scaffold_quality = sorted(flatten(connect_radius_to_quality_no_scaffold[k]))

            if len(scaffold_quality) is 0 or len(no_scaffold_quality) is 0:
                print("no data")
                to_remove.append(k)
            else:
                scaffold_mean.append(np.mean(scaffold_quality))
                scaffold_lower.append(scaffold_quality[int(len(scaffold_quality) * 0.05)])
                scaffold_higher.append(scaffold_quality[int(len(scaffold_quality) * 0.95)])
                no_scaffold_mean.append(np.mean(no_scaffold_quality))
                no_scaffold_lower.append(no_scaffold_quality[int(len(no_scaffold_quality) * 0.05)])
                no_scaffold_higher.append(no_scaffold_quality[int(len(no_scaffold_quality) * 0.95)])

        lower_bound = no_scaffold_lower
        upper_bound = no_scaffold_higher

        plt.plot(keys, lower_bound, color='black')
        plt.plot(keys, upper_bound, color='black')
        plt.fill_between(keys, lower_bound, upper_bound, facecolor=(0.8, 0.8, 0.8, 1))
        plt.plot(keys, no_scaffold_mean, color='black')

        lower_bound = scaffold_lower
        upper_bound = scaffold_higher
        
        plt.plot(keys, lower_bound, color='red')
        plt.plot(keys, upper_bound, color='red')
        plt.fill_between(keys, lower_bound, upper_bound, facecolor=(1, 0.8, 0.8, 1))
        plt.plot(keys, scaffold_mean, color='red')

        plt.ylabel('PRM Path Plan Length (95% confidence)')
        plt.xlabel('Connect Radius (mm)')

    elif type == "failure":
        keys = sorted([k for k in total_verticies_to_percent_failure_no_scaffold])
        percent_failure_no_scaffold = []
        percent_failure_scaffold = []
        for k in keys:
            numer = 0
            denom = 0
            for kv in total_verticies_to_percent_failure_no_scaffold[k]:
                numer += kv[0]
                denom += kv[1]
            percent_failure_no_scaffold.append(numer / denom)
            numer = 0
            denom = 0
            for kv in total_verticies_to_percent_failure_scaffold[k]:
                numer += kv[0]
                denom += kv[1]

            percent_failure_scaffold.append(numer / denom)
            

        print(percent_failure_scaffold)
        print(percent_failure_no_scaffold)
        plt.plot(keys, percent_failure_no_scaffold, color='black')
        plt.plot(keys, percent_failure_scaffold, color='red')

        plt.ylabel('PRM failures ([0, 1] scale)')
        plt.xlabel('Total verticies')

    else:
        print("{} is not valid...".format(type))
        exit(0)


    # for k in sorted([k for k in total_verticies_to_runtimes_no_scaffold]):
    #     print("{} {}".format(k, total_verticies_to_runtimes_no_scaffold[k]))
    #     print()
        
    # if type is "number samples":
    #     lower_bound = [-1.96 * s[0] + s[1] for s in zip(no_scaffold_samples_distance_std_dev, no_scaffold_samples_distance)]
    #     upper_bound = [1.96 * s[0] + s[1] for s in zip(no_scaffold_samples_distance_std_dev, no_scaffold_samples_distance)]
    #     plt.plot(scaffold_bias, lower_bound, color='black')
    #     plt.plot(scaffold_bias, upper_bound, color='black')
    #     plt.fill_between(scaffold_bias, lower_bound, upper_bound, facecolor=(0.8, 0.8, 0.8, 1))
    #     plt.plot(scaffold_bias, no_scaffold_samples_distance, color='black')
    
    #     lower_bound = [-1.96 * s[0] + s[1] for s in zip(scaffold_samples_distance_std_dev, scaffold_samples_distance)]
    #     upper_bound = [1.96 * s[0] + s[1] for s in zip(scaffold_samples_distance_std_dev, scaffold_samples_distance)]
    
    #     plt.plot(scaffold_bias, lower_bound, color='red')
    #     plt.plot(scaffold_bias, upper_bound, color='red')
    #     plt.fill_between(scaffold_bias, lower_bound, upper_bound, facecolor=(1, 0.8, 0.8, 1))
    #                      # where=lower_bound <= upper_bound,
    #                      # facecolor=(1, 0.8, 0.8, 1), interpolate=True, linewidth=0.0, hatch="X",
    #                      # edgecolor=(1, 0.6, 0.6, 1))
    #     plt.plot(scaffold_bias, scaffold_samples_distance, color='red')
    #     plt.ylabel('Number of samples (95% confidence)')

    # elif type is "path length":
    #     lower_bound = no_scaffold_path_length_min
    #     upper_bound = no_scaffold_path_length_max
    #     plt.plot(scaffold_bias, lower_bound, color='black')
    #     plt.plot(scaffold_bias, upper_bound, color='black')
    #     plt.fill_between(scaffold_bias, lower_bound, upper_bound, facecolor=(0.8, 0.8, 0.8, 1))
    #     plt.plot(scaffold_bias, no_scaffold_path_length, color='black')

    #     lower_bound = scaffold_path_length_min
    #     upper_bound = scaffold_path_length_max
    
    #     plt.plot(scaffold_bias, lower_bound, color='red')
    #     plt.plot(scaffold_bias, upper_bound, color='red')
    #     plt.fill_between(scaffold_bias, lower_bound, upper_bound, facecolor=(1, 0.8, 0.8, 1))
    #                      # where=lower_bound <= upper_bound,
    #                      # facecolor=(1, 0.8, 0.8, 1), interpolate=True, linewidth=0.0, hatch="X",
    #                      # edgecolor=(1, 0.6, 0.6, 1))
    #     plt.plot(scaffold_bias, scaffold_path_length, color='red')

    #     plt.ylabel('Path length (95% confidence)')

    #plt.title("Scenario: {}".format(scenario.scenerio_name))

    # plt.xlabel('Scaffold bias')
    red_patch = patches.Patch(color='red', label='Scaffolded')
    black_patch = patches.Patch(color='black', label='Unscaffolded')
    #plt.legend(handles=[red_patch, black_patch])
    fig = plt.gcf()
    ax = fig.gca()
    fig.set_size_inches(15, 15)
    fig.savefig('graphprm{}{}.png'.format(type, scenario.scenerio_name))
    #plt.ylim([0.8,1.2])
    plt.show()

