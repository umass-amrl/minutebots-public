#!/usr/bin/env python3

import numpy as np
import subprocess
import multiprocessing as mp
import time
import itertools
from functools import reduce
import random
import json
import sys
import os
from collections import OrderedDict

kKickSpeed = 1.5

def exec_soccer(args):
    x, y, angle, file = args
    vx, vy = speed_to_velocity(angle)
    file = file + "_" + str(x) + "_" + str(y) + "_" + str(angle) + ".txt"
    command = "./bin/soccer -S -b {},{},{},{} -dp -tb -py -T {}".format(x, y, vx, vy, file)
    result = subprocess.call(command, shell=True)
    return (x, y, vx, vy, angle, (result == 0))

def speed_to_velocity(angle_deg):
    assert(angle_deg >= 0)
    assert(angle_deg <= 360)
    return (np.cos(np.deg2rad(angle_deg)) * kKickSpeed, 
            np.sin(np.deg2rad(angle_deg)) * kKickSpeed)

def merge_result(result_dict, result):
    x, y, vx, vy, angle, success = result
    existing = result_dict.get((x, y, angle), 0)
    if success:
        existing += 1
    result_dict[str((x, y, angle))] = existing
    return result_dict

def save_to_file(result_dict, state_name, name):
    filename = 'scripts/srtr/brass/results/' + name + '_' + state_name
    text_file = open(filename + '_results.json', "w")
    json.dump(result_dict, text_file, indent=2)
    text_file.close()

def ReadTestsFromFile(filename):
    with open(filename) as input_file:
        file_json = json.load(input_file)
        return file_json

def RunTestScenario(state_name, scenario):
  # Retrieve the scenario setup
  name = scenario['name']
  parameters = scenario['parameters']
  tasks = []
  filename = 'scripts/srtr/brass/results/' + state_name + '/' + name
  for test in scenario['tests']:
    tasks.append([test['ball_x'], 
                  test['ball_y'], 
                  test['ball_velocity_angle'], 
                  filename])
  # Spawns two threads, so divide CPUs by two.
  count = mp.cpu_count() // 2
  pool = mp.Pool(processes=count)
  # Run the jobs in parallel!!!!
  exec_results = pool.map(exec_soccer, tasks)
  return reduce(merge_result, exec_results, {})

def LoadResults(state_name, scenario):
  filename = "scripts/srtr/brass/results/{}_{}_results.json".format(scenario, state_name)
  return ReadTestsFromFile(filename)

def AnalyzeResults(name, result, nominal_result, degraded_result):
  penalty = 0
  reward = 0
  possible_fixes = 0
  total = 0
  filtered_result = {}
  for key in nominal_result:
    total += 1
    filtered_result[key] = result[key]
    adapted_score = result[key]
    degraded_score = degraded_result[key]
    if (degraded_score == 0): # Degraded was broken
      possible_fixes += 1
      if (adapted_score == 1): # Either we fix it or not
        reward += 1
    elif (adapted_score == 0): # otherwise we could have broken it
      penalty += 1
  # Generate a short summary
  summary = OrderedDict()
  summary["Nominal Successes"] = total
  summary["Degraded Failures"] = possible_fixes
  summary["Successfully Repaired"] = reward
  summary["Added Failures"] = penalty
  summary["Fix Percentage"] = reward / possible_fixes
  summary["Failure Percentage"] = penalty / total
  # Output Summary json to file
  filename = 'scripts/srtr/brass/results/' + name + '_summary.json'
  text_file = open(filename, "w")
  json.dump(summary, text_file, indent=2)
  text_file.close()
  return reward / possible_fixes

def RunSetupTests():
  # Grab nominal and degraded test lists
  nominal_file = 'scripts/srtr/brass/results/nominal_tests.json'
  degraded_file = 'scripts/srtr/brass/results/degraded_tests.json'
  nominal_json = ReadTestsFromFile(nominal_file)
  degraded_json = ReadTestsFromFile(degraded_file)
  adapted = False
  # For each test scenario
  for i in range(len(nominal_json)):
    # Run the tests
    name = nominal_json[i]['name']

    # Replace the attacker config with the nominal parameters.
    parameters = nominal_json[i]['parameters']

    with open('src/configs/attacker_config.json', 'w') as attacker_config:
      json.dump(parameters, attacker_config, indent=2)
    nominal_results = RunTestScenario('nominal_traces', nominal_json[i])

    # Replace the attacker config with the degraded parameters.
    parameters = degraded_json[i]['parameters']
    adaptation = "scripts/srtr/brass/results/adaptations/{}_adaptation.json".format(name)
    if (os.path.isfile(adaptation)):
      adapted = True
      command = "cp {} src/configs/attacker_config.json".format(adaptation)
      result = subprocess.call(command, shell=True)
    else:
      with open('src/configs/attacker_config.json', 'w') as attacker_config:
        json.dump(parameters, attacker_config, indent=2)
    degraded_results = RunTestScenario('degraded_traces', degraded_json[i])
    # Save both end results to file
    save_to_file(nominal_results, 'nominal', name)
    save_to_file(degraded_results, 'degraded', name)
    if (not adapted):
      save_to_file(degraded_results, 'original_degraded', name)    

def RunAdaptedTests(starved):
  # Grab the nominal json for the test list
  nominal_file = 'scripts/srtr/brass/results/nominal_tests.json'
  nominal_json = ReadTestsFromFile(nominal_file)
  results = 0
  # For each test scenario
  for i in range(len(nominal_json)):
    # Run the tests
    name = nominal_json[i]['name']
    # Replace the attacker config with the adapted parameters.
    adaptation = "scripts/srtr/brass/results/adaptations/{}_adaptation.json".format(name)
    directory_name = "adapted_traces"
    analysis_name = name
    if (starved): 
      adaptation = "scripts/srtr/brass/results/adaptations/{}_starved_adaptation.json".format(name)
      directory_name += "_starved"
      analysis_name += "_starved"
    command = "cp {} src/configs/attacker_config.json".format(adaptation)
    result = subprocess.call(command, shell=True)

    # Run the adapted attacker
    adapted_result = RunTestScenario(directory_name , nominal_json[i])

    # Load the other results, trim nominal failures, and compare
    nominal_result = LoadResults('nominal', name)
    degraded_result = LoadResults('original_degraded', name)
    results += AnalyzeResults(analysis_name, adapted_result, nominal_result, degraded_result)
    # Save the adapted results to file
    output_name = 'adapted'
    if (starved):
      output_name += '_starved'
    save_to_file(adapted_result, output_name, name)
  return results

def RunBrassTests(arg):
  value = 0
  if (arg == "setup"):
   value = RunSetupTests()
  elif(arg == "starved"):
    value = RunAdaptedTests(True)
  else: 
    value  = RunAdaptedTests(False)
  return value