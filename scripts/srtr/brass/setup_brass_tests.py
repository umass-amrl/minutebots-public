#!/usr/bin/env python3

import itertools
import random
import json
from collections import OrderedDict
import sys
import copy

def load_nominal_values(nominal_file):
  nominal_values = OrderedDict()
  with open(nominal_file) as nominal:
    nominal_json = json.load(nominal)
    for x in nominal_json:
      nominal_values[x['name']] = x['nominalValue']
  return nominal_values

def LoadTests():
  with open("scripts/srtr/brass/test_library.json") as input_file:
    file_json = json.load(input_file)
    return file_json

def convert_to_json(poses, test_name, scenario, nominal):
  degraded_tests = []
  nominal_tests = []
  test_scenario = OrderedDict();
  test_scenario['name'] = test_name
  test_scenario['parameters'] = copy.deepcopy(nominal)
  test_scenario['tests'] = poses
  nominal_tests.append(copy.deepcopy(test_scenario))
  for perturb in scenario['perturbations']:
    name = perturb['name']
    test_scenario['parameters'][name] = perturb['perturbedValue']
  degraded_tests.append(test_scenario)
  return nominal_tests,degraded_tests

if (len(sys.argv) != 2):
  print(len(sys.argv))
  print("Expects 1 argument: path to input test generation json file.")

# Load the file with the nominally successful tests
test_list = LoadTests()
nominal = load_nominal_values('scripts/srtr/brass/parameter_enumeration.json')

with open(sys.argv[1]) as test_file:
  input_json = json.load(test_file)
  nominal_json = []
  degraded_json = []
  for x in input_json:
    # Randomly sample N poses
    n_poses = random.sample(test_list, x['numScenarios'])
    nominal_part,degraded_part = convert_to_json(n_poses, x['name'], x, nominal)
    nominal_json = nominal_json + nominal_part
    degraded_json = degraded_json + degraded_part
  with open('scripts/srtr/brass/results/degraded_tests.json', 'w') as output:
    json.dump(degraded_json, output, indent=2)
  with open('scripts/srtr/brass/results/nominal_tests.json', 'w') as output:
    json.dump(nominal_json, output, indent=2)
