#!/usr/bin/env python3

import subprocess
import sys
import os
import json
from run_brass_tests import RunBrassTests

directory_path = "scripts/srtr/brass/archive/"
if (os.path.exists(directory_path)):
  # Archive the last run
  num_files = len(os.listdir(directory_path))
  new_file = "scripts/srtr/brass/archive/results_{}/".format(num_files)
  os.mkdir(new_file)
  command = "mv scripts/srtr/brass/results/* {}".format(new_file)
  result = subprocess.call(command, shell=True)
else:
  # First run
  os.mkdir(directory_path)

# Check for input file
if (len(sys.argv) != 2):
  print(len(sys.argv))
  print("Expects 1 argument: path to input test generation json file.")
  raise SystemExit

if (not os.path.exists("scripts/srtr/brass/results/")):
  os.mkdir("scripts/srtr/brass/results/")

os.mkdir("scripts/srtr/brass/results/adaptations")
os.mkdir("scripts/srtr/brass/results/traces")

fix_percentage = 0;
iterations = 0;
original_failures = 0;
command = "scripts/srtr/brass/setup_brass_tests.py {}".format(sys.argv[1])
result = subprocess.call(command, shell=True)
num_test_scenarios = 1
with open(sys.argv[1]) as test_file:
  input_json = json.load(test_file)
  num_test_scenarios = len(input_json)
# Continue adapting until performance is sufficiently high or some cutoff
while (iterations < 1):
  os.mkdir("scripts/srtr/brass/results/adapted_traces")
  os.mkdir("scripts/srtr/brass/results/adapted_traces_starved")
  os.mkdir("scripts/srtr/brass/results/nominal_traces")
  os.mkdir("scripts/srtr/brass/results/degraded_traces")
  # First run for nominal and degraded results
  RunBrassTests("setup")
  # Generate the corrections
  command = "scripts/srtr/brass/get_all_corrections.py"
  result = subprocess.call(command, shell=True)
  # Get adapted results and starved adapted results
  fix_percentage = RunBrassTests("adapted")
  RunBrassTests("starved")
  # Save the traces from the current iteration
  folder = "scripts/srtr/brass/results/traces/trace_" + str(iterations) + "/"
  os.mkdir(folder)
  command = "mv scripts/srtr/brass/results/*_traces* " + folder
  result = subprocess.call(command, shell=True)
  iterations += 1
