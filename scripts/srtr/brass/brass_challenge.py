#!/usr/bin/env python3

import subprocess
import sys
import os

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

os.mkdir("scripts/srtr/brass/results/adapted_traces")
os.mkdir("scripts/srtr/brass/results/adapted_traces_starved")
os.mkdir("scripts/srtr/brass/results/nominal_traces")
os.mkdir("scripts/srtr/brass/results/degraded_traces")
os.mkdir("scripts/srtr/brass/results/adaptations")

command = "scripts/srtr/brass/setup_brass_tests.py {}".format(sys.argv[1])
result = subprocess.call(command, shell=True)
command = "scripts/srtr/brass/run_brass_tests.py setup"
result = subprocess.call(command, shell=True)
command = "scripts/srtr/brass/get_all_corrections.py"
result = subprocess.call(command, shell=True)
command = "scripts/srtr/brass/run_brass_tests.py adapted"
result = subprocess.call(command, shell=True)
command = "scripts/srtr/brass/run_brass_tests.py starved"
result = subprocess.call(command, shell=True)