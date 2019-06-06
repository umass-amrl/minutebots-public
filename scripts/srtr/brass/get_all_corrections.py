#!/usr/bin/env python3

import subprocess
import json
import sys
import os

def CopyAndAdapt(test_scenario):
  # Move the old corrections to a stored file
  correction_file =  "scripts/srtr/brass/results/{}_corrections.txt".format(test_scenario)
  command = "mv scripts/srtr/brass/results/correction_trace.txt {}".format(correction_file)
  result = subprocess.call(command, shell=True)
  # Get the adaptation from SRTR
  command = "./bin/srtr_brass {}".format(correction_file)
  result = subprocess.call(command, shell=True)
  command = "mv brass_srtr.json scripts/srtr/brass/results/adaptations/{}_adaptation.json".format(test_scenario)
  result = subprocess.call(command, shell=True)
  command = "mv brass_srtr_starved.json scripts/srtr/brass/results/adaptations/{}_starved_adaptation.json".format(test_scenario)
  result = subprocess.call(command, shell=True)

directory_path = "scripts/srtr/brass/results/"
nominal_path = directory_path + "nominal_traces/"
degraded_path = directory_path + "degraded_traces/"
test_scenario = ""
for file in sorted(os.listdir(nominal_path)):
  filename = os.fsdecode(file)
  nominal_file = nominal_path + filename
  degraded_file = degraded_path + filename
  first_underscore = filename.find("_")
  current_scenario = filename[:first_underscore]
  # Identify if we've changed test scenarios or not
  print(test_scenario)
  print(current_scenario)
  if (current_scenario != test_scenario and test_scenario != ""):
    print("Adapting")
    CopyAndAdapt(test_scenario)
  test_scenario = current_scenario
  # Run the correction generation
  if (os.path.isfile(nominal_file) and os.path.isfile(degraded_file)):
    command = "./bin/gen_corrections {} {}".format(nominal_file,
                                                   degraded_file)
    result = subprocess.call(command, shell=True)
# Finish the last test scenario
CopyAndAdapt(test_scenario)
