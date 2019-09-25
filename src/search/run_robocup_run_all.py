#!/usr/bin/env python3
import sys
import subprocess
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import plot_helper
from matplotlib.ticker import FuncFormatter

scripts = """src/search/run_robocup_lazy_vs_eager_individual_lines.py
src/search/run_robocup_xstar_epsilon_sweep.py
src/search/run_robocup_xstar_radius_sweep.py
src/search/run_robocup_xstar_vs_mstar_runtime.py
src/search/run_robocup_xstar_vs_mstar_runtime_random.py
src/search/run_robocup_xstar_vs_nwastar_vs_astar.py""".split('\n')

def run_script(script):
    result = subprocess.run([str(e) for e in [script, 'false' ]], stdout=subprocess.PIPE)
    print(result.stdout.decode('UTF-8').split("\n"))
    
for script in scripts:
    print("Script:", script)
    run_script(script)
