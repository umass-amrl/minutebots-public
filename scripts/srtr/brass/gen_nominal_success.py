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
    x, y, angle= args
    vx, vy = speed_to_velocity(angle)
    command = "./bin/soccer -S -b {},{},{},{} -dp -tb -py".format(x, y, vx, vy)
    result = subprocess.call(command, shell=True)
    return (x, y, vx, vy, angle, (result == 0))

def speed_to_velocity(angle_deg):
    assert(angle_deg >= 0)
    assert(angle_deg <= 360)
    return (np.cos(np.deg2rad(angle_deg)) * kKickSpeed, 
            np.sin(np.deg2rad(angle_deg)) * kKickSpeed)

def merge_result(result_list, result):
    x, y, vx, vy, angle, success = result
    #existing = result_dict.get((x, y, angle), 0)
    if success:
        temp_dict = {}
        temp_dict['ball_x'] = x
        temp_dict['ball_y'] = y 
        temp_dict['ball_velocity_angle'] = angle
        result_list.append(temp_dict)
    return result_list

def save_to_file(result_list):
    filename = 'scripts/srtr/brass/test_library.json'
    text_file = open(filename, "w")
    json.dump(result_list, text_file, indent=2)
    text_file.close()

xs = [(e * 4000/9) for e in list(range(10))]
ys = [e * (3000/10) for e in list(range(-10, 10))]
angles = [e * 36 for e in list(range(10))]
tasks = list(itertools.product(xs, ys, angles))

#Spawns two threads, so divide CPUs by two.
count = mp.cpu_count()
pool = mp.Pool(processes=count)

#Run the jobs in parallel!!!!
exec_results = pool.map(exec_soccer, tasks)
result_list = reduce(merge_result, exec_results, [])
save_to_file(result_list)