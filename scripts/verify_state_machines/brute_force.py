#!/usr/bin/env python3

import numpy as np
import subprocess
import multiprocessing as mp
import time
import itertools
from functools import reduce

kKickSpeed = 1.5

def exec_soccer(args):
    x, y, angle = args
    vx, vy = speed_to_velocity(angle)
    command = "./bin/soccer -S -b {},{},{},{} -dp -tb -py".format(x, y, vx, vy)
    result = subprocess.call(command, shell=True)
    return (x, y, vx, vy, (result == 0))

def speed_to_velocity(angle_deg):
    assert(angle_deg >= 0)
    assert(angle_deg <= 360)
    return (np.cos(np.deg2rad(angle_deg)) * kKickSpeed, np.sin(np.deg2rad(angle_deg)) * kKickSpeed)

def merge_result(result_dict, result):
    x, y, vx, vy, success = result
    existing = result_dict.get((x,y), 0)
    if success:
        existing += 1
    result_dict[(x, y)] = existing
    return result_dict

def save_to_file(result_dict):
    text_file = open("main_attacker_brute_force.txt", "w")
    text_file.write(str(result_dict))
    text_file.close()
    
    text_file = open("main_attacker_brute_force.csv", "w")
    text_file.write("X,Y,Score\n")
    for x, y in result_dict:
        text_file.write("{},{},{}\n".format(x, y, result_dict[(x, y)]))
    text_file.close()

xs = [(e * 4000/64) for e in list(range(65))]
ys = [e * (3000/40) for e in list(range(-40, 41))]
angles = [e * 36 for e in list(range(10))]

tasks = list(itertools.product(*[xs, ys, angles]))

#Spawns two threads, so divide CPUs by two.
count = mp.cpu_count()
pool = mp.Pool(processes=count)

#Run the jobs in parallel!!!!
exec_results = pool.map(exec_soccer, tasks)
result_dict = reduce(merge_result, exec_results, {})
save_to_file(result_dict)
