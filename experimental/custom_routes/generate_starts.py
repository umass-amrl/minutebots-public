#!/usr/bin/env python3

import glob

def get_key(s):
    return int(s.split("route")[1].split(".")[0])

routes = sorted(glob.glob("route*.txt"), key=get_key)

def read_first_line(file_name):
    return open(file_name).readline()

starts = [read_first_line(r) for r in routes ][1:]
starts = ["1000 1000\n"] + starts
output_file = 'starts.txt'
output = open(output_file, 'w')
for s in starts:
    output.write(s)
print("Written to", output_file)
