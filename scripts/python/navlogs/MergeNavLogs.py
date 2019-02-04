#!/usr/bin/env python3
import os
import sys
from struct import *

def usage():
  if len(sys.argv) != 2:
    print("Usage:", sys.argv[0], "[Nav log file]")
    exit(1)

usage()

# Adds build directory to import search path.
sys.path.insert(0, str(os.path.join(os.getcwd(), "build/")))

from google.protobuf import text_format
import navigation_logging_pb2

f = open(sys.argv[1], 'rb')
data_amount = unpack('i', f.read(4))[0]
print(data_amount)
f.seek(4)
data = f.read(data_amount)
print(data)
f.seek(31)
print(f.read(3))

message = text_format.Parse(data, 
                            navigation_logging_pb2.NavigationLogEntry())
print(message)
