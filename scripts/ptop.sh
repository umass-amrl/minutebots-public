#!/bin/bash
if [ -z $1 ]; then
  echo "ERROR: must specify process name!"
  exit
fi

# PID=$(ps aux | grep -v grep | grep $1 | cut -f 2 -d " ")
PID=$(ps aux | grep -v grep | grep $1 | head -n1 | cut -f 2 -d " ")
top -p $PID $2
