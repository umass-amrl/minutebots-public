#/bin/bash
# Runs the soccer stack repeatedly to auto-restart in the case of seg-faults.
# No, I'm not sorry.
while [ true ]; do
  ./bin/viewer -r10033
done
