#/bin/bash
# Runs the unit tests until they fail. Useful for tests that involve randomness.
./bin/run_unit_tests
while [ -n "$?" ]; do
  ./bin/run_unit_tests
  sleep 0.01
done
