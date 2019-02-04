#!/usr/bin/python

import numpy as np
from numpy.linalg import lstsq
from numpy import genfromtxt
import matplotlib.pyplot as plt
from scipy.stats import linregress

def norm(vec):
  return np.sqrt(vec[0] * vec[0] + vec[1] * vec[1])

def estimate(data):
  times = data[:,0]
  commands = data[:,1]
  # observed velocity is L2 norm of difference between two location observations
  # observed times is midpoints between timesteps, since the observed velocity
  # gives us a good estimate for velocity at that time
  observed_times = np.zeros((len(times) - 1))
  observed_vels = np.zeros((len(times) - 1))
  for i in range(len(observed_times)):
    observed_times[i] = (times[i + 1] + times[i]) / 2
    pos_diff = data[i + 1,2:] - data[i,2:]
    observed_vels[i] = norm(pos_diff) / (times[i + 1] - times[i])

  # do linear regression for commands (should be very close to commands,
  # since we command constant acceleration)
  command_slope, command_intercept, _1, _2, _3 = linregress(times, commands)
  print("\tcommand slope =", command_slope, "command intercept =", command_intercept)
  # do linear regression for observations
  obs_slope, obs_intercept, _1, _2, stderr = linregress(observed_times, observed_vels)
  print("\tobs slope =", obs_slope, "obs intercept =", obs_intercept)
  predicted_vels = [t * obs_slope + obs_intercept for t in observed_times]
  residuals = observed_vels - predicted_vels
  # remove outliers
  non_outliers_observed = [[observed_times[i], observed_vels[i]] for i, t in enumerate(observed_times) if np.abs(residuals[i]) < 2 * stderr]
  observed_times = [entry[0] for entry in non_outliers_observed]
  observed_vels = [entry[1] for entry in non_outliers_observed]
  # The latency how far the observed line is behind the commanded line
  # This is (time at which commanded vel is half of max commanded vel) - (time at which observed vel is half of max commanded vel)
  midpoint_vel = commands[len(commands) // 2]
  commanded_midpoint_time = (midpoint_vel - command_intercept) / command_slope
  observed_midpoint_time = (midpoint_vel - obs_intercept) / obs_slope
  latency = observed_midpoint_time - commanded_midpoint_time
  print("\tlatency =", latency)
  return latency

data = genfromtxt('latency.csv', delimiter=',')
i = 0
delta_t = 1 / 62.5
latency_estimates = []
while i < len(data) - 1:
  if data[i + 1,0] - data[i,0] > 3 * delta_t: # if there's a gap in time
    data_iter = data[:i + 1,:]
    print("on estimate", len(latency_estimates))
    latency_estimates += [estimate(data_iter)]
    data = data[i + 1:,:]
    i = 0
  else:
    i += 1
print("latency estimate is", np.mean(latency_estimates))
