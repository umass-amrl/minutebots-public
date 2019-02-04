#!/usr/bin/env python3
import numpy as np
from scipy.optimize import leastsq
from numpy import genfromtxt
import matplotlib.pyplot as plt
import pdb

data = genfromtxt('sine_controller.csv', delimiter=',')


N = data.shape[0] # number of data points
t_obs = data[:, 0]
obs = data[:, 1]
t_exp = data[:, 2]
exp = data[:, 3]

# Prune everything from before we have commands
prev_index = 0
for i in range(0, N):
  if t_obs[i] >= t_exp[0]:
    prev_index = i
    break

t_obs = np.delete(t_obs, range(0, prev_index))
obs = np.delete(obs, range(0, prev_index))

# Prune everything from after we don't have observations
N = t_obs.shape[0]
next_index = 0
for i in range(1, N-1) :
  if (t_exp[-i] <= t_obs[-1]):
    next_index = i-1
    break

M = t_exp.shape[0]
t_exp = np.delete(t_exp, range(M-next_index, M))
exp = np.delete(exp, range(M-next_index, M)) 

#pdb.set_trace()

t_obs -= t_exp[0]
t_exp -= t_exp[0]


obs_guess_mean = np.mean(obs)
obs_guess_std = 3*np.std(obs)/(2**0.5)
obs_guess_phase = 0

exp_guess_mean = np.mean(exp)
exp_guess_std = 3*np.std(exp)/(2**0.5)
exp_guess_phase = 0

obs_optimize_func = lambda x: x[0]*np.sin(t_obs+x[1]) + x[2] - obs
obs_std, obs_phase, obs_mean = leastsq(obs_optimize_func, [obs_guess_std, obs_guess_phase, obs_guess_mean])[0]

obs_fit = obs_std*np.sin(t_obs+obs_phase) + obs_mean

exp_optimize_func = lambda x: x[0]*np.sin(t_exp+x[1]) + x[2] - exp
exp_std, exp_phase, exp_mean = leastsq(exp_optimize_func, [exp_guess_std, exp_guess_phase, exp_guess_mean])[0]

exp_fit = exp_std*np.sin(t_exp+exp_phase) + exp_mean

#pdb.set_trace()

print(obs_phase)

plt.plot(t_obs, obs, 'ro')
plt.plot(t_exp, exp, 'bo')
plt.plot(t_obs, obs_fit, 'r')
plt.plot(t_exp, exp_fit, 'b')
plt.grid()
plt.show()
