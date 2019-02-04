#!/usr/bin/env python3

import numpy as np
import csv
import sys
import matplotlib.pyplot as plt


class Plotter(object):
  def __init__(self, ax, paths, obstacles, radii):
    self.axes = ax
    self.canvas = ax.figure.canvas
    self.paths = paths
    self.obstacles = obstacles
    self.radii = radii

    self.index = len(paths) - 1
    ax.set_aspect('equal', 'datalim')
    self.PlotPath(self.paths[self.index])

    self.cid = self.canvas.mpl_connect('key_press_event', self.Press)
  
  def Press(self, event):
    if event.key == 'n' or event.key == 'right':
      self.index = self.index + 1
    elif event.key == 'p' or event.key == 'left':
      self.index = self.index - 1
    elif event.key == ' ':
      self.index = 0
    elif event.key == 'e':
      self.index = len(paths) - 1
    if (self.index < 0):
      self.index = 0
    elif (self.index >= len(paths)):
      self.index = len(paths) - 1    

    self.axes.clear()
    self.PlotPath(self.paths[self.index])
    self.canvas.draw()
  
  def PlotPath(self, path):
    path_matrix = np.array(path)
    ax.plot(path_matrix[:, 0], path_matrix[:, 1], color='black')

    for i in range(0, len(obstacle_positions)):
      ax.add_artist(plt.Circle((self.obstacles[i][0], self.obstacles[i][1]), self.radii[i], color='r')) 

# Load obstacle positions and radii from a file
# Obstacles are given as comma seperaated values each on their own line
# x, y, radius
def LoadObstacles(filename):
  obstacle_positions = []
  radii = []
  with open(filename + '.obs', 'r') as csvfile:
    obstaclereader = csv.reader(csvfile)
    for row in obstaclereader:
      obstacle_positions.append(np.array([float(row[0]), float(row[1])]))
      radii.append(float(row[2]))
  return (obstacle_positions, radii)

def LoadPaths(filename):
  paths = []

  with open(filename, 'r') as csvfile:
    pathreader = csv.reader(csvfile)
    for row in pathreader:
      path = []
      x_values = []
      y_values = []
      for i in range(0, len(row)):
        if i%2 == 0:
          x_values.append(float(row[i]))
        else:
          y_values.append(float(row[i]))
 
      for i in range(0, len(x_values)):
        path.append(np.array([x_values[i], y_values[i]]))
      paths.append(path)
  return paths

def PlotAllPaths(ax, paths, obstacles, radii):
  for path in paths:
    path_matrix = np.array(path)
    ax.plot(path_matrix[:, 0], path_matrix[:, 1], color='grey')

  path_matrix = np.array(paths[-1])
  ax.plot(path_matrix[:, 0], path_matrix[:, 1], color='black', linewidth = 4.0)

  for i in range(0, len(obstacle_positions)):
    ax.add_artist(plt.Circle((obstacles[i][0], obstacles[i][1]), radii[i], color='r')) 
  
  ax.set_aspect('equal', 'datalim')
 
  canvas = ax.figure.canvas
  canvas.draw()

if __name__ == "__main__":
  (obstacle_positions, radii) = LoadObstacles(sys.argv[1])
  paths = LoadPaths(sys.argv[1])

  ax = plt.axes()  

  # pltr = Plotter(ax, paths, obstacle_positions, radii)
  PlotAllPaths(ax, paths, obstacle_positions, radii)
  plt.show()
