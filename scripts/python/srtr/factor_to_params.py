#!/usr/bin/env python3
import os
import sys
import glob
import shutil
from sets import Set
import networkx as nx
from networkx.algorithms import bipartite
import matplotlib.pyplot as plt

dest = os.path.join(os.getcwd(), 'scripts/python/srtr/')
assert(os.path.isdir(dest))
for filename in glob.glob(os.path.join("build/", '*.py')):
  shutil.copy2(filename, dest)

import tuning_data_pb2
from google.protobuf import text_format

# Large matrix to represent the actual graph to generate.
relationships = [[0,0,0,0,1,
                  0,1,0,0,0,
                  0,0,0,0,0,
                  0,0,0,0,0,
                  0,0,0,0,1],
                 [0,0,0,1,0,
                  0,1,0,0,0,
                  0,0,0,0,0,
                  0,0,0,0,0,
                  0,0,0,0,1],
                 [0,0,1,0,0,
                  0,1,0,0,0,
                  0,0,0,0,0,
                  0,0,0,0,0,
                  0,0,0,0,1],
                 [0,1,0,0,0,
                  0,1,0,0,0,
                  0,0,0,0,0,
                  0,0,0,0,0,
                  0,0,0,0,1],
                 [1,0,0,0,0,
                  0,1,0,0,0,
                  0,0,0,0,0,
                  0,0,0,0,0,
                  0,0,0,0,1]]

def generate_fake_factor_data():
  # Create a protobuf
  data = tuning_data_pb2.FactorSet();
  # Fill the protobuf
  # Let's do five factors and five params.
  for i in range(0,5):
    data.factor_names.append('factor_' + str(i));
    data.param_names.append('param_' + str(i));
  # Generates the file, still need to be filled in with relationship
  count = -1
  for k in range(1,6):
    for i in range(0,5):
      for offset in range(i, 6 - k):
        if (not (k == 1 and offset > i)):
          data_point = tuning_data_pb2.FactorTuningData()
          count += 1
          for x in range(0,5):
            data_point.adjusted_factors.append(False)
            data_point.adjusted_params.append(False)
            data_point.factor_adjustments.append(0)
            data_point.param_adjustments.append(0)
          data_point.adjusted_factors[i] = True
          data_point.factor_adjustments[i] = 5.0
          for j in range(1,k):
            entry = (offset + j)
            if entry < 5:
              data_point.adjusted_factors[entry] = True
              data_point.factor_adjustments[entry] = 5.0
          for param in range(0,5):
            if (relationships[param][count]):
              data_point.adjusted_params[param] = True
              data_point.param_adjustments[param] = 1.0
          data.factors_tuned.extend([data_point])

  # Write to file

  return data


def build_graph(labels=None, graph_layout='shell',
               node_size=1600, node_color='blue', node_alpha=0.3,
               node_text_size=12,
               edge_color='blue', edge_alpha=0.3, edge_tickness=1,
               edge_text_pos=0.3,
               text_font='sans-serif'):
  # Read in or generate the dataset
  data = generate_fake_factor_data()
  sets = []
  # Make a set of sets for each parameter
  nodes = set()
  for i in range(0, len(data.param_names)):
    sets.append([])
    nodes.add(data.param_names[i])

  # Iterate over each datapoint
  for point in data.factors_tuned:
    # If a parameter is changed add the factors adjusted as a set to
    # associated parameters sets
    adjusted_factors = []
    for i in range(0, len(point.adjusted_factors)):
      if (point.adjusted_factors[i]):
        adjusted_factors.append(data.factor_names[i])
    for i in range(0, len(point.adjusted_params)):
      if point.adjusted_params[i]:
        sets[i].append(adjusted_factors)

  # For all parameter sets remove all supersets of smaller sets.
  for param in sets:
    remove_list = []
    for i in range(0,len(param)):
      for j in range(i, len(param)):
        if set(param[i]) < set(param[j]):
          remove_list.append(param[j])
        if set(param[j]) < set(param[i]):
          remove_list.append(param[i])
    for element in remove_list:
      if element in param:
        param.remove(element)

  # Build adjacency list

  # Print adjacency list

  # Find a way to display as a graph
  G = nx.Graph()
  nodes_2 = []
  edges = []
  for i in range (0, len(sets)):
    param = sets[i]
    param_name = data.param_names[i]
    for entry in param:
      nodes_2.append(str(entry))
      edges.append((str(entry), param_name))

  G.add_nodes_from(list(nodes), bipartite=0)
  G.add_nodes_from(list(nodes_2), bipartite=1)
  G.add_edges_from(edges)
  X = set(n for n,d in G.nodes(data=True) if d['bipartite']==0)
  Y = set(G) - X
  pos = dict()
  pos.update( (n, (1, i)) for i, n in enumerate(X) ) # put nodes from X at x=1
  pos.update( (n, (2, i)) for i, n in enumerate(Y) ) # put nodes from Y at x=2
  nx.draw_networkx(G, pos=pos)
  #edge_labels = dict(zip(graph, labels))
  #nx.draw_networkx_edge_labels(G, graph_pos)

  # show graph
  plt.show()


build_graph()