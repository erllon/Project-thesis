from agent import Agent
from feasible_space import FeasibleSpace
import numpy as np
import matplotlib.pyplot as plt
from itertools import combinations
from scipy.spatial import distance

from helpers import get_covered_polygon

from random import random
from copy import deepcopy

import time

from shapely.geometry import Polygon, MultiPolygon
from shapely.ops import unary_union

from multiprocessing import Process

import time

def do_step(agents):
  agent.do_step()

def closest_node(node, nodes):
    closest_index = distance.cdist([node], nodes).argmin()
    return nodes[closest_index]

  

if __name__ == "__main__":

  # set styles
  try:
      # installed with "pip install SciencePLots" (https://github.com/garrettj403/SciencePlots.git)
      # gives quite nice plots
      plt_styles = ["science", "grid", "bright", "no-latex"]
      plt.style.use(plt_styles)
      print(f"pyplot using style set {plt_styles}")
  except Exception as e:
      print(e)
      print("setting grid and only grid and legend manually")
      plt.rcParams.update(
          {
              # setgrid
              "axes.grid": False,#True,
              "grid.linestyle": ":",
              "grid.color": "k",
              "grid.alpha": 0.5,
              "grid.linewidth": 0.5,
              # Legend
              "legend.frameon": True,
              "legend.framealpha": 1.0,
              "legend.fancybox": True,
              "legend.numpoints": 1,
          }
      )

  base_pos = np.array([-16,-16])
  # obstacles = [
  #   np.array([
  #     (0.26-0.5, -0.80),
  #     (0.26-0.5,  0.80),
  #     (0.25-0.5,  0.80),
  #     (0.25-0.5, -0.80)
  #   ])]
  obstacles = [
    # np.array([
    #   (-6, -6),
    #   (-5,  -6),
    #   (-5, 15.995),
    #   (-6,  15.995)]),
    np.array([
      (-15.995, 15.995),
      (-6.005,  15.995),
      (-6.005, -6),
      (-15.995,  -6)
      ]),
    np.array([
      (6, -15.995),
      (7,  -15.995),
      (7, 10),
      (6,  10)
      ])
  ]
  # obstacles = [
  #     np.array([
  #       (-1,-15.995),
  #       (0, -15.995),
  #       (0, -5),
  #       (-1,-5)
  #     ]),
  #     np.array([
  #       (-15.995,-1),
  #       (-10, -1),
  #       (-10, 0),
  #       (-15.995, 0)
  #     ])
  #    ]

  obstacles2 = [
    np.array([
      (-8, 5),
      (-9, 5),
      (-9, -10),
      (-8,-10)
    ])
    ]
    #Obstacles2 over fungerer bra med 3 droner
  # obstacles2 = [
  #     np.array([
  #       (-6, 5),
  #       (-7, 5),
  #       (-7, -5),
  #       (-6, -5)
  #     ]),
  #     np.array([
  #       (-7.005, -4),
  #       (-7.005, -5),
  #       (-15.995,-5),
  #       (-15.995,-4)
  #     ])
  #    ]
    
  F = FeasibleSpace(
    np.array([
      (-16, -16),
      (16, -16),
      (16, 16),
      (-16, 16)
    ]),obstacles#obstacles2#obstacles2#[] #obstacles
  )

  N_agents =  6# 3funker fint med stor obstacle
  #3 funker ok
  #3,4 funker ok, 5 funker ikke

  sensing_r = 4
  comm_r = 2*sensing_r#2*sensing_r

  agents = [
    Agent(
      base_pos + np.array([(i+1)*0.05, (i+1)*0.05]),#+ np.random.uniform(0, 0.16, 2), #np.array([-16, -16]) + np.random.uniform(0, 0.16, 2),
      F, sensing_radius=sensing_r, communication_radius=comm_r)
    for i in range(N_agents)]
  for agent in agents:
    print(f"pos:{agent.s}")
  for agent in agents:
    agent.other_agents = [a for a in agents if a != agent]
    agent.compute_opt_dependencies()
  initial_cover = get_covered_polygon(F, agents) #Not used
  
  
  fig, axs = plt.subplots(2)
  
  axs[0].set_title("Initial")
  axs[0].axis('equal')

  F.plot(axs[0])
  for agent in agents:
    agent.plot(axs[0])
  

  #obj_val = 0
  hist_obj_val = np.zeros(1)
  cov_area = np.zeros(1)
  obj_val = 0
  while not np.array([agent.converged for agent in agents]).all():
    for agent in agents:
      agent.do_step()
      cov_area = np.append(cov_area, get_covered_polygon(F,agents).area)
      obj_val = 0
      for a in agents:
        obj_val += agent.get_objective_at_point(a.s)
      hist_obj_val = np.append(hist_obj_val, obj_val)
      # obj_val = 0
      # for a in agents:
      #   obj_val += agent.objective_val
      #hist_obj_val = np.append(hist_obj_val, obj_val)
  print(f"hist_obj_val:\n{hist_obj_val}")
  print(f"cov_area:\n{cov_area}")

    
  colors = ['g','r','c','m','y','k']
  axs[1].set_title("Final")
  axs[1].axis('equal')
  F.plot(axs[1])
  positions = []
  for agent in agents:
  #for i in range(len(agents)):
    #agents[i].plot(axs[1], colors[i])
    agent.plot(axs[1])
    #positions.append(agents[i].s)
    positions.append(agent.s)
  axs[1].plot(base_pos[0], base_pos[1],"r^")
    # agent.compute_opt_dependencies()
  
  # for i in range(len(agents)):
  #   print(f"Pos{i}: {agents[i].s}")
  #   # print(f"final covered area: {final_cover.area}")
  # pos_agents = []
  # for agent in agents:
  #   pos_agents.append(agent.s)

# %% Base
# for agent in agents:

#   pos = closest_node(base_pos,positions)

#   x_values = [base_pos[0], pos[0]]
#   y_values = [base_pos[1], pos[1]]

#   axs[1].plot(x_values, y_values, 'b')
  

# %% Agents
  for agent in agents:
    #agent.compute_c()
    # conn = []
    # other_positions = []
    if agent.compute_c(base_pos) == 1:
      x_values_b = [agent.s[0], base_pos[0]]
      y_values_b = [agent.s[1], base_pos[1]]
    
      axs[1].plot(x_values_b, y_values_b, 'b')
    for other in agent.other_agents:
      if agent.compute_c(other.s) == 1:# and agent.in_FoV(other.s) == True:
        agent.neighboors.append(other.node_id)
        x_values = [agent.s[0], other.s[0]]
        y_values = [agent.s[1], other.s[1]]
    
        axs[1].plot(x_values, y_values, 'b')
      #other_positions.append(other.s)
    #pos = closest_node(agent.s, other_positions)
    
    # x_values = [agent.s[0], pos[0]]
    # y_values = [agent.s[1], pos[1]]
    # agent.compute_opt_dependencies()
    # neighbor_positions = []
    # for neigh_idx in agent.B:
    #   neigh_agent = agents[neigh_idx]
    #   x_values = [agent.s[0], neigh_agent.s[0]]
    #   y_values = [agent.s[1], neigh_agent.s[1]]
    #   axs[1].plot(x_values, y_values, 'b')

    # axs[1].plot(x_values, y_values, 'b')


  fig2, axs2 = plt.subplots(num=2,clear=True)
  axs2.set_title("Covered area")
  axs2.plot(range(len(cov_area)), cov_area)

  fig3, axs3 = plt.subplots(num=3,clear=True)
  axs3.set_title("Cost function")
  axs3.plot(range(len(hist_obj_val)), hist_obj_val)


  final_cover = get_covered_polygon(F, agents)
  if isinstance(final_cover, Polygon):
    x, y = final_cover.exterior.xy
    axs[1].fill(x, y, color="blue", alpha=0.5)
  else:
    for geom in final_cover.geoms:
      x, y = geom.exterior.xy
      axs[1].fill(x, y, color="blue", alpha=0.5)
  #print(f"initial covered area: {initial_cover.area}")
  print(f"final covered area: {final_cover.area}")
  print(f"total possibble coverage: {N_agents * np.pi * sensing_r**2}")

  plt.show()

  