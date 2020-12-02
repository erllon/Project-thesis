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
  obstacles = [
    np.array([
      (-15.995, 15.995),
      (-6.005,  15.995),
      (-6.005, -6),
      (-15.995,  -6)
      ]),
    np.array([
      (6, -15.995),
      (15.995,  -15.995),
      (15.995, 10),
      (6,  10)
      ])
  ]

  obstacles2 = [
    np.array([
      (-8, 5),
      (-9, 5),
      (-9, -10),
      (-8,-10)
    ])
    ]
    
  F = FeasibleSpace(
    np.array([
      (-16, -16),
      (16, -16),
      (16, 16),
      (-16, 16)
    ]),obstacles#obstacles2#obstacles2#[] #obstacles
  )

  ############################################
  ############## 3 DRONES ####################
  ############################################
  N_agents =  3
  sensing_r = 4
  comm_r = 2*sensing_r

  agents3 = [
    Agent(
      base_pos + np.array([(i+1)*0.05, (i+1)*0.05]),
      F, sensing_radius=sensing_r, communication_radius=comm_r)
    for i in range(N_agents)]
  for agent in agents3:
    print(f"pos:{agent.s}")
  for agent in agents3:
    agent.other_agents = [a for a in agents3 if a != agent]
    agent.compute_opt_dependencies()
  initial_cover = get_covered_polygon(F, agents3) #Not used
  
  fig, axs = plt.subplots(1,2, num=1)
  
  axs[0].set_title("Final formation, 3 drones")
  axs[0].axis('equal')

  F.plot(axs[0])
  
  hist_obj_val = np.zeros(1)
  cov_area_3 = np.zeros(1)
  obj_val = 0
  while not np.array([agent.converged for agent in agents3]).all():
    for agent in agents3:
      agent.do_step()
      cov_area_3 = np.append(cov_area_3, get_covered_polygon(F,agents3).area)
      obj_val = 0
      for a in agents3:
        obj_val += agent.get_objective_at_point(a.s)
      hist_obj_val = np.append(hist_obj_val, obj_val)
  print(f"hist_obj_val:\n{hist_obj_val}")
  print(f"cov_area_3:\n{cov_area_3}")
    

  F.plot(axs[0])
  positions = []
  for agent in agents3:
    agent.plot(axs[0])
    positions.append(agent.s)
  axs[0].plot(base_pos[0], base_pos[1],"r^")
  

# %% Agents
  for agent in agents3:
    if agent.compute_c(base_pos) == 1:
      x_values_b = [agent.s[0], base_pos[0]]
      y_values_b = [agent.s[1], base_pos[1]]
    
      axs[0].plot(x_values_b, y_values_b, 'r')
    for other in agent.other_agents:
      if agent.compute_c(other.s) == 1:
        agent.neighboors.append(other.node_id)
        x_values = [agent.s[0], other.s[0]]
        y_values = [agent.s[1], other.s[1]]
    
        axs[0].plot(x_values, y_values, 'r')


  fig2, axs2 = plt.subplots(1,2, num=2,clear=True)
  axs2[0].set(xlim=(-4,90), ylim=(-10,225))
  axs2[1].set(xlim=(-4,90), ylim=(-10,225))

  axs2[0].set_title("Covered area, 3 drones")
  axs2[1].set_title("Covered area, 5 drones")

  axs2[0].plot(cov_area_3)

  fi3, axs3 = plt.subplots(num=3)
  axs3.set_title("Covered area")
  axs3.plot(cov_area_3,label="3 drones")

  final_cover = get_covered_polygon(F, agents3)
  if isinstance(final_cover, Polygon):
    x, y = final_cover.exterior.xy
    axs[0].fill(x, y, color='b', alpha=0.5)
  else:
    for geom in final_cover.geoms:
      x, y = geom.exterior.xy
      axs[0].fill(x, y, color='b', alpha=0.5)
  print(f"final covered area: {final_cover.area}")
  print(f"total possibble coverage: {N_agents * np.pi * sensing_r**2}")

  print("************FERDIG MED SIMULERING FOR 3 DRONER***********")
  print("*************BEGYNNER SIMULERING MED 5 DRONER************")

  

















  ############################################
  ############## 5 DRONES ####################
  ############################################

  N_agents =  5

  sensing_r = 4
  comm_r = 2*sensing_r

  agents5 = [
    Agent(
      base_pos + np.array([(i+1)*0.05, (i+1)*0.05]),
      F, sensing_radius=sensing_r, communication_radius=comm_r)
    for i in range(N_agents)]
  for agent in agents5:
    print(f"pos:{agent.s}")
  for agent in agents5:
    agent.other_agents = [a for a in agents5 if a != agent]
    agent.compute_opt_dependencies()
  initial_cover = get_covered_polygon(F, agents5) #Not used

  F.plot(axs[1])
  
  hist_obj_val = np.zeros(1)
  cov_area_5 = np.zeros(1)
  obj_val = 0
  while not np.array([agent.converged for agent in agents5]).all():
    for agent in agents5:
      agent.do_step()
      cov_area_5 = np.append(cov_area_5, get_covered_polygon(F,agents5).area)
      obj_val = 0
      for a in agents5:
        obj_val += agent.get_objective_at_point(a.s)
      hist_obj_val = np.append(hist_obj_val, obj_val)
  print(f"hist_obj_val:\n{hist_obj_val}")
  print(f"cov_area_5:\n{cov_area_5}")

  axs[1].set_title("Final formation, 5 drones")
  axs[1].axis('equal')
  # F.plot(axs[1])

  for agent in agents5:
    agent.plot(axs[1])
    positions.append(agent.s)
  axs[1].plot(base_pos[0], base_pos[1],"r^")  

# %% Agents
  for agent in agents5:
    if agent.compute_c(base_pos) == 1:
      x_values_b = [agent.s[0], base_pos[0]]
      y_values_b = [agent.s[1], base_pos[1]]
    
      axs[1].plot(x_values_b, y_values_b, 'r')

    for other in agent.other_agents:
      if agent.compute_c(other.s) == 1:
        agent.neighboors.append(other.node_id)
        x_values = [agent.s[0], other.s[0]]
        y_values = [agent.s[1], other.s[1]]
    
        axs[1].plot(x_values, y_values, 'r')

  axs2[1].plot(cov_area_5)

  axs3.plot(cov_area_5,label="5 drones")
  axs3.legend()


  final_cover = get_covered_polygon(F, agents5)
  if isinstance(final_cover, Polygon):
    x, y = final_cover.exterior.xy
    axs[1].fill(x, y, color='b', alpha=0.5)
  else:
    for geom in final_cover.geoms:
      x, y = geom.exterior.xy
      axs[1].fill(x, y, color='b', alpha=0.5)
  print(f"final covered area: {final_cover.area}")
  print(f"total possibble coverage: {N_agents * np.pi * sensing_r**2}")

  plt.show()