from agent import Agent
from helpers import get_covered_polygon

import time
import matplotlib.pyplot as plt
import numpy as np
from random import random
from copy import deepcopy

def one_at_the_time_enter(F):
  feasible_area = F.area

  agents = [Agent(np.array([-3.9 + 0.1*i, -3.9], dtype="float32"), F, sensing_radius=3, scheme_degree=3) for i in range(2)]
  for agent in agents:
    agent.other_agents = [agnt for agnt in agents if agnt != agent]
    agent.compute_opt_dependencies()
  
  cmap = plt.cm.get_cmap("hsv", 20)
  _, pos_axs = plt.subplots(2)
  for ax in pos_axs.flat:
    F.plot(ax)
    ax.set_xlim(-4.1, 4.1)
    ax.set_ylim(-4.1, 4.1)
  
  for j in range(len(agents)):
    agents[j].compute_opt_dependencies()
    agents[j].plot(pos_axs[0], color=cmap(j))

  covered_area = get_covered_area(F, agents)
  total_iters = 0
  
  t0 = time.time()
  while covered_area/feasible_area < 0.9:
    print("Computing initial pos for new agent")
    avg_neighbour_dir = agents[-1].get_avg_neighbour_dir()
    max_dist = 3
    dist = None
    initial_pos = agents[-1].s - max_dist*avg_neighbour_dir
    while not F.is_point_feasible(initial_pos):
      dist = random()*max_dist
      initial_pos = agents[-1].s - dist*avg_neighbour_dir
      max_dist = deepcopy(dist)
    print(f"New agent seeded at {initial_pos}")

    new_agent = Agent(initial_pos, F, sensing_radius=3, scheme_degree=3)
    new_agent.other_agents = [agent for agent in agents]
    new_agent.compute_opt_dependencies()
    new_agent.plot(pos_axs.flat[0], cmap(len(agents)))
    new_agent_iters = 0
    agents.append(new_agent)
    while not new_agent.converged:
      new_agent_iters += 1
      total_iters += 1
      new_agent.compute_gradient()
      new_agent.do_step()
      covered_area = get_covered_polygon(F, agents).area
      print(f"step {new_agent_iters} for new agent done. Total iters: {total_iters} Covered area: {covered_area}, local objective change: {new_agent.objective_val - new_agent.prev_objective_val}")
    new_agent.plot(pos_axs.flat[1], cmap(len(agents)))
  t1 = time.time()
  print(f"Elapsed time: {t1-t0}")

  pos_axs[0].set_title("Initial")
  pos_axs[1].set_title(f"Converged after {total_iters} steps")
  for j in range(len(agents)):
    agents[j].plot(pos_axs[1], color=cmap(j))

  plt.show()