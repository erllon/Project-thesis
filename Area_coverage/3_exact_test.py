import numpy as np
import shapely.geometry as sg
from shapely.ops import cascaded_union
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from scipy.integrate import dblquad

from set_test import get_index_combs

class Agnt():
  def __init__(self, s, r = 1):
    self.s = s
    self.r = r
    self.geom = sg.Point(self.s).buffer(self.r)
  
  def p_hat(self, x):
    return 1 - (1/(1 + np.power(10**10, -(np.linalg.norm(self.s - x) - self.r))))

  def plot(self, ax):
    ax.scatter(*tuple(self.s), color="red")
    ax.add_patch(Circle(tuple(self.s), self.r, color="red", alpha=0.1))

def get_act_area(agnts):
  N = len(agnts)
  index_combinations = get_index_combs(N, 3)
  union = sg.Polygon()
  for index_combination in index_combinations:
    intersection = agnts[index_combination[0]].geom
    for i in range(1, len(index_combination)):
      intersection = intersection.intersection(agnts[index_combination[i]].geom)
    union = union.union(intersection)
  return union

def P(agnts, x):
  _sum = 0
  for index_combination in get_index_combs(len(agnts), 3):
    prod = 1
    for j in range(len(agnts)):
      if j in index_combination:
        prod *= agnts[j].p_hat(x)
      else:
        prod *= 1 - agnts[j].p_hat(x)
    _sum += prod
  return _sum

import math 
if __name__ == "__main__":
  agnts = [
    Agnt(np.array([-0.5, -1])),
    Agnt(np.array([0, 0])),
    Agnt(np.array([0.5, -1])),
  ]
  fig, ax = plt.subplots()
  for agnt in agnts:
    agnt.plot(ax)

  u = cascaded_union([agnt.geom for agnt in agnts])
  xmin, ymin, xmax, ymax = u.bounds
  integral, err = dblquad(lambda y, x: P(agnts, np.array([x, y])), xmin, xmax, lambda x: ymin, lambda x: ymax)
  print(integral)
  area = get_act_area(agnts)
  print(f"actual area: {area.area}")
  x, y = area.exterior.xy
  ax.fill(x, y, color="blue", alpha = 0.5)
  plt.show()