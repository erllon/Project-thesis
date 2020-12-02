from shapely.geometry import Polygon, Point
from shapely.prepared import prep

import numpy as np

class ReflexVertex():
  def __init__(self, xy, parent):
    self.coords = np.array(xy)
    self.point = Point(xy)
    self.parent = parent

class FeasibleSpace(Polygon):

  def __init__(self, mission_space, obstacles = []):
    super().__init__(mission_space, obstacles)
    self.__init_reflex_vertices()
    self.__init_max_line_length()
    self.prepped = prep(self)

  def plot(self, axis):
    x, y = self.exterior.xy
    axis.plot(x, y, color="black")
    for interior in self.interiors:
      x, y = interior.xy
      axis.fill(x, y, fc="grey")

  def is_point_feasible(self, point):
    return self.prepped.contains(Point(point))

  def __init_reflex_vertices(self):
    self.reflex_vertices = []
    for interior in self.interiors:
      int_coords = interior.coords[:-1]
      _max = len(int_coords) - 1
      for i in range(_max + 1):
        v1 = np.array(int_coords[i-1]) - np.array(int_coords[i])
        v2 = np.array(int_coords[i+1] if i < _max else int_coords[0]) - np.array(int_coords[i])
        ang = np.arccos(v1@v2.T/(np.linalg.norm(v1)*np.linalg.norm(v2))).item(0)
        ang = ang if v1[0]*v2[1] > v1[1]*v2[0] else 2*np.pi - ang
        if ang > np.pi:
          self.reflex_vertices.append(ReflexVertex(int_coords[i], interior))

  def __init_max_line_length(self):
    x_min, y_min, x_max, y_max = self.bounds
    self.max_line_length =  max(x_max - x_min, y_max - y_min, np.sqrt((x_max - x_min)**2 + (y_max - y_min)**2))*1000