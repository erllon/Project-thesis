from shapely.geometry import Polygon, MultiPolygon
from shapely.ops import unary_union, cascaded_union, polygonize
from itertools import combinations

def get_covered_polygon(feasible_space, agents):
  intersections = []
  for agent in agents:
    intersections.append(agent.visible_set)
  
  return cascaded_union(intersections)