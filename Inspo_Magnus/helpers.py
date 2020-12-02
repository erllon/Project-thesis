from shapely.geometry import Polygon, MultiPolygon
from shapely.ops import unary_union, cascaded_union, polygonize
from itertools import combinations

def get_covered_polygon(feasible_space, agents):
  intersections = []
  visible_sets = []
  # exteriors = []
  # for agent in agents:
  #   #visible_sets.append(agent.visible_set)
  #   exteriors.append(Polygon(agent.visible_set.exterior))
  # for combination in combinations(agents, 3):
  #   intersection = combination[0].visible_set
  #   for i in range(1, 3):      
  #     intersection = intersection.union(combination[i].visible_set) #intersection.intersection(combination[i].visible_set)
  #     intersections.append(intersection)
  for agent in agents:
    intersections.append(agent.visible_set)
  
  return cascaded_union(intersections) #intersection(intersections)# intersections#Polygon(intersections).area
  # intersections.append(agents[0].visible_set)
  # return cascaded_union(intersections) #unary_union(intersections)
  # p = MultiPolygon(exteriors)
  # return cascaded_union(p)#(visible_sets)