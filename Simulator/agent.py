from shapely.geometry import Polygon, Point, LineString, MultiPoint
from shapely.ops import unary_union, triangulate, cascaded_union
from shapely.prepared import prep
from copy import deepcopy

from scipy.spatial import Delaunay

import numpy as np
import quadpy as qp

class Agent():
  num_of_drones = 0
  def __init__(
    self, pos, feasible_space,
    sensing_radius = 4, communication_radius = 4, _lambda = 10**(-5),
    p0 = 1, R = lambda x: 1, scheme_degree = 10,
    optimization_tolerance = 10**(-10)
    ):    
    assert feasible_space.is_point_feasible(pos), "Agent initied at infeasible position"
    Agent.num_of_drones += 1
    self.node_id = Agent.num_of_drones
    self.neighboors = []
    self.s = pos
    self.feasible_space = feasible_space
    self.sensing_radius = sensing_radius
    self.communication_radius = communication_radius
    self._lambda = _lambda
    self.p0 = p0
    self.R = R
    self.grad_traj = np.zeros((2, 1))
    self.other_agents = []
    self.downstream = []
    self.upstream = []
    self.scheme_degree = scheme_degree
    self.objective_val = None
    self.prev_objective_val = -float("inf")
    self.optimization_tolerance = optimization_tolerance
    self.converged = False
    

  def get_avg_neighbour_dir(self):
    vec = np.sum([self.other_agents[j].s - self.s for j in self.B], axis=0)
    return vec/np.linalg.norm(vec)
  
  def get_objective_at_point(self, s):
    agent = Agent(pos=s, feasible_space=self.feasible_space, sensing_radius=self.sensing_radius, communication_radius=self.communication_radius, _lambda=self._lambda,
                  p0=self.p0, R=self.R, scheme_degree=self.scheme_degree)
    agent.other_agents = self.other_agents
    agent.compute_opt_dependencies()

    coords = np.array(agent.visible_set.exterior.coords)
    triangles = coords[Delaunay(coords).simplices.T]

    scheme = qp.t2.get_good_scheme(agent.scheme_degree)
    val = scheme.integrate(agent.__compute_objective_aux, triangles)
    return np.sum(val)
    

  def compute_gradient(self):
    E_I = self.__get_E_I()
    E_B = self.__get_E_B()

    self.grad_traj = np.hstack((self.grad_traj, (E_B + E_B).reshape(2, 1)))
    self.gradient = E_I + E_B

  def do_step(self):
    """
    TODO: figure out efficient way to compute step length
    """
    self.compute_gradient()

    if self.converged:
      self.converged = False
    step_size = 4#2/3 #4
    if np.linalg.norm(self.gradient) == 0:
      self.converged = True
      print("converged due to zero gradient")
      return
    step_dir = self.gradient/np.linalg.norm(self.gradient)
    if self.objective_val is None:
      self.objective_val = self.get_objective_at_point(self.s)

    perform_step = False
    step_s = None
    obj_at_s = None
    while not perform_step:
      step_s = self.s + step_size*step_dir
      if (step_s == self.s).all():
        self.converged = True
        print(f"converged due to zero step size {step_size}")
        return
      feasible = self.feasible_space.is_point_feasible(step_s) and self.in_FoV(step_s)
      obj_at_s = -float("inf") if not feasible else self.get_objective_at_point(step_s)
      drones_within_comm_range = np.array([j for j in range(len(self.other_agents)) if np.linalg.norm(step_s - self.other_agents[j].s) <= self.communication_radius])

      if feasible and obj_at_s >= self.objective_val and len(drones_within_comm_range) > 0: # ==len(self.B): 
        perform_step = True
      else:
        step_size *= 0.5 #0.5 produces good results, 0.8 leads to loops in the network 
    print(f"step direction: {step_dir}")
    self.s = step_s
    self.prev_objective_val = deepcopy(self.objective_val)
    self.objective_val = obj_at_s
    self.compute_opt_dependencies()
    self.converged = self.objective_val - self.prev_objective_val < 10**(-3)
    if self.converged:
      print("converged due to small change in objective")

  def get_step_length(self):
    alpha_bar = 1
    rho = 0.5
    c = 10**(-4)
    self.compute_gradient()
    p_k = self.gradient
    grad_f_k = self.gradient
    f_k = self.get_objective_at_point(self.s)
    alpha = alpha_bar
    while self.get_objective_at_point(self.s + alpha*p_k) < f_k + c*alpha*grad_f_k.reshape(2, 1).T@p_k.reshape(2, 1)\
      or not self.feasible_space.is_point_feasible(self.s + alpha*p_k):
      alpha *= rho
    return alpha
    

  def compute_opt_dependencies(self):
    self.set_B()
    self.__set_visible_set()
    self.__set_anchors()
    self.__set_impact_points()

  def P(self, x, agents):
    N = len(agents)
    self.set_B()
    self.set_C(agents)
    phi = self.phi(x, self.B)
    phi_bar = self.phi(x, self.C)
    theta = self.theta(x, self.B)
    theta_bar = self.theta(x, self.C)
    psi = self.psi(x, self.B)
    psi_bar = self.psi(x, self.C)
    return 1 + (N-2)*phi*phi_bar - theta*phi_bar - theta_bar*phi\
      - ((1/2)*(N-2)*(N-1)*phi*phi_bar - (N-2)*(theta*phi_bar + theta_bar*phi) + theta*theta_bar + psi*phi_bar + psi_bar*phi)*(1 - self.p_hat(x))

  
  def plot(self, axis, color="red"):
    axis.scatter(self.s[0], self.s[1], color=color)
    xs, ys = self.visible_set.exterior.xy
    axis.fill(xs, ys, alpha=0.1, fc=color, ec='none')

  def __compute_objective_aux(self, points):
    return self.R(points)*self.phi(points,self.B)*self.p(points)

  def in_FoV(self, x):
    line = LineString([self.s, x])
    for interior in self.feasible_space.interiors:
      if interior.intersects(line):
        return False
    return True

  def __set_visible_set_aux(self, vs):
    self.visible_set = vs
    self.prepped_visible_set = prep(vs)
    self.prepped_visible_set_boundary = prep(vs.boundary)

  def __set_visible_set(self):
    num_interiors = len(self.feasible_space.interiors)
    polygons = []
    for i in range(num_interiors):
      interior_polygon = Polygon(self.feasible_space.interiors[i])
      for j in range(len(self.feasible_space.interiors[i].coords)-1):
        p1 = np.array(LineString([self.s, self.feasible_space.interiors[i].coords[j]]).difference(interior_polygon).coords[1])
        p2 = np.array(LineString([self.s, self.feasible_space.interiors[i].coords[j+1]]).difference(interior_polygon).coords[1])
        polygons.append(Polygon([
          p1,
          p1 + (p1 - self.s)*self.feasible_space.max_line_length,
          p2 + (p2 - self.s)*self.feasible_space.max_line_length,
          p2
        ]))
    vs = Point(self.s).buffer(self.sensing_radius).difference(cascaded_union(polygons)).intersection(self.feasible_space) #unary_union
    if not isinstance(vs, Polygon):
      self.__set_visible_set_aux(max(vs, key = lambda geom: geom.area))
    else:
      self.__set_visible_set_aux(vs)

  def __set_anchors(self):
    visible_rvs = [rv for rv in self.feasible_space.reflex_vertices if self.prepped_visible_set_boundary.contains(rv.point)]
    self.anchors = []

    for i in range(len(visible_rvs)):
      l = LineString([visible_rvs[i].coords, visible_rvs[i].coords + (visible_rvs[i].coords - self.s)*self.feasible_space.max_line_length])
      if visible_rvs[i].parent.touches(l):
        self.anchors.append(visible_rvs[i].coords)
    return self.anchors

  def __set_impact_points(self):
    self.impact_points = [None]*len(self.anchors)
    for i in range(len(self.anchors)):
      l = LineString([self.anchors[i], self.anchors[i] + (self.anchors[i] - self.s)*self.feasible_space.max_line_length])
      intersection = self.feasible_space.boundary.intersection(l).difference(Point(self.anchors[i]))
      if not isinstance(intersection, Point):
        intersection = min(intersection, key = lambda point: np.linalg.norm(self.s - np.array(point)))
      self.impact_points[i] = np.array([intersection.x, intersection.y])
  
  def __get_anchor_normal_vector(self, anchor):
    n_opts = [np.array([n_x_opt, -((anchor[0]-self.s[0])/(anchor[1]-self.s[1]))*n_x_opt]) for n_x_opt in [-1, 1]]

    for i in range(2):
      n_opts[i] = n_opts[i]/np.linalg.norm(n_opts[i])

    epsilon = 10**(-5)

    for n_opt in n_opts:
      if self.prepped_visible_set.contains(Point(anchor + epsilon*n_opt)):
        return n_opt
    assert False, f"No normal vector to {anchor} pointing to the interior of visible_set"
  
  def set_B(self):
    self.B = np.array([j for j in range(len(self.other_agents)) if np.linalg.norm(self.s - self.other_agents[j].s) <= 2*self.sensing_radius])

  def set_C(self, agents):
    self.C = np.array([j for j in range(len(self.other_agents)) if np.linalg.norm(self.s - self.other_agents[j].s) > 2*self.sensing_radius])

  def compute_c(self, s_other):
    return 1 if np.linalg.norm(self.s - s_other) <= 1.01*self.communication_radius else 0
    

  def p(self, x):
    pos = None
    try:
      _, w, d = x.shape
      pos = np.repeat(np.repeat(self.s.reshape(2, 1, -1), w, axis=1), d, axis=2)
    except ValueError:
      _, w = x.shape
      pos = np.repeat(self.s.reshape(2, 1), w, axis=1)
    return self.p0*np.exp(-self._lambda*(np.linalg.norm(pos - x, axis=0)-self.sensing_radius))

  def p_hat(self, x):
    mask = None
    try:
      _, w, d = x.shape
      mask = np.array([[self.prepped_visible_set.contains(Point(x[:, i, j])) for j in range(d)] for i in range(w)])
    except ValueError:
      _, w = x.shape
      mask = np.array([self.prepped_visible_set.contains(Point(x[:, i])) for i in range(w)])
    return self.p(x)*mask
  
  def der_p_d_dist(self, x):
    return -self._lambda*self.p(x)

  def __get_ret_shape(self, x):
    try:
      _, w, d = x.shape
      return (w, d)
    except ValueError:
      _, w = x.shape
      return (1, w)

  def phi(self, x, B):
    ones = np.ones(self.__get_ret_shape(x))
    try:
      return np.prod(
        np.stack(
          [
            ones - self.other_agents[j].p_hat(x) for j in B
          ], axis=0
        ), axis = 0
      )
    except ValueError:
      return ones

  def theta(self, x, B):
    try:
      return np.sum(
        np.stack(
          [
            self.phi(x, B[np.where(B != j)]) for j in B
          ], axis=0
        ), axis = 0
      )
    except ValueError:
      return np.zeros(self.__get_ret_shape(x))

  def psi(self, x, B):
    try:
      lB = len(B)
      return np.sum(
        np.stack(
          [
            self.phi(x, B[np.logical_and(B != B[j], B != B[k])]) for j in range(lB) for k in range(j+1, lB)
          ], axis=0
        ), axis = 0
      )
    except ValueError:
      return np.zeros(self.__get_ret_shape(x))

  def g_V(self, x):
    lB = len(self.B)
    phi = self.phi(x, self.B)
    theta = self.theta(x, self.B)
    psi = self.psi(x, self.B)
    ret = (lB-1)*((lB/2)*phi - theta) + psi
    #ret = phi
    #print(phi.shape, theta.shape, psi.shape, ret.shape)
    return ret

  def __E_I_aux(self, points):
    _, w, d = points.shape
    pos = np.repeat(np.repeat(self.s.reshape(2, 1, -1), w, axis=1), d, axis=2)
    return self.phi(points,self.B)*self.der_p_d_dist(points)*(pos-points)/np.linalg.norm(pos - points, axis=0) #self.g_V(points)*self.der_p_d_dist(points)*(pos-points)/np.linalg.norm(pos - points, axis=0)

  def __get_E_I(self):
    coords = np.array(self.visible_set.exterior.coords)
    triangles = coords[Delaunay(coords).simplices.T]
    
    scheme = qp.t2.get_good_scheme(self.scheme_degree)
    val = scheme.integrate(self.__E_I_aux, triangles)
    return np.sum(val, axis=1)

  def get_rho(self, r, anchor, impact_point):
    diff = (impact_point - anchor).reshape(2, 1)
    return diff*r/np.linalg.norm(diff) + anchor.reshape(2, 1)

  def __E_B_aux(self, r, anchor, impact_point):
    rho = self.get_rho(r, anchor, impact_point)
    # return self.R(rho)*self.g_V(rho)*self.p(rho)*r
    return self.R(rho)*self.phi(rho, self.B)*self.p(rho)*r
  
  def __get_E_B(self):
    anchor_indices = []
    for i in range(len(self.anchors)):
      if np.linalg.norm(self.s - self.anchors[i]) < self.sensing_radius:
        anchor_indices.append(i)
    E_B = np.zeros((2, ))
    for index in anchor_indices:
      anchor_vec = self.s - self.anchors[index]
      anchor_angle = np.arctan(anchor_vec[1]/anchor_vec[0])
      self_to_anchor_dist = np.linalg.norm(self.s - self.anchors[index])
      integral = qp.c1.gauss_patterson(7).integrate(
        lambda r: self.__E_B_aux(r, self.anchors[index], self.impact_points[index]),
        [
          0,
          min(
            np.linalg.norm(self.anchors[index] - self.impact_points[index]),
            self.sensing_radius - self_to_anchor_dist
          )
        ]).item(0)
      E_B += (1/self_to_anchor_dist)*np.sign(self.__get_anchor_normal_vector(self.anchors[index]))*np.array([np.sin(anchor_angle), np.cos(anchor_angle)])*integral
    return E_B