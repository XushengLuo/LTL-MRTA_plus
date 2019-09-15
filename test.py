# import networkx as nx
# import datetime
# s = datetime.datetime.now()
# d = nx.DiGraph()
# d.add_nodes_from([1, 2, 3, 4])
# d.add_edges_from([(1,2),(2,4), (1,4)])
# d = nx.transitive_reduction(d)
# for e in d.edges:
#     print(e)
# width = max([len(o) for o in nx.antichains(d)])
# print(width)
# print((datetime.datetime.now()-s).total_seconds())
#
# import math
# import random
# from gurobipy import *
# m = Model()
# a = m.addVars([1,2 ], [1,2])
# e = [quicksum([a[1,1]]), quicksum([a[1,2]])]
# print(e)
#
# import networkx as nx
# g = nx.DiGraph()
# a = {'w': 2}
# g.add_edge(1, 2, **a)
# print(g)

# expr = LinExpr()
# for i in [0, 1]:
#     expr.add(e[i], 2)
#
# print(1/3)
#
#
#

# # Callback - use lazy constraints to eliminate sub-tours
#
# def subtourelim(model, where):
#   if where == GRB.callback.MIPSOL:
#     selected = []
#     # make a list of edges selected in the solution
#     for i in range(n):
#       sol = model.cbGetSolution([model._vars[i,j] for j in range(n)])
#       selected += [(i,j) for j in range(n) if sol[j] > 0.5]
#     # find the shortest cycle in the selected edge list
#     tour = subtour(selected)
#     if len(tour) < n:
#       # add a subtour elimination constraint
#       expr = 0
#       for i in range(len(tour)):
#         for j in range(i+1, len(tour)):
#           expr += model._vars[tour[i], tour[j]]
#       model.cbLazy(expr <= len(tour)-1)
#
#
# # Euclidean distance between two points
#
# def distance(points, i, j):
#   dx = points[i][0] - points[j][0]
#   dy = points[i][1] - points[j][1]
#   return math.sqrt(dx*dx + dy*dy)
#
#
# # Given a list of edges, finds the shortest subtour
#
# def subtour(edges):
#   visited = [False]*n
#   cycles = []
#   lengths = []
#   selected = [[] for i in range(n)]
#   for x,y in edges:
#     selected[x].append(y)
#   while True:
#     current = visited.index(False)
#     thiscycle = [current]
#     while True:
#       visited[current] = True
#       neighbors = [x for x in selected[current] if not visited[x]]
#       if len(neighbors) == 0:
#         break
#       current = neighbors[0]
#       thiscycle.append(current)
#     cycles.append(thiscycle)
#     lengths.append(len(thiscycle))
#     if sum(lengths) == n:
#       break
#   return cycles[lengths.index(min(lengths))]
#
# n = 50
#
# # Create n random points
#
# random.seed(1)
# points = []
# for i in range(n):
#   points.append((random.randint(0,100),random.randint(0,100)))
#
# m = Model()
#
#
# # Create variables
#
# vars = {}
# for i in range(n):
#    for j in range(i+1):
#      vars[i,j] = m.addVar(obj=distance(points, i, j), vtype=GRB.BINARY,
#                           name='e'+str(i)+'_'+str(j))
#      vars[j,i] = vars[i,j]
#    m.update()
#
#
# # Add degree-2 constraint, and forbid loops
#
# for i in range(n):
#   m.addConstr(quicksum(vars[i,j] for j in range(n)) == 2)
#   vars[i,i].ub = 0
#
# m.update()
#
#
# # Optimize model
#
# m._vars = vars
# m.params.LazyConstraints = 1
# m.optimize(subtourelim)
#
# solution = m.getAttr('x', vars)
# selected = [(i,j) for i in range(n) for j in range(n) if solution[i,j] > 0.5]
# assert len(subtour(selected)) == n


# for order in strict_poset_relation:
#     if order[1] == element:
#         larger_edge_label = pruned_subgraph.edges[element2edge[order[0]]]['label']
#         for c_edge in range(len(larger_edge_label)):
#             i = element_component_clause_literal_node[(order[0], 1, c_edge, 0)][0]  # any node
#             m.addConstr(quicksum(t_vars[(i, k, 1)]
#                                  for k in range(type_num[ts.nodes[i]['location_type_component'][1]]))
#                         + M * (c_vars[(order[0], 1, c_edge)] - 1) <=
#                         quicksum(t_vars[(clause_nodes[0][0], k, 1)]
#                                  for k in range(type_num[ts.nodes[clause_nodes[0][0]]['location_type_component'][1]]))
#                         + M * (1 - c_vars[(element, 1, c)]))

# for order in strict_poset_relation:
#     if order[1] == element:
#         larger_edge_label = pruned_subgraph.edges[element2edge[order[0]]]['label']
#         for c_edge in range(len(larger_edge_label)):
#             i = element_component_clause_literal_node[(order[0], 1, c_edge, 0)][0]
#             for l in clause_nodes:
#                 for j in l:
#                     m.addConstr(quicksum(t_vars[(j, k, 0)] for k in
#                                          range(type_num[ts.nodes[i]['location_type_component'][1]]))
#                                 + M * (c_vars[(element, 0, c)] - 1) <=
#                                 quicksum(t_vars[(i, k, 1)]
#                                          for k in range(type_num[ts.nodes[j]['location_type_component'][1]]))
#                                 + M * (1 - c_vars[(element, 1, c_edge)]))

# import matplotlib.pyplot as plt
# import numpy as np
#
# # run-length encoding, instead of a list of lists with a bunch of zeros
# data = [(2, 1), (1, 2), (3, 1), (2, 2), (2, 3), (1, 1), (1, 3), (1, 1), (1, 3)]
#
# fig = plt.figure()
# ax = fig.add_subplot(111)
# ax.axes.get_yaxis().set_visible(False)
# ax.set_aspect(1)
#
#
# def avg(a, b):
#     return (a + b) / 2.0
#
# plt.rc('text', usetex=True)
#
# for i, (num, cat) in enumerate(data):
#
#     if i > 0:
#         x_start += data[i-1][0] # get previous end position
#     else:
#         x_start = i             # start from 0
#
#     x1 = [x_start, x_start+num]
#
#     y1 = [0, 0]
#     y2 = [1, 1]
#
#     if cat == 1:
#         plt.fill_between(x1, y1, y2=y2, color='red')
#         plt.text(avg(x1[0], x1[1]), avg(y1[0], y2[0]), r'${}_{{{}}}$'.format('l', 1),
#                  horizontalalignment='center',
#                  verticalalignment='center')
#     if cat == 2:
#         plt.fill_between(x1, y1, y2=y2, color='green')
#         plt.text(avg(x1[0], x1[1]), avg(y1[0], y2[0]), r'${}_{{{}}}$'.format('l', 2),
#                  horizontalalignment='center',
#                  verticalalignment='center')
#     if cat == 3:
#         plt.fill_between(x1, y1, y2=y2, color='yellow')
#         plt.text(avg(x1[0], x1[1]), avg(y1[0], y2[0]), r'${}_{{{}}}$'.format('l', 3),
#                  horizontalalignment='center',
#                  verticalalignment='center')
#
# plt.ylim(1, 0)
# plt.show()


# """
# Animation of Elastic collisions with Gravity
#
# author: Jake Vanderplas
# email: vanderplas@astro.washington.edu
# website: http://jakevdp.github.com
# license: BSD
# Please feel free to use and modify this, but keep the above information. Thanks!
# """
# import numpy as np
# from scipy.spatial.distance import pdist, squareform
#
# import matplotlib.pyplot as plt
# import scipy.integrate as integrate
# import matplotlib.animation as animation
#
#
# class ParticleBox:
#     """Orbits class
#
#     init_state is an [N x 4] array, where N is the number of particles:
#        [[x1, y1, vx1, vy1],
#         [x2, y2, vx2, vy2],
#         ...               ]
#
#     bounds is the size of the box: [xmin, xmax, ymin, ymax]
#     """
#
#     def __init__(self,
#                  init_state=[[1, 0, 0, -1],
#                              [-0.5, 0.5, 0.5, 0.5],
#                              [-0.5, -0.5, -0.5, 0.5]],
#                  bounds=[-2, 2, -2, 2],
#                  size=0.04,
#                  M=0.05,
#                  G=9.8):
#         self.init_state = np.asarray(init_state, dtype=float)
#         self.M = M * np.ones(self.init_state.shape[0])
#         self.size = size
#         self.state = self.init_state.copy()
#         self.time_elapsed = 0
#         self.bounds = bounds
#         self.G = G
#
#     def step(self, dt):
#         """step once by dt seconds"""
#         self.time_elapsed += dt
#
#         # update positions
#         self.state[:, :2] += dt * self.state[:, 2:]
#
#         # find pairs of particles undergoing a collision
#         D = squareform(pdist(self.state[:, :2]))
#         ind1, ind2 = np.where(D < 2 * self.size)
#         unique = (ind1 < ind2)
#         ind1 = ind1[unique]
#         ind2 = ind2[unique]
#
#         # update velocities of colliding pairs
#         for i1, i2 in zip(ind1, ind2):
#             # mass
#             m1 = self.M[i1]
#             m2 = self.M[i2]
#
#             # location vector
#             r1 = self.state[i1, :2]
#             r2 = self.state[i2, :2]
#
#             # velocity vector
#             v1 = self.state[i1, 2:]
#             v2 = self.state[i2, 2:]
#
#             # relative location & velocity vectors
#             r_rel = r1 - r2
#             v_rel = v1 - v2
#
#             # momentum vector of the center of mass
#             v_cm = (m1 * v1 + m2 * v2) / (m1 + m2)
#
#             # collisions of spheres reflect v_rel over r_rel
#             rr_rel = np.dot(r_rel, r_rel)
#             vr_rel = np.dot(v_rel, r_rel)
#             v_rel = 2 * r_rel * vr_rel / rr_rel - v_rel
#
#             # assign new velocities
#             self.state[i1, 2:] = v_cm + v_rel * m2 / (m1 + m2)
#             self.state[i2, 2:] = v_cm - v_rel * m1 / (m1 + m2)
#
#             # check for crossing boundary
#         crossed_x1 = (self.state[:, 0] < self.bounds[0] + self.size)
#         crossed_x2 = (self.state[:, 0] > self.bounds[1] - self.size)
#         crossed_y1 = (self.state[:, 1] < self.bounds[2] + self.size)
#         crossed_y2 = (self.state[:, 1] > self.bounds[3] - self.size)
#
#         self.state[crossed_x1, 0] = self.bounds[0] + self.size
#         self.state[crossed_x2, 0] = self.bounds[1] - self.size
#
#         self.state[crossed_y1, 1] = self.bounds[2] + self.size
#         self.state[crossed_y2, 1] = self.bounds[3] - self.size
#
#         self.state[crossed_x1 | crossed_x2, 2] *= -1
#         self.state[crossed_y1 | crossed_y2, 3] *= -1
#
#         # add gravity
#         self.state[:, 3] -= self.M * self.G * dt
#
#
# # ------------------------------------------------------------
# # set up initial state
# np.random.seed(0)
# init_state = -0.5 + np.random.random((50, 4))
# init_state[:, :2] *= 3.9
#
# box = ParticleBox(init_state, size=0.04)
# dt = 1. / 30  # 30fps
#
# # ------------------------------------------------------------
# # set up figure and animation
# fig = plt.figure()
# fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
# ax = fig.add_subplot(111, aspect='equal', autoscale_on=False,
#                      xlim=(-3.2, 3.2), ylim=(-2.4, 2.4))
#
# # particles holds the locations of the particles
# particles, = ax.plot([], [], 'bo', ms=6)
#
# # rect is the box edge
# rect = plt.Rectangle(box.bounds[::2],
#                      box.bounds[1] - box.bounds[0],
#                      box.bounds[3] - box.bounds[2],
#                      ec='none', lw=2, fc='none')
# ax.add_patch(rect)
#
#
# def init():
#     """initialize animation"""
#     global box, rect
#     particles.set_data([], [])
#     rect.set_edgecolor('none')
#     return particles, rect
#
#
# def animate(i):
#     """perform animation step"""
#     global box, rect, dt, ax, fig
#     box.step(dt)
#
#     ms = int(fig.dpi * 2 * box.size * fig.get_figwidth()
#              / np.diff(ax.get_xbound())[0])
#
#     # update pieces of the animation
#     rect.set_edgecolor('k')
#     for i in range(25):
#         particles.set_data(box.state[:i, 0], box.state[:i, 1])
#         particles.set_color('r')
#     # particles.set_data(box.state[:25, 0], box.state[:25, 1])
#     # particles.set_color('r')
#     # particles.set_data(box.state[49:, 0], box.state[49:, 1])
#
#     # particles.set_markersize(ms)
#     return particles, rect
#
#
# ani = animation.FuncAnimation(fig, animate, frames=600,
#                               interval=1000, blit=True, init_func=init)
#
# # save the animation as an mp4.  This requires ffmpeg or mencoder to be
# # installed.  The extra_args ensure that the x264 codec is used, so that
# # the video can be embedded in html5.  You may need to adjust this for
# # your system: for more information, see
# # http://matplotlib.sourceforge.net/api/animation_api.html
# ani.save('particle_box.mp4', fps=40, dpi=400, extra_args=['-vcodec', 'libx264'])

# plt.show()

# import matplotlib.pyplot as plt
# import matplotlib.animation as anim
# import numpy as np

# n = 14

# class ParticleBox:
#     def __init__(self,i_state,c_state):
#         self.i_state = np.asarray(i_state, dtype=float)
#         self.c_state = np.asarray(c_state, dtype=float)
#         self.state = self.i_state.copy()
#         self.color = self.c_state.copy()
#     def iterate(self):
#
#
#         self.state += (np.random.random((n, 2))-0.5)/3.
#         self.state[self.state > 10.] = 10
#         self.state[self.state < -10] = -10
#
#         self.color += (np.random.random(n)-0.5)/89.
#         self.color[self.color>1.] = 1.
#         self.color[self.color<0] = 0.
#
#         # return annots
#
# i_state = -5 + 10 * np.random.random((n, 2))
# # c_state = np.random.random(n)
# c_state = np.array([0.5]*n)
#
# pbox = ParticleBox(i_state, c_state)
#
# fig = plt.figure()
# ax =  fig.add_subplot(111, xlim=(-10,10), ylim=(-10,10))
#
# particles = ax.scatter([], [], c=[],s=25, cmap="hsv", vmin=0, vmax=1)
# annots = [ax.text(100,100,"p") for _ in range(n)]
#
# def animate(i):
#     pbox.iterate()
#
#     for t, new_x_i, new_y_i in zip(annots, pbox.state[:, 0], pbox.state[:, 1]):
#         t.set_position((new_x_i, new_y_i))
#
#     particles.set_offsets(pbox.state)
#     particles.set_array(pbox.color)
#     return [particles]+annots
#
# ani = anim.FuncAnimation(fig, animate, frames = 60,
#                          interval = 300, blit=True)
# ani.save('clock.mp4', fps=1.0, dpi=200)
#
# plt.show()


import itertools
import operator

data=[(1, 'A', 'foo'),
    (2, 'A', 'bar'),
    (2, 'A', 'foo-bar'),

    ('xx', 'B', 'foobar'),
    ('yy', 'B', 'foo'),

    (1000, 'C', 'py'),
    (200, 'C', 'foo'),
    ]

for key,group in itertools.groupby(data,operator.itemgetter(0,1)):
    print(key, list(group))
