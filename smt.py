"""
__author__ = chrislaw
__project__ = RRT*_LTL
__date_
"""
# Optimization-based Trajectory Generation with Linear Temporal Logic Specifications


from z3 import *
import cplex
import datetime
import numpy as np
from Problem import problemFormulation
import triangle
import triangle.plot as plot
import matplotlib.pyplot as plt
from WorkspacePlot import region_plot
from itertools import repeat
import sys
from cLTL_problem import case_0, case_1, case_2, case_3, case_4, OR


def IMPLIES(b1, b2):
    return z3.Implies(b1, b2)


def BoolVar2Int(b):
    return If(b, 1, 0)


def tri(x, y):
    # plot the plan
    box = triangle.get_data('box')
    #
    # ax1 = plt.subplot(121, aspect='equal')
    # triangle.plot.plot(ax1, **box)

    t = triangle.triangulate(box, 'pc')

    ax2 = plt.subplot(111)  # , sharex=ax1, sharey=ax1)
    plot.plot(ax2, **t)
    plt.plot(x, y, 'yo-')

    plt.show()


def contain(v_a, v_b, v_c):
    # whether a point is in the triangle
    # https://stackoverflow.com/questions/14757920/count-points-inside-triangle-fast
    x_a = v_a[0]
    y_a = v_a[1]
    x_b = v_b[0]
    y_b = v_b[1]
    x_c = v_c[0]
    y_c = v_c[1]
    v_0 = np.array([x_c - x_a, y_c - y_a])
    v_1 = np.array([x_b - x_a, y_b - y_a])

    dot00 = np.dot(v_0, v_0)
    dot01 = np.dot(v_0, v_1)
    dot11 = np.dot(v_1, v_1)

    invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01)
    x1 = invDenom * dot11 * (x_c - x_a) - invDenom * dot01 * (x_b - x_a)
    y1 = invDenom * dot11 * (y_c - y_a) - invDenom * dot01 * (y_b - y_a)
    c1 = -invDenom * dot11 * x_a * (x_c - x_a) - invDenom * dot11 * y_a * (y_c - y_a) \
         + invDenom * dot01 * x_a * (x_b - x_a) + invDenom * dot01 * y_a * (y_b - y_a)

    x2 = invDenom * dot00 * (x_b - x_a) - invDenom * dot01 * (x_c - x_a)
    y2 = invDenom * dot00 * (y_b - y_a) - invDenom * dot01 * (y_c - y_a)
    c2 = -invDenom * dot00 * x_a * (x_b - x_a) - invDenom * dot00 * y_a * (y_b - y_a) + invDenom * dot01 * x_a * (
        x_c - x_a) + invDenom * dot01 * y_a * (y_c - y_a)

    x3 = x1 + x2
    y3 = y1 + y2
    c3 = c1 + c2

    # lhs = np.array([[x1 * (-1), y1 * (-1), -1, 0, 0, 0, 0, 0.0],
    #                 [x2 * (-1), y2 * (-1), 0, -1, 0, 0, 0, 0],
    #                 [x3, y3, 0, 0, -1, 0, 0, 0],
    #                 [0, 0, 1, 0, 0, -1, 0, 0],
    #                 [0, 0, -1, 0, 0, -1, 0, 0],
    #                 [0, 0, 0, 1, 0, 0, -1, 0],
    #                 [0, 0, 0, -1, 0, 0, -1, 0],
    #                 [0, 0, 0, 0, 1, 0, 0, -1],
    #                 [0, 0, 0, 0, -1, 0, 0, -1],
    #                 ]
    #                )
    # rhs = np.array([c1, c2, 1 - c3, 0, 0, 0, 0, 0, 0.0])
    # sense = ["L", "L", "L", "L", "L", "L", "L", "L", "L"]

    lhs = np.array([[x1 * (-1), y1 * (-1)],
                    [x2 * (-1), y2 * (-1)],
                    [x3, y3]]
                   )

    rhs = np.array([c1, c2, 1 - c3])
    sense = ["L", "L", "L"]

    return lhs, rhs, sense


# number of robots case : #robots
robot = {1: 1,
         2: 2,
         3: 16,
         4: 16,
         5: 16,
         6: 20,
         7: 20,
         21: 4,
         32: 32}
n_Robot = robot[32]  # robot[int(sys.argv[1])]
# specify the case
case = 4  # case = 3 including 3,4,5,6,7
# iteration starting point case:= : #robots
ite = {3: 27,
       2: 18,
       1: 21,
       21: 23,
       4: 30}

# inilize SAT solver
start_z3 = datetime.datetime.now()

L = 100

r = 0.2
# ------------------------ r = 0.2 -------------------------
# adjacency relation
adj_region = {0: [0],
              1: [1, 2, 4],
              2: [2, 1, 3, 37],
              3: [3, 2, 6],
              4: [4, 1, 5, 37],
              5: [5, 4, 9],
              6: [6, 3, 7, 42],
              7: [7, 6, 8, 12],
              8: [8, 7, 9, 37],
              9: [9, 5, 8, 10],
              10: [10, 9, 11, 24],
              11: [11, 10, 23, 40],
              12: [12, 7, 14, 40],
              13: [13, 14, 42],
              14: [14, 12, 13, 15],
              15: [15, 14, 16, 20],
              16: [16, 15, 17, 41],
              17: [17, 16, 18],
              18: [18, 17, 19],
              19: [19, 18, 41],
              20: [20, 15, 21, 40],
              21: [21, 20, 22],
              22: [22, 21, 23, 34],
              23: [23, 11, 22, 39],
              24: [24, 10, 25],
              25: [25, 24, 26, 30],
              26: [26, 25, 27, 38],
              27: [27, 26, 28],
              28: [28, 27, 29],
              29: [29, 28, 33, 38],
              30: [30, 25, 31, 38],
              31: [31, 30, 32, 39],
              32: [32, 31, 33, 35],
              33: [33, 29, 32],
              34: [34, 22, 35, 39],
              35: [35, 32, 34, 36],
              36: [36, 35],
              37: [37, 2, 4, 8],  # l1
              38: [38, 26, 29, 30],  # l2
              39: [39, 23, 31, 34],  # l3
              40: [40, 11, 12, 20],  # l4
              41: [41, 16, 19],  # l5
              42: [42, 6, 13],  # l6
              }

# labeled region
region_dict = {"l1": 37,
               "l2": 38,
               "l3": 39,
               "l4": 40,
               "l5": 41,
               "l6": 42}
tri_dict = {37: "l1",
            38: "l2",
            39: "l3",
            40: "l4",
            41: "l5",
            42: "l6"}

n_Region = len(adj_region)

for n_Horizon in range(ite[case], L):
# for n_Horizon in range(int(sys.argv[2]), L):
# for n_Horizon in [27, 28, 27]:
    # Z3 solver
    s = Solver()
    s.reset()

    # declare Boolean variables denoting robot i is in region j at horizon (time) k
    #            position           |
    # robot      [               ]  | horizon 1
    #            position           |
    # robot      [               ]  | horizon 2

    Robot_Region_Horizon = BoolVector("b", n_Robot * n_Region * n_Horizon)

    loop = BoolVector("l", n_Horizon)
    loop_c = [loop[0] == False] + [sum([If(loop[horizonCounter], 1, 0)
                                        for horizonCounter in
                                        range(1, n_Horizon)]) == 1]  # loop starts from the second step
    # loop constraint
    loop_region_c = [
        IMPLIES(loop[k], Robot_Region_Horizon[(k - 1) * n_Region * n_Robot + robotCounter * n_Region + region]
                == Robot_Region_Horizon[(n_Horizon - 1) * n_Region * n_Robot + robotCounter * n_Region + region])
        for k in range(1, n_Horizon) for robotCounter in range(n_Robot) for region in range(n_Region)]

    s.add(loop_c + loop_region_c)

    # initial state all in tri 36
    init_c = [Robot_Region_Horizon[robotCounter * n_Region + 36] for robotCounter in range(n_Robot)]
    s.add(init_c)

    # each robot has to be in one region at any time instance
    one = []
    for horizonCounter in range(0, n_Horizon):
        indexShift = horizonCounter * n_Region * n_Robot
        for robotCounter in range(0, n_Robot):
            one = one + [sum([BoolVar2Int(Robot_Region_Horizon[i + robotCounter * n_Region + indexShift]) for i in
                              range(0, n_Region)]) == 1]
    s.add(one)

    # constraints on adjacent regions
    adj = []
    for horizonCounter in range(0, n_Horizon - 1):
        indexShift = horizonCounter * n_Region * n_Robot
        indexShift_plus = (horizonCounter + 1) * n_Region * n_Robot
        for robotCounter in range(n_Robot):
            for counter in range(0, n_Region):
                adjacent = adj_region[counter]
                child = [Robot_Region_Horizon[i + robotCounter * n_Region + indexShift_plus] for i in adjacent]
                # print 'anticedent of', counter+indexShift, 'is ', anticedent
                adj = adj + [IMPLIES(Robot_Region_Horizon[counter + robotCounter * n_Region + indexShift], OR(*child))]
                # the * used to unpack the python list to arguments                        )
    s.add(adj)

    # case selection
    if case == 1:
        case_1(n_Robot, n_Region, n_Horizon, region_dict, s, Robot_Region_Horizon, loop)
    elif case == 2:
        case_2(n_Robot, n_Region, n_Horizon, region_dict, s, Robot_Region_Horizon, loop)
    elif case == 3:
        case_3(n_Robot, n_Region, n_Horizon, region_dict, s, Robot_Region_Horizon, loop)
    elif case == 21:
        case_2_1(n_Robot, n_Region, n_Horizon, region_dict, s, Robot_Region_Horizon, loop)
    elif case == 4:
        case_4(n_Robot, n_Region, n_Horizon, region_dict, s, Robot_Region_Horizon, loop)
    # without considering formulation
    start_z3_less = datetime.datetime.now()
    if s.check() == sat:
        z3_time_less = (datetime.datetime.now() - start_z3_less).total_seconds()
        z3_time = (datetime.datetime.now() - start_z3).total_seconds()
        print(z3_time, z3_time_less)
        # print("Success when # horizon is {0}".format(n_Horizon))
        m = s.model()
        # print the plan
        for robotCounter in range(n_Robot):
            print("Robot ", robotCounter + 1, ': ', end=""),
            for horizonCounter in range(1, n_Horizon):
                if m.evaluate(loop[horizonCounter]):
                    print('<{0}> '.format(horizonCounter + 1), end="")
            for horizonCounter in range(n_Horizon):
                for counter in range(n_Region):
                    indexShift = horizonCounter * n_Robot * n_Region + robotCounter * n_Region
                    if m.evaluate(Robot_Region_Horizon[indexShift + counter]):
                        try:
                            print("[[", tri_dict[counter], "]]", ' --> ', end="")
                        except KeyError:
                            print(counter, '--> ', end=""),
                        break
            print()

        x = [[] for i in repeat(None, n_Robot)]
        y = [[] for i in repeat(None, n_Robot)]

        for robotCounter in range(n_Robot):
            for horizonCounter in range(n_Horizon):
                x[robotCounter].append(values[8 * n_Robot * horizonCounter + robotCounter * 8])
                y[robotCounter].append(values[8 * n_Robot * horizonCounter + robotCounter * 8 + 1])

        z = [[] for i in repeat(None, n_Horizon)]

        for horizonCounter in range(n_Horizon):
            for robotCounter in range(n_Robot):
                z[horizonCounter] = z[horizonCounter] + [x[robotCounter][horizonCounter], y[robotCounter][horizonCounter]]

        cost = 0
        for horizonCounter in range(n_Horizon-1):
            cost = cost + np.linalg.norm(np.subtract(z[horizonCounter+1], z[horizonCounter]))
        # print("cost", cost/2)
        print(z3_time, z3_time_less, cplex_time, cost/2)
        # workspace, regions, obs, init_state, uni_cost, formula, formula_comp, exclusion, no = problemFormulation().Formulation()
        # ts = {'workspace': workspace, 'region': regions, 'obs': obs, 'uni_cost': uni_cost}
        # # plot the workspace
        # ax = plt.figure(1).gca()
        # region_plot(regions, 'region', ax)
        # region_plot(obs, 'obs', ax)
        # color = ['r', 'y', 'b', 'k', 'c', 'g']
        # for robotCounter in range(n_Robot):
        #     pre = plt.quiver(x[robotCounter][:-1], y[robotCounter][:-1], np.array(x[robotCounter][1:]) - np.array(x[robotCounter][:-1]), np.array(y[robotCounter][1:]) - np.array(y[robotCounter][:-1]), color='{0}'.format(color[robotCounter]),
        #                      scale_units='xy', angles='xy', scale=1)
        #     plt.plot(x[robotCounter], y[robotCounter], '{0}o-'.format(color[robotCounter]))
        # plt.show()
        # break
    else:
        print("Failure when # horizon is {0}".format(n_Horizon))
