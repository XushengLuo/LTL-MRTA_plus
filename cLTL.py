import datetime
from workspace import Workspace
import matplotlib.pyplot as plt
from z3 import *
from termcolor import colored
import pickle


def AND(*b):
    return z3.And(b)


def OR(*b):
    return z3.Or(b)


def IMPLIES(b1, b2):
    return z3.Implies(b1, b2)


def BoolVar2Int(b):
    return If(b, 1, 0)


def until(subformula, k, L, loop):
    if k == L:
        return Or(subformula[L + k - 1],  # plus 1
                  And([subformula[k - 1]] + [Or([And(loop[i], until_aux(subformula, i + 1, L)) for i in range(1, L)])]))
    else:
        return Or(subformula[L + k - 1], And(subformula[k - 1], until(subformula, k + 1, L, loop)))


def until_aux(subformula, k, L):
    if k == L:
        return subformula[L + k - 1]
    else:
        return Or(subformula[L + k - 1], And(subformula[k - 1], until_aux(subformula, k + 1, L)))


def encode_workspace(ws):
    code = 0
    # map code to cell
    code2cell = dict()
    for w in range(ws.width):
        for l in range(ws.length):
            if (w, l) in ws.obstacles.values():
                continue
            else:
                code2cell[code] = (w, l)
                code += 1
    # map cell to code
    cell2code = {cell: code for code, cell in code2cell.items()}

    # adjacency between regions
    adjacent_region = dict()
    for code in code2cell.keys():
        adjacent_region[code] = [code] + [cell2code[c] for c in ws.graph_workspace.neighbors(code2cell[code])]

    # map region to code
    region2code = {region: cell2code[cell] for region, cell in ws.regions.items()}
    code2region = {code: region for region, code in region2code.items()}

    # initial location
    robot2code = {ws.n*(type_robot[0]-1)+type_robot[1]: cell2code[location]
                  for type_robot, location in ws.type_robot_location.items()}

    return code2cell, cell2code, adjacent_region, region2code, code2region, robot2code


def cost(path, l):
    c = 0
    for robot, p in path.items():
        for t in range(l-1):
            c = c + abs(p[t][0]-p[t+1][0]) + abs(p[t][1]-p[t+1][1])
        for t in range(l-1, len(p)-1):
            c = c + abs(p[t][0]-p[t+1][0]) + abs(p[t][1]-p[t+1][1])
        c = c + abs(p[len(p)-1][0]-p[l-1][0]) + abs(p[len(p)-1][1]-p[l-1][1])

    return c*0.5


def constraint(s, adj_region, robot2code, n_Robot, n_Region, ws):

    start_z3_less = datetime.datetime.now()

    # declare Boolean variables denoting robot i is in region j at horizon (time) k
    #            position           |
    # robot      [               ]  | horizon 1
    #            position           |
    # robot      [               ]  | horizon 2

    Robot_Region_Horizon = BoolVector("b", n_Robot * n_Region * n_Horizon)

    loop = BoolVector("l", n_Horizon)
    loop_c = [sum([If(loop[horizonCounter], 1, 0) for horizonCounter in
                   range(1, n_Horizon)]) == 1]
    # loop constraint
    loop_region_c = [
        IMPLIES(loop[k], Robot_Region_Horizon[(k - 1) * n_Region * n_Robot + robotCounter * n_Region + region]
                == Robot_Region_Horizon[(n_Horizon - 1) * n_Region * n_Robot + robotCounter * n_Region + region])
        for k in range(1, n_Horizon-1) for robotCounter in range(n_Robot) for region in range(n_Region)]

    s.add(loop_c + loop_region_c)

    # initial state
    init_c = [Robot_Region_Horizon[robotCounter * n_Region + robot2code[robotCounter]]
              for robotCounter in range(n_Robot)]
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
                adj = adj + [
                    IMPLIES(Robot_Region_Horizon[counter + robotCounter * n_Region + indexShift], OR(*child))]
                # the * used to unpack the python list to arguments                        )
    s.add(adj)

    # case  <> l2_1_1
    # n_formula = 2
    # formula = BoolVector('f', n_formula * n_Horizon)
    # # type = 1
    # formulaCounter = 0
    # # region = region2code["l2"]
    # formula_1 = [formula[formulaCounter * n_Horizon + horizonCounter] for horizonCounter in range(n_Horizon)] + \
    #             [formula[(formulaCounter + 1) * n_Horizon + horizonCounter]
    #              == (And([sum([BoolVar2Int(Robot_Region_Horizon[n_Robot * n_Region *
    #                 horizonCounter + robot * n_Region + region2code["l2"]]) for robot in range(0*ws.n, 1*ws.n)]) >= 1,
    #                       sum([BoolVar2Int(Robot_Region_Horizon[n_Robot * n_Region *
    #                 horizonCounter + robot * n_Region + region2code["l3"]]) for robot in range(1*ws.n, 2*ws.n)]) >= 2]))
    #              for horizonCounter in range(n_Horizon)]
    # eventually = [until(formula[formulaCounter * n_Horizon: (formulaCounter + 2) * n_Horizon], 1, n_Horizon, loop)]
    # s.add(formula_1 + eventually)

    # case []<> (l1_1_1 && l1_2_1)
    n_formula = 5
    formula = BoolVector('f', n_formula * n_Horizon)
    formulaCounter = 0
    formula_1 = [formula[formulaCounter * n_Horizon + horizonCounter]
                 == (And([sum([BoolVar2Int(Robot_Region_Horizon[n_Robot * n_Region *
                    horizonCounter + robot * n_Region + region2code["l1"]]) for robot in range(0*ws.n, 1*ws.n)]) >= ws.n//3,
                          sum([BoolVar2Int(Robot_Region_Horizon[n_Robot * n_Region *
                    horizonCounter + robot * n_Region + region2code["l1"]]) for robot in range(1*ws.n, 2*ws.n)]) >= ws.n//3]))
                 for horizonCounter in range(n_Horizon)]
    always_eventually_1 = [
        Or([And(loop[i], Or([formula[formulaCounter * n_Horizon + horizonCounter]
                             for horizonCounter in range(i, n_Horizon)])) for i in
            range(1, n_Horizon)])]
    s.add(formula_1 + always_eventually_1)

    # case []<> (l2_1_1 && l2_2_1)
    formulaCounter += 1
    formula_2 = [formula[formulaCounter * n_Horizon + horizonCounter]
                 == (And([sum([BoolVar2Int(Robot_Region_Horizon[n_Robot * n_Region *
                    horizonCounter + robot * n_Region + region2code["l2"]]) for robot in range(1*ws.n, 2*ws.n)]) >= ws.n//3,
                          sum([BoolVar2Int(Robot_Region_Horizon[n_Robot * n_Region *
                    horizonCounter + robot * n_Region + region2code["l2"]]) for robot in range(2*ws.n, 3*ws.n)]) >= ws.n//3]))
                 for horizonCounter in range(n_Horizon)]
    always_eventually_2 = [
        Or([And(loop[i], Or([formula[formulaCounter * n_Horizon + horizonCounter]
                             for horizonCounter in range(i, n_Horizon)])) for i in
            range(1, n_Horizon)])]
    s.add(formula_2 + always_eventually_2)

    # case []<> (l3_3_1 && l3_4_1)
    formulaCounter += 1
    formula_3 = [formula[formulaCounter * n_Horizon + horizonCounter]
                 == (And([sum([BoolVar2Int(Robot_Region_Horizon[n_Robot * n_Region *
                    horizonCounter + robot * n_Region + region2code["l3"]]) for robot in range(2*ws.n, 3*ws.n)]) >= ws.n//3,
                          sum([BoolVar2Int(Robot_Region_Horizon[n_Robot * n_Region *
                    horizonCounter + robot * n_Region + region2code["l3"]]) for robot in range(3*ws.n, 4*ws.n)]) >= ws.n//3]))
                 for horizonCounter in range(n_Horizon)]
    always_eventually_3 = [
        Or([And(loop[i], Or([formula[formulaCounter * n_Horizon + horizonCounter]
                             for horizonCounter in range(i, n_Horizon)])) for i in
            range(1, n_Horizon)])]
    s.add(formula_3 + always_eventually_3)

    # case []<> (l4_4_1 && l4_5_1)
    formulaCounter += 1
    formula_4 = [formula[formulaCounter * n_Horizon + horizonCounter]
                 == (And([sum([BoolVar2Int(Robot_Region_Horizon[n_Robot * n_Region *
                    horizonCounter + robot * n_Region + region2code["l4"]]) for robot in range(3*ws.n, 4*ws.n)]) >= ws.n//3,
                          sum([BoolVar2Int(Robot_Region_Horizon[n_Robot * n_Region *
                    horizonCounter + robot * n_Region + region2code["l4"]]) for robot in range(4*ws.n, 5*ws.n)]) >= ws.n//3]))
                 for horizonCounter in range(n_Horizon)]
    always_eventually_4 = [
        Or([And(loop[i], Or([formula[formulaCounter * n_Horizon + horizonCounter]
                             for horizonCounter in range(i, n_Horizon)])) for i in
            range(1, n_Horizon)])]
    s.add(formula_4 + always_eventually_4)

    if s.check() == sat:
        z3_time_less = (datetime.datetime.now() - start_z3_less).total_seconds()
        z3_time = (datetime.datetime.now() - start_z3).total_seconds()
        # print(z3_time, z3_time_less)
        # print("Success when # horizon is {0}".format(n_Horizon))
        m = s.model()

        l = 0
        path = dict()
        for robotCounter in range(n_Robot):
            path[(robotCounter//workspace.n+1, robotCounter % workspace.n)] = []
            # print("Robot ", (robotCounter//workspace.n+1, robotCounter % workspace.n), ': ', end=""),
            for horizonCounter in range(1, n_Horizon):
                if m.evaluate(loop[horizonCounter]):
                    # print('<{0}> '.format(horizonCounter + 1), end="")
                    l = horizonCounter
            for horizonCounter in range(n_Horizon):
                for counter in range(n_Region):
                    indexShift = horizonCounter * n_Robot * n_Region + robotCounter * n_Region
                    if m.evaluate(Robot_Region_Horizon[indexShift + counter]):
                        path[(robotCounter // workspace.n + 1, robotCounter % workspace.n)].append(code2cell[counter])
                        # try:
                        #     try:
                        #         print("[[", colored(code2region[counter], 'yellow'), "]]", ' --> ', end="")
                        #     except KeyError:
                        #         print("[[", code2cell[counter], "]]", ' --> ', end="")
                        # except KeyError:
                        #     print(counter, '--> ', end=""),
                        break
            # print()
        c = cost(path, l)
        print(int(sys.argv[2]), n_Horizon, z3_time, z3_time_less, c)

        return True
    return False


# workspace = Workspace()
with open('data/workspace', 'rb') as filehandle:
    workspace = pickle.load(filehandle)
code2cell, cell2code, adjacent_region, region2code, code2region, robot2code = encode_workspace(workspace)

start_z3 = datetime.datetime.now()
# inilize SAT solver
for n_Horizon in range(int(sys.argv[2]), 20):
    # Z3 solver
    s = Solver()
    s.reset()
    if constraint(s, adjacent_region, robot2code, len(workspace.type_num)*list(workspace.type_num.values())[0],
                  len(adjacent_region), workspace):
        break

# # print(code2cell)
# # print(cell2code)
# # print(adjacent_region)

# workspace.plot_workspace()
# plt.show()