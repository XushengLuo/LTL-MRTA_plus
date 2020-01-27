from z3 import *


def AND(*b):
    return z3.And(b)


def OR(*b):
    return z3.Or(b)


def NOT(b):
    return z3.Not(b)


def always(subformula, k, L):
    if k == L:
        return subformula[k - 1]
    else:
        return And(subformula[k - 1], always(subformula, k + 1, L))


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


def case_0(n_Robot, n_Region, n_Horizon, s, Robot_Region_Horizon, loop):
    n_formula = 2
    formula = BoolVector('f', n_formula * n_Horizon)
    robot = 0
    region = 37
    formulaCounter = 0
    formula_2 = [formula[formulaCounter * n_Horizon + horizonCounter] for horizonCounter in range(n_Horizon)] + \
                [formula[(formulaCounter + 1) * n_Horizon + horizonCounter] == Robot_Region_Horizon[n_Robot * n_Region *
                                                                                                    horizonCounter + robot * n_Region + region]
                 for horizonCounter in range(n_Horizon)]
    eventually = [until(formula[formulaCounter * n_Horizon: (formulaCounter + 2) * n_Horizon], 1, n_Horizon, loop)]
    # always_eventually = [Or([And(loop[i], Or([formula[formulaCounter * n_Horizon + horizonCounter]
    #                                           for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
    s.add(formula_2 + eventually)


def case_1(n_Robot, n_Region, n_Horizon, region_dict, s, Robot_Region_Horizon, loop):
    # A. Single Robot Planning
    n_formula = 9
    formula = BoolVector('f', n_formula * n_Horizon)
    #             horizon_1      ......      horizon_n
    #
    # formula1_1
    #    .
    #    .
    # formulan_1
    # []!l5_1
    robot = 0
    region = region_dict["l5"]
    formulaCounter = 0
    formula_1 = [formula[formulaCounter * n_Horizon + horizonCounter] == Not(Robot_Region_Horizon[n_Robot * n_Region *
                                                                                                  horizonCounter + robot * n_Region + region])
                 for horizonCounter in range(n_Horizon)]
    always_not = [always(formula[formulaCounter * n_Horizon: (formulaCounter + 1) * n_Horizon], 1, n_Horizon)]
    s.add(formula_1 + always_not)

    # <> l4_1
    robot = 0
    region = region_dict["l4"]
    formulaCounter = formulaCounter + 1
    formula_2 = [formula[formulaCounter * n_Horizon + horizonCounter] for horizonCounter in range(n_Horizon)] + \
                [formula[(formulaCounter + 1) * n_Horizon + horizonCounter] == Robot_Region_Horizon[n_Robot * n_Region *
                                                                                                    horizonCounter + robot * n_Region + region]
                 for horizonCounter in range(n_Horizon)]
    eventually = [until(formula[formulaCounter * n_Horizon: (formulaCounter + 2) * n_Horizon], 1, n_Horizon, loop)]
    # always_eventually = [Or([And(loop[i], Or([formula[formulaCounter * n_Horizon + horizonCounter]
    #                                           for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
    s.add(formula_2 + eventually)

    # <> l1_1
    robot = 0
    region = region_dict["l1"]
    formulaCounter = formulaCounter + 2
    formula_3 = [formula[formulaCounter * n_Horizon + horizonCounter] for horizonCounter in range(n_Horizon)] + [
        formula[(formulaCounter + 1) * n_Horizon + horizonCounter] == Robot_Region_Horizon[n_Robot * n_Region *
                                                                                           horizonCounter + robot * n_Region + region]
        for horizonCounter in range(n_Horizon)]
    # l3_1
    region = region_dict["l3"]
    formula_4 = [formula[(formulaCounter + 2) * n_Horizon + horizonCounter] == Robot_Region_Horizon[n_Robot * n_Region *
                                                                                                    horizonCounter + robot * n_Region + region]
                 for horizonCounter in range(n_Horizon)]
    # l3_1 && <> l1_1
    formula_5 = [formula[(formulaCounter + 3) * n_Horizon + horizonCounter] ==
                 And(formula[(formulaCounter + 2) * n_Horizon + horizonCounter],
                     until(formula[formulaCounter * n_Horizon: (formulaCounter + 2) * n_Horizon], horizonCounter + 1,
                           n_Horizon, loop)) for horizonCounter in range(n_Horizon)]
    # []<> (l3 && <> l1_1)
    always_eventually = [
        Or([And(loop[i], Or([formula[(formulaCounter + 3) * n_Horizon + horizonCounter]
                             for horizonCounter in range(i, n_Horizon)])) for i in
            range(1, n_Horizon)])]
    s.add(formula_3 + formula_4 + formula_5 + always_eventually)

    # !l1_1 U l2_1
    robot = 0
    region1 = region_dict["l1"]
    region2 = region_dict["l2"]
    formulaCounter = formulaCounter + 4
    formula_6 = [formula[formulaCounter * n_Horizon + horizonCounter] == Not(Robot_Region_Horizon[n_Robot * n_Region *
                                                                                                  horizonCounter + robot * n_Region + region1])
                 for horizonCounter in range(n_Horizon)] + [
                    formula[(formulaCounter + 1) * n_Horizon + horizonCounter] == Robot_Region_Horizon[
                        n_Robot * n_Region *
                        horizonCounter + robot * n_Region + region2] for horizonCounter in range(n_Horizon)]
    not_until = [until(formula[formulaCounter * n_Horizon: (formulaCounter + 2) * n_Horizon], 1, n_Horizon, loop)]
    s.add(formula_6 + not_until)


def case_2(n_Robot, n_Region, n_Horizon, region_dict, s, Robot_Region_Horizon, loop):
    # B. Two Robots Planning
    n_formula = 5
    formula = BoolVector('f', n_formula * n_Horizon)

    # []<> l1_1
    robot = 0
    formulaCounter = 0
    region = region_dict["l1"]
    formula_1 = [formula[formulaCounter * n_Horizon + horizonCounter] == Robot_Region_Horizon[n_Robot * n_Region *
                                                                                              horizonCounter + robot * n_Region + region]
                 for horizonCounter in range(n_Horizon)]
    always_eventually_1 = [
        Or([And(loop[i], Or([formula[formulaCounter * n_Horizon + horizonCounter]
                             for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
    s.add(formula_1 + always_eventually_1)

    # []<> l2_2
    robot = 1
    formulaCounter = formulaCounter + 1
    region = region_dict["l2"]
    formula_2 = [formula[formulaCounter * n_Horizon + horizonCounter] == Robot_Region_Horizon[n_Robot * n_Region *
                                                                                              horizonCounter + robot * n_Region + region]
                 for horizonCounter in range(n_Horizon)]
    always_eventually_2 = [
        Or([And(loop[i], Or([formula[formulaCounter * n_Horizon + horizonCounter]
                             for horizonCounter in range(i, n_Horizon)])) for i in
            range(1, n_Horizon)])]
    s.add(formula_2 + always_eventually_2)

    # <> l4_2
    robot1 = 1
    region = region_dict["l4"]
    formulaCounter = formulaCounter + 1
    formula_3 = [formula[formulaCounter * n_Horizon + horizonCounter] for horizonCounter in range(n_Horizon)] + [
        formula[(formulaCounter + 1) * n_Horizon + horizonCounter] == Robot_Region_Horizon[n_Robot * n_Region *
                                                                                           horizonCounter + robot1 * n_Region + region]
        for horizonCounter in range(n_Horizon)]
    # l4_1
    robot2 = 0
    region = region_dict["l4"]
    # formula_4 = [formula[(formulaCounter + 2) * n_Horizon + horizonCounter] == Robot_Region_Horizon[n_Robot * n_Region *
    #                    horizonCounter + robot2 * n_Region + region] for horizonCounter in range(n_Horizon)]
    # l4_1 && <> l4_2
    formula_4 = [formula[(formulaCounter + 2) * n_Horizon + horizonCounter] ==
                 And(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + robot2 * n_Region + region],
                     until(formula[formulaCounter * n_Horizon: (formulaCounter + 2) * n_Horizon], horizonCounter + 1,
                           n_Horizon, loop)) for horizonCounter in range(n_Horizon)]
    # []<> (l4_1 && <> l4_2)
    always_eventually = [
        Or([And(loop[i], Or([formula[(formulaCounter + 2) * n_Horizon + horizonCounter]
                             for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
    s.add(formula_3 + formula_4 + always_eventually)


def case_2_1(n_Robot, n_Region, n_Horizon, region_dict, s, Robot_Region_Horizon, loop):
    # n_formula = 1
    # formula = BoolVector('f', n_formula * n_Horizon)
    #
    # # []<> (l1_1 && l2_2 && l3_3 && l4_4 && l5_5 && l6_6)
    # formulaCounter = 0
    # formula_1 = [formula[formulaCounter * n_Horizon + horizonCounter] == And(
    #     Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 0 * n_Region + region_dict["l1"]],
    #     Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 1 * n_Region + region_dict["l2"]],
    #     Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 2 * n_Region + region_dict["l3"]],
    #     Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 3 * n_Region + region_dict["l4"]],
    #     Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 4 * n_Region + region_dict["l5"]],
    #     Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 5 * n_Region + region_dict["l6"]]
    #     ) for horizonCounter in
    #              range(n_Horizon)]
    #
    # always_eventually_1 = [
    #     Or([And(loop[i], Or([formula[formulaCounter * n_Horizon + horizonCounter]
    #                          for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
    # s.add(formula_1 + always_eventually_1)

    # []<> (l1_1 && (l1_2 || l1_3) && (l1_4 || l1_5) && l1_6)
    n_formula = 9
    formula = BoolVector('f', n_formula * n_Horizon)
    formulaCounter = 0
    # []<> (l1_1 && l1_2)
    formula_1 = [formula[formulaCounter * n_Horizon + horizonCounter] == And(
        Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 0 * n_Region + region_dict["l1"]],
        Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 1 * n_Region + region_dict["l1"]]) for
                 horizonCounter in
                 range(n_Horizon)]
    always_eventually_1 = [
        Or([And(loop[i],
                Or([formula[formulaCounter * n_Horizon + horizonCounter]
                    for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
    s.add(formula_1 + always_eventually_1)

    # []<> (l2_2 && l2_3)
    formulaCounter = formulaCounter + 1
    formula_2 = [formula[formulaCounter * n_Horizon + horizonCounter] == And(
        Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 1 * n_Region + region_dict["l2"]],
        Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 2 * n_Region + region_dict["l2"]]) for
                 horizonCounter in
                 range(n_Horizon)]
    always_eventually_2 = [
        Or([And(loop[i],
                Or([formula[formulaCounter * n_Horizon + horizonCounter]
                    for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
    s.add(formula_2 + always_eventually_2)

    # []<> (l3_3 && l3_4)
    formulaCounter = formulaCounter + 1
    formula_3 = [formula[formulaCounter * n_Horizon + horizonCounter] == And(
        Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 2 * n_Region + region_dict["l3"]],
        Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 3 * n_Region + region_dict["l3"]]) for
                 horizonCounter in
                 range(n_Horizon)]
    always_eventually_3 = [
        Or([And(loop[i],
                Or([formula[formulaCounter * n_Horizon + horizonCounter]
                    for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
    s.add(formula_3 + always_eventually_3)

    # <> l6_3
    robot1 = 2
    region = region_dict["l6"]
    formulaCounter = formulaCounter + 1
    formula_3 = [formula[formulaCounter * n_Horizon + horizonCounter] for horizonCounter in range(n_Horizon)] + [
        formula[(formulaCounter + 1) * n_Horizon + horizonCounter] == Robot_Region_Horizon[n_Robot * n_Region *
                                                                                           horizonCounter + robot1 * n_Region + region]
        for horizonCounter in range(n_Horizon)]
    # l5_4 && <> l6_3
    robot2 = 3
    region = region_dict["l5"]
    formula_4 = [formula[(formulaCounter + 2) * n_Horizon + horizonCounter] ==
                 And(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + robot2 * n_Region + region],
                     until(formula[formulaCounter * n_Horizon: (formulaCounter + 2) * n_Horizon],
                           horizonCounter + 1,
                           n_Horizon, loop)) for horizonCounter in range(n_Horizon)]
    # <> (l5_4 && <> l6_3)
    formula_5 = [formula[(formulaCounter + 3) * n_Horizon + horizonCounter] for horizonCounter in
                 range(n_Horizon)] + [
                    formula[(formulaCounter + 4) * n_Horizon + horizonCounter] == formula[
                        (formulaCounter + 2) * n_Horizon + horizonCounter]
                    for horizonCounter in range(n_Horizon)]
    # l4_1 && <> (l5_4 && <> l6_3)
    robot3 = 0
    region = region_dict["l4"]
    formula_6 = [formula[(formulaCounter + 5) * n_Horizon + horizonCounter] ==
                 And(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + robot3 * n_Region + region],
                     until(formula[(formulaCounter + 3) * n_Horizon: (formulaCounter + 5) * n_Horizon],
                           horizonCounter + 1,
                           n_Horizon, loop)) for horizonCounter in range(n_Horizon)]

    # []<> (l4_1 && <> (l5_4 && <> l6_3))
    always_eventually = [
        Or([And(loop[i],
                Or([formula[(formulaCounter + 5) * n_Horizon + horizonCounter]
                    for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
    s.add(formula_3 + formula_4 + formula_5 + formula_6 + always_eventually)


def case_3(n_Robot, n_Region, n_Horizon, region_dict, s, Robot_Region_Horizon, loop):
    # C. Multi-robot Planning
    # n_formula = 9 + 1 # + 2 + 2  # + 2 + 1 + 2
    # n_formula = 12 + 2 + 2 + 1 + 2
    case = 3  # int(sys.argv[1])
    if case == 3:
        n_formula = 9
    elif case == 4:
        n_formula = 10
    elif case == 5:
        n_formula = 12
    elif case == 6:
        n_formula = 14
    elif case == 7:
        n_formula = 16

    formula = BoolVector('f', n_formula * n_Horizon)

    formulaCounter = 0

    if case >= 3:
        # []<> (l1_1 && (l1_2 || l1_3) && (l1_4 || l1_5) && l1_6)
        region = region_dict["l1"]
        formula_1 = [formula[formulaCounter * n_Horizon + horizonCounter] == And(
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 0 * n_Region + region],
            Or(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 1 * n_Region + region],
               Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 2 * n_Region + region]),
            Or(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 3 * n_Region + region],
               Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 4 * n_Region + region]),
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 5 * n_Region + region]) for horizonCounter in
                     range(n_Horizon)]
        always_eventually_1 = [
            Or([And(loop[i],
                    Or([formula[formulaCounter * n_Horizon + horizonCounter]
                        for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
        s.add(formula_1 + always_eventually_1)

        # []<> (l2_6 && (l2_7 || l2_8) && (l2_9 || l2_10) && l2_11)
        formulaCounter = formulaCounter + 1
        region = region_dict["l2"]
        formula_2 = [formula[formulaCounter * n_Horizon + horizonCounter] == And(
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 5 * n_Region + region],
            Or(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 6 * n_Region + region],
               Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 7 * n_Region + region]),
            Or(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 8 * n_Region + region],
               Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 9 * n_Region + region]),
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 10 * n_Region + region]) for horizonCounter in
                     range(n_Horizon)]
        always_eventually_2 = [
            Or([And(loop[i],
                    Or([formula[formulaCounter * n_Horizon + horizonCounter]
                        for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
        s.add(formula_2 + always_eventually_2)

        # []<> (l3_11 && (l3_12 || l3_13) && (l3_14 || l3_15) && l3_16)
        formulaCounter = formulaCounter + 1
        region = region_dict["l3"]
        formula_3 = [formula[formulaCounter * n_Horizon + horizonCounter] == And(
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 10 * n_Region + region],
            Or(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 11 * n_Region + region],
               Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 12 * n_Region + region]),
            Or(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 13 * n_Region + region],
               Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 14 * n_Region + region]),
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 15 * n_Region + region]) for horizonCounter in
                     range(n_Horizon)]
        always_eventually_3 = [
            Or([And(loop[i],
                    Or([formula[formulaCounter * n_Horizon + horizonCounter]
                        for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
        s.add(formula_3 + always_eventually_3)

        # formula = 3 up to now

        # l6_11
        robot1 = 10
        region = region_dict["l6"]
        formulaCounter = formulaCounter + 1
        formula_3 = [formula[formulaCounter * n_Horizon + horizonCounter] for horizonCounter in range(n_Horizon)] + [
            formula[(formulaCounter + 1) * n_Horizon + horizonCounter] == Robot_Region_Horizon[n_Robot * n_Region *
                                                                                               horizonCounter + robot1 * n_Region + region]
            for horizonCounter in range(n_Horizon)]
        # l5_6 && <> l6_11
        robot2 = 5
        region = region_dict["l5"]
        formula_4 = [formula[(formulaCounter + 2) * n_Horizon + horizonCounter] ==
                     And(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + robot2 * n_Region + region],
                         until(formula[formulaCounter * n_Horizon: (formulaCounter + 2) * n_Horizon],
                               horizonCounter + 1,
                               n_Horizon, loop)) for horizonCounter in range(n_Horizon)]
        # <> (l5_6 && <> l6_11)
        formula_5 = [formula[(formulaCounter + 3) * n_Horizon + horizonCounter] for horizonCounter in
                     range(n_Horizon)] + [
                        formula[(formulaCounter + 4) * n_Horizon + horizonCounter] == formula[
                            (formulaCounter + 2) * n_Horizon + horizonCounter]
                        for horizonCounter in range(n_Horizon)]
        # l4_1 && <> (l5_6 && <> l6_11)
        robot3 = 0
        region = region_dict["l4"]
        formula_6 = [formula[(formulaCounter + 5) * n_Horizon + horizonCounter] ==
                     And(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + robot3 * n_Region + region],
                         until(formula[(formulaCounter + 3) * n_Horizon: (formulaCounter + 5) * n_Horizon],
                               horizonCounter + 1,
                               n_Horizon, loop)) for horizonCounter in range(n_Horizon)]

        # []<> (l4_1 && <> (l5_6 && <> l6_11))
        always_eventually = [
            Or([And(loop[i],
                    Or([formula[(formulaCounter + 5) * n_Horizon + horizonCounter]
                        for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
        s.add(formula_3 + formula_4 + formula_5 + formula_6 + always_eventually)

        # formula = 9 up to now

    if case >= 4:
        # [] !(l3_4 || l3_9)
        region = region_dict["l3"]
        formulaCounter = formulaCounter + 6
        formula_7 = [
            formula[formulaCounter * n_Horizon + horizonCounter] == Not(Or(Robot_Region_Horizon[n_Robot * n_Region *
                                                                                                horizonCounter + 3 * n_Region + region],
                                                                           Robot_Region_Horizon[n_Robot * n_Region *
                                                                                                horizonCounter + 8 * n_Region + region]))
            for horizonCounter in range(n_Horizon)]
        always_not = [always(formula[formulaCounter * n_Horizon: (formulaCounter + 1) * n_Horizon], 1, n_Horizon)]
        s.add(formula_7 + always_not)

        # formula = 10 up to now

    if case >= 5:
        # <> (l4_3 && l5_8 && l6_13)
        formulaCounter = formulaCounter + 1
        formula_8 = [formula[formulaCounter * n_Horizon + horizonCounter] for horizonCounter in range(n_Horizon)] + [
            formula[(formulaCounter + 1) * n_Horizon + horizonCounter] == And(
                Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 2 * n_Region + region_dict["l4"]],
                Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 7 * n_Region + region_dict["l5"]],
                Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 12 * n_Region + region_dict["l6"]])
            for horizonCounter in range(n_Horizon)]
        eventually = [until(formula[formulaCounter * n_Horizon: (formulaCounter + 2) * n_Horizon], 1, n_Horizon, loop)]

        s.add(formula_8 + eventually)

        # formula = 12 up to now

    if case >= 6:
        # []<> (l2_17 && l2_18 && l4_19 & l4_20)
        formulaCounter = formulaCounter + 2
        formula_9 = [formula[formulaCounter * n_Horizon + horizonCounter] == And(
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 16 * n_Region + region_dict["l2"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 17 * n_Region + region_dict["l2"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 18 * n_Region + region_dict["l4"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 19 * n_Region + region_dict["l4"]]) for
                     horizonCounter in
                     range(n_Horizon)]
        always_eventually_4 = [
            Or([And(loop[i],
                    Or([formula[formulaCounter * n_Horizon + horizonCounter]
                        for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
        s.add(formula_9 + always_eventually_4)

        # formula = 14 up to now

    if case >= 7:
        # !(l4_3 && l5_8 && l6_13) U (l2_17 && l2_18 && l4_19 & l4_20)
        formulaCounter = formulaCounter + 1
        formula_10 = [formula[formulaCounter * n_Horizon + horizonCounter] == Not(
            formula[(formulaCounter - 2) * n_Horizon + horizonCounter]) for horizonCounter in range(n_Horizon)] + [
                         formula[(formulaCounter + 1) * n_Horizon + horizonCounter] == formula[
                             (formulaCounter - 1) * n_Horizon + horizonCounter]
                         for horizonCounter in range(n_Horizon)]
        until_1 = [until(formula[formulaCounter * n_Horizon: (formulaCounter + 2) * n_Horizon], 1, n_Horizon, loop)]

        s.add(formula_10 + until_1)


def case_4(n_Robot, n_Region, n_Horizon, region_dict, s, Robot_Region_Horizon, loop):
    # C. Multi-robot Planning
    # n_formula = 9 + 1 # + 2 + 2  # + 2 + 1 + 2
    # n_formula = 12 + 2 + 2 + 1 + 2
    case = 7  # int(sys.argv[1])
    n_formula = 16
    # if case == 3:
    #     n_formula = 9
    # elif case == 4:
    #     n_formula = 10
    # elif case == 5:
    #     n_formula = 12
    # elif case == 6:
    #     n_formula = 14
    # elif case == 7:
    #     n_formula = 16

    formula = BoolVector('f', n_formula * n_Horizon)

    formulaCounter = 0

    if case >= 3:
        # []<> (l1_1 && (l1_2 || l1_3) && (l1_4 || l1_5) && l1_6 && l4_21 && l5_24 && l3_30)
        region = region_dict["l1"]
        formula_1 = [formula[formulaCounter * n_Horizon + horizonCounter] == And(
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 0 * n_Region + region],
            Or(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 1 * n_Region + region],
               Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 2 * n_Region + region]),
            Or(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 3 * n_Region + region],
               Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 4 * n_Region + region]),
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 5 * n_Region + region],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 20 * n_Region + region_dict["l4"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 23 * n_Region + region_dict["l5"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 29 * n_Region + region_dict["l3"]])
                     for horizonCounter in range(n_Horizon)]
        always_eventually_1 = [
            Or([And(loop[i],
                    Or([formula[formulaCounter * n_Horizon + horizonCounter]
                        for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
        s.add(formula_1 + always_eventually_1)

        # []<> (l2_6 && (l2_7 || l2_8) && (l2_9 || l2_10) && l2_11 && l5_22 && l6_25 && l4_31)
        formulaCounter = formulaCounter + 1
        region = region_dict["l2"]
        formula_2 = [formula[formulaCounter * n_Horizon + horizonCounter] == And(
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 5 * n_Region + region],
            Or(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 6 * n_Region + region],
               Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 7 * n_Region + region]),
            Or(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 8 * n_Region + region],
               Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 9 * n_Region + region]),
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 10 * n_Region + region],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 21 * n_Region + region_dict["l5"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 24 * n_Region + region_dict["l6"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 30 * n_Region + region_dict["l4"]])
                     for horizonCounter in range(n_Horizon)]
        always_eventually_2 = [
            Or([And(loop[i],
                    Or([formula[formulaCounter * n_Horizon + horizonCounter]
                        for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
        s.add(formula_2 + always_eventually_2)

        # []<> (l3_11 && (l3_12 || l3_13) && (l3_14 || l3_15) && l3_16 && l6_23 && l4_26 && l5_32)
        formulaCounter = formulaCounter + 1
        region = region_dict["l3"]
        formula_3 = [formula[formulaCounter * n_Horizon + horizonCounter] == And(
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 10 * n_Region + region],
            Or(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 11 * n_Region + region],
               Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 12 * n_Region + region]),
            Or(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 13 * n_Region + region],
               Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 14 * n_Region + region]),
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 15 * n_Region + region],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 22 * n_Region + region_dict["l6"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 25 * n_Region + region_dict["l4"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 31 * n_Region + region_dict["l5"]])
                     for horizonCounter in range(n_Horizon)]
        always_eventually_3 = [
            Or([And(loop[i],
                    Or([formula[formulaCounter * n_Horizon + horizonCounter]
                        for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
        s.add(formula_3 + always_eventually_3)

        # formula = 3 up to now

        # l6_11 && l6_29 && l6_32
        robot1 = 10
        region = region_dict["l6"]
        formulaCounter = formulaCounter + 1
        formula_3 = [formula[formulaCounter * n_Horizon + horizonCounter] for horizonCounter in range(n_Horizon)] + [
            formula[(formulaCounter + 1) * n_Horizon + horizonCounter] == And(
                Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + robot1 * n_Region + region],
                Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 28 * n_Region + region],
                Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 31 * n_Region + region])
            for horizonCounter in range(n_Horizon)]
        # l5_6 && l5_28 && l5_31 && <> (l6_11 && l6_29 && l6_32)
        robot2 = 5
        region = region_dict["l5"]
        formula_4 = [formula[(formulaCounter + 2) * n_Horizon + horizonCounter] == And(
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + robot2 * n_Region + region],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 27 * n_Region + region],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 30 * n_Region + region],
            until(formula[formulaCounter * n_Horizon: (formulaCounter + 2) * n_Horizon],
                  horizonCounter + 1, n_Horizon, loop)) for horizonCounter in range(n_Horizon)]
        # <> (l5_6 && l5_28 && l5_31 && <> (l6_11 && l6_29 && l6_32))
        formula_5 = [formula[(formulaCounter + 3) * n_Horizon + horizonCounter] for horizonCounter in
                     range(n_Horizon)] + [
                        formula[(formulaCounter + 4) * n_Horizon + horizonCounter] == formula[
                            (formulaCounter + 2) * n_Horizon + horizonCounter]
                        for horizonCounter in range(n_Horizon)]
        # l4_1 && l4_27 && l4_30 && <> (l5_6 && l5_28 && l5_31 && <> (l6_11 && l6_29 && l6_32))
        robot3 = 0
        region = region_dict["l4"]
        formula_6 = [formula[(formulaCounter + 5) * n_Horizon + horizonCounter] == And(
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + robot3 * n_Region + region],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 26 * n_Region + region],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 29 * n_Region + region],
            until(formula[(formulaCounter + 3) * n_Horizon: (formulaCounter + 5) * n_Horizon],
                  horizonCounter + 1,
                  n_Horizon, loop)) for horizonCounter in range(n_Horizon)]

        # []<> (l4_1 && l4_27 && l4_30 && <> (l5_6 && l5_28 && l5_31 && <> (l6_11 && l6_29 && l6_32)))
        always_eventually = [
            Or([And(loop[i],
                    Or([formula[(formulaCounter + 5) * n_Horizon + horizonCounter]
                        for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
        s.add(formula_3 + formula_4 + formula_5 + formula_6 + always_eventually)

        # formula = 9 up to now

    if case >= 4:
        # [] !(l3_4 || l3_9)
        region = region_dict["l3"]
        formulaCounter = formulaCounter + 6
        formula_7 = [
            formula[formulaCounter * n_Horizon + horizonCounter] == Not(Or(Robot_Region_Horizon[n_Robot * n_Region *
                                                                                                horizonCounter + 3 * n_Region + region],
                                                                           Robot_Region_Horizon[n_Robot * n_Region *
                                                                                                horizonCounter + 8 * n_Region + region]))
            for horizonCounter in range(n_Horizon)]
        always_not = [always(formula[formulaCounter * n_Horizon: (formulaCounter + 1) * n_Horizon], 1, n_Horizon)]
        s.add(formula_7 + always_not)

        # formula = 10 up to now

    if case >= 5:
        # <> (l4_3 && l5_8 && l6_13 && l1_21 && l2_22 && l3_23)
        formulaCounter = formulaCounter + 1
        formula_8 = [formula[formulaCounter * n_Horizon + horizonCounter] for horizonCounter in range(n_Horizon)] + [
            formula[(formulaCounter + 1) * n_Horizon + horizonCounter] == And(
                Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 2 * n_Region + region_dict["l4"]],
                Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 7 * n_Region + region_dict["l5"]],
                Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 12 * n_Region + region_dict["l6"]],
                Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 20 * n_Region + region_dict["l1"]],
                Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 21 * n_Region + region_dict["l2"]],
                Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 22 * n_Region + region_dict["l3"]])
            for horizonCounter in range(n_Horizon)]
        eventually = [until(formula[formulaCounter * n_Horizon: (formulaCounter + 2) * n_Horizon], 1, n_Horizon, loop)]

        s.add(formula_8 + eventually)

        # formula = 12 up to now

    if case >= 6:
        # []<> (l2_17 && l2_18 && l4_19 && l4_20 && l1_24 && l2_25 && l3_26)
        formulaCounter = formulaCounter + 2
        formula_9 = [formula[formulaCounter * n_Horizon + horizonCounter] == And(
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 16 * n_Region + region_dict["l2"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 17 * n_Region + region_dict["l2"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 18 * n_Region + region_dict["l4"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 19 * n_Region + region_dict["l4"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 23 * n_Region + region_dict["l1"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 24 * n_Region + region_dict["l2"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 25 * n_Region + region_dict["l3"]]) for
                     horizonCounter in range(n_Horizon)]
        always_eventually_4 = [
            Or([And(loop[i],
                    Or([formula[formulaCounter * n_Horizon + horizonCounter]
                        for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
        s.add(formula_9 + always_eventually_4)

        # formula = 14 up to now

    if case >= 7:
        # !(l4_3 && l5_8 && l6_13 && l1_21 && l2_22 && l3_23) U
        #  (l2_17 && l2_18 && l4_19 && l4_20 && l1_24 && l2_25 && l3_26)
        formulaCounter = formulaCounter + 1
        formula_10 = [formula[formulaCounter * n_Horizon + horizonCounter] == Not(
            formula[(formulaCounter - 2) * n_Horizon + horizonCounter]) for horizonCounter in range(n_Horizon)] + [
                         formula[(formulaCounter + 1) * n_Horizon + horizonCounter] == formula[
                             (formulaCounter - 1) * n_Horizon + horizonCounter]
                         for horizonCounter in range(n_Horizon)]
        until_1 = [until(formula[formulaCounter * n_Horizon: (formulaCounter + 2) * n_Horizon], 1, n_Horizon, loop)]

        s.add(formula_10 + until_1)


def case_5(n_Robot, n_Region, n_Horizon, region_dict, s, Robot_Region_Horizon, loop):
    # C. Multi-robot Planning
    # n_formula = 9 + 1 # + 2 + 2  # + 2 + 1 + 2
    # n_formula = 12 + 2 + 2 + 1 + 2
    case = 7  # int(sys.argv[1])
    n_formula = 16
    # if case == 3:
    #     n_formula = 9
    # elif case == 4:
    #     n_formula = 10
    # elif case == 5:
    #     n_formula = 12
    # elif case == 6:
    #     n_formula = 14
    # elif case == 7:
    #     n_formula = 16

    formula = BoolVector('f', n_formula * n_Horizon)

    formulaCounter = 0

    if case >= 3:
        # []<> (l1_1 && (l1_2 || l1_3) && (l1_4 || l1_5) && l1_6 && l4_21 && l5_24 && l3_30 && l2_33 && l6_36),
        region = region_dict["l1"]
        formula_1 = [formula[formulaCounter * n_Horizon + horizonCounter] == And(
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 0 * n_Region + region],
            Or(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 1 * n_Region + region],
               Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 2 * n_Region + region]),
            Or(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 3 * n_Region + region],
               Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 4 * n_Region + region]),
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 5 * n_Region + region],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 20 * n_Region + region_dict["l4"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 23 * n_Region + region_dict["l5"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 29 * n_Region + region_dict["l3"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 32 * n_Region + region_dict["l2"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 35 * n_Region + region_dict["l6"]])
                     for horizonCounter in range(n_Horizon)]
        always_eventually_1 = [
            Or([And(loop[i],
                    Or([formula[formulaCounter * n_Horizon + horizonCounter]
                        for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
        s.add(formula_1 + always_eventually_1)

        # []<> (l2_6 && (l2_7 || l2_8) && (l2_9 || l2_10) && l2_11 && l5_22 && l6_25 && l4_31 && l3_34 && l1_37),

        formulaCounter = formulaCounter + 1
        region = region_dict["l2"]
        formula_2 = [formula[formulaCounter * n_Horizon + horizonCounter] == And(
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 5 * n_Region + region],
            Or(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 6 * n_Region + region],
               Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 7 * n_Region + region]),
            Or(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 8 * n_Region + region],
               Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 9 * n_Region + region]),
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 10 * n_Region + region],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 21 * n_Region + region_dict["l5"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 24 * n_Region + region_dict["l6"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 30 * n_Region + region_dict["l4"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 30 * n_Region + region_dict["l4"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 33 * n_Region + region_dict["l3"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 36 * n_Region + region_dict["l1"]])
                     for horizonCounter in range(n_Horizon)]
        always_eventually_2 = [
            Or([And(loop[i],
                    Or([formula[formulaCounter * n_Horizon + horizonCounter]
                        for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
        s.add(formula_2 + always_eventually_2)

        # []<> (l3_11 && (l3_12 || l3_13) && (l3_14 || l3_15) && l3_16 && l6_23 && l4_26 && l5_32 && l2_35 && l1_38),
        formulaCounter = formulaCounter + 1
        region = region_dict["l3"]
        formula_3 = [formula[formulaCounter * n_Horizon + horizonCounter] == And(
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 10 * n_Region + region],
            Or(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 11 * n_Region + region],
               Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 12 * n_Region + region]),
            Or(Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 13 * n_Region + region],
               Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 14 * n_Region + region]),
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 15 * n_Region + region],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 22 * n_Region + region_dict["l6"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 25 * n_Region + region_dict["l4"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 31 * n_Region + region_dict["l5"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 34 * n_Region + region_dict["l2"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 37 * n_Region + region_dict["l1"]])
                     for horizonCounter in range(n_Horizon)]
        always_eventually_3 = [
            Or([And(loop[i],
                    Or([formula[formulaCounter * n_Horizon + horizonCounter]
                        for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
        s.add(formula_3 + always_eventually_3)

        # formula = 3 up to now

        # (l6_11 & & l6_29 & & l6_32 & & l6_35 & & l6_38)
        robot1 = 10
        region = region_dict["l6"]
        formulaCounter = formulaCounter + 1
        formula_3 = [formula[formulaCounter * n_Horizon + horizonCounter] for horizonCounter in range(n_Horizon)] + [
            formula[(formulaCounter + 1) * n_Horizon + horizonCounter] == And(
                Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + robot1 * n_Region + region],
                Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 28 * n_Region + region],
                Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 31 * n_Region + region],
                Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 34 * n_Region + region],
                Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 37 * n_Region + region])
            for horizonCounter in range(n_Horizon)]
        # l5_6 && l5_28 && l5_31 && l5_34 && l5_37 && <> (l6_11 & & l6_29 & & l6_32 & & l6_35 & & l6_38)
        robot2 = 5
        region = region_dict["l5"]
        formula_4 = [formula[(formulaCounter + 2) * n_Horizon + horizonCounter] == And(
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + robot2 * n_Region + region],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 27 * n_Region + region],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 30 * n_Region + region],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 33 * n_Region + region],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 36 * n_Region + region],
            until(formula[formulaCounter * n_Horizon: (formulaCounter + 2) * n_Horizon],
                  horizonCounter + 1, n_Horizon, loop)) for horizonCounter in range(n_Horizon)]
        # <> (l5_6 && l5_28 && l5_31 && l5_34 && l5_37 && <> (l6_11 & & l6_29 & & l6_32 & & l6_35 & & l6_38))
        formula_5 = [formula[(formulaCounter + 3) * n_Horizon + horizonCounter] for horizonCounter in
                     range(n_Horizon)] + [
                        formula[(formulaCounter + 4) * n_Horizon + horizonCounter] == formula[
                            (formulaCounter + 2) * n_Horizon + horizonCounter]
                        for horizonCounter in range(n_Horizon)]
        # (l4_1 && l4_27 && l4_30 && l4_33 && l4_36) && <> (l5_6 && l5_28 && l5_31 && l5_34 && l5_37
        # && <> (l6_11 & & l6_29 & & l6_32 & & l6_35 & & l6_38))
        robot3 = 0
        region = region_dict["l4"]
        formula_6 = [formula[(formulaCounter + 5) * n_Horizon + horizonCounter] == And(
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + robot3 * n_Region + region],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 26 * n_Region + region],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 29 * n_Region + region],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 32 * n_Region + region],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 35 * n_Region + region],
            until(formula[(formulaCounter + 3) * n_Horizon: (formulaCounter + 5) * n_Horizon],
                  horizonCounter + 1,
                  n_Horizon, loop)) for horizonCounter in range(n_Horizon)]

        # []<> (l4_1 && l4_27 && l4_30 && <> (l5_6 && l5_28 && l5_31 && <> (l6_11 && l6_29 && l6_32)))
        always_eventually = [
            Or([And(loop[i],
                    Or([formula[(formulaCounter + 5) * n_Horizon + horizonCounter]
                        for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
        s.add(formula_3 + formula_4 + formula_5 + formula_6 + always_eventually)

        # formula = 9 up to now

    if case >= 4:
        # [] !(l3_4 || l3_9)
        region = region_dict["l3"]
        formulaCounter = formulaCounter + 6
        formula_7 = [
            formula[formulaCounter * n_Horizon + horizonCounter] == Not(Or(Robot_Region_Horizon[n_Robot * n_Region *
                                                                                                horizonCounter + 3 * n_Region + region],
                                                                           Robot_Region_Horizon[n_Robot * n_Region *
                                                                                                horizonCounter + 8 * n_Region + region]))
            for horizonCounter in range(n_Horizon)]
        always_not = [always(formula[formulaCounter * n_Horizon: (formulaCounter + 1) * n_Horizon], 1, n_Horizon)]
        s.add(formula_7 + always_not)

        # formula = 10 up to now

    if case >= 5:
        # <> (l4_3 && l5_8 && l6_13 && l1_21 && l2_22 && l3_23 && l3_39)
        formulaCounter = formulaCounter + 1
        formula_8 = [formula[formulaCounter * n_Horizon + horizonCounter] for horizonCounter in range(n_Horizon)] + [
            formula[(formulaCounter + 1) * n_Horizon + horizonCounter] == And(
                Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 2 * n_Region + region_dict["l4"]],
                Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 7 * n_Region + region_dict["l5"]],
                Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 12 * n_Region + region_dict["l6"]],
                Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 20 * n_Region + region_dict["l1"]],
                Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 21 * n_Region + region_dict["l2"]],
                Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 22 * n_Region + region_dict["l3"]],
                Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 38 * n_Region + region_dict["l3"]])
            for horizonCounter in range(n_Horizon)]
        eventually = [until(formula[formulaCounter * n_Horizon: (formulaCounter + 2) * n_Horizon], 1, n_Horizon, loop)]

        s.add(formula_8 + eventually)

        # formula = 12 up to now

    if case >= 6:
        # []<> (l2_17 && l2_18 && l4_19 && l4_20 && l1_24 && l2_25 && l3_26 && l1_40)
        formulaCounter = formulaCounter + 2
        formula_9 = [formula[formulaCounter * n_Horizon + horizonCounter] == And(
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 16 * n_Region + region_dict["l2"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 17 * n_Region + region_dict["l2"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 18 * n_Region + region_dict["l4"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 19 * n_Region + region_dict["l4"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 23 * n_Region + region_dict["l1"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 24 * n_Region + region_dict["l2"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 25 * n_Region + region_dict["l3"]],
            Robot_Region_Horizon[n_Robot * n_Region * horizonCounter + 39 * n_Region + region_dict["l1"]]) for
                     horizonCounter in range(n_Horizon)]
        always_eventually_4 = [
            Or([And(loop[i],
                    Or([formula[formulaCounter * n_Horizon + horizonCounter]
                        for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
        s.add(formula_9 + always_eventually_4)

        # formula = 14 up to now

    if case >= 7:
        # !(l4_3 && l5_8 && l6_13 && l1_21 && l2_22 && l3_23) U
        #  (l2_17 && l2_18 && l4_19 && l4_20 && l1_24 && l2_25 && l3_26)
        formulaCounter = formulaCounter + 1
        formula_10 = [formula[formulaCounter * n_Horizon + horizonCounter] == Not(
            formula[(formulaCounter - 2) * n_Horizon + horizonCounter]) for horizonCounter in range(n_Horizon)] + [
                         formula[(formulaCounter + 1) * n_Horizon + horizonCounter] == formula[
                             (formulaCounter - 1) * n_Horizon + horizonCounter]
                         for horizonCounter in range(n_Horizon)]
        until_1 = [until(formula[formulaCounter * n_Horizon: (formulaCounter + 2) * n_Horizon], 1, n_Horizon, loop)]

        s.add(formula_10 + until_1)


def case_auto(n_Robot, n_Region, n_Horizon, region_dict, s, Robot_Region_Horizon, loop):
    formula_comp = \
        {1: '(l5_8 && l2_1 && l3_3)', 2: '(l6_12 && l1_10 && l5_7)', 3: '(l4_11 && l5_9 && l5_10)',
         4: '(l1_15 && l1_5 && l2_8)', 5: '(l1_7 && l4_16 && l3_8)', 6: '(l1_13 && l4_4 && l5_3)',
         7: '(l5_2 && l2_3 && l3_4)', 8: '(l2_14 && l4_6 && l3_3)'}

    formu = '[]<> e1 && []<> e2 && []<> e3 && []<>(e4 && <>(e5 && <> e6)) && <> e7 && []<>e8 && (!e7 U e8)'
    # print(formula_comp)
    # print(formu)
    case = 7  # int(sys.argv[1])
    n_formula = 15

    formula = BoolVector('f', n_formula * n_Horizon)

    formulaCounter = 0

    if case >= 3:
        f = formula_comp[1].strip('(').strip(')').split(' && ')
        formula_1 = [formula[formulaCounter * n_Horizon + horizonCounter] == And([
            Robot_Region_Horizon[
                n_Robot * n_Region * horizonCounter + (int(f[k].split('_')[1]) - 1) * n_Region + region_dict[
                    f[k].split('_')[0]]]
            for k in range(len(f))]) for horizonCounter in range(n_Horizon)]
        always_eventually_1 = [
            Or([And(loop[i],
                    Or([formula[formulaCounter * n_Horizon + horizonCounter]
                        for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
        s.add(formula_1 + always_eventually_1)

        # []<> (l2_6 && (l2_7 || l2_8) && (l2_9 || l2_10) && l2_11 && l5_22 && l6_25 && l4_31 && l3_34 && l1_37),
        f = formula_comp[2].strip('(').strip(')').split(' && ')
        formulaCounter = formulaCounter + 1
        formula_2 = [formula[formulaCounter * n_Horizon + horizonCounter] == And([
            Robot_Region_Horizon[
                n_Robot * n_Region * horizonCounter + (int(f[k].split('_')[1]) - 1) * n_Region + region_dict[
                    f[k].split('_')[0]]]
            for k in range(len(f))]) for horizonCounter in range(n_Horizon)]
        always_eventually_2 = [
            Or([And(loop[i],
                    Or([formula[formulaCounter * n_Horizon + horizonCounter]
                        for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
        s.add(formula_2 + always_eventually_2)

        # []<> (l3_11 && (l3_12 || l3_13) && (l3_14 || l3_15) && l3_16 && l6_23 && l4_26 && l5_32 && l2_35 && l1_38),
        f = formula_comp[3].strip('(').strip(')').split(' && ')
        formulaCounter = formulaCounter + 1
        formula_3 = [formula[formulaCounter * n_Horizon + horizonCounter] == And([
            Robot_Region_Horizon[
                n_Robot * n_Region * horizonCounter + (int(f[k].split('_')[1]) - 1) * n_Region + region_dict[
                    f[k].split('_')[0]]]
            for k in range(len(f))]) for horizonCounter in range(n_Horizon)]
        always_eventually_3 = [
            Or([And(loop[i],
                    Or([formula[formulaCounter * n_Horizon + horizonCounter]
                        for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
        s.add(formula_3 + always_eventually_3)

        # formula = 3 up to now

        # (l6_11 & & l6_29 & & l6_32 & & l6_35 & & l6_38)
        f = formula_comp[6].strip('(').strip(')').split(' && ')

        formulaCounter = formulaCounter + 1
        formula_3 = [formula[formulaCounter * n_Horizon + horizonCounter] for horizonCounter in range(n_Horizon)] + [
            formula[(formulaCounter + 1) * n_Horizon + horizonCounter] == And([
                Robot_Region_Horizon[
                    n_Robot * n_Region * horizonCounter + (int(f[k].split('_')[1]) - 1) * n_Region + region_dict[
                        f[k].split('_')[0]]]
                for k in range(len(f))]) for horizonCounter in range(n_Horizon)]
        # l5_6 && l5_28 && l5_31 && l5_34 && l5_37 && <> (l6_11 & & l6_29 & & l6_32 & & l6_35 & & l6_38)
        f = formula_comp[5].strip('(').strip(')').split(' && ')

        formula_4 = [formula[(formulaCounter + 2) * n_Horizon + horizonCounter] == And(And([
            Robot_Region_Horizon[
                n_Robot * n_Region * horizonCounter + (int(f[k].split('_')[1]) - 1) * n_Region + region_dict[
                    f[k].split('_')[0]]]
            for k in range(len(f))]),
            until(formula[formulaCounter * n_Horizon: (formulaCounter + 2) * n_Horizon],
                  horizonCounter + 1, n_Horizon, loop)) for horizonCounter in range(n_Horizon)]

        # <> (l5_6 && l5_28 && l5_31 && l5_34 && l5_37 && <> (l6_11 & & l6_29 & & l6_32 & & l6_35 & & l6_38))
        formula_5 = [formula[(formulaCounter + 3) * n_Horizon + horizonCounter] for horizonCounter in
                     range(n_Horizon)] + [
                        formula[(formulaCounter + 4) * n_Horizon + horizonCounter] == formula[
                            (formulaCounter + 2) * n_Horizon + horizonCounter]
                        for horizonCounter in range(n_Horizon)]
        # (l4_1 && l4_27 && l4_30 && l4_33 && l4_36) && <> (l5_6 && l5_28 && l5_31 && l5_34 && l5_37
        # && <> (l6_11 & & l6_29 & & l6_32 & & l6_35 & & l6_38))
        f = formula_comp[4].strip('(').strip(')').split(' && ')

        formula_6 = [formula[(formulaCounter + 5) * n_Horizon + horizonCounter] == And(And([
            Robot_Region_Horizon[
                n_Robot * n_Region * horizonCounter + (int(f[k].split('_')[1]) - 1) * n_Region + region_dict[
                    f[k].split('_')[0]]]
            for k in range(len(f))]),
            until(formula[(formulaCounter + 3) * n_Horizon: (formulaCounter + 5) * n_Horizon],
                  horizonCounter + 1,
                  n_Horizon, loop)) for horizonCounter in range(n_Horizon)]

        # []<> (l4_1 && l4_27 && l4_30 && <> (l5_6 && l5_28 && l5_31 && <> (l6_11 && l6_29 && l6_32)))
        always_eventually = [
            Or([And(loop[i],
                    Or([formula[(formulaCounter + 5) * n_Horizon + horizonCounter]
                        for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
        s.add(formula_3 + formula_4 + formula_5 + formula_6 + always_eventually)

        # formula = 9 up to now

        if case >= 5:
            # <> (l4_3 && l5_8 && l6_13 && l1_21 && l2_22 && l3_23 && l3_39)
            formulaCounter = formulaCounter + 6
            f = formula_comp[7].strip('(').strip(')').split(' && ')

            formula_8 = [formula[formulaCounter * n_Horizon + horizonCounter] for horizonCounter in
                         range(n_Horizon)] + [
                            formula[(formulaCounter + 1) * n_Horizon + horizonCounter] == And([
                                Robot_Region_Horizon[
                                    n_Robot * n_Region * horizonCounter + (int(f[k].split('_')[1]) - 1) * n_Region +
                                    region_dict[
                                        f[k].split('_')[0]]]
                                for k in range(len(f))])
                            for horizonCounter in range(n_Horizon)]
            eventually = [
                until(formula[formulaCounter * n_Horizon: (formulaCounter + 2) * n_Horizon], 1, n_Horizon, loop)]

            s.add(formula_8 + eventually)

        if case >= 6:
            # []<> (l2_17 && l2_18 && l4_19 && l4_20 && l1_24 && l2_25 && l3_26 && l1_40)
            formulaCounter = formulaCounter + 2
            f = formula_comp[8].strip('(').strip(')').split(' && ')

            formula_9 = [formula[formulaCounter * n_Horizon + horizonCounter] == And([
                Robot_Region_Horizon[
                    n_Robot * n_Region * horizonCounter + (int(f[k].split('_')[1]) - 1) * n_Region + region_dict[
                        f[k].split('_')[0]]]
                for k in range(len(f))]) for
                         horizonCounter in range(n_Horizon)]
            always_eventually_4 = [
                Or([And(loop[i],
                        Or([formula[formulaCounter * n_Horizon + horizonCounter]
                            for horizonCounter in range(i, n_Horizon)])) for i in range(1, n_Horizon)])]
            s.add(formula_9 + always_eventually_4)

        if case >= 7:
            # !(l4_3 && l5_8 && l6_13 && l1_21 && l2_22 && l3_23) U
            #  (l2_17 && l2_18 && l4_19 && l4_20 && l1_24 && l2_25 && l3_26)
            formulaCounter = formulaCounter + 1
            formula_10 = [formula[formulaCounter * n_Horizon + horizonCounter] == Not(
                formula[(formulaCounter - 2) * n_Horizon + horizonCounter]) for horizonCounter in range(n_Horizon)] + [
                             formula[(formulaCounter + 1) * n_Horizon + horizonCounter] == formula[
                                 (formulaCounter - 1) * n_Horizon + horizonCounter]
                             for horizonCounter in range(n_Horizon)]
            until_1 = [until(formula[formulaCounter * n_Horizon: (formulaCounter + 2) * n_Horizon], 1, n_Horizon, loop)]

            s.add(formula_10 + until_1)
