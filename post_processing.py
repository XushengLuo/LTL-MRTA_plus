def run(graph, path, regions, initial, accept):
    frontier = [[initial, 0, {initial: 0}, dict()]]
    while True:
        node, step, acpt_run, robot = frontier.pop()
        is_enabled = False
        # loop over each successor to see whether progress can be made
        for succ in graph.succ[node]:
            if satisfy(path, step, graph.edges[(node, succ)]['label'], robot, regions):
                # step, the exact time when transition occurs
                acpt_run.update({succ: step})
                # step + 1, the immediate time step that should be verified
                frontier.append([succ, step+1, acpt_run, robot])
                is_enabled = True
                if succ == accept:
                    print(acpt_run)
                    return
        # stay in the same node
        if not is_enabled:
            if satisfy(path, step, graph.nodes[node]['label'], robot, regions):
                frontier.append([node, step+1, acpt_run, robot])


def satisfy(path, step, label, robot, regions):
    if label == '1':
        return True
    for clause in label:
        sat = True
        same_robot = dict()
        for lit in clause:
            # for each literal, whether enough involved robots reach
            r = []
            for type_robot in path.keys():
                if type_robot[0] == lit[1]:
                    try:
                        if path[type_robot][step] == regions[lit[0]]:
                            r.append(type_robot[1])
                    except IndexError:
                        if path[type_robot][-1] == regions[lit[0]]:
                            r.append(type_robot[1])

            if len(r) >= lit[2]:
                # whether a nonzero indicator is encountered
                if lit[3] > 0:
                    # the first time that this nonzero indicator shows up
                    if lit[3] not in robot.keys():
                        same_robot[lit[3]] = r
                    # if not the first time, whether the same set of robots is used
                    else:
                        if set(r) == set(robot[lit[3]]):
                            continue
                        else:
                            sat = False
                            break
                # zero indicator
                else:
                    continue
            # not enough robots
            else:
                sat = False
                break
        # a clause is satisfied
        if sat:
            robot.update(same_robot)
            return True
    # no clause is satisfied
    return False
