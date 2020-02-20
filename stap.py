from task import Task
from buchi_parse import Buchi
from datetime import datetime
import poset
from workspace import Workspace
import matplotlib.pyplot as plt
import weighted_ts
import milp
import milp_suffix
import pickle
from vis import vis
import numpy
from post_processing import run
import networkx as nx

factor = 1

try:
    workspace = Workspace()
except nx.exception.NetworkXNoPath:
    workspace = Workspace()

# plot_workspace(workspace)
# plt.show()

with open('data/workspace', 'wb') as filehandle:
    pickle.dump(workspace, filehandle)

# with open('data/workspace', 'rb') as filehandle:
#     workspace.p2p = pickle.load(filehandle)
# workspace.plot_workspace()
# plt.show()

start = datetime.now()

task = Task()

buchi = Buchi(task, workspace)


buchi.construct_buchi_graph()
print((datetime.now()-start).total_seconds())
# print('partial time: {0}'.format((datetime.now() - start).total_seconds()))
print(buchi.buchi_graph.number_of_nodes(), buchi.buchi_graph.number_of_edges())


init_state, accept_state, is_nonempty_self_loop = buchi.get_init_accept()

# is_nonempty_self_loop = False

pruned_subgraph, paths = buchi.get_subgraph(init_state, accept_state, is_nonempty_self_loop)

print(pruned_subgraph.number_of_nodes(), pruned_subgraph.number_of_edges())
# print('partial time to pruned_graph: {0}'.format((datetime.now() - start).total_seconds()))

edge2element, element2edge = buchi.get_element(pruned_subgraph)

element_component2label = buchi.element2label2eccl(element2edge, pruned_subgraph)

poset_relation, pos, hasse_diagram = buchi.map_path_to_element_sequence(edge2element, element2edge, pruned_subgraph, paths)

robot2eccl = poset.element2robot2eccl(pos, element2edge, pruned_subgraph)

poset.strict_larger(element2edge, pruned_subgraph, poset_relation)

# print('partial time to poset: {0}'.format((datetime.now() - start).total_seconds()))

# for order in poset_relation:
#     print(pruned_subgraph.edges[element2edge[order[0]]]['formula'], ' -> ',
#           pruned_subgraph.edges[element2edge[order[1]]]['formula'])

incomparable_element, larger_element = poset.incomparable_larger(pos, hasse_diagram, pruned_subgraph,
                                                                 element2edge)
init_type_robot_node, element_component_clause_literal_node, node_location_type_component_element, num_nodes = \
    weighted_ts.construct_node_set(pos, element2edge, pruned_subgraph, workspace.type_robot_label)

edge_set = weighted_ts.construct_edge_set(pos, element_component_clause_literal_node,
                                          element2edge, pruned_subgraph, element_component2label, init_type_robot_node,
                                          incomparable_element, larger_element)

ts = weighted_ts.construct_graph(num_nodes, node_location_type_component_element, edge_set, workspace.p2p, factor)


# with open('data/workspace', 'wb') as filehandle:
#     pickle.dump(ts, filehandle)
# print(numpy.mean([ts.edges[e]['weight'] for e in ts.edges]))

# with open('data/workspace', 'rb') as filehandle:
#     ts_10 = pickle.load(filehandle)

# print('partial time before milp: {0}'.format((datetime.now() - start).total_seconds()))

robot_waypoint_pre, robot_time_pre, robot2robots = milp.construct_milp_constraint(ts, workspace.type_num, pos, pruned_subgraph,
                                                                    element2edge, element_component_clause_literal_node,
                                                                    poset_relation, init_type_robot_node,
                                                                    incomparable_element, larger_element, robot2eccl, factor)
# print('total time: {0}'.format((datetime.now() - start).total_seconds()))
#
robot_path_pre = milp.get_path(robot_waypoint_pre, robot_time_pre, workspace)

if is_nonempty_self_loop:
    end = datetime.now()
    # print('total time: {0}'.format((end - start).total_seconds()))
    for robot, time in list(robot_time_pre.items()):
        if time[-1] == 0:
            del robot_path_pre[robot]
            del robot_time_pre[robot]
            del robot_waypoint_pre[robot]

    robot_pre_suf_time = dict()
    for robot in robot_time_pre.keys():
        robot_pre_suf_time[robot] = [robot_time_pre[robot][-1]] * 2

    # vis(workspace, robot_path_pre, robot_pre_suf_time, task.ap)
    # for type_robot, waypoint in robot_waypoint_pre.items():
    #     print(type_robot, " : ", waypoint)
    #     print(type_robot, " : ", robot_time_pre[type_robot])
    #     print(type_robot, " : ", robot_path_pre[type_robot])
    #     # print(type_robot, " : ", list(range(round(robot_time_pre[type_robot][-1])+1)))
    c = milp.cost(robot_path_pre)
    print((end - start).total_seconds(), c)
# workspace.plot_workspace()
# workspace.path_plot(robot_path_pre)
# plt.show()
# run(pruned_subgraph, robot_path_pre, workspace.regions, init_state, accept_state)

if not is_nonempty_self_loop:
    pruned_subgraph, paths = buchi.get_subgraph(accept_state, accept_state, False)

    edge2element, element2edge = buchi.get_element(pruned_subgraph)

    element_component2label = buchi.element2label2eccl(element2edge, pruned_subgraph)

    poset_relation, pos, hasse_diagram = buchi.map_path_to_element_sequence(edge2element, element2edge, pruned_subgraph, paths)

    robot2eccl = poset.element2robot2eccl(pos, element2edge, pruned_subgraph)

    poset.strict_larger(element2edge, pruned_subgraph, poset_relation)

    incomparable_element, larger_element = poset.incomparable_larger(pos, hasse_diagram, pruned_subgraph,
                                                                     element2edge)
    # # update poset and relevant tems
    # max_pos = max(pos)
    # poset.update_poset4_suffix(pos, max_pos, element2edge, element_component2label, poset_relation, incomparable_element,
    #                            larger_element, pruned_subgraph)
    # update initial locations and terminating time
    terminator = {}
    for type_robot, label in workspace.type_robot_label.items():
        workspace.type_robot_label[type_robot] = robot_waypoint_pre[type_robot][-1]
        try:
            workspace.type_robot_location[type_robot] = workspace.regions[workspace.type_robot_label[type_robot]]
        except KeyError:
            workspace.type_robot_location[type_robot] = workspace.label_location[workspace.type_robot_label[type_robot]]

        terminator[type_robot] = robot_time_pre[type_robot][-1] / factor

    init_type_robot_node, element_component_clause_literal_node, node_location_type_component_element, num_nodes = \
        weighted_ts.construct_node_set(pos, element2edge, pruned_subgraph, workspace.type_robot_label)

    edge_set = weighted_ts.construct_edge_set(pos, element_component_clause_literal_node,
                                              element2edge, pruned_subgraph, element_component2label,
                                              init_type_robot_node, incomparable_element, larger_element)

    ts = weighted_ts.construct_graph(num_nodes, node_location_type_component_element, edge_set,
                                     workspace.p2p, factor)

    robot_waypoint_suf, robot_waypoint_plus_suf, robot_time_suf = milp_suffix.construct_milp_constraint(ts, workspace.type_num, pos,
                                                                               pruned_subgraph, element2edge,
                                                                               element_component_clause_literal_node,
                                                                               poset_relation, init_type_robot_node,
                                                                               incomparable_element, larger_element,
                                                                               terminator, robot2eccl, robot2robots,
                                                                                                        workspace, factor)
    robot_path_suf = milp_suffix.get_path(robot_waypoint_suf, robot_time_suf, workspace)
    end = datetime.now()
    print('total time: {0}'.format((end - start).total_seconds()))

    robot_path = dict()
    robot_pre_suf_time = dict()
    for type_robot in robot_path_pre.keys():
        robot_path[type_robot] = robot_path_pre[type_robot] + robot_path_suf[type_robot][1:]
        if len(robot_path_suf[type_robot]) == 1:
            robot_pre_suf_time[type_robot] = [robot_time_pre[type_robot][-1], robot_time_pre[type_robot][-1]]
        else:
            robot_pre_suf_time[type_robot] = [robot_time_suf[type_robot][1], robot_time_suf[type_robot][-1]]
    # # only display robots that are assigned tasks
    for robot, time in list(robot_pre_suf_time.items()):
        if time[-1] == 0:
            del robot_path[robot]
            del robot_pre_suf_time[robot]

    vis(workspace, robot_path, robot_pre_suf_time, task.ap)

    for type_robot, waypoint in robot_waypoint_pre.items():
        print(type_robot, " : ", waypoint)
        print(type_robot, " : ", robot_time_pre[type_robot])
        print(type_robot, " : ", robot_path_pre[type_robot])
        print(type_robot, " : ", list(range(round(robot_time_pre[type_robot][-1])+1)))

    for type_robot, waypoint in robot_waypoint_plus_suf.items():
        print(type_robot, " : ", waypoint)
        print(type_robot, " : ", robot_time_suf[type_robot])
        print(type_robot, " : ", robot_path_suf[type_robot])
        print(type_robot, " : ", list(range(round(robot_time_suf[type_robot][0]),
                                            round(robot_time_suf[type_robot][-1]) + 1)))
