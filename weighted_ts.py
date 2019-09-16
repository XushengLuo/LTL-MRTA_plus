import itertools
import networkx as nx


def construct_node_set(poset, element2edge, pruned_subgraph, type_robot_label):
    node_index = 0
    # nodes for initial location
    init_type_robot_node = dict()
    # nodes for label
    element_component_clause_literal_node = dict()
    # attributes of nodes
    node_location_type_component_element = dict()
    # type_robot (robot type, robot index in the type), location: l2,r1
    for type_robot, location in type_robot_label.items():
        init_type_robot_node[type_robot] = node_index
        # treat as edge component of element -1
        node_location_type_component_element[node_index] = [location, type_robot[0], 1, -1]
        node_index += 1

    # node set for self-loop label
    for element in poset:
        edge_label = pruned_subgraph.edges[element2edge[element]]['label']
        # node set for edge label, edge label not equal 1
        if edge_label != '1':
            node_index = construct_node_set_helper(element, 1, edge_label, element_component_clause_literal_node,
                                                   node_location_type_component_element, node_index)
        else:
            continue  # no node for the element if the edge label if '1'
        # with self loop
        self_loop_label = pruned_subgraph.nodes[element2edge[element][0]]['label']
        if self_loop_label and self_loop_label != '1':
            node_index = construct_node_set_helper(element, 0, self_loop_label, element_component_clause_literal_node,
                                                   node_location_type_component_element, node_index)

    return init_type_robot_node, element_component_clause_literal_node, node_location_type_component_element, node_index


def construct_node_set_helper(element, component, label, element_component_clause_literal_node,
                              node_location_type_component_element, node_index):
    for c in range(len(label)):
        for l in range(len(label[c])):
            end = node_index + label[c][l][2]
            element_component_clause_literal_node[(element, component, c, l)] = list(range(node_index, end))
            node_location_type_component_element.update({node: label[c][l][0:2] + [component, element]
                                                         for node in list(range(node_index, end))})
            node_index = end
    return node_index


def construct_edge_set(poset, element_component_clause_literal_node, element2edge, pruned_subgraph,
                       element_component2label, init_type_robot_node, incomparable_element, larger_element):

    edge_set = []
    for element in poset:
        incmp_element = incomparable_element[element] + larger_element[element]

        self_loop_label = pruned_subgraph.nodes[element2edge[element][0]]['label']
        edge_label = pruned_subgraph.edges[element2edge[element]]['label']

        # edge label
        if edge_label != '1':
            construct_edge_set_helper(element, 1, element_component2label,
                                      element_component_clause_literal_node,
                                      init_type_robot_node, incmp_element, edge_set)
        else:
            continue
        # with self loop
        if self_loop_label and self_loop_label != '1':
            construct_edge_set_helper(element, 0, element_component2label,
                                      element_component_clause_literal_node,
                                      init_type_robot_node, incmp_element, edge_set)

    return edge_set


def construct_edge_set_helper(element, component, element_component2label, element_component_clause_literal_node,
                              init_type_robot_node, incmp_element, edge_set):
    for label, eccls in element_component2label[(element, component)].items():
        # from initial locations
        from_node = [node for type_robot, node in init_type_robot_node.items() if type_robot[0] == label[1]]
        # collect all nodes that share same region and robot type
        for eccl in eccls:
            to_node = element_component_clause_literal_node[eccl]
            edge_set += list(itertools.product(from_node, to_node))
            # find all qualified nodes that share the same robot type from incomparable or precedent nodes
            for in_ele in incmp_element:
                # self loop label
                try:
                    for in_label, in_eccls in element_component2label[(in_ele, 0)].items():
                        if label[1] == in_label[1]:  # same robot type
                            for in_eccl in in_eccls:
                                if len(element_component_clause_literal_node[in_eccl]) >= len(to_node):
                                    from_node = element_component_clause_literal_node[in_eccl][:len(to_node)]
                                    edge_set += [(from_node[i], to_node[i]) for i in range(len(to_node))]
                                else:
                                    edge_set += list(itertools.product(element_component_clause_literal_node[in_eccl],
                                                                       to_node))

                except KeyError:  # self loop label is 1
                    pass
                # edge label
                for in_label, in_eccls in element_component2label[(in_ele, 1)].items():
                    if label[1] == in_label[1]:
                        for in_eccl in in_eccls:
                            if len(element_component_clause_literal_node[in_eccl]) >= len(to_node):
                                from_node = element_component_clause_literal_node[in_eccl][:len(to_node)]
                                edge_set += [(from_node[i], to_node[i]) for i in range(len(to_node))]
                            else:
                                edge_set += list(itertools.product(element_component_clause_literal_node[in_eccl],
                                                                   to_node))

            # additional nodes for edge label, nodes from self loop label of the current element
            if component == 1:
                # self loop label
                try:
                    for in_label, in_eccls in element_component2label[(element, 0)].items():
                        if label[1] == in_label[1]:  # same robot type
                            for in_eccl in in_eccls:
                                if len(element_component_clause_literal_node[in_eccl]) >= len(to_node):
                                    from_node = element_component_clause_literal_node[in_eccl][:len(to_node)]
                                    edge_set += [(from_node[i], to_node[i]) for i in range(len(to_node))]
                                else:
                                    edge_set += list(itertools.product(element_component_clause_literal_node[in_eccl],
                                                                       to_node))

                except KeyError:
                    pass


def construct_graph(num_nodes, node_location_type_component_element, edge_set, p2p, factor):

    ts = nx.DiGraph()
    for node in list(range(num_nodes)):
        ts.add_node(node, location_type_component_element=node_location_type_component_element[node])
    for edge in edge_set:
        ts.add_edge(edge[0], edge[1], weight=1/factor*p2p[(ts.nodes[edge[0]]['location_type_component_element'][0],
                                                           ts.nodes[edge[1]]['location_type_component_element'][0])])
    return ts


# those functions below are designed for the idea of the first round, where robot enter the cycle and run cycle once and
# repeat along the cycle
# def construct_node_set_suffix(poset, element2edge, pruned_subgraph, type_robot_label):
#     node_index = 0
#     # nodes for initial location
#     init_type_robot_node = dict()
#     # nodes for label
#     element_component_clause_literal_node = dict()
#     # attributes of nodes
#     node_location_type_component_element = dict()
#     # type_robot (robot type, robot index in the type), location: l2,r1
#     for type_robot, label in type_robot_label.items():
#         init_type_robot_node[type_robot] = node_index
#         # treat as edge component of element -1
#         node_location_type_component_element[node_index] = [label, type_robot[0], 1, -1]
#         node_index += 1
#
#     # node set for self-loop label
#     for element in poset:
#         edge_label = pruned_subgraph.edges[element2edge[element]]['label']
#         # node set for edge label, edge label not equal 1
#         if edge_label != '1':
#             node_index = construct_node_set_helper_suffix(element, 1, edge_label,
#                                                           element_component_clause_literal_node,
#                                                           node_location_type_component_element, node_index)
#         else:
#             continue  # no node for the element if the edge label if '1'
#         # with self loop
#         self_loop_label = pruned_subgraph.nodes[element2edge[element][0]]['label']
#         if self_loop_label and self_loop_label != '1':
#             node_index = construct_node_set_helper_suffix(element, 0, self_loop_label,
#                                                           element_component_clause_literal_node,
#                                                           node_location_type_component_element, node_index)
#
#     return init_type_robot_node, element_component_clause_literal_node, node_location_type_component_element, node_index
#
#
# def construct_node_set_helper_suffix(element, component, label, element_component_clause_literal_node,
#                                      node_location_type_component_element, node_index):
#     for c in range(len(label)):
#         for l in range(len(label[c])):
#             end = node_index + label[c][l][2]
#             element_component_clause_literal_node[(element, component, c, l)] = list(range(node_index, end))
#             node_location_type_component_element.update({node: label[c][l][0:2] + [component, element]
#                                                          for node in list(range(node_index, end))})
#             node_index = end
#     return node_index
#
#
# def construct_edge_set_suffix(poset, element_component_clause_literal_node, element2edge,
#                               pruned_subgraph, element_component2label, init_type_robot_node,
#                               incomparable_element, larger_element, max_pos):
#
#     edge_set = []
#     for element in poset:
#         self_loop_label = pruned_subgraph.nodes[element2edge[element][0]]['label']
#         edge_label = pruned_subgraph.edges[element2edge[element]]['label']
#
#         # edge label
#         if edge_label != '1':
#             construct_edge_set_helper_suffix(element, 1, element_component2label, element_component_clause_literal_node,
#                                              init_type_robot_node, incomparable_element, larger_element, edge_set,
#                                              max_pos)
#         else:
#             continue
#         # with self loop
#         if self_loop_label and self_loop_label != '1':
#             construct_edge_set_helper_suffix(element, 0, element_component2label, element_component_clause_literal_node,
#                                              init_type_robot_node, incomparable_element, larger_element, edge_set,
#                                              max_pos)
#
#     return edge_set
#
#
# def construct_edge_set_helper_suffix(element, component, element_component2label,
#                                      element_component_clause_literal_node,
#                                      init_type_robot_node, incomparable_element, larger_element, edge_set, max_pos):
#     incmp_larger = incomparable_element[element] + larger_element[element]
#     for label, eccls in element_component2label[(element, component)].items():
#         # from initial locations
#         from_node = [node for type_robot, node in init_type_robot_node.items() if type_robot[0] == label[1]]
#         # collect all nodes that share same region and robot type
#         for eccl in eccls:
#             to_node = element_component_clause_literal_node[eccl]
#             if element <= max_pos:
#                 edge_set += list(itertools.product(from_node, to_node))
#             # find all qualified nodes that share the same robot type from incomparable or precedent nodes
#             for in_ele in incmp_larger:
#                 # self loop label
#                 try:
#                     for in_label, in_eccls in element_component2label[(in_ele, 0)].items():
#                         if label[1] == in_label[1]:  # same robot type
#                             for in_eccl in in_eccls:
#                                 if len(element_component_clause_literal_node[in_eccl]) >= len(to_node):
#                                     from_node = element_component_clause_literal_node[in_eccl][:len(to_node)]
#                                     edge_set += [(from_node[i], to_node[i]) for i in range(len(to_node))]
#                                 else:
#                                     edge_set += list(itertools.product(element_component_clause_literal_node[in_eccl],
#                                                                        to_node))
#
#                 except KeyError:  # self loop label is 1
#                     pass
#                 # edge label
#                 for in_label, in_eccls in element_component2label[(in_ele, 1)].items():
#                     if label[1] == in_label[1]:
#                         for in_eccl in in_eccls:
#                             if len(element_component_clause_literal_node[in_eccl]) >= len(to_node):
#                                 from_node = element_component_clause_literal_node[in_eccl][:len(to_node)]
#                                 edge_set += [(from_node[i], to_node[i]) for i in range(len(to_node))]
#                             else:
#                                 edge_set += list(itertools.product(element_component_clause_literal_node[in_eccl],
#                                                                    to_node))
#
#             # additional nodes for edge label, nodes from self loop label of the current element
#             if component == 1:
#                 # self loop label
#                 try:
#                     for in_label, in_eccls in element_component2label[(element, 0)].items():
#                         if label[1] == in_label[1]:  # same robot type
#                             for in_eccl in in_eccls:
#                                 if len(element_component_clause_literal_node[in_eccl]) >= len(to_node):
#                                     from_node = element_component_clause_literal_node[in_eccl][:len(to_node)]
#                                     edge_set += [(from_node[i], to_node[i]) for i in range(len(to_node))]
#                                 else:
#                                     edge_set += list(itertools.product(element_component_clause_literal_node[in_eccl],
#                                                                        to_node))
#
#                 except KeyError:
#                     pass
