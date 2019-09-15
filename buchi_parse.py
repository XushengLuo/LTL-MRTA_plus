# -*- coding: utf-8 -*-

import subprocess
import os.path
import re
import networkx as nx
import numpy as np
from networkx.classes.digraph import DiGraph
import datetime
from sympy.logic.boolalg import to_dnf


class Buchi(object):
    """
    construct buchi automaton graph
    """

    def __init__(self, task, workspace):
        """
        initialization
        :param task: task specified in LTL
        """
        # task specified in LTL
        self.formula = task.formula
        self.type_num = workspace.type_num
        # graph of buchi automaton
        self.buchi_graph = DiGraph(type='buchi', init=[], accept=[])

    def construct_buchi_graph(self):
        """
        parse the output of the program ltl2ba and build the buchi automaton
        """
        # directory of the program ltl2ba
        dirname = os.path.dirname(__file__)
        # output of the program ltl2ba
        output = subprocess.check_output(dirname + "/./ltl2ba -f \"" + self.formula + "\"", shell=True).decode(
            "utf-8")

        # find all states/nodes in the buchi automaton
        state_re = re.compile(r'\n(\w+):\n\t')
        state_group = re.findall(state_re, output)

        # find initial and accepting states
        init = [s for s in state_group if 'init' in s]
        # treat the node accept_init as init node
        accept = [s for s in state_group if 'accept' in s]
        # finish the inilization of the graph of the buchi automaton
        self.buchi_graph.graph['init'] = init
        self.buchi_graph.graph['accept'] = accept

        # for each state/node, find it transition relations
        for state in state_group:
            # add node
            self.buchi_graph.add_node(state, label='', formula='')
            # loop over all transitions starting from current state
            state_if_fi = re.findall(state + r':\n\tif(.*?)fi', output, re.DOTALL)
            if state_if_fi:
                relation_group = re.findall(r':: (\(.*?\)) -> goto (\w+)\n\t', state_if_fi[0])
                for symbol, next_state in relation_group:
                    symbol = symbol.replace('||', '|').replace('&&', '&')
                    edge_label = self.merge_check_feasible(symbol)
                    if edge_label:
                        # update node, no edges for selfloop
                        if state == next_state:
                            self.buchi_graph.nodes[state]['label'] = edge_label
                            self.buchi_graph.nodes[state]['formula'] = to_dnf(symbol)
                            continue
                        # add edge
                        self.buchi_graph.add_edge(state, next_state, label=edge_label, formula=to_dnf(symbol))

            else:
                state_skip = re.findall(state + r':\n\tskip\n', output, re.DOTALL)
                if state_skip:
                    self.buchi_graph.nodes[state]['label'] = 'skip'
                    self.buchi_graph.nodes[state]['formula'] = 'skip'

        self.delete_node_no_selfloop_except_init_accept()

    def delete_node_no_selfloop_except_init_accept(self):

        remove_node = []
        remove_edge = []
        for node in self.buchi_graph.nodes():
            # if no selfloop
            if self.buchi_graph.nodes[node]['label'] == '':
                # delete if not init or accept
                if 'init' not in node and 'accept' not in node:
                    remove_node.append(node)
                # init or accept in node then only keep the edge with label '1'
                elif 'init' in node:  # or 'accept' in node:
                    for n in self.buchi_graph.succ[node]:
                        if self.buchi_graph.edges[(node, n)]['label'] != '1':
                            remove_edge.append((node, n))
            # if there is self loop but not equal 1, only keep the edge with label '1'
            elif 'init' in node and self.buchi_graph.nodes[node]['label'] != '1':
                for n in self.buchi_graph.succ[node]:
                    if self.buchi_graph.edges[(node, n)]['label'] != '1':
                        remove_edge.append((node, n))

        self.buchi_graph.remove_edges_from(remove_edge)
        self.buchi_graph.remove_nodes_from(remove_node)

    def get_init_accept(self):
        """
        search the shortest path from a node to another, i.e., # of transitions in the path
        :return:
        """
        # shortest path for each accepting state to itself, including self-loop
        accept_accept = dict()
        for accept in self.buchi_graph.graph['accept']:
            # 0 if with self loop or without outgoing edges
            if self.buchi_graph.nodes[accept]['formula'] == 'skip' or self.buchi_graph.nodes[accept]['formula']:
                accept_accept[accept] = 0
                continue
            # find the shortest path back to itself
            length = np.inf
            for suc in self.buchi_graph.succ[accept]:
                try:
                    len1, _ = nx.algorithms.single_source_dijkstra(self.buchi_graph,
                                                                        source=suc, target=accept)
                except nx.exception.NetworkXNoPath:
                    len1 = np.inf
                if len1 < length:
                    length = len1 + 1
            accept_accept[accept] = length

        # shortest path from initial to accept
        init_state = None
        accept_state = None
        length = np.inf
        for init in self.buchi_graph.graph['init']:
            for accept in self.buchi_graph.graph['accept']:
                # if same, find the shorest path to itself
                if init == accept:
                    len1 = np.inf
                    for suc in self.buchi_graph.succ[init]:
                        try:
                            len2, _ = nx.algorithms.single_source_dijkstra(self.buchi_graph,
                                                                           source=suc, target=accept)
                        except nx.exception.NetworkXNoPath:
                            len2 = np.inf
                        if len2 + 1 < len1:
                            len1 = len2 + 1
                # else different, find the shorest path to accept
                else:
                    try:
                        len1, _ = nx.algorithms.single_source_dijkstra(self.buchi_graph,
                                                                         source=init, target=accept)
                    except nx.exception.NetworkXNoPath:
                        continue
                # init_to_accept + accept_to_accept
                len1 = len1 + accept_accept[accept]
                # update
                if len1 < length:
                    init_state = init
                    accept_state = accept
                    length = len1

        # self loop around the accepting state
        self_loop_label = self.buchi_graph.nodes[accept_state]['label']
        return init_state, accept_state, self_loop_label != '1' and self_loop_label != ''

    def merge_check_feasible(self, symbol):
        # [[[literal], .. ], [[literal], .. ], [[literal], .. ]]
        #       clause            clause            clause
        pruned_clause = []
        if symbol == '(1)':
            return '1'
        # non-empty symbol
        else:
            clause = symbol.split(' | ')
            # merge
            for c in clause:
                literals_ = c.strip('(').strip(')').split(' & ')
                literals_.sort()
                literals_.reverse()
                i = 0
                literals = []
                while i < len(literals_):
                    curr = literals_[i].split('_')
                    literals.append(curr)
                    # loop over subsequent elements
                    j = i + 1
                    while j < len(literals_):
                        subsq = literals_[j].split('_')
                        if curr[0] != subsq[0]:
                            break
                        # same # of robots of same type with indicator 0
                        if curr[1] == subsq[1] and curr[2] >= subsq[2] and int(curr[3]) + int(subsq[3]) == 0:
                            del literals_[j]
                            continue
                        j += 1
                    # make it number
                    curr[1] = int(curr[1])
                    curr[2] = int(curr[2])
                    curr[3] = int(curr[3])
                    i += 1

                # if two literal use the same robot, infeasible
                if sum([l[3] for l in literals]) != sum(set([l[3] for l in literals])):
                    continue
                # # check feasibility, whether required robots of same type for one literal exceeds provided robots
                for i in range(len(literals)):
                    if self.type_num[literals[i][1]] < literals[i][2]:
                        raise ValueError('{0} required robots exceeds provided robots'.format(literals[i]))
                # check feasibility, the required number of robots of same type for all literals exceed provided robots
                for robot_type, num in self.type_num.items():
                    if num < sum([l[2] for l in literals if l[1] == robot_type]):
                        literals = []
                        break
                if literals:
                    pruned_clause.append(literals)

        return pruned_clause

    def get_subgraph(self, head, tail, is_nonempty_self_loop):
        # node set
        nodes = self.find_all_nodes(head, tail)
        subgraph = self.buchi_graph.subgraph(nodes).copy()
        subgraph.graph['init'] = head
        subgraph.graph['accept'] = tail

        # remove all outgoing edges of the accepting state from subgraph for the prefix part
        if head != tail:
            remove_edge = list(subgraph.edges(tail))
            subgraph.remove_edges_from(remove_edge)

        # prune the subgraph
        self.prune_subgraph(subgraph)

        # get all paths in the pruned subgraph
        paths = []
        if head != tail:
            paths = list(nx.all_simple_paths(subgraph, source=head, target=tail))
        else:
            for s in subgraph.succ[head]:
                paths = paths + [[head] + p for p in list(nx.all_simple_paths(subgraph, source=s, target=tail))]
        # if self loop around the accepting state, then create an element with edge label 1,
        # solving prefix and suffix together
        if is_nonempty_self_loop and subgraph.nodes[tail]['label'] != 'skip':
            subgraph.add_edge(tail, tail, label=subgraph.nodes[tail]['label'], formula=subgraph.nodes[tail]['formula'])
            for path in paths:
                path.append(tail)

        return subgraph, paths

    def get_element(self, pruned_subgraph):

        edges = list(pruned_subgraph.edges)
        curr_element = 0
        # map each edge to element
        edge2element = dict()
        element2edge = dict()
        while len(edges) > 0:
            # put all elements with same node label and edge label into one list
            curr = edges[0]
            self_loop_label = [curr]
            del edges[0]
            j = 0
            while j < len(edges):
                if pruned_subgraph.nodes[curr[0]]['formula'] == pruned_subgraph.nodes[edges[j][0]]['formula'] \
                        and pruned_subgraph.edges[curr]['formula'] == pruned_subgraph.edges[edges[j]]['formula']:
                    self_loop_label.append(edges[j])
                    del edges[j]
                    continue
                else:
                    j += 1

            # partition into two groups
            dependent = []
            independent = []
            while len(self_loop_label) > 0:
                # pick the first element
                is_independent = True
                curr = self_loop_label[0]
                del self_loop_label[0]
                k = 0
                while k < len(self_loop_label):
                    if nx.has_path(pruned_subgraph, curr[1], self_loop_label[k][0]):
                        # if there is a path between the first element and current element
                        is_independent = False
                        dependent.append(self_loop_label[k])
                        del self_loop_label[k]
                        continue
                    else:
                        k += 1
                # address the first element
                if is_independent:
                    independent.append(curr)
                else:
                    dependent.append(curr)
            # map edge/element to element/edge
            if independent:
                curr_element += 1
                element2edge[curr_element] = independent[0]
                for edge in independent:
                    edge2element[edge] = curr_element

            if dependent:
                for edge in dependent:
                    curr_element += 1
                    edge2element[edge] = curr_element
                    element2edge[curr_element] = edge

        return edge2element, element2edge

    def element2label2eccl(self, element2edge, pruned_subgraph):
        element_component2label = dict()
        # colloect eccl that correponds to the same label
        for element in element2edge.keys():
            # node label
            self_loop_label = pruned_subgraph.nodes[element2edge[element][0]]['label']
            if self_loop_label and self_loop_label != '1':
                element_component2label[(element, 0)] = self.element2label2eccl_helper(element, 0, self_loop_label)

            # edge label
            edge_label = pruned_subgraph.edges[element2edge[element]]['label']
            if edge_label != '1':
                element_component2label[(element, 1)] = self.element2label2eccl_helper(element, 1, edge_label)

        return element_component2label

    def element2label2eccl_helper(self, element, component, label):
        label2eccl = dict()
        for c, clause in enumerate(label):
            for l, literal in enumerate(clause):
                region_type = tuple(literal[0:2])
                if region_type in label2eccl.keys():
                    label2eccl[region_type].append((element, component, c, l))
                else:
                    label2eccl[region_type] = [(element, component, c, l)]
        return label2eccl

    def map_path_to_element_sequence(self, edge2element, paths):
        element_sequences = []
        # put all path that share the same set of elements into one group
        for path in paths:
            element_sequence = []
            for i in range(len(path)-1):
                element_sequence.append(edge2element[(path[i], path[i+1])])
            is_added = False
            for i in range(len(element_sequences)):
                if set(element_sequences[i][0]) == set(element_sequence):
                    element_sequences[i].append(element_sequence)
                    is_added = True
                    break
            if not is_added:
                element_sequences.append([element_sequence])

        graph_with_maximum_width = DiGraph()
        width = 0
        height = 0
        # for each group, find one poset
        for ele_seq in element_sequences:
            # all pairs of elements from the first element
            linear_order = []
            for i in range(len(ele_seq[0])):
                for j in range(i+1, len(ele_seq[0])):
                    linear_order.append((ele_seq[0][i], ele_seq[0][j]))
            # remove contradictive pairs
            for i in range(1, len(ele_seq)):
                for j in range(len(ele_seq[1])-1):
                    if (ele_seq[i][j+1], ele_seq[i][j]) in linear_order:
                        linear_order.remove((ele_seq[i][j+1], ele_seq[i][j]))
            # hasse diagram
            hasse = DiGraph()
            hasse.add_nodes_from(ele_seq[0])
            hasse.add_edges_from(linear_order)
            self.prune_subgraph(hasse)
            w = max([len(o) for o in nx.antichains(hasse)])
            h = nx.dag_longest_path_length(hasse)
            if w > width or (w == width and h > height):
                graph_with_maximum_width = hasse
                width = w
                height = h

        return {edge for edge in graph_with_maximum_width.edges}, list(graph_with_maximum_width.nodes), \
                            graph_with_maximum_width

    def prune_subgraph(self, subgraph):
        original_edges = list(subgraph.edges)
        for edge in original_edges:
            attr = subgraph.get_edge_data(edge[0], edge[1])
            subgraph.remove_edge(edge[0], edge[1])
            if nx.has_path(subgraph, edge[0], edge[1]):
                continue
            else:
                subgraph.add_edge(edge[0], edge[1], **attr)

    def element2label2eccl_suffix(self, element2edge, pruned_subgraph):
        stage_element_component2label = dict()
        # colloect eccl that correponds to the same label
        for element in element2edge.keys():
            # node label
            self_loop_label = pruned_subgraph.nodes[element2edge[element][0]]['label']
            if self_loop_label and self_loop_label != '1':
                stage_element_component2label[(1, element, 0)] = self.element2label2eccl_helper_suffix(1, element, 0,
                                                                                                       self_loop_label)
                stage_element_component2label[(2, element, 0)] = self.element2label2eccl_helper_suffix(2, element, 0,
                                                                                                       self_loop_label)

            # edge label
            edge_label = pruned_subgraph.edges[element2edge[element]]['label']
            if edge_label != '1':
                stage_element_component2label[(1, element, 1)] = self.element2label2eccl_helper_suffix(1, element, 1,
                                                                                                       edge_label)
                stage_element_component2label[(2, element, 1)] = self.element2label2eccl_helper_suffix(2, element, 1,
                                                                                                       edge_label)

        return stage_element_component2label

    def element2label2eccl_helper_suffix(self, stage, element, component, label):
        label2eccl = dict()
        for c, clause in enumerate(label):
            for l, literal in enumerate(clause):
                region_type = tuple(literal[0:2])
                if region_type in label2eccl.keys():
                    label2eccl[region_type].append((stage, element, component, c, l))
                else:
                    label2eccl[region_type] = [(stage, element, component, c, l)]
        return label2eccl

    def find_all_nodes(self, head, tail):
        front = set({head})
        explored = set()
        in_between = set({head})
        while True:
            try:
                node = front.pop()
                explored.add(node)
                for s in self.buchi_graph.succ[node]:
                    if nx.has_path(self.buchi_graph, s, tail) and s not in explored and s not in front:
                        front.add(s)
                        in_between.add(s)
            except KeyError:
                break
        return in_between
