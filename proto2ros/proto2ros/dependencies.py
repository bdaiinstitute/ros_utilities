# Copyright (c) 2023 Boston Dynamics AI Institute LLC. All rights reserved.

"""
This module provides APIs to manipulate dependencies between Protobuf <-> ROS message equivalences.
"""

import warnings
from typing import List

from rosidl_adapter.parser import MessageSpecification

from proto2ros.compatibility import networkx as nx
from proto2ros.utilities import pairwise, to_ros_base_type


def message_dependency_graph(message_specs: List[MessageSpecification]) -> nx.DiGraph:
    """
    Returns the dependency graph for the given ROS message specifications.

    This dependency graph is a directed multi-graph where message types make up nodes
    and composition relationships (has-a) make up edges. Nodes are annotated with the
    corresponding message specification, while edges are annotated with the corresponding
    field specification.
    """
    dependency_graph = nx.MultiDiGraph()
    for message in message_specs:
        dependency_graph.add_node(str(message.base_type), message=message)
        for field in message.fields:
            if field.type.is_primitive_type():
                continue
            dependency_graph.add_edge(str(message.base_type), to_ros_base_type(field.type), field=field)
    return dependency_graph


def fix_dependency_cycles(message_specs: List[MessageSpecification], quiet: bool = True) -> None:
    """
    Fixes dependency cycles among ROS message specifications.

    ROS messages do not support recursive definitions, this functions works around this
    limitation by type erasing the thinnest link (least number of offending fields) for
    each cycle.
    """
    dependency_graph = message_dependency_graph(message_specs)
    for cycle in nx.simple_cycles(dependency_graph):
        cycle = [*cycle, cycle[0]]  # close the loop
        if not quiet:
            message_types = [dependency_graph.nodes[node]["message"].base_type for node in cycle]
            dependency_cycle_depiction = " -> ".join(str(type_) for type_ in message_types)
            warnings.warn("Dependency cycle found: " + dependency_cycle_depiction)

        explicit_edges = []
        for parent, child in pairwise(cycle):
            message = dependency_graph.nodes[child]["message"]
            if message.annotations["proto-class"] == "message":
                explicit_edges.append((parent, child))

        parent, child = min(explicit_edges, key=lambda edge: dependency_graph.number_of_edges(*edge))
        for data in dependency_graph[parent][child].values():
            field = data["field"]
            if not quiet:
                message_type = dependency_graph.nodes[parent]["message"].base_type
                warnings.warn(f"Type erasing {field.name} member in {message_type} to break recursion")
            field.annotations["type-erased"] = True
