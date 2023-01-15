import sys

import osmnx as ox
import numpy as np
import queue
import math
import priority_dict
from visualization import plot_path

map_graph = ox.graph.graph_from_address('Gummersbach, Steinmüllerallee 1, Germany', dist=5000, network_type='bike')

# Breiten- und Längengrad
origin_point = (50.985108, 7.542490)
destination_point = (51.022255, 7.562705)  # Das Ziel, der Campus Gummersbach

origin = ox.distance.nearest_nodes(map_graph, origin_point[1], origin_point[0])
destination = ox.distance.nearest_nodes(map_graph, destination_point[1], destination_point[0])

print(origin)
print(destination)

shortest_path = ox.distance.shortest_path(map_graph, origin, destination, weight='length')

plot_path(map_graph, shortest_path, origin_point, destination_point)


def dijkstras_search(origin_key, goal_key, graph):
    known = set()
    predecessors = dict()
    perimeter = priority_dict.priority_dict()

    for node in graph:
        perimeter[node] = sys.maxsize
        predecessors[node] = None

    perimeter[origin_key] = 0

    while perimeter:
        current_key, current_distance = perimeter.pop_smallest()
        known.add(current_key)

        for edge in graph.out_edges([current_key], data=True):
            neighbor_key = edge[1]
            distance_from_current_to_neighbor = edge[2]['length']

            if neighbor_key in known:
                continue

            old_distance_to_neighbor = perimeter[neighbor_key]
            new_distance_to_neighbor = current_distance + distance_from_current_to_neighbor

            if new_distance_to_neighbor < old_distance_to_neighbor:
                perimeter[neighbor_key] = new_distance_to_neighbor
                predecessors[neighbor_key] = current_key

    return get_path(origin_key, goal_key, predecessors)


def get_path(origin_key, goal_key, predecessors):
    key = goal_key
    path = [goal_key]
    while key != origin_key:
        key = predecessors[key]
        path.insert(0, key)

    return path


path = dijkstras_search(origin, destination, map_graph)
plot_path(map_graph, path, origin_point, destination_point)
