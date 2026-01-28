# path_planning/graph_search.py
import networkx as nx
import numpy as np

def find_optimal_path(graph, car_pos, car_orientation):
    """
    Finds the path from the node closest to the car, 
    moving roughly in the car's orientation.
    """
    if len(graph.nodes) == 0:
        return []

    # 1. Find Start Node (Closest to Car)
    start_node = None
    min_dist = float('inf')
    
    for node_id in graph.nodes:
        pos = graph.nodes[node_id]['pos']
        dist = np.linalg.norm(pos - car_pos)
        if dist < min_dist:
            min_dist = dist
            start_node = node_id

    if start_node is None:
        return []

    # 2. Find End Node
    # Heuristic: Find the node that is furthest away from the start
    # (In a full race code, you would use car_orientation to filter nodes behind the car)
    path_lengths = nx.single_source_dijkstra_path_length(graph, start_node)
    
    # Just pick the furthest connected node for now
    end_node = max(path_lengths, key=path_lengths.get)

    # 3. Retrieve Path
    try:
        path_indices = nx.shortest_path(graph, start_node, end_node)
        # Convert indices back to coordinates
        final_path = [graph.nodes[i]['pos'] for i in path_indices]
        return final_path
    except nx.NetworkXNoPath:
        return []