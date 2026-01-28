# path_planning/filters.py
import numpy as np
import networkx as nx

def build_safe_graph(vor, colors):
    """
    Filters Voronoi ridges based on Color Mismatch and Distance.
    Returns a NetworkX graph of safe paths.
    """
    G = nx.Graph()
    
    # Iterate through all potential paths (ridges)
    for (p1_idx, p2_idx), (v1_idx, v2_idx) in vor.ridge_dict.items():
        
        # 1. Color Check: Logic Step 4
        # Skip if both cones are the same color (Blue-Blue or Yellow-Yellow)
        if colors[p1_idx] == colors[p2_idx]:
            continue

        # Check for infinite ridges (open ends)
        if v1_idx == -1 or v2_idx == -1:
            continue

        # Get the actual physical points of the path line
        p_start = vor.vertices[v1_idx]
        p_end = vor.vertices[v2_idx]

        # 2. Distance Check: Logic Step 4 (Pruning)
        dist = np.linalg.norm(p_start - p_end)
        if dist > 3.0:  # Max allowable path segment length
            continue

        # Add to Graph
        # We use the vertex index as the Node ID, and store 'pos' data
        G.add_node(v1_idx, pos=p_start)
        G.add_node(v2_idx, pos=p_end)
        G.add_edge(v1_idx, v2_idx, weight=dist)

    return G