"""
Filter module for building safe graph from Voronoi diagram.
Applies color matching, distance pruning, and collision detection.
"""

import numpy as np
import networkx as nx
from .collision import is_collision, build_obstacle_tree


def remove_ghost_cones(cone_data, max_neighbor_dist=6.0, min_neighbors=1, partner_x_tolerance=1.5, y_alignment_tolerance=0.8):
    """
    Remove isolated or misaligned cones likely to be false positives ("ghosts").

    A cone is kept if:
    1. It has at least `min_neighbors` other cones within `max_neighbor_dist`
    2. (Only if mixed colors exist) It has opposite-color partner cones aligned on X-axis
    3. It is aligned with same-color group on Y-axis (not floating in middle)

    :param cone_data: List of tuples [(x, y, color), ...]
    :param max_neighbor_dist: Neighbor radius for validation
    :param min_neighbors: Minimum count of neighbors required to keep a cone
    :param partner_x_tolerance: Max X-distance to find opposite-color partner
    :param y_alignment_tolerance: Max Y-distance to find same-color group alignment
    :return: Filtered cone data list
    """
    if len(cone_data) <= 1:
        return cone_data

    points = np.array([[c[0], c[1]] for c in cone_data])
    tree = build_obstacle_tree(cone_data)
    
    # Check if we have both colors
    colors_present = set(c[2] for c in cone_data)
    has_mixed_colors = len(colors_present) > 1

    filtered = []
    removed_count = 0
    
    for idx, cone in enumerate(cone_data):
        neighbors = tree.query_ball_point(points[idx], r=max_neighbor_dist)
        neighbor_count = max(len(neighbors) - 1, 0)  # exclude self
        
        # Check 1: Must have neighbors
        if neighbor_count < min_neighbors:
            removed_count += 1
            print(f"[GHOST FILTER] Removed ghost cone at ({cone[0]:.2f}, {cone[1]:.2f}, '{cone[2]}') - isolated (no neighbors within {max_neighbor_dist}m)")
            continue
        
        # Check 2: If mixed colors exist, must have opposite-color partner on X-axis
        if has_mixed_colors:
            cone_color = cone[2]
            opposite_color = 'b' if cone_color == 'y' else 'y'
            
            # Find opposite-color cones roughly aligned on X-axis
            aligned_opposite = [
                c for c in cone_data
                if c[2] == opposite_color and abs(c[0] - cone[0]) <= partner_x_tolerance
            ]
            
            if len(aligned_opposite) == 0:
                removed_count += 1
                print(f"[GHOST FILTER] Removed ghost cone at ({cone[0]:.2f}, {cone[1]:.2f}, '{cone[2]}') - no opposite-color partner aligned on X-axis")
                continue
        
        # Check 3: Must be aligned with same-color group on Y-axis (not floating in middle)
        if has_mixed_colors:
            same_color_cones = [c for c in cone_data if c[2] == cone[2]]
            
            if len(same_color_cones) > 1:
                # Get Y-positions of other same-color cones
                other_y_positions = [c[1] for c in same_color_cones if c != cone]
                
                # Find if this cone's Y is aligned with the group
                min_y = min(other_y_positions)
                max_y = max(other_y_positions)
                avg_y = np.mean(other_y_positions)
                
                # Cone should be close to the cluster center, not floating away
                if abs(cone[1] - avg_y) > y_alignment_tolerance:
                    removed_count += 1
                    print(f"[GHOST FILTER] Removed ghost cone at ({cone[0]:.2f}, {cone[1]:.2f}, '{cone[2]}') - Y-misaligned with same-color group (avg Y={avg_y:.2f})")
                    continue
        
        filtered.append(cone)

    if removed_count > 0:
        print(f"[GHOST FILTER] Total ghost cones removed: {removed_count}")
    
    return filtered


def build_safe_graph(vor, colors, cone_data, robot_radius, max_edge_len, safety_margin):
    """
    A function that accepts a voronoi structure vor and a list of cone colors.
    Filters Voronoi ridges based on:
    1. Color Mismatch (blue-yellow pairs only)
    2. Distance pruning
    3. Collision detection with cones
    
    Returns a NetworkX graph of safe paths.
    
    :param vor: Voronoi diagram object
    :param colors: List of cone colors corresponding to Voronoi input points
    :param cone_data: Original cone data [(x, y, color), ...]
    :param robot_radius: Effective radius of vehicle
    :param max_edge_len: Maximum allowable edge length
    :param safety_margin: Additional safety buffer around robot
    :return: NetworkX graph with safe edges
    """
    G = nx.Graph()  # Create an empty graph to hold safe edges and vertices

    # Build KDTree for efficient collision detection
    obstacle_tree = build_obstacle_tree(cone_data)
    
    # Effective robot radius with safety margin
    effective_radius = robot_radius + safety_margin

    # Iterate through all potential paths (ridges)
    for (p1_idx, p2_idx), (v1_idx, v2_idx) in vor.ridge_dict.items():

        # 1. Color Check: Skip if both cones are the same color
        if colors[p1_idx] == colors[p2_idx]:
            continue

        # Check for infinite ridges (open ends)
        if v1_idx == -1 or v2_idx == -1:
            continue

        # Get the actual physical 2D points of the voronoi vertices
        p_start = vor.vertices[v1_idx]
        p_end = vor.vertices[v2_idx]

        # 2. Distance Check: Prune edges that are too long
        dist = np.linalg.norm(p_start - p_end)
        if dist > max_edge_len:
            continue

        # 3. Collision Check: Ensure edge doesn't collide with cones
        collision_detected = is_collision(
            p_start[0], p_start[1],
            p_end[0], p_end[1],
            effective_radius,
            obstacle_tree,
            max_edge_len
        )
        
        if collision_detected:
            continue  # Skip this edge if collision detected

        # Add to Graph (only if all checks passed)
        G.add_node(v1_idx, pos=p_start)
        G.add_node(v2_idx, pos=p_end)
        G.add_edge(v1_idx, v2_idx, weight=dist)

    return G