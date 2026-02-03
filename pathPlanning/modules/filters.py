"""
Filter module for building safe graph from Voronoi diagram.
Applies color matching, distance pruning, and collision detection.
"""

import numpy as np
import networkx as nx
from scipy.interpolate import splprep, splev
from .collision import is_collision, build_obstacle_tree


def _fit_trajectory_curve(cone_positions):
    """
    Fit a smooth spline through cone positions (sorted by X).
    Returns a function that predicts Y given X along the trajectory.
    
    :param cone_positions: List of (x, y) tuples
    :return: Function f(x) -> y, or None if fitting fails
    """
    if len(cone_positions) < 3:
        return None
    
    # Sort by X coordinate
    sorted_cones = sorted(cone_positions, key=lambda c: c[0])
    xs = np.array([c[0] for c in sorted_cones])
    ys = np.array([c[1] for c in sorted_cones])
    
    try:
        # Fit B-spline through trajectory
        tck, _ = splprep([xs, ys], s=0.1, k=min(3, len(xs)-1))
        
        def trajectory_func(x_val):
            # Use spline to predict Y at given X
            u_val = np.interp(x_val, xs, np.linspace(0, 1, len(xs)))
            xy = splev(u_val, tck)
            return xy[1]  # Return Y coordinate
        
        return trajectory_func
    except:
        return None


def _calculate_trajectory_deviation(cone, same_color_cones):
    """
    Calculate how far a cone deviates from its color group's trajectory.
    Only applies to curved tracks (detects curvature first).
    
    :param cone: (x, y, color) tuple to test
    :param same_color_cones: List of same-color cones
    :return: Deviation distance in meters, or None if can't fit or track is straight
    """
    if len(same_color_cones) < 4:
        return None  # Need at least 4 cones to fit meaningful trajectory
    
    # Get cone positions without the candidate
    other_positions = [(c[0], c[1]) for c in same_color_cones if c != cone]
    
    if len(other_positions) < 4:
        return None
    
    # Check if track is actually curved (not just straight line with noise)
    ys = [p[1] for p in other_positions]
    y_variance = np.var(ys)
    if y_variance < 1.0:  # Straight track, skip trajectory check
        return None
    
    # Fit trajectory through other cones
    trajectory_func = _fit_trajectory_curve(other_positions)
    if trajectory_func is None:
        return None
    
    # Calculate expected Y at this cone's X
    try:
        expected_y = trajectory_func(cone[0])
        deviation = abs(cone[1] - expected_y)
        return deviation
    except:
        return None


def remove_ghost_cones(cone_data, max_neighbor_dist=6.0, min_neighbors=1, partner_x_tolerance=1.5, trajectory_deviation_tolerance=1.0):
    """
    Remove isolated or misaligned cones likely to be false positives ("ghosts").
    Uses trajectory-based detection to work on curved tracks.

    A cone is kept if:
    1. It has at least `min_neighbors` other cones within `max_neighbor_dist`
    2. (Only if mixed colors exist) It has opposite-color partner cones aligned on X-axis
    3. It follows the trajectory of same-color cones (deviation < tolerance)

    :param cone_data: List of tuples [(x, y, color), ...]
    :param max_neighbor_dist: Neighbor radius for validation
    :param min_neighbors: Minimum count of neighbors required to keep a cone
    :param partner_x_tolerance: Max X-distance to find opposite-color partner
    :param trajectory_deviation_tolerance: Max deviation from same-color trajectory in meters
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
        
        # Check 3: Must be aligned with same-color group
        if has_mixed_colors:
            same_color_cones = [c for c in cone_data if c[2] == cone[2]]
            
            if len(same_color_cones) > 3:
                # Look at nearby cones only (exclude distant outliers)
                nearby_cones = [c for c in same_color_cones if abs(c[0] - cone[0]) <= 10.0]
                
                if len(nearby_cones) > 2:
                    ys = [c[1] for c in nearby_cones]
                    y_variance = np.var(ys)
                    
                    if y_variance < 1.0:
                        # STRAIGHT TRACK: Check Y-alignment
                        avg_y = np.mean(ys)
                        if abs(cone[1] - avg_y) > 0.8:
                            removed_count += 1
                            print(f"[GHOST FILTER] Removed ghost cone at ({cone[0]:.2f}, {cone[1]:.2f}, '{cone[2]}') - Y-misaligned (cone Y={cone[1]:.2f}, group avg Y={avg_y:.2f})")
                            continue
                    else:
                        # CURVED TRACK: Check trajectory deviation
                        deviation = _calculate_trajectory_deviation(cone, nearby_cones)
                        
                        if deviation is not None and deviation > trajectory_deviation_tolerance:
                            removed_count += 1
                            print(f"[GHOST FILTER] Removed ghost cone at ({cone[0]:.2f}, {cone[1]:.2f}, '{cone[2]}') - trajectory deviation {deviation:.2f}m > {trajectory_deviation_tolerance}m")
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