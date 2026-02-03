# main_test.py
import matplotlib.pyplot as plt
import numpy as np
from modules.planner import PathPlanner

# Test 0: One-side only cones (all Yellow)
print("\n" + "="*60)
print("TEST 0: ONE-SIDE ONLY CONES (All Yellow)")
print("="*60)

cone_data_one_side = [
    # Right Side (Yellow) - 6 cones
    (0, 0, 'y'), (2, 0, 'y'), (4, 0, 'y'), (6, 0, 'y'), (8, 0, 'y'), (10, 0, 'y')
]

car_data_one_side = [(0.0, 1.5, 0.0)]

planner_one_side = PathPlanner(robot_radius=0.5, safety_margin=0.2, max_edge_len=5.0)
print(f"Planner config: robot_radius={planner_one_side.robot_radius}, safety_margin={planner_one_side.safety_margin}, max_edge_len={planner_one_side.max_edge_len}")
path_points_one_side = planner_one_side.execute_cycle(cone_data_one_side, car_data_one_side)

print(f"Path Generated with {len(path_points_one_side)} points.")

xs_one = [c[0] for c in cone_data_one_side]
ys_one = [c[1] for c in cone_data_one_side]
colors_one = ['gold' if c[2] == 'y' else 'blue' for c in cone_data_one_side]

plt.figure(figsize=(10, 6))
plt.scatter(xs_one, ys_one, c=colors_one, s=100, label='Cones', edgecolors='black', linewidth=2)
plt.plot(car_data_one_side[0][0], car_data_one_side[0][1], 'r^', markersize=15, label='Car Start')

if path_points_one_side:
    px_one = [p[0] for p in path_points_one_side]
    py_one = [p[1] for p in path_points_one_side]
    plt.plot(px_one, py_one, '-g', linewidth=2, label='Calculated Path')
    plt.scatter(px_one, py_one, c='green', s=20, alpha=0.5)

plt.title('Test 0: One-Side Only Cones (All Yellow)')
plt.legend()
plt.axis('equal')
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.show()

# Test 3: Ghost cone (isolated false positive)
print("\n" + "="*60)
print("TEST 3: GHOST CONE (Isolated false positive)")
print("="*60)

cone_data_ghost = [
    # Right Side (Yellow)
    (0, 0, 'y'), (2, 0, 'y'), (4, 0, 'y'), (6, 0, 'y'), (8, 0, 'y'), (10, 0, 'y'),
    # Left Side (Blue)
    (0, 3, 'b'), (2, 3, 'b'), (4, 3, 'b'), (6, 3, 'b'), (8, 3, 'b'), (10, 3, 'b'),
    # Ghost cone (isolated false positive)
    (25, 15, 'y')
]

car_data_ghost = [(0.0, 1.5, 0.0)]

planner_ghost = PathPlanner(
    robot_radius=0.5,
    safety_margin=0.2,
    max_edge_len=5.0,
    ghost_filter_enabled=True,
    ghost_max_neighbor_dist=4.0,
    ghost_min_neighbors=1
)
print(
    f"Planner config: robot_radius={planner_ghost.robot_radius}, safety_margin={planner_ghost.safety_margin}, "
    f"max_edge_len={planner_ghost.max_edge_len}, ghost_filter_enabled={planner_ghost.ghost_filter_enabled}"
)
path_points_ghost = planner_ghost.execute_cycle(cone_data_ghost, car_data_ghost)

print(f"Path Generated with {len(path_points_ghost)} points.")

xs_ghost = [c[0] for c in cone_data_ghost]
ys_ghost = [c[1] for c in cone_data_ghost]
colors_ghost = ['gold' if c[2] == 'y' else 'blue' for c in cone_data_ghost]

plt.figure(figsize=(10, 6))
plt.scatter(xs_ghost, ys_ghost, c=colors_ghost, s=100, label='Cones', edgecolors='black', linewidth=2)
plt.plot(car_data_ghost[0][0], car_data_ghost[0][1], 'r^', markersize=15, label='Car Start')

if path_points_ghost:
    px_ghost = [p[0] for p in path_points_ghost]
    py_ghost = [p[1] for p in path_points_ghost]
    plt.plot(px_ghost, py_ghost, '-g', linewidth=2, label='Calculated Path')
    plt.scatter(px_ghost, py_ghost, c='green', s=20, alpha=0.5)

plt.title('Test 3: Ghost Cone (Isolated False Positive)')
plt.legend()
plt.axis('equal')
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.show()

# Test 4: Ghost cone BETWEEN real cones (harder case)
print("\n" + "="*60)
print("TEST 4: GHOST CONE BETWEEN REAL CONES (Harder case)")
print("="*60)

cone_data_ghost_between = [
    # Right Side (Yellow)
    (0, 0, 'y'), (2, 0, 'y'), (4, 0, 'y'), (6, 0, 'y'), (8, 0, 'y'), (10, 0, 'y'),
    # Left Side (Blue)
    (0, 3, 'b'), (2, 3, 'b'), (4, 3, 'b'), (6, 3, 'b'), (8, 3, 'b'), (10, 3, 'b'),
    # Ghost cone BETWEEN real cones but misaligned (closer to middle, not paired)
    (5.2, 1.6, 'y')  # Slightly offset from the 5.0 line, creates false positive
]

car_data_ghost_between = [(0.0, 1.5, 0.0)]

planner_ghost_between = PathPlanner(
    robot_radius=0.5,
    safety_margin=0.2,
    max_edge_len=5.0,
    ghost_filter_enabled=True,
    ghost_max_neighbor_dist=4.0,
    ghost_min_neighbors=1
)
print(
    f"Planner config: robot_radius={planner_ghost_between.robot_radius}, safety_margin={planner_ghost_between.safety_margin}, "
    f"max_edge_len={planner_ghost_between.max_edge_len}, ghost_filter_enabled={planner_ghost_between.ghost_filter_enabled}"
)
path_points_ghost_between = planner_ghost_between.execute_cycle(cone_data_ghost_between, car_data_ghost_between)

print(f"Path Generated with {len(path_points_ghost_between)} points.")

xs_gb = [c[0] for c in cone_data_ghost_between]
ys_gb = [c[1] for c in cone_data_ghost_between]
colors_gb = ['gold' if c[2] == 'y' else 'blue' for c in cone_data_ghost_between]

plt.figure(figsize=(10, 6))
plt.scatter(xs_gb, ys_gb, c=colors_gb, s=100, label='Cones', edgecolors='black', linewidth=2)
plt.plot(car_data_ghost_between[0][0], car_data_ghost_between[0][1], 'r^', markersize=15, label='Car Start')

if path_points_ghost_between:
    px_gb = [p[0] for p in path_points_ghost_between]
    py_gb = [p[1] for p in path_points_ghost_between]
    plt.plot(px_gb, py_gb, '-g', linewidth=2, label='Calculated Path')
    plt.scatter(px_gb, py_gb, c='green', s=20, alpha=0.5)

plt.title('Test 4: Ghost Cone Between Real Cones (Harder case)')
plt.legend()
plt.axis('equal')
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.show()

# Test 5: Ghost cone on curved track
print("\n" + "="*60)
print("TEST 5: GHOST CONE ON CURVED TRACK")
print("="*60)

cone_data_curved = [
    # Right Side (Yellow) - curved path
    (0, 0, 'y'), (2, 0.3, 'y'), (4, 1.2, 'y'), (6, 2.1, 'y'), (8, 3.2, 'y'), (10, 4.1, 'y'),
    # Left Side (Blue) - curved path
    (0, 3, 'b'), (2, 3.3, 'b'), (4, 4.2, 'b'), (6, 5.1, 'b'), (8, 6.2, 'b'), (10, 7.1, 'b'),
    # Ghost cone off the curved trajectory
    (5.1, 2.5, 'y')  # Should be at ~1.65 following curve, but is at 2.5
]

car_data_curved = [(0.0, 1.5, 0.0)]

planner_curved = PathPlanner(
    robot_radius=0.5,
    safety_margin=0.2,
    max_edge_len=5.0,
    ghost_filter_enabled=True,
    ghost_trajectory_deviation_tolerance=1.0
)
print(
    f"Planner config: robot_radius={planner_curved.robot_radius}, safety_margin={planner_curved.safety_margin}, "
    f"max_edge_len={planner_curved.max_edge_len}, ghost_filter_enabled={planner_curved.ghost_filter_enabled}"
)
path_points_curved = planner_curved.execute_cycle(cone_data_curved, car_data_curved)

print(f"Path Generated with {len(path_points_curved)} points.")

xs_curved = [c[0] for c in cone_data_curved]
ys_curved = [c[1] for c in cone_data_curved]
colors_curved = ['gold' if c[2] == 'y' else 'blue' for c in cone_data_curved]

plt.figure(figsize=(10, 6))
plt.scatter(xs_curved, ys_curved, c=colors_curved, s=100, label='Cones', edgecolors='black', linewidth=2)
plt.plot(car_data_curved[0][0], car_data_curved[0][1], 'r^', markersize=15, label='Car Start')

if path_points_curved:
    px_curved = [p[0] for p in path_points_curved]
    py_curved = [p[1] for p in path_points_curved]
    plt.plot(px_curved, py_curved, '-g', linewidth=2, label='Calculated Path')
    plt.scatter(px_curved, py_curved, c='green', s=20, alpha=0.5)

plt.title('Test 5: Ghost Cone on Curved Track')
plt.legend()
plt.axis('equal')
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.show()

# Test 1: Balanced cones (original)
print("\n" + "="*60)
print("TEST 1: BALANCED CONES (6 Yellow, 6 Blue)")
print("="*60)

# --- 1. New Input Format ---
# List of tuples: [(x, y, color), ...]
cone_data = [
    # Right Side (Yellow)
    (0, 0, 'y'), (2, 0, 'y'), (4, 1, 'y'), (6, 1, 'y'), (8, 0, 'y'), (10, 0, 'y'),
    # Left Side (Blue)
    (0, 3, 'b'), (2, 3, 'b'), (4, 4, 'b'), (6, 4, 'b'), (8, 3, 'b'), (10, 3, 'b')
]
# Recommended Car Start: (0.0, 1.5, 0.0)


# Car input: [(x, y, orientation_in_radians)]
# Orientation 0.0 points East (Right)
car_data = [(0.0, 1.5, 0.0)]

# --- 2. Execution ---
planner = PathPlanner(robot_radius=0.5, safety_margin=0.2, max_edge_len=5.0)
print(f"Planner config: robot_radius={planner.robot_radius}, safety_margin={planner.safety_margin}, max_edge_len={planner.max_edge_len}")
path_points = planner.execute_cycle(cone_data, car_data)

# --- 3. Visualization ---
print(f"Path Generated with {len(path_points)} points.")

# Unpack data for plotting
xs = [c[0] for c in cone_data]
ys = [c[1] for c in cone_data]
colors = ['gold' if c[2] == 'y' else 'blue' for c in cone_data]

plt.figure(figsize=(10, 6))
plt.scatter(xs, ys, c=colors, s=100, label='Cones', edgecolors='black', linewidth=2)
plt.plot(car_data[0][0], car_data[0][1], 'r^', markersize=15, label='Car Start')

if path_points:
    px = [p[0] for p in path_points]
    py = [p[1] for p in path_points]
    plt.plot(px, py, '-g', linewidth=2, label='Calculated Path')
    plt.scatter(px, py, c='green', s=20, alpha=0.5)

plt.title('Test 1: Balanced Cones (6 Yellow, 6 Blue)')
plt.legend()
plt.axis('equal')
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.show()

# Test 2: Uneven cones (4 Yellow, 1 Blue)
print("\n" + "="*60)
print("TEST 2: SIMPLE UNEVEN CONES (4 Yellow, 1 Blue)")
print("="*60)

cone_data_simple = [
    # Right Side (Yellow) - 4 cones
    (0, 0, 'y'), (4, 0, 'y'), (8, 0, 'y'), (12, 0, 'y'),
    # Left Side (Blue) - 1 cone
    (6, 3, 'b')
]

car_data_simple = [(0.0, 1.5, 0.0)]

planner_simple = PathPlanner(
    robot_radius=0.5,
    safety_margin=0.2,
    max_edge_len=5.0,
    ghost_filter_enabled=False  # Disable for uneven cone scenario
)
print(f"Planner config: robot_radius={planner_simple.robot_radius}, safety_margin={planner_simple.safety_margin}, max_edge_len={planner_simple.max_edge_len}")
path_points_simple = planner_simple.execute_cycle(cone_data_simple, car_data_simple)

# --- 3. Visualization ---
print(f"Path Generated with {len(path_points_simple)} points.")

# Unpack data for plotting
xs_simple = [c[0] for c in cone_data_simple]
ys_simple = [c[1] for c in cone_data_simple]
colors_simple = ['gold' if c[2] == 'y' else 'blue' for c in cone_data_simple]

plt.figure(figsize=(10, 6))
plt.scatter(xs_simple, ys_simple, c=colors_simple, s=100, label='Original Cones', edgecolors='black', linewidth=2)
plt.plot(car_data_simple[0][0], car_data_simple[0][1], 'r^', markersize=15, label='Car Start')

if path_points_simple:
    px_simple = [p[0] for p in path_points_simple]
    py_simple = [p[1] for p in path_points_simple]
    plt.plot(px_simple, py_simple, '-g', linewidth=2, label='Calculated Path')
    plt.scatter(px_simple, py_simple, c='green', s=20, alpha=0.5)

plt.title('Test 2: Simple Uneven Cones - Virtual Cones Added (4 Yellow, 1 Blue)')
plt.legend()
plt.axis('equal')
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.show()