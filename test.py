# 1. Import the class we just created
from voronoi_path import VoronoiPath
import numpy as np

# --- Setup Dummy Data (Simulating Perception) ---
# Cone coordinates (imagine a slight curve)
cones_x = [0, 2, 5, 8, 1, 3, 6, 9] 
cones_y = [0, 0, 1, 2, 4, 4, 5, 6]
# 'b' for blue, 'y' for yellow
colors = ['b', 'b', 'b', 'b', 'y', 'y', 'y', 'y'] 

# --- Using the Class ---

# 2. Instantiate the object
# We pass the data, triggering the __init__ method in the other file
path_planner = VoronoiPath(cones_x, cones_y, colors)

# 3. Call the methods
print("\n--- Step 1: Generating Map ---")
path_planner.generate_map()

print("\n--- Step 2: Retrieving Calculations ---")
# Get the edges calculated inside the class
path_segments = path_planner.calculate_edges()

# 4. Display results
print(f"Found {len(path_segments)} valid path segments.")

# Print the first 3 segments to verify
for i, (start, end, dist) in enumerate(path_segments[:3]):
    # Formatting output for readability
    print(f"Segment {i+1}: Start={np.round(start, 2)}, End={np.round(end, 2)}, Length={dist:.2f}")