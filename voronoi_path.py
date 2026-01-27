import numpy as np
from scipy.spatial import Voronoi

class VoronoiPath:
    def __init__(self, cones_x, cones_y, cone_colors):
        """
        Initialize the path planner with cone data.
        """
        # We combine X and Y into a single list of (x, y) coordinates
        # because scipy.spatial.Voronoi expects an array of points.
        self.points = list(zip(cones_x, cones_y))
        self.colors = cone_colors
        self.voronoi_diagram = None # Placeholder for the map
        
        print(f"--> Class Initialized with {len(self.points)} cones.")

    def generate_map(self):
        """
        Computes the Voronoi diagram based on the stored points.
        """
        if len(self.points) < 3:
            print("Not enough points to generate a Voronoi map.")
            return

        # The heavy lifting is done here
        self.voronoi_diagram = Voronoi(self.points)
        print("--> Voronoi Diagram generated successfully.")

    def calculate_edges(self):
        """
        Basic calculation: Extract the line segments (ridges) that form the path.
        Returns a list of start/end points for the path lines.
        """
        if self.voronoi_diagram is None:
            return "Error: Map not generated yet."

        ridge_points = self.voronoi_diagram.ridge_vertices
        valid_edges = []

        # Filter out infinite edges (indicated by -1 in scipy)
        for ridge in ridge_points:
            if -1 not in ridge:
                p1_idx = ridge[0]
                p2_idx = ridge[1]
                
                # Get the actual coordinates from the vertices list
                p1 = self.voronoi_diagram.vertices[p1_idx]
                p2 = self.voronoi_diagram.vertices[p2_idx]
                
                # Calculate distance (Length of the path segment)
                distance = np.linalg.norm(p1 - p2)
                
                valid_edges.append((p1, p2, distance))

        return valid_edges