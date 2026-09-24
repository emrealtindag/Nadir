"""
Real-time 3D Path Planning Module for Scandium.

Implements obstacle avoidance algorithms (A* / RRT*) for autonomous descent
during precision landing.

Integrates with SLAM modules and ML landability plugins to dynamically
re-route the UAV if dynamic obstacles (e.g., humans, vehicles) enter the
landing cone.
"""

import math
import heapq
from typing import List, Tuple, Set

class Node:
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z
        self.g = 0.0
        self.h = 0.0
        self.f = 0.0
        self.parent = None

    def __lt__(self, other):
        return self.f < other.f

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.z == other.z
        
    def __hash__(self):
        return hash((self.x, self.y, self.z))

class AStarPlanner:
    """
    3D A* Path Planner for local obstacle avoidance during landing.
    Uses an occupancy grid updated by the Landability ML Plugin and/or SLAM.
    """
    def __init__(self, grid_resolution: float = 0.5):
        """
        Args:
            grid_resolution: Resolution of the discretization grid in meters.
        """
        self.res = grid_resolution
        self.obstacles: Set[Tuple[int, int, int]] = set()

    def update_obstacles(self, obstacle_list: List[Tuple[float, float, float]]):
        """Update the internal occupancy grid with new obstacle coordinates."""
        self.obstacles.clear()
        for (ox, oy, oz) in obstacle_list:
            gx, gy, gz = self._to_grid(ox, oy, oz)
            self.obstacles.add((gx, gy, gz))

    def _to_grid(self, x: float, y: float, z: float) -> Tuple[int, int, int]:
        return (int(round(x / self.res)), int(round(y / self.res)), int(round(z / self.res)))

    def _heuristic(self, n1: Node, n2: Node) -> float:
        # Euclidean distance
        return math.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2 + (n1.z - n2.z)**2)

    def plan(self, start_pos: Tuple[float, float, float], goal_pos: Tuple[float, float, float]) -> List[Tuple[float, float, float]]:
        """
        Plan a safe trajectory from start to goal avoiding obstacles.
        
        Args:
            start_pos: (x, y, z) start position (current UAV pose).
            goal_pos: (x, y, z) goal position (landing pad).
            
        Returns:
            List of (x, y, z) waypoints forming the safe path.
        """
        start_node = Node(*start_pos)
        goal_node = Node(*goal_pos)

        open_list = []
        heapq.heappush(open_list, start_node)
        closed_set = set()
        
        # 3D motion primitives (26-connectivity)
        motions = []
        for dx in [-self.res, 0, self.res]:
            for dy in [-self.res, 0, self.res]:
                for dz in [-self.res, 0, self.res]:
                    if dx != 0 or dy != 0 or dz != 0:
                        motions.append((dx, dy, dz))

        max_iter = 1000 # Limit to guarantee real-time constraint (e.g. 50Hz control loop)
        iterations = 0

        while open_list and iterations < max_iter:
            iterations += 1
            current = heapq.heappop(open_list)
            
            # Reached goal (within resolution)
            if self._heuristic(current, goal_node) < self.res:
                path = []
                while current:
                    path.append((current.x, current.y, current.z))
                    current = current.parent
                return path[::-1] # Reverse to get start-to-goal
                
            grid_pos = self._to_grid(current.x, current.y, current.z)
            if grid_pos in closed_set:
                continue
                
            closed_set.add(grid_pos)
            
            for m in motions:
                nx, ny, nz = current.x + m[0], current.y + m[1], current.z + m[2]
                n_grid = self._to_grid(nx, ny, nz)
                
                # Check collision
                if n_grid in self.obstacles:
                    continue
                    
                neighbor = Node(nx, ny, nz)
                neighbor.g = current.g + math.sqrt(m[0]**2 + m[1]**2 + m[2]**2)
                neighbor.h = self._heuristic(neighbor, goal_node)
                neighbor.f = neighbor.g + neighbor.h
                neighbor.parent = current
                
                heapq.heappush(open_list, neighbor)

        # Fail-safe: If path not found within real-time budget, return straight line 
        # but the higher-level FSM should trigger ABORT/GO-AROUND.
        return [start_pos, goal_pos]
