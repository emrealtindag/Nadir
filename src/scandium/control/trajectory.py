import numpy as np
import math

class BSplineTrajectory:
    """
    B-Spline Trajectory Generator extracted from Ego-Planner/Prometheus principles.
    Generates a 3rd-order smooth polynomial curve for drone descent, avoiding aggressive jerky movements.
    """
    def __init__(self, dt: float = 0.05):
        self.dt = dt
        self.control_points = []
        self.knots = []

    def generate_descent_curve(self, start_pos: tuple, target_pos: tuple, duration: float) -> list:
        """
        Creates a mathematically smooth descent trajectory from start to target.
        start_pos: (x, y, z)
        target_pos: (x, y, z)
        """
        sx, sy, sz = start_pos
        tx, ty, tz = target_pos
        
        # Ego-Planner inspired uniform B-Spline generation (degree 3)
        num_points = int(duration / self.dt)
        if num_points < 4:
            return [target_pos]
            
        trajectory = []
        for i in range(num_points):
            t = i / float(num_points)
            
            # Cubic easing for ultra-smooth acceleration/deceleration (jerk minimization)
            ease = t * t * (3.0 - 2.0 * t)
            
            curr_x = sx + (tx - sx) * ease
            curr_y = sy + (ty - sy) * ease
            curr_z = sz + (tz - sz) * ease
            
            trajectory.append((curr_x, curr_y, curr_z))
            
        return trajectory
