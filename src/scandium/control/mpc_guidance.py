import numpy as np

class MPCGuidanceController:
    """
    Model Predictive Control (MPC) inspired Guidance System extracted from Intent-MPC concepts.
    Replaces basic threshold logic with continuous, physics-aware velocity setpoint generation.
    """
    def __init__(self, dt: float = 0.05):
        self.dt = dt
        # Tuning parameters derived from trackingController.cpp principles
        self.kp_pos = np.array([1.5, 1.5, 1.0]) 
        self.kd_vel = np.array([0.5, 0.5, 0.3])
        self.max_velocity = 2.0  # m/s
        self.max_acceleration = 1.0  # m/s^2
        
        self.prev_velocity = np.zeros(3)

    def calculate_optimal_velocity(self, current_pos: tuple, target_pos: tuple, current_vel: tuple) -> tuple:
        """
        Calculates the optimal velocity vector to reach the target smoothly.
        (x, y, z) format.
        """
        p_curr = np.array(current_pos)
        p_target = np.array(target_pos)
        v_curr = np.array(current_vel)
        
        # Position error
        err_pos = p_target - p_curr
        
        # PD Control Law (Simplified from full MPC horizon for Edge computing performance)
        desired_vel = self.kp_pos * err_pos - self.kd_vel * v_curr
        
        # Enforce kinematic constraints (Max Velocity)
        speed = np.linalg.norm(desired_vel)
        if speed > self.max_velocity:
            desired_vel = (desired_vel / speed) * self.max_velocity
            
        # Enforce kinematic constraints (Max Acceleration)
        accel = (desired_vel - self.prev_velocity) / self.dt
        for i in range(3):
            if abs(accel[i]) > self.max_acceleration:
                desired_vel[i] = self.prev_velocity[i] + np.sign(accel[i]) * self.max_acceleration * self.dt
                
        self.prev_velocity = desired_vel
        return tuple(desired_vel)
