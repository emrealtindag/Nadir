import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import os

def generate_landing_gif():
    print("Generating EKF Landing Demo GIF...")
    
    # Generate simulated descent data
    frames = 100
    t = np.linspace(0, 10, frames)
    
    # True path (smooth descent to 0,0)
    true_x = 5.0 * np.exp(-0.3 * t) * np.cos(t)
    true_y = 5.0 * np.exp(-0.3 * t) * np.sin(t)
    
    # Noisy sensor data (PnP without filter)
    noise_x = np.random.normal(0, 0.8, frames)
    noise_y = np.random.normal(0, 0.8, frames)
    meas_x = true_x + noise_x
    meas_y = true_y + noise_y
    
    # EKF Filtered path (Exponential Smoothing mock for visualization)
    filt_x = np.zeros(frames)
    filt_y = np.zeros(frames)
    curr_x, curr_y = meas_x[0], meas_y[0]
    alpha = 0.25
    for i in range(frames):
        curr_x = (1 - alpha) * curr_x + alpha * meas_x[i]
        curr_y = (1 - alpha) * curr_y + alpha * meas_y[i]
        filt_x[i] = curr_x
        filt_y[i] = curr_y

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(-6, 6)
    ax.set_ylim(-6, 6)
    ax.set_title("Scandium EKF Precision Landing Simulation", fontsize=14, fontweight='bold')
    ax.set_xlabel("X Distance (meters)")
    ax.set_ylabel("Y Distance (meters)")
    ax.grid(True, linestyle='--', alpha=0.6)
    
    # Target center (ArUco marker)
    ax.plot(0, 0, 'kx', markersize=15, markeredgewidth=3, label="Target Marker")
    target_circle = plt.Circle((0, 0), 0.5, color='g', alpha=0.2, label="Landing Zone (0.5m)")
    ax.add_patch(target_circle)

    # Plot lines
    true_line, = ax.plot([], [], 'k--', alpha=0.5, label='True Trajectory')
    raw_scatter, = ax.plot([], [], 'ro', alpha=0.4, label='Raw Camera (Noisy)', markersize=6)
    filt_line, = ax.plot([], [], 'b-', linewidth=3, label='Scandium EKF')
    
    # Current positions
    drone_raw, = ax.plot([], [], 'rX', markersize=12)
    drone_ekf, = ax.plot([], [], 'bD', markersize=12)

    ax.legend(loc='upper right')

    def init():
        true_line.set_data([], [])
        raw_scatter.set_data([], [])
        filt_line.set_data([], [])
        drone_raw.set_data([], [])
        drone_ekf.set_data([], [])
        return true_line, raw_scatter, filt_line, drone_raw, drone_ekf

    def update(frame):
        true_line.set_data(true_x[:frame], true_y[:frame])
        raw_scatter.set_data(meas_x[:frame], meas_y[:frame])
        filt_line.set_data(filt_x[:frame], filt_y[:frame])
        
        # We must pass sequences to set_data (like [x], [y]) instead of a single scalar
        drone_raw.set_data([meas_x[frame]], [meas_y[frame]])
        drone_ekf.set_data([filt_x[frame]], [filt_y[frame]])
        
        return true_line, raw_scatter, filt_line, drone_raw, drone_ekf

    ani = FuncAnimation(fig, update, frames=frames, init_func=init, blit=True)
    
    out_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'benchmarks')
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, 'landing_demo.gif')
    
    ani.save(out_path, writer=PillowWriter(fps=15))
    print(f"GIF successfully saved to {out_path}")

if __name__ == "__main__":
    generate_landing_gif()
