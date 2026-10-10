#!/usr/bin/env python3
"""
Nadir Precision Landing Filter Benchmark Suite.

Executes a comparative Monte Carlo evaluation across 30 seeded descent trajectories,
benchmarking sensor noise rejection and touchdown centering accuracy across:
1. Unfiltered Raw PnP Baseline
2. Exponential Smoothing Filter (alpha=0.35, default demo profile)
3. 3D Kalman Filter / EKF (Kinematic observer with velocity estimation)

Generates:
- benchmarks/sitl_monte_carlo_results.csv
- benchmarks/landing_accuracy_distribution.png
"""

import os
import sys
import csv
import math
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from nadir.perception.pose.filtering import ExpSmoother, KalmanFilter3D

# Attempt to import compiled C++ EKF pybind11 core; fall back to Python reference
try:
    from nadir_ekf import ExtendedKalmanFilter as CppEKF  # type: ignore
    HAS_CPP_EKF = True
except ImportError:
    HAS_CPP_EKF = False


def simulate_descent_trajectory(
    filter_type: str,
    n_frames: int = 150,
    dt: float = 0.033,  # 30 Hz camera
    wind_drift_mps: float = 0.0,
    seed: int = 42,
) -> tuple[float, float, float, float]:
    """
    Simulates drone descent from 10m AGL to touchdown (0.3m).
    Returns (touchdown_radial_error_m, mean_jitter_cm, final_x_m, final_y_m).
    """
    np.random.seed(seed)

    # Physical trajectory: drone approaches from (x=3.0, y=2.0) towards pad at (0, 0)
    t = np.linspace(0, n_frames * dt, n_frames)
    
    # Closed-loop precision landing convergence to pad center (0, 0)
    decay = np.exp(-4.5 * t / t[-1])
    true_x = 3.0 * decay + (wind_drift_mps * 0.022)
    true_y = 2.0 * decay + (wind_drift_mps * 0.018)
    true_z = 10.0 * (1 - t / t[-1]) + 0.3

    # Camera perspective noise increases with altitude (PnP depth ambiguity)
    pnp_noise_std = 0.02 + 0.012 * true_z  # ~0.14m at 10m, ~0.024m at touchdown

    meas_x = true_x + np.random.normal(0, pnp_noise_std)
    meas_y = true_y + np.random.normal(0, pnp_noise_std)
    meas_z = true_z + np.random.normal(0, pnp_noise_std * 1.5)

    if filter_type == "raw":
        est_x = meas_x
        est_y = meas_y
    elif filter_type == "exp_smooth":
        smoother = ExpSmoother(alpha=0.35)
        est_x, est_y = [], []
        for i in range(n_frames):
            res = smoother.update(np.array([meas_x[i], meas_y[i], meas_z[i]]), timestamp=t[i])
            est_x.append(res.position[0])
            est_y.append(res.position[1])
        est_x = np.array(est_x)
        est_y = np.array(est_y)
    elif filter_type == "ekf":
        if HAS_CPP_EKF:
            ekf = CppEKF(dt=dt, process_noise=0.08, measurement_noise=0.35)
            est_x, est_y = [], []
            for i in range(n_frames):
                ekf.predict()
                pos = ekf.update([float(meas_x[i]), float(meas_y[i]), float(meas_z[i])])
                est_x.append(pos[0])
                est_y.append(pos[1])
            est_x = np.array(est_x)
            est_y = np.array(est_y)
        else:
            # High-fidelity Python KalmanFilter3D observer fallback
            kf = KalmanFilter3D(process_noise=0.08, measurement_noise=0.35, dt=dt)
            est_x, est_y = [], []
            for i in range(n_frames):
                res = kf.update(np.array([meas_x[i], meas_y[i], meas_z[i]]), dt=dt)
                est_x.append(res.position[0])
                est_y.append(res.position[1])
            est_x = np.array(est_x)
            est_y = np.array(est_y)
    else:
        raise ValueError(f"Unknown filter_type: {filter_type}")

    # Final touchdown error (distance from target center at 0, 0)
    final_x = est_x[-1]
    final_y = est_y[-1]
    radial_err = math.hypot(final_x, final_y)

    # Jitter during hover/terminal descent (standard deviation of residuals)
    jitter_cm = float(np.std(np.hypot(est_x[100:] - true_x[100:], est_y[100:] - true_y[100:])) * 100)

    return radial_err, jitter_cm, final_x, final_y


def run_benchmark_suite(n_runs: int = 30, output_dir: str = "benchmarks"):
    os.makedirs(output_dir, exist_ok=True)
    results = []

    # Varying wind conditions across runs: 0.0 m/s (calm), 1.5 m/s (moderate), 3.0 m/s (gusting)
    wind_conditions = [0.0] * 10 + [1.5] * 12 + [3.0] * 8

    backend_str = "Compiled C++ pybind11" if HAS_CPP_EKF else "Python KalmanFilter3D Reference"
    print(f"Benchmarking Nadir Pose Filtering Architectures ({n_runs} Seeded Trajectories)...")
    print(f"EKF Backend: {backend_str}")

    for run_id in range(1, n_runs + 1):
        wind = wind_conditions[(run_id - 1) % len(wind_conditions)]
        seed = 1000 + run_id

        # 1. Raw PnP Baseline
        raw_err, raw_jit, rx, ry = simulate_descent_trajectory("raw", wind_drift_mps=wind, seed=seed)
        results.append({
            "run_id": run_id,
            "filter": "Raw PnP (Unfiltered)",
            "wind_speed_mps": wind,
            "radial_error_m": round(raw_err, 4),
            "jitter_cm": round(raw_jit, 2),
            "error_x_m": round(rx, 4),
            "error_y_m": round(ry, 4),
        })

        # 2. Exp-Smooth (Default Demo Config)
        exp_err, exp_jit, ex, ey = simulate_descent_trajectory("exp_smooth", wind_drift_mps=wind, seed=seed)
        results.append({
            "run_id": run_id,
            "filter": "Exp-Smooth (alpha=0.35)",
            "wind_speed_mps": wind,
            "radial_error_m": round(exp_err, 4),
            "jitter_cm": round(exp_jit, 2),
            "error_x_m": round(ex, 4),
            "error_y_m": round(ey, 4),
        })

        # 3. Kalman Filter / EKF Observer
        ekf_err, ekf_jit, kx, ky = simulate_descent_trajectory("ekf", wind_drift_mps=wind, seed=seed)
        results.append({
            "run_id": run_id,
            "filter": "C++ EKF Observer",
            "wind_speed_mps": wind,
            "radial_error_m": round(ekf_err, 4),
            "jitter_cm": round(ekf_jit, 2),
            "error_x_m": round(kx, 4),
            "error_y_m": round(ky, 4),
        })

    csv_path = os.path.join(output_dir, "sitl_monte_carlo_results.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(results[0].keys()))
        writer.writeheader()
        writer.writerows(results)
    print(f"Saved benchmark results to {csv_path}")

    # Summary Statistics
    summary = {}
    for ftype in ["Raw PnP (Unfiltered)", "Exp-Smooth (alpha=0.35)", "C++ EKF Observer"]:
        sub = np.array([r["radial_error_m"] for r in results if r["filter"] == ftype])
        jit = np.array([r["jitter_cm"] for r in results if r["filter"] == ftype])
        mean_val = np.mean(sub)
        p95_val = np.percentile(sub, 95)
        # Bootstrap 95% Confidence Interval for the mean
        boot_means = [np.mean(np.random.choice(sub, size=len(sub), replace=True)) for _ in range(1000)]
        ci_low, ci_high = np.percentile(boot_means, [2.5, 97.5])

        summary[ftype] = {
            "mean_cm": mean_val * 100,
            "ci_95": (ci_low * 100, ci_high * 100),
            "median_cm": np.median(sub) * 100,
            "p95_cm": p95_val * 100,
            "mean_jitter_cm": np.mean(jit),
        }
        print(f"\n--- {ftype} ---")
        print(f"  Mean Radial Error: {mean_val * 100:.1f} cm (95% CI: [{ci_low*100:.1f}, {ci_high*100:.1f}] cm)")
        print(f"  Median Radial Err: {np.median(sub) * 100:.1f} cm")
        print(f"  95th Percentile  : {p95_val * 100:.1f} cm")
        print(f"  Terminal Jitter  : {np.mean(jit):.1f} cm")

    # Generate Visualization Plot
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    # Plot 1: 2D Touchdown scatter comparing Exp-Smooth vs EKF
    pad_circle = plt.Circle((0, 0), 0.50, color='gray', alpha=0.15, label='Pad Boundary (1.0m)')
    gate_circle = plt.Circle((0, 0), 0.16, color='green', fill=False, linestyle='--', linewidth=2, label='16cm Precision Gate')
    ax1.add_patch(pad_circle)
    ax1.add_patch(gate_circle)

    sub_exp_x = [r["error_x_m"] for r in results if r["filter"] == "Exp-Smooth (alpha=0.35)"]
    sub_exp_y = [r["error_y_m"] for r in results if r["filter"] == "Exp-Smooth (alpha=0.35)"]
    sub_ekf_x = [r["error_x_m"] for r in results if r["filter"] == "C++ EKF Observer"]
    sub_ekf_y = [r["error_y_m"] for r in results if r["filter"] == "C++ EKF Observer"]

    ax1.scatter(sub_exp_x, sub_exp_y, color='#ff7f0e', alpha=0.8, s=45, label='Exp-Smooth (Demo Default)')
    ax1.scatter(sub_ekf_x, sub_ekf_y, color='#1f77b4', marker='^', alpha=0.9, s=55, label='C++ EKF Observer')

    ax1.axhline(0, color='black', linestyle=':', alpha=0.4)
    ax1.axvline(0, color='black', linestyle=':', alpha=0.4)
    ax1.set_xlim(-0.35, 0.35)
    ax1.set_ylim(-0.35, 0.35)
    ax1.set_xlabel("Lateral Error X (m)")
    ax1.set_ylabel("Lateral Error Y (m)")
    ax1.set_title("Touchdown Positions on Pad (N=30 Runs)")
    ax1.legend(loc="lower left", fontsize=8)
    ax1.grid(True, linestyle=":", alpha=0.6)

    # Plot 2: Cumulative Distribution Function (CDF) comparison
    colors = {"Raw PnP (Unfiltered)": "#d62728", "Exp-Smooth (alpha=0.35)": "#ff7f0e", "C++ EKF Observer": "#1f77b4"}
    for ftype, col in colors.items():
        errs = np.sort(np.array([r["radial_error_m"] for r in results if r["filter"] == ftype]) * 100)
        cdf = np.arange(1, len(errs) + 1) / len(errs) * 100
        ax2.plot(errs, cdf, label=f"{ftype} (p95: {summary[ftype]['p95_cm']:.1f}cm)", color=col, linewidth=2.2)

    ax2.axvline(16.0, color='black', linestyle='--', alpha=0.7, label='16 cm Target Threshold')
    ax2.set_xlabel("Radial Touchdown Error (cm)")
    ax2.set_ylabel("Cumulative Probability (%)")
    ax2.set_title("Cumulative Precision Distribution (Raw vs Exp-Smooth vs EKF)")
    ax2.set_xlim(0, 45)
    ax2.set_ylim(0, 105)
    ax2.grid(True, linestyle=":", alpha=0.6)
    ax2.legend(loc="lower right", fontsize=8)

    plt.tight_layout()
    plot_path = os.path.join(output_dir, "landing_accuracy_distribution.png")
    plt.savefig(plot_path, dpi=200)
    plt.close()
    print(f"Saved benchmark comparative plot to {plot_path}")


if __name__ == "__main__":
    run_benchmark_suite(n_runs=30)
