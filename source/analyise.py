#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree

def load_data(path_file, traj_file):
    path_df = pd.read_csv(path_file)
    traj_df = pd.read_csv(traj_file)

    path_pos = path_df[["x", "y", "z"]].values
    traj_pos = traj_df[["x", "y", "z"]].values

    return path_pos, traj_pos

def compute_nearest_errors(path_pos, traj_pos):
    # 构建 KD 树，加速最近邻查询
    tree = cKDTree(path_pos)

    # 查询每个 traj_pos 在 path_pos 中的最近点及其距离
    distances, _ = tree.query(traj_pos, k=1)
    return distances

def plot_error_heatmap(traj_pos, errors):
    x, y, z = traj_pos[:, 0], traj_pos[:, 1], traj_pos[:, 2]

    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    p = ax.scatter(x, y, z, c=errors, cmap='hot', s=15)
    fig.colorbar(p, ax=ax, label='Position Error (m)')
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Trajectory Error Heatmap (Nearest Path Matching)")
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    path_file = "/home/jz/Documents/ads_fpv_ws/src/acados_nmpc_controller/source/path.csv"
    traj_file = "/home/jz/Documents/ads_fpv_ws/src/acados_nmpc_controller/source/trajectory.csv"

    path_pos, traj_pos = load_data(path_file, traj_file)
    errors = compute_nearest_errors(path_pos, traj_pos)
    plot_error_heatmap(traj_pos, errors)
