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

def compute_nearest_position_errors(path_pos, traj_pos):
    tree = cKDTree(path_pos)
    distances, indices = tree.query(traj_pos, k=1)
    matched_path_pos = path_pos[indices]
    return distances, matched_path_pos

def plot_trajectory_comparison(matched_path_pos, traj_pos, save_path):
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(matched_path_pos[:, 0], matched_path_pos[:, 1], matched_path_pos[:, 2],
            label='Matched Ground Truth (by Nearest)', linewidth=2, color='blue')
    ax.plot(traj_pos[:, 0], traj_pos[:, 1], traj_pos[:, 2],
            label='Actual Trajectory', linewidth=2, color='orange')

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Trajectory vs Ground Truth (Matched by Nearest Position)")
    ax.legend()
    plt.tight_layout()
    plt.savefig(save_path, dpi=300)
    print(f"✅ 对比图已保存为: {save_path}")
    plt.show()

def plot_error_curve(errors, save_path):
    plt.figure(figsize=(10, 4))
    plt.plot(errors, label="Nearest Position Error (m)", color="red")
    plt.xlabel("Time Step")
    plt.ylabel("Error (m)")
    plt.title("Position Error over Time (Nearest Matching)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(save_path, dpi=300)
    print(f"✅ 误差折线图已保存为: {save_path}")
    plt.show()

def print_error_stats(errors):
    print("📊 最近邻位置误差统计：")
    print(f"最大误差: {np.max(errors):.3f} m")
    print(f"最小误差: {np.min(errors):.3f} m")
    print(f"平均误差: {np.mean(errors):.3f} m")

def plot_error_heatmap(traj_pos, errors, save_path):
    x, y, z = traj_pos[:, 0], traj_pos[:, 1], traj_pos[:, 2]
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    p = ax.scatter(x, y, z, c=errors, cmap='hot', s=15)
    fig.colorbar(p, ax=ax, label='Position Error (m)')
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Position Error Heatmap (Nearest Path Match)")
    plt.tight_layout()
    plt.savefig(save_path, dpi=300)
    print(f"✅ 热力图已保存为: {save_path}")
    plt.show()




if __name__ == "__main__":
    path_file = "/home/jz/Documents/ads_fpv_ws/src/acados_nmpc_controller/source/path.csv"
    traj_file = "/home/jz/Documents/ads_fpv_ws/src/acados_nmpc_controller/source/trajectory.csv"
    save_fig_path = "/home/jz/Documents/ads_fpv_ws/src/acados_nmpc_controller/source/trajectory_vs_path_nearest.png"
    save_err_plot = "/home/jz/Documents/ads_fpv_ws/src/acados_nmpc_controller/source/trajectory_error_nearest.png"

    path_pos, traj_pos = load_data(path_file, traj_file)
    errors, matched_path_pos = compute_nearest_position_errors(path_pos, traj_pos)

    print_error_stats(errors)
    plot_trajectory_comparison(matched_path_pos, traj_pos, save_fig_path)
    plot_error_curve(errors, save_err_plot)
    save_heatmap = "/home/jz/Documents/ads_fpv_ws/src/acados_nmpc_controller/source/trajectory_error_heatmap.png"

    # 生成热力图
    plot_error_heatmap(traj_pos, errors, save_heatmap)

