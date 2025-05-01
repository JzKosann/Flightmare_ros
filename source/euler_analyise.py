#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R

def load_data(path_file, traj_file):
    path_df = pd.read_csv(path_file)
    traj_df = pd.read_csv(traj_file)
    return path_df, traj_df

def compute_quaternion_rotation_error(path_df, traj_df):
    path_pos = path_df[["x", "y", "z"]].values
    traj_pos = traj_df[["x", "y", "z"]].values
    path_quat = path_df[["qx", "qy", "qz", "qw"]].values
    traj_quat = traj_df[["qx", "qy", "qz", "qw"]].values

    # 使用位置最近邻进行姿态匹配
    tree = cKDTree(path_pos)
    _, indices = tree.query(traj_pos, k=1)
    aligned_path_quat = path_quat[indices]

    # 计算 q_err = q_ref⁻¹ * q_actual
    r_ref = R.from_quat(aligned_path_quat)
    r_actual = R.from_quat(traj_quat)
    r_error = r_ref.inv() * r_actual

    # 提取旋转角（弧度）再转为角度
    angle_errors = r_error.magnitude() * (180 / np.pi)
    return traj_pos, angle_errors

def plot_error_heatmap(traj_pos, angle_errors, save_path="quat_attitude_heatmap.png"):
    x, y, z = traj_pos[:, 0], traj_pos[:, 1], traj_pos[:, 2]
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    p = ax.scatter(x, y, z, c=angle_errors, cmap='hot', s=15)
    fig.colorbar(p, ax=ax, label='Rotation Error (deg)')
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Attitude Error Heatmap (Quaternion Rotation Angle)")
    plt.tight_layout()
    plt.savefig(save_path, dpi=300)
    print(f"✅ 图像已保存为: {save_path}")
    plt.show()

def print_stats(angle_errors):
    print("✅ 姿态误差统计（四元数旋转角）:")
    print(f"最大误差: {np.max(angle_errors):.2f}°")
    print(f"平均误差: {np.mean(angle_errors):.2f}°")
    print(f"标准差: {np.std(angle_errors):.2f}°")

if __name__ == "__main__":
    path_df, traj_df = load_data("/home/jz/Documents/ads_fpv_ws/src/acados_nmpc_controller/source/path.csv", 
                                "/home/jz/Documents/ads_fpv_ws/src/acados_nmpc_controller/source/trajectory.csv")
    traj_pos, angle_errors = compute_quaternion_rotation_error(path_df, traj_df)
    plot_error_heatmap(traj_pos, angle_errors)
    print_stats(angle_errors)
