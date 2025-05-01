#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def plot_velocity_with_stats(df, save_path="/home/jz/Documents/ads_fpv_ws/src/acados_nmpc_controller/source/velocity_with_norm.png"):
    print("📈 正在绘制三轴速度及模长，并显示统计特性...")

    time = df["time"]
    vx, vy, vz = df["vel_x"], df["vel_y"], df["vel_z"]
    vel_norm = np.sqrt(vx**2 + vy**2 + vz**2)
    df["vel_norm"] = vel_norm

    plt.figure(figsize=(12, 6))
    colors = {
        "vel_x": "red",
        "vel_y": "green",
        "vel_z": "blue",
        "vel_norm": "black"
    }

    stats_text = "=== Velocity Statistics ===\n"
    for col in ["vel_x", "vel_y", "vel_z", "vel_norm"]:
        data = df[col]
        plt.plot(time, data, label=col, color=colors[col])
        stats_text += f"{col}: max={np.max(data):.2f}, mean={np.mean(data):.2f}, std={np.std(data):.2f}\n"

    # 在图左上角添加统计信息文本框
    plt.gca().text(0.02, 0.98, stats_text, transform=plt.gca().transAxes,
                   fontsize=9, verticalalignment='top', bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))

    plt.xlabel("Time (s)")
    plt.ylabel("Velocity (m/s)")
    plt.title("Velocity Components and Norm Over Time")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(save_path, dpi=300)
    print(f"✅ 图像已保存为: {save_path}")
    plt.show()

if __name__ == "__main__":
    df = pd.read_csv("/home/jz/Documents/ads_fpv_ws/src/acados_nmpc_controller/source/vel.csv")
    plot_velocity_with_stats(df)
