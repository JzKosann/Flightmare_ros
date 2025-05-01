#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt

def plot_control_vs_error(df, save_path="control_vs_error.png"):
    print("📊 正在绘制控制输入与位置误差对比图...")

    time = df["time"]
    plt.figure(figsize=(12, 6))

    # 绘制误差曲线（黑色加粗）
    plt.plot(time, df["pos_error"], label="Position Error", color="black", linewidth=2)

    # 绘制四个推力通道（虚线）
    for col in ["thrust1", "thrust2", "thrust3", "thrust4"]:
        plt.plot(time, df[col], label=col, linestyle='--')

    plt.xlabel("Time (s)")
    plt.ylabel("Value")
    plt.title("Motor Thrust vs Position Error")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(save_path, dpi=300)
    print(f"✅ 对比图已保存为: {save_path}")
    plt.show()

if __name__ == "__main__":
    df = pd.read_csv("/home/jz/Documents/ads_fpv_ws/src/acados_nmpc_controller/source/control_thrust_pos_error.csv")
    plot_control_vs_error(df, save_path="/home/jz/Documents/ads_fpv_ws/src/acados_nmpc_controller/source/control_vs_error.png")
