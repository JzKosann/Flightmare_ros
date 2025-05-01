#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fftfreq
import os

# === 推力随时间变化图 ===
def plot_control_inputs(df, save_path):
    print("📈 正在绘制推力随时间变化图...")
    plt.figure(figsize=(12, 6))
    time_steps = np.arange(len(df))

    for col in df.columns:
        data = df[col].values
        plt.plot(time_steps, data, label=col)
        max_val = np.max(data)
        avg_val = np.mean(data)
        plt.text(time_steps[-1], max_val, f"max={max_val:.2f}", fontsize=8, color='red')
        plt.text(time_steps[-1], avg_val, f"avg={avg_val:.2f}", fontsize=8, color='blue')

    plt.xlabel("Time Step")
    plt.ylabel("Thrust Value")
    plt.title("Motor Thrust Commands Over Time")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(save_path, dpi=300)
    print(f"✅ 推力时间图已保存为: {save_path}")
    plt.close()

# === FFT 频率响应图 ===
def plot_fft(df, sampling_rate, save_path):
    print("📉 正在绘制推力 FFT 频率响应图...")
    n = len(df)
    freqs = fftfreq(n, d=1.0 / sampling_rate)[:n // 2]

    plt.figure(figsize=(12, 6))
    for col in df.columns:
        signal = df[col].values
        fft_vals = fft(signal)
        fft_magnitude = 2.0 / n * np.abs(fft_vals[:n // 2])
        plt.plot(freqs, fft_magnitude, label=col)

    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Amplitude")
    plt.title("FFT of Motor Thrust Commands")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(save_path, dpi=300)
    print(f"✅ FFT 频率图已保存为: {save_path}")
    plt.close()

# === 推力变化率 Δthrust 图 ===
def plot_delta_u(df, save_path):
    print("📈 正在绘制推力变化率 Δthrust 图...")
    plt.figure(figsize=(12, 6))
    for col in df.columns:
        delta_u = np.diff(df[col].values)
        plt.plot(delta_u, label=f"Δ{col}")
    plt.xlabel("Time Step")
    plt.ylabel("Change in Thrust")
    plt.title("Change Rate of Motor Thrust (Δthrust)")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(save_path, dpi=300)
    print(f"✅ 推力变化率图已保存为: {save_path}")
    plt.close()

# === 推力对称性图（前后/左右）===
def plot_thrust_balance(df, save_path):
    print("📊 正在绘制推力对称性分析图...")
    if all(col in df.columns for col in ["thrust1", "thrust2", "thrust3", "thrust4"]):
        t1, t2, t3, t4 = df["thrust1"], df["thrust2"], df["thrust3"], df["thrust4"]
        balance1 = t1 - t3  # 前后差异
        balance2 = t2 - t4  # 左右差异

        plt.figure(figsize=(12, 4))
        plt.plot(balance1, label="thrust1 - thrust3 (Front-Back)")
        plt.plot(balance2, label="thrust2 - thrust4 (Left-Right)")
        plt.xlabel("Time Step")
        plt.ylabel("Thrust Difference")
        plt.title("Thrust Symmetry Analysis")
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(save_path, dpi=300)
        print(f"✅ 推力对称性图已保存为: {save_path}")
        plt.close()
    else:
        print("❌ 文件中不包含 thrust1~4，无法绘制推力对称性图")

# === 主程序入口 ===
if __name__ == "__main__":
    base_path = "/home/jz/Documents/ads_fpv_ws/src/acados_nmpc_controller/source"
    csv_path = os.path.join(base_path, "control_thrust.csv")

    # 图像保存路径
    plot_time = os.path.join(base_path, "control_input_plot.png")
    plot_fft_path = os.path.join(base_path, "control_input_fft.png")
    plot_delta = os.path.join(base_path, "control_input_delta.png")
    plot_balance = os.path.join(base_path, "thrust_balance.png")

    try:
        df = pd.read_csv(csv_path)
        assert not df.empty, "CSV 文件为空"
        # 仅保留 thrust1~thrust4 列
        df = df[["thrust1", "thrust2", "thrust3", "thrust4"]]
    except Exception as e:
        print(f"❌ 无法读取 CSV 文件: {e}")
        exit(1)

    # 生成全部图
    plot_control_inputs(df, plot_time)
    plot_fft(df, sampling_rate=100.0, save_path=plot_fft_path)
    plot_delta_u(df, plot_delta)
    plot_thrust_balance(df, plot_balance)
