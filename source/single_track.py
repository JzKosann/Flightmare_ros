import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
from matplotlib import gridspec

# === 配置路径 ===
traj_path = "/home/jz/Documents/ads_fpv_ws/src/acados_nmpc_controller/source//single_trajectory.csv"
thrust_path = "/home/jz/Documents/ads_fpv_ws/src/acados_nmpc_controller/source//single_thrust.csv"
save_dir = "/home/jz/Documents/ads_fpv_ws/src/acados_nmpc_controller/source//single_trajectory_analysis"
os.makedirs(save_dir, exist_ok=True)

# === 读取数据 ===
df_traj = pd.read_csv(traj_path)
df_thrust = pd.read_csv(thrust_path)
target = np.array([-2, 8, 2.5])
positions = df_traj[["pos_x", "pos_y", "pos_z"]].values
errors = np.linalg.norm(positions - target, axis=1)

# === 单独保存每张图 ===
def plot_trajectory_3d():
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(df_traj["pos_x"], df_traj["pos_y"], df_traj["pos_z"], label='Trajectory')
    ax.scatter(df_traj["pos_x"].iloc[0], df_traj["pos_y"].iloc[0], df_traj["pos_z"].iloc[0], color='green', label='Start')
    ax.scatter(df_traj["pos_x"].iloc[-1], df_traj["pos_y"].iloc[-1], df_traj["pos_z"].iloc[-1], color='red', label='End')
    ax.scatter(*target, color='orange', label='Target')
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
    ax.set_title("3D Trajectory")
    ax.legend()
    return fig

def plot_target_error():
    fig = plt.figure(figsize=(8, 4))
    plt.plot(errors)
    plt.title("Distance to Target Over Time")
    plt.xlabel("Step"); plt.ylabel("Distance (m)")
    plt.tight_layout()
    return fig

def plot_quaternion():
    fig = plt.figure(figsize=(8, 4))
    plt.plot(df_traj["quat_x"], label="quat_x")
    plt.plot(df_traj["quat_y"], label="quat_y")
    plt.plot(df_traj["quat_z"], label="quat_z")
    plt.plot(df_traj["quat_w"], label="quat_w")
    plt.title("Quaternion Components Over Time")
    plt.xlabel("Step"); plt.ylabel("Quaternion Value")
    plt.legend(); plt.tight_layout()
    return fig

def plot_thrust():
    fig = plt.figure(figsize=(8, 4))
    for i in range(4):
        plt.plot(df_thrust[f"thrust{i+1}"], label=f"thrust{i+1}")
    plt.title("Thrust Forces Over Time")
    plt.xlabel("Step"); plt.ylabel("Thrust")
    plt.legend(); plt.tight_layout()
    return fig

# 保存图像
plot_trajectory_3d().savefig(os.path.join(save_dir, "trajectory_3d.png"))
plt.close()
plot_target_error().savefig(os.path.join(save_dir, "target_error.png"))
plt.close()
plot_quaternion().savefig(os.path.join(save_dir, "quat_error.png"))
plt.close()
plot_thrust().savefig(os.path.join(save_dir, "thrust_components.png"))
plt.close()


# === 仅显示位置轨迹的四图窗口（3D + 三视图） ===
fig = plt.figure(figsize=(14, 12))
gs = gridspec.GridSpec(1,1)

# 原始轨迹数据
x = df_traj["pos_x"]
y = df_traj["pos_y"]
z = df_traj["pos_z"]

# 1. 3D 轨迹图
ax1 = fig.add_subplot(gs[0, 0], projection='3d')
ax1.plot(x, y, z, label='Trajectory')
ax1.scatter(x.iloc[0], y.iloc[0], z.iloc[0], color='green', label='Start')
ax1.scatter(x.iloc[-1], y.iloc[-1], z.iloc[-1], color='red', label='End')
ax1.scatter(*target, color='orange', label='Target')
ax1.set_title("3D Trajectory")
ax1.set_xlabel("X"); ax1.set_ylabel("Y"); ax1.set_zlabel("Z")
ax1.legend()

# # 2. XY 平面投影（俯视图）
# ax2 = fig.add_subplot(gs[0, 1])
# ax2.plot(x, y, label="XY View")
# ax2.set_xlabel("X"); ax2.set_ylabel("Y")
# ax2.set_title("Top View (XY)")
# ax2.grid(True)

# # 3. XZ 平面投影（侧视图）
# ax3 = fig.add_subplot(gs[1, 0])
# ax3.plot(x, z, label="XZ View")
# ax3.set_xlabel("X"); ax3.set_ylabel("Z")
# ax3.set_title("Side View (XZ)")
# ax3.grid(True)

# # 4. YZ 平面投影（前视图）
# ax4 = fig.add_subplot(gs[1, 1])
# ax4.plot(y, z, label="YZ View")
# ax4.set_xlabel("Y"); ax4.set_ylabel("Z")
# ax4.set_title("Front View (YZ)")
# ax4.grid(True)

plt.tight_layout()
plt.show()
# 保存目录
view_save_dir = save_dir

# 1. 3D Trajectory
fig = plt.figure(figsize=(6, 5))
ax = fig.add_subplot(111, projection='3d')
ax.plot(x, y, z, label='Trajectory')
ax.scatter(x.iloc[0], y.iloc[0], z.iloc[0], color='green', label='Start')
ax.scatter(x.iloc[-1], y.iloc[-1], z.iloc[-1], color='red', label='End')
ax.scatter(*target, color='orange', label='Target')
ax.set_title("3D Trajectory")
ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
ax.legend()
fig.tight_layout()
fig.savefig(os.path.join(view_save_dir, "view_3d.png"))
plt.close(fig)

# 2. Top View (XY)
fig = plt.figure(figsize=(6, 4))
plt.plot(x, y)
plt.title("Top View (XY)")
plt.xlabel("X"); plt.ylabel("Y")
plt.grid(True)
fig.tight_layout()
fig.savefig(os.path.join(view_save_dir, "view_xy.png"))
plt.close(fig)

# 3. Side View (XZ)
fig = plt.figure(figsize=(6, 4))
plt.plot(x, z)
plt.title("Side View (XZ)")
plt.xlabel("X"); plt.ylabel("Z")
plt.grid(True)
fig.tight_layout()
fig.savefig(os.path.join(view_save_dir, "view_xz.png"))
plt.close(fig)

# 4. Front View (YZ)
fig = plt.figure(figsize=(6, 4))
plt.plot(y, z)
plt.title("Front View (YZ)")
plt.xlabel("Y"); plt.ylabel("Z")
plt.grid(True)
fig.tight_layout()
fig.savefig(os.path.join(view_save_dir, "view_yz.png"))
plt.close(fig)



def plot_velocity():
    fig = plt.figure(figsize=(8, 4))
    plt.plot(df_traj["vel_x"], label="vx")
    plt.plot(df_traj["vel_y"], label="vy")
    plt.plot(df_traj["vel_z"], label="vz")
    plt.title("Velocity Components Over Time")
    plt.xlabel("Step"); plt.ylabel("Velocity (m/s)")
    plt.legend(); plt.tight_layout()
    return fig

plot_velocity().savefig(os.path.join(save_dir, "velocity_components.png"))
plt.close()
