#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf.transformations import quaternion_from_matrix
from scipy.interpolate import CubicSpline
import csv
import os

def compute_orientation_from_fixed_direction(direction, up=np.array([0, 0, 1])):
    x_axis = direction / (np.linalg.norm(direction) + 1e-6)
    y_axis = np.cross(up, x_axis)
    y_axis /= (np.linalg.norm(y_axis) + 1e-6)
    z_axis = np.cross(x_axis, y_axis)
    z_axis /= (np.linalg.norm(z_axis) + 1e-6)

    R = np.eye(4)
    R[0:3, 0] = x_axis  
    R[0:3, 1] = y_axis  
    R[0:3, 2] = z_axis  
    return quaternion_from_matrix(R)


def generate_trajectory(waypoints, num_points=200, desired_speed=2.0):
    waypoints = np.array(waypoints)
    t = np.linspace(0, 1, len(waypoints))
    T = np.linspace(0, 1, num_points)

    x_spline = CubicSpline(t, waypoints[:, 0])
    y_spline = CubicSpline(t, waypoints[:, 1])
    z_spline = CubicSpline(t, waypoints[:, 2])

    trajectory = []
    for ti in T:
        # 位置
        pos = np.array([x_spline(ti), y_spline(ti), z_spline(ti)])

        # 切线方向（轨迹速度方向）
        dx = x_spline(ti, 1)
        dy = y_spline(ti, 1)
        dz = z_spline(ti, 1)
        direction = np.array([dx, dy, dz])
        norm = np.linalg.norm(direction) + 1e-6
        velocity = desired_speed * (direction / norm)

        # 姿态四元数
        quat = compute_orientation_from_fixed_direction(direction)

        # 位置、速度、姿态
        trajectory.append((pos, velocity, quat))
    return trajectory

def publish_path(trajectory):
    path_msg = Path()
    path_msg.header.frame_id = "world"
    for pos, _,quat in trajectory:
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = pos[2]
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        path_msg.poses.append(pose)
    return path_msg

def generate_path(num_points=150, desired_speed=2.0):
    waypoints = [
        [0,0,2.5],
        [-2, 8, 2.5],
        [9, 9, 2.5],
        [10, 0, 2.5],
        [8.0, -8.0, 2.5],
        [0, -10.0, 2.5],
        [-8, -7, 6.2],
        [-8, -7, 2.5],
    ]
    trajectory = generate_trajectory(waypoints, num_points=num_points, desired_speed=desired_speed)
    return trajectory


def save_trajectory_to_csv(trajectory, save_path="~/trajectory.csv"):
    save_path = os.path.expanduser(save_path)
    with open(save_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["x", "y", "z", "vx", "vy", "vz", "qx", "qy", "qz", "qw"])  # 表头
        for pos, vel, quat in trajectory:
            writer.writerow([
                pos[0], pos[1], pos[2],
                vel[0], vel[1], vel[2],
                quat[0], quat[1], quat[2], quat[3]
            ])
    rospy.loginfo(f"Trajectory saved to: {save_path}")


if __name__ == "__main__":
    waypoints = [
        [0,0,5],
        [-2, 8, 2.5],
        # [9, 9, 2.5],
        # [10, 0, 2.5],
        # [8.0, -8.0, 2.5],
        # [0, -10.0, 2.5],
        # [-8, -7, 6.2],
        # [-8, -7, 2.5],
        # [-8, -7, 6.2]
    ]
    trajectory = generate_trajectory(waypoints, num_points=150)
    # 保存到 CSV
    # save_trajectory_to_csv(trajectory, save_path="/home/jz/Documents/ads_fpv_ws/src/acados_nmpc_controller/source/path.csv")

    publish_path(trajectory)
