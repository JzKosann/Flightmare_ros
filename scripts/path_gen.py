#!/usr/bin/env python3
import numpy as np
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_matrix

import numpy as np
from scipy.spatial.transform import Rotation as R

gates = [
    {"pos": [-10, 10, 2.5], "quat": R.from_quat([0.0, 0.0, np.sin(np.pi/4), np.cos(np.pi/4)])},
    {"pos": [-10, -10, 2.5], "quat": R.from_quat([0.0, 0.0, np.sin(0.33*np.pi/4), np.cos(0.33*np.pi/4)])},
    {"pos": [0, -10, 2.5], "quat": R.from_quat([0.0, 0.0, 0.0, 1.0])},
    {"pos": [10, -10, 2.5], "quat": R.from_quat([0.0, 0.0, np.cos(1.25*np.pi/4), np.cos(1.25*np.pi/4)])},
    {"pos": [10, 0, 2.5], "quat": R.from_quat([0.0, 0.0, np.sin(np.pi/4), np.cos(np.pi/4)])}
]
from scipy.spatial.transform import Slerp, Rotation as R

# t: [0, 1]，插值时间
key_times = [0, 1]
key_rots = R.concatenate([quat_start, quat_end])
slerp = Slerp(key_times, key_rots)
interp_quat = slerp([0.25, 0.5, 0.75])  # 得到中间姿态


def compute_orientation_from_velocity(velocity, up=np.array([0, 0, 1])):
    x_axis = velocity / (np.linalg.norm(velocity) + 1e-6)
    y_axis = np.cross(up, x_axis)
    y_axis /= (np.linalg.norm(y_axis) + 1e-6)
    z_axis = np.cross(x_axis, y_axis)
    z_axis /= (np.linalg.norm(z_axis) + 1e-6)

    R = np.eye(4)
    R[0:3, 0] = x_axis
    R[0:3, 1] = y_axis
    R[0:3, 2] = z_axis
    return quaternion_from_matrix(R)

def interpolate_minimum_snap(points, num_points=200):
    # 使用三次样条近似 minimum snap（可替换为多项式优化器）
    from scipy.interpolate import CubicSpline

    points = np.array(points)
    t = np.linspace(0, 1, len(points))
    T = np.linspace(0, 1, num_points)

    x_spline = CubicSpline(t, points[:, 0])
    y_spline = CubicSpline(t, points[:, 1])
    z_spline = CubicSpline(t, points[:, 2])

    trajectory = []
    for i in range(len(T)):
        pos = np.array([x_spline(T[i]), y_spline(T[i]), z_spline(T[i])])
        vel = np.array([x_spline(T[i], 1), y_spline(T[i], 1), z_spline(T[i], 1)])
        quat = compute_orientation_from_velocity(vel)

        trajectory.append((pos, quat))
    return trajectory

def publish_path(traj):
    pub = rospy.Publisher("/minimum_snap_path", Path, queue_size=10)
    rospy.init_node("minimum_snap_generator", anonymous=True)
    rospy.sleep(1.0)

    path_msg = Path()
    path_msg.header.frame_id = "world"
    path_msg.header.stamp = rospy.Time.now()

    for pos, quat in traj:
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

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        path_msg.header.stamp = rospy.Time.now()
        pub.publish(path_msg)
        rate.sleep()

if __name__ == "__main__":
    # 🏁 赛道门坐标作为轨迹点（从 C++ 中转换）
    waypoints = [
        [-10.0, 10.0, 2.5],
        [-10.0, -10.0, 2.5],
        [0.0, -10.0, 2.5],
        [10.0, -10.0, 2.5],
        [10.0, 0.0, 2.5]
    ]
    trajectory = interpolate_minimum_snap(waypoints, num_points=150)
    publish_path(trajectory)
