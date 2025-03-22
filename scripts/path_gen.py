#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_matrix
from scipy.interpolate import CubicSpline

# 🧭 根据速度向量计算“朝前”飞行姿态（四元数）
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

# 📈 使用三次样条生成近似 minimum snap 轨迹 + 姿态（可扩展为多项式优化器）
def generate_trajectory(waypoints, num_points=200):
    waypoints = np.array(waypoints)
    t = np.linspace(0, 1, len(waypoints))
    T = np.linspace(0, 1, num_points)

    x_spline = CubicSpline(t, waypoints[:, 0])
    y_spline = CubicSpline(t, waypoints[:, 1])
    z_spline = CubicSpline(t, waypoints[:, 2])

    trajectory = []
    for ti in T:
        pos = np.array([x_spline(ti), y_spline(ti), z_spline(ti)])
        vel = np.array([x_spline(ti, 1), y_spline(ti, 1), z_spline(ti, 1)])
        quat = compute_orientation_from_velocity(vel)
        trajectory.append((pos, quat))
    return trajectory

# 🚀 发布 ROS Path 消息（循环发布）
def publish_path(trajectory):
    rospy.init_node("minimum_snap_path_publisher", anonymous=True)
    pub = rospy.Publisher("/minimum_snap_path", Path, queue_size=10)
    rospy.sleep(1.0)

    path_msg = Path()
    path_msg.header.frame_id = "world"

    for pos, quat in trajectory:
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

    rate = rospy.Rate(1)  # 1 Hz 发布
    while not rospy.is_shutdown():
        path_msg.header.stamp = rospy.Time.now()
        pub.publish(path_msg)
        print("sent!")
        rate.sleep()

# 🏁 主函数：定义赛道门 + 执行轨迹生成与发布
if __name__ == "__main__":
    waypoints = [
        [-10.0, 10.0, 2.5],
        [-10.0, -10.0, 2.5],
        [0.0, -10.0, 2.5],
        [10.0, -10.0, 2.5],
        [10.0, 0.0, 2.5]
    ]
    trajectory = generate_trajectory(waypoints, num_points=150)
    publish_path(trajectory)
