import csv
import rospy
import rosbag
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from rospy.rostime import Time

bag = rosbag.Bag('/home/jz/Documents/flightmare_rl/flightrl/examples/nmpc_trajectory.bag', 'w')
path_msg = Path()
path_msg.header.frame_id = "world"
topic_name = '/flightmare_rl/nmpc_drone_path'

with open('/home/jz/Documents/ads_fpv_ws/src/acados_nmpc_controller/source/single_trajectory.csv', newline='') as csvfile:
    reader = csv.DictReader(csvfile)
    t = 0
    for row in reader:
        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = Time.from_sec(t * 0.01)  # 每步间隔0.01秒
        pose.header.frame_id = "world"
        pose.pose.position.x = float(row['pos_x'])
        pose.pose.position.y = float(row['pos_y'])
        pose.pose.position.z = float(row['pos_z'])
        pose.pose.orientation.x = float(row['quat_x'])
        pose.pose.orientation.y = float(row['quat_y'])
        pose.pose.orientation.z = float(row['quat_z'])
        pose.pose.orientation.w = float(row['quat_w'])
        path_msg.poses.append(pose)

        #
        bag.write(topic_name, path_msg, pose.header.stamp)
        t += 1

bag.close()
