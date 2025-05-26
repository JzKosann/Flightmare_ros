#!/home/jz/anaconda3/envs/flightros_env/bin/python
#  coding :utf-8

import rospy
from mav_msgs.msg import QuadThrusts, QuadCurState
from controller import *
from path_gen import *
from std_msgs.msg import Float64
import os
import csv

quad_thrusts_pub = None
quad_state_sub = None
quad_path_pub = None
quad_twist_pub = None
nmpc_controller = NMPC_Controller()
quad_thrusts_msg = QuadThrusts()
quad_state = QuadCurState()
quad_traj_pub = None
current_path = Path()
quad_delay_pub = None
quad_pos_error_pub = None
is_get_state = False    # 是否获取实时状态

flight_mode = None      # 无人机飞行状态 『‘takeoff', 'trajectory'』

def normalize_quaternion(qw, qx, qy, qz):
    norm = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
    return qw/norm, qx/norm, qy/norm, qz/norm

def quad_Callback(data):
    global nmpc_controller,quad_thrusts_msg, quad_state, is_get_state
    msg = data
    msg.qw, msg.qx, msg.qy, msg.qz = normalize_quaternion(msg.qw,msg.qx,msg.qy,msg.qz)
    quad_state = msg
    # print("-------quadrotor_state-------")
    # print(data)
    is_get_state = True
    # print(w[1])

def ros_init():
    global quad_state_sub, quad_thrusts_pub, quad_path_pub, quad_twist_pub, quad_traj_pub, quad_delay_pub,quad_pos_error_pub 
    quad_thrusts_pub = rospy.Publisher('flightmare_control/thrusts', QuadThrusts, queue_size=1) # 发布控制信息
    quad_state_sub = rospy.Subscriber('flightmare_control/state_info', QuadCurState, quad_Callback, queue_size=1)
    quad_path_pub = rospy.Publisher('flightmare_control/desired_path', Path, queue_size= 10)

    quad_traj_pub = rospy.Publisher('flightmare_control/current_trajectory', Path, queue_size=10)
    quad_delay_pub = rospy.Publisher('flightmare_control/control_delay_ms', Float64, queue_size=10)
    quad_pos_error_pub = rospy.Publisher('flightmare_control/control_pos_error', Float64, queue_size=10)
    # quad_twist_pub = rospy.Publisher('flightmare_control/desired_twist', TwistStamped, queue_size=1)



if __name__ == '__main__':
    
    rospy.init_node('acados_py',anonymous=True)
    ros_init()
    rate = rospy.Rate(100)
    trajectory = generate_path()
    quad_path_msg = publish_path(trajectory)


    target_reach_threshold = 1.5
    # index_ahead = 10
    index = 0
    last_tar_pos = np.array([0,0,0])

    last_time = rospy.get_rostime().to_sec()

    flight_mode = 'takeoff'         # 设定起飞模式
    hover_counts = 0                # 悬停计时
    hover_rate = rospy.Rate(1)     # hover_counts = 10 为悬停10s
    hover_reach_threshold = 0.5


    while not rospy.is_shutdown():

        pos_error = 0

        if is_get_state:
            # --- path generate --- #

            cur_state = np.array([quad_state.x, quad_state.y, quad_state.z,
                                quad_state.vx, quad_state.vy, quad_state.vz,
                                quad_state.qw, quad_state.qx, quad_state.qy, quad_state.qz,
                                #   quad_state.phi, quad_state.theta, quad_state.psi,
                                quad_state.p, quad_state.q, quad_state.r])
            cur_quat = np.array([quad_state.qx, quad_state.qy, quad_state.qz, quad_state.qw])
            cur_pos = np.array([quad_state.x, quad_state.y, quad_state.z])

            if flight_mode == 'takeoff': # 起飞模式
                hover_pos = np.array([0, 0, 5])
                hover_quat = np.array([0.50000,0,0,0.86603])
                
                target_pos = np.array([hover_pos[0], hover_pos[1], hover_pos[2],
                                    # tar_vel[0],[1],tar_vel[2],tar_vel[3]
                                    0,0,0,
                                    # tar_quat[3],tar_quat[0],tar_quat[1],tar_quat[2],
                                    hover_quat[0],hover_quat[1],hover_quat[2],hover_quat[3],
                                    0,0,0])
                if np.linalg.norm(cur_pos - hover_pos) < hover_reach_threshold:
                    hover_counts += 1

                if hover_counts >= 500:
                    flight_mode = 'trajectory'
                    hover_counts = 0
                
            elif flight_mode == 'trajectory': # 追踪轨迹
                # distances = [np.linalg.norm(np.array(pos)-cur_pos) for pos, _, _ in trajectory]
                # closest_idx = np.argmin(distances)
                # tar_pos, tar_vel, tar_quat = trajectory[min(closest_idx + 5, len(trajectory)-1)]

                # 目标点和当前点小于阈值 轨迹序列加1
                pos_error = np.linalg.norm(cur_pos - last_tar_pos)                
                pos_error_msg = Float64()
                pos_error_msg.data = pos_error
                quad_pos_error_pub.publish(pos_error_msg)
                if pos_error < target_reach_threshold:
                    index += 1  
                index = min(index,len(trajectory)-1)
                tar_pos, tar_vel, tar_quat = trajectory[index]
                last_tar_pos = np.array([tar_pos[0], tar_pos[1], tar_pos[2]]) # 存储为np矩阵

                print(f"Tracking Point {index}: {tar_pos}")
                
                if np.dot(cur_quat, tar_quat) < 0: # 解决四元数双覆盖问题
                    tar_quat=-tar_quat

                target_pos = np.array([tar_pos[0],tar_pos[1],tar_pos[2],
                                    # tar_vel[0],tar_vel[1],tar_vel[2],
                                    0,0,0,
                                    tar_quat[3],tar_quat[0],tar_quat[1],tar_quat[2],
                                    # 1,0,0,0,
                                    0,0,0])
                target_pos = np.array([-2.0,8.0,2.5,
                                    # tar_vel[0],tar_vel[1],tar_vel[2],
                                    0,0,0,
                                    0.9855, 0.0, 0.0, -0.1690,
                                    # 1,0,0,0,
                                    0,0,0])
                 # --- 发布当前轨迹 --- #
                # current_pose = PoseStamped()
                # current_pose.header.stamp = rospy.Time.now()
                # current_pose.header.frame_id = "world"
                # current_pose.pose.position.x = quad_state.x
                # current_pose.pose.position.y = quad_state.y
                # current_pose.pose.position.z = quad_state.z
                # current_pose.pose.orientation.x = quad_state.qx
                # current_pose.pose.orientation.y = quad_state.qy
                # current_pose.pose.orientation.z = quad_state.qz
                # current_pose.pose.orientation.w = quad_state.qw

                # current_path.header.stamp = rospy.Time.now()
                # current_path.header.frame_id = "world"
                # current_path.poses.append(current_pose)
                # quad_traj_pub.publish(current_path)
                # === 记录轨迹点到内存 ===
                log_time = rospy.Time.now().to_sec()
                single_track_pos_tar = np.array([-2,8,2.5])
                single_pos_error = np.linalg.norm(single_track_pos_tar-cur_pos)
                single_quat_error = np.linalg.norm(cur_quat-np.array([0.9855, 0.0, 0.0, -0.1690]))

            else:
                pass

            # --- quadrotor control --- #
            _dt, w, x_opt_acados = nmpc_controller.nmpc_state_control(current_state=cur_state, target_state=target_pos)
            quad_thrusts_msg.thrusts_1 = w[0]
            quad_thrusts_msg.thrusts_2 = w[1]
            quad_thrusts_msg.thrusts_3 = w[2]
            quad_thrusts_msg.thrusts_4 = w[3]
            quad_thrusts_pub.publish(quad_thrusts_msg)

            current_time = rospy.get_rostime().to_sec()
            print(f"Thrust:[{w[0]},{w[1]},{w[2]},{w[3]}]")
            print(f"delay {(current_time - last_time)*1000}ms")
            print(f"flight_mode: {flight_mode}, {hover_counts}")
            delay_msg=Float64()
            delay_msg.data = (current_time - last_time)*1000
            quad_delay_pub.publish(delay_msg)
            last_time = current_time
            is_get_state = False
        else:
            quad_thrusts_pub.publish(quad_thrusts_msg)
        quad_path_msg.header.stamp = rospy.Time.now()
        quad_path_pub.publish(quad_path_msg)
        
        # test_print()
        # rate.sleep()
