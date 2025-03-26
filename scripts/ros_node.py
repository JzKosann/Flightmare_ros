#!/home/jz/anaconda3/envs/flightros_env/bin/python
#  coding :utf-8

import rospy
from mav_msgs.msg import QuadThrusts, QuadCurState
from controller import *
from path_gen import *

quad_thrusts_pub = None
quad_state_sub = None
quad_path_pub = None
nmpc_controller = NMPC_Controller()
quad_thrusts_msg = QuadThrusts()
quad_state = QuadCurState()

is_get_state = False    # 是否获取状态

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
    global quad_state_sub, quad_thrusts_pub, quad_path_pub
    quad_thrusts_pub = rospy.Publisher('flightmare_control/thrusts', QuadThrusts, queue_size=1) # 发布控制信息
    quad_state_sub = rospy.Subscriber('flightmare_control/state_info', QuadCurState, quad_Callback, queue_size=1)
    quad_path_pub = rospy.Publisher('flightmare_control/desired_path', Path, queue_size= 10)



if __name__ == '__main__':
    
    rospy.init_node('acados_py',anonymous=True)
    ros_init()
    rate = rospy.Rate(100)
    trajectory = generate_path()
    quad_path_msg = publish_path(trajectory)
    target_reach_threshold = 0.3
    # index_ahead = 10
    index = 0
    last_time = rospy.get_rostime().to_sec()
    while not rospy.is_shutdown():

        if is_get_state:
            # --- path generate --- #

            cur_state = np.array([quad_state.x, quad_state.y, quad_state.z,
                                quad_state.vx, quad_state.vy, quad_state.vz,
                                quad_state.qw, quad_state.qx, quad_state.qy, quad_state.qz,
                                #   quad_state.phi, quad_state.theta, quad_state.psi,
                                quad_state.p, quad_state.q, quad_state.r])
            cur_quat = np.array([quad_state.qw, quad_state.qx, quad_state.qy, quad_state.qz])
            cur_pos = np.array([quad_state.x, quad_state.y, quad_state.z])

            distances = [np.linalg.norm(np.array(pos)-cur_pos) for pos, _, _ in trajectory]
            closest_idx = np.argmin(distances)
            tar_pos, tar_vel, tar_quat = trajectory[min(closest_idx , len(trajectory)-1)]
            print(f"Tracking Point {index}: {tar_pos}")
            target_pos = np.array([tar_pos[0],tar_pos[1],tar_pos[2],
                                # tar_vel[0],tar_vel[1],tar_vel[2],
                                0,0,0,
                                tar_quat[3],tar_quat[0],tar_quat[1],tar_quat[2],
                                # 1,0,0,0,
                                0,0,0])
            # tar_quat_test = np.array([0, 0, 0, 1])
            # # if np.dot(cur_quat, tar_quat_test) < 0:
            # #     tar_quat_test = -tar_quat_test
        
            # target_pos = np.array([1,-1,2.5,
            #                     # tar_vel[0],tar_vel[1],tar_vel[2],
            #                     0,0,0,
            #                     # tar_quat[3],tar_quat[0],tar_quat[1],tar_quat[2],
            #                     tar_quat_test[0],tar_quat_test[1],tar_quat_test[2],tar_quat_test[3],
            #                     0,0,0])
            # --- quadrotor control --- #
            _dt, w, x_opt_acados = nmpc_controller.nmpc_state_control(current_state=cur_state, target_state=target_pos)
            quad_thrusts_msg.thrusts_1 = w[0]
            quad_thrusts_msg.thrusts_2 = w[1]
            quad_thrusts_msg.thrusts_3 = w[2]
            quad_thrusts_msg.thrusts_4 = w[3]

            current_time = rospy.get_rostime().to_sec()
            print(f"Thrust:[{w[0]},{w[1]},{w[2]},{w[3]}]")
            print(f"delay {(current_time - last_time)*1000}ms")
            last_time = current_time
            is_get_state = False
            quad_thrusts_pub.publish(quad_thrusts_msg)
        else:
            quad_thrusts_pub.publish(quad_thrusts_msg)
        quad_path_msg.header.stamp = rospy.Time.now()
        quad_path_pub.publish(quad_path_msg)
        
        # test_print()
        # rate.sleep()
