#! /usr/bin/env python
#  coding :utf-8

import rospy
from mav_msgs.msg import QuadThrusts, QuadCurState
from controller import *

# @function # 检查ros的环境配置
# def check_syspath():
#     for p in sys.path:
#        print(p)
#     print()
#     return
quad_thrusts_pub = None
quad_state_sub = None
nmpc_controller = NMPC_Controller()
quad_thrusts_msg = QuadThrusts()
quad_state = QuadCurState()

is_get_state = False    # 是否获取状态

def normalize_quaternion(qw, qx, qy, qz):
    norm = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
    return qw/norm, qx/norm, qy/norm, qz/norm

def quad_Callback(data):
    global nmpc_controller,quad_thrusts_msg, quad_state, is_get_state
    quad_state = data
    quad_state.qw, quad_state.qx, quad_state.qy, quad_state.qz = normalize_quaternion(quad_state.qw,quad_state.qx,quad_state.qy,quad_state.qz)

    # cur_state = np.array([data.x, data.y, data.z,
    #                       data.vx, data.vy, data.vz,
    #                       data.phi, data.theta, data.psi,
    #                       data.p, data.q, data.r])
    # tar_pos = np.array([0,0,20,0,0,0,0,0,0,0,0,0])

    # _dt, w, x_opt_acados = nmpc_controller.nmpc_state_control(current_state=cur_state, target_state=tar_pos)

    # quad_thrusts_msg.thrusts_1 = w[0]
    # quad_thrusts_msg.thrusts_2 = w[1]
    # quad_thrusts_msg.thrusts_3 = w[2]
    # quad_thrusts_msg.thrusts_4 = w[3]
    print("-------quadrotor_state-------")
    print(data)
    is_get_state = True
    # print(w[1])

def ros_init():
    global quad_state_sub, quad_thrusts_pub
    quad_thrusts_pub = rospy.Publisher('flightmare_control/thrusts', QuadThrusts, queue_size=1) # 发布控制信息
    quad_state_sub = rospy.Subscriber('flightmare_control/state_info', QuadCurState, quad_Callback, queue_size=1)


if __name__ == '__main__':
    
    rospy.init_node('acados_py',anonymous=True)
    ros_init()
    rate = rospy.Rate(100)
    # quad_thrusts_msg = QuadThrusts()

    # # cnt = 0 
    # quad_thrusts_msg.thrusts_1 = 0
    # quad_thrusts_msg.thrusts_2 = 0
    # quad_thrusts_msg.thrusts_3 = 0
    # # quad_thrusts_msg.thrusts_4 = 0
    # for cnt in range(50):
    #     quad_thrusts_pub.publish(quad_thrusts_msg)
    #     print(f"arousing.....%{cnt}")
    #     rate.sleep()


    # rospy.spin()
    
    while not rospy.is_shutdown():

        if is_get_state:
            cur_state = np.array([quad_state.x, quad_state.y, quad_state.z,
                                quad_state.vx, quad_state.vy, quad_state.vz,
                                quad_state.qw, quad_state.qx, quad_state.qy, quad_state.qz,
                                #   quad_state.phi, quad_state.theta, quad_state.psi,
                                quad_state.p, quad_state.q, quad_state.r])
            tar_pos = np.array([0,0,20,0,0,0,0,0,0,0,0,0,0])
            _dt, w, x_opt_acados = nmpc_controller.nmpc_state_control(current_state=cur_state, target_state=tar_pos)

            quad_thrusts_msg.thrusts_1 = w[0]
            quad_thrusts_msg.thrusts_2 = w[1]
            quad_thrusts_msg.thrusts_3 = w[2]
            quad_thrusts_msg.thrusts_4 = w[3]
            print(f"Thrust:[{w[0]},{w[1]},{w[2]},{w[3]}]")
            is_get_state = False
            quad_thrusts_pub.publish(quad_thrusts_msg)
        else:
            # quad_thrusts_msg.thrusts_1 = 0
            # quad_thrusts_msg.thrusts_2 = 0
            # quad_thrusts_msg.thrusts_3 = 0
            # quad_thrusts_msg.thrusts_4 = 0
            # ("waiting information...")
            quad_thrusts_pub.publish(quad_thrusts_msg)

        
        # test_print()
        rate.sleep()
