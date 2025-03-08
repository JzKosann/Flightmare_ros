#! /usr/bin/env python
#  coding :utf-8

import rospy
from geometry_msgs.msg import Pose
import sys
## acados import package
from acados_template import AcadosOcp, AcadosOcpSolver
from model import *

# @function # 检查ros的环境配置
# def check_syspath():
#     for p in sys.path:
#        print(p)
#     print()
#     return


if __name__ == '__main__':
    rospy.init_node('acados_py',anonymous=True)
    rate = rospy.Rate(10)

    desire_orinetation_pub = rospy.Publisher('flightmare_control/orinetation', Pose, queue_size=1000) # 发布控制信息
    z_bias =0

    while not rospy.is_shutdown():
        pose_msg = Pose()
        
        z_bias+=0.2
        pose_msg.position.x = 1.0
        pose_msg.position.y = 1.0
        pose_msg.position.z = z_bias
        pose_msg.orientation.w = 0.707
        pose_msg.orientation.x = 0
        pose_msg.orientation.y = 0
        pose_msg.orientation.z = 0.707
        desire_orinetation_pub.publish(pose_msg)
        # test_print()
        rate.sleep()
