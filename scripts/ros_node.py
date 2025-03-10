#! /usr/bin/env python
#  coding :utf-8

import rospy
from geometry_msgs.msg import Pose
from mav_msgs.msg import quad_thrusts
import sys
## acados import package
from acados_template import AcadosOcp, AcadosOcpSolver
from controller import *

# @function # 检查ros的环境配置
# def check_syspath():
#     for p in sys.path:
#        print(p)
#     print()
#     return


if __name__ == '__main__':
    rospy.init_node('acados_py',anonymous=True)
    rate = rospy.Rate(10)

    quad_thrusts_pub = rospy.Publisher('flightmare_control/thrusts', quad_thrusts, queue_size=10) # 发布控制信息
    quad_thrusts_msg = quad_thrusts()
    
    while not rospy.is_shutdown():

        # quad_thrusts.thrusts_1
        
        quad_thrusts_pub.publish(quad_thrusts_msg)
        # test_print()
        rate.sleep()
