#! /usr/bin/env python
#  coding :utf-8

import rospy
from std_msgs.msg import String
import sys

# sys.path.append('/home/jz/Documents/acados/interfaces/acados_template/acados_template')
# sys.path.append('/home/jz/anaconda3/envs/flightros_env/lib/python3.8/site-packages')

for p in sys.path:
    print(p)

print()
# import acados_template

# print(acados_template.__file__)




def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        hello_str = "hello %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInternalException:
        pass