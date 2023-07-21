#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import Float64
import math

def ur5e_move_joints():
    pub1 = rospy.Publisher('/shoulder_lift_joint_position_controller/command', Float64, queue_size=10)
    rospy.init_node('ur5e_joints', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        position = -math.pi/3
        rospy.loginfo(position)
        pub1.publish(position)
        rate.sleep()

if __name__ == '__main__':
    try:
        ur5e_move_joints()
    except rospy.ROSInterruptException:
        pass