#! /usr/bin/env python3

import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()    
group = moveit_commander.MoveGroupCommander("manipulator")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

group_variable_values = group.get_current_joint_values()

group_variable_values[0] = math.radians(0)
group_variable_values[1] = math.radians(-90)
group_variable_values[2] = math.radians(0)
group_variable_values[3] = math.radians(0)
group_variable_values[4] = math.radians(0)
group_variable_values[5] = math.radians(0)

group.set_joint_value_target(group_variable_values)

plan2 = group.plan()

rospy.sleep(5)
group.go(wait=True)
rospy.sleep(5)

print ("Current Joint Values:")
joint_target2=group.get_current_joint_values()
print ("q1 = "+str(round(math.degrees(joint_target2[0]),0)))
print ("q2 = "+str(round(math.degrees(joint_target2[1]),0)))
print ("q3 = "+str(round(math.degrees(joint_target2[2]),0)))
print ("q4 = "+str(round(math.degrees(joint_target2[3]),0)))
print ("q5 = "+str(round(math.degrees(joint_target2[4]),0)))
print ("q6 = "+str(round(math.degrees(joint_target2[5]),0)))
print ("Current Pose:")
pose_target2=group.get_current_pose()
qx=round(pose_target2.pose.orientation.x,2)
qy=round(pose_target2.pose.orientation.y,2)
qz=round(pose_target2.pose.orientation.z,2)
qw=round(pose_target2.pose.orientation.w,2)
print ("x = "+str(round(pose_target2.pose.position.x,1)))
print ("y = "+str(round(pose_target2.pose.position.y,1)))
print ("z = "+str(round(pose_target2.pose.position.z,1)))
print ("qx = "+str(qx))
print ("qy = "+str(qy))
print ("qz = "+str(qz))
print ("qw = "+str(qw))
quat2=[qx, qy, qz, qw]
(roll, pitch, yaw)=euler_from_quaternion(quat2)
print ("roll = "+str(math.degrees(roll)))
print ("pitch = "+str(math.degrees(pitch)))
print ("yaw = "+str(math.degrees(yaw)))

moveit_commander.roscpp_shutdown()