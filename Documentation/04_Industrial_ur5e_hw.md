# **UR5e control real robot**
Detailed information will be found in readme file on:

Tutorial

http://docs.ros.org/en/melodic/api/moveit_tutorials/html/index.html

Videos

https://www.youtube.com/watch?v=b4T577d39dE

https://www.youtube.com/watch?v=BxCik8OI1Fw

1. Control Package
We will use the same package ur5control developed for simulation kinematics control.

we will use the same python files developed for simulation

The main diferences are:

First we open the driver with the callibration file:
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.1.2 kinematics_config:="${HOME}/manel_ROS/ur5arm_ws/my_robot_calibration.yaml"

We have to change the controllers in "controllers.yaml" file in "/home/mpuig/ur5arm_ws/src/fmauch_universal_robot/ur5_e_moveit_config/config/controllers.yaml"
The action_ns has to be changed from: follow_joint_trajectory to: scaled_pos_joint_traj_controller/follow_joint_trajectory

controller_list:
  - name: ""
    action_ns: /scaled_pos_joint_traj_controller/follow_joint_trajectory
    type: FollowJointTrajectory
    joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint
2. ur5e bringup
We have created a speciffic "ur5e_bringup_hw.launch" file to setup

<launch>
  <arg name="sim" default="false" />
  <arg name="limited" default="true"/>
  <arg name="robot_ip" default="192.168.2.2"/>
  <arg name="kinematics_config" default="${HOME}/ur5arm_ws/ur5control/config/my_ur5e_calibration.yaml"/>
  <!-- Launch ur5e_hw -->
   <include file="$(find ur_robot_driver)/launch/ur5e_bringup.launch">
    <arg name="limited" default="$(arg limited)"/>
    <arg name="kinematics_config" default="$(arg kinematics_config)"/>
  </include>
  <!-- Launch planning execution -->
  <include file="$(find ur5_e_moveit_config)/launch/ur5_e_moveit_planning_execution.launch">
    <arg name="limited" default="$(arg limited)"/>
    <arg name="sim" default="$(arg sim)"/>
  </include>
</launch>
roslaunch ur5control ur5e_bringup_hw.launch
3. Joints control
We will use the same "ur5e_joints_control1.py" to control the different link angles:

rosrun ur5control ur5e_joints_control1.py 
3. Pose control
We will use the same "ur5e_pose_control1.py" to control the pose target:

rosrun ur5control ur5e_pose_control1.py 