# **Forward Kinematics**

Important information in:

- http://wiki.ros.org/universal_robots
- https://github.com/ros-industrial/universal_robot
- https://roboticscasual.com/ros-tutorial-simulate-ur5-robot-in-gazebo-urdf-explained/
- https://roboticscasual.com/ros-tutorial-control-the-ur5-robot-with-ros_control-tuning-a-pid-controller/
- https://roboticscasual.com/robotics-tutorials/
- https://discourse.ros.org/t/new-packages-for-noetic-2022-11-30/28592
- http://wiki.ros.org/rqt_joint_trajectory_controller
- https://github.com/dairal/ur5-joint-position-control/tree/main

First of all you need to install the controllers:
```shell
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
sudo apt-get install ros-noetic-rqt-joint-trajectory-controller
```
You can see the installed packages:
- ros_control
- ros_controlers
- rqt_joint_trajectory_controller

In /opt/ros/noetic/share folder

## **Forward Kinematics**
Type:
```shell
roslaunch ur_gazebo ur5e_bringup.launch
rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

Information is:

- https://www.youtube.com/watch?v=3C_F8vhnUPI
- https://classic.gazebosim.org/tutorials?tut=ros_control

## **Control the UR5 robot with ros_control**
We will extend the URDF description with position controllers for every joint and parametrize them with a configuration file.

### **Modifying the URDF**
I will now describe the parts that have to be added to the URDF in oder to use it together with ros_control.

First, we need to insert a ros_control plugin that parses the URDF directly after the opening <robot> tag
```xml
<plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
</plugin>
```
Second, we have to add the position controllers after the <link> and <joint> specifications. This is done inside the <transmission> tag which is presented below for the soulder lift joint:
```xml
<transmission name="shoulder_lift_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="shoulder_lift_joint">
    <hardwareInterface>EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="shoulder_lift_motor">
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```
### **Adding the configuration file**
Apart from modifying the URDF, we have to provide a configuration file that loads the controller parameters to the parameter server. The position controller we will use is a PID controller which means that it consists of one proportional, one integral and one differential term.

The configuration file stored stored in a separate sub folder named config in our project folder. This is what the file looks like for our UR5 robot:
```xml
# Publish all joint states -----------------------------------
joint_state_controller:
  type: joint_state_controller/JointStateController
  publish_rate: 50  
 
# Position Controllers ---------------------------------------
shoulder_pan_joint_position_controller:
  type: effort_controllers/JointPositionController
  joint: shoulder_pan_joint
  pid: {p: 500.0, i: 0.01, d: 50.0, i_clamp_min: -100.0, i_clamp_max: 100.0}
shoulder_lift_joint_position_controller:
  type: effort_controllers/JointPositionController
  joint: shoulder_lift_joint
  pid: {p: 500.0, i: 100.0, d: 30.0, i_clamp_min: -400.0, i_clamp_max: 400.0}
elbow_joint_position_controller:
  type: effort_controllers/JointPositionController
  joint: elbow_joint
  pid: {p: 10000.0, i: 0.01, d: 50.0, i_clamp_min: -100.0, i_clamp_max: 100.0}
wrist_1_joint_position_controller:
  type: effort_controllers/JointPositionController
  joint: wrist_1_joint
  pid: {p: 200.0, i: 10.0, d: 20.0, i_clamp_min: -400.0, i_clamp_max: 400.0}
wrist_2_joint_position_controller:
  type: effort_controllers/JointPositionController
  joint: wrist_2_joint
  pid: {p: 100.0, i: 0.1, d: 10.0, i_clamp_min: -100.0, i_clamp_max: 100.0}
wrist_3_joint_position_controller:
  type: effort_controllers/JointPositionController
  joint: wrist_3_joint
  pid: {p: 100.0, i: 0.1, d: 10.0, i_clamp_min: -100.0, i_clamp_max: 100.0}
  ```
  

There is p,i, and d values defined for every joint controller. At the top there is also a robot_state_publisher. That one publishes the joint states and is not important for us at the moment.