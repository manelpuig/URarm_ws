# **POSE control**

Important information in:

- https://www.udemy.com/course/robotics-with-ros-build-robotic-arm-in-gazebo-and-moveit/learn/lecture/28756138#overview


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

I will now describe the parts that have to be changed to the URDF in oder to use it together with ros_control.

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

### **Commanding the joint positions**
Now we can send position commands to our position controllers in order to move the robot.

For bringup, type:
```shell
roslaunch ur5-joint-position-control ur5_gazebo_joint_position_control.launch
```
The position controllers listen to the topic ur5/controller_name/command/. The easiest way to send commands is to start up the rqt_gui:
```shell
rosrun rqt_gui rqt_gui
```
In the menu bar under Plugins we choose the Message Publisher from the Topics folder.
We will go through the tuning process for the wrist_1 joint so please add the /wrist_1_joint_position_controller/command topic. 
> Press "run" in Gazebo
>
> Select the topic where to publish by clicking on the box

### **Tuning a PID Position Controller**
As before with the Message Publisher, add the Plot plugin from the plugin menu to your rqt window. You can add topics that you want to visualize to the graph 

For tuning, we want to look at the controller command and the system response to it. As an example, add the topics /wrist_1_joint_position_controller/state/process_value which is the (virtually) measured joint position and the /wrist_1_joint_position_controller/command/data to the plot.

To change PID values, you can use Dynamic Reconfigure from the Plugins menu under the Configuration folder. 