# **2. Custom robot: Getting started in virtual environment**

Important information in:

- http://wiki.ros.org/universal_robots
- https://github.com/ros-industrial/universal_robot
- https://sir.upc.edu/projects/rostutorials/10-gazebo_control_tutorial/index.html#

Also:
- https://roboticscasual.com/ros-tutorial-simulate-ur5-robot-in-gazebo-urdf-explained/
- https://roboticscasual.com/ros-tutorial-control-the-ur5-robot-with-ros_control-tuning-a-pid-controller/
- https://roboticscasual.com/robotics-tutorials/
- https://discourse.ros.org/t/new-packages-for-noetic-2022-11-30/28592
- http://wiki.ros.org/rqt_joint_trajectory_controller
- https://github.com/dairal/ur5-joint-position-control/tree/main

First of all you need to install the controllers:
```shell
sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers
```
You can see the installed packages:
- ros_control
- ros_controlers

In /opt/ros/noetic/share folder

## **1. Industrial Robots**

To verify the correct installation follow first instructions in:
- https://github.com/ros-industrial/universal_robot

You can use the "universal_robot" package to control the ur5e robot arm

- Bringup your ur5e robot arm:
```shell
roslaunch ur_gazebo ur5e_bringup.launch
```
**Control joint angle values**

This can be done using rqt.

The rqt tool has two interesting plugins for control purposes:

- The Robot Tools/Controller Manager: To load, unload, start and stop controllers.

- The Robot Tools/Joint Trajectory Controller: To move the robot joints using a Joint Trajectory Controller.

Verify you have installed:
```shell 
sudo apt install ros-noetic-rqt-controller-manager
sudo apt install ros-noetic-rqt-joint-trajectory-controller
```
You can access to each robot joint:
![](./Images/02_getting_started_sw/1_rqt_ur5e.png)

**Control position TCP**

- There are launch files available to bringup a simulated robot. To bring up the simulated robot in Gazebo, run:
```shell
roslaunch ur_gazebo ur5_bringup.launch
```
- MoveIt! with a simulated robot. You can use MoveIt! to control the simulated robot. For setting up the MoveIt! nodes to allow motion planning run:
```shell
roslaunch ur5_moveit_config moveit_planning_execution.launch sim:=true
```

- For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:
```shell
roslaunch ur5_moveit_config moveit_rviz.launch
```
- Test a "random feasible" target to verify the correct behaviour


## **2. Custom robot arm**

We will create a custom robot model os we can use a ready to use robot arm model in urdf format. We will use ur5_model.urdf for the firts exemple.

### **2.1. Custom package**

Create a custom package:
```shell
catkin_create_pkg custom_robot_control rospy controller_manager joint_state_controller robot_state_publisher
```
and inside create the folders:
- urdf
- config
- launch

### **2.2. UR robot arm model**

The robot arm model is located in "ur_description"-->"urdf" folder. 

If you have a model in xacro format, go to this folder and make the conversion to urdf format.

```shell
rosrun xacro xacro ur5e.xacro > ur5e_generated.urdf
```
For any robot arm model you will identify for each link:
- The <transmission> element has to be defined to link actuators to joints. 
```xml
  <transmission name="shoulder_pan_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="shoulder_pan_joint">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="shoulder_pan_motor">
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
```
- A Gazebo plugin needs to be added to the URDF to actually parse the transmission tags and load the appropriate hardware interfaces and controller manager.
```xml
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    </plugin>
  </gazebo>
```
We propose you a ur5 exemple model to practice

### **2.3. Add the configuration file**
Apart from modifying the URDF, we have to provide a configuration file that loads the controller parameters to the parameter server. The position controller we will use is a PID controller.
- The controller type has to correspond to the "Hardware Interface". In our exemple the type is: effort_controllers/JointPositionController

We propose you an exemple of configuration file properly designed for ur5 robot arm.

### **2.4. Spawn UR robot in virtual environment Gazebo**
In the "launch" folder create a new "ur5_custom_bringup.launch" file with:
```xml
<launch>
  <include file="$(find gazebo_ros)/launch/empty_world.launch" >
    <arg name="paused" value="true" />
  </include>

  <node name="spawn_gazebo_model" pkg="gazebo_ros" type="spawn_model" args="-file $(find ur5e_control)/urdf/ur5_model.urdf -urdf -x 0 -y 0 -z 0.1 -model ur5 -J shoulder_lift_joint -0.5 -J elbow_joint 0.5" />

  <!-- Load joint controller configurations from YAML file to parameter server -->
  <rosparam file="$(find ur5e_control)/config/ur5_controllers.yaml" command="load"/>

  <param name="robot_description" textfile="$(find ur5e_control)/urdf/ur5_model_.urdf"/>
  
  <!-- load the controllers -->
  <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
    output="screen" args="shoulder_pan_joint_position_controller shoulder_lift_joint_position_controller elbow_joint_position_controller wrist_1_joint_position_controller wrist_2_joint_position_controller wrist_3_joint_position_controller joint_state_controller"/>

  <!-- convert joint states to TF transforms for rviz, etc -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"
    respawn="false" output="screen">
    <remap from="/joint_states" to="/ur5/joint_states" />
  </node>

</launch>
```
Type:
```shell
roslaunch custom_robot_control custom_ur5_bringup.launch
```

You can see the topics to control the joint angles

### **2.5. Control the Joint positions**

This can be done:

**a) Using rqt**

The rqt tool has two interesting plugins for control purposes:

- Topics message publisher


**b) using topics**

- Bringup first the robot arm with speciffic controllers defined in "ur5_controllers.yaml" file in "config" folder. 

- list all the topics and choose the one to publish an angle value. For exemple "/shoulder_lift_joint_position_controller/command"
```shell
rostopic pub -1 /shoulder_lift_joint_position_controller/command std_msgs/Float64 "data: -1.507"
```
- You can create a node to publish an angle value to each joint. This is performed in a python file "ur5_joint_state.py"
```shell
rosrun custom_robot_control custom_ur5_joint_state.py
```

**Exercise**
Create a node to specify a 6 angle joint configuration for our ur5_model 

