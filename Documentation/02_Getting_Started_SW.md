# **2. Getting started in virtual environment**

To verify the correct installation follow first instructions in:
- https://github.com/ros-industrial/universal_robot

Usage with Gazebo Simulation:

There are launch files available to bringup a simulated robot.

To bring up the simulated robot in Gazebo, run:
```shell
roslaunch ur_gazebo ur5_bringup.launch
```
MoveIt! with a simulated robot


Again, you can use MoveIt! to control the simulated robot.

For setting up the MoveIt! nodes to allow motion planning run:
```shell
roslaunch ur5_moveit_config moveit_planning_execution.launch sim:=true
```

For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:
```shell
roslaunch ur5_moveit_config moveit_rviz.launch
```
Test a "random feasible" target to verify the correct behaviour

Lets now control the joint positions.

## **2.1. Create a control package**

Create first a package:
```shell
catkin_create_pkg ur5e_control rospy controller_manager joint_state_controller robot_state_publisher
```
and inside create the folders:
- urdf
- config
- launch

## **2.2. UR robot arm model**

The robot arm model is located in "ur_description"-->"urdf" folder. Go to this folder and you will see all the models in xacro format.

- The first action is to translate the model in urdf format. Type:
```shell
rosrun xacro xacro ur5e.xacro > ur5e_generated.urdf
```

- You need to add this speciffic "gazebo_ros_control" plugin.
```shell
<plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
</plugin>
```
Place this new model to the urdf folder created in the new package.

## **2.3. Add the configuration file**
Apart from modifying the URDF, we have to provide a configuration file that loads the controller parameters to the parameter server. The position controller we will use is a PID controller.

Copy the "ur5e_controllers.yaml" file from "ur_gazebo">""config" folder to your new created "config" folder in the new package.

## **2.4. Spawn UR robot in virtual environment Gazebo**
In the "launch" folder create a new "ur5e_custom_bringup.launch" file with:
```xml
<launch>
  <include file="$(find gazebo_ros)/launch/empty_world.launch" >
    <arg name="paused" value="true" />
  </include>

  <node name="spawn_gazebo_model" pkg="gazebo_ros" type="spawn_model" args="-file $(find ur5e_control)/urdf/ur5e_generated.urdf -urdf -x 0 -y 0 -z 0.1 -model ur5 -J shoulder_lift_joint -0.5 -J elbow_joint 0.5" />

  <!-- Load joint controller configurations from YAML file to parameter server -->
  <rosparam file="$(find ur5e_control)/config/ur5e_controllers.yaml" command="load"/>

  <param name="robot_description" textfile="$(find ur5e_control)/urdf/ur5e_generated.urdf"/>
  
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
You can see the topics and nodes but there is no topic to control the robot joints. For this purpose, we have to create a specific package.

## **2.3. Control the Joint positions**

Create first a package:
```shell
catkin_create_pkg ur5e_control rospy controller_manager joint_state_controller robot_state_publisher
```

Load again the UR robot in Gazebo to see the topics to control.

