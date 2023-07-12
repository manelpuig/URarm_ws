# **2. Getting sterted in virtual environment**

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
