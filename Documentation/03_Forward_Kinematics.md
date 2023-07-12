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