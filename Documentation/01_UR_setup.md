# **UR setup**

You can use:
- Virtual Box with speciffic virtual machine
- Docker image

To work also with a real UR robot arm, follow installation instructions in: 
- https://github.com/UniversalRobots/Universal_Robots_ROS_Driver

The official Universal Robots package and driver will be properly installed
- https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
- https://github.com/ros-industrial/universal_robot

To work only in virtual environment for simulation purposes, follow instructions in:
- https://github.com/ros-industrial/universal_robot

> Use the "melodic-devel" branch to clone "universal_robot" package in your workspace
>
> Delete the ".git" and ".github" of "universal_robot" package to sync it within your repository.
>
You have to update the dependencies (in main repository folder):
```shell
rosdep update
rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src
# building
catkin_make
```

You are now ready to work with the workspace!