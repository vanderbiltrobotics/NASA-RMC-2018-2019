# motor_control

ROS catkin package, containing the [CTRE Phoenix](https://github.com/CrossTheRoadElec/Phoenix-Linux-SocketCAN-Example) libraries.

## Running a node
This repository contains a motor_control_test_node for testing basic functionality. It can be run with:
```
catkin_make              # Make sure that your workspace is built
source devel/setup.bash  # Setup enviroment to find your built nodes
rosrun motor_control motor_control_test_node
```