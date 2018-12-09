# motor_control

ROS catkin package, containing the [CTRE Phoenix](https://github.com/CrossTheRoadElec/Phoenix-Linux-SocketCAN-Example) libraries.

## How to build

Note, that this repository uses the `catkin` command instead of `catkin_make`. You can read more about this distinction [here](https://catkin-tools.readthedocs.io/en/latest/migration.html).

1. This is a catkin project, therefore you will need a catkin workspace. If you already have a catkin workspace, skip this step.
```
mkdir -p catkin_ws/src  # Create catkin_ws and catkin_ws/src directories
cd catkin_ws
catkin init             # Intialize the workspace
```
2. You then need to clone this repository into the src directory of your catkin workspace. Make sure to execute this command from inside the top level of the workspace.
```
git clone https://github.com/vanderbiltrobotics/motor_control.git src/motor_control
```
3. Finally, you can build your workspace with:
```
catkin build
```

## Running a node
This repository contains a motor_control_test_node for testing basic functionality. It can be run with:
```
catkin build             # Make sure that your workspace is built
source devel/setup.bash  # Setup enviroment to find your built nodes
rosrun motor_control motor_control_test_node
```