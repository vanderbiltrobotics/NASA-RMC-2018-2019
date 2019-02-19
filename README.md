# NASA-RMC-2018-2019

This repository contains all the ROS packages used to control the robot created by Vanderbilt Robotics for participation in the 2019 NASA Robotics Mining Competition.

## How to build these packages

1. This is a catkin project, therefore you will need to create a catkin workspace to contain all the packages. Create a folder called `catkin_ws` and within it, a folder called `src`. Navigate to the catkin_ws directory. This can be done via the command line with the following commands: 
```
mkdir -p catkin_ws/src  # Create catkin_ws and catkin_ws/src directories
cd catkin_ws
```
2. You then need to clone this repository into the src directory of your catkin workspace. Make sure to execute this command from inside the top level of the workspace.
```
git clone https://github.com/vanderbiltrobotics/NASA-RMC-2018-2019.git src
```
The `src` folder of your catkin_ws should now contain all the packages in the repository.

3. Finally, you can build your workspace with:
```
catkin_make              # Build all packages in the workspace
```
4. Sourcing the catkin_ws

Running the command `source {Path to catkin_ws}/catkin_ws/devel/setup.bash` will allow ROS to find your built packages. You will need to run this command each time you open a new terminal if you want ROS to find your packages. To avoid this, you can add that command to the end of your .bashrc and it will run automatically whenever you open a terminal.

## Notes on developing these packages

If you are working on a new version of a particular package, do it on a speparate branch! The master branch should always contain the latest *working version* of each package. Once your new version of a package is working and thoroughly tested, it can be merged into the master branch.
