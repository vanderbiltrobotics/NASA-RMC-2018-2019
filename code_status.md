# Code status


- robot_motor_control: No crashes but we burn't through a few motors last year. Obviously something is wrong.
- robot_navigation: hahahah
- robot_sensors: Works as intended.
- robot_high_level_control: Empty package.
- robot_mission_control: contains code for teleop operation of the robot.
- robot_simulation: no gazebo simulation of the robot was ever made.
- robot_slam: The EKF is supposedly well tested. The aruco localization is unusable though. The pose it publishes oscillates uncontrolably. 
- robot_digging: nothing in here. All the motors were controlled directly from robot_motor_control
