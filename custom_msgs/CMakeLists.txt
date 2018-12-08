cmake_minimum_required(VERSION 2.8.3)
project(custom_msgs)

# Find dependencies
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  DriveMotorSpeeds.msg
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

# Export message runtime dependency
catkin_package(
  CATKIN_DEPENDS message_runtime
)

# Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

