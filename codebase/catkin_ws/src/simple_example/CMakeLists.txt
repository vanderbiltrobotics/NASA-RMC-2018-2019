cmake_minimum_required(VERSION 2.8.3)
project(simple_example)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simple_example
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(simple_example_v1 src/simple_example_v1.cpp)
target_link_libraries(simple_example_v1 ${catkin_LIBRARIES})