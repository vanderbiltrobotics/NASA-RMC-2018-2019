cmake_minimum_required(VERSION 3.1)
project(motor_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Detect system architecture and save it to $arch
EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE arch )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

find_package (Threads REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES CTRE_Phoenix CTRE_PhoenixCCI CTRE_PhoenixPlatformLinuxSocketCan CTRE_PhoenixCanutils
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


## Declare libraries to be imported
ADD_LIBRARY(CTRE_Phoenix STATIC IMPORTED)
ADD_LIBRARY(CTRE_PhoenixCCI STATIC IMPORTED)
ADD_LIBRARY(CTRE_PhoenixPlatformLinuxSocketCan STATIC IMPORTED)
ADD_LIBRARY(CTRE_PhoenixCanutils STATIC IMPORTED)

## Import appropriate libraries for architecture
if(${arch} STREQUAL "x86_64")
    SET_TARGET_PROPERTIES(CTRE_Phoenix PROPERTIES
            IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/x86-64/libCTRE_Phoenix.a)
    SET_TARGET_PROPERTIES(CTRE_PhoenixCCI PROPERTIES
            IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCCI.a)
    SET_TARGET_PROPERTIES(CTRE_PhoenixPlatformLinuxSocketCan PROPERTIES
            IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixPlatformLinuxSocketCan.a)
    SET_TARGET_PROPERTIES(CTRE_PhoenixCanutils PROPERTIES
            IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/x86-64/libCTRE_PhoenixCanutils.a)
elseif(${arch} STREQUAL "armv7l")
    SET_TARGET_PROPERTIES(CTRE_Phoenix PROPERTIES
            IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/raspberry/libCTRE_Phoenix.a)
    SET_TARGET_PROPERTIES(CTRE_PhoenixCCI PROPERTIES
            IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCCI.a)
    SET_TARGET_PROPERTIES(CTRE_PhoenixPlatformLinuxSocketCan PROPERTIES
            IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixPlatformLinuxSocketCan.a)
    SET_TARGET_PROPERTIES(CTRE_PhoenixCanutils PROPERTIES
            IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/raspberry/libCTRE_PhoenixCanutils.a)
else()
    message( FATAL_ERROR "Unrecognized architecture: ${arch}" )
endif()


add_executable(example ${PROJECT_SOURCE_DIR}/src/example.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(example
        ${catkin_LIBRARIES}
        Threads::Threads
        CTRE_Phoenix CTRE_PhoenixCCI CTRE_PhoenixPlatformLinuxSocketCan CTRE_PhoenixCanutils)


install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/${arch}/*
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

## Mark cpp header files for installation
install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )
