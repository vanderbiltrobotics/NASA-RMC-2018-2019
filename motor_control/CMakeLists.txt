cmake_minimum_required(VERSION 2.8.3)
project(motor_control)

## Detect system architecture and save it to $arch
EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE arch )

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        geometry_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package (Threads REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES CTRE_Phoenix CTRE_PhoenixCCI CTRE_PhoenixPlatformLinuxSocketCan CTRE_PhoenixCanutils
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

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

## Declare a C++ executable
add_executable(can_bus_interface ${PROJECT_SOURCE_DIR}/src/can_bus_interface.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(can_bus_interface
    ${catkin_LIBRARIES}
    Threads::Threads
    CTRE_Phoenix CTRE_PhoenixCCI CTRE_PhoenixPlatformLinuxSocketCan CTRE_PhoenixCanutils
)

#############
## Install ##
#############

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/${arch}/*
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)