cmake_minimum_required(VERSION 3.0.2)
project(towfish)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  roslib
  message_generation
)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# Find the required libraries
find_package(GLIB2 REQUIRED)
find_Package(GTHREAD2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonLibs 2.7 REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} -g -O2")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)


## Declare messages and services
add_message_files(
  DIRECTORY msg
   FILES
  DVLBeam.msg
  DVLBeamsArr.msg
  DVLOrient.msg
  DVLVel.msg
  PsData.msg
  Temperature.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Catkin Package
catkin_package(
  INCLUDE_DIRS scripts
  CATKIN_DEPENDS 
  message_runtime 
  roscpp
  rospy 
  std_msgs
  sensor_msgs
  geometry_msgs
  roslib
)

# Our includes
include_directories(
  scripts
  include
  ${catkin_INCLUDE_DIRS}
  ${GLIB2_MAIN_INCLUDE_DIR}
  ${GLIB2_INTERNAL_INCLUDE_DIR}
  ${GTHREAD2_MAIN_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Files that we need to build
add_executable(${PROJECT_NAME}_node
  src/libbot/ringbuf.c
  src/libbot/rotations.c
  src/libbot/serial.c
  src/libbot/timestamp.c
  src/micro/microstrain_comm.cpp
)

# What libraries we need
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${GLIB2_LIBRARY}
  ${GTHREAD2_LIBRARY}
)




