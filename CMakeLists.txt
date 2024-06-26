cmake_minimum_required(VERSION 3.0.2)
project(livelybot_controller)

find_package(gazebo REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    controller_manager
    genmsg
    joint_state_controller
    robot_state_publisher
    roscpp
    rospy
    gazebo_ros
    gazebo_msgs
    std_msgs
    tf
    geometry_msgs
    livelybot_msg  
    livelybot_serial 
)
catkin_package(
    CATKIN_DEPENDS 
    livelybot_msg 
    INCLUDE_DIRS include
)
include_directories(include ${CMAKE_BINARY_DIR})
include_directories(
    include
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    third_party/qpOASES/include
)
link_directories(
    ${GAZEBO_LIBRARY_DIRS}
)
file(GLOB_RECURSE SRC_LIST
"src/*.cpp"
"src/*/*.cpp"
"src/*/*.h"
# "ConvexMPC/*.cpp"
)
add_subdirectory(third_party/qpOASES)
add_executable(main src/main.cpp  ${SRC_LIST} )
target_link_libraries(main ${catkin_LIBRARIES} qpOASES -pthread lcm)
