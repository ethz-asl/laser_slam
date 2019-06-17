cmake_minimum_required(VERSION 2.8.3)
project(laser_slam_ros)

add_definitions(-std=c++11 -DBENCHMARK_ENABLE)

find_package(catkin_simple 0.1.0 REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    genmsg
    message_generation
    tf
    rosbag
    visualization_msgs
)

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME} 
    src/laser_slam_worker.cpp
    src/laser_mapper.cpp
)

cs_add_executable(laser_mapper_node src/laser_mapper_node.cpp)
target_link_libraries(laser_mapper_node ${PROJECT_NAME})

cs_install()
cs_export()

