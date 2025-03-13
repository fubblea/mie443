cmake_minimum_required(VERSION 2.8.3)
project(contest2_testbench)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV 3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    actionlib
    move_base_msgs
    roscpp
    cv_bridge
    image_transport
    gazebo_ros
)
find_package(orocos_kdl REQUIRED)

catkin_package()

include_directories(include  
    ../mie443_contest2/include/
    ${catkin_INCLUDE_DIRS}  
    ${OpenCV_INCLUDE_DIRS})

# Contest 2 Executable
add_executable(testbench 
    src/testbench.cpp)
target_link_libraries(testbench
    ${catkin_LIBRARIES}  
    ${OpenCV_LIBRARIES}  
    orocos-kdl)
