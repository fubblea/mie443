cmake_minimum_required(VERSION 2.8.3)
project(contest2_testbench)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV 3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    cv_bridge
    image_transport
    sensor_msgs
)

catkin_package()

include_directories(include  
    ../mie443_contest2/include/
    ${catkin_INCLUDE_DIRS}  
    ${OpenCV_INCLUDE_DIRS})

# Testbench Executable
add_executable(testbench 
    src/testbench.cpp)
target_link_libraries(testbench
    ${catkin_LIBRARIES}  
    ${OpenCV_LIBRARIES})

# Testrunner Executable
add_executable(testrunner
    src/testrunner.cpp
    ../mie443_contest2/src/imagePipeline.cpp
    ../mie443_contest2/src/utils.cpp
    ../mie443_contest2/src/boxes.cpp)
target_link_libraries(testrunner
    ${catkin_LIBRARIES}  
    ${OpenCV_LIBRARIES})
