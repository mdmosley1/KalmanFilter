cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_STANDARD 14)

project( RobotSim )
find_package( OpenCV REQUIRED )
add_executable( RobotSim RobotSim.cpp )
target_link_libraries( RobotSim ${OpenCV_LIBS} )
