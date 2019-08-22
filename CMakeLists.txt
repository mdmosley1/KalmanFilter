cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_STANDARD 11)

project( KalmanFilter )
find_package( OpenCV REQUIRED )
add_executable( KalmanFilter KalmanFilter.cpp )
target_link_libraries( KalmanFilter ${OpenCV_LIBS} )
