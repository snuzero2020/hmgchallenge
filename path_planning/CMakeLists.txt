cmake_minimum_required(VERSION 3.0.2)
project(path_planning)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(OpenCV 3 REQUIRED)
find_package(Eigen3)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp std_msgs sensor_msgs 

	DEPENDS
	opencv_highgui opencv_imgproc
)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(include)

add_library(quartic_polynomial src/quartic_polynomial.cpp)
add_library(quintic_polynomial src/quintic_polynomial.cpp)


set (SOURCES 
 ${CMAKE_CURRENT_SOURCE_DIR}/src/quartic_polynomial.cpp
 ${CMAKE_CURRENT_SOURCE_DIR}/src/quintic_polynomial.cpp
)

set (HEADERS
 ${CMAKE_CURRENT_SOURCE_DIR}/include/quartic_polynomial.h
 ${CMAKE_CURRENT_SOURCE_DIR}/include/quintic_polynomial.h
 ${CMAKE_CURRENT_SOURCE_DIR}/include/utils.h
)

add_executable(frenet_optimal_trajectory src/frenet_optimal_trajectory.cpp ${HEADERS} ${SOURCES})
add_dependencies(frenet_optimal_trajectory ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(frenet_optimal_trajectory ${catkin_LIBRARIES} ${OpenCV_LIBS} Eigen3::Eigen)