cmake_minimum_required(VERSION 2.8.3)
project(decision_module)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
)

find_package(OpenCV 3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge

	DEPENDS
	opencv_highgui opencv_imgproc
)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(include)


#add_executable(cluster_main src/cluster_main.cpp ${HEADERS} ${SOURCES})
#add_dependencies(cluster_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(cluster_main ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

#######################
##prediction_treacker##
#######################

