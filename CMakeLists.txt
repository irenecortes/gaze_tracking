cmake_minimum_required(VERSION 2.8.3)
project(face_detection)

add_definitions(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  message_filters
  cv_bridge
)

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(dlib REQUIRED)

## Declare a catkin package
catkin_package(
	  CATKIN_DEPENDS 
	geometry_msgs 
	pcl 
	pcl_ros 
	pcl_conversions
	roscpp 
	rospy 
	sensor_msgs 
	stereo_msgs
	std_msgs
	tfm_msgs
	cv_bridge
	image_transport
	opencv2
	message_runtime
	  DEPENDS
	PCL
	DLIB
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${dlib_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${dlib_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_definitions(${OpenCV_DEFINITIONS})
add_definitions(${dlib_DEFINITIONS})


## Declare a C++ executable

add_executable(imagenes_rgbd src/imagenes_rgbd.cpp)
target_link_libraries(imagenes_rgbd ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${dlib_LIBRARIES})


 ##add_executable(dlib_face src/dlib_face.cpp)
 ##target_link_libraries(dlib_face ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${dlib_LIBRARIES})

##add_executable(head_pose_estimator src/head_pose_estimator.cpp)
 ##target_link_libraries(head_pose_estimator ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${dlib_LIBRARIES})

##add_executable(blink_counter src/blink_counter.cpp)
 ##target_link_libraries(blink_counter ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${dlib_LIBRARIES})

add_executable(gaze_tracking src/gaze_tracking.cpp)
target_link_libraries(gaze_tracking ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${dlib_LIBRARIES})

add_executable(personal_calibration src/personal_calibration.cpp)
 target_link_libraries(personal_calibration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${dlib_LIBRARIES})

