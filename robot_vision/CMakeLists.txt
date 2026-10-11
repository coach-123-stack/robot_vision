cmake_minimum_required(VERSION 3.0.2)
project(robot_vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
  image_transport
  image_geometry
  message_filters
)

find_package(OpenCV REQUIRED COMPONENTS
  core
  imgproc
  highgui
  calib3d
  aruco
  videoio
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs cv_bridge image_transport image_geometry
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(aruco_pose_node src/aruco_pose_node.cpp)
add_dependencies(aruco_pose_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(aruco_pose_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
