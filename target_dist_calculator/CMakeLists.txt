cmake_minimum_required(VERSION 3.0.2)
project(target_dist_calculator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  message_generation
  cv_bridge
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  image_transport
)

## Find OpenCV
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

## Find PCL
find_package(PCL REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  DetectOut.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  geometry_msgs
)

## Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge geometry_msgs tf2 tf2_ros tf2_geometry_msgs nav_msgs 
  DEPENDS PCL OpenCV
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

## Declare the executable
add_executable(target_distance_calculator
  src/target_dist_node.cpp
)

## Link the executable with the necessary libraries
target_link_libraries(target_distance_calculator
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

## Install the executable
install(TARGETS target_distance_calculator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Declare the executable
add_executable(target_bearing_calculator
  src/target_bearing_node.cpp
)

## Link the executable with the necessary libraries
target_link_libraries(target_bearing_calculator
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

## Install the executable
install(TARGETS target_bearing_calculator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Declare the executable
add_executable(target_distance_cluster_calculator
  src/target_dist_cluster_node.cpp
)

## Link the executable with the necessary libraries
target_link_libraries(target_distance_cluster_calculator
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

## Install the executable
install(TARGETS target_distance_cluster_calculator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 构建odometry_handler节点
add_executable(odom_trans src/odom_trans.cpp)

## 链接依赖的库
target_link_libraries(odom_trans
  ${catkin_LIBRARIES}
)

## 安装目标
install(TARGETS odom_trans
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)