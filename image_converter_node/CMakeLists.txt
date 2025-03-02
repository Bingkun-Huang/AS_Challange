cmake_minimum_required(VERSION 3.0.2)
project(image_converter_node)

## Compile as C++11, supported in ROS Noetic
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
  sensor_msgs
  tf2_ros
  geometry_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_converter_node
#  CATKIN_DEPENDS roscpp image_transport cv_bridge sensor_msgs tf2_ros geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(image_converter_node src/image_converter.cpp)
add_dependencies(image_converter_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_converter_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
