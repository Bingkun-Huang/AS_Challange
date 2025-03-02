cmake_minimum_required(VERSION 2.0.0)
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
)# Find OpenCV and Eigen3 packages
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)


catkin_package(
  CATKIN_DEPENDS roscpp image_transport cv_bridge sensor_msgs tf2_ros geometry_msgs
  DEPENDS OpenCV Eigen3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Build the image_converter_node executable
add_executable(sem_img_proc_node src/sem_img_proc_node.cpp)
add_dependencies(sem_img_proc_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(sem_img_proc_node ${catkin_LIBRARIES})

