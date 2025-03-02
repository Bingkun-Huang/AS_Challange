cmake_minimum_required(VERSION 2.8.3)
project(manully_controller)

include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wl,--no-as-needed")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  std_msgs 
  geometry_msgs 
  mav_msgs
  eigen_conversions
  tf_conversions
  sensor_msgs
)

catkin_package(CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  mav_msgs 
  tf
  eigen_conversions
  tf_conversions
  sensor_msgs
)

find_package(Eigen3)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(keyboard_traj_node src/keyboard_traj_node.cpp)
add_dependencies(keyboard_traj_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(keyboard_traj_node ${catkin_LIBRARIES})

add_executable(keyboard_input_node src/keyboard_input_node.cpp)
add_dependencies(keyboard_input_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(keyboard_input_node ${catkin_LIBRARIES})
