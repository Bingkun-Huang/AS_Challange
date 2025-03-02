cmake_minimum_required(VERSION 2.8.3)
project(astar_path_planner)

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
  octomap_msgs
  message_filters
  visualization_msgs
  trajectory_msgs
  nav_msgs
  geometry_msgs
  tf_conversions
  cv_bridge
  image_transport
  sensor_msgs

)

catkin_package(CATKIN_DEPENDS 
  roscpp
  std_msgs
  octomap_msgs
  message_filters
  visualization_msgs
  trajectory_msgs
  nav_msgs
  geometry_msgs
  tf_conversions
  cv_bridge
  image_transport
  sensor_msgs
)

find_package(ompl REQUIRED)
find_package(FCL REQUIRED)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
)
# link_libraries("/usr/local/lib/libfcl.so.0.7.0" "/usr/local/lib/libfcl.so.7" "/usr/local/lib/libfcl.so")

add_executable(path_planner src/path_planner.cpp)
add_dependencies(path_planner ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_planner 
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${FCL_LIBRARIES}
)


add_executable(frontier_detecter src/Frontier_Detecter.cpp)
add_dependencies(frontier_detecter ${catkin_EXPORTED_TARGETS})
target_link_libraries(frontier_detecter 
  ${catkin_LIBRARIES}
)

install(TARGETS 
  path_planner 
  frontier_detecter
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)