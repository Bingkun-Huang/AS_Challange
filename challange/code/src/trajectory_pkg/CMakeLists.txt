cmake_minimum_required(VERSION 3.0.2)
project(trajectory_pkg)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11)

############
# BINARIES #
############
cs_add_executable(trajectory_generation_example
        src/example_planner.cc
        src/example_planner_node.cc
)

##########
# EXPORT #
##########
cs_install()
cs_export()