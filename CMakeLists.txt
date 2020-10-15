cmake_minimum_required(VERSION 2.8.3)
project(synthia_to_rosbag)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem)

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/synthia_ros_conversions.cpp
  src/synthia_common.cpp
  src/synthia_parser.cpp
)

############
# BINARIES #
############

cs_add_executable(synthia_live_node
  src/synthia_live_node.cpp
)
target_link_libraries(synthia_live_node ${PROJECT_NAME})

cs_add_executable(synthia_rosbag_converter
  src/synthia_rosbag_converter.cpp
)
target_link_libraries(synthia_rosbag_converter ${PROJECT_NAME})

cs_add_executable(tiny_disp_view
  src/tiny_disp_view.cpp
)
target_link_libraries(tiny_disp_view ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
