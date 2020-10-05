cmake_minimum_required(VERSION 3.5.0)
project(rtr_quadcopter_controller)

add_compile_options(-std=c++17 -O3 -Wall -Wextra)

# Choreonoidのヘッダファイルがあるフォルダ.

find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  trajectory_msgs
  choreonoid
  pcl_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  ${CNOID_PLUGIN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
# multicompet plugin のヘッダファイルをコピーする必要
# ~/catkin_ws/devel/include/choreonoid-1.7/cnoid/src/MulticopterPlugin に choreonoid/src/MulticopterPlugin/exportdecl.h をコピー.


link_directories(
  ${CHOREONOID_LIBRARY_DIRS}
  ${PCL_LIBRARIES}
)

function(add_cnoid_simple_controller)
  set(target ${ARGV0})
  list(REMOVE_AT ARGV 0)
  add_library(${target} SHARED ${ARGV})
  target_link_libraries(${target} ${catkin_LIBRARIES} ${CHOREONOID_BODY_LIBRARIES} CnoidMulticopterPlugin)
  set_target_properties(${target} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_SUBDIR}/simplecontroller PREFIX "")
  install(TARGETS ${target} LIBRARY DESTINATION ${CHOREONOID_PLUGIN_SUBDIR}/simplecontroller)
endfunction()

add_subdirectory(src)
