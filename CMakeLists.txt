cmake_minimum_required(VERSION 2.8.3)
project(spheremap_server)

# Override CXX flags inherited from workspace, if precompiled PCL binaries from debian repos are used
if (DEFINED ENV{PCL_CROSS_COMPILATION})
  set(PCL_CROSS_COMPILATION $ENV{PCL_CROSS_COMPILATION})
else()
  set(PCL_CROSS_COMPILATION "false")
endif()
if(${PCL_CROSS_COMPILATION} STREQUAL "false")
  message("Using precompiled PCL binaries from debian repos. Overriding catkin workspace CMAKE_CXX_FLAGS.")
  set(CMAKE_CXX_FLAGS "-std=c++17")
else()
  message("Using custom-built PCL binaries. Inheriting all CMAKE_CXX_FLAGS from catkin workspace.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# search for everything we need to build the package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet

  pcl_ros
  std_msgs
  geometry_msgs
  message_generation

  octomap_ros
  octomap_msgs

  )

add_message_files(
  FILES
  SegmapMsg.msg
)

add_service_files(
  FILES
  GetSphereMapPathSrv.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
  # actionlib_msgs
)

# export the dependencis of this package for who ever depends on us
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES SpheremapServer 
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs #darpa_msgs message_runtime darpa_planning_lib
  )

# tell catkin where to find the headers for this project
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  )

# Declare a C++ library
add_library(SpheremapServer src/mapper.cpp
  src/pcl_map.cpp
  src/mapper.cpp
  # src/planning_nodelet.cpp
  src/mapping_nodelet.cpp
  src/mapping_structures.cpp
  # src/planning_modules.cpp
  src/utility_functions.cpp
  src/spheremap.cpp
  )

target_link_libraries(SpheremapServer 
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  )
