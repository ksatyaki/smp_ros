cmake_minimum_required(VERSION 2.8.3)
project(smp_ros)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)
set(MRPT_DONT_USE_DBG_LIBS 1)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  costmap_2d
  roscpp
  std_msgs
  tf
  nav_core
)

find_package(MRPT REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES smp_external smp_kdtree smp_extenders smp_ros_planners
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs tf nav_core costmap_2d
  DEPENDS Boost MRPT
)

include_directories(
  include
  ${MRPT_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(smp_external
  src/external_libraries/halton/halton.C
  src/external_libraries/inc_mu_mc/ms.cpp
  src/external_libraries/inc_mu_mc/pt.cpp)

add_library(smp_kdtree
  src/external_libraries/kdtree/kdtree.c)

add_library(smp_extenders
  src/smp/extenders_dubins.cpp
  src/smp/extenders_double_integrator.cpp
  src/smp/extenders_posq.cpp)

add_library(smp_ros_planners
  src/rrtstar_dubins_global_planner.cpp
  src/rrtstar_posq_global_planner.cpp)

target_link_libraries(smp_ros_planners
  smp_external
  smp_kdtree
  smp_extenders
  ${catkin_LIBRARIES}
  ${MRPT_LIBRARIES})

install(TARGETS smp_kdtree smp_external smp_extenders smp_ros_planners
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY include/smp/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/../smp
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
