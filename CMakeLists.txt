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
  LIBRARIES smp_ros_external smp_ros_kdtree rrtstar_dubins_global_planner
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs tf nav_core costmap_2d
  DEPENDS Boost MRPT
)

include_directories(
  include
  ${MRPT_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(smp_ros_external
  src/external_libraries/halton/halton.C
  src/external_libraries/inc_mu_mc/ms.cpp
  src/external_libraries/inc_mu_mc/pt.cpp
)
add_library(smp_ros_kdtree src/external_libraries/kdtree/kdtree.c)
add_library(rrtstar_dubins_global_planner src/rrtstar_dubins_global_planner.cpp)
target_link_libraries(rrtstar_dubins_global_planner smp_ros_external smp_ros_kdtree ${catkin_LIBRARIES} ${MRPT_LIBRARIES})

install(TARGETS smp_ros_kdtree smp_ros_external rrtstar_dubins_global_planner
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

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_smp_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
