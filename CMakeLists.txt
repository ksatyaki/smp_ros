cmake_minimum_required(VERSION 2.8.3)
project(smp_ros)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)
set(MRPT_DONT_USE_DBG_LIBS 1)

find_package(catkin REQUIRED COMPONENTS
  cliffmap_ros
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  nav_core
)

find_package(MRPT REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES smp_ros_external
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs tf nav_core cliffmap_ros
  DEPENDS Boost
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

add_library(kdtree src/external_libraries/kdtree/kdtree.c)

add_executable(test_rrtstar src/test_rrtstar.cpp)
add_executable(test_rrtstar_dubins src/test_rrtstar_dubins.cpp)
add_executable(test_rrtstar_dubins_ros src/test_rrtstar_dubins_ros.cpp)
add_executable(test_double_integrator_ros src/test_double_integrator_ros.cpp)

target_link_libraries(test_rrtstar smp_ros_external kdtree)
target_link_libraries(test_rrtstar_dubins smp_ros_external kdtree)
target_link_libraries(test_rrtstar_dubins_ros smp_ros_external kdtree ${catkin_LIBRARIES} ${MRPT_LIBRARIES})
target_link_libraries(test_double_integrator_ros smp_ros_external kdtree ${catkin_LIBRARIES} ${MRPT_LIBRARIES})


add_library(rrtstar_dubins_global_planner src/rrtstar_dubins_global_planner.cpp)
target_link_libraries(rrtstar_dubins_global_planner smp_ros_external kdtree ${catkin_LIBRARIES} ${MRPT_LIBRARIES})

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

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
