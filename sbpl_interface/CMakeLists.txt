cmake_minimum_required(VERSION 2.8.3)
project(moveit_planners_sbpl)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
include_directories(${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})

find_package(Boost REQUIRED system filesystem date_time thread serialization)
find_package(catkin REQUIRED COMPONENTS
  angles
  eigen_conversions
  moveit_core
  moveit_ros_planning
  pluginlib
  rosconsole
  roscpp
)

catkin_package(
  LIBRARIES
    moveit_sbpl_arm_planner
    ${SBPL_LIBRARIES}
  INCLUDE_DIRS
#    sbpl_interface/include
    include
    ${SBPL_INCLUDE_DIRS}
  CATKIN_DEPENDS
    moveit_core
)

include_directories(SYSTEM
                    ${EIGEN_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS})

#include_directories(sbpl_interface/include
include_directories(include
                    ${catkin_INCLUDE_DIRS}
		            ${SBPL_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})

add_library(moveit_sbpl_arm_planner
  src/bfs3d/BFS_3D.cpp
  src/bfs3d/Search.cpp
  src/environment_chain3d.cpp
  src/environment_chain3d_moveit.cpp
)
target_link_libraries(moveit_sbpl_arm_planner
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${SBPL_LIBRARIES}
)

set(MOVEIT_PLUGIN_NAME moveit_sbpl_planner_plugin)
add_library(${MOVEIT_PLUGIN_NAME}
  src/sbpl_planning_context.cpp
  src/sbpl_planning_manager.cpp
)
target_link_libraries(${MOVEIT_PLUGIN_NAME}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  moveit_sbpl_arm_planner
)

add_subdirectory(test)

#install(FILES sbpl_interface_plugin_description.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
