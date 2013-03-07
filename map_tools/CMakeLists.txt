cmake_minimum_required(VERSION 2.8.3)
project(map_tools)

find_package(Boost REQUIRED)
find_package(octomap REQUIRED)
find_package(catkin REQUIRED roscpp octomap nav_msgs tf)
catkin_package()

include_directories(${OCTOMAP_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})

add_executable(octo_to_2d src/octo_to_2d.cpp)
target_link_libraries(octo_to_2d ${Boost_LIBRARIES}
                                 ${OCTOMAP_LIBRARIES}
                                 ${catkin_LIBRARIES} )

install(TARGETS octo_to_2d
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

