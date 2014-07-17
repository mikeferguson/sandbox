cmake_minimum_required(VERSION 2.8.3)
project(collision_field)

find_package(Boost REQUIRED)
find_package(catkin REQUIRED roscpp moveit_core moveit_msgs)
catkin_package()

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(visualize_field src/visualize_field.cpp)
target_link_libraries(visualize_field ${Boost_LIBRARIES}
                                      ${catkin_LIBRARIES} )

install(TARGETS visualize_field
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
