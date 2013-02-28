cmake_minimum_required(VERSION 2.8.3)
project(depth_viewer)

find_package(Boost REQUIRED)
find_package(catkin REQUIRED roscpp sensor_msgs cv_bridge)
catkin_package()

add_executable(depth_viewer src/depth_viewer.cpp)
target_link_libraries(depth_viewer ${Boost_LIBRARIES}
                                   ${catkin_LIBRARIES} )

install(TARGETS depth_viewer
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
