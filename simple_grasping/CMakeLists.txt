cmake_minimum_required(VERSION 2.8.3)
project(simple_grasping)

find_package(catkin REQUIRED roscpp pcl_ros manipulation_msgs tf)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)

link_directories(${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})

catkin_package(DEPENDS pcl_ros pcl roscpp tf)

include_directories(include
                    SYSTEM
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
                   )

add_executable(object_support_segmentation src/object_support_segmentation.cpp)
target_link_libraries(object_support_segmentation ${Boost_LIBRARIES}
                                                  ${PCL_LIBRARIES}
                                                  ${catkin_LIBRARIES} )

install(TARGETS object_support_segmentation
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
