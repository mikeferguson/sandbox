cmake_minimum_required(VERSION 2.8.3)
project(asio_ros)

find_package(Boost REQUIRED system thread)
find_package(catkin REQUIRED roscpp)
catkin_package()

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(asio_client src/client.cpp)
target_link_libraries(asio_client ${Boost_LIBRARIES}
                                  ${catkin_LIBRARIES}
)

add_executable(asio_server src/server.cpp)
target_link_libraries(asio_server ${Boost_LIBRARIES}
                                  ${catkin_LIBRARIES}
)

install(TARGETS asio_client asio_server
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
