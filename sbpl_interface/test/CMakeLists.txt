catkin_add_gtest(sbpl_interface_test_space test_space.cpp)
target_link_libraries(sbpl_interface_test_space ${Boost_LIBRARIES}
                                                ${catkin_LIBRARIES}
                                                moveit_sbpl_arm_planner)
