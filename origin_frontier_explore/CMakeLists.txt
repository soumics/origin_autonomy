cmake_minimum_required(VERSION 3.8)
project(origin_frontier_explore)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# ROS-independent algorithms (unit-tested)
add_library(${PROJECT_NAME}_core
  src/grid.cpp
  src/frontier_search.cpp
  src/grid_planner.cpp
  src/dwa_controller.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)

set(node_deps
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${visualization_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
)

add_executable(frontier_explorer src/frontier_explorer_node.cpp)
target_link_libraries(frontier_explorer ${PROJECT_NAME}_core ${node_deps})

add_executable(custom_navigator src/custom_navigator_node.cpp)
target_link_libraries(custom_navigator ${PROJECT_NAME}_core ${node_deps})

add_executable(cloud_self_filter src/cloud_self_filter_node.cpp)
target_link_libraries(cloud_self_filter ${node_deps})

install(TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(TARGETS frontier_explorer custom_navigator cloud_self_filter
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/wait_for_transform.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz behavior_trees DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_algorithms test/test_algorithms.cpp)
  target_link_libraries(test_algorithms ${PROJECT_NAME}_core)
endif()

ament_package()
