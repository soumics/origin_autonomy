cmake_minimum_required(VERSION 3.8)
project(origin_lidar_localization)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/tf_synced_relay.py DESTINATION lib/${PROJECT_NAME})

ament_package()
