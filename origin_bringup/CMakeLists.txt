cmake_minimum_required(VERSION 3.8)
project(origin_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/lidar_ray_markers.py DESTINATION lib/${PROJECT_NAME})

ament_package()
