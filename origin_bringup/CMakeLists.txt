cmake_minimum_required(VERSION 3.8)
project(origin_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz worlds DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/lidar_ray_markers.py scripts/mission_manager.py scripts/detect_origin_one.py
                 scripts/tf_relay.py scripts/check_origin_one.py
                 scripts/sensor_restamp.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
