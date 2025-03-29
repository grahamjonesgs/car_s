cmake_minimum_required(VERSION 3.5)
project(car_s)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(car_s_controller src/pi_car_s_controller.cpp)
target_link_libraries(car_s_controller pigpiod_if2)
ament_target_dependencies(car_s_controller nav_msgs rclcpp std_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(imu_normalize src/imu_normalize.cpp)
target_link_libraries(imu_normalize)
ament_target_dependencies(imu_normalize rclcpp std_msgs sensor_msgs)

install(TARGETS
  car_s_controller
  imu_normalize
  DESTINATION lib/${PROJECT_NAME})

ament_package()
