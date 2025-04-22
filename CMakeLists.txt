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

add_executable(car_s_controller_accel src/pi_car_s_controller_accel.cpp)
target_link_libraries(car_s_controller_accel pigpiod_if2)
ament_target_dependencies(car_s_controller_accel nav_msgs rclcpp std_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

add_executable(imu_normalize src/imu_normalize.cpp)
target_link_libraries(imu_normalize)
ament_target_dependencies(imu_normalize rclcpp std_msgs sensor_msgs)

add_executable(car_imu src/pi_car_imu.cpp)
target_link_libraries(car_imu pigpiod_if2)
ament_target_dependencies(car_imu rclcpp std_msgs sensor_msgs)

add_executable(car_mag src/pi_car_mag.cpp)
target_link_libraries(car_mag pigpiod_if2)
ament_target_dependencies(car_mag rclcpp std_msgs sensor_msgs)

add_executable(pwm_led src/pwm_led.cpp)
target_link_libraries(pwm_led pigpiod_if2)
ament_target_dependencies(pwm_led rclcpp std_msgs sensor_msgs)

add_executable(key_cmd src/key_cmd.cpp)
target_link_libraries(key_cmd)
ament_target_dependencies(key_cmd rclcpp std_msgs tf2_geometry_msgs)

add_executable(pi_power_control src/pi_power_control.cpp)
target_link_libraries(pi_power_control pigpiod_if2)
ament_target_dependencies(pi_power_control rclcpp)

install(TARGETS
  car_s_controller_accel
  imu_normalize
  pi_power_control
  car_imu
  car_mag
  pwm_led
  key_cmd
  DESTINATION lib/${PROJECT_NAME})

ament_package()
