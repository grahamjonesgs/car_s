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
find_package(std_srvs REQUIRED)

# Nodes that run on any machine (e.g. the remote control PC)
add_executable(imu_normalize src/imu_normalize.cpp)
target_link_libraries(imu_normalize)
ament_target_dependencies(imu_normalize rclcpp std_msgs sensor_msgs)

add_executable(key_cmd src/key_cmd.cpp)
target_link_libraries(key_cmd)
ament_target_dependencies(key_cmd rclcpp std_msgs std_srvs tf2_geometry_msgs)

install(TARGETS
  imu_normalize
  key_cmd
  DESTINATION lib/${PROJECT_NAME})

# Hardware nodes need pigpio, so are only built where it is installed (the Pi)
find_library(PIGPIOD_IF2_LIB pigpiod_if2)
if(PIGPIOD_IF2_LIB)
  add_executable(car_s_controller_accel src/pi_car_s_controller_accel.cpp)
  target_link_libraries(car_s_controller_accel ${PIGPIOD_IF2_LIB})
  ament_target_dependencies(car_s_controller_accel nav_msgs rclcpp std_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

  add_executable(car_imu src/pi_car_imu.cpp)
  target_link_libraries(car_imu ${PIGPIOD_IF2_LIB})
  ament_target_dependencies(car_imu rclcpp std_msgs sensor_msgs)

  add_executable(car_mag src/pi_car_mag.cpp)
  target_link_libraries(car_mag ${PIGPIOD_IF2_LIB})
  ament_target_dependencies(car_mag rclcpp std_msgs sensor_msgs)

  install(TARGETS
    car_s_controller_accel
    car_imu
    car_mag
    DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "pigpio not found: building key_cmd and imu_normalize only")
endif()

# Shutdown and reboot services for the Pi, and the PC's watch on the car
install(PROGRAMS scripts/car_power.py DESTINATION lib/${PROJECT_NAME} RENAME car_power)
install(PROGRAMS scripts/car_guard.py DESTINATION lib/${PROJECT_NAME} RENAME car_guard)

install(DIRECTORY launch urdf config rviz DESTINATION share/${PROJECT_NAME})

ament_package()
