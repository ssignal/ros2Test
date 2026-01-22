cmake_minimum_required(VERSION 3.8)
project(ssros_ams)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(test_interfaces REQUIRED)

add_executable(gesture_publisher src/gesture_publisher.cpp)
ament_target_dependencies(gesture_publisher rclcpp test_interfaces)

add_executable(gesture_subscriber src/gesture_subscriber.cpp)
ament_target_dependencies(gesture_subscriber rclcpp test_interfaces)

add_executable(set_fan_pwm_server src/set_fan_pwm_server.cpp)
ament_target_dependencies(set_fan_pwm_server rclcpp test_interfaces)

add_executable(set_fan_pwm_client src/set_fan_pwm_client.cpp)
ament_target_dependencies(set_fan_pwm_client rclcpp test_interfaces)

add_executable(spin_action_server src/spin_action_server.cpp)
ament_target_dependencies(spin_action_server rclcpp rclcpp_action test_interfaces)

add_executable(spin_action_client src/spin_action_client.cpp)
ament_target_dependencies(spin_action_client rclcpp rclcpp_action test_interfaces)

install(TARGETS
  gesture_publisher
  gesture_subscriber
  set_fan_pwm_server
  set_fan_pwm_client
  spin_action_server
  spin_action_client
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
