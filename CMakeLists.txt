cmake_minimum_required(VERSION 3.8)
project(lifecycle_prac)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

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

include_directories(include)


add_executable(parameter_node src/parameter_node.cpp)
ament_target_dependencies(parameter_node rclcpp)

add_executable(async_lc_node src/async_lc_node.cpp)
ament_target_dependencies(async_lc_node 
  lifecycle_msgs
  rclcpp_lifecycle
  std_msgs)

add_executable(change_state_client src/change_state_client.cpp)
  ament_target_dependencies(change_state_client 
    lifecycle_msgs
    rclcpp_lifecycle
    std_msgs)
  

add_executable(listen src/listen.cpp)
ament_target_dependencies(listen 
  lifecycle_msgs
  rclcpp_lifecycle
  std_msgs)

add_executable(defer_node src/defer_node.cpp)
ament_target_dependencies(defer_node 
  lifecycle_msgs
  rclcpp_lifecycle
  std_msgs
  example_interfaces)

add_executable(simple_two_ints src/simple_two_ints.cpp)
ament_target_dependencies(simple_two_ints 
  lifecycle_msgs
  rclcpp_lifecycle
  std_msgs
  example_interfaces)

install(TARGETS
    parameter_node
    async_lc_node
    listen
    change_state_client
    defer_node
    simple_two_ints
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
