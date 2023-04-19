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

add_executable(prac src/prac.cpp)
ament_target_dependencies(prac rclcpp)

add_executable(talk src/talk.cpp)
ament_target_dependencies(talk 
  lifecycle_msgs
  rclcpp_lifecycle
  std_msgs)

add_executable(service src/service.cpp)
  ament_target_dependencies(service 
    lifecycle_msgs
    rclcpp_lifecycle
    std_msgs)
  

add_executable(listen src/listen.cpp)
ament_target_dependencies(listen 
  lifecycle_msgs
  rclcpp_lifecycle
  std_msgs)


install(TARGETS
    prac
    talk
    listen
    service
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
