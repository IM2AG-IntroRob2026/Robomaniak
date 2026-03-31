cmake_minimum_required(VERSION 3.8)
project(turtle_drawing_boundaries)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(turtle_boundaries_interfaces REQUIRED)

add_executable(turtle_boundaries_node src/turtle_boundaries.cpp)
ament_target_dependencies(turtle_boundaries_node
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  turtlesim
  turtle_boundaries_interfaces
)

target_include_directories(turtle_boundaries_node PRIVATE src)

add_executable(turtle_boundaries_keyboard src/turtle_boundaries_keyboard.cpp)
ament_target_dependencies(turtle_boundaries_keyboard
  rclcpp
  geometry_msgs
  std_msgs
)

install(TARGETS
  turtle_boundaries_node
  turtle_boundaries_keyboard
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
