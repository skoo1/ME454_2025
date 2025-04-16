cmake_minimum_required(VERSION 3.8)
project(midterm_part2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Per-student macro
macro(build_node arg)

  # Make executable
  add_executable(state_forwarder_${arg} src/state_forwarder_${arg}.cpp)
  ament_target_dependencies(state_forwarder_${arg} rclcpp std_msgs gazebo_msgs geometry_msgs)

  install(TARGETS
    state_forwarder_${arg}
    DESTINATION lib/${PROJECT_NAME})

endmacro()

add_executable(state_printer src/state_printer.cpp)
ament_target_dependencies(state_printer rclcpp std_msgs gazebo_msgs geometry_msgs)

install(TARGETS
  state_printer
  DESTINATION lib/${PROJECT_NAME})

build_node(20250000) # your student id here

install(DIRECTORY
  launch
  rsc
  src
  worlds
  DESTINATION share/${PROJECT_NAME})

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